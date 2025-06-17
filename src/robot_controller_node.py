import time

import os
import sys




import math
import time
import os
import sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose, Quaternion
from std_msgs.msg import Float32MultiArray
import sys
import os
import rospkg
from std_srvs.srv import Empty, SetBool
path_src = rospkg.get_ros_package_path().split(':')[0]
# sys.path.insert(0, path_src + "/../projects/utils/")

import time
import numpy as np
import os
import sys


#!/usr/bin/env python3
import os
import sys

sys.path.append("../arx5-sdk/python")
sys.path.append("../arx5-sdk/lib")

import arx5_interface as arx5
from arx5_interface import Arx5CartesianController, EEFState, Gain, LogLevel

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.transformation_helper import quaternion_camera_pose_to_extrinsic_matrix, arm_urdf_path, neck_urdf_path
from utils.transformation_helper import left_start_body_pose, right_start_body_pose, neck_start_pose, pick_place_neck_start_pose, T_right_shoulder_to_body, T_left_shoulder_to_body, inverse_transformation_matrix, quaternion_to_homogeneous_matrix, pose_to_homogeneous,  transform_pose 

from std_msgs.msg import Float64MultiArray

from scipy.spatial.transform import Rotation as R


import argparse


def easeInOutQuad(t):
    t *= 2
    if t < 1:
        return t * t / 2
    else:
        t -= 1
        return -(t * (t - 2) - 1) / 2

def get_args():
    parser = argparse.ArgumentParser()

    args = parser.parse_args()
    return args


class ARX5_Binmanual:
    def __init__(self, args):
        np.set_printoptions(precision=3, suppress=True)

        model = "L5"
        left_interface = "can_left"
        right_interface = "can_right"
        neck_interface = "can_top"
        joint_dof = 6

        # Transformation matrix from world frame to right shoulder frame
        self.T_right_shoulder_to_world = T_right_shoulder_to_body
        self.T_world_to_right_shoulder = inverse_transformation_matrix(self.T_right_shoulder_to_world)

        self.T_left_shoulder_to_world = T_left_shoulder_to_body        
        self.T_world_to_left_shoulder = inverse_transformation_matrix(self.T_left_shoulder_to_world)


        # joint to EE pose w.r.t shoulder
        robot_config = arx5.RobotConfigFactory.get_instance().get_config("L5")

        self.arm_solver = arx5.Arx5Solver(
                                    arm_urdf_path,
                                    joint_dof,
                                    robot_config.joint_pos_min,
                                    robot_config.joint_pos_max,
                                    # "base_link",
                                    # "eef_link",
                                    # np.array([0, 0, -9.807], dtype=np.float64),
                                    )

        self.neck_solver = arx5.Arx5Solver(
                                    neck_urdf_path,
                                    joint_dof,
                                    robot_config.joint_pos_min,
                                    robot_config.joint_pos_max,
                                    # "base_link",
                                    # "eef_link",
                                    # np.array([0, 0, -9.807], dtype=np.float64),
                                    )


        # build neck robot


    
        # build neck robot

        neck_robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        neck_robot_config.gravity_vector = np.array([0, 0, -9.81])
        neck_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "cartesian_controller", neck_robot_config.joint_dof
        )
        

        self.neck_controller = Arx5CartesianController(
            robot_config, neck_controller_config, neck_interface, neck_urdf_path
        )

        gain = self.neck_controller.get_gain()
        self.neck_controller.set_gain(gain)  




        self.neck_robot_config = self.neck_controller.get_robot_config()
        self.neck_controller_config = self.neck_controller.get_controller_config()
        self.neck_action_prev = None


        # build left arm robot

        left_robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        left_robot_config.gravity_vector = np.array([0, 9.81*np.sqrt(2)/2, 9.81*np.sqrt(2)/2])
        left_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "cartesian_controller", left_robot_config.joint_dof
        )

        self.left_arx5_controller = Arx5CartesianController(
            left_robot_config, left_controller_config, left_interface, arm_urdf_path
        )

        gain = self.left_arx5_controller.get_gain()
        self.left_arx5_controller.set_gain(gain)  

 


        self.left_robot_config = self.left_arx5_controller.get_robot_config()
        self.left_controller_config = self.left_arx5_controller.get_controller_config()

        self.left_jts_prev = None; self.left_jts = None
        self.left_action_prev = None; self.left_jts = None


        
        # build right arm robot

        right_robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        right_robot_config.gravity_vector = np.array([0, -9.81*np.sqrt(2)/2, 9.81*np.sqrt(2)/2])
        right_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "cartesian_controller", right_robot_config.joint_dof
        )
        

        self.right_arx5_controller = Arx5CartesianController(
            right_robot_config, right_controller_config, right_interface, arm_urdf_path
        )

        gain = self.right_arx5_controller.get_gain()
        self.right_arx5_controller.set_gain(gain)  


        self.right_robot_config = self.right_arx5_controller.get_robot_config()
        self.right_controller_config = self.right_arx5_controller.get_controller_config()

        self.right_jts_prev = None; self.right_jts = None
        self.right_action_prev = None; self.right_jts = None
        

        # reset neck and arms

        self.neck_controller.reset_to_home()
        self.left_arx5_controller.reset_to_home()
        self.right_arx5_controller.reset_to_home()

        # neck start pose
        
        end_pose = neck_start_pose.copy()

        current_state = self.neck_controller.get_eef_state()
        eefstate = EEFState(end_pose, 0)
        eefstate.timestamp = current_state.timestamp + 3
        self.neck_controller.set_eef_cmd(eefstate)





        left_body_pose_homogeneous = pose_to_homogeneous(left_start_body_pose)
        left_end_pose = transform_pose(left_body_pose_homogeneous, self.T_world_to_left_shoulder, is_quaternion=0, is_rpy=1)
        eefstate = EEFState(left_end_pose, 1)
        current_state = self.left_arx5_controller.get_eef_state()
        eefstate.timestamp = current_state.timestamp + 3
        self.left_arx5_controller.set_eef_cmd(eefstate)


        right_body_pose_homogeneous = pose_to_homogeneous(right_start_body_pose)
        right_end_pose = transform_pose(right_body_pose_homogeneous, self.T_world_to_right_shoulder, is_quaternion=0, is_rpy=1)
        eefstate = EEFState(right_end_pose, 1)
        current_state = self.right_arx5_controller.get_eef_state()
        eefstate.timestamp = current_state.timestamp + 3
        self.right_arx5_controller.set_eef_cmd(eefstate)
        time.sleep(3)








        # gello command subscriber
        self.right_gello_tracker = rospy.Subscriber(f"/arm/right_gello/", JointState, self._right_receive_joints)
        self.left_gello_tracker = rospy.Subscriber("/arm/left_gello/", JointState, self._left_receive_joints)
        # rollout command subscriber
        self.right_action_tracker = rospy.Subscriber(f"/arm/right_action/", JointState, self._right_receive_actions)
        self.left_action_tracker = rospy.Subscriber("/arm/left_action/", JointState, self._left_receive_actions)
        self.neck_action_sub = rospy.Subscriber("/neck/action", JointState, self._neck_receive_actions)

        self.neck_init_sub = rospy.Subscriber("/neck/init_neck_action", JointState, self._neck_receive_init_actions)
        self.right_init_sub = rospy.Subscriber("/arm/init_right_action", JointState, self._right_receive_init_actions)
        self.left_init_sub = rospy.Subscriber("/arm/init_left_action", JointState, self._left_receive_init_actions)



        # proprioception publisher
        self.right_proprio_pub = rospy.Publisher(name=("/arm/right_proprio"), data_class=JointState, queue_size=10)
        self.left_proprio_pub = rospy.Publisher(name=("/arm/left_proprio"), data_class=JointState, queue_size=10)
        self.right_redundant_proprio_pub = rospy.Publisher(name=("/arm/right_redundant"), data_class=JointState, queue_size=10)
        self.left_redundant_proprio_pub = rospy.Publisher(name=("/arm/left_redundant"), data_class=JointState, queue_size=10)
        self.neck_proprio_pub = rospy.Publisher("/neck/proprio", JointState, queue_size=10)
        self.neck_redundant_pub = rospy.Publisher("/neck/neck_redundant", JointState, queue_size=10)






        self.last_time = time.time()



        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_proprio_data()
            r.sleep()


        
        

    def publish_proprio_data(self):
        """
        Continuously publishes the proprioceptive data for both arms.
        """
  

        
        neck_joint_state = self.neck_controller.get_joint_state()
        neck_joint_pos = np.array(neck_joint_state.pos().copy().flatten())
        neck_joint_vel = np.array(neck_joint_state.vel().copy().flatten())
        neck_gripper_vel = np.array(neck_joint_state.gripper_vel)
        neck_gripper_pos = np.array(neck_joint_state.gripper_pos)

        neck_redundant_proprio = (np.hstack((
            neck_joint_pos,
            neck_joint_vel,
            neck_gripper_vel,
            neck_gripper_pos,
        )))



        neck_ee_state = self.neck_controller.get_eef_state()
        neck_ee_pose = neck_ee_state.pose_6d()

        rotation_matrix = R.from_euler('xyz', neck_ee_pose[3:], degrees=False).as_matrix()
        neck_proprio = np.concatenate([neck_ee_pose[:3], R.from_matrix(rotation_matrix).as_quat()])



        neck_timestamp = rospy.Time.now()
        neck_proprio_msg = JointState()
        neck_proprio_msg.position = list(map(float, neck_proprio.copy().flatten()))
        neck_proprio_msg.header.stamp = neck_timestamp
        self.neck_proprio_pub.publish(neck_proprio_msg)

        neck_redundant_proprio_msg = JointState()
        neck_redundant_proprio_msg.position = list(map(float, neck_redundant_proprio.copy().flatten()))
        neck_redundant_proprio_msg.header.stamp = neck_timestamp
        self.neck_redundant_pub.publish(neck_redundant_proprio_msg)







        
        right_joint_state = self.right_arx5_controller.get_joint_state()
        right_joint_pos = np.array(right_joint_state.pos().copy().flatten())
        right_joint_vel = np.array(right_joint_state.vel().copy().flatten())
        right_gripper_vel = np.array(right_joint_state.gripper_vel)

        right_redundant_proprio = (np.hstack((
            right_joint_pos,
            right_joint_vel,
            right_gripper_vel,
        )))


        right_gripper_pos = np.array(right_joint_state.gripper_pos)
        right_ee_state = self.right_arx5_controller.get_eef_state()
        right_shoulder_pose = right_ee_state.pose_6d()
        right_pose_homogeneous = pose_to_homogeneous(right_shoulder_pose)
        right_ee_body_pose = transform_pose(right_pose_homogeneous, self.T_right_shoulder_to_world, is_quaternion=1, is_rpy=0) # body pose, quat
        right_ee_body_pose = np.array(right_ee_body_pose.copy().flatten())

        right_proprio = (np.hstack((
            right_ee_body_pose,
            right_gripper_pos,
        )))



        right_timestamp = rospy.Time.now()


        right_proprio_msg = JointState()
        right_proprio_msg.position = list(map(float, right_proprio.copy().flatten()))
        right_proprio_msg.header.stamp = right_timestamp
        self.right_proprio_pub.publish(right_proprio_msg)

        right_redundant_proprio_msg = JointState()
        right_redundant_proprio_msg.position = list(map(float, right_redundant_proprio.copy().flatten()))
        right_redundant_proprio_msg.header.stamp = right_timestamp
        self.right_redundant_proprio_pub.publish(right_redundant_proprio_msg)
        



        left_joint_state = self.left_arx5_controller.get_joint_state()
        left_joint_pos = np.array(left_joint_state.pos().copy().flatten())
        left_joint_vel = np.array(left_joint_state.vel().copy().flatten())
        left_gripper_vel = np.array(left_joint_state.gripper_vel)


        # print(right_joint_pos)

        left_redundant_proprio = (np.hstack((
            left_joint_pos,
            left_joint_vel,
            left_gripper_vel,
        )))

        left_gripper_pos = np.array(left_joint_state.gripper_pos)
        left_ee_state = self.left_arx5_controller.get_eef_state()
        left_shoulder_pose = left_ee_state.pose_6d()
        left_pose_homogeneous = pose_to_homogeneous(left_shoulder_pose)
        left_ee_body_pose = transform_pose(left_pose_homogeneous, self.T_left_shoulder_to_world, is_quaternion=1, is_rpy=0)  # body pose, quat
        left_ee_body_pose = np.array(left_ee_body_pose.copy().flatten())

        left_proprio = (np.hstack((
            left_ee_body_pose,
            left_gripper_pos,
        )))

        left_timestamp = rospy.Time.now()

        left_proprio_msg = JointState()
        left_proprio_msg.position = list(map(float, left_proprio.copy().flatten()))
        left_proprio_msg.header.stamp = left_timestamp
        self.left_proprio_pub.publish(left_proprio_msg)

        left_redundant_proprio_msg = JointState()
        left_redundant_proprio_msg.position = list(map(float, left_redundant_proprio.copy().flatten()))
        left_redundant_proprio_msg.header.stamp = left_timestamp
        self.left_redundant_proprio_pub.publish(left_redundant_proprio_msg)

        

        left_euler_angles = R.from_quat(left_ee_body_pose[-4:]).as_euler('xyz', degrees=True) 

        right_euler_angles = R.from_quat(right_ee_body_pose[-4:]).as_euler('xyz', degrees=True) 
        neck_euler_angles = R.from_quat(neck_ee_pose[-4:]).as_euler('xyz', degrees=True) 


        # print("right pose world", right_ee_body_pose[:3], right_euler_angles, "left pose world", left_ee_body_pose[:3], left_euler_angles)
        # print("left pose world", left_ee_body_pose[:3], left_euler_angles)

        # print("neck_euler_angles", neck_proprio[:3])




    def _left_receive_joints(self, data: JointState):


        if self.left_jts is None:

            self.left_jts = np.array(data.position)

            # joint to EE pose w.r.t shoulder
            fk_pose = self.arm_solver.forward_kinematics(self.left_jts[:6])


            eefstate = EEFState(fk_pose, easeInOutQuad(self.left_jts[6]) * 0.08)
            current_state = self.left_arx5_controller.get_eef_state()
            eefstate.timestamp = current_state.timestamp + 3
            self.left_arx5_controller.set_eef_cmd(eefstate)


            time.sleep(3)

            self.left_jts_prev = self.left_jts

        else:
                
            self.left_jts = np.array(data.position)



            # joint to EE pose w.r.t shoulder
            fk_pose = self.arm_solver.forward_kinematics(self.left_jts[:6])



            eefstate = EEFState(fk_pose, easeInOutQuad(self.left_jts[6]) * 0.08)
            current_state = self.left_arx5_controller.get_eef_state()
            eefstate.timestamp = current_state.timestamp + 0.2
            self.left_arx5_controller.set_eef_cmd(eefstate)

            # print("left_gello_action", self.left_jts, "fk_pose", fk_pose)

            time.sleep(self.left_controller_config.controller_dt)





    def _right_receive_joints(self, data: JointState):
        

        if self.right_jts_prev is None:

            self.right_jts = np.array(data.position)

            # joint to EE pose w.r.t shoulder
            fk_pose = self.arm_solver.forward_kinematics(self.right_jts[:6])
            eefstate = EEFState(fk_pose, easeInOutQuad(self.right_jts[6]) * 0.08)
            current_state = self.right_arx5_controller.get_eef_state()
            eefstate.timestamp = current_state.timestamp + 3
            self.right_arx5_controller.set_eef_cmd(eefstate)


            time.sleep(3)
            self.right_jts_prev = self.right_jts

        else:
            self.right_jts = np.array(data.position)

            # joint to EE pose w.r.t shoulder
            fk_pose = self.arm_solver.forward_kinematics(self.right_jts[:6])


            eefstate = EEFState(fk_pose, easeInOutQuad(self.right_jts[6]) * 0.08)
            current_state = self.right_arx5_controller.get_eef_state()
            eefstate.timestamp = current_state.timestamp + 0.2
            self.right_arx5_controller.set_eef_cmd(eefstate)


            time.sleep(self.right_controller_config.controller_dt)



    def _left_receive_actions(self, data: JointState):


        action = np.array(data.position)#.reshape(self.start_pose.shape)

        
        body_pose = action[:7]
        body_pose_homogeneous = quaternion_to_homogeneous_matrix(body_pose)
        end_pose = transform_pose(body_pose_homogeneous, self.T_world_to_left_shoulder, is_quaternion=0, is_rpy=1)


        eefstate = EEFState(end_pose, easeInOutQuad(action[7]) * 0.08)
        current_state = self.left_arx5_controller.get_eef_state()



        
        if self.left_action_prev is None:
            eefstate.timestamp = current_state.timestamp + 3
            self.left_arx5_controller.set_eef_cmd(eefstate)
            time.sleep(3)
            self.left_action_prev = action

        else:
            eefstate.timestamp = current_state.timestamp + 0.2
            self.left_arx5_controller.set_eef_cmd(eefstate)

            time.sleep(self.left_controller_config.controller_dt)


    def _left_receive_init_actions(self, data: JointState):


        action = np.array(data.position)#.reshape(self.start_pose.shape)

        
        body_pose = action[:7]
        body_pose_homogeneous = quaternion_to_homogeneous_matrix(body_pose)
        end_pose = transform_pose(body_pose_homogeneous, self.T_world_to_left_shoulder, is_quaternion=0, is_rpy=1)


        eefstate = EEFState(end_pose, easeInOutQuad(action[7]) * 0.08)
        current_state = self.left_arx5_controller.get_eef_state()



    
        eefstate.timestamp = current_state.timestamp + 3
        self.left_arx5_controller.set_eef_cmd(eefstate)
        time.sleep(3)
        self.left_action_prev = action



    def _right_receive_actions(self, data: JointState):

        action = np.array(data.position)#.reshape(self.start_pose.shape)
        
        body_pose = action[:7]
        body_pose_homogeneous = quaternion_to_homogeneous_matrix(body_pose)
        end_pose = transform_pose(body_pose_homogeneous, self.T_world_to_right_shoulder, is_quaternion=0, is_rpy=1)


        eefstate = EEFState(end_pose, easeInOutQuad(action[7]) * 0.08)
        current_state = self.right_arx5_controller.get_eef_state()



        if self.right_action_prev is None:
            eefstate.timestamp = current_state.timestamp + 3
            self.right_arx5_controller.set_eef_cmd(eefstate)
            time.sleep(3)
            self.right_action_prev = action

        else:
            eefstate.timestamp = current_state.timestamp + 0.2
            self.right_arx5_controller.set_eef_cmd(eefstate)

            time.sleep(self.right_controller_config.controller_dt)



    def _right_receive_init_actions(self, data: JointState):

        action = np.array(data.position)#.reshape(self.start_pose.shape)
        
        body_pose = action[:7]
        body_pose_homogeneous = quaternion_to_homogeneous_matrix(body_pose)
        end_pose = transform_pose(body_pose_homogeneous, self.T_world_to_right_shoulder, is_quaternion=0, is_rpy=1)


        eefstate = EEFState(end_pose, easeInOutQuad(action[7]) * 0.08)
        current_state = self.right_arx5_controller.get_eef_state()



        eefstate.timestamp = current_state.timestamp + 3
        self.right_arx5_controller.set_eef_cmd(eefstate)
        time.sleep(3)
        self.right_action_prev = action




    def _neck_receive_init_actions(self, data: JointState):

        action = np.array(data.position)

        rotation_matrix = R.from_quat(action[-4:]).as_matrix()
        rpy = R.from_matrix(rotation_matrix).as_euler('xyz')
        action_euler = np.concatenate([action[:3], rpy])

        eefstate = EEFState(action_euler, 0)
        current_state = self.neck_controller.get_eef_state()


        eefstate.timestamp = current_state.timestamp + 3
        self.neck_controller.set_eef_cmd(eefstate)
        time.sleep(3)
        self.neck_action_prev = action




    def _neck_receive_actions(self, data: JointState):

        action = np.array(data.position)

        rotation_matrix = R.from_quat(action[-4:]).as_matrix()
        rpy = R.from_matrix(rotation_matrix).as_euler('xyz')
        action_euler = np.concatenate([action[:3], rpy])

        eefstate = EEFState(action_euler, 0)
        current_state = self.neck_controller.get_eef_state()


        if self.neck_action_prev is None:
            eefstate.timestamp = current_state.timestamp + 3
            self.neck_controller.set_eef_cmd(eefstate)
            time.sleep(3)
            self.neck_action_prev = action

        else:
            eefstate.timestamp = current_state.timestamp + 0.2
            self.neck_controller.set_eef_cmd(eefstate)
            time.sleep(self.neck_controller_config.controller_dt)




if __name__ == "__main__":

    args = get_args()


    rospy.init_node("ARX5_Binmanual")
    bimanual_node = ARX5_Binmanual(args)


