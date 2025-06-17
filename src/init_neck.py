
#!/usr/bin/env python3
import os
import sys

sys.path.append("../../arx5-sdk/python")
sys.path.append("../../arx5-sdk/lib")

import arx5_interface as arx5
from arx5_interface import Arx5CartesianController, EEFState, Gain, LogLevel

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
sys.path.insert(0, path_src + "/../projects/utils/")

import time
import numpy as np


import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import threading


from queue import Queue


class ARX5_init:
    def __init__(self):
        np.set_printoptions(precision=3, suppress=True)

        urdf_path = "../../arx5-sdk/models/arx5_iphone.urdf"
        model = "L5"
        interface = "can11"


        # joint to EE pose w.r.t shoulder
        joint_dof = 6
        robot_config = arx5.RobotConfigFactory.get_instance().get_config("L5")
        self.solver = arx5.Arx5Solver(
                                    urdf_path,
                                    joint_dof,
                                    robot_config.joint_pos_min,
                                    robot_config.joint_pos_max,
                                    # "base_link",
                                    # "eef_link",
                                    # np.array([0, 0, -9.807], dtype=np.float64),
                                    )

        # build neck robot

        robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        robot_config.gravity_vector = np.array([0, 0, -9.81])
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "cartesian_controller", robot_config.joint_dof
        )
        

        self.controller = Arx5CartesianController(
            robot_config, controller_config, interface, urdf_path
        )

        gain = self.controller.get_gain()
        self.controller.set_gain(gain)  


        self.robot_config = self.controller.get_robot_config()
        self.controller_config = self.controller.get_controller_config()



        # reset neck robot


        self.controller.reset_to_home()


        self.reset_pose  = np.array([0.15, 0, 0.3, -np.pi/2, 0, 0])
    
        current_state = self.controller.get_eef_state()
        end_pose = self.reset_pose  # world frame starting pose
        eefstate = EEFState(end_pose, 0)
        eefstate.timestamp = current_state.timestamp + 2
        self.controller.set_eef_cmd(eefstate)
        time.sleep(3)

        current_state = self.controller.get_eef_state()
        current_neck_pose = current_state.pose_6d()
        print("Transformed translation in body frame:", current_neck_pose[:3])
        print("Transformed rotation in body frame:", np.degrees(current_neck_pose[3:]))


        print("initalized!")
        os._exit(0)           










if __name__ == "__main__":




    rospy.init_node("ARX5_init")
    neck_init = ARX5_init()
