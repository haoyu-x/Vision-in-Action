
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

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

import rosbag
from pathlib import Path
import pandas as pd
import cv2
import numpy as np


import argparse
import pickle
import json
from sensor_msgs.msg import JointState
import rosbag
from rospy import Time
import time as timer

import os






def create_joint_message(pos: np.ndarray):
    msg = JointState()
    msg.position = pos
    return msg
def get_messages_from_bag(bag, topic_name):
    messages = []
    times = []
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        messages.append(msg)
        times.append(t)
    return messages, times

def fetch_unique_topics(bag):
    return list(bag.get_type_and_topic_info()[1].keys())




    
    
import zarr




def load_from_zarr(save_path):
    # Open the Zarr file
    zarr_store = zarr.open(save_path, mode='r')
    
    # Load data from the store
    data = {}
    for key in zarr_store:
        data[key] = zarr_store[key][:]
        
    return data



class Replay:
    def __init__(self):







        train_dataset_path= '/home/haoyux/teddy_data/replay_pot.zarr.zip'
        train_dataset = load_from_zarr(train_dataset_path)

        # (N, D)
        train_data = {
            # first two dims of state vector are agent (i.e. gripper) locations
            'agent_pos': np.array(train_dataset["proprio"]).astype(np.float32),
            'action': np.array(train_dataset["action"]).astype(np.float32),
        }
        episode_ends = np.array(train_dataset["episode_ends"])


    
        
        traj = 0
        # action_list = train_data["action"][episode_ends[traj]+1:episode_ends[traj+1]]
        # action_list = train_data["action"][episode_ends[0]+1: episode_ends[1]]
        # action_list = train_data["action"][0:episode_ends[traj]]
        action_list = train_data["action"][episode_ends[1]+1: episode_ends[2]]

 

        self.right_pub_tracker = rospy.Publisher(name=("/arm/right_action/"), data_class=JointState, queue_size=3)
        self.left_pub_tracker = rospy.Publisher(name=("/arm/left_action/"), data_class=JointState, queue_size=3)
        self.neck_pub_tracker = rospy.Publisher(name=("/neck/action/"), data_class=JointState, queue_size=3)

        time.sleep(0.1)

        self.publish_action(action_list[0])

        time.sleep(5)


        # desired_interval = 1/30


        step = 0
  
        
        desired_interval = 0.1
        while not rospy.is_shutdown():
            print(step)
            if step % 8 == 0:
                time.sleep(0.1)
            iteration_start = time.time()

            action = action_list[step]

        



            self.publish_action(action)


            iteration_duration = time.time() - iteration_start
            sleep_time = desired_interval - iteration_duration
        
            if sleep_time > 0:
                rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility


            step = step+1

            if step == len(action_list):

                print("finished!")
                os._exit(0)           




    def _easeInOutQuad(self, t):
        t *= 2
        if t < 1:
            return t * t / 2
        else:
            t -= 1
            return -(t * (t - 2) - 1) / 2


    def publish_action(self, action):
        # Publish joint states to ROS

        neck_action = action[:7]
        right_arm_action = action[7:15]
        left_arm_action = action[15:]
        print(left_arm_action.shape)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, neck_action))
        self.neck_pub_tracker.publish(tracker_msg)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, right_arm_action))
        self.right_pub_tracker.publish(tracker_msg)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, left_arm_action))
        self.left_pub_tracker.publish(tracker_msg)




    def rgb_callback(self, msg):
        # Convert the CompressedImage message to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        rgb_image = cv2.rotate(rgb_image, cv2.ROTATE_90_CLOCKWISE)

        # Display the RGB image
        cv2.imshow('iPhone RGB Subscriber', rgb_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User requested shutdown')

        self.current_image = cv2.resize(rgb_image, (96, 96))



    def _rotate(self):

        end_pose = np.array([0.3, 0, 0.15, 0, 0, 0])
        eefstate = EEFState(end_pose, 0)
        current_state = self.controller.get_eef_state()

        eefstate.timestamp = current_state.timestamp + 1
        self.controller.set_eef_cmd(eefstate)
        # time.sleep(2)


        self.start_pose = end_pose + np.array([0, 0, 0, 0, np.pi/9, 0])



    def _filter_head_pose(self, vr_start_pose, pos_speed, ori_speed, exe_queue_head, head_mat):


        vr_pose = self._mat2pose(head_mat)
        execute = np.array(vr_pose) - np.array(vr_start_pose)
        execute[3:] = np.array(vr_pose)[3:]


        x = -execute[2]
        y = -execute[0]
        z = execute[1]
        execute[0] = x
        execute[1] = y
        execute[2] = z
        roll = execute[3] # rot
        pitch = execute[4] # left right
        yaw = execute[5] # up down
        execute[3] = -roll
        execute[4] = -yaw
        execute[5] = pitch
        execute[:3] = execute[:3] * pos_speed 
        execute[3:] = execute[3:] * ori_speed  




        # smooth the motion
        if (
                exe_queue_head.maxsize > 0
                and exe_queue_head._qsize() == exe_queue_head.maxsize
        ):
            exe_queue_head._get()
        exe_queue_head.put_nowait(execute)

        execute_smooth = np.mean(np.array(list(exe_queue_head.queue)), axis=0)
        


        return execute_smooth
        




    def _mat2pose(self, matrix):
        # Extract the rotation matrix
        R = matrix[:3, :3]
        
        # Extract the translation vector
        translation = matrix[:3, 3]
        
        # Calculate pitch (theta)
        theta = np.arcsin(-R[2, 0])
        
        # Calculate the cosine of theta for use in other calculations
        cos_theta = np.cos(theta)
        
        # Calculate yaw (psi)
        if cos_theta != 0:
            psi = np.arctan2(R[1, 0] / cos_theta, R[0, 0] / cos_theta)
        else:
            psi = 0  # When cos_theta is zero, the yaw cannot be computed directly
        
        # Calculate roll (phi)
        if cos_theta != 0:
            phi = np.arctan2(R[2, 1] / cos_theta, R[2, 2] / cos_theta)
        else:
            phi = 0  # When cos_theta is zero, the roll cannot be computed directly

        yaw = np.array(psi)
        pitch = np.array(theta)
        roll = np.array(phi)
        
        # Convert radians to degrees
        
        # yaw = np.degrees(psi)
        # pitch = np.degrees(theta)
        # roll = np.degrees(phi)
        
        # Return results
        return [translation[0], translation[1], translation[2], yaw, pitch, roll]

if __name__ == "__main__":




    rospy.init_node("Replay")
    replay_node = Replay()
