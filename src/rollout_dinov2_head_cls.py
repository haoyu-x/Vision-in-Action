
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



from cv_bridge import CvBridge

global bridge
bridge = CvBridge()

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

sys.path.append("../via_diffusion_policy")
from models import get_resnet, replace_bn_with_gn, ConditionalUnet1D

import threading



import torch
# import torch_fpsamples
# refer: https://github.com/leonardodalinky/pytorch_fpsample
import zarr


from sensor_msgs.msg import Image as Image_class

from diffusers.training_utils import EMAModel
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.schedulers.scheduling_ddim import DDIMScheduler

import collections

import torch
import torch.nn as nn
from scipy.spatial.transform import Rotation as R

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


# normalize data
def get_data_stats(data):
    data = data.reshape(-1,data.shape[-1])
    stats = {
        'min': np.min(data, axis=0),
        'max': np.max(data, axis=0)
    }

    return stats

def normalize_data(data, stats):
    # nomalize to [0,1]
    ndata = (data - stats['min']) / (stats['max'] - stats['min'])
    # normalize to [-1, 1]
    ndata = ndata * 2 - 1
    return ndata

def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    data = ndata * (stats['max'] - stats['min']) + stats['min']
    return data

def normalize_data(data, stats):
    # Normalize to [0,1]
    diff = stats['max'] - stats['min']
    diff[diff == 0] = 1  # Avoid division by zero by setting diff to 1 where max == min
    ndata = (data - stats['min']) / diff
    
    # Normalize to [-1,1]
    ndata = ndata * 2 - 1
    return ndata

def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    diff = stats['max'] - stats['min']
    diff[diff == 0] = 1  # Avoid division by zero by setting diff to 1 where max == min
    data = ndata * diff + stats['min']
    return data

def normalize_image(train_image):
    train_image = train_image * (1. / 127.5) - 1.0
    return train_image


def quaternion_to_homogeneous_matrix(pose):
    # Convert quaternion to rotation matrix
    r = R.from_quat(pose[-4:])
    rotation_matrix = r.as_matrix()
    
    # Create the homogeneous transformation matrix (4x4)
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix  # Top-left 3x3 is the rotation matrix
    homogeneous_matrix[:3, 3] = pose[:3]  # Top-right 3x1 is the translation vector
    
    return homogeneous_matrix

def pose_to_homogeneous(pose):
    """Convert a 6D pose (x, y, z, roll, pitch, yaw) into a 4x4 homogeneous matrix."""
    position = pose[:3]
    rpy_angles = pose[3:]
    
    # Convert Euler angles to a rotation matrix
    rotation_matrix = R.from_euler('xyz', rpy_angles, degrees=False).as_matrix()
    
    # Create homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = position
    return homogeneous_matrix

def transform_pose(pose_homogeneous, transformation_matrix, is_quaternion = 0, is_rpy = 0):
    """Transform a 6D pose using a transformation matrix."""
    # Convert the input pose to a homogeneous matrix
    
    # Apply the transformation matrix (multiplication)
    transformed_pose_homogeneous = transformation_matrix @ pose_homogeneous
    
    # Extract position and orientation from the transformed homogeneous matrix
    transformed_position = transformed_pose_homogeneous[:3, 3]
    transformed_rotation_matrix = transformed_pose_homogeneous[:3, :3]
    
    # Convert the rotation matrix back to Euler angles (xyz convention)

    transformed_rpy = R.from_matrix(transformed_rotation_matrix).as_euler('xyz')
    transformed_rotvec = R.from_matrix(transformed_rotation_matrix).as_rotvec()
    quaternion = R.from_matrix(transformed_rotation_matrix).as_quat()  # Quaternion is [x, y, z, w]

    # Return the transformed 6D pose
    if is_quaternion: 
        return np.concatenate([transformed_position, quaternion])

    if is_rpy:
        return np.concatenate([transformed_position, transformed_rpy])

def inverse_transformation_matrix(T):
    """Compute the inverse of a 4x4 homogeneous transformation matrix."""
    R_inv = T[:3, :3].T  # Transpose of the rotation matrix
    t_inv = -R_inv @ T[:3, 3]  # Invert the translation
    
    # Construct the inverse transformation matrix
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv




def load_from_zarr(save_path):
    # Open the Zarr file
    zarr_store = zarr.open(save_path, mode='r')
    
    # Load data from the store
    data = {}
    for key in zarr_store:
        data[key] = zarr_store[key][:]
        
    return data



# Standard ImageNet normalization constants
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

def normalize_image_imagenet(image_array: np.ndarray) -> np.ndarray:
    """
    image_array: (N, H, W, 3) or (H, W, 3)  (i.e., channel-last) *before* you move axis,
                 or (N, 3, H, W) or (3, H, W) (channel-first) *after* you move axis.
    We just need to handle the shape carefully.
    
    Steps:
      1) scale from [0,255] to [0,1]
      2) subtract mean, divide by std (per channel)
    """

    # 1) If images are originally in [0,255], scale to [0,1]
    #    But only if you haven’t already scaled them!
    #    For example, if your original images are already float in [0,1], skip this step.
    #    Here we assume they are in [0,255].
    #    If you’ve already done `image_array = image_array / 255.0` prior to calling
    #    this function, remove this line.
    #
    # image_array /= 255.0

    # 2) Subtract mean, divide by std. This depends on shape.
    #    We'll handle (N, 3, H, W), (3, H, W), (N, H, W, 3), (H, W, 3).
    
    if image_array.ndim == 4:
        # Possibly (N, H, W, 3) or (N, 3, H, W)
        if image_array.shape[1] == 3:
            # shape = (N, 3, H, W) → channel-first
            # We can broadcast mean/std across [N, 3, H, W]
            image_array[:, 0, :, :] = (image_array[:, 0, :, :] - IMAGENET_MEAN[0]) / IMAGENET_STD[0]
            image_array[:, 1, :, :] = (image_array[:, 1, :, :] - IMAGENET_MEAN[1]) / IMAGENET_STD[1]
            image_array[:, 2, :, :] = (image_array[:, 2, :, :] - IMAGENET_MEAN[2]) / IMAGENET_STD[2]
        else:
            # shape = (N, H, W, 3) → channel-last
            image_array[:, :, :, 0] = (image_array[:, :, :, 0] - IMAGENET_MEAN[0]) / IMAGENET_STD[0]
            image_array[:, :, :, 1] = (image_array[:, :, :, 1] - IMAGENET_MEAN[1]) / IMAGENET_STD[1]
            image_array[:, :, :, 2] = (image_array[:, :, :, 2] - IMAGENET_MEAN[2]) / IMAGENET_STD[2]

    elif image_array.ndim == 3:
        # Possibly (3, H, W) or (H, W, 3)
        if image_array.shape[0] == 3:
            # shape = (3, H, W)
            image_array[0, :, :] = (image_array[0, :, :] - IMAGENET_MEAN[0]) / IMAGENET_STD[0]
            image_array[1, :, :] = (image_array[1, :, :] - IMAGENET_MEAN[1]) / IMAGENET_STD[1]
            image_array[2, :, :] = (image_array[2, :, :] - IMAGENET_MEAN[2]) / IMAGENET_STD[2]
        else:
            # shape = (H, W, 3)
            image_array[:, :, 0] = (image_array[:, :, 0] - IMAGENET_MEAN[0]) / IMAGENET_STD[0]
            image_array[:, :, 1] = (image_array[:, :, 1] - IMAGENET_MEAN[1]) / IMAGENET_STD[1]
            image_array[:, :, 2] = (image_array[:, :, 2] - IMAGENET_MEAN[2]) / IMAGENET_STD[2]

    else:
        raise ValueError(f"Expected 3D or 4D shape for image, got shape {image_array.shape}")

    return image_array



class Rollout:
    def __init__(self):



        # Transformation matrix from world frame to right shoulder frame
        self.T_right_shoulder_to_world = np.array([
            [1,  0,     0,       0.18],
            [0, -0.707, -0.707, -0.14563],
            [0, 0.707, -0.707,  0.037426],
            [0,  0,     0,          1]
            ])  

        self.T_world_to_right_shoulder = inverse_transformation_matrix(self.T_right_shoulder_to_world)


        self.T_left_shoulder_to_world = np.array([
            [1,  0,     0,       0.18],
            [0, -0.707, 0.707, 0.14563],
            [0, -0.707, -0.707,  0.037426],
            [0,  0,     0,          1]
            ])  
        
        self.T_world_to_left_shoulder = inverse_transformation_matrix(self.T_left_shoulder_to_world)

        np.set_printoptions(precision=3, suppress=True)


        self.sub_RGB = rospy.Subscriber("/camera/iphone_rgb", CompressedImage, self.rgb_callback)
        self.sub_Depth = rospy.Subscriber("/camera/iphone_depth", Image_class, self.depth_callback)
        self.sub_extrinsic = rospy.Subscriber("/camera/iphone_extrinsic", JointState, self.extrinsic_callback)
        self.sub_intrinsic = rospy.Subscriber("/camera/iphone_intrinsic", JointState, self.intrinsic_callback)


        self.sub_right_RGB = rospy.Subscriber("/camera/right_RGB", CompressedImage, self.right_rgb_callback)
        self.sub_left_RGB = rospy.Subscriber("/camera/left_RGB", CompressedImage, self.left_rgb_callback)


        self.nech_proprio_Subscriber = rospy.Subscriber("/neck/proprio", JointState, self.neck_callback)
        self.right_arm_proprio_Subscriber = rospy.Subscriber("/arm/right_proprio", JointState, self.right_callback)
        self.left_arm_proprio_Subscriber = rospy.Subscriber("/arm/left_proprio", JointState, self.left_callback)



        self.neck_pub_tracker = rospy.Publisher("/neck/action", JointState, queue_size=10)
        self.right_pub_tracker = rospy.Publisher(name=("/arm/right_action"), data_class=JointState, queue_size=3)
        self.left_pub_tracker = rospy.Publisher(name=("/arm/left_action"), data_class=JointState, queue_size=3)
        


        self.neck_init_pub = rospy.Publisher("/neck/init_neck_action", JointState, queue_size=10)
        self.right_init_pub = rospy.Publisher("/arm/init_right_action", JointState, queue_size=10)
        self.left_init_pub = rospy.Publisher("/arm/init_left_action", JointState, queue_size=10)





        # parameters
        pred_horizon = 16
        obs_horizon = 1
        action_horizon = 8
        #|o|o|                             observations: 2
        #| |a|a|a|a|a|a|a|a|               actions executed: 8
        #|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16

        # load model
        action_dim = 23
        lowdim_obs_dim = 23
        num_diffusion_iters = 50
        test_noise_steps = 16


        time.sleep(1)
        obs = dict()
        # keep a queue of last 2 steps of observations
        self.obs_deque = collections.deque(
            [obs] * obs_horizon, maxlen=obs_horizon)

        t = threading.Thread(target=self._get_obs)

        # Start both threads
        t.start()
    


   

        load_dir = "/home/haoyux/ckp_final"
        # shelf
        # model_name = 'teddy_head_rgb_dinov2_cls_ema_500.pth' 
        # train_dataset_path= '/home/haoyux/teddy_data/shelf_rgbd_train.zarr.zip'
        
        # pot
        model_name = 'teddy_head_rgb_dinov2_cls_pot_lime_ema_500.pth'
        train_dataset_path= '/home/haoyux/teddy_data/pot_train.zarr.zip'
    
    
        # bag
        # model_name = 'teddy_head_rgb_dinov2_cls_bag_ema_500.pth'
        # train_dataset_path= '/home/haoyux/teddy_data/bag_train.zarr.zip'


        loaded_dateset = load_from_zarr(train_dataset_path)

        train_data = {
            # first two dims of state vector are agent (i.e. gripper) locations
            'agent_pos': np.array(loaded_dateset["proprio"]).astype(np.float32),
            'action': np.array(loaded_dateset["action"]).astype(np.float32)
        }
        episode_ends = np.array(loaded_dateset["episode_ends"])

    
        # compute statistics and normalized data to [-1,1]
        stats = dict()
        for key, data in train_data.items():

            stats[key] = get_data_stats(data)

        nets = self.load_model(action_dim = action_dim, obs_horizon = obs_horizon, lowdim_obs_dim=lowdim_obs_dim, load_dir=load_dir, model_name=model_name)



        step_idx = 0

        device = torch.device('cuda')

        noise_scheduler = DDIMScheduler(
            num_train_timesteps=num_diffusion_iters,
            beta_start = .0001,
            beta_end=0.02,
            # the choise of beta schedule has big impact on performance
            # we found squared cosine works the best
            beta_schedule='squaredcos_cap_v2',
            # clip output to [-1,1] to improve stability
            clip_sample=True,
            # our network predicts noise (instead of denoised action)
            set_alpha_to_one=True,
            steps_offset=0,

            prediction_type='epsilon'
        )




        while not rospy.is_shutdown():
            
            if start_flag:

                action_list = train_data["action"][episode_ends[12]+1:episode_ends[13]]
                # action_list = train_data["action"][episode_ends[2]+1:episode_ends[3]]


                print("inital..")
                self.init_action(action_list[0])
                time.sleep(5)
                print("start..")

                step_idx = 0
            else:
                rospy.sleep(0.1)
                continue

            while start_flag:        
                    
                B = 1

                main_image = np.stack([x['main-image'] for x in self.obs_deque])
                # right_image = np.stack([x['right_RGB_arr'] for x in self.obs_deque])
                # left_image = np.stack([x['left_RGB_arr'] for x in self.obs_deque])



                main_image = main_image.astype('float64') / 255.0
                # right_image = right_image.astype('float64') / 255.0
                # left_image = left_image.astype('float64') / 255.0


    
                main_image = normalize_image_imagenet(main_image)
                # right_image = normalize_image_imagenet(right_image)
                # left_image  = normalize_image_imagenet(left_image)

                main_image = np.moveaxis(main_image, -1, 1)
                # right_image = np.moveaxis(right_image, -1, 1)
                # left_image  = np.moveaxis(left_image, -1, 1)

                # main_image = normalize_image(main_image)
                # main_image = np.moveaxis(main_image, -1,1)

                # right_image = normalize_image(right_image)
                # right_image = np.moveaxis(right_image, -1,1)

                # left_image = normalize_image(left_image)
                # left_image = np.moveaxis(left_image, -1,1)



                agent_poses = np.stack([x['agent_pos'] for x in self.obs_deque])
                # images = self._get_current_obs()['image'].reshape(1,96,96,3)
                # agent_poses = self._get_current_obs()['agent_pos']

                # normalize observation
                # nagent_poses = agent_poses #normalize_data(agent_poses, stats=stats['agent_pos'])
                nagent_poses = normalize_data(agent_poses, stats=stats['agent_pos'])
            
                nimages = torch.from_numpy(main_image).to(device, dtype=torch.float32)
                nagent_poses = torch.from_numpy(nagent_poses).to(device, dtype=torch.float32)

                # nright_images = torch.from_numpy(right_image).to(device, dtype=torch.float32)
                # nleft_images = torch.from_numpy(left_image).to(device, dtype=torch.float32)                  





                # infer action
                with torch.no_grad():
                    # get image features
                    d = time.time()
                    print(nimages.shape)
                    main_image_features = nets['vision_encoder'].forward_features(nimages)
                    # (2,512)
                    main_image_features = main_image_features["x_norm_clstoken"] # [1, 256, 384])



                    print("main_image_features",main_image_features.shape)
                    print("nagent_poses",nagent_poses.shape)


                    # print(right_image_features.shape)
                    # concat with low-dim observations
                    obs_features = torch.cat([main_image_features, nagent_poses], dim=-1)

                    # reshape observation to (B,obs_horizon*obs_dim)
                    obs_cond = obs_features.unsqueeze(0).flatten(start_dim=1)




                    # initialize action from Guassian noise
                    noisy_action = torch.randn(
                        (B, pred_horizon, action_dim), device=device)
                    naction = noisy_action

                    # init scheduler
                    noise_scheduler.set_timesteps(test_noise_steps)

                    for k in noise_scheduler.timesteps:
                        # predict noise
                        noise_pred = nets['noise_pred_net'](
                            sample=naction,
                            timestep=k,
                            global_cond=obs_cond
                        )

                        # inverse diffusion step (remove noise)
                        naction = noise_scheduler.step(
                            model_output=noise_pred,
                            timestep=k,
                            sample=naction
                        ).prev_sample



                    # unnormalize action
                    naction = naction.detach().to('cpu').numpy()
                    # (B, pred_horizon, action_dim)
                    naction = naction[0]
                    # action_pred = naction #unnormalize_data(naction, stats=stats['action'])
                    action_pred = unnormalize_data(naction, stats=stats['action'])

                    # only take action_horizon number of actions
                    start = obs_horizon - 1
                    end = start + action_horizon
                    action = action_pred[start:end,:]
                    # (action_horizon, action_dim)
                    print("net", time.time()-d)
                    # time.sleep()

                    desired_interval = 0.1
                    for i in range(len(action)):

                        iteration_start = time.time()

                        print("step", step_idx)                       
                        step_idx += 1

                        naction =  action[i]


                        self.publish_action(naction)


                        iteration_duration = time.time() - iteration_start
                        sleep_time = desired_interval - iteration_duration
                    
                        if sleep_time > 0:
                            rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility


                

                        # iteration_duration = time.time() - iteration_start
                        # sleep_time = control_interval - iteration_duration
                    
                        # if sleep_time > 0:
                        #     rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility





    def depth_callback(self, data):

        try:
            depth = bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth = np.array(depth.copy())


        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")










    def init_action(self, action):
        # Publish joint states to ROS


        neck_action = action[:7]
        right_arm_action = action[7:15]
        left_arm_action = action[15:]


        tracker_msg = JointState()
        tracker_msg.position = list(map(float, neck_action))
        self.neck_init_pub.publish(tracker_msg)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, right_arm_action))
        self.right_init_pub.publish(tracker_msg)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, left_arm_action))
        self.left_init_pub.publish(tracker_msg)

    def publish_action(self, action):
        # Publish joint states to ROS


        neck_action = action[:7]
        right_arm_action = action[7:15]
        left_arm_action = action[15:]


        tracker_msg = JointState()
        tracker_msg.position = list(map(float, neck_action))
        self.neck_pub_tracker.publish(tracker_msg)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, right_arm_action))
        self.right_pub_tracker.publish(tracker_msg)

        tracker_msg = JointState()
        tracker_msg.position = list(map(float, left_arm_action))
        self.left_pub_tracker.publish(tracker_msg)




    def _easeInOutQuad(self, t):
        t *= 2
        if t < 1:
            return t * t / 2
        else:
            t -= 1
            return -(t * (t - 2) - 1) / 2



    def _get_obs(self):

        while True:
                
            obs = dict()
            obs["main-image"] = np.array(self.current_image)

            neck_proprio_arr = np.array(self.current_neck)
            right_proprio_arr = np.array(self.current_right_arm)
            left_proprio_arr = np.array(self.current_left_arm)

            proprio_arr = np.hstack((neck_proprio_arr, right_proprio_arr, left_proprio_arr))


            obs["agent_pos"] = proprio_arr





            self.obs_deque.append(obs)
            time.sleep(0.001)




       
       


    def rgb_callback(self, msg):
        # Convert the CompressedImage message to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform the center crop
        crop_size = 184
        height, width, _ = rgb_image.shape
        start_x = (width - crop_size) // 2
        start_y = (height - crop_size) // 2
        rgb_image = rgb_image[start_y:start_y+crop_size, start_x:start_x+crop_size]
        # Perform the center crop
    
        rgb_image = cv2.rotate(rgb_image, cv2.ROTATE_90_CLOCKWISE)
        self.current_image = cv2.resize(rgb_image, (224, 224))








    def right_rgb_callback(self, msg):
        # Convert the CompressedImage message to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform the center crop
        crop_size = 720
        height, width, _ = rgb_image.shape
        start_x = (width - crop_size) // 2
        start_y = (height - crop_size) // 2
        rgb_image = rgb_image[start_y:start_y+crop_size, start_x:start_x+crop_size]
        # Perform the center crop
    
        self.current_right_image = cv2.resize(rgb_image, (224, 224))



    def left_rgb_callback(self, msg):


        np_arr = np.frombuffer(msg.data, np.uint8)
        rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform the center crop
        crop_size = 720
        height, width, _ = rgb_image.shape
        start_x = (width - crop_size) // 2
        start_y = (height - crop_size) // 2
        rgb_image = rgb_image[start_y:start_y+crop_size, start_x:start_x+crop_size]
        # Perform the center crop
    
        self.current_left_image = cv2.resize(rgb_image, (224, 224))





    def extrinsic_callback(self, data):
        # Convert ROS Float64MultiArray message to NumPy array
        self.extrinsic_matrix = np.array(data.position).reshape((4, 4))
        # rospy.loginfo("Received Extrinsic Matrix")
        # # Process the extrinsic matrix (e.g., use in transformations)
        # print("Extrinsic Matrix:\n", extrinsic_matrix)

    def intrinsic_callback(self, data):
        # Convert ROS Float64MultiArray message to NumPy array
        self.intrinsic_matrix = np.array(data.position).reshape((3, 3))
        # rospy.loginfo("Received Intrinsic Matrix")
        # # Process the intrinsic matrix (e.g., use in camera calibration)
        # print("Intrinsic Matrix:\n", intrinsic_matrix)



    def neck_callback(self, msg): 
        self.current_neck = msg.position

    def right_callback(self, msg): 
        self.current_right_arm = msg.position

    def left_callback(self, msg): 
        self.current_left_arm = msg.position


    def load_model(self, action_dim, obs_horizon, lowdim_obs_dim, load_dir, model_name):

        # observation feature has 514 dims in total per step
        obs_dim = 384 + lowdim_obs_dim



        # create network object
        noise_pred_net = ConditionalUnet1D(
            input_dim=action_dim,
            global_cond_dim=obs_dim*obs_horizon
        )

        dinov2_vits14 = torch.hub.load('facebookresearch/dinov2', 'dinov2_vits14')
        dinov2_vits14 = dinov2_vits14.cuda()
        # the final arch has 2 parts
        nets = nn.ModuleDict({
            # 'pcd_encoder': pcd_encoder,

            'vision_encoder': dinov2_vits14,

            'noise_pred_net': noise_pred_net
        })


        ema = EMAModel(
            parameters=nets.parameters(),
            power=0.75)

        ema_nets = nets
        ema.copy_to(ema_nets.parameters())



        # Load directory
        # Load EMA model
        model_path = os.path.join(load_dir, model_name)

        ema_nets.load_state_dict(torch.load(model_path,  map_location='cpu'))
        print("Models successfully loaded from", load_dir, model_name)
        # Set the model to evaluation mode before inference
        ema_nets.eval()
        ema_nets.to('cuda')


        # # example
        # nimages = torch.randn(2, 3, 96, 96).to('cuda')  # Move to GPU if available
        # # Disable gradient calculation for inference
        # with torch.no_grad():
        #     # Pass the input data through the model for inference
        #     image_features = ema_nets['vision_encoder'](nimages)
        # print("Inference output:", image_features.shape)

        return ema_nets



if __name__ == "__main__":




    import threading
    from pynput import keyboard

    # Global variable to indicate if 'a' key is pressed
    start_flag = False
    end_flag = False

    def monitor_key_presses():
        def on_press(key):
            global start_flag


            try:
                if key.char == 's':
                    start_flag = True
                    print('Key "s" pressed. start_flag set to True.')
                if key.char == 'c':
                    start_flag = False
                    print('Key "c" pressed. end_flag set to True.')
                     
            except AttributeError:
                # Handle special keys if needed
                pass


        # Set up the listener
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    # Create and start the thread
    key_thread = threading.Thread(target=monitor_key_presses)
    key_thread.start()




    rospy.init_node("Rollout")
    rollout_node = Rollout()