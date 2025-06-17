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
import open3d as o3d


import argparse
import pickle
import os
from cv_bridge import CvBridge

global bridge
bridge = CvBridge()
import gzip


#!/usr/bin/env python3
import os
import sys

sys.path.append("../../arx5-sdk/python")
sys.path.append("../../arx5-sdk/lib")
import arx5_interface as arx5
from arx5_interface import Arx5CartesianController, EEFState, Gain, LogLevel
from scipy.spatial.transform import Rotation as R
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from utils.transformation_helper import pose_to_homogeneous, resize_depth_and_rgb, rpy_pose_extrinsic_matrix, rgbd_to_camera_frame_pcd, T_ET, neck_urdf_path, x_min, x_max, y_max, y_min, z_max, z_min, T_left_shoulder_to_body, T_right_shoulder_to_body

from utils.plotly_vis import Visualizer
import zarr
import numpy as np
from pathlib import Path
import gzip
import argparse
import os
import torch
# import torch_fpsample
# refer: https://github.com/leonardodalinky/pytorch_fpsample



def save_to_zarr(save_path, data):
    """Save dictionary data to a Zarr file."""
    store = zarr.ZipStore(save_path, mode='w')
    root = zarr.group(store)

    # Save each item in the data dictionary
    for key, value in data.items():
        if isinstance(value, list) and all(isinstance(arr, np.ndarray) for arr in value):
            # Stack arrays if they are of consistent shapes, otherwise store as individual datasets
            if all(arr.shape == value[0].shape for arr in value):
                # Stack and save if shapes are consistent
                data_arr = np.stack(value)
                root.array(
                    key,
                    data_arr,
                    dtype=data_arr.dtype,
                    compressor=zarr.Blosc(cname='zstd', clevel=3, shuffle=zarr.Blosc.SHUFFLE)
                )
            else:
                # Save as individual datasets if shapes vary
                group = root.create_group(key)
                for i, arr in enumerate(value):
                    group.array(
                        f"{key}_{i}",
                        arr,
                        dtype=arr.dtype,
                        compressor=zarr.Blosc(cname='zstd', clevel=3, shuffle=zarr.Blosc.SHUFFLE)
                    )
        else:
            # If the item is not a list or doesn't contain numpy arrays, save directly
            root.array(
                key,
                value,
                dtype=value[0].dtype if isinstance(value[0], np.ndarray) else None,
                compressor=zarr.Blosc(cname='zstd', clevel=3, shuffle=zarr.Blosc.SHUFFLE)
            )

    store.close()
    print(f"Saved data to {save_path}")

urdf_path = "../../arx5-sdk/models/arx5_webcam.urdf"

# joint to EE pose w.r.t shoulder
joint_dof = 6
robot_config = arx5.RobotConfigFactory.get_instance().get_config("L5")
solver = arx5.Arx5Solver(
                            urdf_path,
                            joint_dof,
                            robot_config.joint_pos_min,
                            robot_config.joint_pos_max,
                            # "base_link",
                            # "eef_link",
                            # np.array([0, 0, -9.807], dtype=np.float64),
                            )


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


def quaternion_to_homogeneous_matrix(pose):
    # Convert quaternion to rotation matrix
    r = R.from_quat(pose[-4:])
    rotation_matrix = r.as_matrix()
    
    # Create the homogeneous transformation matrix (4x4)
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix  # Top-left 3x3 is the rotation matrix
    homogeneous_matrix[:3, 3] = pose[:3]  # Top-right 3x1 is the translation vector
    
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




    
def subsample_bag(rosbag_path: Path, save_new=True):

    print(rosbag_path)
    filename = os.path.basename(rosbag_path)              # '03_31_25-14:45:54.bag'
    name_without_ext = os.path.splitext(filename)[0]   # '03_31_25-14:45:54'

    directory = os.path.dirname(rosbag_path) + "/"

    # new_bag_path = rosbag_path.parent / (rosbag_path.stem + "_subsample.bag")
    # if new_bag_path.exists():
    #     my_bag = rosbag.Bag(new_bag_path, 'r')
    #     all_topics = fetch_unique_topics(my_bag)
    #     aligned_msg_map = {}
    #     for topic in all_topics:
    #         msg, timestamps = get_messages_from_bag(my_bag, topic)
    #         aligned_msg_map[topic] = msg
    #     return aligned_msg_map
        
    my_bag = rosbag.Bag(rosbag_path, 'r')
    
    all_topics = fetch_unique_topics(my_bag)
    cam_topics = [topic for topic in all_topics if "RGB" in topic]



    print("===== Raw Message =====")

    for topic in all_topics:
        print(f"{topic} : {len(get_messages_from_bag(my_bag, topic)[0])} messages")
        msg, time = get_messages_from_bag(my_bag, topic)
        print(time[0].to_sec(), time[-1].to_sec(), "duration", time[-1].to_sec()- time[0].to_sec())

    main_topic ="/camera/right_RGB"
    main_msg, main_timestamps = get_messages_from_bag(my_bag, main_topic)
    print(f"Main topic = '{main_topic}': {len(main_msg)} messages")
    





    msg_timestamp_map = {}
    for topic in all_topics:
        if topic != main_topic:
            msg_timestamp_map[topic] = get_messages_from_bag(my_bag, topic)
    
    aligned_msg_map = {main_topic: main_msg}
    for topic, (messages, timestamps) in msg_timestamp_map.items():
        if not messages: continue
        aligned_messages = []
        pointer = 0



        print(topic)
        timestamp_error = 0
        for main_timestamp in main_timestamps:
            while pointer+1<len(messages) \
                and timestamps[pointer+1].to_sec() < main_timestamp.to_sec():
                pointer += 1


            paired_msg = messages[pointer]
            paired_timestamp = timestamps[pointer]
            timestamp_error = timestamp_error + timestamps[pointer].to_sec()- main_timestamp.to_sec()



            paired_msg.header.stamp = paired_timestamp
            aligned_messages.append(paired_msg)
        print("mean timestamp_error", timestamp_error/len(aligned_messages))

        assert len(aligned_messages) == len(main_msg)
        aligned_msg_map[topic] = aligned_messages


    print("===== Subsampled Message =====")
    for topic, msgs in aligned_msg_map.items():
        print(f"{topic} : {len(msgs)} messages")
    print("===============================")
    

    # visualization check and turn to replay buffer

    # obs
    main_rgb_msgs = aligned_msg_map["/camera/iphone_rgb"]
    main_depth_msgs = aligned_msg_map["/camera/iphone_depth"]


    exo_rgb_msgs = aligned_msg_map["/exo/iphone_rgb"]


    right_RGB_msgs = aligned_msg_map["/camera/right_RGB"]
    left_RGB_msgs = aligned_msg_map["/camera/left_RGB"]
    top_RGB_msgs = aligned_msg_map["/camera/top_RGB"]




    train_pcd_list = []
    train_iphone_rgb_list = []

    train_right_rgb_list = []
    train_left_rgb_list = []
    train_top_rgb_list = []

    train_proprio_list = []
    train_action_list = []

    train_redundant_proprio_list = []








    rgb_width = 720
    rgb_height = 960

    depth_width = 192
    depth_height = 256
    downsample_rate = 1

    scale = 1
    resolution = (int(depth_height * scale), int(depth_width * scale))

    space_range = [x_min, x_max, y_min, y_max, z_min, z_max]

    plotly_vis = Visualizer()


    file_name = os.path.splitext(os.path.basename(rosbag_path))[0]



    # Add this before the loop
    # frames = []  # List to store visualization frames
    
    
    frames = []


    for i in range(len(right_RGB_msgs)):
        if i%downsample_rate ==0:
            print(i)
            d = timer.time()
            # iphone rgb input
            
            main_rgb_msg_arr = np.frombuffer(main_rgb_msgs[i].data, np.uint8)
            main_rgb_msg_arr = cv2.imdecode(main_rgb_msg_arr, cv2.IMREAD_COLOR)
            main_rgb_msg_arr = cv2.rotate(main_rgb_msg_arr, cv2.ROTATE_90_CLOCKWISE)
            
            frames.append(cv2.cvtColor(main_rgb_msg_arr, cv2.COLOR_BGR2RGB))


            # exo_rgb_msg_arr = np.frombuffer(exo_rgb_msgs[i].data, np.uint8)
            # exo_rgb_msg_arr = cv2.imdecode(exo_rgb_msg_arr, cv2.IMREAD_COLOR)
            # exo_rgb_msg_arr = cv2.rotate(exo_rgb_msg_arr, cv2.ROTATE_90_CLOCKWISE)
            # exo_frames.append(exo_rgb_msg_arr)



            # # depth   
            # main_depth = main_depth_msgs[i]
            # main_depth = bridge.imgmsg_to_cv2(main_depth, "32FC1")
            # main_depth = np.array(main_depth.copy())
            # main_depth = cv2.rotate(main_depth, cv2.ROTATE_90_CLOCKWISE)
            # visual_depth = cv2.normalize(main_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)  # Normalize to 0-255





            # exo_rgb_msg_arr = np.frombuffer(exo_rgb_msgs[i].data, np.uint8)
            # exo_rgb_msg_arr = cv2.imdecode(exo_rgb_msg_arr, cv2.IMREAD_COLOR)
            # exo_rgb_msg_arr = cv2.cvtColor(exo_rgb_msg_arr, cv2.COLOR_BGR2RGB)
            # visual_exo = exo_rgb_msg_arr


            # main_rgb_msg_arr = cv2.resize(main_rgb_msg_arr, (96, 96))
            # train_iphone_rgb_list.append(main_rgb_msg_arr)


            # # wrist and top rgb input


            # right_RGB_arr = np.frombuffer(right_RGB_msgs[i].data, np.uint8)
            # right_RGB_arr = cv2.imdecode(right_RGB_arr, cv2.IMREAD_COLOR)

            # # fisheye crop
            # height, width, _ = right_RGB_arr.shape
            # crop_size = 720
            # start_x = (width - crop_size) // 2
            # start_y = (height - crop_size) // 2


    
            # right_RGB_arr = right_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # right_wrist_frames.append(right_RGB_arr)




            # left_RGB_arr = np.frombuffer(left_RGB_msgs[i].data, np.uint8)
            # left_RGB_arr = cv2.imdecode(left_RGB_arr, cv2.IMREAD_COLOR)
            # left_RGB_arr = left_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # left_wrist_frames.append(left_RGB_arr)




            # top_RGB_arr = np.frombuffer(top_RGB_msgs[i].data, np.uint8)
            # top_RGB_arr = cv2.imdecode(top_RGB_arr, cv2.IMREAD_COLOR)
            # top_RGB_arr = top_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # chest_frames.append(top_RGB_arr)




            # ########### visualization #############

            # if i == 0:

            #     # Create a window for each camera
            #     window_names = ["visualization"]
            #     for window_name in window_names:
            #         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

  
  


            # # visual_background = visual_exo
            # visual_background = visual_iphone

            # visual_iphone_h = 192
            # visual_iphone_w = 256


            # visual_wrist_h = 144
            # visual_wrist_w = 256

            # visual_iphone_resized = cv2.resize(visual_iphone, (visual_iphone_w, visual_iphone_h))  # For top-right corner
            # # visual_depth_resize = cv2.resize(visual_depth, (visual_iphone_w, visual_iphone_h))  # For top-right corner
            # # visual_depth_resize = cv2.applyColorMap(visual_depth_resize, cv2.COLORMAP_JET)

            # visual_right_resized = cv2.resize(visual_right_RGB_arr, (visual_wrist_w, visual_wrist_h))  # Half for top-left
            # visual_left_resized = cv2.resize(visual_left_RGB_arr, (visual_wrist_w, visual_wrist_h))  # Half for top-left
            # visual_chest_resized = cv2.resize(visual_top_RGB_arr, (visual_wrist_w, visual_wrist_h))  # Half for top-left

            # # Combine the left visuals vertically (visual_right_RGB_arr and visual_left_RGB_arr)
            # visual_combined_pad = np.zeros((visual_background.shape[0], visual_wrist_w, 3), dtype=np.uint8)

            # visual_combined_pad[:visual_iphone_h, :visual_iphone_w] = visual_iphone_resized  # Top half

            # # visual_combined_pad[visual_iphone_h:visual_iphone_h*2, :visual_iphone_w] = visual_depth_resize  # Top half

            # # visual_combined_pad[visual_iphone_h:visual_iphone_h+visual_wrist_h, :visual_wrist_w] = visual_right_resized  # Bottom half
            # # visual_combined_pad[visual_iphone_h+visual_wrist_h:visual_iphone_h+visual_wrist_h*2, :visual_wrist_w] = visual_left_resized  # Bottom half
            # # visual_combined_pad[visual_iphone_h+visual_wrist_h*2:visual_iphone_h+visual_wrist_h*3, :visual_wrist_w] = visual_chest_resized  # Bottom half


            # # Place the left visuals in the top-left corner
            # # visual_background = np.hstack([visual_background, visual_combined_pad])

            # visual_background = cv2.resize(visual_background, (int(visual_background.shape[1]), int(visual_background.shape[0]))) 

            # frames.append(cv2.cvtColor(visual_background, cv2.COLOR_BGR2RGB))
            # iphone_frames.append(cv2.cvtColor(visual_iphone, cv2.COLOR_BGR2RGB))



            # ########## visualization #############

            # cv2.imshow("visualization", (visual_background))
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break


            # iteration_duration = timer.time() - d
            # sleep_time = 1/30 - iteration_duration
        
            # if sleep_time > 0:
            #     timer.sleep(sleep_time)  
                


            # ########## visualization #############



    import imageio.v3 as iio

    frame_rate = 10/downsample_rate  # FPS for the video

    output_file = directory+name_without_ext+'_iphone_output_video.mp4'
    iio.imwrite(output_file, frames, fps=frame_rate, plugin="pyav", codec="libx264")







    frames = []


    for i in range(len(right_RGB_msgs)):
        if i%downsample_rate ==0:
            print(i)
            d = timer.time()
            # iphone rgb input
            
            # main_rgb_msg_arr = np.frombuffer(main_rgb_msgs[i].data, np.uint8)
            # main_rgb_msg_arr = cv2.imdecode(main_rgb_msg_arr, cv2.IMREAD_COLOR)
            # main_rgb_msg_arr = cv2.rotate(main_rgb_msg_arr, cv2.ROTATE_90_CLOCKWISE)
            
            # frames.append(cv2.cvtColor(main_rgb_msg_arr, cv2.COLOR_BGR2RGB))


            exo_rgb_msg_arr = np.frombuffer(exo_rgb_msgs[i].data, np.uint8)
            exo_rgb_msg_arr = cv2.imdecode(exo_rgb_msg_arr, cv2.IMREAD_COLOR)
            # exo_rgb_msg_arr = cv2.rotate(exo_rgb_msg_arr, cv2.ROTATE_90_CLOCKWISE)
            # frames.append(cv2.cvtColor(exo_rgb_msg_arr, cv2.COLOR_BGR2RGB))
            frames.append(exo_rgb_msg_arr)





    output_file = directory+name_without_ext+'_exo_output_video.mp4'
    iio.imwrite(output_file, frames, fps=frame_rate, plugin="pyav", codec="libx264")




    frames = []


    for i in range(len(right_RGB_msgs)):
        if i%downsample_rate ==0:
            print(i)
            d = timer.time()



            right_RGB_arr = np.frombuffer(right_RGB_msgs[i].data, np.uint8)
            right_RGB_arr = cv2.imdecode(right_RGB_arr, cv2.IMREAD_COLOR)

            # fisheye crop
            height, width, _ = right_RGB_arr.shape
            crop_size = 720
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2


    
            right_RGB_arr = right_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            frames.append(cv2.cvtColor(right_RGB_arr, cv2.COLOR_BGR2RGB))








    output_file = directory+name_without_ext+'_right_output_video.mp4'
    iio.imwrite(output_file, frames, fps=frame_rate, plugin="pyav", codec="libx264")





    frames = []


    for i in range(len(right_RGB_msgs)):
        if i%downsample_rate ==0:
            print(i)
            d = timer.time()
 

            # wrist and top rgb input



            left_RGB_arr = np.frombuffer(left_RGB_msgs[i].data, np.uint8)
            left_RGB_arr = cv2.imdecode(left_RGB_arr, cv2.IMREAD_COLOR)

            # fisheye crop
            height, width, _ = left_RGB_arr.shape
            crop_size = 720
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2

            left_RGB_arr = left_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            frames.append(cv2.cvtColor(left_RGB_arr, cv2.COLOR_BGR2RGB))






    output_file = directory+name_without_ext+'_left_wrist_video.mp4'
    iio.imwrite(output_file, frames, fps=frame_rate, plugin="pyav", codec="libx264")






    frames = []


    for i in range(len(right_RGB_msgs)):
        if i%downsample_rate ==0:
            print(i)
            d = timer.time()
 

            # wrist and top rgb input




            top_RGB_arr = np.frombuffer(top_RGB_msgs[i].data, np.uint8)
            top_RGB_arr = cv2.imdecode(top_RGB_arr, cv2.IMREAD_COLOR)

            # fisheye crop
            height, width, _ = top_RGB_arr.shape
            crop_size = 720
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2

            top_RGB_arr = top_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            frames.append(cv2.cvtColor(top_RGB_arr, cv2.COLOR_BGR2RGB))


    output_file = directory+name_without_ext+'_chest_wrist_video.mp4'
    iio.imwrite(output_file, frames, fps=frame_rate, plugin="pyav", codec="libx264")




    print("saved!")






if __name__ == '__main__':

    # Set up argument parser
    parser = argparse.ArgumentParser()
    # parser.add_argument('--name', type=str, required=True, help="Directory to save pickle file") # 'train_data_ee_iphone.zarr.zip'
    parser.add_argument('--data', type=str, required=True, help="Directory to save pickle file") # '/home/haoyux/teddy_data/data_train/'
    args = parser.parse_args()


    # Specify the folder path
    folder_path = args.data
    files_list = []
    # List all files in the folder
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        
        # Check if it is a file (not a directory)
        if os.path.isfile(file_path):
            file_name = folder_path+filename
            # print(file_name)
            files_list.append(file_name)



    for filename in files_list:


        subsample_bag(rosbag_path = filename)

