import rosbag
from pathlib import Path
# import pandas as pd
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
# import open3d as o3d


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
import torch_fpsample
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

    main_topic = "/camera/iphone_rgb"
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
    main_extrinsic_msgs = aligned_msg_map["/camera/iphone_extrinsic"]
    main_intrinsic_msgs = aligned_msg_map["/camera/iphone_intrinsic"]


    right_RGB_msgs = aligned_msg_map["/camera/right_RGB"]
    left_RGB_msgs = aligned_msg_map["/camera/left_RGB"]
    top_RGB_msgs = aligned_msg_map["/camera/top_RGB"]



    neck_proprio_msgs = aligned_msg_map["/neck/proprio"]
    right_arx_proprio_msgs = aligned_msg_map["/arm/right_proprio"]
    left_arx_proprio_msgs = aligned_msg_map["/arm/left_proprio"]

    right_redundant_proprio_msgs = aligned_msg_map["/arm/right_redundant"]
    left_redundant_proprio_msgs = aligned_msg_map["/arm/left_redundant"]
    neck_redundant_proprio_msgs = aligned_msg_map["/neck/neck_redundant"]



    # action
    neck_action_msgs = aligned_msg_map["/neck/action"]
    right_arm_action_msgs = aligned_msg_map["/arm/right_gello"]
    left_arm_action_msgs = aligned_msg_map["/arm/left_gello"]

    train_depth_list = []

    train_pcd_list = []
    train_iphone_rgb_list = []

    train_right_rgb_list = []
    train_left_rgb_list = []
    train_top_rgb_list = []

    train_proprio_list = []
    train_action_list = []

    train_redundant_proprio_list = []






    init_camera_extrinsic = np.array(main_extrinsic_msgs[0].position).reshape((4, 4))

    pose_quat = np.array(neck_proprio_msgs[0].position)#.reshape(self.start_pose.shape)
    rotation_matrix = R.from_quat(pose_quat[-4:]).as_matrix()
    rpy = R.from_matrix(rotation_matrix).as_euler('xyz')
    init_neck_proprio = np.concatenate([pose_quat[:3], rpy])
    init_neck_pose = rpy_pose_extrinsic_matrix(init_neck_proprio)

    # pcd = o3d.geometry.PointCloud()


    rgb_width = 720
    rgb_height = 960

    depth_width = 192
    depth_height = 256
    downsample_rate = 3

    scale = 1
    resolution = (int(224), int(224))

    space_range = [x_min, x_max, y_min, y_max, z_min, z_max]

    plotly_vis = Visualizer()


    file_name = os.path.splitext(os.path.basename(rosbag_path))[0]
    from skimage.transform import resize

    for i in range(len(main_rgb_msgs)):
        if i % downsample_rate ==0:


            main_rgb_msg_arr = np.frombuffer(main_rgb_msgs[i].data, np.uint8)
            main_rgb_msg_arr = cv2.imdecode(main_rgb_msg_arr, cv2.IMREAD_COLOR)

            # Perform the center crop
            crop_size = 184
            height, width, _ = main_rgb_msg_arr.shape
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2
            main_rgb_msg_arr = main_rgb_msg_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # Perform the center crop
          

            main_rgb_msg_arr = cv2.rotate(main_rgb_msg_arr, cv2.ROTATE_90_CLOCKWISE)
            main_rgb_msg_arr = cv2.resize(main_rgb_msg_arr, (224, 224))

            
            train_iphone_rgb_list.append(main_rgb_msg_arr)





            # # depth   
            # main_depth = main_depth_msgs[i]
            # main_depth = bridge.imgmsg_to_cv2(main_depth, "32FC1")
            # main_depth = np.array(main_depth.copy())



            # # Perform the center crop
            # crop_size = 184
            # height, width = main_depth.shape
            # start_x = (width - crop_size) // 2
            # start_y = (height - crop_size) // 2
            # main_depth = main_depth[start_y:start_y+crop_size, start_x:start_x+crop_size]



            # main_depth = cv2.rotate(main_depth, cv2.ROTATE_90_CLOCKWISE)


            # pil_depth = np.asarray(main_depth)
            # new_shape = (224, 224) # Scale up by a factor of 2
            # main_depth = resize(pil_depth, new_shape, mode='reflect', order=0)

    


            # train_depth_list.append(main_depth)




            # wrist and chest rgb input

            right_RGB_arr = np.frombuffer(right_RGB_msgs[i].data, np.uint8)
            right_RGB_arr = cv2.imdecode(right_RGB_arr, cv2.IMREAD_COLOR)
            # Perform the center crop
            crop_size = 720
            height, width, _ = right_RGB_arr.shape
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2
            right_RGB_arr = right_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # Perform the center crop
            right_RGB_arr = cv2.resize(right_RGB_arr, (224, 224))
            train_right_rgb_list.append(right_RGB_arr)



            left_RGB_arr = np.frombuffer(left_RGB_msgs[i].data, np.uint8)
            left_RGB_arr = cv2.imdecode(left_RGB_arr, cv2.IMREAD_COLOR)
            # Perform the center crop
            crop_size = 720
            height, width, _ = left_RGB_arr.shape
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2
            left_RGB_arr = left_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # Perform the center crop
            left_RGB_arr = cv2.resize(left_RGB_arr, (224, 224))
            train_left_rgb_list.append(left_RGB_arr)



            top_RGB_arr = np.frombuffer(top_RGB_msgs[i].data, np.uint8)
            top_RGB_arr = cv2.imdecode(top_RGB_arr, cv2.IMREAD_COLOR)
            # Perform the center crop
            crop_size = 720
            height, width, _ = top_RGB_arr.shape
            start_x = (width - crop_size) // 2
            start_y = (height - crop_size) // 2
            top_RGB_arr = top_RGB_arr[start_y:start_y+crop_size, start_x:start_x+crop_size]
            # Perform the center crop
            top_RGB_arr = cv2.resize(top_RGB_arr, (224, 224))
            train_top_rgb_list.append(top_RGB_arr)


            # ########### visualization #############
            # if i == 0:

            #     # Create a window for each camera
            #     window_names = ["visualization"]
            #     for window_name in window_names:
            #         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

            # cv2.imshow("visualization", (main_rgb_msg_arr))
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            # timer.sleep(1/30)
            # ########### visualization #############



            
            # ########### visualization #############

            # if i == 0:

            #     # Create a window for each camera
            #     window_names = ["visualization"]
            #     for window_name in window_names:
            #         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)


            # visual_iphone_h = 192+96-48
            # visual_iphone_w = 256+128-64

            # visual_wrist_h = 144
            # visual_wrist_w = 256

            # visual_iphone_resized = cv2.resize(visual_iphone, (visual_iphone_w, visual_iphone_h))  # For top-right corner
            # visual_right_resized = cv2.resize(visual_right_RGB_arr, (visual_wrist_w, visual_wrist_h))  # Half for top-left
            # visual_left_resized = cv2.resize(visual_left_RGB_arr, (visual_wrist_w, visual_wrist_h))  # Half for top-left


            # # Combine the left visuals vertically (visual_right_RGB_arr and visual_left_RGB_arr)
            # visual_combined_left = np.zeros((visual_wrist_h*2, visual_wrist_w, 3), dtype=np.uint8)
            # visual_combined_left[:visual_wrist_h, :visual_wrist_w] = visual_left_resized  # Top half
            # visual_combined_left[visual_wrist_h:, :visual_wrist_w] = visual_right_resized  # Bottom half
            # # Place the left visuals in the top-left corner
            # visual_top_RGB_arr[:visual_wrist_h*2, :visual_wrist_w] = visual_combined_left
            # # Place the iPhone visual in the top-right corner
            # visual_top_RGB_arr[:visual_iphone_h, -visual_iphone_w:] = visual_iphone_resized

            # cv2.imshow("visualization", (visual_top_RGB_arr))
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            # timer.sleep(1/30)

            # ########### visualization #############
            



            # depth   
            main_depth = main_depth_msgs[i]
            main_depth = bridge.imgmsg_to_cv2(main_depth, "32FC1")
            main_depth = np.array(main_depth.copy())


            #rgbd

            np_arr = np.frombuffer(main_rgb_msgs[i].data, np.uint8)
            rgb_image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image_np = cv2.cvtColor(rgb_image_np, cv2.COLOR_BGR2RGB)
            main_rgb = np.array(rgb_image_np.copy())

            main_depth, main_rgb = resize_depth_and_rgb(main_depth, main_rgb, resolution)
            main_extrinsic = np.array(main_extrinsic_msgs[i].position).reshape((4, 4))
            main_intrinsic = np.array(main_intrinsic_msgs[i].position).reshape((3, 3))
            # pcd
            pcd = rgbd_to_camera_frame_pcd(main_depth, main_rgb, main_intrinsic)
            # world frame pcd
            pcd.transform(main_extrinsic)
            pcd.transform(np.linalg.inv(init_camera_extrinsic))
            pcd.transform(T_ET)
            pcd.transform(init_neck_pose)


   
            # filter x y z workspace 
            pointcloud = np.zeros((np.array(pcd.points).shape[0], 6))
            pointcloud[:, 0:3] = np.array(pcd.points)
            pointcloud[:, -3:] = np.array(pcd.colors)
            pointcloud = pointcloud[pointcloud[:, 2] >= z_min] 
            pointcloud = pointcloud[pointcloud[:, 1] <= y_max]
            pointcloud = pointcloud[pointcloud[:, 1] >= y_min]
            pointcloud = pointcloud[pointcloud[:, 0] <= x_max]
            pointcloud = pointcloud[pointcloud[:, 0] >= x_min]
 

            # Perform downsampling with torch_fpsample
            num_points = 1024

            pointcloud = torch.tensor(pointcloud, dtype=torch.float32)
            points = pointcloud[:, :3]  # (N, 3) - Spatial coordinates (x, y, z)
            colors = pointcloud[:, 3:]  # (N, 3) - RGB color values

            # Apply FPS on spatial coordinates
            sampled_points, indices = torch_fpsample.sample(points.unsqueeze(0), num_points)  # (1, num_points, 3)

            # Use the indices to get the corresponding color information from the original point cloud
            sampled_colors = colors[indices[0]]  # (num_points, 3)

            # Combine the sampled points and their corresponding colors
            pointcloud = torch.cat([sampled_points.squeeze(0), sampled_colors], dim=1)  # (num_points, 6)
            pointcloud = pointcloud.cpu().numpy()

            # pcd input
            train_pcd_list.append(pointcloud)









            # proprio input

            neck_proprio_arr = np.array(neck_proprio_msgs[i].position)
            right_proprio_arr = np.array(right_arx_proprio_msgs[i].position)
            left_proprio_arr = np.array(left_arx_proprio_msgs[i].position)

            proprio_arr = np.hstack((neck_proprio_arr, right_proprio_arr, left_proprio_arr))
            train_proprio_list.append(proprio_arr)



            # action output

            neck_action_arr = np.array(neck_action_msgs[i].position)


            right_gello_action_arr = np.array(right_arm_action_msgs[i].position)
            right_action_gripper = right_gello_action_arr[6]
            # joints --> EE pose w.r.t shoulder
            right_gello_action_arr[:6] = solver.forward_kinematics(right_gello_action_arr[:6])
            # EE pose w.r.t. shoulder --> EE w.r.t. body frame
            pose_homogeneous = pose_to_homogeneous(right_gello_action_arr[:6])
            body_pose = transform_pose(pose_homogeneous, T_right_shoulder_to_body, is_quaternion=1, is_rpy=0) 
            # body pose, quat
            right_action_msg_arr = np.hstack((body_pose, right_action_gripper))



            left_gello_action_arr = np.array(left_arm_action_msgs[i].position)
            left_action_gripper = left_gello_action_arr[6]
            # joints --> EE pose w.r.t shoulder
            left_gello_action_arr[:6] = solver.forward_kinematics(left_gello_action_arr[:6])
            # EE pose w.r.t. shoulder --> EE w.r.t. body frame
            pose_homogeneous = pose_to_homogeneous(left_gello_action_arr[:6])
            body_pose = transform_pose(pose_homogeneous, T_left_shoulder_to_body, is_quaternion=1, is_rpy=0) 
            # body pose, quat
            left_action_msg_arr = np.hstack((body_pose, left_action_gripper))

            

            action_msg_arr = np.hstack((neck_action_arr, right_action_msg_arr, left_action_msg_arr))
            train_action_list.append(action_msg_arr)




            # # redundant proprio

            # neck_redundant_proprio_arr = np.array(neck_redundant_proprio_msgs[i].position)
            # right_redundant_proprio_arr = np.array(right_redundant_proprio_msgs[i].position)
            # left_redundant_proprio_arr = np.array(left_redundant_proprio_msgs[i].position)

            # redundant_proprio_arr = np.hstack((neck_redundant_proprio_arr, right_redundant_proprio_arr, left_redundant_proprio_arr))
            # train_redundant_proprio_list.append(redundant_proprio_arr)


    # print("saving...")

    # vis_pcds = []
    # pcd = o3d.geometry.PointCloud()
    # for pointcloud in train_pcd_list:
    #     pcd.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    #     pcd.colors = o3d.utility.Vector3dVector(pointcloud[:, -3:])

    #     # Perform downsampling
    #     num_points = 5000
    #     pcd = pcd.farthest_point_down_sample(num_samples=num_points) 
    #     pointcloud = np.zeros((np.array(pcd.points).shape[0], 6))        
    #     pointcloud[:, 0:3] = np.array(pcd.points)
    #     pointcloud[:, -3:] = np.array(pcd.colors)*255
    #     vis_pcds.append(pointcloud)

    # plotly_vis.save_pointclouds_interactive(vis_pcds,  space_range=space_range,  file_path="/home/haoyux/teddy_data/"+str(file_name)+"_save.html")


    return train_pcd_list, train_depth_list, train_iphone_rgb_list, train_top_rgb_list, train_right_rgb_list, train_left_rgb_list, train_proprio_list, train_action_list, train_redundant_proprio_list, len(train_pcd_list)





if __name__ == '__main__':

    # Set up argument parser
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', type=str, required=True, help="Directory to save pickle file") # 'train_data_ee_iphone.zarr.zip'
    parser.add_argument('--data', type=str, required=True, help="Directory to load raw data") # '/home/haoyux/teddy_data/data_train/'
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


    train_pcd = []
    train_depth = []
    train_img = []

    train_top_rgb = []
    train_left_rgb = []
    train_right_rgb = []

    train_proprio = []
    train_action = []

    train_redundant_proprio = []

    episode_ends = []
    episode_index = -1
    count = 0
    for filename in files_list:
        count = count + 1
        print(count)
        print(filename)


        train_pcd_list, train_depth_list, train_img_list, train_top_rgb_list, train_right_rgb_list, train_left_rgb_list, train_proprio_list, train_action_list, train_redundant_proprio_list, length = subsample_bag(rosbag_path = filename)

        train_pcd = train_pcd + train_pcd_list
        train_depth = train_depth +train_depth_list
        train_img = train_img +train_img_list
        train_top_rgb = train_top_rgb + train_top_rgb_list
        train_right_rgb = train_right_rgb + train_right_rgb_list
        train_left_rgb = train_left_rgb + train_left_rgb_list
        train_redundant_proprio = train_redundant_proprio + train_redundant_proprio_list
        train_proprio = train_proprio +train_proprio_list
        train_action = train_action +train_action_list
        episode_index = episode_index + length
        episode_ends.append(episode_index)


    delta_distances = [np.sum(np.abs(train_action[i] - train_action[i-1])) for i in range(1, len(train_action))]

    import statistics
    print("del", statistics.mean(delta_distances))
    print("del", min(delta_distances))
    print("del", max(delta_distances))


    # Prepare save data dictionary
    # save_dict = {
    #     "pointcloud": np.array(train_pcd),
    #     "image": np.array(train_img),
    #     "proprio": np.array(train_proprio),
    #     "action": np.array(train_action),
    #     "episode_ends": np.array(episode_ends)
    # }
    save_dict = {
        # "main-depth": train_depth,
        "main-image": train_img,
        "right-image": train_right_rgb,
        "left-image": train_left_rgb,
        "top-image": train_top_rgb,
        "pointcloud": train_pcd,


        "proprio": train_proprio,
        # "redundant_proprio": train_redundant_proprio,

        "action": train_action,
        "episode_ends": episode_ends
    }
    # Save to Zarr
    save_path = os.path.join("/store/real/haoyux", args.name)
    save_to_zarr(save_path, save_dict)
    print("saved!")

