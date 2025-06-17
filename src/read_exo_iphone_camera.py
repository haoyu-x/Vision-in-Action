#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, UInt8MultiArray
import cv2
from cv_bridge import CvBridge
from record3d import Record3DStream
from PIL import Image as PILImage

import open3d as o3d
from multiprocessing import Event
import time
from scipy.spatial.transform import Rotation as R
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.transformation_helper import quaternion_camera_pose_to_extrinsic_matrix
from utils.transformation_helper import resize_depth_and_rgb, rpy_pose_extrinsic_matrix, rgbd_to_camera_frame_pcd



class ReadiPhone:
    def __init__(self):
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1

        self.connect_to_device(dev_idx=1)

        # ROS Publishers
        self.pub_RGB = rospy.Publisher("/exo/iphone_rgb", CompressedImage, queue_size=10)
        self.pub_Depth = rospy.Publisher("/exo/iphone_depth", Image, queue_size=10)
        self.pub_extrinsic = rospy.Publisher("/exo/iphone_extrinsic", JointState, queue_size=10)
        self.pub_intrinsic = rospy.Publisher("/exo/iphone_intrinsic", JointState, queue_size=10)

        # Initialize CvBridge
        bridge = CvBridge()
        desired_interval = 1 / 30.0
        self.rgb_width = 720
        self.rgb_height = 960

        self.depth_width = 192
        self.depth_height = 256

        scale = 1
        self.resolution = (int(self.depth_height * scale), int(self.depth_width * scale))


        # open3d
        visual_o3d = 0
        if visual_o3d:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window("iPhone Point Cloud Steaming", width=self.rgb_width, height=self.rgb_height)
            self.vis.get_view_control()
            self.pcd = o3d.geometry.PointCloud()
            self.init_camera_pose = None

            
        while not rospy.is_shutdown():
            iteration_start = time.time()
            self.event.wait()  # Wait for new frame to arrive


            # Copy the newly arrived RGBD frame
            rgb = self.session.get_rgb_frame()
            depth = self.session.get_depth_frame()

            # depth, rgb = resize_depth_and_rgb(depth, rgb, self.resolution)

            # confidence = self.session.get_confidence_frame()
            intrinsic = self.get_intrinsic_mat_from_coeffs(self.session.get_intrinsic_mat())

            camera_pose = self.session.get_camera_pose()  
            # camera_pose.[qx|qy|qz|qw|tx|ty|tz])
            extrinsic = quaternion_camera_pose_to_extrinsic_matrix(camera_pose)


            d = time.time()
            timestamp = rospy.Time.now()
            # Convert the images to ROS messages


            width = int(rgb.shape[1])
            height = int(rgb.shape[0])
            dim = (width, height)
            visual_rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            # visual_rgb = cv2.resize(visual_rgb, dim, interpolation=cv2.INTER_AREA)
            visual_rgb = cv2.rotate(visual_rgb, cv2.ROTATE_90_CLOCKWISE)


            visual_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)  # Normalize to 0-255
            visual_depth = cv2.resize(visual_depth, dim, interpolation=cv2.INTER_AREA)
            visual_depth = cv2.rotate(visual_depth, cv2.ROTATE_90_CLOCKWISE)
            visual_depth = cv2.applyColorMap(visual_depth, cv2.COLORMAP_JET)

            # Stack the RGB and Depth images horizontally
            combined_image = np.hstack((visual_rgb, visual_depth))
            # print(visual_rgb.shape)
            # print(visual_rgb.mean())
            cv2.namedWindow('iphone RGBD', cv2.WINDOW_NORMAL)
            cv2.imshow('iphone RGBD', visual_rgb)
            # Check for a quit signal
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



            rgb = cv2.cvtColor(visual_rgb, cv2.COLOR_BGR2RGB)
            _, compressed_rgb = cv2.imencode('.jpg', np.array(rgb).astype(np.uint8))
            rgb_msg = CompressedImage()
            rgb_msg.header.stamp = timestamp
            rgb_msg.format = "jpeg"
            rgb_msg.data = np.array(compressed_rgb).tobytes()

            depth_msg = bridge.cv2_to_imgmsg(np.array(depth), encoding="32FC1")
            depth_msg.header.stamp = timestamp

         

            intrinsic_msg = JointState()
            intrinsic_msg.position = list(map(float, intrinsic.flatten()))
            intrinsic_msg.header.stamp = timestamp

            print(extrinsic)
            extrinsic_msg = JointState()
            extrinsic_msg.position = list(map(float, extrinsic.flatten()))
            extrinsic_msg.header.stamp = timestamp

 

            self.pub_RGB.publish(rgb_msg)
            self.pub_Depth.publish(depth_msg)
            self.pub_extrinsic.publish(extrinsic_msg)
            self.pub_intrinsic.publish(intrinsic_msg)
 
 




            if visual_o3d:


                if self.init_camera_pose is None:


                    self.init_camera_pose = extrinsic

                    self.get_global_xyz(depth, cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), intrinsic, extrinsic, depth_scale=1000.0, only_confident=False)

        
                    self.vis.add_geometry(self.pcd)


                else:
                    self.get_global_xyz(depth, cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), intrinsic, extrinsic, depth_scale=1000.0, only_confident=False)

                    self.vis.update_geometry(self.pcd)

                    self.vis.poll_events()
                    self.vis.update_renderer()













            iteration_duration = time.time() - iteration_start
            sleep_time = desired_interval - iteration_duration
        
            if sleep_time > 0:
                rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility

            # print("iphone FPS", 1/(time.time() - iteration_start))



    def get_global_xyz(self, depth, rgb, intrinsics, extrinsic, depth_scale=1000.0, only_confident=False):

        depth_o3d = o3d.geometry.Image(
            np.ascontiguousarray(depth_scale * depth).astype(np.float32)
        )
        rgb_o3d = o3d.geometry.Image(
            np.ascontiguousarray(rgb).astype(np.uint8)
        )

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d, convert_rgb_to_intensity=False
        )


        # print("intrinsics", intrinsics)
        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=int(self.depth_width),
            height=int(self.depth_height),
            fx=intrinsics[0, 0] * self.depth_width / self.rgb_width,
            fy=intrinsics[1, 1] * self.depth_height / self.rgb_height,
            cx=intrinsics[0, 2] * self.depth_width / self.rgb_width,
            cy=intrinsics[1, 2] * self.depth_height / self.rgb_height,
        )


        temp = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, camera_intrinsics)
        temp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        self.pcd.points = temp.points
        self.pcd.colors = temp.colors


        # Now transform everything by camera pose to world frame.

        self.pcd.transform(extrinsic)

        self.pcd.transform(np.linalg.inv(self.init_camera_pose))

        # save pcd

        # self.save_pcds.append(self.pcd)



    def on_new_frame(self):
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        print('Stream stopped')

    def connect_to_device(self, dev_idx):
        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'.format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])





if __name__ == "__main__":
    rospy.init_node("ReadEXO")
    read_iphone = ReadiPhone()
