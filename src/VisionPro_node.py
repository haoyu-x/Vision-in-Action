#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, JointState
from sensor_msgs.msg import Image as Image_class
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

from cv_bridge import CvBridge
from record3d import Record3DStream
from PIL import Image as PILImage
import cv2
from quaternion import as_rotation_matrix, quaternion
import open3d as o3d

from multiprocessing import Array, Process, shared_memory, Manager, Event, Semaphore







from record3d import Record3DStream

from threading import Event

import numpy as np
import open3d as o3d

from PIL import Image
from quaternion import as_rotation_matrix, quaternion



import plotly.graph_objs as go
import plotly.io as pio
from scipy.spatial.transform import Rotation

import time

from datetime import datetime

import cv2


import time
from vuer import Vuer
from vuer.events import ClientEvent
from vuer.schemas import ImageBackground, group, Hands, DefaultScene# WebRTCStereoVideoPlane, 
from multiprocessing import Array, Process, shared_memory, Manager, Event, Semaphore
import numpy as np
import asyncio

from vuer.schemas import DefaultScene, Ply, PointCloud


import math
import numpy as np

np.set_printoptions(precision=2, suppress=True)
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pytransform3d import rotations

import time
import cv2
# from constants_vuer import *



from TeleVision import OpenTeleVision

import cv2
import sys

import os
import keyboard

from skimage.transform import resize

from queue import Queue

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.transformation_helper import resize_depth_and_rgb, mat2pose, quaternion_camera_pose_to_extrinsic_matrix, arm_urdf_path, neck_urdf_path
from utils.transformation_helper import neck_start_pose, pick_place_neck_start_pose, T_right_shoulder_to_body, T_left_shoulder_to_body, inverse_transformation_matrix, quaternion_to_homogeneous_matrix, pose_to_homogeneous,  transform_pose 

from std_msgs.msg import Float64MultiArray



def adjust_exposure(image, alpha=1.0, beta=0):
    """
    Adjust the exposure of the image.
    
    Parameters:
        image (numpy.ndarray): The input image as a NumPy array.
        alpha (float): Contrast control (1.0 = original, <1 = less contrast, >1 = more contrast).
        beta (int): Brightness control (0 = original brightness).
    
    Returns:
        numpy.ndarray: The adjusted image.
    """
    # Ensure the image is in float32 for processing
    img_float = image.astype(np.float32)
    
    # Adjust brightness and contrast
    adjusted_img = cv2.convertScaleAbs(img_float, alpha=alpha, beta=beta)
    
    return adjusted_img

class VisionPro:
    def __init__(self):


        # Define resolution and crop sizes
        resolution = (960, 720)
        scale = 0.4
        self.resolution = (int(resolution[0] * scale), int(resolution[1] * scale))

        crop_size_w = 0
        crop_size_h = 0
        resolution_cropped = (self.resolution[0] - crop_size_h, self.resolution[1] - 2 * crop_size_w)

        # Define shapes
        rgb_shape = (resolution_cropped[0], resolution_cropped[1], 3)
        depth_shape = (resolution_cropped[0], resolution_cropped[1])
        extrinsic_shape = (4, 4)

        # Calculate sizes in bytes
        rgb_size = np.prod(rgb_shape) * np.uint8().itemsize
        depth_size = np.prod(depth_shape) * np.float32().itemsize
        extrinsic_size = np.prod(extrinsic_shape) * np.float32().itemsize

        # Total size
        total_size = rgb_size + depth_size + extrinsic_size

        # Allocate shared memory for the total size
        self.shm = shared_memory.SharedMemory(create=True, size=total_size)

        # Create numpy arrays with offsets
        self.rgb_array = np.ndarray(rgb_shape, dtype=np.uint8, buffer=self.shm.buf[:rgb_size])
        self.depth_array = np.ndarray(depth_shape, dtype=np.float32, buffer=self.shm.buf[rgb_size:rgb_size + depth_size])
        self.extrinsic_array = np.ndarray(extrinsic_shape, dtype=np.float32, buffer=self.shm.buf[rgb_size + depth_size:])



        global bridge
        bridge = CvBridge()


        self.sub_RGB = rospy.Subscriber("/camera/iphone_rgb", CompressedImage, self.rgb_callback)
        self.sub_Depth = rospy.Subscriber("/camera/iphone_depth", Image_class, self.depth_callback)
        self.sub_extrinsic = rospy.Subscriber("/camera/iphone_extrinsic", JointState, self.extrinsic_callback)
        self.sub_intrinsic = rospy.Subscriber("/camera/iphone_intrinsic", JointState, self.intrinsic_callback)

        self.init_camera_pose = None
        self.neck_action_pub = rospy.Publisher("/neck/action", JointState, queue_size=10)


        time.sleep(1)

        # Queue and Event
        self.image_queue = Queue()
        self.toggle_streaming = Event()


        self.exe_queue_head = Queue(20)


        self.vr_start_pose = None
        thread = threading.Thread(target=self.publish_head_pose)
        thread.start()  # Start the thread


        
        

        while not rospy.is_shutdown():


            rgb = self.rgb.copy()
            depth = self.depth.copy()

            intrinsic_mat = self.intrinsic_matrix.copy()
            extrinsic = self.extrinsic_matrix.copy()
            depth, rgb = resize_depth_and_rgb(depth, rgb, self.resolution)



            if self.init_camera_pose is None:

                self.init_camera_pose = extrinsic
                self.tv = OpenTeleVision(self.resolution, self.shm.name, self.image_queue, self.toggle_streaming, self.init_camera_pose, intrinsic_mat, ngrok=False)
            rgb = adjust_exposure(rgb, alpha=0.8, beta=-20)




            np.copyto(self.rgb_array, rgb)
            np.copyto(self.depth_array, depth)
            np.copyto(self.extrinsic_array, extrinsic)


 

    def publish_head_pose(self):
            desired_interval = 0.1

            while True:
                    
                iteration_start = time.time()

                if start_flag == 1:  # Check if the 'a' key is pressed

                    head_matrix = np.array(self.tv.head_matrix).reshape((4, 4))
                    if self.vr_start_pose == None:
                        self.vr_start_pose = mat2pose(head_matrix)
                    else:
                        execute_smooth_head = self._filter_head_pose(vr_start_pose= self.vr_start_pose, pos_speed=1, ori_speed=1, exe_queue_head=self.exe_queue_head, head_mat=head_matrix)
                        execute_smooth_head = execute_smooth_head + neck_start_pose

                        exe_action = pose_to_homogeneous(execute_smooth_head)
                        exe_action_position = exe_action[:3, 3]
                        exe_action_rotation_matrix = exe_action[:3, :3]
                        exe_action_quaternion = R.from_matrix(exe_action_rotation_matrix).as_quat()  # Quaternion is [x, y, z, w]
                        exe_action = np.concatenate([exe_action_position, exe_action_quaternion])



                        timestamp = rospy.Time.now()
                        tracker_msg = JointState()
                        tracker_msg.position = list(map(float,exe_action))
                        tracker_msg.header.stamp = timestamp

                        self.neck_action_pub.publish(tracker_msg)

                        print("exe_action", exe_action)


                        iteration_duration = time.time() - iteration_start
                        sleep_time = desired_interval - iteration_duration
                    
                        if sleep_time > 0:
                            rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility

                        # print("VisionPro FPS", 1/(time.time() - iteration_start))



    def _filter_head_pose(self, vr_start_pose, pos_speed, ori_speed, exe_queue_head, head_mat):


        vr_pose = mat2pose(head_mat)
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
        


        # return execute_smooth
        
        return execute



    def rgb_callback(self, data):
        # rgb_image_np = bridge.imgmsg_to_cv2(data, "bgr8")
        # self.rgb = np.array(rgb_image_np.copy()).reshape((960,720,3))

    
        try:
            # Convert the compressed image message to a cv2 image
            np_arr = np.frombuffer(data.data, np.uint8)
            rgb_image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image_np = cv2.cvtColor(rgb_image_np, cv2.COLOR_BGR2RGB)

            # # Display the image using OpenCV
            # cv2.imshow("Compressed Image", rgb_image_np)
            # cv2.waitKey(1)
        
            
            self.rgb = np.array(rgb_image_np.copy())

            # # Process the RGB image
            # cv2.imshow("RGB Image", rgb_image)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")


    def depth_callback(self, data):

        try:
            depth = bridge.imgmsg_to_cv2(data, "32FC1")
            self.depth = np.array(depth.copy())

        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")


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





if __name__ == "__main__":
    
    import threading
    from pynput import keyboard

    # Global variable to indicate if 'a' key is pressed
    start_flag = False
    end_flag = False

    def monitor_key_presses():
        def on_press(key):
            global start_flag
            global end_flag

            try:
                if key.char == '~':
                    start_flag = True
                    print('Key "~" pressed. start_flag set to True.')
                if key.char == ']':
                    end_flag = True
                    print('Key "]" pressed. end_flag set to True.')
                     
            except AttributeError:
                # Handle special keys if needed
                pass


        # Set up the listener
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    # Create and start the thread
    key_thread = threading.Thread(target=monitor_key_presses)
    key_thread.start()



    rospy.init_node("VisionPro")
    vr = VisionPro()
    
