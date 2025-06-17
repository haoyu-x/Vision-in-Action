#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage, JointState

from std_msgs.msg import Float64MultiArray, UInt8MultiArray
import cv2
from cv_bridge import CvBridge
#from record3d import Record3DStream
from PIL import Image as PILImage
from quaternion import as_rotation_matrix, quaternion
#import open3d as o3d
from multiprocessing import Event
import time
import cv2
import numpy as np
import time

# Define the paths to your cameras
top_paths = {
    'top': "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_92BFE4FE-video-index0",

}

# Define the paths to your cameras
right_paths = {
    'right': "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_766C49AE-video-index0",
}

left_paths = {
    'left': "/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_608EF4FE-video-index0",
}

CAMERA_FOCUS = 0
CAMERA_TEMPERATURE = 3900
CAMERA_EXPOSURE = 156
CAMERA_GAIN = 10



class ReadChestCam:
    def __init__(self):

        
        self.publisher = {
            # 'right': rospy.Publisher('/camera/right_RGB', CompressedImage, queue_size=10),
            # 'left': rospy.Publisher('/camera/left_RGB', CompressedImage, queue_size=10),
            'top': rospy.Publisher('/camera/top_RGB', CompressedImage, queue_size=10),
        }
 
 


        # Initialize video capture object with the path
        # Open video capture objects for each camera
        self.caps = {}


        for key, path in top_paths.items():
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                print(f"Error: Could not open camera {key}")
            else:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                cap.set(cv2.CAP_PROP_FPS, 60)
                cap.set(cv2.CAP_PROP_ZOOM, 100)
                # Disable all auto
                cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # White balance
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = off, 3 = on
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Gives much better latency


                # Read several frames to let settings (especially gain/exposure) stabilize
                for _ in range(30):
                    cap.read()
                    cap.set(cv2.CAP_PROP_FOCUS, CAMERA_FOCUS)  # Fixed focus
                    cap.set(cv2.CAP_PROP_TEMPERATURE, CAMERA_TEMPERATURE)  # Fixed white balance
                    cap.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)  # Fixed exposure
                    cap.set(cv2.CAP_PROP_GAIN, CAMERA_GAIN)  # Fixed gain

                # Check all settings match expected
                assert cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 1280
                assert cap.get(cv2.CAP_PROP_FRAME_HEIGHT) == 720
                assert cap.get(cv2.CAP_PROP_BUFFERSIZE) == 1
                assert cap.get(cv2.CAP_PROP_AUTOFOCUS) == 0
                assert cap.get(cv2.CAP_PROP_AUTO_WB) == 0
                assert cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 1
                assert cap.get(cv2.CAP_PROP_FOCUS) == CAMERA_FOCUS
                assert cap.get(cv2.CAP_PROP_TEMPERATURE) == CAMERA_TEMPERATURE
                assert cap.get(cv2.CAP_PROP_EXPOSURE) == CAMERA_EXPOSURE
                assert cap.get(cv2.CAP_PROP_GAIN) == CAMERA_GAIN

                self.caps[key] = cap


        # for key, path in right_paths.items():
        #     cap = cv2.VideoCapture(path)
        #     if not cap.isOpened():
        #         print(f"Error: Could not open camera {key}")
        #     else:
        #         cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        #         cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #         cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        #         cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Gives much better latency

        #         cap.set(cv2.CAP_PROP_FPS, 30)
            
        #         # Disable all auto
        #         cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        #         cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # White balance
        #         cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = off, 3 = on


        #         # Read several frames to let settings (especially gain/exposure) stabilize
        #         for _ in range(30):
        #             cap.read()
        #             cap.set(cv2.CAP_PROP_FOCUS, CAMERA_FOCUS)  # Fixed focus
        #             cap.set(cv2.CAP_PROP_TEMPERATURE, CAMERA_TEMPERATURE)  # Fixed white balance
        #             cap.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)  # Fixed exposure
        #             cap.set(cv2.CAP_PROP_GAIN, CAMERA_GAIN)  # Fixed gain


        #         # Check all settings match expected
        #         assert cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 1280
        #         assert cap.get(cv2.CAP_PROP_FRAME_HEIGHT) == 720
        #         assert cap.get(cv2.CAP_PROP_BUFFERSIZE) == 1
        #         assert cap.get(cv2.CAP_PROP_AUTOFOCUS) == 0
        #         assert cap.get(cv2.CAP_PROP_AUTO_WB) == 0
        #         assert cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 1
        #         assert cap.get(cv2.CAP_PROP_FOCUS) == CAMERA_FOCUS
        #         assert cap.get(cv2.CAP_PROP_TEMPERATURE) == CAMERA_TEMPERATURE
        #         assert cap.get(cv2.CAP_PROP_EXPOSURE) == CAMERA_EXPOSURE
        #         assert cap.get(cv2.CAP_PROP_GAIN) == CAMERA_GAIN



        #         self.caps[key] = cap



        # for key, path in left_paths.items():
        #     cap = cv2.VideoCapture(path)
        #     if not cap.isOpened():
        #         print(f"Error: Could not open camera {key}")
        #     else:
        #         cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        #         cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #         cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        #         cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Gives much better latency

        #         cap.set(cv2.CAP_PROP_FPS, 30)
            
        #         # Disable all auto
        #         cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        #         cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # White balance
        #         cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = off, 3 = on


        #         # Read several frames to let settings (especially gain/exposure) stabilize
        #         for _ in range(30):
        #             cap.read()
        #             cap.set(cv2.CAP_PROP_FOCUS, CAMERA_FOCUS)  # Fixed focus
        #             cap.set(cv2.CAP_PROP_TEMPERATURE, CAMERA_TEMPERATURE)  # Fixed white balance
        #             cap.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)  # Fixed exposure
        #             cap.set(cv2.CAP_PROP_GAIN, CAMERA_GAIN)  # Fixed gain


        #         # Check all settings match expected
        #         assert cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 1280
        #         assert cap.get(cv2.CAP_PROP_FRAME_HEIGHT) == 720
        #         assert cap.get(cv2.CAP_PROP_BUFFERSIZE) == 1
        #         assert cap.get(cv2.CAP_PROP_AUTOFOCUS) == 0
        #         assert cap.get(cv2.CAP_PROP_AUTO_WB) == 0
        #         assert cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) == 1
        #         assert cap.get(cv2.CAP_PROP_FOCUS) == CAMERA_FOCUS
        #         assert cap.get(cv2.CAP_PROP_TEMPERATURE) == CAMERA_TEMPERATURE
        #         assert cap.get(cv2.CAP_PROP_EXPOSURE) == CAMERA_EXPOSURE
        #         assert cap.get(cv2.CAP_PROP_GAIN) == CAMERA_GAIN


        #         self.caps[key] = cap


        # Initialize CvBridge
        bridge = CvBridge()


        desired_interval = 1 / 10.0

        # Create a window for each camera
        window_names = list(self.caps.keys())
        for window_name in window_names:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
        while True:
            iteration_start = time.time()

            # Capture frames from each camera
            frames = {}
            for key, cap in self.caps.items():
                # d = time.time()
                ret, frame = cap.read()# Flush buffered frame
                ret, frame = cap.read()

                # print(key, time.time()-d  )
                if ret:
                    frames[key] = frame
                else:
                    print(f"Error: Could not read frame from camera {key}")






            # Display the frames
            for key, frame in frames.items():
                cv2.imshow(key, frame)

            # Check for a quit signal
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break




 
            for key, frame in frames.items():

                _, compressed_rgb = cv2.imencode('.jpg', np.array(frame).astype(np.uint8))
                rgb_msg = CompressedImage()
                rgb_msg.header.stamp = rospy.Time.now()
                rgb_msg.format = "jpeg"
                rgb_msg.data = np.array(compressed_rgb).tobytes()

                self.publisher[key].publish(rgb_msg)


            iteration_duration = time.time() - iteration_start
            sleep_time = desired_interval - iteration_duration
        
            if sleep_time > 0:
                rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility

            print("camera streaming FPS", 1/(time.time() - iteration_start))




if __name__ == "__main__":
    rospy.init_node("ReadChestCam")
    readchestcam = ReadChestCam()
