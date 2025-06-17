#!/usr/bin/env python3

import rospy
import pyzed.sl as sl
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, JointState
from cv_bridge import CvBridge

class ZEDMiniStreamer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('zed_mini_streamer', anonymous=True)

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE  # RGB only

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr(f"ZED open failed: {repr(err)}")
            exit(1)

        self.runtime_params = sl.RuntimeParameters()
        self.left_image = sl.Mat()
        self.right_image = sl.Mat()

        # ROS Publishers
        self.pub_left = rospy.Publisher("/camera/zed_left", CompressedImage, queue_size=10)
        self.pub_right = rospy.Publisher("/camera/zed_right", CompressedImage, queue_size=10)


        self.bridge = CvBridge()

        self.main_loop()

    def main_loop(self):
        rate = rospy.Rate(30)

        camera_info = self.zed.get_camera_information()
        # intrinsics = camera_info.calibration_parameters.left_cam

        while not rospy.is_shutdown():
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
                self.zed.retrieve_image(self.right_image, sl.VIEW.RIGHT)

                left_np = self.left_image.get_data()
                right_np = self.right_image.get_data()
                
                # Encode to JPEG
                _, left_jpg = cv2.imencode('.jpg', left_np)
                _, right_jpg = cv2.imencode('.jpg', right_np)

                # Timestamp
                stamp = rospy.Time.now()

                # Publish compressed RGB
                left_msg = CompressedImage()
                left_msg.header.stamp = stamp
                left_msg.format = "jpeg"
                left_msg.data = left_jpg.tobytes()
                self.pub_left.publish(left_msg)

                right_msg = CompressedImage()
                right_msg.header.stamp = stamp
                right_msg.format = "jpeg"
                right_msg.data = right_jpg.tobytes()
                self.pub_right.publish(right_msg)

    



                # Optional visualization
                vis = np.hstack((cv2.resize(left_np, (256,144), interpolation=cv2.INTER_AREA),    cv2.resize(right_np, (256,144), interpolation=cv2.INTER_AREA),))
                cv2.imshow("ZED Mini RGB", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            rate.sleep()

        self.zed.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        ZEDMiniStreamer()
    except rospy.ROSInterruptException:
        pass
