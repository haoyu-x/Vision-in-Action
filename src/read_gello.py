#!/usr/bin/env python3

import glob
import time
import numpy as np
from gello.agents.gello_agent import GelloAgent
from sensor_msgs.msg import JointState
import rospy

def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)

class ReadGello:
    def __init__(self):

        right_agent = GelloAgent(port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EL3G-if00-port0", 
        start_joints=np.deg2rad([0, 0, 0, 0, 0, 0, 0])) # __Serial_Converter_FT94EMYD # __Serial_Converter_FT94EL3G
        self.right_pub_tracker = rospy.Publisher(name=("/arm/"+"right_gello"), data_class=JointState, queue_size=3)
        
        
        left_agent = GelloAgent(port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U2NG-if00-port0", 
        start_joints=np.deg2rad([0, 0, 0, 0, 0, 0, 0]))        
        self.left_pub_tracker = rospy.Publisher(name=("/arm/"+"left_gello"), data_class=JointState, queue_size=3)
        




        self.matching = False
        start_time = time.time()
        
        desired_interval = 1 / 10.0

        while not rospy.is_shutdown():
            num = time.time() - start_time
            d = time.time()
            # message = f"\rTime passed: {round(num, 2)} Matching Status: {self.matching}         "
            # print_color(
            #     message,
            #     color="white",
            #     attrs=("bold",),
            #     end="",
            #     flush=True,
            # )
            right_gello_action = right_agent.act([])
            tracker_msg = JointState()
            tracker_msg.position = list(map(float,right_gello_action))
            self.right_pub_tracker.publish(tracker_msg)
            # print("right", right_gello_action)

            left_gello_action = left_agent.act([])
            tracker_msg = JointState()
            tracker_msg.position = list(map(float,left_gello_action))
            self.left_pub_tracker.publish(tracker_msg)
            # print("left", np.degrees(left_gello_action))





            iteration_duration = time.time() - d
            sleep_time = desired_interval - iteration_duration
        
            if sleep_time > 0:
                rospy.sleep(sleep_time)  # Use rospy.sleep for ROS compatibility

            print("gello FPS", 1/(time.time() - d))


if __name__ == "__main__": 
    rospy.init_node("ReadGello")
    read_gello = ReadGello()
