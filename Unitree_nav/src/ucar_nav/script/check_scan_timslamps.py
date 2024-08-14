#!/usr/bin/env python
import cv2

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

prev_time = None

def scan_callback(msg):
    global prev_time
    current_time = msg.header.stamp

    if prev_time is not None:
        time_diff=(current_time-prev_time).to_sec()
        rospy.loginfo("Time difference:%f seconds", time_diff)
    prev_time=current_time


def main():
    rospy.init_node("check_scan_timestamps")
    # rospy.Subscriber("/slamware_ros_sdk_server_node/odom", Odometry, scan_callback)
    rospy.Subscriber("/slamware_ros_sdk_server_node/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ =="__main__":
    main()
       