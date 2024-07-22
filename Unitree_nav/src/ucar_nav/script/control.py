import rospy
import time
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import HighCmd, HighState
from unitree_legged_msgs.msg import LED
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Int8, Int8MultiArray, Float32MultiArray, String

import sys
import time

sys.path.append('/home/lijixiang/test-unitree/test-unitree/unitree_legged_sdk-3.8.4/lib/python/amd64')
import robot_interface as sdk


class Control_transform:
    def __init__(self) -> None:      
        self.control_pub = rospy.Publisher('/high_cmd',
                                           HighCmd,
                                           queue_size=2)

        self.motion_time = 0
        
        self.cmd_vel_listener = rospy.Subscriber('/cmd_vel',
                                                 Twist,
                                                 self.control_tf_callback)

        
        # 激光雷达数据监听器
        self.Laser_sub = rospy.Subscriber('/slamware_ros_sdk_server_node/scan', LaserScan, 
                                          self.callback_scan, queue_size=1)
        
        self.my_timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        
        self.La_flag = 1
    

    def callback_scan(self, data):
        self.La_flag = 1

    
    def timer_callback(self):
        self.La_flag -= 0.1
        
    def control_tf_callback(self, msg):
        
        highcmd = HighCmd()
        highcmd.levelFlag = 0
        highcmd.commVersion = 2
        highcmd.robotID = 3
        highcmd.SN = 4
        highcmd.bandWidth = 5
        highcmd.mode = 2
        self.motion_time += 1
            
        highcmd.gaitType = 0
        highcmd.speedLevel = 0
        highcmd.dFootRaiseHeight = 0.1
        highcmd.dBodyHeight = 0.0
        highcmd.position = [0.0] * 2
        highcmd.rpy = [0.0, 0.0, 0.0]
        
        # try:
        #     lidar_scan = rospy.wait_for_message("/slamware_ros_sdk_server_node/scan", LaserScan, timeout=1)
        # except:
        #     lidar_scan = None
        print("self.La_flag: ", self.La_flag)
        if self.La_flag > 0.5:
            highcmd.velocity = [msg.linear.x, msg.linear.y]
            highcmd.yawSpeed = msg.angular.z
            print("msg.linear.x, msg.linear.y: ", msg.linear.x, msg.linear.y)
            print("msg.angular.z: ", msg.angular.z)
        else:
            highcmd.velocity = [0, 0]
            highcmd.yawSpeed = 0
            print("lidar time out, time: ", self.motion_time)
           
        highcmd.led = [LED() for _ in range(4)]
        highcmd.wirelessRemote = [0] * 40
        highcmd.reserve = 0
        highcmd.crc = 0
        
        self.control_pub.publish(highcmd)

if __name__ == "__main__":
    # pre_action()
    
    rospy.init_node("control_tf")
    control_tf = Control_transform()
    rospy.spin()