#!/usr/bin/env python

import rospy
from your_package.msg import HighCmd
from geometry_msgs.msg import Twist
from your_package.srv import TriggerAction, TriggerActionResponse

class ControlTransform:
    def __init__(self):
        self.control_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=10)
        self.motion_time = 0
        self.service = rospy.Service('/trigger_action', TriggerAction, self.handle_trigger_action)

    def control_tf_callback(self, msg):
        highcmd = HighCmd()
        highcmd.levelFlag = 0
        highcmd.commVersion = 2
        highcmd.robotID = 3
        highcmd.SN = 4
        highcmd.bandWidth = 5
        
        if self.motion_time < 1000:
            highcmd.mode = 1
        else:
            highcmd.mode = 2
        self.motion_time += 1
            
        highcmd.gaitType = 0
        highcmd.speedLevel = 0
        highcmd.dFootRaiseHeight = 0.1
        highcmd.dBodyHeight = 0.0
        highcmd.position = [0.0] * 2
        highcmd.rpy = [0.0, 0.0, msg.angular.z]
        highcmd.velocity = [msg.linear.x, msg.linear.y]
        highcmd.yawSpeed = msg.angular.z
        highcmd.led = [LED() for _ in range(4)]
        highcmd.wirelessRemote = [0] * 40
        highcmd.reserve = 0
        highcmd.crc = 0
        
        print("msg.linear.x, msg.linear.y: ", msg.linear.x, msg.linear.y)
        print("msg.angular.z: ", msg.angular.z)
        self.control_pub.publish(highcmd)

    def handle_trigger_action(self, request):
        # Assuming the service does not require any specific request data
        # You might want to add request processing logic here if needed
        # For now, directly calling control_tf_callback with a dummy message
        dummy_msg = Twist()
        self.control_tf_callback(dummy_msg)
        return TriggerActionResponse("Action triggered successfully")

if __name__ == '__main__':
    rospy.init_node('control_transform_node')
    control_transform = ControlTransform()
    rospy.spin()
