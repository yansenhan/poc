import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Twist, Point, PointStamped, Pose

def publish_markers():
    rospy.init_node('custom_marker_publisher')
    marker_pub = rospy.Publisher('/test_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 HZ
    
    while not rospy.is_shutdown(): 
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "world_coordinate"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
                
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.a = 1  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        marker.pose.position.x = 2 # best_est[0]
        marker.pose.position.y = 2 # best_est[1]
        marker.pose.position.z = 0
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker_pub.publish(marker)
        print("Marker")
        rate. sleep()
        
if __name__ == "__main__":
    publish_markers()
                
