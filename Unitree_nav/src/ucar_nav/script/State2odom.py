#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from unitree_legged_msgs.msg import HighState, IMU  # 替换为你的实际包名
import tf
from geometry_msgs.msg import Quaternion, Vector3



def highstate_callback(msg):
    print('get')
    # 解析HighState消息并发布Odometry消息
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"

    # 假设position是一个长度为3的float32数组
    odom.pose.pose.position.x = msg.position[0]
    odom.pose.pose.position.y = msg.position[1]
    odom.pose.pose.position.z = msg.position[2]

    # 假设yawSpeed表示以弧度为单位的偏航角速率
    # 将偏航角速率转换为四元数
    quat = tf.transformations.quaternion_from_euler(0, 0, msg.yawSpeed)
    odom.pose.pose.orientation = Quaternion(*quat)

    # 假设velocity是一个长度为3的float32数组
    odom.twist.twist.linear.x = msg.velocity[0]
    odom.twist.twist.linear.y = msg.velocity[1]
    odom.twist.twist.linear.z = msg.velocity[2]
    odom.twist.twist.angular.z = msg.yawSpeed  # 假设yawSpeed是绕z轴的角速度

    odom_pub.publish(odom)

    # 解析HighState消息并发布IMU消息
    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = "imu"

    imu.orientation = Quaternion(
        msg.imu.quaternion[0],
        msg.imu.quaternion[1],
        msg.imu.quaternion[2],
        msg.imu.quaternion[3]
    )
    imu.angular_velocity = Vector3(
        msg.imu.gyroscope[0],
        msg.imu.gyroscope[1],
        msg.imu.gyroscope[2]
    )
    imu.linear_acceleration = Vector3(
        msg.imu.accelerometer[0],
        msg.imu.accelerometer[1],
        msg.imu.accelerometer[2]
    )

    imu_pub.publish(imu)


if __name__ == '__main__':
    print('ok')
    # import pdb; pdb.set_trace()
    rospy.init_node('highstate_to_odom_imu')
    print('ok')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.Subscriber('/high_state', HighState, highstate_callback)
    rospy.spin()
