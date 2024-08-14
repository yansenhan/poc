#!/usr/bin/env python
import cv2
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, Int8, Int8MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist, Point, PointStamped, Pose
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import linalg as LA
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ucar_nav.msg import Boxinfo
import time
from visualization_msgs.msg import Marker

cam_wid = 848  # 摄像头width
cam_hig = 480  # 摄像头height
f = 700 # 摄像头焦距
LaserLen = 360 # 激光雷达数据长度
qs = 100         # 传感器队列长

Origin = np.array([0, 0])

cam_tf_las = np.array([[1, 0, -0.2],
                      [0, 1, 0],
                      [0, 0, 1]])   # 相机坐标系与激光雷达坐标系之间的变换矩阵

las_tf_bas = np.array([[1, 0, -0.15],
                      [0, 1, 0],
                      [0, 0, 1]])   # 激光雷达坐标系与车体坐标系之间的变换矩阵


def dist(p1, p2):
    # 计算两个点的欧氏距离
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def In_mapbox(p1, box):
    # 计算一个位姿是否在特定的地图框中
    if box[0]<p1[0]<box[1] and box[2]<p1[1]<box[3]:
        if abs(p1[2]-box[4])<0.4 or abs(p1[2]-box[4]+2*np.pi)<0.4 or abs(p1[2]-box[4]-2*np.pi)<0.4:
            return True
    else:
        return False

class Scan_cam_test:
    def __init__(self):
        self.global_map = None
        self.local_map = None
        self.scan_data = None
        self.buffer_size = 10  # Adjust buffer size as needed
        self.ScanFIFO = qs * [None, ]
        self.PoseFIFO = qs * [None, ]
        self.road_len = 0
        self.best_est = None
        self.reach_flag = False
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # 目标发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.ggoal = MoveBaseGoal()

        # 火源位置计算服务终端
        # self.Boxtf_srv = rospy.Service('/box_place', Boxinfo, self.callback_box)
        self.Boxtf_srv = rospy.Subscriber('/box_place', Boxinfo, self.callback_box)

        # 激光雷达数据监听器
        self.Laser_sub = rospy.Subscriber('/slamware_ros_sdk_server_node/scan', LaserScan, self.callback_scan, queue_size=2)
        
        
        # 位姿计算及状态更新器
        # self.current_pose = rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, self.odometry_callback, queue_size=2)
        self.current_pose = rospy.Subscriber('/posemsg', Float32MultiArray, self.odometry_callback)
        # self.current_pose = rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, self.odometry_callback)

        self.marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)

    def callback_scan(self, data):
        scandata = data.ranges   # 激光雷达数据，是一个元组
        scandata1 = scandata[180:]   # 调整激光雷达数据的顺序
        scandata2 = scandata[:180]
        scandata = scandata1 + scandata2  # 重新拼接列表 # validate the data
        scandata = np.array(scandata)
        scandata[np.isinf(scandata)] = 0.7  # 改造值为inf的数据
        scandata[np.where(scandata<0.01)[0]] = 0.7  # 改造值为0的数据

        self.ScanFIFO[1:qs] = self.ScanFIFO[0:qs-1]           # 队列更新
        self.ScanFIFO[0] = (scandata, data.header.stamp)  # 监听激光雷达扫描的数据


    def odometry_callback(self, msg):
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        # orien = msg.pose.pose.orientation
        # olist = [orien.x, orien.y, orien.z, orien.w]
        # (_, _, yaw) = euler_from_quaternion(olist)

        # 接收地图消息
        self.robot_x_pose = msg.data[0] # msg.pose.pose.position.x
        self.robot_y_pose = msg.data[1] #msg.pose.pose.position.y
        yaw = msg.data[2]
        self.Pose = [self.robot_x_pose, self.robot_y_pose, yaw]
        self.PoseFIFO[1:qs] = self.PoseFIFO[0:qs-1]
        self.PoseFIFO[0] = (self.Pose, rospy.Time.now())
        # self.PoseFIFO[0] = (self.Pose, rospy.Time.from_sec(msg.header.stamp.secs))
        
        dist_threshold = 1
        if self.best_est is not None:
            print("self.Pose: ", self.Pose)
            print("self.best_est: ", self.best_est)
            print("dist(self.best_est, self.Pose): ", dist(self.best_est, self.Pose))

            if dist(self.best_est, self.Pose) <= dist_threshold:
                print("+------- Reaching Goal! -------+")
                self.reach_flag = True
            else:
                print("send")
                # self.send_goals([self.best_est[0], self.best_est[1], yaw])
                # publish the marker
            self.marker_publish(self.best_est, self.marker_pub)

    def marker_publish(self, position, publisher):
        # 将预估点可视化为一个绿色的球体
        rate = rospy.Rate(1)  # 1 HZ
        # while not rospy.is_shutdown(): 
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "world_coordinate"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.a = 1  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # marker.lifetime = rospy.Duration(60.0)
        # while not rospy.is_shutdown(): 
        publisher.publish(marker)
        rate. sleep()

    def callback_box(self, req):
        # 相机模型计算火源位置
        # Boxs = req.Boxs
        x = req.box_x
        y = req.box_y
        w = req.box_w
        h = req.box_h
        box = [int(x - w/2), int(x + w/2)]  # 人像框对应的左右像素索引

        theta1 = np.arctan((cam_wid / 2 - box[0]) / f)   # box左边界点在camera坐标系下的极角
        theta2 = np.arctan((cam_wid / 2 - box[1]) / f)   # box右边界点在camera坐标系下的极角
        # print("theta1: {}, theta2: {}".format(str(theta1), str(theta2)))

        min_err = 100      # 误差计数值e
        best_est = None    # 最优坐标估计

        # Cp_idx, Sc_idx = 0, 0     # 时间对齐处对应的位姿\雷达数据队列中的索引
        # Cp_min, Sc_min = 100, 100 # 最小对齐时间
        Cp_idx, Sc_idx = 0, 0     # 时间对齐处对应的位姿\雷达数据队列中的索引
        Cp_min, Sc_min = 1, 0.1 # 最小对齐时间
        for k in range(qs):       # 选择最为恰当的时间点，即摄像头，激光雷达，位姿三者的对齐时刻
            # print("self.PoseFIFO[k][1]: ", self.PoseFIFO[k][1])= dist_threshold
            if self.PoseFIFO[k] is None: continue
            if Cp_min > abs((self.PoseFIFO[k][1] - req.header.stamp).to_sec()):
                Cp_min = abs((self.PoseFIFO[k][1] - req.header.stamp).to_sec())
                Cp_idx = k
            if Sc_min > abs((self.ScanFIFO[k][1] - req.header.stamp).to_sec()):
                Sc_min = abs((self.ScanFIFO[k][1] - req.header.stamp).to_sec())
                Sc_idx = k
        # print("Cp_idx: {}, Sc_idx: {}".format(Cp_idx, Sc_idx))
        # print("cp time diff: {}, sc time diff: {}".format((self.PoseFIFO[Cp_idx][1] - req.header.stamp).to_sec(), (self.ScanFIFO[Sc_idx][1] - req.header.stamp).to_sec()))
        scandata = self.ScanFIFO[Sc_idx][0]  # 提取出对齐时间的激光雷达数据
        cpose = self.PoseFIFO[Cp_idx][0]     # 提取出对齐时间的位姿数据
        
        ind1 = int(theta1 * LaserLen / (2*np.pi))  # box左边界点在激光雷达数据中的索引
        ind2 = int(theta2 * LaserLen / (2*np.pi))  # box右边界点在激光雷达数据中的索引
        # print("ind1: {}, ind2: {}".format(ind1, ind2))
        # print("scan_data: ", scandata[ind2:ind1])
        if ind1 >= 0 and ind2 < 0:
            laser_data = np.concatenate([scandata[ind2:], scandata[:ind1]])
        else:
            laser_data = scandata[ind2:ind1]
        
        # delete the infinity point
        real_laser_data = []
        for l_data in laser_data:
            if l_data != 0.7:
                real_laser_data.append(l_data)
        
        # the distance to fire
        dis_laser = np.median(real_laser_data)

        # 不考虑4m以上距离的火源位置
        # fire_distance = 4
        # if dis_laser > fire_distance: 
        #     print("[WARNING] Not consider the fire beyond {} meters".format(fire_distance))
        #     print("[WARNING] The fire is {} meters far from the dog".format(dis_laser))
        #     return 

        # 计算火源的最优估计点 (雷达坐标系)
        p1 = dis_laser * np.array([np.cos(theta1), - np.sin(theta1), 1./dis_laser])
        p2 = dis_laser * np.array([np.cos(theta2), - np.sin(theta2), 1./dis_laser])
        best_est = (p1 + p2) / 2
        # print("best_est 1: ", best_est)
        
        # 换算到全局地图
        best_est = np.matmul(las_tf_bas, best_est)   # 换算到车体坐标系下
        bas_tf_map = np.array([[np.cos(cpose[2]), -np.sin(cpose[2]), cpose[0]],
                               [np.sin(cpose[2]),  np.cos(cpose[2]), cpose[1]],
                               [0, 0, 1]])
        best_est = np.matmul(bas_tf_map, best_est)   # 换算到全局地图坐标系下
        # print('best_est 2: ', best_est)
        if self.reach_flag is True or self.best_est is None:
            print(" ######## Goal changed ######## ")
            self.best_est = best_est
            self.reach_flag = False
        
        self.best_est = best_est ### need to delete
    
    def send_goals(self, pos):
        # 发布move_base的目标位姿的程序
        header = Header(seq=0, stamp=rospy.Time.now(), frame_id="map")
        pose_ = Pose()
        pose_.position.x = pos[0]
        pose_.position.y = pos[1]
        pose_.position.z = 0
        quaternion = quaternion_from_euler(0, 0, pos[2])
        pose_.orientation.x = quaternion[0]
        pose_.orientation.y = quaternion[1]
        pose_.orientation.z = quaternion[2]
        pose_.orientation.w = quaternion[3]
        Potst_msg = PoseStamped(header=header, pose=pose_)
        self.goal_pub.publish(Potst_msg)
        print('OK')

def main():
    rospy.init_node('Scan_cam_test')
    Scan_cam_test()
    rospy.spin()


if __name__ == '__main__':
    main()
