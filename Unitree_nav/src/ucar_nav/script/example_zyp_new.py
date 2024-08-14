#!/usr/bin/env python
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
from ucar_nav.srv import Boxinfo, BoxinfoResponse
import random 
from unitree_legged_msgs.msg import HighCmd, HighState
import time
from scipy.ndimage import binary_erosion
from PIL import Image, ImageDraw
import math
import argparse
import os, time, cv2
import yaml
import numpy as np
import sys
import shutil
import pickle
import gym
import torch as th
import torch.nn as nn
import matplotlib.pyplot as plt
from PIL import Image

cam_wid = 848  # 摄像头width
cam_hig = 480  # 摄像头height
f = 700 # 摄像头焦距
LaserLen = 360 # 激光雷达数据长度
qs = 8         # 传感器队列长

Origin = np.array([0, 0])

cam_tf_las = np.array([[1, 0, -0.2],
                      [0, 1, 0],
                      [0, 0, 1]])   # 相机坐标系与激光雷达坐标系之间的变换矩阵

las_tf_bas = np.array([[1, 0, -0.15],
                      [0, 1, 0],
                      [0, 0, 1]])   # 激光雷达坐标系与车体坐标系之间的变换矩阵


def dist(p1, p2): # 计算两个点的欧氏距离
   
    try:
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    except:
        return 0.5

def In_mapbox(p1, box):
    # 计算一个位姿是否在特定的地图框中
    if box[0]<p1[0]<box[1] and box[2]<p1[1]<box[3]:
        if abs(p1[2]-box[4])<0.4 or abs(p1[2]-box[4]+2*np.pi)<0.4 or abs(p1[2]-box[4]-2*np.pi)<0.4:
            return True
    else:
        return False


class MapAndScanBuffer:
    def __init__(self):
        self.global_map = None
        self.local_map = None
        self.scan_data = None
        self.buffer_size = 10  # Adjust buffer size as needed
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # 目标发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.ggoal = MoveBaseGoal()
        self.Pose = [0, 0, 0]  # 小车姿态[x, y, yaw]
        self.cmdvel = Twist()  # 控制器控制序列记录
        self.start_time  = 0   # 起始时间
        self.road_len = 0      # 小车从起点出发行驶的里程
        self.ScanFIFO = qs * [None, ]        
        self.map_array = None       
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0
        self.map_info = None
        self.previous_goal = None

        # 订阅全局代价地图
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_map_callback) 

        # 位姿计算及状态更新器
        self.current_pose = rospy.Subscriber('/posemsg', Float32MultiArray, self.odometry_callback)
        
        # 火源位置计算服务终端
        self.Boxtf_srv = rospy.Service('/box_place', Boxinfo, self.callback_box)

        # 激光雷达数据监听器
        self.Laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan, queue_size=2)

    def cmd_vel_fun(self, msg):
        if self.cmd_vel == msg:
            self.cmd_vel = None
        else:
            self.cmd_vel = msg
            
    def control_fun(self, msg):
        if self.high_cmd == msg:
            self.high_cmd = None
        else:
            self.high_cmd = msg

    def distance(self,point1, point2):
        """ 计算两点之间的欧氏距离 """
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def find_nearest_point(self,robot_position, points):
        """ 找到机器人位置最近的点 """
        min_distance = float('inf')
        nearest_point = None

        for point in points:
            dist = self.distance(robot_position, point)
            if dist < min_distance:
                min_distance = dist
                nearest_point = point

        return nearest_point

    def filter_points(self, points, current_point, max_distance=20):
        """ 从点列表中过滤掉特定点和距离当前点小于等于max_distance的点 """
        return [point for point in points if point != current_point and self.distance(point, current_point) > max_distance]

    def global_map_callback(self, msg):
        self.global_map = msg


  


    def callback_scan(self, data):
        scandata = data.ranges   # 激光雷达数据，是一个元组
        scandata1 = scandata[387:-1]   # 调整激光雷达数据的顺序
        scandata2 = scandata[0:387]
        scandata = scandata1 + scandata2  # 重新拼接列表
        scandata = np.array(scandata)
        scandata[np.isinf(scandata)] = 0.7  # 改造值为inf的数据
        scandata[np.where(scandata<0.01)[0]] = 0.7  # 改造值为0的数据

        self.ScanFIFO[1:qs] = self.ScanFIFO[0:qs-1]           # 队列更新
        self.ScanFIFO[0] = (scandata, data.header.stamp)  # 监听激光雷达扫描的数据
############################################################################################################################################
    def near_obstacle(self, x, y, map_data, threshold, downsample_factor):#边界周围三个单元格是否有障碍物
        height, width = map_data.shape
        downsampled_x = x // downsample_factor #下采样，更粗力度
        downsampled_y = y // downsample_factor    
        start_x = max(0, downsampled_x - threshold)#创建一个边框，不超出地图范围
        end_x = min(width // downsample_factor, downsampled_x + threshold + 1)
        start_y = max(0, downsampled_y - threshold)
        end_y = min(height // downsample_factor, downsampled_y + threshold + 1)      
        neighborhood = map_data[start_y:end_y, start_x:end_x]       
        return np.any(neighborhood == 100)#100表示障碍物


    def detect_frontiers(self, map_info):       #所有边界点
        frontiers = []
        for y in range(map_info.height):
            for x in range(map_info.width):
                if self.map_array[y, x] == -1:  # unknown cell  #机器人还未探索区域
                    neighbors = self.map_array[y - 1:y + 2, x - 1:x + 2]
                    if 0 in neighbors:  # free cell##可走区域
                        if self.is_valid_frontier(x, y, map_info):##没有超过图像边界并且周围有障碍物
                            frontiers.append((x, y))
        return frontiers

    def is_valid_frontier(self, x, y, map_info):
        if not ( 0 <= x < map_info.width and 0 <= y < map_info.height):#超过图像边界
            return False
        if self.near_obstacle(x, y, self.downsampled_map, threshold=5, downsample_factor=2):#周围有障碍物
            return False        
        return True 

    def select_goal(self, frontiers, map_data):#边界和地图
        valid_frontiers = []
        for x, y in frontiers:#检查边界周围是否有障碍物
            if not self.near_obstacle(x, y, map_data, threshold=5,downsample_factor=2):
                valid_frontiers.append((x, y))
        if valid_frontiers:#随机选择一个点为导航点
            goal_x, goal_y = random.choice(valid_frontiers)
            real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x#栅格地图到真实地图转换
            real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y
            if (real_x, real_y) == self.previous_goal or (self.robot_x_pose == real_x and self.robot_y_pose == real_y):#新的目标点是否与上一个目标点相同
                print("和上一个目标点相同，重新选择")
                valid_frontiers.remove((goal_x, goal_y))
                if valid_frontiers: #可以遍历完所有边界，意味着可以遍历完整个地图
                    mid_index = len(valid_frontiers) // 2
                    goal_x, goal_y = valid_frontiers[mid_index]
                    real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x
                    real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y
                else:
                    print("所有边界点全都探索完.")
                    return None, None
            self.previous_goal = (real_x, real_y)
            return real_x, real_y
        else:
            return None, None



    def odometry_callback(self, msg):        
         #接收地图消息
        self.robot_x_pose=msg.data[0]
        self.robot_y_pose=msg.data[1]      
        self.map_array = np.array(self.global_map.data).reshape((self.global_map.info.height, self.global_map.info.width))
        self.map_info = self.global_map.info           
        downsample_factor = 2  
        downsampled_height = self.global_map.info.height // downsample_factor
        downsampled_width = self.global_map.info.width // downsample_factor
        self.downsampled_map = np.zeros((downsampled_height, downsampled_width))     
        for y in range(downsampled_height):
            for x in range(downsampled_width):
                start_y = y * downsample_factor
                end_y = (y + 1) * downsample_factor
                start_x = x * downsample_factor
                end_x = (x + 1) * downsample_factor             
                self.downsampled_map[y, x] = np.mean(self.map_array[start_y:end_y, start_x:end_x])       
        struct_elem = np.ones((3, 3), dtype=int)  # Define a 3x3 structuring element 
        self.downsampled_map = binary_erosion(self.downsampled_map, structure=struct_elem).astype(int)   #地图  
       
        frontiers = self.detect_frontiers(map_info=self.map_info)
        goal = self.select_goal(frontiers, self.map_array)
        print("Selected Goal:", goal)
        self.send_goals([goal[0], goal[1], 0])
       
       
########################################################################################################################################

    def callback_box(self, req):
        # 相机模型计算火源位置
        # Boxs = req.Boxs
        x = req.box_x
        y = req.box_y
        w = req.box_w
        h = req.box_h
        box = [int(x - w/2), int(x + w/2)]  # 人像框对应的左右像素索引
        print("box: ", box)

        theta1 = np.arctan((cam_wid / 2 - box[0]) / f)   # box左边界点在camera坐标系下的极角
        theta2 = np.arctan((cam_wid / 2 - box[1]) / f)   # box右边界点在camera坐标系下的极角
        print("theta1: {}, theta2: {}".format(str(theta1), str(theta2)))

        min_err = 100      # 误差计数值
        best_est = None    # 最优坐标估计

        Cp_idx, Sc_idx = 0, 0   # 时间对齐处对应的位姿\雷达数据队列中的索引
        Cp_min, Sc_min = 100, 100 # 最小对齐时间
        for k in range(qs):   # 选择最为恰当的时间点，即摄像头，激光雷达，位姿三者的对齐时刻
            print("self.PoseFIFO[k][1]: ", self.PoseFIFO[k][1])
            if Cp_min > abs((self.PoseFIFO[k][1] - req.header.stamp).to_sec()):
                Cp_min = abs((self.PoseFIFO[k][1] - req.header.stamp).to_sec())
                Cp_idx = k
            if Sc_min > abs((self.ScanFIFO[k][1] - req.header.stamp).to_sec()):
                Sc_min = abs((self.ScanFIFO[k][1] - req.header.stamp).to_sec())
                Sc_idx = k

        scandata = self.ScanFIFO[Sc_idx][0]  # 提取出对齐时间的激光雷达数据
        cpose = self.PoseFIFO[Cp_idx][0]     # 提取出对齐时间的位姿数据
        
        ind1 = int(theta1 * LaserLen / (2*np.pi))  # box左边界点在激光雷达数据中的索引
        ind2 = int(theta2 * LaserLen / (2*np.pi))  # box右边界点在激光雷达数据中的索引
        print("ind1: {}, ind2: {}".format(ind1, ind2))
        print("scan_data: ", scandata)
        if ind1 >= 0 and ind2 < 0:
            dis_laser = np.median(np.concatenate([scandata[ind2:], scandata[:ind1]]))
        else:
            dis_laser = np.median(scandata[ind2:ind1])  # 激光雷达得到的到Box距离的平均值
            
        print("dis_laser: ", dis_laser)
        p1 = dis_laser * np.array([np.cos(theta1), - np.sin(theta1), 1./dis_laser])
        p2 = dis_laser * np.array([np.cos(theta2), - np.sin(theta2), 1./dis_laser])
        best_est = (p1 + p2) / 2
        print("best_est 1: ", best_est)
        
        best_est = np.matmul(las_tf_bas, best_est)   # 换算到车体坐标系下
        bas_tf_map = np.array([[np.cos(cpose[2]), -np.sin(cpose[2]), cpose[0]],
                                [np.sin(cpose[2]), np.cos(cpose[2]), cpose[1]],
                                [0, 0, 1]])
        best_est = np.matmul(bas_tf_map, best_est)   # 换算到全局地图坐标系下
        print('best_est 2: ', best_est)
        if -0.125 < best_est[0] < 3.875 and -3.875 < best_est[1] < 1.5:
            print('*******************************')
            print('best_est:', best_est)
            print('theta:', (theta1 + theta2) / 2)
            print('where:', [cpose[0], cpose[1], 180*cpose[2]/np.pi])
            print()

            x = int(21 + 50*best_est[0])
            y = int(21 - 50*best_est[1])
            sx = int(21 + 50*cpose[0])
            sy = int(21 - 50*cpose[1])
            ex = int(sx + 6*np.cos(cpose[2]))
            ey = int(sy - 6*np.sin(cpose[2]))

            # # 在img原始图片中划圈，其圈的中心点为（520，430），半径=300，颜色为（255，0，0），粗细=1
            # cv2.circle(self.img, (x,y), 4, (255,0,0), 1)
            # # 在img原始图片中画箭头，箭头表示车采集到图像时候的位姿
            # cv2.arrowedLine(self.img, (sx,sy), (ex,ey), (0, 0, 255), thickness=1, line_type=cv2.LINE_4, shift=0, tipLength=0.1)
            
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
            
            # point = Point()
            # point.x = 1.0
            # point.y = 2.0
            # point.z = 1.0
            # marker.points.append(point)
            marker.pose.position.x = best_est[0]
            marker.pose.position.y = best_est[1]
            marker.pose.position.z = 0
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.a = 1  # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            # marker.lifetime = rospy.Duration(100.0)
            # while not rospy.is_shutdown(): 
            self.marker_pub.publish(marker)
            rate.sleep()
            
            self.send_goals([best_est[0], best_est[1], self.road_len])
            return BoxinfoResponse(best_est[0], best_est[1], self.road_len)
        else:
            return BoxinfoResponse(0, 0, self.road_len)
  
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
        rospy.sleep(0.1)
        print('OK')



def main():
    #import pdb;pdb.set_trace()
    rospy.init_node('map_and_scan_buffer_node')
    print("ok 1")
    buffer = MapAndScanBuffer()
    rospy.spin()

if __name__ == '__main__':
    main()
