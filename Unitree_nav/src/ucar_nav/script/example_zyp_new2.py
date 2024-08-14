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
import std_msgs.msg
# from ucar_nav.srv import Boxinfo, BoxinfoResponse
from ucar_nav.msg import Boxinfo
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
from visualization_msgs.msg import Marker
import math
import dynamic_reconfigure.client
from sensor_msgs.msg import PointCloud2, PointField
import struct

from sklearn.cluster import KMeans, DBSCAN
from sklearn.mixture import GaussianMixture

cam_wid = 848  # 摄像头width
cam_hig = 480  # 摄像头height
f = 700 # 摄像头焦距
LaserLen = 360 # 激光雷达数据长度
qs = 10         # 传感器队列长

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
        self.PoseFIFO = qs * [None, ]
        self.map_array = None       
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0
        self.map_info = None
        self.previous_goal =(0,0)
        self.visited_frontiers_worldpose = []
        self.visited_fire_points = []
        self.best_est = None
        self.reach_flag = False
        self.test_count = 0
        self.flag=1
        self.start_exploration = True
        self.previous_robot_pose=(0,0)
        self.frontiers1=[]
        self.height=0
        self.width=0
        self.global_map_ori=None
        self.reaching_fire_count = 0
        self.reach_frontier=0
        self.middle_point = (0, 0)
        self.x_min_max = (0, 0)
        self.y_min_max = (0, 0)
        self.obstacle_points = []
        self.x_min_max_world = (0, 0)
        self.y_min_max_world = (0, 0)

        self.kmeans = KMeans(n_clusters=2)
        self.DBSCAN = DBSCAN(eps=30, min_samples=30)
        self.gmm = GaussianMixture(n_components=1, random_state=0)
        
        # 订阅全局代价地图
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_map_callback)
        time.sleep(1)       

        # 位姿计算及状态更新器
        # self.update_pose = rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, self.pose_listener, queue_size=2)
        self.update_pose = rospy.Subscriber('/posemsg', Float32MultiArray, self.pose_listener)
        # self.current_pose = rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, self.odometry_callback, queue_size=2)
        self.current_pose = rospy.Timer(rospy.Duration(0.2), self.odometry_callback)

        # # 火源位置计算服务终端
        self.Boxtf_srv = rospy.Subscriber('/box_place', Boxinfo, self.callback_box)

        # 激光雷达数据监听器
        # self.Laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan, queue_size=2)
        self.Laser_sub = rospy.Subscriber('/slamware_ros_sdk_server_node/scan', LaserScan, self.callback_scan, queue_size=2)

        # marker publisher
        self.marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)

        self.obs_pub = [rospy.Publisher('/obstacle_marker1', Marker, queue_size=10), \
                        rospy.Publisher('/obstacle_marker2', Marker, queue_size=10), \
                        rospy.Publisher('/obstacle_marker3', Marker, queue_size=10)]

    
        self.pointcloud_pub = rospy.Publisher('pointcloud_obs', PointCloud2, queue_size=5)

        # self.dyn_param_client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS/max_vel_theta")
        self.dyn_param_client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")

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

    def global_map_callback(self, msg):
        self.global_map = msg

    def callback_scan(self, data):
        scandata = data.ranges   # 激光雷达数据，是一个元组
        scandata1 = scandata[180:]   # 调整激光雷达数据的顺序
        scandata2 = scandata[:180]
        scandata = scandata1 + scandata2  # 重新拼接列表
        scandata = np.array(scandata)
        scandata[np.isinf(scandata)] = 0.7  # 改造值为inf的数据
        scandata[np.where(scandata<0.01)[0]] = 0.7  # 改造值为0的数据

        self.ScanFIFO[1:qs] = self.ScanFIFO[0:qs-1]           # 队列更新
        self.ScanFIFO[0] = (scandata, data.header.stamp)  # 监听激光雷达扫描的数据

    def distance(self, point1, point2):
        """ 计算两点之间的欧氏距离 """
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
 
    def update_obstacle_pts(self):
        # import pandas as pd
        # df = pd.DataFrame(self.map_array)
        # df.to_csv("output1.csv", index=False, header=None)
        # print("self.map_array: ", self.map_array.tolist())
        self.detect_frontiers_for_obs(self.map_info)
        mod_map = np.where(self.map_array != 0, 100, self.map_array)
        mod_map_size = np.shape(mod_map)

        collected_obstacle_pts = set()
        collected_obstacle_pts_pixel = []
        all_points = []
        for i in range(mod_map_size[0]): #width
            for j in range(mod_map_size[1]): # height
                if mod_map[i][j] != 100:
                    continue
                else:

                    add_x = j * self.map_info.resolution + self.map_info.origin.position.x
                    add_x = round(add_x, 2)
                    add_y = i * self.map_info.resolution + self.map_info.origin.position.y
                    add_y = round(add_y, 2)

                    all_points.append((j, i))

                    if j < self.x_min_max_for_obs[0] + 40 or j > self.x_min_max_for_obs[1] - 40:
                        continue
                    if i < self.y_min_max_for_obs[0] + 40 or i > self.y_min_max_for_obs[1] - 30:
                        continue
                    
                    collected_obstacle_pts_pixel.append([j, i])
                    collected_obstacle_pts.add((add_x, add_y))
        
        collected_obstacle_pts = list(collected_obstacle_pts)
        # print("collected_obstacle_pts: ", collected_obstacle_pts[:5])
        # print("len of collected_obstacle_pts: ", len(collected_obstacle_pts))
        get_center_array = np.array(collected_obstacle_pts_pixel)

        # all_points_array = np.array(all_points)
        image_size = (self.height, self.width, 3)
        image = np.zeros(image_size, dtype=np.uint8)
        background_color = (255, 255, 255)
        for pt in all_points:
            cv2.circle(image, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1) #green  others
        for pt in collected_obstacle_pts_pixel:
            cv2.circle(image, (int(pt[0]), int(pt[1])), 2, (255, 0, 0), -1) #blue obs

        # self.kmeans.fit(get_center_array)
        # self.obstacle_points = self.kmeans.cluster_centers_

        # self.DBSCAN.fit(get_center_array)
        # labels = self.DBSCAN.labels_
        # print(np.unique(labels))
        # centers = []
        # # for i in range(len(set(labels)) - 1):
        # for i in range(2):
        #     cluster_pts = get_center_array[labels == i]
        #     center = np.mean(cluster_pts, axis=0)
        #     centers.append(center)

        print("get_center_array", get_center_array)
        self.gmm.fit(get_center_array)
        labels = self.gmm.predict(get_center_array)
        print("labels: ", np.unique(labels))
        
        centers = []
        for i in range(len(set(labels))):
        # for i in range(2):
            cluster_pts = get_center_array[labels == i]
            center = np.mean(cluster_pts, axis=0)
            cv2.circle(image, (int(center[0]), int(center[1])), 2, (0, 0, 255), -1) #red obs
            x = center[0] * self.map_info.resolution + self.map_info.origin.position.x
            y = center[1] * self.map_info.resolution + self.map_info.origin.position.y
            centers.append([x, y])
        
        for x in self.x_min_max_for_obs:
            for y in self.y_min_max_for_obs:
                cv2.circle(image, (int(x), int(y)), 2, (0, 0, 255), -1) #red obs

        print("centers: ", centers)

        # cv2.imshow("obs plot", image)
        # cv2.waitKey(0)
        cv2.imwrite("obs_plot.png", image)
        cv2.destroyAllWindows()

        self.obstacle_points = centers
        # print("self.obstacle_points of cluster center", self.obstacle_points)
        
    def pose_listener(self, msg):
        self.robot_x_pose =  msg.data[0] #msg.pose.pose.position.x
        self.robot_y_pose = msg.data[1] #msg.pose.pose.position.y
        yaw = msg.data[2]
        self.Pose = [self.robot_x_pose, self.robot_y_pose, yaw]
        self.PoseFIFO[1:qs] = self.PoseFIFO[0:qs-1]
        self.PoseFIFO[0] = ([self.robot_x_pose, self.robot_y_pose, yaw], rospy.Time.now())

        
        # self.robot_x_pose =  msg.pose.pose.position.x
        # self.robot_y_pose = msg.pose.pose.position.y
        # orien = msg.pose.pose.orientation
        # olist = [orien.x, orien.y, orien.z, orien.w]
        # (_, _, yaw) = euler_from_quaternion(olist)
        # self.Pose = [self.robot_x_pose, self.robot_y_pose, yaw]
        # self.PoseFIFO[1:qs] = self.PoseFIFO[0:qs-1]
        # self.PoseFIFO[0] = ([self.robot_x_pose, self.robot_y_pose, yaw], rospy.Time.from_sec(msg.header.stamp.secs))
        # self.PoseFIFO[0] = ([self.robot_x_pose, self.robot_y_pose, yaw], rospy.Time.now())

    def odometry_callback(self, msg):   
        # print("#################### odometry {} #####################".format(self.test_count))
        self.test_count += 1        
        # 接收地图消息
        # self.robot_x_pose =  msg.pose.pose.position.x
        # self.robot_y_pose = msg.pose.pose.position.y
        # orien = msg.pose.pose.orientation
        # olist = [orien.x, orien.y, orien.z, orien.w]
        # (_, _, yaw) = euler_from_quaternion(olist)
        # self.Pose = [self.robot_x_pose, self.robot_y_pose, yaw]
        # self.PoseFIFO[1:qs] = self.PoseFIFO[0:qs-1]
        # self.PoseFIFO[0] = ([self.robot_x_pose, self.robot_y_pose, yaw], rospy.Time.now())
        # self.PoseFIFO[0] = ([self.robot_x_pose, self.robot_y_pose, yaw], rospy.Time.from_sec(msg.header.stamp.secs))
        # print("time diff: ", (rospy.Time.now() - rospy.Time.from_sec(msg.header.stamp.secs)).to_sec())
        
        # if len(self.visited_fire_points) > 6: return # 08.11 add for termination
        if self.flag==1:
            self.height=self.global_map.info.height
            self.width=self.global_map.info.width
            self.global_map_ori=self.global_map.data
            self.map_array = np.array(self.global_map_ori).reshape((self.height,  self.width))
            self.map_info = self.global_map.info             
            downsample_factor = 2  
            downsampled_height = self.height // downsample_factor
            downsampled_width = self.width // downsample_factor
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

        if not self.start_exploration:
        # if True:
            dist_threshold = 1
            if self.best_est is not None:
                current_pose = self.Pose
                print("current_pose: ", current_pose)
                print("self.best_est: ", self.best_est)
                print("dist(self.best_est, current_pose): ", dist(self.best_est, current_pose))

                if dist(self.best_est, current_pose) <= dist_threshold or self.reaching_fire_count > 20:
                    print("+------- Goal Reached! -------+")
                    self.reach_flag = True
                    self.start_exploration = True
                    self.visited_frontiers_worldpose = []  #到达火点以后需要释放以前的边界点
                else:
                    print("send fire goal")
                    self.reaching_fire_count += 1
                    self.send_goals([self.reaching_fire_count / 20. * self.best_est[0]\
                                         + ( 1 - self.reaching_fire_count / 20.) * self.robot_x_pose, \
                                     self.reaching_fire_count / 20. * self.best_est[1] \
                                     + ( 1 - self.reaching_fire_count / 20.) * self.robot_y_pose, \
                                     current_pose[2]])
                    # self.send_goals([self.best_est[0], self.best_est[1], 0])
                    
                    # publish the marker
                    self.marker_publish(self.best_est, self.marker_pub)
        else:
            print("#######################flag###########################:",self.flag)
            if self.flag==1:
                self.flag=0
                self.frontiers1 = self.detect_frontiers(map_info=self.map_info)   
                self.detect_frontiers_for_obs(map_info=self.map_info)
                self.update_obstacle_pts()
                # print("self.obstacle_points: ", self.obstacle_points)
                print("self.x_min_max_world: ", self.x_min_max_world)
                print("self.y_min_max_world: ", self.y_min_max_world)
            goal = self.select_goal(self.frontiers1 , self.map_array)
            print("没有找到目标点,探索目标，Selected Goal:", goal)
            print("已经探索过的点的个数：",len(self.visited_frontiers_worldpose))
            # import pdb; pdb.set_trace()
            print("目标点：",goal,"中心点：",self.middle_point)            
            if self.middle_point[0] < goal[0]:
                target_yaw = np.pi + np.arctan((self.middle_point[1] - goal[1]) / (self.middle_point[0] - goal[0]))  
            else:
                target_yaw = np.arctan((self.middle_point[1] - goal[1]) / (self.middle_point[0] - goal[0]))           
            
            #angle_robot = math.atan((goal[1]-self.previous_robot_pose[1])/(goal[0]-self.previous_robot_pose[0]))
            self.send_goals([goal[0], goal[1], target_yaw])
            # self.dyn_param_client.update_configuration({"max_vel_theta": 0.5})
            ##### publish the marker
            # self.marker_publish([goal[0], goal[1]], self.marker_pub)
            self.marker_publish([self.middle_point[0], self.middle_point[1]], self.marker_pub)
            for pt_idx, pt in enumerate(self.obstacle_points):
                self.marker_publish(pt, self.obs_pub[pt_idx])


### Exploration Strategy: Start ###

    def find_near_origin_point(self,origin_position, points,min_distance):
        """ 找到机器人位置最近的点 """
        origin_point_distance=[]        
        for point in points:
            dist = self.distance(origin_position, point)
            if dist < min_distance:                
                origin_point_distance.append(point)
        return origin_point_distance

    def find_nearest_point(self,robot_position, points):
        """ 找到机器人位置最近的点 """
        if len(points) == 0: return None

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

    def near_obstacle(self, x, y, map_data, threshold, downsample_factor):#边界周围20个单元格是否有障碍物 也就是当前1m范围内有没有障碍物
        height, width = map_data.shape
        downsampled_x = x // downsample_factor #下采样，更粗力度
        downsampled_y = y // downsample_factor    
        start_x = max(0, downsampled_x - threshold)#创建一个边框，不超出地图范围
        end_x = min(width // downsample_factor, downsampled_x + threshold + 1)
        start_y = max(0, downsampled_y - threshold)
        end_y = min(height // downsample_factor, downsampled_y + threshold + 1)      
        neighborhood = map_data[start_y:end_y, start_x:end_x]       
        return np.any(neighborhood == 100)#100表示障碍物


    # def detect_frontiers1(self, map_info):  #所有边界点
    #     frontiers = []
    #     for y in range(map_info.height):
    #         for x in range(map_info.width):
    #             if self.map_array[y, x] == -1:  # unknown cell  #机器人还未探索区域的邻居是边界
    #                 neighbors = self.map_array[y - 1:y + 2, x - 1:x + 2]
    #                 if 0 in neighbors:  # free cell##可走区域 并且边界需要是可以到达的点
    #                     #if self.is_valid_frontier(x, y, map_info):##没有超过图像边界并且周围有障碍物 可以到达的点需要判断周围的障碍物情况
    #                         frontiers.append((round(x,2),round(y,2)))
    #     img = np.zeros((self.global_map.info.height, self.global_map.info.width, 3), dtype=np.uint8)  
    #     for point in frontiers:
    #             x, y = point       
    #             cv2.circle(img, (x, y), 5, (255, 0, 0), -1)#蓝点
            
    #     cv2.imwrite('aaapoints_image.png', img) 
        
    #     return frontiers


    def detect_frontiers(self, map_info):  #所有边界点
        
        from matplotlib import image as mpimg
        #lidar_map=np.load("/home/lijixiang/Thu_unitree/Unitree_nav/lidar_map.npy")
        aa=np.where(self.map_array==99,100,self.map_array)
        lidar_map=np.where(aa==100,255,30)        
        kernel=np.ones((25,25))
        kernel=kernel.astype('uint8')
        lidar_map=lidar_map.astype('uint8')
        image=cv2.dilate(lidar_map,kernel)  
        map_1=np.where(self.map_array==0,255,self.map_array) #可通行区域
        frontiers_ori=np.where(map_1== image,255,0)
        frontiers_ori=frontiers_ori.astype('uint8')
        # kernel=np.ones((3,3))
        # frontiers_ori=cv2.erode(frontiers_ori,kernel)
        num_labels,labels=cv2.connectedComponents(frontiers_ori)
        sizes=np.bincount(labels.ravel())       
        max_label=np.argmax(sizes[1:])+1#排除背景
        max_connected_component=np.zeros_like(frontiers_ori)
        max_connected_component[labels==max_label]=255
        mpimg.imsave("dddd.jpg",max_connected_component)
  
        frontiers = []
        for y in range(map_info.height):
            for x in range(map_info.width):
                if max_connected_component[y, x] == 255:   
                        if self.is_valid_frontier(x, y, map_info):                 
                            frontiers.append((round(x,2),round(y,2)))

        #######中心点周围1m的也加入边界点中，，，，，也就是20个格子
        # frontiers_all= []  
        # for y in range(map_info.height):
        #     for x in range(map_info.width):                                
        #                     frontiers_all.append((round(x,2),round(y,2)))

        # origin_point_distance = self.find_near_origin_point((int(map_info.width/2),int(map_info.height/2)),frontiers_all,20)         
        # for i in range(len(origin_point_distance)):
        #        if self.map_array[origin_point_distance[i]]==0 :
        #             frontiers.append(origin_point_distance[i])    
        self.x_min_max, self.y_min_max = self.get_frontier_max_and_min(frontiers)
        # self.y_min_max, self.x_min_max = self.get_frontier_max_and_min2(self.map_array)
        print("add self.x_min_max: ", self.x_min_max)
        print("add self.y_min_max: ", self.y_min_max)
        self.middle_point = [np.mean(self.x_min_max), np.mean(self.y_min_max)]
        # self.mid_pts = []
        # for x in x_min_max:
        #     for y in y_min_max:
        #         mid_pt = self.get_middle_of_frontiers(x, y, frontiers)
        #         self.mid_pts.append(mid_pt)
        # self.middle_point[0] = self.middle_point[0] * self.map_info.resolution + self.map_info.origin.position.x
        # self.middle_point[1] = self.middle_point[1] * self.map_info.resolution + self.map_info.origin.position.y 
        self.middle_point = self.get_world_coordinates(self.middle_point)
        self.x_min_max_world = self.get_world_coordinates(self.x_min_max)
        self.y_min_max_world = self.get_world_coordinates(self.y_min_max)
        
        return frontiers

    def detect_frontiers_for_obs(self, map_info):  #所有边界点
        
        from matplotlib import image as mpimg
        #lidar_map=np.load("/home/lijixiang/Thu_unitree/Unitree_nav/lidar_map.npy")
        aa=np.where(self.map_array==99,100,self.map_array)
        lidar_map=np.where(aa==100,255,30)        
        kernel=np.ones((20,20))
        kernel=kernel.astype('uint8')
        lidar_map=lidar_map.astype('uint8')
        image=cv2.dilate(lidar_map,kernel)  
        map_1=np.where(self.map_array==0,255,self.map_array) #可通行区域
        frontiers_ori=np.where(map_1== image,255,0)
        frontiers_ori=frontiers_ori.astype('uint8')
        # kernel=np.ones((3,3))
        # frontiers_ori=cv2.erode(frontiers_ori,kernel)
        num_labels,labels=cv2.connectedComponents(frontiers_ori)
        sizes=np.bincount(labels.ravel())       
        max_label=np.argmax(sizes[1:])+1#排除背景
        max_connected_component=np.zeros_like(frontiers_ori)
        max_connected_component[labels==max_label]=255
        mpimg.imsave("dddd.jpg",max_connected_component)
  
        frontiers = []
        for y in range(map_info.height):
            for x in range(map_info.width):
                if max_connected_component[y, x] == 255:   
                        if self.is_valid_frontier(x, y, map_info):                 
                            frontiers.append((round(x,2),round(y,2)))

        #######中心点周围1m的也加入边界点中，，，，，也就是20个格子
        # frontiers_all= []  
        # for y in range(map_info.height):
        #     for x in range(map_info.width):                                
        #                     frontiers_all.append((round(x,2),round(y,2)))

        # origin_point_distance = self.find_near_origin_point((int(map_info.width/2),int(map_info.height/2)),frontiers_all,20)         
        # for i in range(len(origin_point_distance)):
        #        if self.map_array[origin_point_distance[i]]==0 :
        #             frontiers.append(origin_point_distance[i])    
        self.x_min_max_for_obs, self.y_min_max_for_obs = self.get_frontier_max_and_min(frontiers)
        # self.y_min_max, self.x_min_max = self.get_frontier_max_and_min2(self.map_array)
        print("add self.x_min_max: ", self.x_min_max_for_obs)
        print("add self.y_min_max: ", self.y_min_max_for_obs)
        # self.mid_pts = []
        # for x in x_min_max:
        #     for y in y_min_max:
        #         mid_pt = self.get_middle_of_frontiers(x, y, frontiers)
        #         self.mid_pts.append(mid_pt)
        # self.middle_point[0] = self.middle_point[0] * self.map_info.resolution + self.map_info.origin.position.x
        # self.middle_point[1] = self.middle_point[1] * self.map_info.resolution + self.map_info.origin.position.y 
        # self.x_min_max_world = self.get_world_coordinates(self.x_min_max)
        # self.y_min_max_world = self.get_world_coordinates(self.y_min_max)
        
        return frontiers

    def get_world_coordinates(self, pixel_coord):
        x = pixel_coord[0] * self.map_info.resolution + self.map_info.origin.position.x
        y = pixel_coord[1] * self.map_info.resolution + self.map_info.origin.position.y

        return x, y

    def get_frontier_max_and_min(self, frontiers):

        x_min, y_min = 1e+5, 1e+5
        x_max, y_max = -1e+5, -1e+5

        for x, y in frontiers:
            if x < x_min: x_min = x
            if x > x_max: x_max = x

            if y < y_min: y_min = y
            if y > y_max: y_max = y
        
        return (x_min, x_max), (y_min, y_max)
    
    def get_frontier_max_and_min2(self, map):
        h, w = np.shape(map)

        row_sums = map.sum(axis=1)
        max_row_idx_upper = np.argmax(row_sums[:h//2])
        max_row_idx_down  = h//2 + np.argmax(row_sums[h//2:])

        col_sums = map.sum(axis = 0)
        max_col_idx_left  = np.argmax(col_sums[:w//2])
        max_col_idx_right = w//2 + np.argmax(col_sums[w//2:])

        x_min_max = (max_row_idx_upper, max_row_idx_down)
        y_min_max = (max_col_idx_left, max_col_idx_right)
        return x_min_max, y_min_max


    def get_middle_of_frontiers(self, x_mid, y_mid, frontiers):

        min_dist = 1e+5
        min_pt = None
        for pt in frontiers:
            dist_to_mid = self.distance((x_mid, y_mid), pt)
            if dist_to_mid < min_dist: min_pt = (x_mid, y_mid)
        
        if min_pt == None: 
            assert False, "Error"
        else:
            return min_pt

    def is_valid_frontier(self, x, y, map_info):
        if not ( 0 <= x < map_info.width and 0 <= y < map_info.height):# 由于雷达失效可能，超过图像边界的点（一般进不来）
            return False
        if self.near_obstacle(x, y, self.downsampled_map, threshold=5, downsample_factor=2):#目标点周围障碍物情况，周围1m是否有障碍物
            return False        
        return True 

        
    # def select_goal1(self, frontiers, map_data):#边界和地图
    #     valid_frontiers = []
    #     for x, y in frontiers:#检查边界周围是否有障碍物
    #         if not self.near_obstacle(x, y, map_data, threshold=3,downsample_factor=2):
    #             valid_frontiers.append((x, y))
    #     if valid_frontiers:#随机选择一个点为导航点
    #         goal_x, goal_y = random.choice(valid_frontiers)
    #         real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x#栅格地图到真实地图转换
    #         real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y
    #         if (real_x, real_y) == self.previous_goal or (self.robot_x_pose == real_x and self.robot_y_pose == real_y):#新的目标点是否与上一个目标点相同
    #             print("和上一个目标点相同，重新选择")
    #             valid_frontiers.remove((goal_x, goal_y))
    #             if valid_frontiers: #可以遍历完所有边界，意味着可以遍历完整个地图
    #                 mid_index = len(valid_frontiers) // 2
    #                 goal_x, goal_y = valid_frontiers[mid_index]
    #                 real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x
    #                 real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y
    #             else:
    #                 print("所有边界点全都探索完.")
    #                 return None, None
    #         self.previous_goal = (real_x, real_y)
    #         return real_x, real_y
    #     else:
    #         return None, None

    def select_goal(self, frontiers, map_data):#边界和地图
        #import pdb;pdb.set_trace()
        import cv2
        import numpy as np
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
   
        valid_frontiers = []   
        for x, y in frontiers:
            if not self.near_obstacle(x, y, map_data, threshold=5,downsample_factor=2): #检查边界周围是否有障碍物
                real_x = x * self.map_info.resolution + self.map_info.origin.position.x
                real_y = y * self.map_info.resolution + self.map_info.origin.position.y    # Yansen 
                if((real_x,real_y) not in self.visited_frontiers_worldpose ):   #边界不在探索过的边界点里，则成为候选边界点                                         
                    valid_frontiers.append((x, y))

        real_robot_x_pose = int((self.robot_x_pose-self.map_info.origin.position.x) /self.map_info.resolution )#真实到栅格 机器人所在位置
        real_robot_y_pose = int((self.robot_y_pose -self.map_info.origin.position.y)/self.map_info.resolution )
          
        while valid_frontiers:        
            #nearest_point = random.choice(valid_frontiers)
            nearest_point=self.find_nearest_point((real_robot_x_pose,real_robot_y_pose),valid_frontiers)#从候选边界点里选择最近的导航点    
            goal_x, goal_y = nearest_point
            real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x#栅格地图到真实地图转换 要导航点
            real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y   
           # if (self.robot_x_pose,self.robot_y_pose)==(real_x,real_y) or self.previous_goal == (real_x, real_y):# 新選擇的點距離目標點太近    
            if self.distance((self.robot_x_pose,self.robot_y_pose),(real_x,real_y))<0.8 and self.reach_frontier>=1:
                    self.reach_frontier=0                
                    #import pdb;pdb.set_trace()               
                    self.visited_frontiers_worldpose.append((real_x,real_y)) 
                    for i in range (self.width):
                       for j in range (self.height):
                            distance=self.distance((goal_x,goal_y),(i,j))
                            if (distance<30):#20个格子 也就是1m,边界
                                i_world = i * self.map_info.resolution + self.map_info.origin.position.x#栅格地图到真实地图转换 要导航点
                                j_world = j * self.map_info.resolution + self.map_info.origin.position.y  
                                self.visited_frontiers_worldpose.append((i_world,j_world)) 
                                for dx in [-1,0,1]:
                                    for dy in [-1,0,1]:                                        
                                        if (i+dx,j+dy) in valid_frontiers:                                            
                                            valid_frontiers.remove((i+dx,j+dy))

            self.previous_robot_pose=(self.robot_x_pose,self.robot_y_pose)
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)        
            for point in valid_frontiers:
                x, y = point       
                cv2.circle(img, (x, y), 5, (255, 0, 0), -1)#蓝点
            real_robot_x_pose = int((self.robot_x_pose-self.map_info.origin.position.x) /self.map_info.resolution )#真实到栅格 机器人所在位置
            real_robot_y_pose = int((self.robot_y_pose -self.map_info.origin.position.y)/self.map_info.resolution )          
            cv2.circle(img, (real_robot_x_pose,real_robot_y_pose), 5, (0, 0, 255), -1)#红点
            cv2.circle(img, nearest_point, 5, (0, 255, 0), -1)#目标点 绿点
            cv2.imwrite('aaapoints_image3.png', img) 
            cv2.destroyAllWindows()
            # print("机器人到目标点的距离：",self.distance((self.robot_x_pose,self.robot_y_pose),(real_x,real_y)),
            #     "机器人上一步和这一步的差距：",self.distance(self.previous_robot_pose,(self.robot_x_pose,self.robot_y_pose)),
            #     "机器人当前坐标：",self.robot_x_pose,self.robot_y_pose,
            #     "边界点坐标：",real_x,real_y)
            self.reach_frontier+=1
            self.previous_goal = (real_x, real_y) 
                
            return real_x, real_y
        # print("所有边界点全都探索完.整个房间探索完毕")
        return None, None
  
### Exploration Strategy: End ###

########################################################################################################################################

### Fire Detection: Start ###
    def pointcloud_publish(self, data, publisher):
        # 将预估点可视化为一个绿色的球体
        rate = rospy.Rate(1)  # 1 HZ
        while not rospy.is_shutdown(): 
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'

            msg_field = [0, 1, 2]
            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = len(data)
            msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                # PointField('z', 8, PointField.FLOAT32, 1)
            ]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = msg.point_step * msg.width

            msg.is_dense = False
            msg.data = np.asarray(data, np.float32).tobytes()

            publisher.publish(msg)
            rate.sleep()

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

        # self.dyn_param_client.update_configuration({"max_vel_theta": 0.1})
        # 相机模型计算火源位置
        # Boxs = req.Boxs
        x = req.box_x
        y = req.box_y
        w = req.box_w
        h = req.box_h
        box = [int(x - w/2), int(x + w/2)]  # 人像框对应的左右像素索引

        #try:
        #    laser_scan = rospy.wait_for_message("/slamware_ros_sdk_server_node/scan", LaserScan, timeout=0.1)
        #except:
        #    print("[WARNING] Cannot receive laser data in callback_box!")
        #    return

        theta1 = np.arctan((cam_wid / 2 - box[0]) / f)   # box左边界点在camera坐标系下的极角
        theta2 = np.arctan((cam_wid / 2 - box[1]) / f)   # box右边界点在camera坐标系下的极角
        # print("theta1: {}, theta2: {}".format(str(theta1), str(theta2)))

        min_err = 100      # 误差计数值e
        best_est = None    # 最优坐标估计

        # Cp_idx, Sc_idx = 0, 0     # 时间对齐处对应的位姿\雷达数据队列中的索引
        Cp_idx, Sc_idx = 0, -1     # 时间对齐处对应的位姿\雷达数据队列中的索引
        Cp_min, Sc_min = 1, 0.1 # 最小对齐时间(秒)
        # print("len(self.PoseFIFO): ", len(self.PoseFIFO))
        scan_queue = self.ScanFIFO.copy()
        pose_queue = self.PoseFIFO.copy()
        
        # data sequence: boxplace > scan > pose
        # obtain the nearest scan data
        for k in range(len(scan_queue)):       # 选择最为恰当的时间点，即摄像头，激光雷达，位姿三者的对齐时刻
            # print("self.PoseFIFO[k][1]: ", self.PoseFIFO[k][1])= dist_threshold
            if scan_queue[k] is None: continue
            # if Sc_min > abs((scan_queue[k][1] - req.header.stamp).to_sec())\
            #     and (scan_queue[k][1] - req.header.stamp).to_sec() > 0:
            if Sc_min > abs((scan_queue[k][1] - req.header.stamp).to_sec()):
                Sc_min = abs((scan_queue[k][1] - req.header.stamp).to_sec())
                Sc_idx = k
        
        if Sc_idx != -1:
            nearest_scan_time = scan_queue[Sc_idx][1]
            for k in range(len(pose_queue)):       # 选择最为恰当的时间点，即摄像头，激光雷达，位姿三者的对齐时刻
                # print("self.PoseFIFO[k][1]: ", self.PoseFIFO[k][1])= dist_threshold
                if pose_queue[k] is None: continue

                pose_time_sub_scan = (pose_queue[k][1] - nearest_scan_time).to_sec()
                if pose_time_sub_scan > 0 and Cp_min > pose_time_sub_scan:
                    Cp_min = pose_time_sub_scan
                    Cp_idx = k
            
            # if Cp_min > abs((pose_queue[k][1] - req.header.stamp).to_sec()):
            #     Cp_min = abs((pose_queue[k][1] - req.header.stamp).to_sec())
            #     Cp_idx = k
        
            print("req.header.stamp: ", req.header.stamp)
            print("Cp_idx: {}, Sc_idx: {}".format(Cp_idx, Sc_idx))
            print("cp time - scan time: {}, \n sc time - boxplace time: {}"\
                .format((pose_queue[Cp_idx][1] - scan_queue[Sc_idx][1]).to_sec(), \
                        (scan_queue[Sc_idx][1] - req.header.stamp).to_sec()))
        # if self.PoseFIFO[0] is not None:
        #     print("self.PoseFIFO[0][1] stamp: ", self.PoseFIFO[0][1], (self.PoseFIFO[0][1] - req.header.stamp).to_sec())
        # if self.PoseFIFO[-1] is not None:
        #     print("self.PoseFIFO[-1][1] stamp: ", self.PoseFIFO[-1][1], (self.PoseFIFO[-1][1] - req.header.stamp).to_sec())
        # if self.ScanFIFO[0] is not None:
        #     print("self.ScanFIFO[0][1] stamp: ", self.ScanFIFO[0][1], (self.ScanFIFO[0][1] - req.header.stamp).to_sec())
        # if self.ScanFIFO[-1] is not None:
        #     print("self.ScanFIFO[-1][1] stamp: ", self.ScanFIFO[-1][1], (self.ScanFIFO[-1][1] - req.header.stamp).to_sec())

        # if Cp_idx == -1:
        #     print("[WARNING] Cannot align the time of Pose and Image ")
        if Sc_idx == -1:
            print("[WARNING] Cannot align the time of lidar and Image ")
        # if Cp_idx == -1 or Sc_idx == -1:
            return


        scandata = scan_queue[Sc_idx][0]  # 提取出对齐时间的激光雷达数据
        cpose = pose_queue[Cp_idx][0]     # 提取出对齐时间的位姿数据
        
        ind1 = int(theta1 * LaserLen / (2*np.pi))  # box左边界点在激光雷达数据中的索引
        ind2 = int(theta2 * LaserLen / (2*np.pi))  # box右边界点在激光雷达数据中的索引
        # print("ind1: {}, ind2: {}".format(ind1, ind2))
        # print("scan_data: ", scandata)
        # if ind1 >= 0 and ind2 < 0:
        #     dis_laser = np.median(np.concatenate([scandata[ind2:], scandata[:ind1]]))
        # else:
        #     dis_laser = np.median(scandata[ind2:ind1])  # 激光雷达得到的到Box距离的平均值

        if ind1 >= 0 and ind2 < 0:
            laser_data = np.concatenate([scandata[ind2:], scandata[:ind1]])
        else:
            laser_data = scandata[ind2:ind1]
        
        # delete the infinity point
        real_laser_data = []
        for l_data in laser_data:
            if l_data != 0.7:
                real_laser_data.append(l_data)
        if len(real_laser_data) == 0:
            return

        # the distance to fire
        print("cpose: ", cpose)
        print("laser_data: ", laser_data)
        print("real_laser_data: ", real_laser_data)
        dis_laser = np.median(real_laser_data)
        
        # 不考虑 X m以上距离的火源位置
        fire_distance = 7
        if dis_laser > fire_distance: 
            print("[WARNING] Not consider the fire beyond {} meters".format(fire_distance))
            print("[WARNING] The fire is {} meters far from the dog".format(dis_laser))
            return 

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
        if self.reach_flag is True or self.best_est is None:
            print('best_est 2: ', best_est)
            print(" ######## Goal changed ######## ")
            # back = 0.5
            # print("best_est[0] > self.x_min_max_world[0] + back: ", best_est[0] > self.x_min_max_world[0] + back)
            # print("best_est[0] < self.x_min_max_world[1] - back: ", best_est[0] < self.x_min_max_world[1] - back)
            # print("best_est[1] > self.y_min_max_world[1] + back: ", best_est[1] > self.y_min_max_world[1] + back)
            # print("best_est[1] < self.y_min_max_world[1] - back: ", best_est[1] < self.y_min_max_world[1] - back)
            
            modified_goal = self.get_fire_pt_strategy(best_est, dis_laser, Cp_min)
            if modified_goal is not None:
                print("########### Using the obstacle strategy #############")
                # self.update_obstacle_pts() # may have delay
                # obs_pt = self.find_nearest_point(best_est, self.obstacle_points)
                if self.distance(modified_goal, cpose) > fire_distance:
                    print("[WARNING] ################ Cancel the obstacle strategy")
                    return
                if not self.is_near(modified_goal, self.visited_fire_points):  ### 08.11 add
                    best_est = modified_goal

            if self.is_near(best_est, self.visited_fire_points):
                return

            # map_1 = np.where(self.map_array==0,255,self.map_array) #可通行区域
            # min_distance = 10000
            # min_distance_pt = [0, 0]
            # for i in range(self.height):
            #     for j in range(self.width):
            #         map_array_world = self.get_world_coordinates([j, i])
            #         dist_to_best_est = self.distance(self.best_est, map_array_world)
            #         if self.map_array[i, j]== 0 and dist_to_best_est < min_distance:
            #             min_distance = dist_to_best_est
            #             min_distance_pt = map_array_world
            # self.best_est = min_distance_pt

            self.best_est = best_est
            self.visited_fire_points.append([self.best_est[0], self.best_est[1]])
            self.reach_flag = False
            self.start_exploration = False
            self.reaching_fire_count = 0

    def is_near(self, pt, pt_list, threshold=1):
        for p in pt_list:
            if self.distance(pt, p) < threshold:
                return True
        
        return False

    def get_fire_pt_strategy(self, est_goal, laser_distance, diff_sec):

        obs_array = np.array(self.obstacle_points)
        x_min = np.min(obs_array[:, 0])
        x_max = np.max(obs_array[:, 0])
        y_min = np.min(obs_array[:, 1])
        y_max = np.max(obs_array[:, 1])

        modified_goal = None
        if est_goal[0] > x_min - 1 and est_goal[0] < x_max + 1:
            if est_goal[1] > y_min - 1 and est_goal[1] < y_max + 1:
                modified_goal = self.find_nearest_point(est_goal, self.obstacle_points)

        # modified_goal = None
        # min_distance = 1e+5
        # for pt in self.obstacle_points:
        #     dist = self.distance(pt, est_goal) 
        #     if dist < min_distance and dist < 0.5 * diff_sec * laser_distance:
        #         modified_goal = pt
        #         min_distance = dist

        return modified_goal


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
        # print('OK')

### Fire Detection: End ###

def main():
    rospy.init_node('map_and_scan_buffer_node')
   # print("ok 1")
    buffer = MapAndScanBuffer()
   # print("ok 2")
    
    rospy.spin()

if __name__ == '__main__':
    main()
