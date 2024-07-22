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
from ucar_nav.srv import Boxinfo, BoxinfoResponse

from visualization_msgs.msg import Marker

cam_wid = 848  # 摄像头width
cam_hig = 480  # 摄像头height
f = 405 # 摄像头焦距
LaserLen = 360 # 激光雷达数据长度
qs = 8         # 传感器队列长

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
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # 目标发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.ggoal = MoveBaseGoal()

        # 火源位置计算服务终端
        self.Boxtf_srv = rospy.Service('/box_place', Boxinfo, self.callback_box)

        # 激光雷达数据监听器
        self.Laser_sub = rospy.Subscriber('/slamware_ros_sdk_server_node/scan', LaserScan, self.callback_scan, queue_size=2)
        
        self.current_pose = rospy.Subscriber('/slamware_ros_sdk_server_node/odom', Odometry, self.odometry_callback)

        self.marker_pub = rospy.Publisher('/ss_marker', Marker, queue_size=10)

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
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orien = msg.pose.pose.orientation
        olist = [orien.x, orien.y, orien.z, orien.w]
        (_, _, yaw) = euler_from_quaternion(olist)
        self.Pose = [x, y, yaw]
        self.PoseFIFO[1:qs] = self.PoseFIFO[0:qs-1]
        self.PoseFIFO[0] = (self.Pose, rospy.Time.now())

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
        # print("ind1: {}, ind2: {}".format(ind1, ind2))
        # print("scan_data: ", scandata)
        if ind1 >= 0 and ind2 < 0:
            dis_laser = np.median(np.concatenate([scandata[ind2:], scandata[:ind1]]))
        else:
            dis_laser = np.median(scandata[ind2:ind1])  # 激光雷达得到的到Box距离的平均值
            
        # print("dis_laser: ", dis_laser)
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
        # if -0.125 < best_est[0] < 3.875 and -3.875 < best_est[1] < 8:
        if True:
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
            
            # marker.lifetime = rospy.Duration(60.0)
            # while not rospy.is_shutdown(): 
            self.marker_pub.publish(marker)
            rate. sleep()
            
            print("dist(best_est, cpose) : ", dist(best_est, cpose) )
            while dist(best_est, cpose) > 0.1:
                print("send")
                self.send_goals([best_est[0], best_est[1], self.road_len])
                rospy.sleep(20)
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
        print('OK')

def main():
    rospy.init_node('Scan_cam_test')
    Scan_cam_test()
    rospy.spin()


if __name__ == '__main__':
    main()
