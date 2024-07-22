import numpy as np
import scipy
import skimage.morphology
import torch
import torch.nn as nn
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from home_robot.mapping.semantic.constants import MapConstants as MC
from home_robot.utils.morphology import binary_dilation

import cv2
from PIL import Image

bShow = False
bShowGoal = True

# 两个都是False,执行原来的算法
# bNewFrontier True 表示边线
# bNewRec True 表示优化起始容器

bNewFrontier = False
bNewRec = True

No_ = 10000

furniture_relationships = [[1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                            [0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1],
                            [1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0],
                            [0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1],
                            [0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                            [0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1]]


class ObjectNavFrontierExplorationPolicy(nn.Module):
    """
    Policy to select high-level goals for Object Goal Navigation:
    go to object goal if it is mapped and explore frontier (closest
    unexplored region) otherwise.
    """

    def __init__(self, exploration_strategy: str):
        super().__init__()
        assert exploration_strategy in ["seen_frontier", "been_close_to_frontier"]
        self.exploration_strategy = exploration_strategy
        #import pdb;pdb.set_trace()
        self.dilate_explored_kernel = nn.Parameter(
            torch.from_numpy(skimage.morphology.disk(10)).unsqueeze(0).unsqueeze(0).float(), requires_grad=False, )
        self.select_border_kernel = nn.Parameter(
            torch.from_numpy(skimage.morphology.disk(1)).unsqueeze(0).unsqueeze(0).float(), requires_grad=False, )

        # 记录每次的生效的连通域的面积 长 宽和出现次数，
        # self.record2 = [[1, 2, 3, 4], ]
        self.rec_record = {}
        self.No_ = 10000
        self.count=0
        self.index=1000
        self.rec={}
        self.shortest={}
        self.area=-1
        self.list=[]
        self.found_temp_goal=torch.tensor([False])
        self.tuple=(torch.tensor([1000,1001]),1000)
    @property
    def goal_update_steps(self):
        return 1

    def read_distance_matrix(self,file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
            distance_matrix = []
            for line in lines:
                distances = list(map(float, line.strip().split()))
                distances = [10000 if distance == 0 else distance for distance in distances]
                distance_matrix.append(distances)      
        return distance_matrix

    import torch

    def dijkstra_with_path(self,start,end):
        graph=self.read_distance_matrix("/home/ubuntu/zyp/home-robot/distence/ave_1.txt")
        num_vertices = len(graph)
        INF = float('inf')
        distances = [INF] * num_vertices
        distances[start] = 0
        visited = [False] * num_vertices
        previous = [None] * num_vertices

        for _ in range(num_vertices):
            min_distance = INF
            min_index = -1
            for i in range(num_vertices):
                if not visited[i] and distances[i] < min_distance:
                    min_distance = distances[i]
                    min_index = i

            visited[min_index] = True

            for i in range(num_vertices):
                if (not visited[i]) and (graph[min_index][i] != INF) and (distances[min_index] + graph[min_index][i] < distances[i]):
                    distances[i] = distances[min_index] + graph[min_index][i]
                    previous[i] = min_index

        # Retrieve the shortest path
        path = []
        current_vertex = end
        
        while current_vertex is not None:
            path.append(current_vertex)
            current_vertex = previous[current_vertex]
        path.reverse()
        #import pdb;pdb.set_trace()
        return distances[end], torch.tensor(path) 


    #shortest_distance, shortest_path = self.dijkstra_with_path(22, 3)#最短路径长度   从开始到最后走过的容器


    def reach_single_category(self, map_features, category):
        # if the goal is found, reach it
        goal_map, found_goal = self.reach_goal_if_in_map(map_features, category)
        # otherwise, do frontier exploration
        goal_map = self.explore_otherwise(map_features, goal_map, found_goal)
        return goal_map, found_goal
    
    # 组合
    def reach_object_recep_combination(self, map_features, object_category, recep_category):
        import pdb;pdb.set_trace()
        goal_map, found_goal = self.reach_goal_if_in_map(map_features, recep_category, small_goal_category=object_category, )

        if bNewRec:
            goal_map, found_rec_goal = self.reach_goal_if_in_map_rec(map_features, recep_category, goal_map=goal_map, found_goal=found_goal, )

            goal_map,found_temp_goal1 = self.explore_otherwise(map_features, goal_map, found_rec_goal,recep_category)
            found_rec_goal1=torch.tensor([False])
            if bShowGoal:
                if found_goal[0] == True:
                    print("目标物体")
                   # import pdb;pdb.set_trace()
                else:
                    if found_rec_goal[0] == True :
                
                        print("起始容器")
                        #import pdb; pdb.set_trace()
                    elif  found_temp_goal1[0]==True:
                        print("临时容器")
                        found_rec_goal1[0]=True
                    else:
                        print("边界")            
         
            return goal_map, found_goal, found_rec_goal1[0]

   

    def forward(self, map_features, object_category=None, start_recep_category=None, end_recep_category=None,
                nav_to_recep=None, ):
        """
        Arguments:
            map_features: semantic map features of shape
             (batch_size, 9 + num_sem_categories, M, M)
            object_category: object goal category
            start_recep_category: start receptacle category
            end_recep_category: end receptacle category
            nav_to_recep: If both object_category and recep_category are specified, whether to navigate to receptacle
        Returns:
            goal_map: binary map encoding goal(s) of shape (batch_size, M, M)
            found_goal: binary variables to denote whether we found the object
            goal category of shape (batch_size,)
        """
        assert object_category is not None or end_recep_category is not None

        if object_category is not None and start_recep_category is not None:
            if nav_to_recep is None or end_recep_category is None:
                nav_to_recep = torch.tensor([0] * map_features.shape[0])

            if nav_to_recep.sum() < map_features.shape[0]:#第一阶段
                goal_map_o, found_goal_o, found_rec_o = self.reach_object_recep_combination(map_features,
                                                                                            object_category,
                                                                                            start_recep_category)
            elif nav_to_recep.sum() > 0:#第三阶段
                goal_map_r, found_goal_r = self.reach_single_category(map_features, end_recep_category)

            if nav_to_recep.sum() == 0:
                return goal_map_o, found_goal_o, found_rec_o
            elif nav_to_recep.sum() == map_features.shape[0]:

                batch_size, _, height, width = map_features.shape
                device = map_features.device
                found_rec_r = torch.zeros(batch_size, dtype=torch.bool, device=device)
                return goal_map_r, found_goal_r, found_rec_r

            else:
                goal_map = (goal_map_o * nav_to_recep.view(-1, 1, 1) + (1 - nav_to_recep).view(-1, 1, 1) * goal_map_o)
                found_goal = (found_goal_r * nav_to_recep + (1 - nav_to_recep) * found_goal_r)
                return goal_map, found_goal
        else:
            # Here, the goal is specified by a single object or receptacle to navigate to with no additional constraints (eg. the given object can be on any receptacle)
            goal_category = (object_category if object_category is not None else end_recep_category)
            return self.reach_single_category(map_features, goal_category)

    def cluster_filtering(self, m):
        # m is a 480x480 goal map
        if not m.any():
            return m
        device = m.device

        # cluster goal points
        k = DBSCAN(eps=4, min_samples=1)
        m = m.cpu().numpy()
        data = np.array(m.nonzero()).T
        k.fit(data)

        # mask all points not in the largest cluster
        mode = scipy.stats.mode(k.labels_, keepdims=True).mode.item()
        mode_mask = (k.labels_ != mode).nonzero()
        x = data[mode_mask]

        m_filtered = np.copy(m)
        m_filtered[x] = 0.0
        m_filtered = torch.tensor(m_filtered, device=device)

        return m_filtered

    def reach_goal_if_in_map(self, map_features, goal_category, small_goal_category=None, reject_visited_regions=False,
                             goal_map=None, found_goal=None):
        """If the desired goal is in the semantic map, reach it."""
        batch_size, _, height, width = map_features.shape
        device = map_features.device
        if goal_map is None and found_goal is None:
            goal_map = torch.zeros((batch_size, height, width), device=device)
            found_goal_current = torch.zeros(batch_size, dtype=torch.bool, device=device)

        else:
            # crate a fresh map
            found_goal_current = torch.clone(found_goal)

        # found_goal_current
        for e in range(batch_size):
            # if the category goal was not found previously
            if not found_goal_current[e]:
                # 如果当前 还没有找到

                category_map = map_features[e, goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :]#全局

                # if reject_visited_regions:
                #     # remove the receptacles that the already been close to
                #     category_map = category_map * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :])#局部
               
                if small_goal_category is not None:
                    # additionally check if the category has the required small object on it

                    category_map = (category_map * map_features[e, small_goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :])

                    # category_map = map_features[e, small_goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :]

                if (category_map == 1).sum() > 0:
                    goal_map[e] = category_map == 1
                    found_goal_current[e] = True
       # import pdb;pdb.set_trace()
        return goal_map, found_goal_current

    def deleteContour(self, rec_map_all_cpu, rec_map_part_cpu,features_rec_id):
        # 1 起始容器 最小生效面积
        minArea_ = 20
        # 2 起始容器已经有0.7 看过了，剩下的不再看
        maxClearRatio_ = 0.7
        # 没探索过的 探索过的允许停留次数
        maxUnseen_Seen = [400, 15]

        rec_map_all_cpu = np.where(rec_map_all_cpu == 1, 1, 0)#所有桌子
        # plt.imshow(rec_map_all_cpu)
        # plt.savefig("/home/ubuntu/zyp/home-robot/datadump/bb.png")
        rec_map_part_cpu = np.where(rec_map_part_cpu == 1, 1, 0)#从所有桌子中去除探索过的，剩下没有探索的，或者小面积

        rec_map_all_cpu = rec_map_all_cpu.astype(np.uint8)
        rec_map_part_cpu = rec_map_part_cpu.astype(np.uint8)

        # 一、去除小面积 
        opencv_rec = cv2.merge([rec_map_part_cpu])
        num_labels, labels = cv2.connectedComponents(opencv_rec, connectivity=8)
        for i in range(num_labels):
            if i > 0:
                count = np.sum(labels == i)#各个连通区域的面积
                if bShow:
                    print(i, count)
                if count < minArea_:#各个连通面积小于20则清除
                    labels = np.where(labels == i, 0, labels)
        labels = np.where(labels > 0, 1, 0)#所有连通区域为1 

        # 二、去除处于边界的
        labels1 = labels.astype(np.uint8)
        opencv_rec = cv2.merge([labels1])
        num_labels1, labels1 = cv2.connectedComponents(opencv_rec, connectivity=8)
        for i in range(num_labels1):
            if i > 0:
                labels_ = np.where(labels1 == i, 1, 0)#提取一个连通区域
                nonzero_indices = np.where(labels_ != 0)
                # 找到非零元素的横坐标的最大值和最小值
                min_x = np.min(nonzero_indices[1])
                max_x = np.max(nonzero_indices[1])
                min_y = np.min(nonzero_indices[0])
                max_y = np.max(nonzero_indices[0])
                if min_x == 0 or min_y == 0 or max_x == 479 or max_y == 479:#去除在边界的连通区域
                    labels = np.where(labels_ == 1, 0, labels)

        # 三、 如果剩余占比原来的连通域的 不到30% 同样删去,
        # 说明该容器 已经大部分看过了，剩余一部分因为没清除到，滞留下来
        opencv_rec = cv2.merge([rec_map_all_cpu])
        num_labels2, labels2 = cv2.connectedComponents(opencv_rec, connectivity=8)
        for i in range(num_labels2):
            if i > 0:
                labels_ = np.where(labels2 == i, 1, 0)#提取各个连通区域
                AreaOld = np.sum(labels_ == 1) #看当前连通区域的面积
                labels_2 = np.where(labels == 1, 0, labels_)#标出当前连通区域探索过的
                AreaSeen = np.sum(labels_2 == 1)#
                #print(",,,,,,,,,,,,,,,,,,",AreaSeen,AreaOld)
                leftRatio = AreaSeen * 1.0 / AreaOld #单个桌子走过的占总的面积
                if bShow:
                    print(i, AreaSeen, AreaOld, leftRatio)
                if leftRatio > maxClearRatio_:#>看过了70%，则清除整个连通区域
                    labels = np.where(labels_ == 1, 0, labels)#没走过的容器

        # 四、如果当前的labels 中的连通域存在太久，也一并删去
        if bShow:
            print("self.rec_record ", self.rec_record)

        # 因为各种原因，长期滞留的起始容器 需要清除
        # 其中，完全没清除过的容器，停留步数超过60，需要清除
        # 其他的，有被清除掉部分的容器，停留步数超过20，需要清除

        if np.sum(labels == 1) > 0:
            # 增加一个新的mask，对labels上的连通域进行划分
            opencv_rec = cv2.merge([rec_map_all_cpu])
            label_tag = np.where(opencv_rec > 0, 0, 0)
            num_labels2, labels2 = cv2.connectedComponents(opencv_rec, connectivity=8)
            for i in range(num_labels2):
                if i > 0:
                    # 当前连通域，如果当前连通域
                    labels_ = np.where(labels2 == i, 1, 0)
                    labels_2 = np.where(labels == 1, 0, labels_)#探索过的容器
                    AreaSeen = np.sum(labels_2 == 1)
                    # 说明原始的 起始容器一点都没有被清除过，需要保留
                    # 在 label_tag 中标记处没有探索过的大连通域
                    if AreaSeen == 0:#该容器没有被探索过
                        label_tag = np.where(labels_ == 1, 1, label_tag)

            labels3 = labels.astype(np.uint8)
            opencv_rec = cv2.merge([labels3])
            num_labels3, labels3 = cv2.connectedComponents(opencv_rec, connectivity=8)
            # 遍历当前剩余的所有有效起始容器
            for i in range(num_labels3):
                if i > 0:
                    labels_ = np.where(labels3 == i, 1, 0)
                    # 面积 长 宽
                    area = np.sum(labels_ == 1)
                    nonzero_indices = np.where(labels_ != 0)
                    min_y = np.min(nonzero_indices[1])
                    max_y = np.max(nonzero_indices[1])
                    min_x = np.min(nonzero_indices[0])
                    max_x = np.max(nonzero_indices[0])
                    height = max_x - min_x + 1
                    width = max_y - min_y + 1
                    found = False
                    key_ = (features_rec_id,area, height, width)

                    # 先确定键值，面积,高,宽
                    # 需要判断当前连通域是不是完整的连通域
                    # label_tag 保存了原始的起始容器中 完整保留下来的连通域

                    # labels_如果 labels_ 和label_tag 有交集，说明当前的labels_连通域是 没被探索过的
                    # labels_如果在label_tag上全是0，说明该区域是被探索过的，母连通域在labels_tag中不存在

                    labels_count = np.where(labels_ == 1, label_tag, 0)
                    Area_ = np.sum(labels_count == 1)
                    # 有交集，说明当前连通域还没探索过
                    if Area_ > 0:
                        index_ = 0 #400步
                    else:
                        index_ = 1 #15步
                    #import pdb ;pdb.set_trace()
                    #各个容器方位次数是否存在rec_record中，一种是没有探索过的容器，一种是探索次数过多的容器清掉
                    # 存在直接加1，判定
                    if key_ in self.rec_record: #容器没变化
                        found = True
                        self.rec_record[key_] = self.rec_record[key_] + 1
                        if self.rec_record[key_] > maxUnseen_Seen[index_]:
                            labels = np.where(labels_ == 1, 0, labels)

                    # 不存在 需要遍历进行比较
                    else:
                        for key in self.rec_record: #从已经存在的列表中与当前key_计较，可能容器有细微变化了
                            #为了适应所有容器，而非只是目标容器
                            if key[0]==key_[0] and key[1] > 100 and area - 10 < key[1] < area + 10 and height - 3 <   key[2] < height + 3 and width - 3 < key[3] < width + 3:
                                found = True
                                self.rec_record[key] = self.rec_record[key] + 1
                                if self.rec_record[key] > maxUnseen_Seen[index_]:
                                    labels = np.where(labels_ == 1, 0, labels)
                                break

                    if found == False:
                        self.rec_record[key_] = 1

        clear_map_cpu_new = labels
        return clear_map_cpu_new

    # 容器作为目标物体
    def reach_goal_if_in_map_rec(self, map_features, goal_category, goal_map=None, found_goal=None):
        batch_size, _, height, width = map_features.shape
        device = map_features.device
        if goal_map is None and found_goal is None:
            goal_map = torch.zeros((batch_size, height, width), device=device)
            found_goal_current = torch.zeros(batch_size, dtype=torch.bool, device=device)
        else:
            found_goal_current = torch.clone(found_goal)
       #0 障碍物地图 1 探索过的区域 2 当前点，3走过的区域 4 走过的区域（圆圈） 0-4局部 5-9全局 10-33局部 
        for e in range(batch_size):
            if not found_goal_current[e]:
                
                category_map = map_features[e, goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :] # 6+2*5
                rec_map_all = category_map #包含桌子1 桌子2的所有桌子
                category_map = category_map * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :])#4 走过的区域（圆圈） 以前走过的都清掉，
                rec_map_part = category_map#从所有桌子中去除探索过的，剩下没有探索的，或者小面积
                
                rec_map_all_cpu = rec_map_all.cpu().numpy()
                rec_map_part_cpu = rec_map_part.cpu().numpy()
                
                clear_map_cpu_new = self.deleteContour(rec_map_all_cpu, rec_map_part_cpu,goal_category[e] + 2 * MC.NON_SEM_CHANNELS)
                category_map = torch.tensor(clear_map_cpu_new).cuda()
             
                if (category_map == 1).sum() > 0:
                    goal_map[e] = category_map == 1
                    found_goal_current[e] = True
        
        return goal_map, found_goal_current

    def get_frontier_map(self, map_features):
        # Select unexplored area
        # 怎么定义地图边界，或者说已知视野边界
        #import pdb;pdb.set_trace()
        if self.exploration_strategy == "seen_frontier":            
            frontier_map = (map_features[:, [MC.EXPLORED_MAP], :, :] == 0).float()#1
        elif self.exploration_strategy == "been_close_to_frontier":
            frontier_map = (map_features[:, [MC.BEEN_CLOSE_MAP], :, :] == 0).float()

        self.No_ = self.No_ + 1
        strno = str(self.No_)[1:]

        # aa = map_features[:, [MC.EXPLORED_MAP], :, :].cpu().numpy()
        # binary_image = aa.astype(np.uint8)[0][0]
        # binary_image = binary_image * 255
        # image = Image.fromarray(binary_image)
        # image.save('/home/zgh/home-robot/datadump/images/yqt/' + strno + "_map_features.bmp")

        # aa = frontier_map.cpu().numpy()
        # binary_image = aa.astype(np.uint8)[0][0]
        # binary_image = binary_image * 255
        # image = Image.fromarray(binary_image)
        # image.save('/home/zgh/home-robot/datadump/images/yqt/' + strno + "_frontier_map.bmp")

        # Dilate explored area
        frontier_map = 1 - binary_dilation(1 - frontier_map, self.dilate_explored_kernel)

        # aa = frontier_map.cpu().numpy()
        # binary_image = aa.astype(np.uint8)[0][0]
        # binary_image = binary_image * 255
        # image = Image.fromarray(binary_image)
        # image.save('/home/zgh/home-robot/datadump/images/yqt/' + strno + "_frontier_map2.bmp")

        # Select the frontier
        frontier_map = (binary_dilation(frontier_map, self.select_border_kernel) - frontier_map)

        # aa = frontier_map.cpu().numpy()
        # binary_image = aa.astype(np.uint8)[0][0]
        # binary_image = binary_image * 255
        # image = Image.fromarray(binary_image)
        # image.save('/home/zgh/home-robot/datadump/images/yqt/' + strno + "_frontier_map3.bmp")

        return frontier_map
   

    
    def explore_otherwise(self, map_features, goal_map, found_goal,recep_category):
        """Explore closest unexplored region otherwise."""
        
        frontier_map = self.get_frontier_map(map_features)
        batch_size = map_features.shape[0] 
        for e in range(batch_size):
            flag=False
            if not found_goal[e]: #没发现目标容器               
                rec_nonzero=np.nonzero(torch.sum(map_features[e,12:33,:,:],axis=(-2,-1))>5)   #0-20  21类容器
               
                aa=len(rec_nonzero) 

                if recep_category[e].cpu()-2 in rec_nonzero:
                     b=1
                else:
                     b=0
                     
                   
                if (aa==0 or  len(self.list)==aa-b ) :      #不存在容器，走边界
                  
                    print("容器都靠近完，但是没发现目标")        
                    goal_map[e] = frontier_map[e]                 
                else:
                    shortest1=[] 
                    shortest={}                   
                    for i in range (aa):
                        list1=[temp[0] for temp in self.list]
                        if(rec_nonzero[i][0]!= recep_category[e].cpu()-2 and (not rec_nonzero[i][0]+1 in list1) ):                             #txt从1开始到21是对应容器(前面是misc)  代码从2开始是容器（前两个是obj与misc)
                            
                            if furniture_relationships[rec_nonzero[i][0]][recep_category[e]-2]==1:
                                shortest1.append(rec_nonzero[i][0])
                                shortest_distance1, shortest_path1 = self.dijkstra_with_path(rec_nonzero[i][0]+1, recep_category[e]-1)#最短路径长度   从开始到最后走过的容器
                                shortest[shortest_path1]=shortest_distance1 #比正常的容器大1
                    if len(shortest1)>0:
                        #import pdb;pdb.set_trace()
                        min_tuple = min(shortest.items(), key=lambda x: x[1]) #选出n条路径中距离最短的,排除了已经到达的

                        if min_tuple[1]>self.tuple[1] : #其他容器不如当前容器优
                            print("容器",min_tuple[0][0],",,,,",min_tuple[1],"不如",self.tuple[0][0],",,,",self.tuple[1],"路径短，所以走边界")
                            goal_map[e] = frontier_map[e]
                            self.found_temp_goal[e]=False
                        else:
                            if min_tuple[0][0]==self.tuple[0][0] :#这一步和上一步同一个桌子
                                area_current=torch.sum(map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]  * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :]))
                                if( self.area>area_current+5 and self.area!=-1)or self.count>30000:#已经到达了该容器                                                                     
                                    for key in shortest.keys():
                                        #import pdb;pdb.set_trace()
                                        self.rec[key]= map_features[e,key[0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  
                                        self.list.append(key) 
                                    self.count=0       
                                    goal_map[e] = frontier_map[e]        
                                    self.found_temp_goal[e]=False
                                    print("到达当前临时容器，但没有目标容器，走边界")        
                                else:#没到达
                                    print("靠近当前临时容器")
                                    self.area=area_current
                                    goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
                                    self.found_temp_goal[e]=True
                                    self.count+=1
                            else: #不是同一个容器，另一个容器更优

                                self.area=-1      
                                goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
                                self.found_temp_goal[e]=True
                                print("向另一个更优的临时容器靠近",min_tuple[0][0],min_tuple[1])            
                                
                                
                            self.tuple=min_tuple
                    else:
                            print("当前范围内容器如目标容器无关，走边界")
                            goal_map[e] = frontier_map[e]
                            self.found_temp_goal[e]=False

                    
                idx=0                                
                for i in self.list:                  
                    if (torch.sum(self.rec[i])+500<torch.sum(map_features[e,i[0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  ) ):                            
                        del self.list[idx]
                    idx+=1
        return goal_map ,self.found_temp_goal

    # def explore_otherwise(self, map_features, goal_map, found_goal,recep_category):
    #     """Explore closest unexplored region otherwise."""
        
    #     frontier_map = self.get_frontier_map(map_features)
    #     batch_size = map_features.shape[0] 
    #     for e in range(batch_size):
    #         flag=False
    #         if not found_goal[e]: #没发现目标容器               
    #             rec_nonzero=np.nonzero(torch.sum(map_features[e,12:33,:,:],axis=(-2,-1))>5)   #0-20  21类容器
               
    #             aa=len(rec_nonzero) 

    #             if recep_category[e].cpu()-2 in rec_nonzero:
    #                  b=1
    #             else:
    #                  b=0
                     
                   
    #             if (aa==0 or  len(self.list)==aa-b ) :      #不存在容器，走边界
                  
    #                 print("容器都靠近完，但是没发现目标")        
    #                 goal_map[e] = frontier_map[e]                 
    #             else:
    #                 shortest1=[] 
    #                 shortest={}                   
    #                 for i in range (aa):
    #                     list1=[temp[0] for temp in self.list]
    #                     if(rec_nonzero[i][0]!= recep_category[e].cpu()-2 and (not rec_nonzero[i][0]+1 in list1) ):                             #txt从1开始到21是对应容器(前面是misc)  代码从2开始是容器（前两个是obj与misc)
                            
    #                         if furniture_relationships[rec_nonzero[i][0]][recep_category[e]-2]==1:
    #                             shortest1.append(rec_nonzero[i][0])
    #                             shortest_distance1, shortest_path1 = self.dijkstra_with_path(rec_nonzero[i][0]+1, recep_category[e]-1)#最短路径长度   从开始到最后走过的容器
    #                             shortest[shortest_path1]=shortest_distance1 #比正常的容器大1
    #                 if len(shortest1)>1:
    #                     min_tuple = min(shortest.items(), key=lambda x: x[1]) #选出n条路径中距离最短的,排除了已经到达的

    #                     if min_tuple[1]>self.tuple[1] : #其他容器不如当前容器优
    #                         print("容器",min_tuple[0][0],",,,,",min_tuple[1],"不如",self.tuple[0][0],",,,",self.tuple[1],"路径短，所以走边界")
    #                         goal_map[e] = frontier_map[e]
    #                         self.found_temp_goal[e]=False
    #                     else:
    #                         if min_tuple[0][0]==self.tuple[0][0] :#这一步和上一步同一个桌子
    #                             area_current=torch.sum(map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]  * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :]))
    #                             if( self.area>area_current+5 and self.area!=-1)or self.count>30000:#已经到达了该容器                                                                     
    #                                 self.rec[min_tuple[0]]= map_features[e,min_tuple[0][0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  
    #                                 self.list.append(min_tuple[0])  
    #                                 self.count=0       
    #                                 goal_map[e] = frontier_map[e]        
    #                                 self.found_temp_goal[e]=False
    #                                 print("到达当前临时容器，但没有目标容器，走边界")        
    #                             else:#没到达
    #                                 print("靠近当前临时容器")
    #                                 self.area=area_current
    #                                 goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
    #                                 self.found_temp_goal[e]=True
    #                                 self.count+=1
    #                         else: #不是同一个容器，另一个容器更优

    #                             self.area=-1      
    #                             goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
    #                             self.found_temp_goal[e]=True
    #                             print("向另一个更优的临时容器靠近",min_tuple[0][0],min_tuple[1])            
                                
                                
    #                         self.tuple=min_tuple
    #                 else:
    #                         print("当前范围内容器如目标容器无关，走边界")
    #                         goal_map[e] = frontier_map[e]
    #                         self.found_temp_goal[e]=False

                    
    #             idx=0                                
    #             for i in self.list:                  
    #                 if (torch.sum(self.rec[i])+50<torch.sum(map_features[e,i[0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  ) ):                            
    #                     del self.list[idx]
    #                 idx+=1
    #     return goal_map ,self.found_temp_goal

    # def explore_otherwise(self, map_features, goal_map, found_goal,recep_category):
    #     """Explore closest unexplored region otherwise."""
        
    #     frontier_map = self.get_frontier_map(map_features)
    #     batch_size = map_features.shape[0] 
    #     for e in range(batch_size):
    #         flag=False
    #         if not found_goal[e]: #没发现目标容器               
    #             rec_nonzero=np.nonzero(torch.sum(map_features[e,12:33,:,:],axis=(-2,-1))>5)   #0-20  21类容器
               
    #             aa=len(rec_nonzero) 

    #             if recep_category[e].cpu()-2 in rec_nonzero:
    #                  b=1
    #             else:
    #                  b=0
                     
    #             #import pdb;pdb.set_trace()      
    #             if (aa==0 or  len(self.list)==aa-b ) :      #不存在容器，走边界
                  
    #                 print("容器都靠近完，但是没发现目标")        
    #                 goal_map[e] = frontier_map[e]                 
    #             else:
    #                 shortest={}                   
    #                 for i in range (aa):
    #                     list1=[temp[0] for temp in self.list]
    #                     if(rec_nonzero[i][0]!= recep_category[e].cpu()-2 and (not rec_nonzero[i][0]+1 in list1) ):                               #txt从1开始到21是对应容器(前面是misc)  代码从2开始是容器（前两个是obj与misc)
    #                         shortest_distance1, shortest_path1 = self.dijkstra_with_path(rec_nonzero[i][0]+1, recep_category[e]-1)#最短路径长度   从开始到最后走过的容器
    #                         shortest[shortest_path1]=shortest_distance1 #比正常的容器大1
                          
    #                 min_tuple = min(shortest.items(), key=lambda x: x[1]) #选出n条路径中距离最短的,排除了已经到达的
                
    #                 #import pdb;pdb.set_trace()
    #                 if min_tuple[1]>self.tuple[1] : #其他容器不如当前容器优
    #                     print("容器",min_tuple[0][0],",,,,",min_tuple[1],"不如",self.tuple[0][0],",,,",self.tuple[1],"路径短，所以走边界")
    #                     goal_map[e] = frontier_map[e]
    #                     self.found_temp_goal[e]=False
    #                 else:    #1、其他容器优   2、 当前容器到达目标点  3、当前容器没有达到目标点                  
    #                     if min_tuple[0][0]==self.tuple[0][0] :#这一步和上一步同一个桌子
    #                         area_current=torch.sum(map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]  * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :]))
    #                         if( self.area>area_current+5 and self.area!=-1)or self.count>30000:#已经到达了该容器                                                                     
    #                             self.rec[min_tuple[0]]= map_features[e,min_tuple[0][0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  
    #                             self.list.append(min_tuple[0])  
    #                             self.count=0       
    #                             goal_map[e] = frontier_map[e]        
    #                             self.found_temp_goal[e]=False
    #                             print("到达临时目标点，走边界")        
    #                         else:#没到达
    #                             self.area=area_current
    #                             goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
    #                             self.found_temp_goal[e]=True
    #                             self.count+=1
    #                     else: #不是同一个容器，另一个容器更优

    #                         self.area=-1      
    #                         goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
    #                         self.found_temp_goal[e]=True
    #                         #plt.imshow(goal_map[e].cpu())
    #                         #plt.savefig("/home/ubuntu/zyp/home-robot/datadump/aa.png")
    #                         # print("ccccccccc",goal_map.sum())
    #                         #import pdb;pdb.set_trace()
                            
    #                         print("向临时目标点靠近",min_tuple[0][0],min_tuple[1])            
                               
                             
    #                     self.tuple=min_tuple
                    
    #             idx=0                                
    #             for i in self.list:                  
    #                 if (torch.sum(self.rec[i])+50<torch.sum(map_features[e,i[0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  ) ):                            
    #                     del self.list[idx]
    #                 idx+=1
    #     return goal_map ,self.found_temp_goal



                

# def explore_otherwise(self, map_features, goal_map, found_goal,recep_category):
#         """Explore closest unexplored region otherwise."""
        
#         frontier_map = self.get_frontier_map(map_features)
#         batch_size = map_features.shape[0] 
#         for e in range(batch_size):
#             flag=False
#             if not found_goal[e]: #没发现目标容器               
#                 rec_nonzero=np.nonzero(torch.sum(map_features[e,12:33,:,:],axis=(-2,-1)))   #0-20  21类容器
#                 aa=len(rec_nonzero) 

#                 if recep_category[e].cpu()-2 in rec_nonzero:
#                      b=1
#                 else:
#                      b=0
                     
#                 print("ddddddddddddddddddd",self.list)           
#                 if aa==0 or  len(self.list)==aa-b:      #不存在容器，走边界
                  
#                     print("容器都靠近完，但是没发现目标")        
#                     goal_map[e] = frontier_map[e]                 
#                 else:
#                     shortest={}                   
#                     for i in range (aa):
#                         list1=[temp[0] for temp in self.list]
#                         if(rec_nonzero[i][0]!= recep_category[e].cpu()-2 and (not rec_nonzero[i][0]+1 in list1) ):                               #txt从1开始到21是对应容器(前面是misc)  代码从2开始是容器（前两个是obj与misc)
#                             shortest_distance1, shortest_path1 = self.dijkstra_with_path(rec_nonzero[i][0]+1, recep_category[e]-1)#最短路径长度   从开始到最后走过的容器
#                             shortest[shortest_path1]=shortest_distance1 #比正常的容器大1
                          
#                     min_tuple = min(shortest.items(), key=lambda x: x[1]) #选出n条路径中距离最短的,排除了已经到达的
#                     #import pdb;pdb.set_trace()
#                     if min_tuple[0][0]==self.index:#这一步和上一步同一个桌子
#                         area_current=torch.sum(map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]  * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :]))
#                         if( self.area>area_current and self.area!=-1)or self.count>30:#已经到达了桌子一号，＋5是为了防止细微变化 
#                             #import pdb;pdb.set_trace()                           
#                             flag=True                         
#                             self.rec[min_tuple[0]]= map_features[e,min_tuple[0][0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  
#                             self.list.append(min_tuple[0])  
#                             self.count=0                    
#                         else:
#                             self.area=area_current
#                             self.count+=1
#                     else:
#                         self.area=-1
#                     self.index=min_tuple[0][0] 
#                     if (flag==True) :#当前视野范围内容器和目标容器关联性不大
#                         goal_map[e] = frontier_map[e]
#                     else:             
#                         goal_map[e]=map_features[e,2 * MC.NON_SEM_CHANNELS+min_tuple[0][0]+1,:,:]#最短路径的第一个索引
#                 idx=0                                
#                 for i in self.list:
#                     #import pdb;pdb.set_trace()
#                     if (torch.sum(self.rec[i])+50<torch.sum(map_features[e,i[0]+1+ 2 * MC.NON_SEM_CHANNELS, :, :]  ) ):                            
#                         del self.list[idx]
#                     idx+=1
#         return goal_map



                





