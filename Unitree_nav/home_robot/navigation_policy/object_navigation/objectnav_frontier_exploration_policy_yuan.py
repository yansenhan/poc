import numpy as np
import scipy
import skimage.morphology
import torch
import torch.nn as nn
from sklearn.cluster import DBSCAN

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
bNewRec = False


No_ = 10000


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

        self.dilate_explored_kernel = nn.Parameter(
            torch.from_numpy(skimage.morphology.disk(10)).unsqueeze(0).unsqueeze(0).float(), requires_grad=False, )
        self.select_border_kernel = nn.Parameter(
            torch.from_numpy(skimage.morphology.disk(1)).unsqueeze(0).unsqueeze(0).float(), requires_grad=False, )

        # 记录每次的生效的连通域的面积 长 宽和出现次数，
        # self.record2 = [[1, 2, 3, 4], ]
        self.rec_record = {}
        self.No_ = 10000

    @property
    def goal_update_steps(self):
        return 1

    def reach_single_category(self, map_features, category):
        # if the goal is found, reach it
        goal_map, found_goal = self.reach_goal_if_in_map(map_features, category)
        # otherwise, do frontier exploration
        goal_map = self.explore_otherwise(map_features, goal_map, found_goal)
        return goal_map, found_goal

    # 组合
    def reach_object_recep_combination(self, map_features, object_category, recep_category):
        #import pdb;pdb.set_trace()
      
        goal_map, found_goal = self.reach_goal_if_in_map(map_features, recep_category, #容器和物体
                                                         small_goal_category=object_category, )
        if bNewFrontier:
            goal_map = self.explore_otherwise(map_features, goal_map, found_goal)
            batch_size, _, height, width = map_features.shape
            device = map_features.device
            found_rec_r = torch.zeros(batch_size, dtype=torch.bool, device=device)
            if bShowGoal:
                if found_goal[0] == True:
                    print("目标物体")
                else:
                    print("边界")
            return goal_map, found_goal, found_rec_r
        #import pdb;pdb.set_trace()
        if bNewRec:
            goal_map, found_rec_goal = self.reach_goal_if_in_map_rec(map_features, recep_category,
                                                                     goal_map=goal_map, found_goal=found_goal, )
            goal_map = self.explore_otherwise(map_features, goal_map, found_rec_goal)

            if bShowGoal:
                if found_goal[0] == True:
                    print("目标物体")
                    #import pdb;pdb.set_trace()
                else:
                    if found_rec_goal[0] == True:
                        print("起始容器")
                     #   import pdb;pdb.set_trace()
                    else:
                        print("边界")
            # import matplotlib.pyplot as plt
            # plt.imshow(goal_map[0].cpu())
            # plt.savefig("aaa")
            return goal_map, found_goal, found_rec_goal

        goal_map, found_rec_goal = self.reach_goal_if_in_map(map_features, recep_category,
                                                             reject_visited_regions=True,
                                                             goal_map=goal_map, found_goal=found_goal, )
        goal_map = self.explore_otherwise(map_features, goal_map, found_rec_goal)
        if bShowGoal:
            if found_goal[0] == True:
                print("目标物体")
            else:
                if found_rec_goal[0] == True:
                    print("起始容器")
                else:
                    print("边界")
        # import matplotlib.pyplot as plt
        # plt.imshow(goal_map[0].cpu())
        # plt.savefig("/nanhu-src/hi/aaa.png")
       # import pdb;pdb.set_trace()
        return goal_map, found_goal, found_rec_goal

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
                # 如果当前 还没有找到目标

                category_map = map_features[e, goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :]

                if reject_visited_regions:
                   # import pdb;pdb.set_trace()
                    # remove the receptacles that the already been close to
                    category_map = category_map * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :]) #剩下没有接近过的容器
               
                if small_goal_category is not None:
                    # additionally check if the category has the required small object on it

                    category_map = (
                            category_map * map_features[e, small_goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :])

                    # category_map = map_features[e, small_goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :]

                if (category_map == 1).sum() > 0:
                    goal_map[e] = category_map == 1
                    found_goal_current[e] = True

        return goal_map, found_goal_current

    def deleteContour(self, rec_map_all_cpu, rec_map_part_cpu):
        # 1 起始容器 最小生效面积
        minArea_ = 20
        # 2 起始容器已经有0.7 看过了，剩下的不再看
        maxClearRatio_ = 0.7
        # 没探索过的 探索过的允许停留次数
        maxUnseen_Seen = [400, 15]

        rec_map_all_cpu = np.where(rec_map_all_cpu == 1, 1, 0)
        rec_map_part_cpu = np.where(rec_map_part_cpu == 1, 1, 0)

        rec_map_all_cpu = rec_map_all_cpu.astype(np.uint8)
        rec_map_part_cpu = rec_map_part_cpu.astype(np.uint8)

        # 一、去除小面积
        opencv_rec = cv2.merge([rec_map_part_cpu])
        num_labels, labels = cv2.connectedComponents(opencv_rec, connectivity=8)
        for i in range(num_labels):
            if i > 0:
                count = np.sum(labels == i)
                if bShow:
                    print(i, count)
                if count < minArea_:
                    labels = np.where(labels == i, 0, labels)
        labels = np.where(labels > 0, 1, 0)

        # 二、去除处于边界的
        labels1 = labels.astype(np.uint8)
        opencv_rec = cv2.merge([labels1])
        num_labels1, labels1 = cv2.connectedComponents(opencv_rec, connectivity=8)
        for i in range(num_labels1):
            if i > 0:
                labels_ = np.where(labels1 == i, 1, 0)
                nonzero_indices = np.where(labels_ != 0)
                # 找到非零元素的横坐标的最大值和最小值
                min_x = np.min(nonzero_indices[1])
                max_x = np.max(nonzero_indices[1])
                min_y = np.min(nonzero_indices[0])
                max_y = np.max(nonzero_indices[0])
                if min_x == 0 or min_y == 0 or max_x == 479 or max_y == 479:
                    labels = np.where(labels_ == 1, 0, labels)

        # 三、 如果剩余占比原来的连通域的 不到30% 同样删去,
        # 说明该容器 已经大部分看过了，剩余一部分因为没清除到，滞留下来
        opencv_rec = cv2.merge([rec_map_all_cpu])
        num_labels2, labels2 = cv2.connectedComponents(opencv_rec, connectivity=8)
        for i in range(num_labels2):
            if i > 0:
                labels_ = np.where(labels2 == i, 1, 0)
                AreaOld = np.sum(labels_ == 1)
                labels_2 = np.where(labels == 1, 0, labels_)
                AreaSeen = np.sum(labels_2 == 1)
                leftRatio = AreaSeen * 1.0 / AreaOld
                if bShow:
                    print(i, AreaSeen, AreaOld, leftRatio)
                if leftRatio > maxClearRatio_:
                    labels = np.where(labels_ == 1, 0, labels)

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
                    labels_2 = np.where(labels == 1, 0, labels_)
                    AreaSeen = np.sum(labels_2 == 1)
                    # 说明原始的 起始容器一点都没有被清除过，需要保留
                    # 在 label_tag 中标记处没有探索过的大连通域
                    if AreaSeen == 0:
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
                    key_ = (area, height, width)

                    # 先确定键值，面积,高,宽
                    # 需要判断当前连通域是不是完整的连通域
                    # label_tag 保存了原始的起始容器中 完整保留下来的连通域

                    # labels_如果 labels_ 和label_tag 有交集，说明当前的labels_连通域是 没被探索过的
                    # labels_如果在label_tag上全是0，说明该区域是被探索过的，母连通域在labels_tag中不存在

                    labels_count = np.where(labels_ == 1, label_tag, 0)
                    Area_ = np.sum(labels_count == 1)
                    # 有交集，说明当前连通域还没探索过
                    if Area_ > 0:
                        index_ = 0
                    else:
                        index_ = 1

                    # 存在直接加1，判定
                    if key_ in self.rec_record:
                        found = True
                        self.rec_record[key_] = self.rec_record[key_] + 1
                        if self.rec_record[key_] > maxUnseen_Seen[index_]:
                            labels = np.where(labels_ == 1, 0, labels)

                    # 不存在 需要遍历进行比较
                    else:
                        for key in self.rec_record:
                            if key[0] > 100 and area - 10 < key[0] < area + 10 and height - 3 < \
                                    key[1] < height + 3 and width - 3 < key[2] < width + 3:
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

        for e in range(batch_size):
            if not found_goal_current[e]:
                # import pdb;pdb.set_trace()
                category_map = map_features[e, goal_category[e] + 2 * MC.NON_SEM_CHANNELS, :, :]
                rec_map_all = category_map
                category_map = category_map * (1 - map_features[e, MC.BEEN_CLOSE_MAP, :, :])
                rec_map_part = category_map
                # import matplotlib.pyplot as plt
                # plt.imshow(map_features[e, MC.BEEN_CLOSE_MAP, :, :].cpu())
                # plt.savefig("ddd")
                rec_map_all_cpu = rec_map_all.cpu().numpy()
                rec_map_part_cpu = rec_map_part.cpu().numpy()

                clear_map_cpu_new = self.deleteContour(rec_map_all_cpu, rec_map_part_cpu)
                category_map = torch.tensor(clear_map_cpu_new).cuda()

                if (category_map == 1).sum() > 0:
                    goal_map[e] = category_map == 1
                    found_goal_current[e] = True

        return goal_map, found_goal_current

    def get_frontier_map(self, map_features):
        # Select unexplored area
        # 怎么定义地图边界，或者说已知视野边界
        if self.exploration_strategy == "seen_frontier":
            frontier_map = (map_features[:, [MC.EXPLORED_MAP], :, :] == 0).float()
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

    def explore_otherwise(self, map_features, goal_map, found_goal):
        """Explore closest unexplored region otherwise."""
        frontier_map = self.get_frontier_map(map_features)
        batch_size = map_features.shape[0]
        for e in range(batch_size):
            if not found_goal[e]:
                goal_map[e] = frontier_map[e]

        return goal_map
