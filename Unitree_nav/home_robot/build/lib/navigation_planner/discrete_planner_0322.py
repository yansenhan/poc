# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import math
import os
import shutil
import time
from typing import List, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
import skimage.morphology

import home_robot.utils.pose as pu
from home_robot.core.interfaces import (
    ContinuousNavigationAction,
    DiscreteNavigationAction,
)
from home_robot.utils.geometry import xyt_global_to_base

from .fmm_planner import FMMPlanner
from scipy import ndimage

CM_TO_METERS = 0.01


def has_majority_duplicate(arr):
    flattened_arr = np.array(arr).flatten()
    unique_elements, counts = np.unique(flattened_arr, return_counts=True)
    max_count = np.max(counts)
    return max_count > len(flattened_arr) / 2


def add_boundary(mat: np.ndarray, value=1) -> np.ndarray:
    h, w = mat.shape
    new_mat = np.zeros((h + 2, w + 2)) + value
    new_mat[1: h + 1, 1: w + 1] = mat
    return new_mat


def remove_boundary(mat: np.ndarray, value=1) -> np.ndarray:
    return mat[value:-value, value:-value]


class DiscretePlanner:
    """
    This class translates planner inputs into a discrete low-level action
    using an FMM planner.

    This is a wrapper used to navigate to a particular object/goal location.
    """

    def __init__(
            self,
            turn_angle: float,
            collision_threshold: float,
            step_size: int,
            obs_dilation_selem_radius: int,
            goal_dilation_selem_radius: int,
            map_size_cm: int,
            map_resolution: int,
            visualize: bool,
            print_images: bool,
            dump_location: str,
            exp_name: str,
            min_goal_distance_cm: float = 50.0,
            min_obs_dilation_selem_radius: int = 1,
            agent_cell_radius: int = 1,
            map_downsample_factor: float = 1.0,
            map_update_frequency: int = 1,
            goal_tolerance: float = 0.01,
            discrete_actions: bool = True,
            continuous_angle_tolerance: float = 30.0,
    ):

        self.discrete_actions = discrete_actions
        self.visualize = visualize
        self.print_images = print_images
        self.default_vis_dir = f"{dump_location}/images/{exp_name}"
        os.makedirs(self.default_vis_dir, exist_ok=True)

        self.map_size_cm = map_size_cm
        self.map_resolution = map_resolution
        self.map_shape = (
            self.map_size_cm // self.map_resolution,
            self.map_size_cm // self.map_resolution,
        )
        self.turn_angle = turn_angle
        self.collision_threshold = collision_threshold
        self.step_size = step_size
        self.start_obs_dilation_selem_radius = obs_dilation_selem_radius
        self.goal_dilation_selem_radius = goal_dilation_selem_radius
        self.min_obs_dilation_selem_radius = min_obs_dilation_selem_radius
        self.agent_cell_radius = agent_cell_radius
        self.goal_tolerance = goal_tolerance
        self.continuous_angle_tolerance = continuous_angle_tolerance

        self.vis_dir = None
        self.collision_map = None
        self.visited_map = None
        self.col_width = None
        self.last_pose = None
        self.curr_pose = None
        self.last_action = None
        self.timestep = 0
        self.curr_obs_dilation_selem_radius = None
        self.obs_dilation_selem = None
        self.obs_dilation_selem_collis = None
        self.min_goal_distance_cm = min_goal_distance_cm
        self.dd = None

        self.map_downsample_factor = map_downsample_factor
        self.map_update_frequency = map_update_frequency
        self.count1 = 0

        self.collision_mask_list = []
        self.collision_mask_list480 = []

        self.last_stg = None
        self.last_stg480 = None

        self.pt_dict = {}
        self.pos_dict = {}

        self.delete_map = None
        self.obs_dilation_selem_delete = None

        # 表示当前是边界还是容器
        self.bFontier = False

    def reset(self):
        self.collision_mask_list = []
        self.collision_mask_list480 = []
        self.last_stg = None
        self.last_stg480 = None
        self.bFontier = False
        self.pt_dict.clear()
        self.pos_dict.clear()

        self.vis_dir = self.default_vis_dir
        self.collision_map = np.zeros(self.map_shape)
        self.visited_map = np.zeros(self.map_shape)
        self.delete_map = np.zeros(self.map_shape)
        self.col_width = 1
        self.last_pose = None
        self.curr_pose = [
            self.map_size_cm / 100.0 / 2.0,
            self.map_size_cm / 100.0 / 2.0,
            0.0,
        ]
        self.last_action = None
        self.timestep = 1
        self.curr_obs_dilation_selem_radius = self.start_obs_dilation_selem_radius
        self.obs_dilation_selem = skimage.morphology.disk(self.curr_obs_dilation_selem_radius)
        self.goal_dilation_selem = skimage.morphology.disk(self.goal_dilation_selem_radius)
        self.obs_dilation_selem_collis = skimage.morphology.disk(1)
        self.obs_dilation_selem_delete = skimage.morphology.disk(20)

    def set_vis_dir(self, scene_id: str, episode_id: str):
        self.vis_dir = os.path.join(self.default_vis_dir, f"{scene_id}_{episode_id}")
        shutil.rmtree(self.vis_dir, ignore_errors=True)
        os.makedirs(self.vis_dir, exist_ok=True)

    def disable_print_images(self):
        self.print_images = False

    def get_action(
            self,
            relative_stg_x: float,
            relative_stg_y: float,
            relative_angle_to_stg: float,
            relative_angle_to_closest_goal: float,
            start_compass: float,
            found_goal: bool,
            stop: bool,
            debug: bool,
            fmm_dist: float,
            step_size: float,
    ):
        """
        Gets discrete/continuous action given short-term goal. Agent orients to closest goal if found_goal=True and stop=True
        """
        # Short-term goal -> deterministic local policy
        if not (found_goal and stop):
            if self.discrete_actions:
                if relative_angle_to_stg > self.turn_angle / 2.0:
                    action = DiscreteNavigationAction.TURN_RIGHT
                elif relative_angle_to_stg < -self.turn_angle / 2.0:
                    action = DiscreteNavigationAction.TURN_LEFT
                else:
                    action = DiscreteNavigationAction.MOVE_FORWARD
            else:
                # Use the short-term goal to set where we should be heading next
                m_relative_stg_x, m_relative_stg_y = [
                    CM_TO_METERS * self.map_resolution * d
                    for d in [relative_stg_x, relative_stg_y]
                ]
                if np.abs(relative_angle_to_stg) > self.turn_angle / 2.0:
                    # Must return commands in radians and meters
                    relative_angle_to_stg = math.radians(relative_angle_to_stg)
                    action = ContinuousNavigationAction([0, 0, -relative_angle_to_stg])
                else:
                    # Must return commands in radians and meters
                    relative_angle_to_stg = math.radians(relative_angle_to_stg)
                    xyt_global = [
                        m_relative_stg_y,
                        m_relative_stg_x,
                        -relative_angle_to_stg,
                    ]

                    xyt_local = xyt_global_to_base(
                        xyt_global, [0, 0, math.radians(start_compass)]
                    )
                    xyt_local[
                        2
                    ] = (
                        -relative_angle_to_stg
                    )  # the original angle was already in base frame
                    action = ContinuousNavigationAction(xyt_local)
        else:
            # Try to orient towards the goal object - or at least any point sampled from the goal
            # object.

            if self.discrete_actions:
                if relative_angle_to_closest_goal > 2 * self.turn_angle / 3.0:
                    action = DiscreteNavigationAction.TURN_RIGHT
                elif relative_angle_to_closest_goal < -2 * self.turn_angle / 3.0:
                    action = DiscreteNavigationAction.TURN_LEFT
                else:
                    # import pdb;pdb.set_trace()
                    action = DiscreteNavigationAction.STOP
            elif (
                    np.abs(relative_angle_to_closest_goal) > self.continuous_angle_tolerance
            ):

                relative_angle_to_closest_goal = math.radians(
                    relative_angle_to_closest_goal
                )
                action = ContinuousNavigationAction(
                    [0, 0, -relative_angle_to_closest_goal]
                )
            else:
                action = DiscreteNavigationAction.STOP
                if debug:
                    print("!!! DONE !!!")
        self.last_action = action
        return action

    def _get_short_term_goal(
            self,
            obstacle_map: np.ndarray,
            goal_map: np.ndarray,
            start: List[int],
            planning_window: List[int],
            nav_to_recep: float,
            found_goal: bool,
            found_rec: bool,
            plan_to_dilated_goal=False,
            frontier_map=None,
            visualize=False,
    ) -> Tuple[Tuple[int, int], np.ndarray, bool, bool]:

        gx1, gx2, gy1, gy2 = planning_window

        bNoRad = False  # 一个表示可导航到的区域 使用的膨胀半径，如果是容器或者边界的时候，不能直接退出，需要进行清空。如果是目标，直接退出
        bExit = False  # 一个是是否目标图被清空，需要直接退出
        if obstacle_map.shape[0] != 480 or obstacle_map.shape[1] != 480:
            # print("图像不是480 480 的例子！！")
            # import pdb;pdb.set_trace()
            bExit = True
            return (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, bNoRad, bExit)

        # 疑问，这边恒定 0 480 0 480
        obstacles = obstacle_map[0:480, 0:480]

        dilated_obstacles = cv2.dilate(obstacles, self.obs_dilation_selem, iterations=1)
        traversible = 1 - dilated_obstacles
        traversible[self.collision_map[gx1:gx2, gy1:gy2][0:480, 0:480] == 1] = 0

        # 对新增的永久障碍膨胀一次
        colli2 = self.collision_map[gx1:gx2, gy1:gy2][0:480, 0:480]
        colli3 = cv2.dilate(colli2, self.obs_dilation_selem_collis, iterations=1)
        traversible[colli3 == 1] = 0
        traversible[self.visited_map[gx1:gx2, gy1:gy2][0:480, 0:480] == 1] = 1

        # 把机器人视作圆柱，所以自身占地处也可以走，实际上不一定。
        agent_rad = self.agent_cell_radius
        traversible[int(start[0]) - agent_rad: int(start[0]) + agent_rad + 1,
        int(start[1]) - agent_rad: int(start[1]) + agent_rad + 1, ] = 1

        # 清除的逻辑
        # 如果是第一阶段的目标物体，不进行操作
        # 如果是第一阶段的容器或者第三阶段的容器，需要进行清除，清除后，没有，就切到边界
        # 剩余的情况，如果是边界，清除，直到边界为空，需要退出

        b3Frontier = False
        if nav_to_recep == 1 or found_goal == False:
            delete_map_ = self.delete_map[gx1:gx2, gy1:gy2][0:480, 0:480]
            if np.sum(delete_map_ == 1) > 0:
                # self.bFontier = np.array_equal(goal_map, frontier_map)
                # if self.bFontier == False:#容器 或目标
                if found_goal == True or found_rec == True:
                    goal_map = np.where(delete_map_ == 1, 0, goal_map)
                    # 如果是容器，不管是第一还是第三阶段，容器被清空，就切到边界
                    if np.sum(goal_map == 1) == 0:
                        b3Frontier = True
                        goal_map = frontier_map
                        goal_map = np.where(delete_map_ == 1, 0, goal_map)
                        if np.sum(goal_map == 1) == 0:
                            # import pdb;pdb.set_trace()
                            bExit = True
                            return (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, bNoRad, bExit)
                else:  # 边界
                    goal_map = np.where(delete_map_ == 1, 0, goal_map)
                    # 边界被清空就退出
                    if np.sum(goal_map == 1) == 0:
                        # import pdb;pdb.set_trace()
                        bExit = True
                        return (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, bNoRad, bExit)

       
        # 添加圈  所以后面 相对窗口中的坐标都 + 1
        traversible = add_boundary(traversible)
        goal_map = add_boundary(goal_map, value=0)
        # import pdb;pdb.set_trace()
        planner = FMMPlanner(
            traversible,
            step_size=self.step_size,
            vis_dir=self.vis_dir,
            visualize=self.visualize,
            print_images=self.print_images,
            goal_tolerance=self.goal_tolerance,
        )
        if nav_to_recep == 0:  # 第一阶段
            if not found_goal:  # 边界 容器
                go_tolerance = 0.01
                self.min_goal_distance_cm = 50
                navigable_goal_map, _ = planner._find_within_distance_to_multi_goal(
                    goal_map,
                    self.min_goal_distance_cm / self.map_resolution,
                    timestep=self.timestep,
                    vis_dir=self.vis_dir,
                )
                if not np.any(navigable_goal_map):  # 非边界时                  
                    frontier_map = add_boundary(frontier_map, value=0)
                    navigable_goal_map = frontier_map
                ##############################################改###################################
                self.dd = planner.set_multi_goal(
                    navigable_goal_map,
                    self.timestep,
                    self.dd,
                    self.map_downsample_factor,
                    self.map_update_frequency,
                )

                goal_distance_map, closest_goal_pt = self.get_closest_goal(goal_map, start)

                self.timestep += 1
                state = [start[0] + 1, start[1] + 1]
                stg_x, stg_y, liantong, replan, stop, step_size, xx, yy = planner.get_short_term_goal(state,
                                                                                                    self.collision_mask_list,
                                                                                                    go_tolerance)
                 ##############################################改###################################
            
            else:  # 目标点
                # import pdb;pdb.set_trace()
                go_tolerance = 3
                self.min_goal_distance_cm = 20
                while self.min_goal_distance_cm <= 50:
                    self.min_goal_distance_cm += 5
                    navigable_goal_map, _ = planner._find_within_distance_to_multi_goal(
                        goal_map,
                        self.min_goal_distance_cm / self.map_resolution,
                        timestep=self.timestep,
                        vis_dir=self.vis_dir,
                    )
                    if np.sum(navigable_goal_map == 1) < 1:
                        continue
                    self.dd = planner.set_multi_goal(
                        navigable_goal_map,
                        self.timestep,
                        self.dd,
                        self.map_downsample_factor,
                        self.map_update_frequency,
                    )
                    if has_majority_duplicate(self.dd) == False:
                        break

                if self.min_goal_distance_cm > 50:  # 非边界时
                    # import pdb;pdb.set_trace()
                    frontier_map = add_boundary(frontier_map, value=0)
                    navigable_goal_map = frontier_map

                self.dd = planner.set_multi_goal(
                    navigable_goal_map,
                    self.timestep,
                    self.dd,
                    self.map_downsample_factor,
                    self.map_update_frequency,
                )

                goal_distance_map, closest_goal_pt = self.get_closest_goal(goal_map, start)

                self.timestep += 1
                state = [start[0] + 1, start[1] + 1]

                stg_x, stg_y, liantong, replan, stop, step_size, xx, yy = planner.get_short_term_goal(state,
                                                                                                    self.collision_mask_list,
                                                                                                    go_tolerance)


        else:  # 第三阶段
            if found_goal == True:  # 目标点####################
                dilated_goal_map = cv2.dilate(goal_map, self.obs_dilation_selem, iterations=1)  # 更近些
                navigable_goal_map = dilated_goal_map
                go_tolerance = 5

            else:
                go_tolerance = 0.01
                self.min_goal_distance_cm = 50
                navigable_goal_map, _ = planner._find_within_distance_to_multi_goal(
                    goal_map,
                    self.min_goal_distance_cm / self.map_resolution,
                    timestep=self.timestep,
                    vis_dir=self.vis_dir,
                )

            self.dd = planner.set_multi_goal(
                navigable_goal_map,
                self.timestep,
                self.dd,
                self.map_downsample_factor,
                self.map_update_frequency,
            )

            goal_distance_map, closest_goal_pt = self.get_closest_goal(goal_map, start)

            self.timestep += 1
            state = [start[0] + 1, start[1] + 1]

            stg_x, stg_y, liantong, replan, stop, step_size, xx, yy = planner.get_short_term_goal(state,
                                                                                                self.collision_mask_list,
                                                                                                go_tolerance)

        stg_x, stg_y = stg_x - 1, stg_y - 1
        short_term_goal = int(stg_x), int(stg_y)
        # print("replan", replan, "stop", stop, self.min_goal_distance_cm)

        return (
            short_term_goal,
            goal_distance_map,
            liantong,
            replan,
            stop,
            closest_goal_pt,
            dilated_obstacles,
            planner.fmm_dist[state[0], state[1]],
            step_size,
            xx,
            yy,
            bNoRad,
            bExit
        )

    def get_closest_goal(self, goal_map, start):
        """closest goal, avoiding any obstacles."""
        empty = np.ones_like(goal_map)
        empty_planner = FMMPlanner(empty)
        empty_planner.set_goal(start)
        dist_map = empty_planner.fmm_dist * goal_map
        dist_map[dist_map == 0] = 10000
        closest_goal_map = dist_map == dist_map.min()
        closest_goal_map = remove_boundary(closest_goal_map)
        closest_goal_pt = np.unravel_index(closest_goal_map.argmax(), closest_goal_map.shape)
        return closest_goal_map, closest_goal_pt

    def _check_collision(self, frontier_map, gx1, gx2, gy1, gy2):
        """Check whether we had a collision and update the collision map."""
        x1, y1, t1 = self.last_pose
        x2, y2, _ = self.curr_pose

        dist = pu.get_l2_distance(x1, x2, y1, y2)
        # if dist < self.collision_threshold:
        if dist == 0:
            self.collision_mask_list.append(self.last_stg)
            self.collision_mask_list480.append(self.last_stg480)
        # import pdb;pdb.set_trace()

        else:

            # 清空所有屏蔽点，因为发生了移动
            self.collision_mask_list.clear()
            self.collision_mask_list480.clear()

    def deleteGoal(self, closest_goal_pt, gx1, gx2, gy1, gy2):
        delete_ = np.zeros((480, 480))
        delete_[closest_goal_pt[0], closest_goal_pt[1]] = 1
        dilated_delete_ = cv2.dilate(delete_, self.obs_dilation_selem_delete, iterations=1)
        self.delete_map[gx1:gx2, gy1:gy2] = self.delete_map[gx1:gx2, gy1:gy2] + dilated_delete_
        self.delete_map[gx1:gx2, gy1:gy2] = np.where(self.delete_map[gx1:gx2, gy1:gy2] > 0, 1, 0)

    def plan(
            self,
            obstacle_map: np.ndarray,
            goal_map: np.ndarray,
            frontier_map: np.ndarray,
            sensor_pose: np.ndarray,
            found_goal: bool,
            found_rec: bool,
            nav_to_recep: float,
            debug: bool = True,
            use_dilation_for_stg: bool = False,
            timestep: int = None,
    ) -> Tuple[DiscreteNavigationAction, np.ndarray]:

        if nav_to_recep == 1 and found_goal == True:
            self.curr_obs_dilation_selem_radius = 1
            self.obs_dilation_selem = skimage.morphology.disk(self.curr_obs_dilation_selem_radius)

        # 用于记录反复的移动数据，一般只需要最近的两组数据
        if len(self.pos_dict) > 5:
            self.pos_dict.clear()

        if timestep is not None:
            self.timestep = timestep

        self.last_pose = self.curr_pose
        obstacle_map = np.rint(obstacle_map)

        start_x, start_y, start_o, gx1, gx2, gy1, gy2 = sensor_pose
        gx1, gx2, gy1, gy2 = int(gx1), int(gx2), int(gy1), int(gy2)
        planning_window = [gx1, gx2, gy1, gy2]

        start = [
            int(start_y * 100.0 / self.map_resolution - gx1),
            int(start_x * 100.0 / self.map_resolution - gy1),
        ]
        start = pu.threshold_poses(start, obstacle_map.shape)
        start = np.array(start)
        self.curr_pose = [start_x, start_y, start_o]

        self.visited_map[gx1:gx2, gy1:gy2][start[0] - 0: start[0] + 1, start[1] - 0: start[1] + 1] = 1

        # Check collisions if we have just moved and are uncertain
        if self.last_action == DiscreteNavigationAction.MOVE_FORWARD or \
                (
                        type(self.last_action) == ContinuousNavigationAction and np.linalg.norm(
                    self.last_action.xyt[:2]) > 0):
            self._check_collision(goal_map, gx1, gx2, gy1, gy2)

        try:
            (
                short_term_goal,
                closest_goal_map,
                liantong,
                replan,
                stop,
                closest_goal_pt,
                dilated_obstacles,
                fmm_dist,
                step_size,
                xx,
                yy,
                bNorad,
                bExit
            ) = self._get_short_term_goal(
                obstacle_map,
                np.copy(goal_map),
                start,
                planning_window,
                nav_to_recep,
                found_goal,
                found_rec,
                plan_to_dilated_goal=use_dilation_for_stg,
                frontier_map=frontier_map,
            )
        except Exception as e:
            print("Warning! Planner crashed with error:", e)
            return (
                DiscreteNavigationAction.STOP,
                np.zeros(goal_map.shape),
                (0, 0),
                np.zeros(goal_map.shape),
            )
        ############################################################无可走点############################################
        # 退出条件：其他目标图被清空，提前退出
        if bExit:
            # import pdb;pdb.set_trace()
            return (
                DiscreteNavigationAction.STOP,
                np.zeros(goal_map.shape),
                (0, 0),
                np.zeros(goal_map.shape),
            )
        ##########################################异常fmm_dist退出（不触发）#############################################################
        # 找可导航区域，使用的半径过大
        if bNorad == True:
            # 一阶段目标物体 退出

            if nav_to_recep == 0 and found_goal == True:
                # import pdb;pdb.set_trace()
                return (
                    DiscreteNavigationAction.STOP,
                    np.zeros(goal_map.shape),
                    (0, 0),
                    np.zeros(goal_map.shape),
                )
            # 其余情况 一三阶段 目标边界或容器 尝试清除目标点，进行新的移动
            self.deleteGoal(closest_goal_pt, gx1, gx2, gy1, gy2)
            self.last_action = DiscreteNavigationAction.TURN_LEFT
            return (
                DiscreteNavigationAction.TURN_LEFT,
                np.zeros(goal_map.shape),
                closest_goal_pt,
                np.zeros(goal_map.shape),
            )
        ###################################两个点来回走######################################################################
        # 发生移动时，再次检查，是否来回走，放在这边，是因为 清除要使用到closest_goal_pt 变量
        if self.last_action == DiscreteNavigationAction.MOVE_FORWARD or \
                (
                        type(self.last_action) == ContinuousNavigationAction and np.linalg.norm(
                    self.last_action.xyt[:2]) > 0):

            # 发生移动要进行记录，是否来回走
            curr_pose = [int(self.curr_pose[0] * 100.0), int(self.curr_pose[1] * 100.0)]
            last_pose = [int(self.last_pose[0] * 100.0), int(self.last_pose[1] * 100.0)]

            # 如果发生移动，进行记录
            if curr_pose[0] != last_pose[0] or curr_pose[1] != last_pose[1]:
                key_ = (curr_pose[0], curr_pose[1], last_pose[0], last_pose[1])
                # 如果存在
                if key_ in self.pos_dict:
                    self.pos_dict[key_] = self.pos_dict[key_] + 1
                    if self.pos_dict[key_] > 1:
                        key_2 = (last_pose[0], last_pose[1], curr_pose[0], curr_pose[1])
                        if key_2 in self.pos_dict:
                            if self.pos_dict[key_2] > 0:
                                # 同样，一阶段目标物体，比如穿墙的例子导致来回走，退出
                                if nav_to_recep == 0 and found_goal == True:
                                    # import pdb;pdb.set_trace()
                                    return (
                                        DiscreteNavigationAction.STOP,
                                        np.zeros(goal_map.shape),
                                        (0, 0),
                                        np.zeros(goal_map.shape),
                                    )
                                # 其他的情况，全部是 容器和边界，同样执行清除，希望通过换目标点发生移动，走出当前位置
                                self.deleteGoal(closest_goal_pt, gx1, gx2, gy1, gy2)
                                # 清目标点后，削减一次 反复移动的记录
                                self.pos_dict[key_] = self.pos_dict[key_] - 1
                                self.pos_dict[key_2] = self.pos_dict[key_2] - 1
                                self.last_action = DiscreteNavigationAction.TURN_LEFT
                                return (
                                    DiscreteNavigationAction.TURN_LEFT,
                                    np.zeros(goal_map.shape),
                                    closest_goal_pt,
                                    np.zeros(goal_map.shape),
                                )
                else:
                    self.pos_dict[key_] = 1
            ##########################################在某一地方停留太久，不动#######################################
            # 一三阶段的容器和边界
            # 额外记录 发生移动后新位置的坐标点，这个策略和屏蔽点的策略目标是一样，都是希望不要过多停留，执行上看谁先执行
            # 如果没有该策略，路线差不多，但步数上升
            key_ = (curr_pose[0], curr_pose[1])
            if key_ in self.pt_dict:
                self.pt_dict[key_] = self.pt_dict[key_] + 1
            else:
                self.pt_dict[key_] = 1
            # 10和mask的设计有关，当目标是容器或者边界时，在一个地方停留过久，进行清除，并削减5次记录
            if self.pt_dict[key_] > 10:
                if found_goal == False or nav_to_recep == 1:
                    self.deleteGoal(closest_goal_pt, gx1, gx2, gy1, gy2)
                    self.pt_dict[key_] = self.pt_dict[key_] - 5
                    self.last_action = DiscreteNavigationAction.TURN_LEFT
                    return (
                        DiscreteNavigationAction.TURN_LEFT,
                        np.zeros(goal_map.shape),
                        closest_goal_pt,
                        np.zeros(goal_map.shape),
                    )

        # 进入replan的唯一条件：选中的点是(5,5)，即计算结果找不到方向

        if replan and not stop:

            # 一三阶段的容器和边界，并且当前点是最小点，我希望直接触发减少膨胀，先换目标边界，直到我走出去，直到遍历没有，才会执行后面的减少膨胀
            # 该效果和pt_dict记停留数 差不多，但是在某些情况下更耗步数，这里不会清空当前的屏蔽点，因为位置没移动，切换目标点后，原先的屏蔽点仍然生效
            if (found_goal == False or nav_to_recep == 1) and liantong == True:
                self.deleteGoal(closest_goal_pt, gx1, gx2, gy1, gy2)
                self.last_aciton = DiscreteNavigationAction.TURN_LEFT
                return (
                    DiscreteNavigationAction.TURN_LEFT,
                    np.zeros(goal_map.shape),
                    closest_goal_pt,
                    np.zeros(goal_map.shape),
                )

            # 半径缩小到1，(5,5)已经不联通了。目标边界和容器 一般不会出现这个情况，因为触发(5,5)优先清目标，最后会以目标图为空退出，
            # 这个一般是目标物体，说明当前已经不能到达目标物体
            if self.curr_obs_dilation_selem_radius == self.min_obs_dilation_selem_radius and liantong == False:
                # 目标物体
                if nav_to_recep == 0 and found_goal == True:
                    # import pdb;pdb.set_trace()
                    return (
                        DiscreteNavigationAction.STOP,
                        closest_goal_map,
                        short_term_goal,
                        dilated_obstacles,
                    )
                else:
                    # 其他情况，一般不会触发，这里保险也加上吧。目标边界和容器 一般不会出现这个情况，因为触发(5,5)优先清目标，最后会以目标图为空退出
                    # 已经不连通了，尝试通过切目标点改变不连通的情况？感觉可以直接退出
                    self.deleteGoal(closest_goal_pt, gx1, gx2, gy1, gy2)
                    self.collision_map *= 0
                    self.collision_mask_list.clear()
                    self.collision_mask_list480.clear()
                    self.last_aciton = DiscreteNavigationAction.TURN_LEFT
                    return (
                        DiscreteNavigationAction.TURN_LEFT,
                        np.zeros(goal_map.shape),
                        closest_goal_pt,
                        np.zeros(goal_map.shape),
                    )

            # 最小半径，尚且联通，永久增加障碍，不允许走这个路，掉头回去
            # 掉头回去可能会有新路，可能会永久不达到，后者进入第一个判断，添加障碍
            if self.curr_obs_dilation_selem_radius == self.min_obs_dilation_selem_radius and liantong == True:
                if xx == 5 and yy == 5:
                    list_len = len(self.collision_mask_list480)
                    for i in range(list_len):
                        self.collision_map[gx1:gx2, gy1:gy2][
                            self.collision_mask_list480[i][0], self.collision_mask_list480[i][1]] = 1
                    self.last_aciton = DiscreteNavigationAction.TURN_LEFT
                    return (
                        DiscreteNavigationAction.TURN_LEFT,
                        np.zeros(goal_map.shape),
                        closest_goal_pt,
                        np.zeros(goal_map.shape),
                    )

            # 其余情况，缩小半径，并重置 目标清空
            # 这个缩小障碍，执行时是很后面的步数，因为优先是清空目标。这里可能会有一些问题。
            if self.curr_obs_dilation_selem_radius > self.min_obs_dilation_selem_radius:
                self.curr_obs_dilation_selem_radius -= 1
                self.obs_dilation_selem = skimage.morphology.disk(self.curr_obs_dilation_selem_radius)
                self.delete_map *= 0

        # 其余 forward 的指令
        angle_agent = pu.normalize_angle(start_o)

        stg_x, stg_y = short_term_goal
        relative_stg_x, relative_stg_y = stg_x - start[0], stg_y - start[1]
        angle_st_goal = math.degrees(math.atan2(relative_stg_x, relative_stg_y))
        relative_angle_to_stg = pu.normalize_angle(angle_agent - angle_st_goal)

        # Compute angle to the final goal
        goal_x, goal_y = closest_goal_pt
        angle_goal = math.degrees(math.atan2(goal_x - start[0], goal_y - start[1]))
        relative_angle_to_closest_goal = pu.normalize_angle(angle_agent - angle_goal)

        action = self.get_action(
            relative_stg_x,
            relative_stg_y,
            relative_angle_to_stg,
            relative_angle_to_closest_goal,
            start_o,
            found_goal,
            stop,
            debug,
            fmm_dist,
            step_size,
        )

        # 移动指令
        self.last_stg = [xx, yy]
        self.last_stg480 = [short_term_goal[0], short_term_goal[1]]
        self.last_action = action

        return action, closest_goal_map, short_term_goal, dilated_obstacles
