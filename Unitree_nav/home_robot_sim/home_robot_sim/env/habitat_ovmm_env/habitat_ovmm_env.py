# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from enum import IntEnum
from typing import Any, Dict, Optional, Union
import sys
# sys.path.insert(0,'/home/zgh/zyp/home-robot/src/home_robot')
import habitat
import numpy as np
import torch
from habitat.core.environments import GymHabitatEnv
from habitat.core.simulator import Observations

import home_robot
import home_robot.core.interfaces
from home_robot.core.interfaces import (
    ContinuousFullBodyAction,
    ContinuousNavigationAction,
    DiscreteNavigationAction,
)
from home_robot.motion.stretch import (
    STRETCH_ARM_EXTENSION,
    STRETCH_ARM_LIFT,
    STRETCH_NAVIGATION_Q,
    STRETCH_PREGRASP_Q,
    map_joint_q_state_to_action_space,
)
from home_robot.utils.constants import (
    MAX_DEPTH_REPLACEMENT_VALUE,
    MIN_DEPTH_REPLACEMENT_VALUE,
)
from home_robot_sim.env.habitat_abstract_env import HabitatEnv
from home_robot_sim.env.habitat_objectnav_env.visualizer import Visualizer


class SimJointActionIndex(IntEnum):
    """
    Enum representing the indices of different joints in the action space.
    """

    # TODO: This needs to be common between sim and real as they share the same API for actions
    ARM = 0  # A single value is used to control the extension
    LIFT = 1
    WRIST_YAW = 2
    WRIST_PITCH = 3
    WRIST_ROLL = 4
    HEAD_PAN = 5
    HEAD_TILT = 6


class HabitatOpenVocabManipEnv(HabitatEnv):
    joints_dof = 7

    def __init__(self, habitat_env: habitat.core.env.Env, config, dataset):
        super().__init__(habitat_env)
        self.min_depth = config.ENVIRONMENT.min_depth  # 0
        self.max_depth = config.ENVIRONMENT.max_depth  # 10
        self.yansen_debug = config.get("YANSEN_DEBUG", False)
        self.ground_truth_semantics = config.GROUND_TRUTH_SEMANTICS  # 0
        self._dataset = dataset
        self.visualize = config.VISUALIZE or config.PRINT_IMAGES
        if self.visualize:
            self.visualizer = Visualizer(config, dataset)

        self.episodes_data_path = config.habitat.dataset.data_path  # 'data/datasets/ovmm/{split}/episodes.json.gz'
        self.video_dir = config.habitat_baselines.video_dir
        self.max_forward = (config.habitat.task.actions.base_velocity.max_displacement_along_axis)  # 1
        self.max_turn_degrees = (config.habitat.task.actions.base_velocity.max_turn_degrees)  # 180
        self.max_joints_delta = (config.habitat.task.actions.arm_action.max_delta_pos)  # for normalizing arm delta 6.28
        self.max_turn = (self.max_turn_degrees / 180 * np.pi)  # for normalizing turn angle3.14
        self.discrete_forward = (config.ENVIRONMENT.forward)  # amount the agent can move in a discrete step 0.25
        self.discrete_forward_1 = (config.ENVIRONMENT.forward_1)
        self.discrete_forward_2 = (config.ENVIRONMENT.forward_2)
        self.discrete_forward_3 = (config.ENVIRONMENT.forward_3)
        self.discrete_forward_4 = (config.ENVIRONMENT.forward_4)
        self.discrete_turn_degrees = (config.ENVIRONMENT.turn_angle)  # amount the agent turns in a discrete turn30
        self.joints_mask = np.array(
            config.habitat.task.actions.arm_action.arm_joint_mask)  # mask specifying which arm joints are to be setarray([1, 0, 0, 0, 1, 1, 1, 1, 1, 1])
        self.config = config

        self._obj_name_to_id_mapping = self._dataset.obj_category_to_obj_category_id
        # 'action_figure': 0, 'android_figure': 1, 'apple': 2, 'backpack': 3, 'baseballbat': 4, 'basket': 5, 'basketball': 6, 'bath_towel': 7, 'battery_charger': 8, 'board_game': 9, 'book': 10, 'bottle': 11, 'bowl': 12, 'box': 13, 'bread': 14, 'bundt_pan': 15, 'butter_dish': 16, 'c-clamp': 17, 'cake_pan': 18, 'can': 19, 'can_opener': 20, 'candle': 21, 'candle_holder': 22, 'candy_bar': 23, 'canister': 24, 'carrying_case': 25, 'casserole': 26, 'cellphone': 27, 'clock': 28, 'cloth': 29, 'credit_card': 30, 'cup': 31, 'cushion': 32, 'dish': 33, 'doll': 34, 'dumbbell': 35, 'egg': 36, 'electric_kettle': 37, 'electronic_cable': 38, 'file_sorter': 39, 'folder': 40, 'fork': 41, 'gaming_console': 42, 'glass': 43, 'hammer': 44, 'hand_towel': 45, 'handbag': 46, 'hard_drive': 47, 'hat': 48, 'helmet': 49, 'jar': 50, 'jug': 51, 'kettle': 52, 'keychain': 53, 'knife': 54, 'ladle': 55, 'lamp': 56, 'laptop': 57, 'laptop_cover': 58, 'laptop_stand': 59, 'lettuce': 60, 'lunch_box': 61, 'milk_frother_cup': 62, 'monitor_stand': 63, 'mouse_pad': 64, 'multiport_hub': 65, 'newspaper': 66, 'pan': 67, 'pen': 68, 'pencil_case': 69, 'phone_stand': 70, 'picture_frame': 71, 'pitcher': 72, 'plant_container': 73, 'plant_saucer': 74, 'plate': 75, 'plunger': 76, 'pot': 77, 'potato': 78, 'ramekin': 79, 'remote': 80, 'salt_and_pepper_shaker': 81, 'scissors': 82, 'screwdriver': 83, 'shoe': 84, 'soap': 85, 'soap_dish': 86, 'soap_dispenser': 87, 'spatula': 88, 'spectacles': 89, 'spicemill': 90, 'sponge': 91, 'spoon': 92, 'spray_bottle': 93, 'squeezer': 94, 'statue': 95, 'stuffed_toy': 96, 'sushi_mat': 97, 'tape': 98, 'teapot': 99, 'tennis_racquet': 100, 'tissue_box': 101, 'toiletry': 102, 'tomato': 103, 'toy_airplane': 104, 'toy_animal': 105, 'toy_bee': 106, 'toy_cactus': 107, 'toy_construction_set': 108, 'toy_fire_truck': 109, 'toy_food': 110, 'toy_fruits': 111, 'toy_lamp': 112, 'toy_pineapple': 113, 'toy_rattle': 114, 'toy_refrigerator': 115, 'toy_sink': 116, 'toy_sofa': 117, 'toy_swing': 118, 'toy_table': 119, 'toy_vehicle': 120, 'tray': 121, 'utensil_holder_cup': 122, 'vase': 123, 'video_game_cartridge': 124, 'watch': 125, 'watering_can': 126, 'wine_bottle': 127}
        self._rec_name_to_id_mapping = self._dataset.recep_category_to_recep_category_id
        # {'bathtub': 0, 'bed': 1, 'bench': 2, 'cabinet': 3, 'chair': 4, 'chest_of_drawers': 5, 'couch': 6, 'counter': 7, 'filing_cabinet': 8, 'hamper': 9, 'serving_cart': 10, 'shelves': 11, 'shoe_rack': 12, 'sink': 13, 'stand': 14, 'stool': 15, 'table': 16, 'toilet': 17, 'trunk': 18, 'wardrobe': 19, 'washer_dryer': 20}
        self._obj_id_to_name_mapping = {k: v for v, k in self._obj_name_to_id_mapping.items()}
        # {0: 'action_figure', 1: 'android_figure', 2: 'apple', 3: 'backpack', 4: 'baseballbat', 5: 'basket', 6: 'basketball', 7: 'bath_towel', 8: 'battery_charger', 9: 'board_game', 10: 'book', 11: 'bottle', 12: 'bowl', 13: 'box', 14: 'bread', 15: 'bundt_pan', 16: 'butter_dish', 17: 'c-clamp', 18: 'cake_pan', 19: 'can', 20: 'can_opener', 21: 'candle', 22: 'candle_holder', 23: 'candy_bar', 24: 'canister', 25: 'carrying_case', 26: 'casserole', 27: 'cellphone', 28: 'clock', 29: 'cloth', 30: 'credit_card', 31: 'cup', 32: 'cushion', 33: 'dish', 34: 'doll', 35: 'dumbbell', 36: 'egg', 37: 'electric_kettle', 38: 'electronic_cable', 39: 'file_sorter', 40: 'folder', 41: 'fork', 42: 'gaming_console', 43: 'glass', 44: 'hammer', 45: 'hand_towel', 46: 'handbag', 47: 'hard_drive', 48: 'hat', 49: 'helmet', 50: 'jar', 51: 'jug', 52: 'kettle', 53: 'keychain', 54: 'knife', 55: 'ladle', 56: 'lamp', 57: 'laptop', 58: 'laptop_cover', 59: 'laptop_stand', 60: 'lettuce', 61: 'lunch_box', 62: 'milk_frother_cup', 63: 'monitor_stand', 64: 'mouse_pad', 65: 'multiport_hub', 66: 'newspaper', 67: 'pan', 68: 'pen', 69: 'pencil_case', 70: 'phone_stand', 71: 'picture_frame', 72: 'pitcher', 73: 'plant_container', 74: 'plant_saucer', 75: 'plate', 76: 'plunger', 77: 'pot', 78: 'potato', 79: 'ramekin', 80: 'remote', 81: 'salt_and_pepper_shaker', 82: 'scissors', 83: 'screwdriver', 84: 'shoe', 85: 'soap', 86: 'soap_dish', 87: 'soap_dispenser', 88: 'spatula', 89: 'spectacles', 90: 'spicemill', 91: 'sponge', 92: 'spoon', 93: 'spray_bottle', 94: 'squeezer', 95: 'statue', 96: 'stuffed_toy', 97: 'sushi_mat', 98: 'tape', 99: 'teapot', 100: 'tennis_racquet', 101: 'tissue_box', 102: 'toiletry', 103: 'tomato', 104: 'toy_airplane', 105: 'toy_animal', 106: 'toy_bee', 107: 'toy_cactus', 108: 'toy_construction_set', 109: 'toy_fire_truck', 110: 'toy_food', 111: 'toy_fruits', 112: 'toy_lamp', 113: 'toy_pineapple', 114: 'toy_rattle', 115: 'toy_refrigerator', 116: 'toy_sink', 117: 'toy_sofa', 118: 'toy_swing', 119: 'toy_table', 120: 'toy_vehicle', 121: 'tray', 122: 'utensil_holder_cup', 123: 'vase', 124: 'video_game_cartridge', 125: 'watch', 126: 'watering_can', 127: 'wine_bottle'}
        self._rec_id_to_name_mapping = {k: v for v, k in self._rec_name_to_id_mapping.items()}
        # {0: 'bathtub', 1: 'bed', 2: 'bench', 3: 'cabinet', 4: 'chair', 5: 'chest_of_drawers', 6: 'couch', 7: 'counter', 8: 'filing_cabinet', 9: 'hamper', 10: 'serving_cart', 11: 'shelves', 12: 'shoe_rack', 13: 'sink', 14: 'stand', 15: 'stool', 16: 'table', 17: 'toilet', 18: 'trunk', 19: 'wardrobe', 20: 'washer_dryer'}
        self._instance_ids_start_in_panoptic = (config.habitat.simulator.instance_ids_start)  # 50
        self._last_habitat_obs = None
        # self.obj_ids = []
        # self.rec_ids = []

    def get_current_episode(self):
        if isinstance(self.habitat_env, GymHabitatEnv):
            return self.habitat_env.current_episode()
        else:
            return self.habitat_env.current_episode

    def set_vis_dir(self):
        scene_id = self.get_current_episode().scene_id.split("/")[-1].split(".")[0]
        episode_id = self.get_current_episode().episode_id
        self.visualizer.set_vis_dir(scene_id=scene_id, episode_id=episode_id)

    def reset(self):

        habitat_obs = self.habitat_env.reset()
        # for key, value in habitat_obs.items():
        #     print(f"{key}: {value}")
        # print("\n\n\n\n")

        self._last_habitat_obs = habitat_obs
        self._last_obs = self._preprocess_obs(habitat_obs)

        # for key, value in vars(self._last_obs).items():
        #     print(f"{key}: {value}")
        # exit()

        if self.visualize:
            self.visualizer.reset()
            self.set_vis_dir()
        return self._last_obs

    def convert_pose_to_real_world_axis(self, hab_pose):
        """Update axis convention of habitat pose to match the real-world axis convention"""
        hab_pose[[0, 1, 2]] = hab_pose[[2, 0, 1]]
        hab_pose[:, [0, 1, 2]] = hab_pose[:, [2, 0, 1]]
        return hab_pose

    def _preprocess_xy(self, xy: np.array) -> np.array:
        """Translate Habitat navigation (x, y) (i.e., GPS sensor) into robot (x, y)."""
        return np.array([xy[1], xy[0]])

    def _preprocess_obs(self,
                        habitat_obs: habitat.core.simulator.Observations) -> home_robot.core.interfaces.Observations:

        # for key, value in habitat_obs.items():
        #     print(key, value)
        # robot_head_depth(640, 480, 1)
        # robot_head_rgb(640, 480, 3)
        # robot_third_rgb(512, 512, 3)
        # start_receptacle(1, )
        # goal_receptacle(1, )
        # robot_start_gps(2, )
        # robot_start_compass(1, )
        # joint(10, )    ?
        # object_segmentation(640, 480, 1)   0.0
        # start_recep_segmentation(640, 480, 1)   0.0
        # goal_recep_segmentation(640, 480, 1)  0.0
        # object_category(1, )
        # object_embedding(512, )   not None
        # is_holding(1, )    0.0
        # relative_resting_position(3, )
        # ovmm_nav_goal_segmentation(640, 480, 2)    0.0
        # receptacle_segmentation(640, 480, 1)   0.0

        # 第0 张 进行预处理
        # 深度的归一化 按照预设的最大最小深度 0 10.0 归一化 把0 10.0 置成无限距离

        depth = self._preprocess_depth(habitat_obs["robot_head_depth"])  # (640,480)

        # 第三视角，系统直接给出
        if self.visualize and "robot_third_rgb" in habitat_obs:
            third_person_image = habitat_obs["robot_third_rgb"]
        else:
            third_person_image = None

        # 组合成 obs 类  没有改动数据，object embedding 直接从数据集里拿的
        obs = home_robot.core.interfaces.Observations(
            rgb=habitat_obs["robot_head_rgb"],
            depth=depth,
            compass=habitat_obs["robot_start_compass"],
            gps=self._preprocess_xy(habitat_obs["robot_start_gps"]),
            task_observations={
                "object_embedding": habitat_obs["object_embedding"],
                "start_receptacle": habitat_obs["start_receptacle"],
                "goal_receptacle": habitat_obs["goal_receptacle"],
                "prev_grasp_success": habitat_obs["is_holding"],
            },
            joint=habitat_obs["joint"],
            relative_resting_position=habitat_obs["relative_resting_position"],
            third_person_image=third_person_image,
            camera_pose=self.convert_pose_to_real_world_axis(
                np.asarray(habitat_obs["camera_pose"])
            ),
        )

        # 数字目标信息  goal的 id 字符串 这些信息
        obs = self._preprocess_goal(obs, habitat_obs)
        #  预处理？ 直接拿编码
        # 只有 gt 的时候才会生效，从 gt 里面拿 编码，否则要后需要，机器人的视觉模型去处理
        obs = self._preprocess_semantic(obs, habitat_obs)

        # print(obs.task_observations["goal_name"])
        # 这是啥，好像没有
        if "robot_head_panoptic" in habitat_obs:  # F
            gt_instance_ids = \
            np.maximum(0, habitat_obs["robot_head_panoptic"] - self._instance_ids_start_in_panoptic + 1, )[..., 0]
            # to be used for evaluating the instance map
            obs.task_observations["gt_instance_ids"] = gt_instance_ids
            if self.ground_truth_semantics:
                obs.task_observations["instance_map"] = gt_instance_ids
        return obs

    def _preprocess_semantic(  # 语义预处理
            self, obs: home_robot.core.interfaces.Observations, habitat_obs
    ) -> home_robot.core.interfaces.Observations:
        if self.ground_truth_semantics:
            semantic = torch.from_numpy(
                habitat_obs["object_segmentation"].squeeze(-1).astype(np.int64)
            )
            recep_seg = (
                habitat_obs["receptacle_segmentation"].squeeze(-1).astype(np.int64)
            )
            # if not torch.sum(semantic) == 0:
            #     import pdb; pdb.set_trace()
            recep_seg[recep_seg != 0] += 1
            recep_seg = torch.from_numpy(recep_seg)
            semantic = semantic + recep_seg
            semantic[semantic == 0] = len(self._rec_id_to_name_mapping) + 2
            obs.semantic = semantic.numpy()
            obs.task_observations["recep_idx"] = 2
            obs.task_observations["semantic_max_val"] = (len(self._rec_id_to_name_mapping) + 2)
        return obs

    def _preprocess_depth(self, depth: np.array) -> np.array:
        # 深度图，全局的归一化
        rescaled_depth = self.min_depth + depth * (self.max_depth - self.min_depth)
        rescaled_depth[depth == 0.0] = MIN_DEPTH_REPLACEMENT_VALUE
        rescaled_depth[depth == 1.0] = MAX_DEPTH_REPLACEMENT_VALUE
        return rescaled_depth[:, :, -1]

    def _preprocess_goal(
            self, obs: home_robot.core.interfaces.Observations, habitat_obs: Observations
    ) -> home_robot.core.interfaces.Observations:
        assert "object_category" in habitat_obs
        assert "start_receptacle" in habitat_obs
        assert "goal_receptacle" in habitat_obs

        obj_name = self._obj_id_to_name_mapping[habitat_obs["object_category"][0]]  #
        # {0: 'action_figure', 1: 'android_figure', 2: 'apple', 3: 'backpack', 4: 'baseballbat', 5: 'basket',
        #  6: 'basketball', 7: 'bath_towel', 8: 'battery_charger', 9: 'board_game', 10: 'book', 11: 'bottle', 12: 'bowl',
        #  13: 'box', 14: 'bread', 15: 'bundt_pan', 16: 'butter_dish', 17: 'c-clamp', 18: 'cake_pan', 19: 'can',
        #  20: 'can_opener', 21: 'candle', 22: 'candle_holder', 23: 'candy_bar', 24: 'canister', 25: 'carrying_case',
        #  26: 'casserole', 27: 'cellphone', 28: 'clock', 29: 'cloth', 30: 'credit_card', 31: 'cup', 32: 'cushion',
        #  33: 'dish', 34: 'doll', 35: 'dumbbell', 36: 'egg', 37: 'electric_kettle', 38: 'electronic_cable',
        #  39: 'file_sorter', 40: 'folder', 41: 'fork', 42: 'gaming_console', 43: 'glass', 44: 'hammer', 45: 'hand_towel',
        #  46: 'handbag', 47: 'hard_drive', 48: 'hat', 49: 'helmet', 50: 'jar', 51: 'jug', 52: 'kettle', 53: 'keychain',
        #  54: 'knife', 55: 'ladle', 56: 'lamp', 57: 'laptop', 58: 'laptop_cover', 59: 'laptop_stand', 60: 'lettuce',
        #  61: 'lunch_box', 62: 'milk_frother_cup', 63: 'monitor_stand', 64: 'mouse_pad', 65: 'multiport_hub',
        #  66: 'newspaper', 67: 'pan', 68: 'pen', 69: 'pencil_case', 70: 'phone_stand', 71: 'picture_frame',
        #  72: 'pitcher', 73: 'plant_container', 74: 'plant_saucer', 75: 'plate', 76: 'plunger', 77: 'pot', 78: 'potato',
        #  79: 'ramekin', 80: 'remote', 81: 'salt_and_pepper_shaker', 82: 'scissors', 83: 'screwdriver', 84: 'shoe',
        #  85: 'soap', 86: 'soap_dish', 87: 'soap_dispenser', 88: 'spatula', 89: 'spectacles', 90: 'spicemill',
        #  91: 'sponge', 92: 'spoon', 93: 'spray_bottle', 94: 'squeezer', 95: 'statue', 96: 'stuffed_toy',
        #  97: 'sushi_mat', 98: 'tape', 99: 'teapot', 100: 'tennis_racquet', 101: 'tissue_box', 102: 'toiletry',
        #  103: 'tomato', 104: 'toy_airplane', 105: 'toy_animal', 106: 'toy_bee', 107: 'toy_cactus',
        #  108: 'toy_construction_set', 109: 'toy_fire_truck', 110: 'toy_food', 111: 'toy_fruits', 112: 'toy_lamp',
        #  113: 'toy_pineapple', 114: 'toy_rattle', 115: 'toy_refrigerator', 116: 'toy_sink', 117: 'toy_sofa',
        #  118: 'toy_swing', 119: 'toy_table', 120: 'toy_vehicle', 121: 'tray', 122: 'utensil_holder_cup', 123: 'vase',
        #  124: 'video_game_cartridge', 125: 'watch', 126: 'watering_can', 127: 'wine_bottle'}
        start_receptacle = self._rec_id_to_name_mapping[
            habitat_obs["start_receptacle"][0]
        ]
        goal_receptacle = self._rec_id_to_name_mapping[
            habitat_obs["goal_receptacle"][0]
        ]
        goal_name = (
                "Move " + obj_name + " from " + start_receptacle + " to " + goal_receptacle
        )

        obj_goal_id = 1  # semantic sensor returns binary mask for goal object  图像中目标为1，其他为0

        if self.ground_truth_semantics:  # 已知语义信息
            start_rec_goal_id = self._rec_name_to_id_mapping[start_receptacle] + 2  # 随机开始，所以+2
            end_rec_goal_id = self._rec_name_to_id_mapping[goal_receptacle] + 2
        else:
            # To be populated by the agent
            start_rec_goal_id = -1
            end_rec_goal_id = -1

        # "Move " + obj_name + " from " + start_receptacle + " to " + goal_receptacle
        obs.task_observations["goal_name"] = goal_name
        obs.task_observations["object_name"] = obj_name
        obs.task_observations["start_recep_name"] = start_receptacle
        obs.task_observations["place_recep_name"] = goal_receptacle
        obs.task_observations["object_goal"] = obj_goal_id  # 1
        obs.task_observations["start_recep_goal"] = start_rec_goal_id  #
        obs.task_observations["end_recep_goal"] = end_rec_goal_id

        return obs

    def get_current_joint_pos(self, habitat_obs: Dict[str, Any]) -> np.array:
        """Returns the current absolute positions from habitat observations for the joints controlled by the action space"""
        complete_joint_pos = habitat_obs[
            "joint"]  # 10个值[ 0.       ,  0.       ,  0.       ,  0.       ,  0.775    ,        0.       , -1.5707964,  0.       ,  0.       , -0.5236   ],
        curr_joint_pos = complete_joint_pos[
            self.joints_mask == 1
            ]  # The action space will have the same size as curr_joint_pos
        # If action controls the arm extension, get the final extension by summing over individiual joint positions
        if self.joints_mask[0] == 1:  # T
            curr_joint_pos[SimJointActionIndex.ARM] = np.sum(
                complete_joint_pos[:4]
            )  # The first 4 values in sensor add up to give the complete extension
        return curr_joint_pos

    def _preprocess_action(
            self, action: Union[home_robot.core.interfaces.Action, Dict], habitat_obs
    ) -> int:
        # import pdb;pdb.set_trace()
        """convert the ovmm agent's action outputs to continuous Habitat actions"""
        if type(action) in [ContinuousFullBodyAction, ContinuousNavigationAction]:  # 连续动作
            grip_action = -1
            # Keep holding in case holding an object
            if habitat_obs["is_holding"][0] == 1:
                grip_action = 1
            waypoint_x, waypoint_y, turn = 0, 0, 0
            # Set waypoint correctly, if base waypoint is passed with the action
            if action.xyt is not None:
                if action.xyt[0] != 0:
                    waypoint_x = np.clip(action.xyt[0] / self.max_forward, -1, 1)
                if action.xyt[1] != 0:
                    waypoint_y = np.clip(action.xyt[1] / self.max_forward, -1, 1)
                if action.xyt[2] != 0:
                    turn = np.clip(action.xyt[2] / self.max_turn, -1, 1)
            arm_action = np.array([0] * self.joints_dof)
            # If action is of type ContinuousFullBodyAction, it would include waypoints for the joints
            if type(action) == ContinuousFullBodyAction:
                # We specify only one arm extension that rolls over to all the arm joints
                arm_action = np.concatenate([action.joints[0:1], action.joints[4:]])
            cont_action = np.concatenate(
                [
                    np.clip(arm_action / self.max_joints_delta, -1, 1),
                    [grip_action] + [waypoint_x, waypoint_y, turn, -1],
                ]
            )
        elif type(action) == DiscreteNavigationAction:  # 离散动作
            grip_action = -1  # 夹持器处于松开
            if (habitat_obs["is_holding"][
                    0] == 1 and action != DiscreteNavigationAction.DESNAP_OBJECT) or action == DiscreteNavigationAction.SNAP_OBJECT:
                # 夹持器是否正在夹着                    当前动作不是放置动作                                                夹取对象的动作
                grip_action = 1  # 加上

            turn = 0
            forward = 0

            # 底盘 移动

            if action == DiscreteNavigationAction.TURN_RIGHT:  # 左正右负
                turn = -self.discrete_turn_degrees  # 30度
            elif action == DiscreteNavigationAction.TURN_LEFT:
                turn = self.discrete_turn_degrees  # 30度
            elif action == DiscreteNavigationAction.PLACE_TURN_LIFT:
                turn = 15
            elif action == DiscreteNavigationAction.MOVE_FORWARD:
                forward = self.discrete_forward
            elif action == DiscreteNavigationAction.PLACE_MOVE_FORWARD:
                forward = 0.1
            arm_action = np.zeros(self.joints_dof)
            curr_joint_pos = self.get_current_joint_pos(habitat_obs)
            target_joint_pos = curr_joint_pos
            if action == DiscreteNavigationAction.MANIPULATION_MODE:
                # turn left by 90 degrees, positive for turn left
                turn = 90
                target_joint_pos = map_joint_q_state_to_action_space(STRETCH_PREGRASP_Q)
                arm_action = target_joint_pos - curr_joint_pos
            elif action == DiscreteNavigationAction.NEW_MANIPULATION_MODE:
                # turn left by 90 degrees, positive for turn left
                turn = 90
                STRETCH_PREGRASP_Q[3] = 1.2
                target_joint_pos = map_joint_q_state_to_action_space(STRETCH_PREGRASP_Q)
                arm_action = target_joint_pos - curr_joint_pos

            elif action == DiscreteNavigationAction.NAVIGATION_MODE:
                target_joint_pos = map_joint_q_state_to_action_space(STRETCH_NAVIGATION_Q)
                arm_action = target_joint_pos - curr_joint_pos

            elif action == DiscreteNavigationAction.EXTEND_ARM:  # 将机械臂伸展到特定的位置和高度
                target_joint_pos = curr_joint_pos.copy()
                target_joint_pos[SimJointActionIndex.ARM] = STRETCH_ARM_EXTENSION
                target_joint_pos[SimJointActionIndex.LIFT] = STRETCH_ARM_LIFT
                arm_action = target_joint_pos - curr_joint_pos

            stop = float(action == DiscreteNavigationAction.STOP) * 2 - 1  # 停止1  不停止-1
            cont_action = np.concatenate(
                [
                    arm_action / self.max_joints_delta,  # 7#机械臂 /6.28
                    [
                        grip_action,  # 1 夹持器 夹不夹1，-1
                        forward / self.max_forward,  # 1 直行
                        0.0,  # 1   0
                        turn / self.max_turn_degrees,  # 1  转动角度  弧度
                        stop,  # 1  停不停1 -1
                    ],
                ]
            )
        else:
            raise ValueError(
                "Action needs to be of one of the following types: DiscreteNavigationAction, ContinuousNavigationAction or ContinuousFullBodyAction"
            )
        return np.array(cont_action, dtype=np.float32)

    def _process_info(self, info: Dict[str, Any]) -> Any:
        if info and self.visualize:
            self.visualizer.visualize(**info)

    # 执行动作
    def apply_action(
            self,
            action: home_robot.core.interfaces.Action,
            info: Optional[Dict[str, Any]] = None,
    ):
        if info is not None:
            if type(action) == ContinuousNavigationAction:
                info["curr_action"] = str([round(a, 3) for a in action.xyt])
            if type(action) == DiscreteNavigationAction:
                info["curr_action"] = DiscreteNavigationAction(action).name
            self._process_info(info)
        habitat_action = self._preprocess_action(action, self._last_habitat_obs)  # (12,) #
        # print(action,habitat_action)
        # import pdb;pdb.set_trace()
        habitat_obs, _, dones, infos = self.habitat_env.step(habitat_action)  ##################
        # copy the keys in info starting with the prefix "is_curr_skill" into infos
        for key in info:
            if key.startswith("is_curr_skill"):
                infos[key] = info[key]
        self._last_habitat_obs = habitat_obs
        # if self.yansen_debug:
        #     import pdb; pdb.set_trace()
        self._last_obs = self._preprocess_obs(habitat_obs)
        return self._last_obs, dones, infos
