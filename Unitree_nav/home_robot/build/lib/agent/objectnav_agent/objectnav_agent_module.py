# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from typing import Optional

import torch.nn as nn

from home_robot.mapping.semantic.categorical_2d_semantic_map_module import (
    Categorical2DSemanticMapModule,
)
from home_robot.mapping.semantic.instance_tracking_modules import InstanceMemory
from home_robot.navigation_policy.object_navigation.objectnav_frontier_exploration_policy_yuan import (
    ObjectNavFrontierExplorationPolicy,
)

# Do we need to visualize the frontier as we explore?
debug_frontier_map = False


class ObjectNavAgentModule(nn.Module):
    def __init__(self, config, instance_memory: Optional[InstanceMemory] = None):
        super().__init__()
        self.instance_memory = instance_memory
        self.semantic_map_module = Categorical2DSemanticMapModule(
            frame_height=config["ENVIRONMENT"]["frame_height"],
            frame_width=config["ENVIRONMENT"]["frame_width"],
            camera_height=config["ENVIRONMENT"]["camera_height"],
            hfov=config["ENVIRONMENT"]["hfov"],
            num_sem_categories=config["AGENT"]["SEMANTIC_MAP"]["num_sem_categories"],
            map_size_cm=config["AGENT"]["SEMANTIC_MAP"]["map_size_cm"],
            map_resolution=config["AGENT"]["SEMANTIC_MAP"]["map_resolution"],
            vision_range=config["AGENT"]["SEMANTIC_MAP"]["vision_range"],
            explored_radius=config["AGENT"]["SEMANTIC_MAP"]["explored_radius"],
            been_close_to_radius=config["AGENT"]["SEMANTIC_MAP"]["been_close_to_radius"],
            global_downscaling=config["AGENT"]["SEMANTIC_MAP"]["global_downscaling"],
            du_scale=config["AGENT"]["SEMANTIC_MAP"]["du_scale"],
            cat_pred_threshold=config["AGENT"]["SEMANTIC_MAP"]["cat_pred_threshold"],
            exp_pred_threshold=config["AGENT"]["SEMANTIC_MAP"]["exp_pred_threshold"],
            map_pred_threshold=config["AGENT"]["SEMANTIC_MAP"]["map_pred_threshold"],
            must_explore_close=config["AGENT"]["SEMANTIC_MAP"]["must_explore_close"],
            min_obs_height_cm=config["AGENT"]["SEMANTIC_MAP"]["min_obs_height_cm"],
            dilate_obstacles=config["AGENT"]["SEMANTIC_MAP"]["dilate_obstacles"],
            dilate_size=config["AGENT"]["SEMANTIC_MAP"]["dilate_size"],
            dilate_iter=config["AGENT"]["SEMANTIC_MAP"]["dilate_iter"],
            record_instance_ids=getattr(
                config["AGENT"]["SEMANTIC_MAP"], "record_instance_ids", False
            ),
            instance_memory=instance_memory,
            max_instances=getattr(config["AGENT"]["SEMANTIC_MAP"], "max_instances", 0),

            evaluate_instance_tracking=getattr(
                config["ENVIRONMENT"], "evaluate_instance_tracking", False
            ),
        )
        self.policy = ObjectNavFrontierExplorationPolicy(
            exploration_strategy=config["AGENT"]["exploration_strategy"]
        )

    @property
    def goal_update_steps(self):
        return self.policy.goal_update_steps

    def forward(
            self,
            timesteps,
            seq_obs,
            seq_pose_delta,
            seq_dones,
            seq_update_global,
            seq_camera_poses,
            init_local_map,
            init_global_map,
            init_local_pose,
            init_global_pose,
            init_lmb,
            init_origins,
            seq_object_goal_category=None,
            seq_start_recep_goal_category=None,
            seq_end_recep_goal_category=None,
            seq_nav_to_recep=None,
    ):

        # t0 = time.time()
        # Update map with observations and generate map features
        batch_size, sequence_length = seq_obs.shape[:2]
        (
            seq_map_features,
            final_local_map,
            final_global_map,
            seq_local_pose,
            seq_global_pose,
            seq_lmb,
            seq_origins,
        ) = self.semantic_map_module(  #地图中的语义映射
            seq_obs,
            seq_pose_delta,
            seq_dones,
            seq_update_global,
            seq_camera_poses,
            init_local_map,
            init_global_map,
            init_local_pose,
            init_global_pose,
            init_lmb,
            init_origins,
        )

        # t1 = time.time()
        # print(f"[Semantic mapping] Total time: {t1 - t0:.2f}")

        # Predict high-level goals from map features
        # batched across sequence length x num environments
        #import pdb;pdb.set_trace()
        map_features = seq_map_features.flatten(0, 1)
        if seq_object_goal_category is not None:
            seq_object_goal_category = seq_object_goal_category.flatten(0, 1)
        if seq_start_recep_goal_category is not None:
            seq_start_recep_goal_category = seq_start_recep_goal_category.flatten(0, 1)
        if seq_end_recep_goal_category is not None:
            seq_end_recep_goal_category = seq_end_recep_goal_category.flatten(0, 1)

        if timesteps[0] < 2:
            self.policy.rec_record.clear()

        goal_map, found_goal, found_rec = self.policy( #探索策略
            map_features,
            seq_object_goal_category,
            seq_start_recep_goal_category,
            seq_end_recep_goal_category,
            seq_nav_to_recep,
        )

        seq_goal_map = goal_map.view(batch_size, sequence_length, *goal_map.shape[-2:])
        seq_found_goal = found_goal.view(batch_size, sequence_length)
        seq_found_rec = found_rec.view(batch_size, sequence_length)

        # Compute the frontier map here
        frontier_map = self.policy.get_frontier_map(map_features)
        seq_frontier_map = frontier_map.view(
            batch_size, sequence_length, *frontier_map.shape[-2:]
        )
        if debug_frontier_map:
            import matplotlib.pyplot as plt

            plt.subplot(121)
            plt.imshow(seq_frontier_map[0, 0].numpy())
            plt.subplot(122)
            plt.imshow(goal_map[0].numpy())
            plt.show()
            breakpoint()
        # t2 = time.time()
        # print(f"[Policy] Total time: {t2 - t1:.2f}")

        return (
            seq_goal_map,
            seq_found_goal,
            seq_found_rec,
            seq_frontier_map,
            final_local_map,
            final_global_map,
            seq_local_pose,
            seq_global_pose,
            seq_lmb,
            seq_origins,
        )
