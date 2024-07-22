# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, List, Tuple

#import pdb;pdb.set_trace()
import numpy as np
import torch
from torch.nn import DataParallel
import pickle
import home_robot.utils.pose as pu
from home_robot.core.abstract_agent import Agent
from home_robot.core.interfaces import DiscreteNavigationAction, Observations
from home_robot.mapping.semantic.categorical_2d_semantic_map_state import (
    Categorical2DSemanticMapState,
)
from home_robot.mapping.semantic.instance_tracking_modules import InstanceMemory
from home_robot.navigation_planner.discrete_planner import DiscretePlanner

from home_robot.agent.objectnav_agent.objectnav_agent_module import ObjectNavAgentModule

# For visualizing exploration issuesdddddd
debug_frontier_map = False


class ObjectNavAgent(Agent):
    """Simple object nav agent based on a 2D semantic map"""

    # Flag for debugging data flow and task configuraiton
    verbose = False

    def __init__(
            self,
            config,
            device_id: int = 0,
            min_goal_distance_cm: float = 50.0,
            continuous_angle_tolerance: float = 30.0,
            goal_tolerance: float = 0.01,
    ):
        #import pdb;pdb.set_trace()
        self.max_steps = config["AGENT"]["max_steps"]
        self.num_environments = config["NUM_ENVIRONMENTS"]
        self.store_all_categories_in_map = getattr(   config["AGENT"], "store_all_categories", False )
        if config["AGENT"]["panorama_start"]:
            self.panorama_start_steps = int(360 / config["ENVIRONMENT"]["turn_angle"])
        else:
            self.panorama_start_steps = 0

        self.instance_memory = None
        self.record_instance_ids = getattr( config["AGENT"]["SEMANTIC_MAP"], "record_instance_ids", False   )

        if self.record_instance_ids:
            self.instance_memory = InstanceMemory(self.num_environments,   config["AGENT"]["SEMANTIC_MAP.du_scale"],    debug_visualize=config["PRINT_IMAGES"],     )

        self._module = ObjectNavAgentModule(  config, instance_memory=self.instance_memory   )

        if config["NO_GPU"]:
            self.device = torch.device("cpu")
            self.module = self._module
        else:
            self.device_id = device_id
            self.device = torch.device(f"cuda:{self.device_id}")
            self._module = self._module.to(self.device)
            # Use DataParallel only as a wrapper to move model inputs to GPU
            self.module = DataParallel(self._module, device_ids=[self.device_id])

        self.visualize = config["VISUALIZE"] or config["PRINT_IMAGES"]
        self.use_dilation_for_stg = config["AGENT"]["PLANNER"]["use_dilation_for_stg"]
        self.semantic_map = Categorical2DSemanticMapState(
            device=self.device,
            num_environments=self.num_environments,
            num_sem_categories=config["AGENT"]["SEMANTIC_MAP"]["num_sem_categories"],
            map_resolution=config["AGENT"]["SEMANTIC_MAP"]["map_resolution"],
            map_size_cm=config["AGENT"]["SEMANTIC_MAP"]["map_size_cm"],
            global_downscaling=config["AGENT"]["SEMANTIC_MAP"]["global_downscaling"],
            record_instance_ids=getattr( config["AGENT"]["SEMANTIC_MAP"], "record_instance_ids", False   ),   
            max_instances=getattr(config["AGENT"]["SEMANTIC_MAP"], "max_instances", 0),
            evaluate_instance_tracking=getattr(  config["ENVIRONMENT"], "evaluate_instance_tracking", False   ),
            instance_memory=self.instance_memory,
        )
        agent_radius_cm = config["AGENT"]["radius"] * 100.0
        agent_cell_radius = int(   np.ceil(agent_radius_cm / config["AGENT"]["SEMANTIC_MAP"]["map_resolution"])    )
      
        self.planner = DiscretePlanner(
            turn_angle=config["ENVIRONMENT"]["turn_angle"],
            collision_threshold=config["AGENT"]["PLANNER"]["collision_threshold"],
            step_size=config["AGENT"]["PLANNER"]["step_size"],
            obs_dilation_selem_radius=config["AGENT"]["PLANNER"]["obs_dilation_selem_radius"],
            goal_dilation_selem_radius=config["AGENT"]["PLANNER"]["goal_dilation_selem_radius"],
            map_size_cm=config["AGENT"]["SEMANTIC_MAP"]["map_size_cm"],
            map_resolution=config["AGENT"]["SEMANTIC_MAP"]["map_resolution"],
            visualize=config["VISUALIZE"],
            print_images=config["PRINT_IMAGES"],
            dump_location=config["DUMP_LOCATION"],
            exp_name=config["EXP_NAME"],
            agent_cell_radius=agent_cell_radius,
            min_obs_dilation_selem_radius=config["AGENT"]["PLANNER"]["min_obs_dilation_selem_radius"],
            map_downsample_factor=config["AGENT"]["PLANNER"]["map_downsample_factor"],
            map_update_frequency=config["AGENT"]["PLANNER"]["map_update_frequency"],
            discrete_actions=config["AGENT"]["PLANNER"]["discrete_actions"],
            min_goal_distance_cm=min_goal_distance_cm,
            goal_tolerance=goal_tolerance,
            continuous_angle_tolerance=continuous_angle_tolerance,
        )
        self.one_hot_encoding = torch.eye(                                        #一共的类别           
            config["AGENT"]["SEMANTIC_MAP"]["num_sem_categories"], device=self.device
        )

        self.goal_update_steps = self._module.goal_update_steps
        self.timesteps = None
        self.timesteps_before_goal_update = None
        self.episode_panorama_start_steps = None
        self.last_poses = None

        # import pdb;pdb.set_trace()
        self.verbose = config["AGENT"]["PLANNER"]["verbose"]

        self.evaluate_instance_tracking = getattr(
            config["ENVIRONMENT"], "evaluate_instance_tracking", False
        )
        self.one_hot_instance_encoding = None
        if self.evaluate_instance_tracking:
            self.one_hot_instance_encoding = torch.eye(
                config["AGENT"]["SEMANTIC_MAP"]["max_instances"] + 1, device=self.device
            )
        self.config = config

    # ------------------------------------------------------------------
    # Inference methods to interact with vectorized simulation
    # environments
    # ------------------------------------------------------------------

    @torch.no_grad()
    def prepare_planner_inputs(
            self,
            obs: torch.Tensor,
            pose_delta: torch.Tensor,
            object_goal_category: torch.Tensor = None,
            start_recep_goal_category: torch.Tensor = None,
            end_recep_goal_category: torch.Tensor = None,
            nav_to_recep: torch.Tensor = None,
            camera_pose: torch.Tensor = None,
    ) -> Tuple[List[dict], List[dict]]:
        """Prepare low-level planner inputs from an observation - this is
        the main inference function of the agent that lets it interact with
        vectorized environments.

        This function assumes that the agent has been initialized.

        Args:
            obs: current frame containing (RGB, depth, segmentation) of shape
             (num_environments, 3 + 1 + num_sem_categories, frame_height, frame_width)
            pose_delta: sensor pose delta (dy, dx, dtheta) since last frame
             of shape (num_environments, 3)
            object_goal_category: semantic category of small object goals
            start_recep_goal_category: semantic category of start receptacle goals
            end_recep_goal_category: semantic category of end receptacle goals
            camera_pose: camera extrinsic pose of shape (num_environments, 4, 4)

        Returns:
            planner_inputs: list of num_environments planner inputs dicts containing
                obstacle_map: (M, M) binary np.ndarray local obstacle map
                 prediction
                sensor_pose: (7,) np.ndarray denoting global pose (x, y, o)
                 and local map boundaries planning window (gx1, gx2, gy1, gy2)
                goal_map: (M, M) binary np.ndarray denoting goal location
            vis_inputs: list of num_environments visualization info dicts containing
                explored_map: (M, M) binary np.ndarray local explored map
                 prediction
                semantic_map: (M, M) np.ndarray containing local semantic map
                 predictions
        """
        #import pdb;pdb.set_trace()
        dones = torch.tensor([False] * self.num_environments)
        update_global = torch.tensor(   [self.timesteps_before_goal_update[e] == 0     for e in range(self.num_environments) ]  )
        if object_goal_category is not None:
            object_goal_category = object_goal_category.unsqueeze(1)
        if start_recep_goal_category is not None:
            start_recep_goal_category = start_recep_goal_category.unsqueeze(1)
        if end_recep_goal_category is not None:
            end_recep_goal_category = end_recep_goal_category.unsqueeze(1)
        # import pdb;pdb.set_trace()
        (
            goal_map,
            found_goal,
            found_rec,
            frontier_map,
            self.semantic_map.local_map,
            self.semantic_map.global_map,
            seq_local_pose,
            seq_global_pose,
            seq_lmb,
            seq_origins,
        ) = self.module(
            self.timesteps,
            obs.unsqueeze(1),
            pose_delta.unsqueeze(1),
            dones.unsqueeze(1),
            update_global.unsqueeze(1),
            camera_pose,
            self.semantic_map.local_map,
            self.semantic_map.global_map,
            self.semantic_map.local_pose,
            self.semantic_map.global_pose,
            self.semantic_map.lmb,
            self.semantic_map.origins,
            seq_object_goal_category=object_goal_category,
            seq_start_recep_goal_category=start_recep_goal_category,
            seq_end_recep_goal_category=end_recep_goal_category,
            seq_nav_to_recep=nav_to_recep,
        )
        #import pdb;pdb.set_trace()
        # import matplotlib.pyplot as plt
        # aa=np.array((self.semantic_map.global_map[:,2:3,:,:].cpu()).squeeze())
        # plt.imshow(np.transpose(aa, (1, 2, 0)))
        # plt.savefig("/home/ubuntu/zyp/home-robot/datadumpaif/ccc.jpg")
        self.semantic_map.local_pose = seq_local_pose[:, -1]
        self.semantic_map.global_pose = seq_global_pose[:, -1]
        self.semantic_map.lmb = seq_lmb[:, -1]
        self.semantic_map.origins = seq_origins[:, -1]

        goal_map = goal_map.squeeze(1).cpu().numpy()
        found_goal = found_goal.squeeze(1).cpu()
        found_rec = found_rec.squeeze(1).cpu()

        for e in range(self.num_environments):
            self.semantic_map.update_frontier_map(e, frontier_map[e][0].cpu().numpy())
            if found_goal[e] or self.timesteps_before_goal_update[e] == 0:
                self.semantic_map.update_global_goal_for_env(e, goal_map[e])
                if self.timesteps_before_goal_update[e] == 0:
                    self.timesteps_before_goal_update[e] = self.goal_update_steps
            self.timesteps[e] = self.timesteps[e] + 1
            self.timesteps_before_goal_update[e] = (
                    self.timesteps_before_goal_update[e] - 1
            )

        if debug_frontier_map:
            import matplotlib.pyplot as plt

            plt.subplot(131)
            plt.imshow(self.semantic_map.get_frontier_map(e))
            plt.subplot(132)
            plt.imshow(frontier_map[e][0])
            plt.subplot(133)
            plt.imshow(self.semantic_map.get_goal_map(e))
            plt.show()

        planner_inputs = [
            {
                "obstacle_map": self.semantic_map.get_obstacle_map(e),
                "goal_map": self.semantic_map.get_goal_map(e),
                "frontier_map": self.semantic_map.get_frontier_map(e),
                "sensor_pose": self.semantic_map.get_planner_pose_inputs(e),
                "found_goal": found_goal[e].item(),
                "found_rec": found_rec[e].item(),
            }
            for e in range(self.num_environments)
        ]
        if self.visualize:
            vis_inputs = [
                {
                    "explored_map": self.semantic_map.get_explored_map(e),
                    "semantic_map": self.semantic_map.get_semantic_map(e),
                    "been_close_map": self.semantic_map.get_been_close_map(e),
                    "timestep": self.timesteps[e],
                }
                for e in range(self.num_environments)
            ]
            if self.record_instance_ids:
                for e in range(self.num_environments):
                    vis_inputs[e]["instance_map"] = self.semantic_map.get_instance_map(
                        e
                    )

        else:
            vis_inputs = [{} for e in range(self.num_environments)]
        return planner_inputs, vis_inputs

    def reset_vectorized(self):
        """Initialize agent state."""

        self.timesteps = [0] * self.num_environments
        self.timesteps_before_goal_update = [0] * self.num_environments
        self.last_poses = [np.zeros(3)] * self.num_environments
        self.semantic_map.init_map_and_pose()
        self.episode_panorama_start_steps = self.panorama_start_steps
        self.planner.reset()

    def reset_vectorized_for_env(self, e: int):

        """Initialize agent state for a specific environment."""

        self.timesteps[e] = 0
        self.timesteps_before_goal_update[e] = 0
        self.last_poses[e] = np.zeros(3)
        self.semantic_map.init_map_and_pose_for_env(e)
        self.episode_panorama_start_steps = self.panorama_start_steps
        self.planner.reset()

    # ---------------------------------------------------------------------
    # Inference methods to interact with the robot or a single un-vectorized
    # simulation environment
    # ---------------------------------------------------------------------

    def reset(self):

        """Initialize agent state."""
        self.reset_vectorized()
        self.planner.reset()
        if self.verbose:
            print("ObjectNavAgent reset")

    def get_nav_to_recep(self):
        return None
    ########################################################
    def _preprocess_obs(self, obs: Observations):
        """Take a home-robot observation, preprocess it to put it into the correct format for the
        semantic map."""
        rgb = torch.from_numpy(obs.rgb).to(self.device)
        depth = ( torch.from_numpy(obs.depth).unsqueeze(-1).to(self.device) * 100.0 )  # m to cm
        #import pdb;pdb.set_trace()
        #print("self.store_all_categories_in_map ", self.store_all_categories_in_map)
        self.store_all_categories_in_map=True##############全种类
        if self.store_all_categories_in_map:
            semantic = obs.semantic
            obj_goal_idx = obs.task_observations["object_goal"]
            if "start_recep_goal" in obs.task_observations:
                start_recep_idx = obs.task_observations["start_recep_goal"]
            if "end_recep_goal" in obs.task_observations:
                end_recep_idx = obs.task_observations["end_recep_goal"]
        else: #别的用4填充，目标物体用1，起始容器用2，目标容器用3 
            semantic = np.full_like(obs.semantic, 4) #(640, 480) 里面的值为4
            obj_goal_idx, start_recep_idx, end_recep_idx = 1, 2, 3
            semantic[ obs.semantic == obs.task_observations["object_goal"] ] = obj_goal_idx #观测中包含task所需类别则给值123
            if "start_recep_goal" in obs.task_observations:
                semantic[obs.semantic == obs.task_observations["start_recep_goal"] ] = start_recep_idx
            if "end_recep_goal" in obs.task_observations:
                semantic[obs.semantic == obs.task_observations["end_recep_goal"] ] = end_recep_idx
        #import pdb;pdb.set_trace()
        semantic = self.one_hot_encoding[torch.from_numpy(semantic).to(self.device)]       #[640, 480, 5] 5为总类别
        obs_preprocessed = torch.cat([rgb, depth, semantic], dim=-1) #[640, 480, 3] [640, 480, 1] [640, 480, 5] 共9
       
        if self.record_instance_ids:#false
            instances = obs.task_observations["instance_map"]
            # first create a mapping to 1, 2, ... num_instances
            instance_ids = np.unique(instances)
            # map instance id to index
            instance_id_to_idx = { instance_id: idx for idx, instance_id in enumerate(instance_ids)   }
            # convert instance ids to indices, use vectorized lookup
            instances = torch.from_numpy(  np.vectorize(instance_id_to_idx.get)(instances)  ).to(self.device)
            # create a one-hot encoding
            instances = torch.eye(len(instance_ids), device=self.device)[instances]
            obs_preprocessed = torch.cat([obs_preprocessed, instances], dim=-1)

        if self.evaluate_instance_tracking:#false
            gt_instance_ids = (torch.from_numpy(obs.task_observations["gt_instance_ids"])  .to(self.device)  .long()  )
            gt_instance_ids = self.one_hot_instance_encoding[gt_instance_ids]
            obs_preprocessed = torch.cat([obs_preprocessed, gt_instance_ids], dim=-1)

        obs_preprocessed = obs_preprocessed.unsqueeze(0).permute(0, 3, 1, 2)

        curr_pose = np.array([obs.gps[0], obs.gps[1], obs.compass[0]])
        print("传感器信息：",curr_pose[0],curr_pose[1],curr_pose[2]*57.295780490442965)
        #import pdb;pdb.set_trace()

        pose_delta = torch.tensor( pu.get_rel_pose_change(curr_pose, self.last_poses[0])).unsqueeze(0)#相对姿势变化的信息
        print("姿态变化：",pose_delta[0][0],pose_delta[0][1],pose_delta[0][2]*57.295780490442965)
        self.last_poses[0] = curr_pose

        object_goal_category = None
        end_recep_goal_category = None
        start_recep_goal_category = None

        if (  "object_goal" in obs.task_observations   and obs.task_observations["object_goal"] is not None):          
            object_goal_category = torch.tensor(obj_goal_idx).unsqueeze(0)        

        if ( "start_recep_goal" in obs.task_observations  and obs.task_observations["start_recep_goal"] is not None   ):            
            start_recep_goal_category = torch.tensor(start_recep_idx).unsqueeze(0)
            
        if (  "end_recep_goal" in obs.task_observations   and obs.task_observations["end_recep_goal"] is not None  ):          
            end_recep_goal_category = torch.tensor(end_recep_idx).unsqueeze(0)

        goal_name = [obs.task_observations["goal_name"]]  
        camera_pose = obs.camera_pose

        if camera_pose is not None:
            camera_pose = torch.tensor(np.asarray(camera_pose)).unsqueeze(0)

        return (
            obs_preprocessed,
            pose_delta,
            object_goal_category,
            start_recep_goal_category,
            end_recep_goal_category,
            goal_name,
            camera_pose,
        )

    # act()
    def act(self, obs: Observations) -> Tuple[DiscreteNavigationAction, Dict[str, Any]]:
        """Act end-to-end."""
        
        # 1 - Obs preprocessing
        (
            obs_preprocessed,
            pose_delta,
            object_goal_category,
            start_recep_goal_category,
            end_recep_goal_category,
            goal_name,
            camera_pose,
        ) = self._preprocess_obs(obs)

        # 2 - Semantic mapping + policy
        #import pdb;pdb.set_trace()
        planner_inputs, vis_inputs = self.prepare_planner_inputs(
            obs_preprocessed,
            pose_delta,
            object_goal_category=object_goal_category,
            start_recep_goal_category=start_recep_goal_category,
            end_recep_goal_category=end_recep_goal_category,
            #nav_to_recep=None,
            nav_to_recep=self.get_nav_to_recep(),
            camera_pose=camera_pose,
          
        )

        # print("planner_inputs  ",planner_inputs)

        # t2 = time.time()
        # print(f"[Agent] Semantic mapping and policy time: {t2 - t1:.2f}")
        # import pdb;pdb.set_trace()
        # 3 - Planning
        closest_goal_map = None
        short_term_goal = None
        dilated_obstacle_map = None
        #if self.timesteps[0]==44:
           # import pdb;pdb.set_trace()
        if planner_inputs[0]["found_goal"]:
            self.episode_panorama_start_steps = 0
        if self.timesteps[0] < self.episode_panorama_start_steps:
            action = DiscreteNavigationAction.TURN_RIGHT
        elif self.timesteps[0] > self.max_steps:
            action = DiscreteNavigationAction.STOP
        else:
            #import pdb;pdb.set_trace()
            (
                action,
                closest_goal_map,
                short_term_goal,
                dilated_obstacle_map,
            ) = self.planner.plan(
                **planner_inputs[0],
                nav_to_recep=self.get_nav_to_recep(),
                use_dilation_for_stg=self.use_dilation_for_stg,
                timestep=self.timesteps[0],
                debug=self.verbose,
                lidar=obs.lidar
            )

        # t3 = time.time()
        # print(f"[Agent] Planning time: {t3 - t2:.2f}")
        # print(f"[Agent] Total time: {t3 - t0:.2f}")
        # print()

        vis_inputs[0]["goal_name"] = obs.task_observations["goal_name"]
        if self.visualize:
            vis_inputs[0]["semantic_frame"] = obs.task_observations["semantic_frame"]
            vis_inputs[0]["closest_goal_map"] = closest_goal_map
            vis_inputs[0]["third_person_image"] = obs.third_person_image
            vis_inputs[0]["short_term_goal"] = None
            vis_inputs[0]["dilated_obstacle_map"] = dilated_obstacle_map
            vis_inputs[0]["semantic_map_config"] = self.config["AGENT"]["SEMANTIC_MAP"]
            vis_inputs[0]["instance_memory"] = self.instance_memory
        info = {**planner_inputs[0], **vis_inputs[0]}
        #import pdb;pdb.set_trace()
        return action, info

def main():
    
    for i in range(100):
        with open("/nanhu-src/activate_inference_habitat/homerobot/obs.pkl" ,"rb") as f:
            obs=pickle.load(f)
        
        config={'habitat': {'seed': 7, 'env_task': 'GymHabitatEnv', 'env_task_gym_dependencies': [], 'env_task_gym_id': '', 'environment': {'max_episode_steps': 1250, 'max_episode_seconds': 10000000, 'iterator_options': {'cycle': True, 'shuffle': False, 'group_by_scene': True, 'num_episode_sample': -1, 'max_scene_repeat_episodes': -1, 'max_scene_repeat_steps': 10000, 'step_repetition_range': 0.8}}, 'simulator': {'type': 'OVMMSim-v0', 'action_space_config': 'v0', 'action_space_config_arguments': {}, 'forward_step_size': 0.25, 'create_renderer': False, 'requires_textures': True, 'auto_sleep': True, 'sleep_dist': 3.0, 'step_physics': True, 'concur_render': True, 'needs_markers': False, 'update_robot': True, 'scene': 'data/scene_datasets/habitat-test-scenes/van-gogh-room.glb', 'scene_dataset': 'default', 'additional_object_paths': ['data/objects/train_val/amazon_berkeley/configs/', 'data/objects/train_val/google_scanned/configs/', 'data/objects/train_val/ai2thorhab/configs/objects/', 'data/objects/train_val/hssd/configs/objects/'], 'seed': '${habitat.seed}', 'turn_angle': 10, 'tilt_angle': 15, 'default_agent_id': 0, 'debug_render': False, 'debug_render_robot': False, 'kinematic_mode': True, 'debug_render_goal': False, 'robot_joint_start_noise': 0.0, 'ctrl_freq': 120.0, 'ac_freq_ratio': 4, 'load_objs': False, 'hold_thresh': 0.15, 'grasp_impulse': 10000.0, 'agents': {'main_agent': {'height': 1.41, 'radius': 0.3, 'sim_sensors': {'head_rgb_sensor': {'type': 'HabitatSimRGBSensor', 'height': 640, 'width': 480, 'position': [0.0, 1.25, 0.0], 'orientation': [0.0, 0.0, 0.0], 'hfov': 42, 'sensor_subtype': 'PINHOLE', 'noise_model': 'None', 'noise_model_kwargs': {}, 'uuid': 'robot_head_rgb'}, 'head_depth_sensor': {'type': 'HabitatSimDepthSensor', 'height': 640, 'width': 480, 'position': [0.0, 1.25, 0.0], 'orientation': [0.0, 0.0, 0.0], 'hfov': 42, 'sensor_subtype': 'PINHOLE', 'noise_model': 'None', 'noise_model_kwargs': {}, 'min_depth': 0.0, 'max_depth': 10.0, 'normalize_depth': True, 'uuid': 'robot_head_depth'}, 'head_panoptic_sensor': {'type': 'HabitatSimSemanticSensor', 'height': 640, 'width': 480, 'position': [0.0, 1.25, 0.0], 'orientation': [0.0, 0.0, 0.0], 'hfov': 42, 'sensor_subtype': 'PINHOLE', 'noise_model': 'None', 'noise_model_kwargs': {}, 'uuid': 'robot_head_panoptic'}, 'third_rgb_sensor': {'type': 'HabitatSimRGBSensor', 'height': 512, 'width': 512, 'position': [0.0, 1.25, 0.0], 'orientation': [0.0, 0.0, 0.0], 'hfov': 90, 'sensor_subtype': 'PINHOLE', 'noise_model': 'None', 'noise_model_kwargs': {}, 'uuid': 'robot_third_rgb'}}, 'is_set_start_state': False, 'start_position': [0.0, 0.0, 0.0], 'start_rotation': [0.0, 0.0, 0.0, 1.0], 'joint_start_noise': 0.1, 'robot_urdf': 'data/robots/hab_stretch/urdf/hab_stretch.urdf', 'robot_type': 'StretchRobot', 'ik_arm_urdf': None}}, 'agents_order': ['main_agent'], 'habitat_sim_v0': {'gpu_device_id': 0, 'gpu_gpu': False, 'allow_sliding': False, 'frustum_culling': True, 'enable_physics': True, 'physics_config_file': './data/default.physics_config.json', 'leave_context_with_background_renderer': False, 'enable_gfx_replay_save': False, 'hbao_visual_effects': False, 'pbr_image_based_lighting': True}, 'ep_info': None, 'instance_ids_start': 50}, 'task': {'reward_measure': 'ovmm_place_success', 'success_measure': 'ovmm_place_success', 'success_reward': 10.0, 'slack_reward': -0.005, 'end_on_success': True, 'type': 'OVMMNavToObjTask-v0', 'lab_sensors': {'joint_sensor': {'type': 'JointSensor', 'dimensionality': 10}, 'joint_velocity_sensor': {'type': 'JointVelocitySensor', 'dimensionality': 10}, 'object_category_sensor': {'type': 'ObjectCategorySensor'}, 'start_receptacle_sensor': {'type': 'StartReceptacleSensor'}, 'goal_receptacle_sensor': {'type': 'GoalReceptacleSensor'}, 'object_segmentation_sensor': {'type': 'ObjectSegmentationSensor', 'blank_out_prob': 0.0}, 'object_embedding_sensor': {'type': 'ObjectEmbeddingSensor', 'embeddings_file': 'data/objects/clip_embeddings.pickle', 'dimensionality': 512}, 'start_recep_segmentation_sensor': {'type': 'StartRecepSegmentationSensor', 'blank_out_prob': 0.0}, 'goal_recep_segmentation_sensor': {'type': 'GoalRecepSegmentationSensor', 'blank_out_prob': 0.0}, 'robot_start_gps_sensor': {'type': 'RobotStartGPSSensor', 'dimensionality': 2}, 'robot_start_compass_sensor': {'type': 'RobotStartCompassSensor'}, 'camera_pose_sensor': {'type': 'CameraPoseSensor'}, 'end_effector_sensor': {'type': 'EEPositionSensor'}, 'relative_resting_pos_sensor': {'type': 'RelativeRestingPositionSensor'}, 'is_holding_sensor': {'type': 'IsHoldingSensor'}, 'ovmm_nav_goal_segmentation_sensor': {'type': 'OVMMNavGoalSegmentationSensor', 'blank_out_prob': 0.0}, 'receptacle_segmentation_sensor': {'type': 'ReceptacleSegmentationSensor', 'blank_out_prob': 0.0}}, 'measurements': {'ovmm_object_to_place_goal_distance': {'type': 'OVMMObjectToPlaceGoalDistance'}, 'robot_force': {'type': 'RobotForce', 'min_force': 20.0}, 'force_terminate': {'type': 'ForceTerminate', 'max_accum_force': -1.0, 'max_instant_force': -1.0}, 'robot_colls': {'type': 'RobotCollisions'}, 'does_want_terminate': {'type': 'DoesWantTerminate'}, 'num_steps': {'type': 'NumStepsMeasure'}, 'obj_anywhere_on_goal': {'type': 'ObjAnywhereOnGoal'}, 'end_effector_to_rest_distance': {'type': 'EndEffectorToRestDistance'}, 'did_pick_object': {'type': 'DidPickObjectMeasure'}, 'pick_success': {'type': 'RearrangePickSuccess', 'ee_resting_success_threshold': 100.0, 'object_goal': True}, 'picked_object_linear_vel': {'type': 'PickedObjectLinearVel'}, 'picked_object_angular_vel': {'type': 'PickedObjectAngularVel'}, 'object_at_rest': {'type': 'ObjectAtRest', 'angular_vel_thresh': 0.05, 'linear_vel_thresh': 0.005}, 'ovmm_placement_stability': {'type': 'OVMMPlacementStability', 'stability_steps': 50.0}, 'ovmm_place_success': {'type': 'OVMMPlaceSuccess', 'ee_resting_success_threshold': 100.0, 'check_stability': True}, 'dist_to_pick_goal': {'type': 'OVMMDistToPickGoal', 'use_shortest_path_cache': False}, 'rot_dist_to_pick_goal': {'type': 'OVMMRotDistToPickGoal'}, 'pick_goal_iou_coverage': {'type': 'PickGoalIoUCoverage', 'max_goal_dist': 0.1}, 'nav_to_pick_succ': {'type': 'OVMMNavToPickSucc', 'success_distance': 0.1}, 'nav_orient_to_pick_succ': {'type': 'OVMMNavOrientToPickSucc', 'must_look_at_targ': True, 'must_call_stop': False, 'success_angle_dist': 3.14, 'min_object_coverage_iou': 0.001}, 'dist_to_place_goal': {'type': 'OVMMDistToPlaceGoal', 'use_shortest_path_cache': False}, 'rot_dist_to_place_goal': {'type': 'OVMMRotDistToPlaceGoal'}, 'place_goal_iou_coverage': {'type': 'PlaceGoalIoUCoverage', 'max_goal_dist': 0.1}, 'nav_to_place_succ': {'type': 'OVMMNavToPlaceSucc', 'success_distance': 0.1}, 'nav_orient_to_place_succ': {'type': 'OVMMNavOrientToPlaceSucc', 'must_look_at_targ': True, 'must_call_stop': False, 'success_angle_dist': 3.14, 'min_object_coverage_iou': 0.001}, 'ovmm_find_object_phase_success': {'type': 'OVMMFindObjectPhaseSuccess'}, 'ovmm_pick_object_phase_success': {'type': 'OVMMPickObjectPhaseSuccess'}, 'ovmm_find_recep_phase_success': {'type': 'OVMMFindRecepPhaseSuccess'}, 'ovmm_place_object_phase_success': {'type': 'OVMMPlaceObjectPhaseSuccess'}, 'navmesh_collision': {'type': 'NavmeshCollision'}}, 'goal_sensor_uuid': 'pointgoal', 'count_obj_collisions': True, 'settle_steps': 5, 'constraint_violation_ends_episode': False, 'constraint_violation_drops_object': True, 'force_regenerate': False, 'should_save_to_cache': False, 'must_look_at_targ': True, 'object_in_hand_sample_prob': 0.0, 'min_start_distance': 3.0, 'render_target': True, 'physics_stability_steps': 1, 'num_spawn_attempts': 200, 'spawn_max_dists_to_obj': 1.0, 'base_angle_noise': 0.2618, 'spawn_reference': 'target', 'spawn_reference_sampling': 'uniform', 'ee_sample_factor': 0.2, 'ee_exclude_region': 0.0, 'base_noise': 0.05, 'spawn_region_scale': 0.2, 'joint_max_impulse': -1.0, 'desired_resting_position': [0.5, 0.0, 1.0], 'use_marker_t': True, 'cache_robot_init': False, 'success_state': 0.0, 'easy_init': False, 'should_enforce_target_within_reach': False, 'task_spec_base_path': 'habitat/task/rearrange/pddl/', 'task_spec': '', 'pddl_domain_def': 'replica_cad', 'obj_succ_thresh': 0.3, 'enable_safe_drop': False, 'art_succ_thresh': 0.15, 'robot_at_thresh': 2.0, 'filter_nav_to_tasks': [], 'actions': {'rearrange_stop': {'type': 'RearrangeStopAction'}, 'base_velocity': {'type': 'BaseWaypointTeleportAction', 'lin_speed': 10.0, 'ang_speed': 10.0, 'allow_dyn_slide': True, 'allow_back': True, 'collision_threshold': 1e-05, 'navmesh_offset': [[0.0, 0.0]], 'min_displacement': 0.1, 'max_displacement_along_axis': 1.0, 'max_turn_degrees': 180.0, 'min_turn_degrees': 5.0, 'allow_lateral_movement': True, 'allow_simultaneous_turn': True, 'discrete_movement': False, 'constraint_base_in_manip_mode': False}, 'arm_action': {'type': 'ArmAction', 'arm_controller': 'ArmRelPosReducedActionStretch', 'grip_controller': 'GazeGraspAction', 'arm_joint_mask': [1, 0, 0, 0, 1, 1, 1, 1, 1, 1], 'arm_joint_dimensionality': 7, 'grasp_thresh_dist': 0.8, 'disable_grip': False, 'max_delta_pos': 6.28, 'min_delta_pos': 0.0, 'ee_ctrl_lim': 0.015, 'should_clip': False, 'render_ee_target': False, 'gaze_distance_range': [0.1, 3.0], 'center_cone_angle_threshold': 90.0, 'center_cone_vector': [0.0, 1.0, 0.0], 'wrong_grasp_should_end': False, 'gaze_distance_from': 'camera', 'gaze_center_square_width': 1.0, 'grasp_threshold': 0.0, 'oracle_snap': True}}, 'start_in_manip_mode': False, 'pick_init': False, 'place_init': False, 'episode_init': True, 'camera_tilt': -0.5236, 'receptacle_categories_file': 'data/objects/hssd-receptacles.csv'}, 'dataset': {'type': 'OVMMDataset-v0', 'split': 'val', 'scenes_dir': 'data/hssd-hab/', 'content_scenes': ['*'], 'data_path': 'data/datasets/ovmm/{split}/episodes.json.gz', 'viewpoints_matrix_path': 'data/datasets/ovmm/{split}/viewpoints.npy', 'transformations_matrix_path': 'data/datasets/ovmm/{split}/transformations.npy', 'episode_indices_range': None, 'episode_ids': [66]}, 'gym': {'auto_name': 'NavToObj', 'obs_keys': ['robot_head_depth', 'robot_head_rgb', 'robot_third_rgb', 'start_receptacle', 'goal_receptacle', 'robot_start_gps', 'robot_start_compass', 'joint', 'object_segmentation', 'start_recep_segmentation', 'goal_recep_segmentation', 'object_category', 'camera_pose', 'object_embedding', 'is_holding', 'relative_resting_position', 'ovmm_nav_goal_segmentation', 'receptacle_segmentation', 'robot_head_panoptic'], 'action_keys': None, 'achieved_goal_keys': [], 'desired_goal_keys': []}}, 'habitat_baselines': {'trainer_name': 'ppo', 'torch_gpu_id': 0, 'video_render_views': ['third_rgb_sensor'], 'tensorboard_dir': 'tb', 'writer_type': 'tb', 'video_dir': 'video_dir', 'video_fps': 30, 'test_episode_count': -1, 'eval_ckpt_path_dir': 'data/new_checkpoints', 'num_environments': 1, 'num_processes': -1, 'checkpoint_folder': 'data/new_checkpoints', 'num_updates': 10000, 'num_checkpoints': 20, 'checkpoint_interval': -1, 'total_num_steps': -1.0, 'log_interval': 10, 'log_file': 'train.log', 'force_blind_policy': False, 'verbose': False, 'eval_keys_to_include_in_name': ['reward', 'force', 'success'], 'force_torch_single_threaded': True, 'wb': {'project_name': '', 'entity': '', 'group': '', 'run_name': ''}, 'load_resume_state_config': True, 'eval': {'split': 'val', 'use_ckpt_config': True, 'should_load_ckpt': True, 'evals_per_ep': 1, 'video_option': ['disk'], 'extra_sim_sensors': {}}, 'profiling': {'capture_start_step': -1, 'num_steps_to_capture': -1}, 'rl': {'preemption': {'append_slurm_job_id': False, 'save_resume_state_interval': 100, 'save_state_batch_only': False}, 'policy': {'name': 'PointNavResNetPolicy', 'action_distribution_type': 'categorical', 'action_dist': {'use_log_std': True, 'use_softplus': False, 'std_init': '???', 'log_std_init': 0.0, 'use_std_param': False, 'clamp_std': True, 'min_std': 1e-06, 'max_std': 1, 'min_log_std': -5, 'max_log_std': 2, 'action_activation': 'tanh', 'scheduled_std': False}, 'obs_transforms': {}, 'hierarchical_policy': '???', 'ovrl': False, 'no_downscaling': False, 'use_augmentations': False, 'deterministic_actions': False}, 'ppo': {'clip_param': 0.2, 'ppo_epoch': 4, 'num_mini_batch': 2, 'value_loss_coef': 0.5, 'entropy_coef': 0.01, 'lr': 0.00025, 'eps': 1e-05, 'max_grad_norm': 0.5, 'num_steps': 5, 'use_gae': True, 'use_linear_lr_decay': False, 'use_linear_clip_decay': False, 'gamma': 0.99, 'tau': 0.95, 'reward_window_size': 50, 'use_normalized_advantage': False, 'hidden_size': 512, 'entropy_target_factor': 0.0, 'use_adaptive_entropy_pen': False, 'use_clipped_value_loss': True, 'use_double_buffered_sampler': False}, 'ddppo': {'sync_frac': 0.6, 'distrib_backend': 'GLOO', 'rnn_type': 'GRU', 'num_recurrent_layers': 1, 'backbone': 'resnet18', 'pretrained_weights': 'data/ddppo-models/gibson-2plus-resnet50.pth', 'pretrained': False, 'pretrained_encoder': False, 'train_encoder': True, 'reset_critic': True, 'force_distributed': False}, 'ver': {'variable_experience': True, 'num_inference_workers': 2, 'overlap_rollouts_and_learn': False}, 'auxiliary_losses': {}}}, 'NO_GPU': 0, 'NUM_ENVIRONMENTS': 1, 'DUMP_LOCATION': 'datadump', 'EXP_NAME': 'eval_hssd', 'VISUALIZE': 0, 'PRINT_IMAGES': 1, 'GROUND_TRUTH_SEMANTICS': 1, 'seed': 0, 'SHOW_RL_OBS': False, 'ENVIRONMENT': {'forward': 0.25, 'forward_1': 0.09, 'forward_2': 0.1, 'forward_3': 0.15, 'forward_4': 0.2, 'turn_angle': 30.0, 'frame_height': 640, 'frame_width': 480, 'camera_height': 1.31, 'hfov': 42.0, 'min_depth': 0.0, 'max_depth': 10.0, 'num_receptacles': 21, 'category_map_file': '/home/ubuntu/zyp/home-robot/projects/real_world_ovmm/configs/example_cat_map.json', 'use_detic_viz': False, 'evaluate_instance_tracking': False}, 'EVAL_VECTORIZED': {'simulator_gpu_ids': [1, 2, 3, 4, 5, 6, 7], 'split': 'val', 'num_episodes_per_env': None, 'record_videos': 0, 'record_planner_videos': 0, 'metrics_save_freq': 1}, 'AGENT': {'max_steps': 10000, 'panorama_start': 1, 'exploration_strategy': 'seen_frontier', 'radius': 0.05, 'fall_wait_steps': 200, 'store_all_categories': False, 'detection_module': 'detic', 'SEMANTIC_MAP': {'semantic_categories': 'rearrange', 'num_sem_categories': 24, 'map_size_cm': 4800, 'map_resolution': 5, 'vision_range': 100, 'global_downscaling': 2, 'du_scale': 4, 'cat_pred_threshold': 1.0, 'exp_pred_threshold': 1.0, 'map_pred_threshold': 1.0, 'min_depth': 0.5, 'max_depth': 3.5, 'been_close_to_radius': 100, 'explored_radius': 150, 'must_explore_close': False, 'min_obs_height_cm': 10, 'dilate_obstacles': True, 'dilate_size': 1, 'dilate_iter': 1, 'record_instance_ids': False, 'max_instances': 0}, 'SKILLS': {'GAZE_OBJ': {'type': 'heuristic', 'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/gaze_at_obj.pth', 'rl_config': 'projects/habitat_ovmm/configs/agent/skills/gaze_rl.yaml', 'gym_obs_keys': ['robot_head_depth', 'object_embedding', 'object_segmentation', 'joint', 'is_holding', 'relative_resting_position'], 'allowed_actions': ['arm_action', 'base_velocity'], 'arm_joint_mask': [0, 0, 0, 0, 0, 0, 1], 'max_displacement': 0.25, 'max_turn_degrees': 30.0, 'min_turn_degrees': 5.0, 'min_displacement': 0.1, 'sensor_height': 160, 'sensor_width': 120, 'nav_goal_seg_channels': 1, 'terminate_condition': 'grip', 'grip_threshold': 0.8, 'max_joint_delta': 0.1, 'min_joint_delta': 0.02}, 'PICK': {'type': 'oracle'}, 'NAV_TO_OBJ': {'type': 'heuristic', 'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/nav_to_obj.pth', 'rl_config': 'projects/habitat_ovmm/configs/agent/skills/nav_to_obj_rl.yaml', 'gym_obs_keys': ['robot_head_depth', 'object_embedding', 'ovmm_nav_goal_segmentation', 'receptacle_segmentation', 'start_receptacle', 'robot_start_gps', 'robot_start_compass', 'joint'], 'allowed_actions': ['stop', 'move_forward', 'turn_left', 'turn_right'], 'arm_joint_mask': [0, 0, 0, 0, 0, 0, 0], 'max_displacement': 0.25, 'max_turn_degrees': 30.0, 'min_turn_degrees': 5.0, 'min_displacement': 0.1, 'sensor_height': 160, 'discrete_forward': 0.25, 'discrete_turn_degrees': 10, 'sensor_width': 120, 'terminate_condition': 'discrete_stop', 'nav_goal_seg_channels': 2}, 'NAV_TO_REC': {'type': 'heuristic', 'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/nav_to_rec.pth', 'rl_config': 'projects/habitat_ovmm/configs/agent/skills/nav_to_obj_rl.yaml', 'gym_obs_keys': ['robot_head_depth', 'ovmm_nav_goal_segmentation', 'receptacle_segmentation', 'goal_receptacle', 'robot_start_gps', 'robot_start_compass', 'joint'], 'allowed_actions': ['stop', 'move_forward', 'turn_left', 'turn_right'], 'arm_joint_mask': [0, 0, 0, 0, 0, 0, 0], 'max_displacement': 0.25, 'max_turn_degrees': 30.0, 'min_turn_degrees': 5.0, 'min_displacement': 0.1, 'discrete_forward': 0.25, 'discrete_turn_degrees': 10, 'sensor_height': 160, 'sensor_width': 120, 'terminate_condition': 'discrete_stop', 'nav_goal_seg_channels': 1}, 'PLACE': {'type': 'heuristic', 'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/place.pth', 'rl_config': 'projects/habitat_ovmm/configs/agent/skills/place_rl.yaml', 'gym_obs_keys': ['robot_head_depth', 'goal_receptacle', 'joint', 'goal_recep_segmentation', 'is_holding', 'object_embedding'], 'allowed_actions': ['arm_action', 'base_velocity', 'manipulation_mode'], 'arm_joint_mask': [1, 1, 1, 1, 1, 0, 0], 'max_displacement': 0.25, 'max_turn_degrees': 30.0, 'min_turn_degrees': 5.0, 'min_displacement': 0.1, 'sensor_height': 160, 'sensor_width': 120, 'nav_goal_seg_channels': 1, 'terminate_condition': 'ungrip', 'grip_threshold': -0.8, 'manip_mode_threshold': 0.8, 'constraint_base_in_manip_mode': True, 'max_joint_delta': 0.1, 'min_joint_delta': 0.02}}, 'skip_skills': {'nav_to_obj': False, 'nav_to_rec': True, 'gaze_at_obj': True, 'gaze_at_rec': True, 'pick': True, 'place': True}, 'PLANNER': {'collision_threshold': 0.2, 'obs_dilation_selem_radius': 3, 'goal_dilation_selem_radius': 10, 'step_size': 5, 'use_dilation_for_stg': False, 'min_obs_dilation_selem_radius': 1, 'map_downsample_factor': 1, 'map_update_frequency': 1, 'discrete_actions': True, 'verbose': False}}}
        #print(config)
        
        
        agent=ObjectNavAgent(config)
        agent.reset_vectorized()        
        agent.act(obs)



if __name__ == "__main__":
    main()

