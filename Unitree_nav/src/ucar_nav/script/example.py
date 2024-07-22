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
sys.path.insert(0,"/home/lijixiang/Thu_unitree/Unitree_nav")
import home_robot
from home_robot.core.interfaces import DiscreteNavigationAction, Observations
from home_robot.agent.objectnav_agent.objectnav_agent import ObjectNavAgent
from home_robot.agent.ovmm_agent.ovmm_agent import OpenVocabManipAgent
from scipy.spatial.transform import Rotation as R
#from costmap_2d import Costmap2DROS, Costmap2D
#import tf
config = {'habitat': {'seed': 7, 'env_task': 'GymHabitatEnv', 'env_task_gym_dependencies': [], 'env_task_gym_id': '',
                      'environment': {'max_episode_steps': 1250, 'max_episode_seconds': 10000000,
                                      'iterator_options': {'cycle': True, 'shuffle': False, 'group_by_scene': True,
                                                           'num_episode_sample': -1, 'max_scene_repeat_episodes': -1,
                                                           'max_scene_repeat_steps': 10000,
                                                           'step_repetition_range': 0.8}},
                      'simulator': {'type': 'OVMMSim-v0', 'action_space_config': 'v0',
                                    'action_space_config_arguments': {}, 'forward_step_size': 0.25,
                                    'create_renderer': False, 'requires_textures': True, 'auto_sleep': True,
                                    'sleep_dist': 3.0, 'step_physics': True, 'concur_render': True,
                                    'needs_markers': False, 'update_robot': True,
                                    'scene': 'data/scene_datasets/habitat-test-scenes/van-gogh-room.glb',
                                    'scene_dataset': 'default',
                                    'additional_object_paths': ['data/objects/train_val/amazon_berkeley/configs/',
                                                                'data/objects/train_val/google_scanned/configs/',
                                                                'data/objects/train_val/ai2thorhab/configs/objects/',
                                                                'data/objects/train_val/hssd/configs/objects/'],
                                    'seed': '${habitat.seed}', 'turn_angle': 10, 'tilt_angle': 15,
                                    'default_agent_id': 0, 'debug_render': False, 'debug_render_robot': False,
                                    'kinematic_mode': True, 'debug_render_goal': False, 'robot_joint_start_noise': 0.0,
                                    'ctrl_freq': 120.0, 'ac_freq_ratio': 4, 'load_objs': False, 'hold_thresh': 0.15,
                                    'grasp_impulse': 10000.0, 'agents': {'main_agent': {'height': 1.41, 'radius': 0.3,
                                                                                        'sim_sensors': {
                                                                                            'head_rgb_sensor': {
                                                                                                'type': 'HabitatSimRGBSensor',
                                                                                                'height': 640,
                                                                                                'width': 480,
                                                                                                'position': [0.0, 1.25,
                                                                                                             0.0],
                                                                                                'orientation': [0.0,
                                                                                                                0.0,
                                                                                                                0.0],
                                                                                                'hfov': 47,
                                                                                                'sensor_subtype': 'PINHOLE',
                                                                                                'noise_model': 'None',
                                                                                                'noise_model_kwargs': {},
                                                                                                'uuid': 'robot_head_rgb'},
                                                                                            'head_depth_sensor': {
                                                                                                'type': 'HabitatSimDepthSensor',
                                                                                                'height': 640,
                                                                                                'width': 480,
                                                                                                'position': [0.0, 1.25,
                                                                                                             0.0],
                                                                                                'orientation': [0.0,
                                                                                                                0.0,
                                                                                                                0.0],
                                                                                                'hfov': 47,
                                                                                                'sensor_subtype': 'PINHOLE',
                                                                                                'noise_model': 'None',
                                                                                                'noise_model_kwargs': {},
                                                                                                'min_depth': 0.0,
                                                                                                'max_depth': 10.0,
                                                                                                'normalize_depth': True,
                                                                                                'uuid': 'robot_head_depth'},
                                                                                            'head_panoptic_sensor': {
                                                                                                'type': 'HabitatSimSemanticSensor',
                                                                                                'height': 640,
                                                                                                'width': 480,
                                                                                                'position': [0.0, 1.25,
                                                                                                             0.0],
                                                                                                'orientation': [0.0,
                                                                                                                0.0,
                                                                                                                0.0],
                                                                                                'hfov': 47,
                                                                                                'sensor_subtype': 'PINHOLE',
                                                                                                'noise_model': 'None',
                                                                                                'noise_model_kwargs': {},
                                                                                                'uuid': 'robot_head_panoptic'},
                                                                                            'third_rgb_sensor': {
                                                                                                'type': 'HabitatSimRGBSensor',
                                                                                                'height': 512,
                                                                                                'width': 512,
                                                                                                'position': [0.0, 1.25,
                                                                                                             0.0],
                                                                                                'orientation': [0.0,
                                                                                                                0.0,
                                                                                                                0.0],
                                                                                                'hfov': 90,
                                                                                                'sensor_subtype': 'PINHOLE',
                                                                                                'noise_model': 'None',
                                                                                                'noise_model_kwargs': {},
                                                                                                'uuid': 'robot_third_rgb'}},
                                                                                        'is_set_start_state': False,
                                                                                        'start_position': [0.0, 0.0,
                                                                                                           0.0],
                                                                                        'start_rotation': [0.0, 0.0,
                                                                                                           0.0, 1.0],
                                                                                        'joint_start_noise': 0.1,
                                                                                        'robot_urdf': 'data/robots/hab_stretch/urdf/hab_stretch.urdf',
                                                                                        'robot_type': 'StretchRobot',
                                                                                        'ik_arm_urdf': None}},
                                    'agents_order': ['main_agent'],
                                    'habitat_sim_v0': {'gpu_device_id': 0, 'gpu_gpu': False, 'allow_sliding': False,
                                                       'frustum_culling': True, 'enable_physics': True,
                                                       'physics_config_file': './data/default.physics_config.json',
                                                       'leave_context_with_background_renderer': False,
                                                       'enable_gfx_replay_save': False, 'hbao_visual_effects': False,
                                                       'pbr_image_based_lighting': True}, 'ep_info': None,
                                    'instance_ids_start': 50},
                      'task': {'reward_measure': 'ovmm_place_success', 'success_measure': 'ovmm_place_success',
                               'success_reward': 10.0, 'slack_reward': -0.005, 'end_on_success': True,
                               'type': 'OVMMNavToObjTask-v0',
                               'lab_sensors': {'joint_sensor': {'type': 'JointSensor', 'dimensionality': 10},
                                               'joint_velocity_sensor': {'type': 'JointVelocitySensor',
                                                                         'dimensionality': 10},
                                               'object_category_sensor': {'type': 'ObjectCategorySensor'},
                                               'start_receptacle_sensor': {'type': 'StartReceptacleSensor'},
                                               'goal_receptacle_sensor': {'type': 'GoalReceptacleSensor'},
                                               'object_segmentation_sensor': {'type': 'ObjectSegmentationSensor',
                                                                              'blank_out_prob': 0.0},
                                               'object_embedding_sensor': {'type': 'ObjectEmbeddingSensor',
                                                                           'embeddings_file': 'data/objects/clip_embeddings.pickle',
                                                                           'dimensionality': 512},
                                               'start_recep_segmentation_sensor': {
                                                   'type': 'StartRecepSegmentationSensor', 'blank_out_prob': 0.0},
                                               'goal_recep_segmentation_sensor': {'type': 'GoalRecepSegmentationSensor',
                                                                                  'blank_out_prob': 0.0},
                                               'robot_start_gps_sensor': {'type': 'RobotStartGPSSensor',
                                                                          'dimensionality': 2},
                                               'robot_start_compass_sensor': {'type': 'RobotStartCompassSensor'},
                                               'camera_pose_sensor': {'type': 'CameraPoseSensor'},
                                               'end_effector_sensor': {'type': 'EEPositionSensor'},
                                               'relative_resting_pos_sensor': {'type': 'RelativeRestingPositionSensor'},
                                               'is_holding_sensor': {'type': 'IsHoldingSensor'},
                                               'ovmm_nav_goal_segmentation_sensor': {
                                                   'type': 'OVMMNavGoalSegmentationSensor', 'blank_out_prob': 0.0},
                                               'receptacle_segmentation_sensor': {
                                                   'type': 'ReceptacleSegmentationSensor', 'blank_out_prob': 0.0}},
                               'measurements': {
                                   'ovmm_object_to_place_goal_distance': {'type': 'OVMMObjectToPlaceGoalDistance'},
                                   'robot_force': {'type': 'RobotForce', 'min_force': 20.0},
                                   'force_terminate': {'type': 'ForceTerminate', 'max_accum_force': -1.0,
                                                       'max_instant_force': -1.0},
                                   'robot_colls': {'type': 'RobotCollisions'},
                                   'does_want_terminate': {'type': 'DoesWantTerminate'},
                                   'num_steps': {'type': 'NumStepsMeasure'},
                                   'obj_anywhere_on_goal': {'type': 'ObjAnywhereOnGoal'},
                                   'end_effector_to_rest_distance': {'type': 'EndEffectorToRestDistance'},
                                   'did_pick_object': {'type': 'DidPickObjectMeasure'},
                                   'pick_success': {'type': 'RearrangePickSuccess',
                                                    'ee_resting_success_threshold': 100.0, 'object_goal': True},
                                   'picked_object_linear_vel': {'type': 'PickedObjectLinearVel'},
                                   'picked_object_angular_vel': {'type': 'PickedObjectAngularVel'},
                                   'object_at_rest': {'type': 'ObjectAtRest', 'angular_vel_thresh': 0.05,
                                                      'linear_vel_thresh': 0.005},
                                   'ovmm_placement_stability': {'type': 'OVMMPlacementStability',
                                                                'stability_steps': 50.0},
                                   'ovmm_place_success': {'type': 'OVMMPlaceSuccess',
                                                          'ee_resting_success_threshold': 100.0,
                                                          'check_stability': True},
                                   'dist_to_pick_goal': {'type': 'OVMMDistToPickGoal',
                                                         'use_shortest_path_cache': False},
                                   'rot_dist_to_pick_goal': {'type': 'OVMMRotDistToPickGoal'},
                                   'pick_goal_iou_coverage': {'type': 'PickGoalIoUCoverage', 'max_goal_dist': 0.1},
                                   'nav_to_pick_succ': {'type': 'OVMMNavToPickSucc', 'success_distance': 0.1},
                                   'nav_orient_to_pick_succ': {'type': 'OVMMNavOrientToPickSucc',
                                                               'must_look_at_targ': True, 'must_call_stop': False,
                                                               'success_angle_dist': 3.14,
                                                               'min_object_coverage_iou': 0.001},
                                   'dist_to_place_goal': {'type': 'OVMMDistToPlaceGoal',
                                                          'use_shortest_path_cache': False},
                                   'rot_dist_to_place_goal': {'type': 'OVMMRotDistToPlaceGoal'},
                                   'place_goal_iou_coverage': {'type': 'PlaceGoalIoUCoverage', 'max_goal_dist': 0.1},
                                   'nav_to_place_succ': {'type': 'OVMMNavToPlaceSucc', 'success_distance': 0.1},
                                   'nav_orient_to_place_succ': {'type': 'OVMMNavOrientToPlaceSucc',
                                                                'must_look_at_targ': True, 'must_call_stop': False,
                                                                'success_angle_dist': 3.14,
                                                                'min_object_coverage_iou': 0.001},
                                   'ovmm_find_object_phase_success': {'type': 'OVMMFindObjectPhaseSuccess'},
                                   'ovmm_pick_object_phase_success': {'type': 'OVMMPickObjectPhaseSuccess'},
                                   'ovmm_find_recep_phase_success': {'type': 'OVMMFindRecepPhaseSuccess'},
                                   'ovmm_place_object_phase_success': {'type': 'OVMMPlaceObjectPhaseSuccess'},
                                   'navmesh_collision': {'type': 'NavmeshCollision'}}, 'goal_sensor_uuid': 'pointgoal',
                               'count_obj_collisions': True, 'settle_steps': 5,
                               'constraint_violation_ends_episode': False, 'constraint_violation_drops_object': True,
                               'force_regenerate': False, 'should_save_to_cache': False, 'must_look_at_targ': True,
                               'object_in_hand_sample_prob': 0.0, 'min_start_distance': 3.0, 'render_target': True,
                               'physics_stability_steps': 1, 'num_spawn_attempts': 200, 'spawn_max_dists_to_obj': 1.0,
                               'base_angle_noise': 0.2618, 'spawn_reference': 'target',
                               'spawn_reference_sampling': 'uniform', 'ee_sample_factor': 0.2, 'ee_exclude_region': 0.0,
                               'base_noise': 0.05, 'spawn_region_scale': 0.2, 'joint_max_impulse': -1.0,
                               'desired_resting_position': [0.5, 0.0, 1.0], 'use_marker_t': True,
                               'cache_robot_init': False, 'success_state': 0.0, 'easy_init': False,
                               'should_enforce_target_within_reach': False,
                               'task_spec_base_path': 'habitat/task/rearrange/pddl/', 'task_spec': '',
                               'pddl_domain_def': 'replica_cad', 'obj_succ_thresh': 0.3, 'enable_safe_drop': False,
                               'art_succ_thresh': 0.15, 'robot_at_thresh': 2.0, 'filter_nav_to_tasks': [],
                               'actions': {'rearrange_stop': {'type': 'RearrangeStopAction'},
                                           'base_velocity': {'type': 'BaseWaypointTeleportAction', 'lin_speed': 10.0,
                                                             'ang_speed': 10.0, 'allow_dyn_slide': True,
                                                             'allow_back': True, 'collision_threshold': 1e-05,
                                                             'navmesh_offset': [[0.0, 0.0]], 'min_displacement': 0.1,
                                                             'max_displacement_along_axis': 1.0,
                                                             'max_turn_degrees': 180.0, 'min_turn_degrees': 5.0,
                                                             'allow_lateral_movement': True,
                                                             'allow_simultaneous_turn': True,
                                                             'discrete_movement': False,
                                                             'constraint_base_in_manip_mode': False},
                                           'arm_action': {'type': 'ArmAction',
                                                          'arm_controller': 'ArmRelPosReducedActionStretch',
                                                          'grip_controller': 'GazeGraspAction',
                                                          'arm_joint_mask': [1, 0, 0, 0, 1, 1, 1, 1, 1, 1],
                                                          'arm_joint_dimensionality': 7, 'grasp_thresh_dist': 0.8,
                                                          'disable_grip': False, 'max_delta_pos': 6.28,
                                                          'min_delta_pos': 0.0, 'ee_ctrl_lim': 0.015,
                                                          'should_clip': False, 'render_ee_target': False,
                                                          'gaze_distance_range': [0.1, 3.0],
                                                          'center_cone_angle_threshold': 90.0,
                                                          'center_cone_vector': [0.0, 1.0, 0.0],
                                                          'wrong_grasp_should_end': False,
                                                          'gaze_distance_from': 'camera',
                                                          'gaze_center_square_width': 1.0, 'grasp_threshold': 0.0,
                                                          'oracle_snap': True}}, 'start_in_manip_mode': False,
                               'pick_init': False, 'place_init': False, 'episode_init': True, 'camera_tilt': -0.5236,
                               'receptacle_categories_file': 'data/objects/hssd-receptacles.csv'},
                      'dataset': {'type': 'OVMMDataset-v0', 'split': 'val', 'scenes_dir': 'data/hssd-hab/',
                                  'content_scenes': ['*'], 'data_path': 'data/datasets/ovmm/{split}/episodes.json.gz',
                                  'viewpoints_matrix_path': 'data/datasets/ovmm/{split}/viewpoints.npy',
                                  'transformations_matrix_path': 'data/datasets/ovmm/{split}/transformations.npy',
                                  'episode_indices_range': None, 'episode_ids': [66]}, 'gym': {'auto_name': 'NavToObj',
                                                                                               'obs_keys': [
                                                                                                   'robot_head_depth',
                                                                                                   'robot_head_rgb',
                                                                                                   'robot_third_rgb',
                                                                                                   'start_receptacle',
                                                                                                   'goal_receptacle',
                                                                                                   'robot_start_gps',
                                                                                                   'robot_start_compass',
                                                                                                   'joint',
                                                                                                   'object_segmentation',
                                                                                                   'start_recep_segmentation',
                                                                                                   'goal_recep_segmentation',
                                                                                                   'object_category',
                                                                                                   'camera_pose',
                                                                                                   'object_embedding',
                                                                                                   'is_holding',
                                                                                                   'relative_resting_position',
                                                                                                   'ovmm_nav_goal_segmentation',
                                                                                                   'receptacle_segmentation',
                                                                                                   'robot_head_panoptic'],
                                                                                               'action_keys': None,
                                                                                               'achieved_goal_keys': [],
                                                                                               'desired_goal_keys': []}},
          'habitat_baselines': {'trainer_name': 'ppo', 'torch_gpu_id': 0, 'video_render_views': ['third_rgb_sensor'],
                                'tensorboard_dir': 'tb', 'writer_type': 'tb', 'video_dir': 'video_dir', 'video_fps': 30,
                                'test_episode_count': -1, 'eval_ckpt_path_dir': 'data/new_checkpoints',
                                'num_environments': 1, 'num_processes': -1, 'checkpoint_folder': 'data/new_checkpoints',
                                'num_updates': 10000, 'num_checkpoints': 20, 'checkpoint_interval': -1,
                                'total_num_steps': -1.0, 'log_interval': 10, 'log_file': 'train.log',
                                'force_blind_policy': False, 'verbose': False,
                                'eval_keys_to_include_in_name': ['reward', 'force', 'success'],
                                'force_torch_single_threaded': True,
                                'wb': {'project_name': '', 'entity': '', 'group': '', 'run_name': ''},
                                'load_resume_state_config': True,
                                'eval': {'split': 'val', 'use_ckpt_config': True, 'should_load_ckpt': True,
                                         'evals_per_ep': 1, 'video_option': ['disk'], 'extra_sim_sensors': {}},
                                'profiling': {'capture_start_step': -1, 'num_steps_to_capture': -1}, 'rl': {
                  'preemption': {'append_slurm_job_id': False, 'save_resume_state_interval': 100,
                                 'save_state_batch_only': False},
                  'policy': {'name': 'PointNavResNetPolicy', 'action_distribution_type': 'categorical',
                             'action_dist': {'use_log_std': True, 'use_softplus': False, 'std_init': '???',
                                             'log_std_init': 0.0, 'use_std_param': False, 'clamp_std': True,
                                             'min_std': 1e-06, 'max_std': 1, 'min_log_std': -5, 'max_log_std': 2,
                                             'action_activation': 'tanh', 'scheduled_std': False}, 'obs_transforms': {},
                             'hierarchical_policy': '???', 'ovrl': False, 'no_downscaling': False,
                             'use_augmentations': False, 'deterministic_actions': False},
                  'ppo': {'clip_param': 0.2, 'ppo_epoch': 4, 'num_mini_batch': 2, 'value_loss_coef': 0.5,
                          'entropy_coef': 0.01, 'lr': 0.00025, 'eps': 1e-05, 'max_grad_norm': 0.5, 'num_steps': 5,
                          'use_gae': True, 'use_linear_lr_decay': False, 'use_linear_clip_decay': False, 'gamma': 0.99,
                          'tau': 0.95, 'reward_window_size': 50, 'use_normalized_advantage': False, 'hidden_size': 512,
                          'entropy_target_factor': 0.0, 'use_adaptive_entropy_pen': False,
                          'use_clipped_value_loss': True, 'use_double_buffered_sampler': False},
                  'ddppo': {'sync_frac': 0.6, 'distrib_backend': 'GLOO', 'rnn_type': 'GRU', 'num_recurrent_layers': 1,
                            'backbone': 'resnet18', 'pretrained_weights': 'data/ddppo-models/gibson-2plus-resnet50.pth',
                            'pretrained': False, 'pretrained_encoder': False, 'train_encoder': True,
                            'reset_critic': True, 'force_distributed': False},
                  'ver': {'variable_experience': True, 'num_inference_workers': 2, 'overlap_rollouts_and_learn': False},
                  'auxiliary_losses': {}}}, 'NO_GPU': 0, 'NUM_ENVIRONMENTS': 1, 'DUMP_LOCATION': 'datadump',
          'EXP_NAME': 'eval_hssd', 'VISUALIZE': 0, 'PRINT_IMAGES': 1, 'GROUND_TRUTH_SEMANTICS': 0, 'seed': 0,
          'SHOW_RL_OBS': False,
          'ENVIRONMENT': {'forward': 0.25, 'forward_1': 0.09, 'forward_2': 0.1, 'forward_3': 0.15, 'forward_4': 0.2,
                          'turn_angle': 30.0, 'frame_height': 640, 'frame_width': 480, 'camera_height': 1.31,
                          'hfov': 47.0, 'min_depth': 0.0, 'max_depth': 10.0, 'num_receptacles': 21,
                          'category_map_file': '/home/lijixiang/poc/nanhu-src/example_cat_map.json',
                          'use_detic_viz': False, 'evaluate_instance_tracking': False},
          'EVAL_VECTORIZED': {'simulator_gpu_ids': [1, 2, 3, 4, 5, 6, 7], 'split': 'val', 'num_episodes_per_env': None,
                              'record_videos': 0, 'record_planner_videos': 0, 'metrics_save_freq': 1},
          'AGENT': {'max_steps': 10000, 'panorama_start': 1, 'exploration_strategy': 'been_close_to_frontier', 'radius': 0.05,
                    'fall_wait_steps': 200, 'store_all_categories': False, 'detection_module': 'detic',
                    'SEMANTIC_MAP': {'semantic_categories': 'rearrange', 'num_sem_categories': 5, 'map_size_cm': 4800,
                                     'map_resolution': 5, 'vision_range': 100, 'global_downscaling': 2, 'du_scale': 4,
                                     'cat_pred_threshold': 1.0, 'exp_pred_threshold': 1.0, 'map_pred_threshold': 1.0,
                                     'min_depth': 0.5, 'max_depth': 3.5, 'been_close_to_radius': 100,
                                     'explored_radius': 150, 'must_explore_close': False, 'min_obs_height_cm': 10,
                                     'dilate_obstacles': True, 'dilate_size': 1, 'dilate_iter': 1,
                                     'record_instance_ids': False, 'max_instances': 0}, 'SKILLS': {
                  'GAZE_OBJ': {'type': 'heuristic',
                               'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/gaze_at_obj.pth',
                               'rl_config': 'projects/habitat_ovmm/configs/agent/skills/gaze_rl.yaml',
                               'gym_obs_keys': ['robot_head_depth', 'object_embedding', 'object_segmentation', 'joint',
                                                'is_holding', 'relative_resting_position'],
                               'allowed_actions': ['arm_action', 'base_velocity'],
                               'arm_joint_mask': [0, 0, 0, 0, 0, 0, 1], 'max_displacement': 0.25,
                               'max_turn_degrees': 30.0, 'min_turn_degrees': 5.0, 'min_displacement': 0.1,
                               'sensor_height': 160, 'sensor_width': 120, 'nav_goal_seg_channels': 1,
                               'terminate_condition': 'grip', 'grip_threshold': 0.8, 'max_joint_delta': 0.1,
                               'min_joint_delta': 0.02}, 'PICK': {'type': 'oracle'}, 'NAV_TO_OBJ': {'type': 'heuristic',
                                                                                                    'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/nav_to_obj.pth',
                                                                                                    'rl_config': 'projects/habitat_ovmm/configs/agent/skills/nav_to_obj_rl.yaml',
                                                                                                    'gym_obs_keys': [
                                                                                                        'robot_head_depth',
                                                                                                        'object_embedding',
                                                                                                        'ovmm_nav_goal_segmentation',
                                                                                                        'receptacle_segmentation',
                                                                                                        'start_receptacle',
                                                                                                        'robot_start_gps',
                                                                                                        'robot_start_compass',
                                                                                                        'joint'],
                                                                                                    'allowed_actions': [
                                                                                                        'stop',
                                                                                                        'move_forward',
                                                                                                        'turn_left',
                                                                                                        'turn_right'],
                                                                                                    'arm_joint_mask': [
                                                                                                        0, 0, 0, 0, 0,
                                                                                                        0, 0],
                                                                                                    'max_displacement': 0.25,
                                                                                                    'max_turn_degrees': 30.0,
                                                                                                    'min_turn_degrees': 5.0,
                                                                                                    'min_displacement': 0.1,
                                                                                                    'sensor_height': 160,
                                                                                                    'discrete_forward': 0.25,
                                                                                                    'discrete_turn_degrees': 10,
                                                                                                    'sensor_width': 120,
                                                                                                    'terminate_condition': 'discrete_stop',
                                                                                                    'nav_goal_seg_channels': 2},
                  'NAV_TO_REC': {'type': 'heuristic',
                                 'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/nav_to_rec.pth',
                                 'rl_config': 'projects/habitat_ovmm/configs/agent/skills/nav_to_obj_rl.yaml',
                                 'gym_obs_keys': ['robot_head_depth', 'ovmm_nav_goal_segmentation',
                                                  'receptacle_segmentation', 'goal_receptacle', 'robot_start_gps',
                                                  'robot_start_compass', 'joint'],
                                 'allowed_actions': ['stop', 'move_forward', 'turn_left', 'turn_right'],
                                 'arm_joint_mask': [0, 0, 0, 0, 0, 0, 0], 'max_displacement': 0.25,
                                 'max_turn_degrees': 30.0, 'min_turn_degrees': 5.0, 'min_displacement': 0.1,
                                 'discrete_forward': 0.25, 'discrete_turn_degrees': 10, 'sensor_height': 160,
                                 'sensor_width': 120, 'terminate_condition': 'discrete_stop',
                                 'nav_goal_seg_channels': 1}, 'PLACE': {'type': 'heuristic',
                                                                        'checkpoint_path': 'data/checkpoints/ovmm_baseline_home_robot_challenge_2023/place.pth',
                                                                        'rl_config': 'projects/habitat_ovmm/configs/agent/skills/place_rl.yaml',
                                                                        'gym_obs_keys': ['robot_head_depth',
                                                                                         'goal_receptacle', 'joint',
                                                                                         'goal_recep_segmentation',
                                                                                         'is_holding',
                                                                                         'object_embedding'],
                                                                        'allowed_actions': ['arm_action',
                                                                                            'base_velocity',
                                                                                            'manipulation_mode'],
                                                                        'arm_joint_mask': [1, 1, 1, 1, 1, 0, 0],
                                                                        'max_displacement': 0.25,
                                                                        'max_turn_degrees': 30.0,
                                                                        'min_turn_degrees': 5.0,
                                                                        'min_displacement': 0.1, 'sensor_height': 160,
                                                                        'sensor_width': 120, 'nav_goal_seg_channels': 1,
                                                                        'terminate_condition': 'ungrip',
                                                                        'grip_threshold': -0.8,
                                                                        'manip_mode_threshold': 0.8,
                                                                        'constraint_base_in_manip_mode': True,
                                                                        'max_joint_delta': 0.1,
                                                                        'min_joint_delta': 0.02}},
                    'skip_skills': {'nav_to_obj': False, 'nav_to_rec': True, 'gaze_at_obj': True, 'gaze_at_rec': True,
                                    'pick': True, 'place': True},
                    'PLANNER': {'collision_threshold': 0.2, 'obs_dilation_selem_radius': 3,
                                'goal_dilation_selem_radius': 10, 'step_size': 5, 'use_dilation_for_stg': False,
                                'min_obs_dilation_selem_radius': 1, 'map_downsample_factor': 1,
                                'map_update_frequency': 1, 'discrete_actions': True, 'verbose': False}}}

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
    

class HabitatOpenVocabManipEnv():
    def __init__(self, ):
        self._obj_name_to_id_mapping = {'action_figure': 0, 'android_figure': 1, 'apple': 2, 'backpack': 3,
                                        'baseballbat': 4, 'basket': 5, 'basketball': 6, 'bath_towel': 7,
                                        'battery_charger': 8, 'board_game': 9, 'book': 10, 'bottle': 11, 'bowl': 12,
                                        'box': 13, 'bread': 14, 'bundt_pan': 15, 'butter_dish': 16, 'c-clamp': 17,
                                        'cake_pan': 18, 'can': 19, 'can_opener': 20, 'candle': 21, 'candle_holder': 22,
                                        'candy_bar': 23, 'canister': 24, 'carrying_case': 25, 'casserole': 26,
                                        'cellphone': 27, 'clock': 28, 'cloth': 29, 'credit_card': 30, 'cup': 31,
                                        'cushion': 32, 'dish': 33, 'doll': 34, 'dumbbell': 35, 'egg': 36,
                                        'electric_kettle': 37, 'electronic_cable': 38, 'file_sorter': 39, 'folder': 40,
                                        'fork': 41, 'gaming_console': 42, 'glass': 43, 'hammer': 44, 'hand_towel': 45,
                                        'handbag': 46, 'hard_drive': 47, 'hat': 48, 'helmet': 49, 'jar': 50, 'jug': 51,
                                        'kettle': 52, 'keychain': 53, 'knife': 54, 'ladle': 55, 'lamp': 56,
                                        'laptop': 57, 'laptop_cover': 58, 'laptop_stand': 59, 'lettuce': 60,
                                        'lunch_box': 61, 'milk_frother_cup': 62, 'monitor_stand': 63, 'mouse_pad': 64,
                                        'multiport_hub': 65, 'newspaper': 66, 'pan': 67, 'pen': 68, 'pencil_case': 69,
                                        'phone_stand': 70, 'picture_frame': 71, 'pitcher': 72, 'plant_container': 73,
                                        'plant_saucer': 74, 'plate': 75, 'plunger': 76, 'pot': 77, 'potato': 78,
                                        'ramekin': 79, 'remote': 80, 'salt_and_pepper_shaker': 81, 'scissors': 82,
                                        'screwdriver': 83, 'shoe': 84, 'soap': 85, 'soap_dish': 86,
                                        'soap_dispenser': 87, 'spatula': 88, 'spectacles': 89, 'spicemill': 90,
                                        'sponge': 91, 'spoon': 92, 'spray_bottle': 93, 'squeezer': 94, 'statue': 95,
                                        'stuffed_toy': 96, 'sushi_mat': 97, 'tape': 98, 'teapot': 99,
                                        'tennis_racquet': 100, 'tissue_box': 101, 'toiletry': 102, 'tomato': 103,
                                        'toy_airplane': 104, 'toy_animal': 105, 'toy_bee': 106, 'toy_cactus': 107,
                                        'toy_construction_set': 108, 'toy_fire_truck': 109, 'toy_food': 110,
                                        'toy_fruits': 111, 'toy_lamp': 112, 'toy_pineapple': 113, 'toy_rattle': 114,
                                        'toy_refrigerator': 115, 'toy_sink': 116, 'toy_sofa': 117, 'toy_swing': 118,
                                        'toy_table': 119, 'toy_vehicle': 120, 'tray': 121, 'utensil_holder_cup': 122,
                                        'vase': 123, 'video_game_cartridge': 124, 'watch': 125, 'watering_can': 126,
                                        'wine_bottle': 127,"television":128,"bonsai":129,"oven":130,"laptop":131,"fire":132}

        self._rec_name_to_id_mapping = {'bathtub': 0, 'bed': 1, 'bench': 2, 'cabinet': 3, 'chair': 4,
                                        'chest_of_drawers': 5, 'couch': 6, 'counter': 7, 'filing_cabinet': 8,
                                        'hamper': 9, 'serving_cart': 10, 'shelves': 11, 'shoe_rack': 12, 'sink': 13,
                                        'stand': 14, 'stool': 15, 'table': 16, 'toilet': 17, 'trunk': 18,
                                        'wardrobe': 19, 'washer_dryer': 20}
        self._obj_id_to_name_mapping = {k: v for v, k in self._obj_name_to_id_mapping.items()}
        self._rec_id_to_name_mapping = {k: v for v, k in self._rec_name_to_id_mapping.items()}

    def preprocess_obs(self, state,  cam_pose,c_abs_pose,c_abs_rpy,third_party_perspective) -> home_robot.core.interfaces.Observations:
        
        depth = state["robot0::robot0:eyes_Camera_sensor_depth_linear"]
        rgb = state["robot0::robot0:eyes_Camera_sensor_rgb"][:, :, :3]
        lidar=state['robot0::robot0:scan_link_Lidar_sensor_scan']
        global_map_lidar=state["global_map_lidar"]
        local_map_lidar=state["local_map_lidar"] 
        gps = cam_pose[:2]
        compass=np.expand_dims(cam_pose[2],axis=0)   
        camera_pose=np.stack((c_abs_pose, c_abs_rpy))     
        obs = home_robot.core.interfaces.Observations(
            rgb=rgb,
            lidar=lidar,#360*1
            depth=depth,
            global_map_lidar=global_map_lidar,
            local_map_lidar=local_map_lidar,
            compass=compass,#罗盘 1方位角
            gps=gps,#2 xy
            task_observations={
                "object_embedding": None,          
                "start_receptacle": np.array([20]),
                "goal_receptacle": np.array([15]),
                "prev_grasp_success": np.array([0.]),
            },
            joint=None,
            relative_resting_position=None,
            third_person_image=third_party_perspective,#第三方图像
            camera_pose=camera_pose,
            
        )

        obj_name = self._obj_id_to_name_mapping[131]      
        start_receptacle = self._rec_id_to_name_mapping[20]
        goal_receptacle = self._rec_id_to_name_mapping[15]
        goal_name = ("Find the " + obj_name )
        obj_goal_id = 1  # semantic sensor returns binary mask for goal object  图像中目标为1，其他为0
        start_rec_goal_id = -1
        end_rec_goal_id = -1
        obs.task_observations["goal_name"] = goal_name
        obs.task_observations["object_name"] = obj_name
        obs.task_observations["start_recep_name"] = start_receptacle
        obs.task_observations["place_recep_name"] = goal_receptacle
        obs.task_observations["object_goal"] = obj_goal_id
        obs.task_observations["start_recep_goal"] = start_rec_goal_id
        obs.task_observations["end_recep_goal"] = end_rec_goal_id
        return obs




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
        self.ovmm=HabitatOpenVocabManipEnv()
        self.agent = OpenVocabManipAgent(config)
        self.agent.reset() 
        # rospy.init_node('costmap_query')
        # self.tf_listener = tf.TransformListener()
        # self.global_costmap_ros = Costmap2DROS('global_costmap', self.tf_listener)  

        # # 订阅原始纯边界地图
        # rospy.Subscriber("/raw_map", OccupancyGrid, self.raw_map_callback)

        # 订阅全局代价地图
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.global_map_callback)

        # 订阅局部代价地图
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.local_map_callback)

        # 位姿计算及状态更新器
        self.current_pose = rospy.Subscriber('/posemsg', Float32MultiArray, self.odometry_callback)

        # 火源位置计算服务终端
        self.Boxtf_srv = rospy.Service('/box_place', Boxinfo, self.callback_box)

        # 激光雷达数据监听器
        self.Laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan, queue_size=2)
        
        
    
    

    def global_map_callback(self, msg):
        self.global_map = msg


    def local_map_callback(self, msg):
        self.local_map = msg


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

    # def world_to_map(self, wx, wy):
    #     # Convert world coordinates to map indices
    #     costmap = self.global_costmap_ros.getCostmap()
    #     import pdb;pdb.set_trace(0)
    #     return costmap.world_to_map(wx, wy)
    
    def odometry_callback(self, msg):
        print("当前全局位姿:",msg.data)
        x_map = int((msg.data[0] / 0.05) + self.global_map.info.width//2)
        y_map = int((msg.data[1] / 0.05) + self.global_map.info.height//2)
        pose=(x_map,y_map,msg.data[2])
       # import pdb;pdb.set_trace()
        global_pose = self.lidar_nav(self.global_map.data,self.global_map.info.height,self.global_map.info.width, self.local_map.data, pose) #960*960全局地图，480*480以机器人为中心的局部地图，机器人的全局位姿
        
        global_pose[0]=(global_pose[0]-self.global_map.info.width//2)*0.05
        global_pose[1]=(global_pose[1]-self.global_map.info.height//2)*0.05
        print("返回的全局位姿：",global_pose)
        self.send_goals([global_pose[0], global_pose[1], 0])
        # 在这里添加自己的程序，发布每次的目标点，
       


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
            rate. sleep()
            
            return BoxinfoResponse(best_est[0], best_est[1], self.road_len)
        else:
            return BoxinfoResponse(0, 0, self.road_len)
        
        
    def lidar_nav(self,global_map_lidar,global_map_lidar_height,global_map_lidar_width,local_map_lidar,cam_pose):
        cam_pose=np.array(cam_pose)
        global_map_lidar=np.array(global_map_lidar)   #把全局地图的障碍物设置为255，其他为0
        global_map_lidar=global_map_lidar.reshape(global_map_lidar_height,global_map_lidar_width)
        global_map_lidar=np.where(global_map_lidar==99,255,global_map_lidar)
        global_map_lidar=np.where(global_map_lidar==100,255,global_map_lidar)
        global_map_lidar=np.where(global_map_lidar!=255,0,global_map_lidar)
        
        global_map_lidar_960=np.zeros((960,960))#把不规则的全局地图放到960*960地图中心
        origins_height_global=int(960/2-global_map_lidar_height/2)
        origins_width_global=int(960/2-global_map_lidar_width/2   )     
        for i in range (global_map_lidar_height):
            for j in range (global_map_lidar_width):
                 global_map_lidar_960[origins_height_global+i,origins_width_global+j]=global_map_lidar[i,j]
                 
        #import pdb;pdb.set_trace()         
        cam_pose[0]=int(960/2-cam_pose[0]/2)+cam_pose[0]  #不规则的点转换到960*960 
        cam_pose[1]=int(960/2-cam_pose[1]/2)+cam_pose[1]  #不规则的点转换到960*960  
        
        
        local_map_lidar=np.array(local_map_lidar)#局部地图没用到所以先不管
        local_map_lidar=local_map_lidar.reshape(100,100)
        
        
        local_map_lidar_480=np.zeros((480,480))#把不规则的全局地图放到960*960地图中心
        origins_height_local=int(480/2-100/2)
        origins_width_local=int(480/2-100/2   )     
        for i in range (100):
            for j in range (100):
                 local_map_lidar_480[origins_height_local+i,origins_width_local+j]=local_map_lidar[i,j]
                
        state= {'robot0::robot0:eyes_Camera_sensor_seg_semantic': np.random.random((640, 480)), 'robot0::robot0:eyes_Camera_sensor_rgb':  np.random.randint(0, 256, size=(640, 480,4),dtype=np.uint8), 
        'robot0::robot0:eyes_Camera_sensor_depth_linear': np.random.random((640, 480)), 'robot0::robot0:eyes_Camera_sensor_camera::pose':np.random.random((4, 4)) , 
        'robot0::robot0:eyes_Camera_sensor_camera::fov': 0.8232465683816635, 'robot0::robot0:eyes_Camera_sensor_camera::focal_length': 24.0, 
        'robot0::robot0:eyes_Camera_sensor_camera::horizontal_aperture': 20.954999923706055, 
        'robot0::robot0:eyes_Camera_sensor_camera::view_projection_matrix': np.random.random((4, 4)), 
        'robot0::robot0:eyes_Camera_sensor_camera::resolution::width': 480, 'robot0::robot0:eyes_Camera_sensor_camera::resolution::height': 640, 
        'robot0::robot0:scan_link_Lidar_sensor_scan': np.random.random((360, 1))  , 
        'task::low_dim': np.array([ 3.76993206e-01, -3.87271301e+00,  2.14445317e-03, -1.19317185e-04,   -2.19012777e-04,  3.56289366e-04,  4.24911868e-02, -1.27411307e-03]),
        'global_map_lidar': global_map_lidar_960,'local_map_lidar': local_map_lidar_480}
        
        #cam_pose=[1.1000422239303589, 2.7999167442321777, -1.2157252571918085]
        c_pos=[1.1000422239303589, 2.7999167442321777, -1.2157252571918085] #没用到，随意给
        c_rpy=[1.1000422239303589, 2.7999167442321777, -1.2157252571918085]
    
        third_party_perspective=state['robot0::robot0:eyes_Camera_sensor_rgb'][:,:,0:3]
        obs = self.ovmm.preprocess_obs(state, cam_pose,c_pos,c_rpy,third_party_perspective)         
        cur_angle = obs.compass

        action1, info, _ ,found_fire,global_pose_final,local_map_lidar_fire=self.agent.act(obs)#####global_pose_final为要导航到的点       
                
        global_pose_final[0]=global_pose_final[0]-int(960/2-cam_pose[0]/2)       
        global_pose_final[1]=global_pose_final[1]-int(960/2-cam_pose[1]/2)#960*960的点反变换回到原全局地图大小的位置         
        return  global_pose_final

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
    buffer = MapAndScanBuffer()
    rospy.spin()

if __name__ == '__main__':
    main()
