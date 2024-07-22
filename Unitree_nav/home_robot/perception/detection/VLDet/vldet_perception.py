# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse
import pathlib
import sys
from pathlib import Path
from typing import List, Optional, Tuple
import multiprocessing as mp
import cv2
import numpy as np
import torch
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.engine.defaults import DefaultPredictor
from detectron2.utils.visualizer import ColorMode, Visualizer
from detectron2.utils.logger import setup_logger
from home_robot.core.abstract_perception import PerceptionModule
from home_robot.core.interfaces import Observations
from home_robot.perception.detection.utils import filter_depth, overlay_masks
sys.path.append('/home/ubuntu/zyp/home-robot/src/home_robot/home_robot/perception/detection/VLDet/CenterNet2/projects/CenterNet2')
# sys.path.insert(0, 'CenterNet2/projects/CenterNet2/')
from centernet.config import add_centernet_config  # noqa: E402
from home_robot.perception.detection.VLDet.vldet.config import add_vldet_config
from home_robot.perception.detection.VLDet.vldet.predictor import VisualizationDemo, get_clip_embeddings
from home_robot.perception.detection.VLDet.vldet.modeling.utils import (  # noqa: E402
    reset_cls_test,
)

BUILDIN_CLASSIFIER = {
    'lvis': 'datasets/metadata/lvis_v1_clip_a+cname.npy',
    'objects365': 'datasets/metadata/o365_clip_a+cnamefix.npy',
    'openimages': 'datasets/metadata/oid_clip_a+cname.npy',
    'coco': 'datasets/metadata/coco_clip_a+cname.npy',
}

BUILDIN_METADATA_PATH = {
    'lvis': 'lvis_v1_val',
    'objects365': 'objects365_v2_val',
    'openimages': 'oid_val_expanded',
    'coco': 'coco_2017_val',
}

class VLDetPerception(PerceptionModule):
    def __init__(
        self,
        config_file=None,
        vocabulary="coco",
        custom_vocabulary="",
        checkpoint_file=None,
        sem_gpu_id=0,
        instance_mode=ColorMode.IMAGE,
        parallel=False,
        verbose: bool = False,
    ):
        """Load trained VLDet model for inference.

        Arguments:
            config_file: path to model config
            vocabulary: currently one of "coco" for indoor coco categories or "custom"
             for a custom set of categories
            custom_vocabulary: if vocabulary="custom", this should be a comma-separated
             list of classes (as a single string)
            checkpoint_file: path to model checkpoint
            sem_gpu_id: GPU ID to load the model on, -1 for CPU
            verbose: whether to print out debug information
        """
        self.verbose = verbose
        mp.set_start_method("spawn", force=True)

        # grounding_dino_model = Model(
        #     model_config_path=GROUNDING_DINO_CONFIG_PATH,
        #     model_checkpoint_path=GROUNDING_DINO_CHECKPOINT_PATH,
        # )
        # 引入sam作为分割
        # sam = build_sam_vit_h(SAM_CHECKPOINT_PATH)
        # sam.to(device=DEVICE)
        # sam_predictor = SamPredictor(sam)

        if self.verbose:
            print(
                f"Loading VLDet with config={config_file} and checkpoint={checkpoint_file}"
            )
        # args = get_parser().parse_args()
        args = get_parser().parse_known_args()[0]
        # print(args.config_file)
        # import pdb; pdb.set_trace()
        cfg = setup_cfg(args)
        setup_logger(name="fvcore")
        logger = setup_logger()
        logger.info("Arguments: " + str(args))

        if args.vocabulary == 'custom':
            self.metadata = MetadataCatalog.get("__unused")
            self.metadata.thing_classes = args.custom_vocabulary.split(',')
            classifier = get_clip_embeddings(self.metadata.thing_classes)
        else:
            self.metadata = MetadataCatalog.get(
                BUILDIN_METADATA_PATH[args.vocabulary])
            classifier = BUILDIN_CLASSIFIER[args.vocabulary]

        num_classes = len(self.metadata.thing_classes)
        self.cpu_device = torch.device("cpu")
        self.instance_mode = instance_mode

        self.parallel = parallel
        if parallel:
            num_gpu = torch.cuda.device_count()
            # self.predictor = AsyncPredictor(cfg, num_gpus=num_gpu)
        else:
            self.predictor = DefaultPredictor(cfg)
        self.predictor = DefaultPredictor(cfg)
        reset_cls_test(self.predictor.model, classifier, num_classes)


    def reset_vocab(self, new_vocab: List[str], vocab_type="custom"):
        """Resets the vocabulary of VLDet model allowing you to change detection on
        the fly. Note that previous vocabulary is not preserved.
        Args:
            new_vocab: list of strings representing the new vocabulary
            vocab_type: one of "custom" or "coco"; only "custom" supported right now
        """
        if self.verbose:
            print(f"Resetting vocabulary to {new_vocab}")
        MetadataCatalog.remove("__unused")
        if vocab_type == "custom":
            self.metadata = MetadataCatalog.get("__unused")
            self.metadata.thing_classes = new_vocab
            classifier = get_clip_embeddings(self.metadata.thing_classes)
            self.categories_mapping = {
                i: i for i in range(len(self.metadata.thing_classes))
            }
        else:
            raise NotImplementedError(
                "VLDet does not have support for resetting from custom to coco vocab"
            )
        self.num_sem_categories = len(self.categories_mapping)

        num_classes = len(self.metadata.thing_classes)
        reset_cls_test(self.predictor.model, classifier, num_classes)

    def predict(
        self,
        obs: Observations,
        depth_threshold: Optional[float] = None,
        draw_instance_predictions: bool = True,
    ) -> Observations:
        """
        Arguments:
            obs.rgb: image of shape (H, W, 3) (in RGB order - VLDet expects BGR)
            obs.depth: depth frame of shape (H, W), used for depth filtering
            depth_threshold: if specified, the depth threshold per instance

        Returns:
            obs.semantic: segmentation predictions of shape (H, W) with
             indices in [0, num_sem_categories - 1]
            obs.task_observations["semantic_frame"]: segmentation visualization
             image of shape (H, W, 3)
        """
        image = cv2.cvtColor(obs.rgb, cv2.COLOR_RGB2BGR)
        depth = obs.depth
        height, width, _ = image.shape

        # import netron
        # import pdb; pdb.set_trace()
        pred = self.predictor(image)
        
        # onnx_path = "onnx_model_vldet.onnx"
        # torch.onnx.export(self.predictor, image, onnx_path)
        # netron.start(onnx_path)

        if obs.task_observations is None:
            obs.task_observations = {}

        if draw_instance_predictions:
            visualizer = Visualizer(
                image[:, :, ::-1], self.metadata, instance_mode=self.instance_mode
            )
            visualization = visualizer.draw_instance_predictions(
                predictions=pred["instances"].to(self.cpu_device)
            ).get_image()
            obs.task_observations["semantic_frame"] = visualization
        else:
            obs.task_observations["semantic_frame"] = None

        # Sort instances by mask size
        masks = pred["instances"].pred_masks.cpu().numpy()
        class_idcs = pred["instances"].pred_classes.cpu().numpy()
        scores = pred["instances"].scores.cpu().numpy()

        if depth_threshold is not None and depth is not None:
            masks = np.array(
                [filter_depth(mask, depth, depth_threshold) for mask in masks]
            )

        semantic_map, instance_map = overlay_masks(masks, class_idcs, (height, width))

        obs.semantic = semantic_map.astype(int)
        obs.task_observations["instance_map"] = instance_map
        obs.task_observations["instance_classes"] = class_idcs
        obs.task_observations["instance_scores"] = scores

        return obs


def setup_cfg(args):
    cfg = get_cfg()
    if args.cpu:
        cfg.MODEL.DEVICE="cpu"
    add_centernet_config(cfg)
    add_vldet_config(cfg)
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    # Set score_threshold for builtin models
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = args.confidence_threshold
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence_threshold
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = args.confidence_threshold
    cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand' # load later
    if not args.pred_all_class:
        cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True
    cfg.freeze()
    return cfg


def get_parser():
    parser = argparse.ArgumentParser(description="Detectron2 demo for builtin configs")
    parser.add_argument(
        "--config-file",
        default="/home/ubuntu/zyp/home-robot/src/home_robot/home_robot/perception/detection/VLDet/configs/VLDet_LbaseI_CLIP_SwinB_896b32_2x_ft4x_caption.yaml",  # 换成自己的路径
        metavar="FILE",
        help="path to config file",
    )
    parser.add_argument("--webcam", help="Take inputs from webcam.")
    parser.add_argument("--cpu", action='store_true', help="Use CPU only.")
    parser.add_argument("--video-input", help="Path to video file.")
    parser.add_argument(
        "--input",
        default=["./snapshot_015.png"],
        nargs="+",
        help="A list of space separated input images; "
        "or a single glob pattern such as 'directory/*.jpg'",
    )
    parser.add_argument(
        "--output",
        default="./results",
        help="A file or directory to save output visualizations. "
        "If not given, will show output in an OpenCV window.",
    )
    parser.add_argument(
        "--vocabulary",
        default="custom",
        choices=['lvis', 'openimages', 'objects365', 'coco', 'custom'],
        help="",
    )
    parser.add_argument(
        "--custom_vocabulary",
        default="action_figure,android_figure,apple,backpack,baseballbat,basket, \
                 basketball,bath_towel,battery_charger,board_game,book,bottle,bowl, \
                 box,bread,bundt_pan,butter_dish,c-clamp,cake_pan,can,can_opener, \
                 candle,candle_holder,candy_bar,canister,carrying_case,casserole,cellphone,clock, \
                 cloth,credit_card,cup,cushion,dish,doll,dumbbell,egg,electric_kettle,electronic_cable, \
                 file_sorter,folder,fork,gaming_console,glass,hammer,hand_towel,handbag,hard_drive,hat,helmet, \
                 jar,jug,kettle,keychain,knife,ladle,lamp,laptop,laptop_cover,laptop_stand,lettuce,lunch_box, \
                 milk_frother_cup,monitor_stand,mouse_pad,multiport_hub,newspaper,pan,pen,pencil_case,phone_stand, \
                 picture_frame,pitcher,plant_container,plant_saucer,plate,plunger,pot,potato,ramekin,remote, \
                 salt_and_pepper_shaker,scissors,screwdriver,shoe,soap,soap_dish,soap_dispenser,spatula,spectacles, \
                 spicemill,sponge,spoon,spray_bottle,squeezer,statue,stuffed_toy,sushi_mat,tape,teapot,tennis_racquet, \
                 tissue_box,toiletry,tomato,toy_airplane,toy_animal,toy_bee,toy_cactus,toy_construction_set, \
                 toy_fire_truck,toy_food,toy_fruits,toy_lamp,toy_pineapple,toy_rattle,toy_refrigerator, \
                 toy_sink,toy_sofa,toy_swing,toy_table,toy_vehicle,tray,utensil_holder_cup,vase,video_game_cartridge, \
                 watch,watering_can,wine_bottle,bathtub,bed,bench,cabinet,chair,chest_of_drawers,couch,counter,filing_cabinet, \
                 hamper,serving_cart,shelves,shoe_rack,sink,stand,stool,table,toilet,trunk,wardrobe,washer_dryer",
        help="",
    )
    parser.add_argument("--pred_all_class", action='store_true')
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.5,
        help="Minimum score for instance predictions to be shown",
    )
    parser.add_argument(
        "--opts",
        help="Modify config options using the command-line 'KEY VALUE' pairs",
        default=[],
        nargs=argparse.REMAINDER,
    )
    return parser
