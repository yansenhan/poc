import rospy
from ucar_nav.srv import Boxinfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import cv2
from copy import deepcopy
import torch
import PIL

class Yolo_det:
  
    def __init__(self) -> None:
        modelpath = '/home/lijixiang/Thu_unitree/fire-detection-from-images/demo/self_best.pt'
        self.model = torch.hub.load('ultralytics/yolov5:master', 'custom', modelpath)  # force_reload=True to update

    def yolo(self, im):
        results = self.model(im)  # inference
        xywh_torch_list = results.xywh[0]
        if len(xywh_torch_list):
            # print(xywh_torch_list[:, 4])
            max_index = torch.argmax(xywh_torch_list[:, 4])
            xywh = xywh_torch_list[max_index, :4]
            conf = xywh_torch_list[max_index, 4]
            if conf < 0.6:
                return None, None
        else:
            xywh, conf = None, None
        return xywh, conf


class Yolo_client:
    def __init__(self):
        self.img = None
        self.yolo_det = Yolo_det()
        self.cv_bridge = CvBridge()
        self.cam_sub = rospy.Subscriber('/cam', Image, self.cam_callback, queue_size=2)
        
        
    def cam_callback(self, img):
        cv_img = self.cv_bridge.imgmsg_to_cv2(img)
        im = deepcopy(cv_img)
        pil_img = PIL.Image.fromarray(cv_img)
        xywh, conf = self.yolo_det.yolo(pil_img)
        print(xywh)
        print("conf: ", conf)
        self.cv_show_box(im, xywh)
            
        
    def cv_show_box(self, cv_img, xywh):
        print(type(cv_img), cv_img.shape)
        if xywh is not None:
            
            x, y, w, h = xywh[0], xywh[1], xywh[2], xywh[3]
            cv2.rectangle(cv_img, (int(x-w//2), int(y-h//2)), (int(x+w//2), int(y+h//2)), (0,0,255), 2)
            
        cv2.imshow('Image', cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        cv2.waitKey(2)
        if xywh is not None:
            box_client = rospy.ServiceProxy('/box_place', Boxinfo)
            boxinfo = Boxinfo()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            boxinfo.header = header
            boxinfo.box_x = xywh[0]
            boxinfo.box_y = xywh[1]
            boxinfo.box_w = xywh[2]
            boxinfo.box_h = xywh[3]
        
            response = box_client(header, int(xywh[0]), int(xywh[1]), int(xywh[2]), int(xywh[3]))
            print(f'Response: {response}')
            # response = box_client(boxinfo)
            # return xywh


if __name__ == '__main__':
    print('ok')
    # import pdb; pdb.set_trace()
    rospy.init_node('yolo_client')
    rospy.wait_for_service('/box_place')
    yolo_client = Yolo_client()
    rospy.spin()
    
