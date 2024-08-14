import rospy
from ucar_nav.msg import Boxinfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import cv2
from copy import deepcopy
import torch
import PIL
import numpy as np

class Yolo_det:
  
    def __init__(self) -> None:
        # modelpath = '/home/lijixiang/Thu_unitree/fire-detection-from-images/demo/self_best.pt'
        # modelpath = '/home/lijixiang/Thu_unitree/fire-detection-from-images/demo/self_best.pt'
        modelpath = '/home/lijixiang/Thu_unitree/fire-detection-from-images/demo/self_mix2.pt'
        self.model = torch.hub.load('ultralytics/yolov5:master', 'custom', modelpath)  # force_reload=True to update

    def yolo(self, im):
        results = self.model(im)  # inference
        xywh_torch_list = results.xywh[0]
        if len(xywh_torch_list):
            # print(xywh_torch_list[:, 4])
            max_index = torch.argmax(xywh_torch_list[:, 4])
            xywh = xywh_torch_list[max_index, :4]
            conf = xywh_torch_list[max_index, 4]
            if conf < 0.75:
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
        self.yolo_pub = rospy.Publisher('/box_place', Boxinfo, queue_size=2)
        
        
    def cam_callback(self, img):
        
        cv_bridge = CvBridge()
        cv_image = cv_bridge.imgmsg_to_cv2(img, "mono8") # cv2.imdecode
        cv_img = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)[..., ::-1]
        # cv_img = self.cv_bridge.imgmsg_to_cv2(img)
        im = deepcopy(cv_img)
        # pil_img = PIL.Image.fromarray(cv_img)
        # xywh, conf = self.yolo_det.yolo(pil_img)
        # print("type of im", type(im), im.shape)
        try:
            result_img = self.darken_overexposed(cv_img)
            # print("type of result_img", type(result_img), result_img.shape)
            # pil_img = PIL.Image.fromarray(cv_img)
            pil_img = PIL.Image.fromarray(result_img)
            xywh, conf = self.yolo_det.yolo(pil_img)

            if xywh is not None and self.accum_mask_pt(result_img, xywh) > 0.7:
                xywh = None

            # pixel_val = pil_img.getpixel((xywh[0], xywh[1]))
            # pixel_val = self.get_local_pixel_rgb_mean(pil_img, xywh)
            # print("pixel_val", pixel_val)
            # if pixel_val[0] > 65 and pixel_val[0] < 90 \
            #     and pixel_val[1] > 35 and pixel_val[1] < 55 \
            #     and pixel_val[2] > 8 and pixel_val[2] < 13:
            #     pass
            # else:
            #     print("####### pixel strategy ########")
            #     xywh = None
        except CvBridgeError as e:
            print(e)

        print(xywh)
        print("conf: ", conf)
        self.cv_show_box(deepcopy(result_img), xywh)
    
    def get_local_pixel_rgb_mean(self, img, xywh):
        if xywh is None: return
        
        sum_pixel = np.array([0, 0, 0])
        count = 0
        x, y, w, h = xywh
        for i in range(int(w.item() // 4)):
            for j in range(int(h.item() // 4)):
                self.add_tuple(sum_pixel, img.getpixel((x - i, y - i)))
                self.add_tuple(sum_pixel, img.getpixel((x + i, y - i)))
                self.add_tuple(sum_pixel, img.getpixel((x - i, y + i)))
                self.add_tuple(sum_pixel, img.getpixel((x + i, y + i)))
                count += 4
        mean_pixel = sum_pixel * 1.0 / count

        return mean_pixel

    def add_tuple(self, arr, tp):
        for i in range(len(tp)):
            arr[i] += tp[i]
        return arr

    def accum_mask_pt(self, img, xywh):
        x, y, w, h = xywh
        x_left  = int(x - w // 2)
        x_right = int(x + w // 2)
        y_down  = int(y - h // 2)
        y_up    = int(y + h // 2)

        accum_mask_pt_num = 0
        total_num = 0
        for i in range(x_left, x_right):
            for j in range(y_down, y_up):
                total_num += 1
                pt = img[i, j]
                if (pt == [0, 0, 255]).all():
                    accum_mask_pt_num += 1
        
        return accum_mask_pt_num * 1.0 / total_num


    def darken_overexposed(self, image_array):
        gray_img = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
        threshold = 220

        _, mask = cv2.threshold(gray_img, threshold, 255, cv2.THRESH_BINARY)

        image_array[mask == 255] = [0, 0, 255]

        return image_array
        
    def cv_show_box(self, img, xywh):
        if xywh is not None:
            
            x, y, w, h = xywh[0], xywh[1], xywh[2], xywh[3]
            cv2.rectangle(img, (int(x-w//2), int(y-h//2)), (int(x+w//2), int(y+h//2)), (0,255,0), 2)
            
        cv2.imshow('Image', cv2.cvtColor(cv2.flip(img, 1), cv2.COLOR_BGR2RGB))
        cv2.waitKey(10)
        if xywh is not None:
            # box_client = rospy.ServiceProxy('/box_place', Boxinfo)
            boxinfo = Boxinfo()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            boxinfo.header = header
            boxinfo.box_x = int(xywh[0])
            boxinfo.box_y = int(xywh[1])
            boxinfo.box_w = int(xywh[2])
            boxinfo.box_h = int(xywh[3])
        
            # response = box_client(header, int(xywh[0]), int(xywh[1]), int(xywh[2]), int(xywh[3]))
            # print(f'Response: {response}')
            # response = box_client(boxinfo)
            self.yolo_pub.publish(boxinfo)
            return xywh


if __name__ == '__main__':
    print('ok')
    # import pdb; pdb.set_trace()
    rospy.init_node('yolo_client')
    # rospy.wait_for_service('/box_place')
    yolo_client = Yolo_client()
    rospy.spin()
    
