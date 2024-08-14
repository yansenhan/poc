import numpy as np
from matplotlib import image as mpimg
import cv2
lidar_map=np.load("/home/lijixiang/Thu_unitree/Unitree_nav/lidar_map.npy")


kernel=np.ones((3,3))

kernel=kernel.astype('uint8')
lidar_map=lidar_map.astype('uint8')
image=cv2.erode(lidar_map,kernel)
mpimg.imsave("bbb.jpg",image)

mpimg.imsave("ccc.jpg",lidar_map)
image=cv2.GaussianBlur(lidar_map,(5,5),0)


mpimg.imsave("ddd.jpg",image)

image=cv2.dilate(lidar_map,kernel)
print(np.unique(image))
mpimg.imsave("aaa.jpg",image)
