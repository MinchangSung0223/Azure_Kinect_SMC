import sys
import os
import argparse
from PIL import Image
from primesense import openni2  # , nite2
from primesense import _openni2 as c_api
from matplotlib import pyplot as plt
import time
import numpy as np
import cv2
import threading
from scipy.interpolate import splprep, splev
from ctypes import cdll
import ctypes
from numpy.ctypeslib import ndpointer
import shutil

lib = cdll.LoadLibrary('./viewer_opengl.so')
st = lib.Foo_start
t0 = threading.Thread(target=st)
t0.start()
end = lib.Foo_end
dataread =lib.Foo_dataread
dataread_color =lib.Foo_dataread_color
dataread_depth =lib.Foo_dataread_depth
dataread_color_to_depth =lib.Foo_dataread_color_to_depth
dataread.restype = ndpointer(dtype=ctypes.c_uint8, shape=(720,1280,2))
dataread_color.restype = ndpointer(dtype=ctypes.c_uint8, shape=(720,1280,4))
dataread_depth.restype = ndpointer(dtype=ctypes.c_uint16, shape=(512,512))#ctypes.POINTE
dataread_color_to_depth.restype = ndpointer(dtype=ctypes.c_uint8, shape=(512,512,4))
from indydcp_client import IndyDCPClient
bind_ip = "192.168.0.6"   
server_ip = "192.168.0.7"
robot_name = "NRMK-Indy7"
indy = IndyDCPClient(bind_ip, server_ip, robot_name) 
indy.connect()
print("INDY CONNECTION : "+str(indy.is_connected()))
classname = "test"
classname1 = classname
smooth_rate = 200
classnumber = 4
scale_factor = 1.1
home_path=os.getcwd() 
img_list = os.listdir("/media/sung/613eb739-ef83-417c-a6da-b04fe4d9640e/home/sung/Downloads/data/coco/train2017")
img_list_user = os.listdir("./Camera")

classname = classname+"_"
indy.task_move_to([0.4,0,0.5,-180,0,180])
rgb_segmentation =1
darker = 0;
sensitivity = 245;
x= 0
y = 0
w = 0
h = 0
def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)


def detect_img():
    x = 566
    y = 569
    width  = 728-566
    height = 609-550
    for i in range(0,10000):
      color_data = np.array(dataread_color(),dtype=np.uint8)
      color_img = color_data[:,:,0:3]
      depth_to_color_data = np.array(dataread(),dtype=np.uint8)
      depth_to_color_img = depth_to_color_data[:,:,0]
    
    roi_rgb = color_img[y:y+height,x:x+width]
    roi_depth = depth_to_color_img[y:y+height,x:x+width]
    hsv_roi_rgb = cv2.cvtColor(roi_rgb,cv2.COLOR_BGR2HSV)
    roi_rgb_hist = cv2.calcHist([hsv_roi_rgb],[0],None,[180],[0,180])
    roi_rgb_hist = cv2.normalize(roi_rgb_hist,roi_rgb_hist,0,255,cv2.NORM_MINMAX)
    #print(roi_rgb_hist)
    
    
    cv2.namedWindow("rgb", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("rgb", 1280,720)
    cv2.namedWindow("depth", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("depth", 1280,720)
    prev_rgb = np.zeros(color_img.shape,dtype=np.uint8)
    prev_depth = np.zeros(depth_to_color_img.shape,dtype=np.uint8)

    term_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10,1)
    while 1:
      color_data = np.array(dataread_color(),dtype=np.uint8)
      color_img = color_data[:,:,0:3]
      depth_to_color_data = np.array(dataread(),dtype=np.uint8)
      depth_to_color_img = depth_to_color_data[:,:,0]
      depth_img = depth_to_color_img.copy()

      hsv_color = cv2.cvtColor(color_img,cv2.COLOR_BGR2HSV)
      mask = cv2.calcBackProject([hsv_color],[0],roi_rgb_hist,[0,180],1)
       
      _,track_window_rgb = cv2.meanShift(mask,(x,y,width,height),term_criteria)
      x,y,w,h = track_window_rgb
      box_image = cv2.rectangle(color_img,(x,y),(x+w,y+h),(0,255,0),1)
      

      diff_rgb = cv2.absdiff(color_img,prev_rgb)
      diff_depth= cv2.absdiff(depth_img,prev_depth)

      prev_rgb = color_img.copy()
      prev_depth = depth_img.copy()

      gray_rgb = cv2.cvtColor(diff_rgb, cv2.COLOR_BGR2GRAY)
      gray_depth = diff_depth.copy()

      blur_rgb = cv2.GaussianBlur(gray_rgb,(5,5),0)
      blur_depth = cv2.GaussianBlur(gray_depth,(5,5),0)
       
      _, thresh_rgb = cv2.threshold(blur_rgb,20,255,cv2.THRESH_BINARY)
      _, thresh_depth = cv2.threshold(blur_depth,20,255,cv2.THRESH_BINARY)

      dilated_rgb = cv2.dilate(thresh_rgb,None,iterations=3)
      dilated_depth = cv2.dilate(thresh_depth,None,iterations=3)

      contours_rgb,_ = cv2.findContours(dilated_rgb,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      contours_depth,_ = cv2.findContours(dilated_depth,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

      contour_img_rgb=cv2.drawContours(color_img, contours_rgb,-1,(0,255,0),2)
      contour_img_depth=cv2.drawContours(depth_img, contours_depth,-1,(0,255,0),2)


      cv2.imshow("rgb",box_image )
      cv2.imshow("depth",contour_img_depth)
      cv2.imshow("first rgb",roi_rgb)
      cv2.imshow("first depth",roi_depth)
      k = cv2.waitKey(5) & 0xFF
      if k == ord('s'):
         cv2.destroyWindow("rgb")
    end()
    indy.disconnect()
if __name__ == '__main__':
    t1 = threading.Thread(target=detect_img)
    t1.start()
