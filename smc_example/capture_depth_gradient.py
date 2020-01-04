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
try:
 os.mkdir(classname)
except:
 os.chdir(classname)
 pass
os.chdir(classname)
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
    for i in range(0,100):
      color_data = np.array(dataread_color(),dtype=np.uint8)
      color_img = color_data[:,:,0:3]
      depth_to_color_data = np.array(dataread(),dtype=np.uint8)
      depth_to_color_img = depth_to_color_data[:,:,0]

    cv2.namedWindow("rgb", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("rgb", 1280,720)
    cv2.namedWindow("fake", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("fake", 1280,720)
    cv2.namedWindow("fake2", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("fake2", 1280,720)
    cv2.imshow("rgb",np.uint8(color_img))
    angle = 180
    n=0;
    indy.go_home()
    time.sleep(3)
    while True:
      angle = angle-2
      if angle < 0 :
           os.chdir(home_path)
           end()
           indy.go_home()
           break;
      randz = np.random.rand(1)*0.1-0.05
      randx = np.random.rand(1)*0.1
      randy = np.random.rand(1)*0.2-0.1
      n=n+1
      indy.task_move_to([0.4+randx,0+randy,0.35+randz,-180,0,angle])
      while indy.is_move_finished()==0:
           pass
      try:
        color_data = np.array(dataread_color(),dtype=np.uint8)
        color_img = color_data[:,:,0:3]
        M = np.ones(color_img.shape, dtype = "uint8") * darker
        color_img = cv2.subtract(color_img, M)
        depth_to_color_data = np.array(dataread(),dtype=np.uint8)
        depth_to_color_img = depth_to_color_data[:,:,0]
        width = color_img.shape[1]
        height = color_img.shape[0]
        if rgb_segmentation ==1:
          hsv = cv2.cvtColor(color_img , cv2.COLOR_RGB2HSV)
          lower_white = np.array([25,10,100], dtype=np.uint8)
          upper_white = np.array([90,255,255], dtype=np.uint8)
          mask1 = cv2.inRange(hsv, lower_white, upper_white)    
          mask1 = ~mask1

          color_binary1 = cv2.bitwise_and(color_img,color_img, mask= mask1)
          grayscale = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)


          color_ret, mask2 = cv2.threshold(grayscale , 127, 255, 0)
          mask = cv2.bitwise_and(mask1,mask2)
          color_ret, color_img_binary1 = cv2.threshold(np.uint8(mask1.copy()), 220, 255, 0)
          contours, color_hierachy =cv2.findContours(color_img_binary1.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
          contour_img = cv2.drawContours(color_img.copy(), contours, -1, (255,0,0), 2)
        else :
          depth_to_color_img[depth_to_color_img>157] = 0
          #depth_to_color_img[depth_to_color_img==0]=depth_to_color_img.max()
          gaussian_depth = cv2.GaussianBlur(depth_to_color_img,(3,3),0)
          dx = cv2.Sobel(gaussian_depth ,cv2.CV_64F,1,0,ksize=3)/2
          dy = cv2.Sobel(gaussian_depth ,cv2.CV_64F,0,1,ksize=3)/2
          gradient_img = np.zeros(dx.shape, dtype = "float")
          gradient_img = np.sqrt(np.square(dx)+np.square(dy))
          gradient_img[gradient_img<gradient_img.max()-100]=0
          gradient_gaussian_depth = cv2.GaussianBlur(gradient_img,(5,5),0)
          ret, img_binary = cv2.threshold(np.uint8(gradient_gaussian_depth), 127, 255, 0)
          contours, hierachy = cv2.findContours(img_binary.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
          contour_img = cv2.drawContours(color_img.copy(), contours, -1, (255,0,0), 2)



        max_area = 0
        area_threshold = 5000
        max_cnt = 0
        for cnt in contours:
           M = cv2.moments(cnt)
           if max_area < M['m00']: 
               max_area = M['m00']
               max_cnt = cnt
        c0 = max_cnt

        if max_area > 0:
           txt_c0 = np.reshape(c0,(-1,2))
           x, y, w, h = cv2.boundingRect(txt_c0)
           x_ = float(x+w/2)/width
           y_ = float(y+h/2)/height
           w_ = float(w)/width
           h_ = float(h)/height
           c0_x= txt_c0[:,0]
           c0_y= txt_c0[:,1]
           tck, u = splprep([c0_x,c0_y], s =0, per=True)
           smooth_rate = np.random.rand(1)*200+10
           u_new = np.linspace(u.min(), u.max(), smooth_rate)
           x_new, y_new = splev(u_new, tck)

           tck, u = splprep([x_new, y_new], s =0, per=True)
           u_new = np.linspace(u.min(), u.max(), 100)
           x_new, y_new = splev(u_new, tck)
           cx_new = np.mean(x_new)
           cy_new = np.mean(y_new)
           scale_factor = np.random.rand(1)*0.1+0.95
           x_new = (x_new-cx_new)*scale_factor+cx_new
           y_new = (y_new-cy_new)*scale_factor+cy_new
           res_array =np.zeros((100,2),dtype=int)
           res_array[:,0] = np.array(np.transpose(x_new),dtype=int)
           res_array[:,1] = np.array(np.transpose(y_new),dtype=int)
           txt_c0 = res_array
           txt_c0_=np.reshape(txt_c0,(1,-1,2))   
           box_img = cv2.rectangle(color_img.copy(), (x, y), (x+w, y+h), (255,0,0), 2) 
           show_img = cv2.drawContours(color_img.copy(), txt_c0_, 0, (0,0,255), 4,cv2.FILLED)
           cv2.imshow('rgb',color_img_binary1)#show_img)
           cv2.imshow('box_img ',box_img )    
           filename = classname+str(n)+".png"
           txt_filename = classname+str(n)+".txt"
           txt_mask_filename = classname+str(n)+"_mask.txt"
           image_id = 0
           try:
             
             f = open(txt_filename, 'w')
             cv2.imwrite(filename,color_img)
             txt_data = str(classnumber)+" "+ str(x_)+" "+ str(y_)+" "+ str(w_)+" "+ str(h_)
             f.write(txt_data)
             f.close()
           except:
             print("txt write error")
             f.close()
           zero_img = np.zeros((720,1280))
           mask_img = cv2.drawContours(zero_img, txt_c0_, -1, (255,255,255), -1)
           ret, mask_img = cv2.threshold(np.uint8(mask_img.copy()), 127, 255, 0)
           res_mask =cv2.bitwise_and(color_img,color_img, mask = mask_img)
           mask_bg = mask_img.copy()
           mask_bg[mask_bg == 255] = 1
           mask_bg[mask_bg == 0] = 255
           mask_bg[mask_bg == 1] = 0
           mask_bg[mask_bg == 255] = 1

           for i in range(0,1):
             rand_filenum = int(np.random.rand(1)*len(img_list))
             rand_filenum2 = int(np.random.rand(1)*len(img_list_user))
             print("i :"+str(i))
             train_img_filename ="/media/sung/613eb739-ef83-417c-a6da-b04fe4d9640e/home/sung/Downloads/data/coco/train2017/"+img_list[rand_filenum]
             train_img_filename2 = "/home/sung/yolo_data/Camera/"+img_list_user[rand_filenum2]

             background_img = cv2.imread(train_img_filename,cv2.IMREAD_COLOR)
             background_img2 = cv2.imread(train_img_filename2,cv2.IMREAD_COLOR)

             bg_img = cv2.resize(background_img ,(1280,720),interpolation=cv2.INTER_CUBIC)
             bg_img2 = cv2.resize(background_img2 ,(1280,720),interpolation=cv2.INTER_CUBIC)

             res = cv2.bitwise_and(bg_img,bg_img, mask = mask_bg)
             res2 = cv2.bitwise_and(bg_img2,bg_img2, mask = mask_bg)

             res_mask_ = cv2.bitwise_or(res_mask,res)
             res_mask2_ = cv2.bitwise_or(res_mask,res2)
             multiplyer = int(np.random.rand(1)*50)
             M = np.ones(res_mask_.shape, dtype = "uint8") * multiplyer
             if int(np.random.rand(1)) >0.5:
                  res_mask_ = cv2.subtract(res_mask_, M)
             else:
                  res_mask_ = cv2.add(res_mask_, M)
             filename2= "f_"+classname+str(n)+"_"+str(rand_filenum)+".png"
             filename2_user= "f_u_"+classname+str(n)+"_"+str(rand_filenum2)+".png"
             cv2.imwrite(filename2, res_mask_)
             cv2.imwrite(filename2_user, res_mask2_)
             txt_filename2 = "f_"+classname+str(n)+"_"+str(rand_filenum)+".txt"
             txt_filename2_user = "f_u_"+classname+str(n)+"_"+str(rand_filenum2)+".txt"

             shutil.copy(txt_filename,txt_filename2)
             shutil.copy(txt_filename,txt_filename2_user)
             fake_box_img = cv2.rectangle(res_mask_.copy(), (x, y), (x+w, y+h), (255,0,0), 2) 
             fake_box_img_user = cv2.rectangle(res_mask2_.copy(), (x, y), (x+w, y+h), (255,0,0), 2) 
             cv2.imshow('fake',fake_box_img ) 
             cv2.imshow('fake2',fake_box_img_user ) 
            # shutil.copy(txt_mask_filename,txt_mask_filename2)
             k = cv2.waitKey(5) & 0xFF
             if k == ord('s'):
               cv2.destroyWindow("rgb")
 

      except:
        end()
        indy.disconnect()
    end()
    indy.disconnect()
if __name__ == '__main__':
    t1 = threading.Thread(target=detect_img)
    t1.start()
