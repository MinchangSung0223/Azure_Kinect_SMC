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
classname = "kiwi"
classname1 = classname
smooth_rate = 100
classnumber = 1
scale_factor = 1.1
home_path=os.getcwd() 
img_list = os.listdir("/media/sung/613eb739-ef83-417c-a6da-b04fe4d9640e/home/sung/Downloads/data/coco/train2017")
try:
 os.mkdir(classname)
 os.mkdir(classname+"_fake")
except:
 os.chdir(classname)
 pass
os.chdir(classname)
classname = classname+"_"

def detect_img():

    n = 0
    prev_depth_data= np.array(dataread(),dtype=np.uint8)
    prev_depth_img=  prev_depth_data[:,:,0]
    color_data = np.array(dataread_color(),dtype=np.uint8)
    color_img = color_data[:,:,0:3]
    depth_to_color_data = np.array(dataread(),dtype=np.uint8)
    depth_to_color_img = depth_to_color_data[:,:,0]
    color_data = np.array(dataread_color(),dtype=np.uint8)
    color_img = color_data[:,:,0:3]
    depth_to_color_data = np.array(dataread(),dtype=np.uint8)
    depth_to_color_img = depth_to_color_data[:,:,0]    
    cv2.namedWindow("rgb", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("rgb", 1280,720)
    cv2.namedWindow("box and mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("box and mask", 1280,720)
    cv2.imshow("rgb",np.uint8(color_img))
    k = cv2.waitKey(5) & 0xFF
    if k == ord('s'):
       cv2.destroyWindow("rgb")
    time.sleep(1)
    angle = 180

    while True:
      print("angle : ",angle)
      angle = angle-1
      if angle < 0 :
           os.chdir(home_path)
           end()
           indy.go_home()
           break;

      randz = np.random.rand(1)*0.2-0.1
      indy.task_move_to([0.5,0,0.5+randz,-180,0,angle])
      start = time.time()
      while indy.is_move_finished()==0:
           pass
      print("time :", time.time() - start)      
      try:
        try:
          random_light = np.uint8(np.random.rand(1)*50+50)
          color_data = np.array(dataread_color(),dtype=np.uint8)
          M = np.ones(color_data[:,:,0:3].shape, dtype = "uint8") * random_light
          color_img = cv2.subtract(color_data[:,:,0:3], M)
          depth_to_color_data = np.array(dataread(),dtype=np.uint8)
          depth_to_color_img = depth_to_color_data[:,:,0]
      
          for r in range(0,360):
               depth_to_color_img[r,:] =depth_to_color_img[r,:]+(360-r)*15/360 

          depth_to_color_img[:,1000:] =depth_to_color_img[:,1000:]*1.2
          depth_to_color_img[:,:280] =depth_to_color_img[:,:280]*1.2
          derivative_img =np.zeros((720,1280),dtype=float)


          segment_img = depth_to_color_img.copy()
          segment_img[segment_img>158] = 0
          find_zero = segment_img<1
          derivative_img = np.subtract(segment_img,prev_depth_img)
          prev_depth_img = segment_img.copy()

          derivative_img = abs(derivative_img)
          derivative_img[derivative_img<3] = 255
          derivative_img[derivative_img<255] = 0
          derivative_img[find_zero ] = 0


          ret, img_binary = cv2.threshold(np.uint8(derivative_img.copy()), 127, 255, 0)
          contours, hierachy = cv2.findContours(img_binary.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
          print("imread ok")
          max_area = 0
          area_threshold = 5000
          max_cnt = 0
          for cnt in contours:
             M = cv2.moments(cnt)
             if max_area < M['m00']: 
                 max_area = M['m00']
                 max_cnt = cnt
          #print(max_area)
    
          if max_area < area_threshold:

             #k = input()
             continue;
          c0 = max_cnt
          #print(c0.shape)
          #print("c0 ____",c0)
          x, y, w, h = cv2.boundingRect(c0)
          #derivative_img[derivative_img<255] = 0
           
          contour_img = cv2.drawContours(color_img.copy(), contours, -1, (255,0,0), 2)
          box_img = cv2.rectangle(color_img.copy(), (x, y), (x+w, y+h), (255,0,0), 2)
          #print(x, y, w, h)
          x_ = float(x+w/2)/1280
          y_ = float(y+h/2)/720
          w_ = float(w)/1280
          h_ = float(h)/720

          filename = classname+str(n)+".png"
          txt_filename = classname+str(n)+".txt"
          txt_mask_filename = classname+str(n)+"_mask.txt"
          txt_c0 = np.reshape(c0,(-1,2))
          c0_x= txt_c0[:,0]
          c0_y= txt_c0[:,1]
          #print(c0_x)
          #print("-----------------------------")
          tck, u = splprep([c0_x,c0_y], s =0, per=True)
          u_new = np.linspace(u.min(), u.max(), smooth_rate)
          x_new, y_new = splev(u_new, tck)

          tck, u = splprep([x_new, y_new], s =0, per=True)
          u_new = np.linspace(u.min(), u.max(), 100)
          x_new, y_new = splev(u_new, tck)
          cx_new = np.mean(x_new)
          cy_new = np.mean(y_new)

          #print(x_new)
          x_new = (x_new-cx_new)*scale_factor+cx_new-5
          y_new = (y_new-cy_new)*scale_factor+cy_new-5

          res_array =np.zeros((100,2),dtype=int)
          res_array[:,0] = np.array(np.transpose(x_new),dtype=int)
          res_array[:,1] = np.array(np.transpose(y_new),dtype=int)
          txt_c0 = res_array
          #print(res_array)
          image_id = n;
          image_id = 0
          #print(txt_c0.shape)

          try:
            f = open(txt_filename, 'w')

            txt_data = str(classnumber)+" "+ str(x_)+" "+ str(y_)+" "+ str(w_)+" "+ str(h_)
            f.write(txt_data)
            f.close()
          except:
            f.close()

          try:
            f2 = open(txt_mask_filename, 'w')
            txt_data2 = "{\"segmentation\": [\n[\n"
            ct = 0
            
            for r in txt_c0:
               if ct==0:
                  txt_data2=txt_data2+ str(r[0])+","+str(r[1])+"\n"
                  ct = 1
               else :
                  txt_data2=txt_data2+","+str(r[0])+","+str(r[1])+"\n"
            txt_data2=txt_data2+"]],\n\"area\":"+str(max_area)+",\n \"iscrowd\": 0,\n \"image_id:\""+str(image_id)+",\n\"bbox\":[\n"+str(x)+",\n"+str(y)+",\n"+str(w)+",\n"+str(h)+"\n],\n\"category_id\":"+str(classnumber)+",\n\"id\":"+str(image_id)+"}"
            f2.write(txt_data2)
          except:
            f2.close()
         # print(txt_data2)
          txt_c0_=np.reshape(txt_c0,(1,-1,2))
          #print(txt_c0_.shape)
          #print(max_cnt.shape)
         # print(txt_c0_)

          show_img = cv2.drawContours(box_img.copy(), txt_c0_, 0, (0,0,255), 4,cv2.FILLED)
          zero_img = np.zeros((720,1280))
          mask_img = cv2.drawContours(zero_img, txt_c0_, -1, (255,255,255), -1)
          ret, mask_img = cv2.threshold(np.uint8(mask_img.copy()), 127, 255, 0)
          print(mask_img.shape)
          res_mask =cv2.bitwise_and(color_img,color_img, mask = mask_img)
          filename_angle ="a_"+classname+str(n)+"_"+str(angle)+".png"
          cv2.imwrite(filename_angle, res_mask)
          #hsv = cv2.cvtColor(res_mask, cv2.COLOR_BGR2HSV)
          #lower_gray = np.array([50, 50, 50])
          #upper_gray = np.array([110, 110, 110])
          #mask_color = cv2.inRange(res_mask, lower_gray, upper_gray)
          #mask_color[mask_color==255] = 1
          #mask_color[mask_color==0] = 255
          #mask_color[mask_color==1] = 0


          #res_mask = cv2.bitwise_and(res_mask,res_mask,mask= mask_color)          
          mask_bg = mask_img.copy()
          mask_bg[mask_bg == 255] = 1
          mask_bg[mask_bg == 0] = 255
          mask_bg[mask_bg == 1] = 0
          mask_bg[mask_bg == 255] = 1
          for i in range(0,100):
             rand_filenum = int(np.random.rand(1)*len(img_list))
             print(rand_filenum)
             train_img_filename ="/media/sung/613eb739-ef83-417c-a6da-b04fe4d9640e/home/sung/Downloads/data/coco/train2017/"+img_list[rand_filenum]
             print(train_img_filename)
             background_img = cv2.imread(train_img_filename,cv2.IMREAD_COLOR)
             bg_img = cv2.resize(background_img ,(1280,720),interpolation=cv2.INTER_CUBIC)
             res = cv2.bitwise_and(bg_img,bg_img, mask = mask_bg)

             res_mask_ = cv2.bitwise_or(res_mask,res)
             filename2= "f_"+classname+str(n)+"_"+str(rand_filenum)+".png"
             cv2.imwrite(filename2, res_mask_)
             txt_filename2 = "f_"+classname+str(n)+"_"+str(rand_filenum)+".txt"
             txt_mask_filename2 = "f_"+classname+str(n)+"_"+str(rand_filenum)+"_mask.txt"
             shutil.copy(txt_filename,txt_filename2)
             shutil.copy(txt_mask_filename,txt_mask_filename2)
          cv2.imshow("rgb",np.uint8(res_mask_))
          cv2.imshow("box and mask",np.uint8(show_img))

          k = cv2.waitKey(5) & 0xFF
          if k == ord('s'):
            cv2.destroyWindow("rgb")
          cv2.imwrite(filename, color_img)          


          n = n+1
          print("SAVE "+filename)

        except :
          pass

      except:
        os.chdir(home_path)
        indy.go_home()
        end()
        yolo.close_session()

FLAGS = None

if __name__ == '__main__':
    t1 = threading.Thread(target=detect_img)
    t1.start()




