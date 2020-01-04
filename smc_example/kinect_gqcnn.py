
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from numpy.ctypeslib import ndpointer
import argparse
import json
import os
import time
import numpy as np

from primesense import openni2
from primesense import _openni2 as c_api
#--------------------------------indy------------------------------------#
from indydcp_client import IndyDCPClient

bind_ip = "192.168.0.6"   
server_ip = "192.168.0.7"
robot_name = "NRMK-Indy7"
indy = IndyDCPClient(bind_ip, server_ip, robot_name) 
indy.connect()
print("INDY CONNECTION : "+str(indy.is_connected()))

#-----------------------yolo-----------------------------------#
import sys
sys.path.append("./keras-yolo3-master")
from yolo import YOLO, detect_video
from PIL import Image
from matplotlib import pyplot as plt
FLAGS = None







#--------------------gripper--------------------------------------------__#

import os
import math
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from dynamixel_sdk import port_handler 
from dynamixel_sdk import packet_handler  
ADDR_OPERATING_MODE  =  11
ADDR_TORQUE_ENABLE    =  512
ADDR_GOAL_PWM         =  548
ADDR_GOAL_CURRENT     =  550
ADDR_GOAL_VELOCITY    =  552
ADDR_GOAL_POSITION    =  564
ADDR_MOVING           =  570
ADDR_PRESENT_VELOCITY = 576
ADDR_PRESENT_POSITION =  580

MIN_POSITION        =    0
MAX_POSITION        =    1150
MIN_PWM            =     0
MAX_PWM    =             2009

MIN_VELOCITY       =     0
MAX_VELOCITY       =     2970

MIN_CURRENT     =        0
MAX_CURRENT      =       1984

GRIPPER_ID          =    1
MOTOR_DXL_ID                      = 3  
ADDR_PRO_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 564
ADDR_PRO_PRESENT_POSITION   = 611
MOTOR_ADDR_PRO_TORQUE_ENABLE     =64
MOTOR_ADDR_PRO_GOAL_POSITION      = 116

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1               # Dynamixel ID : 1
BAUDRATE                    = 2000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1150           # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 80              # Dynamixel moving status threshold
index = 0 
portHandler = port_handler.PortHandler(DEVICENAME)
packetHandler = packet_handler.PacketHandler(PROTOCOL_VERSION)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,MOTOR_DXL_ID,MOTOR_ADDR_PRO_TORQUE_ENABLE  , TORQUE_ENABLE)

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error  =packetHandler.write1ByteTxRx(portHandler, GRIPPER_ID, ADDR_OPERATING_MODE, 0);
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY,2900)
current_limit = packetHandler.read2ByteTxRx(portHandler, DXL_ID, 38)

#--------------------gqcnn--------------------------------------------__#




from autolab_core import YamlConfig, Logger
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage,
                        RgbdImage)
from visualization import Visualizer2D as vis

from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
from gqcnn.utils import GripperMode
logger = Logger.get_logger("grabdepth_segmask_gqcnn.py")
import matplotlib.pyplot as plt
from ctypes import cdll
import threading
import time
import ctypes
import threading
from ctypes import cdll
import ctypes
from numpy.ctypeslib import ndpointer
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
#lib = cdll.LoadLibrary('./SimpleViewer.so')
best_center=[0 ,0]
center=[0 ,0]
prev_q_value=0
q_value=0
prev_depth=0

i=0
depth_mem=None
first_flag=True
toggle = 1
t=0

INITIAL = 0
FIND_LOC= 1
MOVE_TARGET = 2
GRIP = 3
GRIP_BOX= 4
MOVE_HOME= 5
ALL_RESET = 6




np.random.seed(0)

image = np.ones((240,320),dtype=float)

state = INITIAL
no_find = 0
loc_count = 0
count =0

lib = cdll.LoadLibrary('./viewer_opengl.so')
st = lib.Foo_start
end = lib.Foo_end
dataread =lib.Foo_dataread
dataread_color =lib.Foo_dataread_color
dataread_depth =lib.Foo_dataread_depth
dataread_color_to_depth =lib.Foo_dataread_color_to_depth
dataread.restype = ndpointer(dtype=ctypes.c_uint8, shape=(720,1280,2))
dataread_color.restype = ndpointer(dtype=ctypes.c_uint8, shape=(720,1280,4))
dataread_depth.restype = ndpointer(dtype=ctypes.c_uint16, shape=(512,512))#ctypes.POINTE

dataread_color_to_depth.restype = ndpointer(dtype=ctypes.c_uint8, shape=(512,512,4))
convert_2d_3d = lib.Foo_convert_2d_3d
convert_2d_3d.restype = ndpointer(dtype=ctypes.c_float, shape=(3))#ctypes.POINTE



def run_gripper():
  global count
  global grip_success
  global direction
  global direction_motor
  direction = -1
  direction_motor =1
  grip_threshold = 736
  print("RUN GRIPPPER!!!!!!!!!!!")
  while 1:
   #print("direction :           "+str(direction))   
   try:

    present_velo = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
    percent = 1
    rd = int(np.random.rand(1)*200)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID,ADDR_GOAL_CURRENT,direction*int(current_limit[0]*percent))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, MOTOR_DXL_ID, 112,750)

    motor_present_pos =    packetHandler.read4ByteTxRx(portHandler, MOTOR_DXL_ID, 132)
   
    if direction_motor == -1:
       if motor_present_pos[0]> 1000 and motor_present_pos[0]< 1050 :  
          dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler,MOTOR_DXL_ID,MOTOR_ADDR_PRO_GOAL_POSITION,motor_present_pos[0]-1000)
       else:
          pass
    elif direction_motor == 1:    
       if motor_present_pos[0]> 4000000000 or motor_present_pos[0]< 50 :  
          dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler,MOTOR_DXL_ID,MOTOR_ADDR_PRO_GOAL_POSITION,motor_present_pos[0]+1000)
    elif direction_motor == 0: 
       dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, MOTOR_DXL_ID, 112,200)
       dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler,MOTOR_DXL_ID,MOTOR_ADDR_PRO_GOAL_POSITION,1024)
       time.sleep(0.2)
       dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler,MOTOR_DXL_ID,MOTOR_ADDR_PRO_GOAL_POSITION,900)
       time.sleep(0.2)
       dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler,MOTOR_DXL_ID,MOTOR_ADDR_PRO_GOAL_POSITION,1024)
       direction_motor = 1
    is_move =    packetHandler.read1ByteTxRx(portHandler, DXL_ID, ADDR_MOVING)
    present_pos =    packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

    if present_pos[0] > 4000000000:
       present_pos_calc = present_pos[0]-4294967295
    else:
       present_pos_calc = present_pos[0]
    if is_move[0]==0 :

      # print(present_pos_calc)
       #print("**************************mean_sector : " +str(mean_sector))
       if present_pos_calc < grip_threshold-4 and present_pos_calc > 0 and mean_sector <50.0 :
          grip_success = 1
       elif present_pos_calc>=grip_threshold+4 and present_pos_calc < grip_threshold*2 and mean_sector <50.0:
          grip_success = 1
       elif present_pos_calc<0 and present_pos_calc>=grip_threshold*2 and mean_sector >1.0 :
          grip_success = 0
       else :
          grip_success = 0
       #print("grip_success : "+str(grip_success))

       count = count+1
       if count>2:
          count=0
    time.sleep(0.1)
   except:
    print("EXCEPTION GRIPPER RUN")


z= 0.5

def run_gqcnn():
   global depth_im
   global color_im
   global segmask
   global policy
   global im3
   global ax3
   global prev_q_value
   global prev_depth
   global toggle
   global t
   global x
   global y
   global z
   global best_angle
   global depth_raw
   global im1
   global ax1
   global num_find_loc
   global state
   global loc_count
   global no_find
   global center
   global q_value
   global angle
   global no_valid_grasp_count
   global no_valid_move_y 
   no_valid_grasp_count = 0;

   best_angle = 0
   x=0.0
   y=0.5
   while 1:
     try:
      if state==FIND_LOC:

       if num_find_loc == 0:
          loc_count = loc_count+1
          if loc_count >10 and no_find <0.2:
             no_find = no_find#+0.05
             loc_count = 0
       else:
           no_find = 0
           loc_count=0
       print("LOCCOUNT :             "+str(loc_count)+" NO_FIND"+str(no_find))  
       #depth_im = depth_im.inpaint(rescale_factor=inpaint_rescale_factor)
       if "input_images" in policy_config["vis"] and policy_config["vis"][
            "input_images"]:
             im1.set_data(depth_im)
             plt.pause(0.001)
             #pass
       # Create state.
       rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
       state_gqcnn = RgbdImageState(rgbd_im, camera_intr, segmask=segmask) 


       # Query policy.
       policy_start = time.time()
       try:
          action = 0
          if indy.is_move_finished()==1 and state==FIND_LOC:
            action = policy(state_gqcnn)
            logger.info("Planning took %.3f sec" % (time.time() - policy_start))
            no_valid_grasp_count =  0;
       except:
          no_valid_grasp_count = no_valid_grasp_count+1;
          time.sleep(0.3)
      # Vis final grasp.
       if policy_config["vis"]["final_grasp"]:
       # vis.imshow(segmask,
        #           vmin=policy_config["vis"]["vmin"],
       #           vmax=policy_config["vis"]["vmax"])
        #im3.set_data(rgbd_im.depth)

        center[0] = action.grasp.center[0]+96
        center[1] = action.grasp.center[1]+96
        q_value = action.q_value
        angle = float(action.grasp.angle)*180/3.141592
        print("center : \t"+str(action.grasp.center))
        print("angle : \t"+str(action.grasp.angle)) 
        print("Depth : \t"+str(action.grasp.depth)) 
        if(prev_q_value<action.q_value):

           #vis.grasp(action.grasp, scale=1, show_center=True, show_axis=True)
           #vis.title("Planned grasp at depth {0:.3f}m with Q={1:.3f}".format(
           #    action.grasp.depth, action.q_value))
           prev_q_value = action.q_value
           prev_depth = action.grasp.depth
           best_center[0] =action.grasp.center[0]+96
           best_center[1] =action.grasp.center[1]+96
           convert_data=convert_2d_3d(int(best_center[0]),int(best_center[1]))
           x = -1*convert_data[0]/1000 +no_valid_move_y 
           y = (-1*convert_data[1]/1000)+0.41
           z = convert_data[2]/1000

          # x=-(best_center[0]-592)*0.00065625 +0.00#592 
          # y=-(best_center[1]-361)*0.000673611+0.46   #361 
           best_angle = action.grasp.angle
           best_angle = float(best_angle)*180/3.141592
           
           print("gqcnn_best_center : \t"+str(x)+","+str(y))
           print("best_angle : \t"+str(best_angle))
           print("Q_value : \t" +str(action.q_value))
        num_find_loc = num_find_loc+1

       # vis.show()
        #plt.show()
        time.sleep(0.3);
        #plt.close()
      else:
        time.sleep(0.001);
     except:

       time.sleep(1);
       pass

def imnormalize(xmax,image):
    """
    Normalize a list of sample image data in the range of 0 to 1
    : image:image data.
    : return: Numpy array of normalize data
    """
    xmin = 0
    a = 0
    b = 255
    
    return ((np.array(image,dtype=np.float32) - xmin) * (b - a)) / (xmax - xmin)

def run_kinect():
    global image
    global depth_im
    global color_im
    global toggle
    global segmask
    global policy
    global image
    global im1
    global ax1
    global best_center
    global t
    global best_angle
    global prev_q_value
    global prev_depth
    global depth_img
    global depth_raw
    global state
    global no_find
    global z
    global center
    global q_value
    global angle
    global mean_sector
    global no_valid_grasp_count
    global color_to_depth_raw
    global grip_success
    global robot_collided
    global no_valid_move_y 

    global r_image
    global out_classes
    global out_boxes
    global r_class_names
    global out_scores
    if maskrcnn_run ==1:
       global img
       global result
       global color_img_raw


    print("_______________RUNKINECT_____________")
    cv2.namedWindow('DEPTH_REAL', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('DEPTH_REAL', 1000,1000)
    cv2.namedWindow('COLOR_TO_DEPTH', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('COLOR_TO_DEPTH', 1000,1000)
    cv2.namedWindow('SEGMENTATION', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('SEGMENTATION', 1000,1000)
    prev_z = 0


    class_color = {'apple':(0,0,255),'bae':(0,135,224),'banana':(0,255,255),'bread':(38, 201, 255),'cake':(157, 229, 252),'carrot':(0, 145, 255),'cucumber':(0, 184, 58),'gagi':(107, 0,98),'grape':(82, 0, 75),'green_apple':(0, 255, 179),'green_pepper':(0, 212, 149),'hamburger':(138, 218, 255),'kiwi':(85, 112, 95),'lemon':(0, 255, 251),'orange':(0, 200, 255),'peach':(217,230, 255),'pepper':(0,0,255),'pumkin':(
30, 100, 186),'tomato':(0,0,255)}
    max_image = 100
    while 1:
     try:
       depth_data  =np.array(dataread(),dtype=np.uint8)
       depth_real_data  =np.array(dataread_depth(),dtype=np.uint16)
       depth_real_img = depth_real_data[96:512-96,96:512-96]/850;
       
      # if max_image < depth_real_data.max():
       #   max_image = depth_real_data.max()
      # print(max_image)
       color_to_depth_data = np.array(dataread_color_to_depth(),dtype=np.uint8)
       color_to_depth_img = color_to_depth_data[96:512-96,96:512-96,0:3];
       color_to_depth_raw  = color_to_depth_img.copy()
       if maskrcnn_run ==1:
           color_img_raw = color_to_depth_raw.copy()
       color_data  =np.array(dataread_color(),dtype=np.uint8)
       depth_img_ = depth_data.copy()/65535
       depth_raw = depth_data[:,:,0]
       depth_temp= depth_img_[:,:,0]
       depth_img = depth_temp.astype("float32")

       depth_img = (depth_raw)/255

       color_img = color_data[:,:,0:3]


       depth_im =DepthImage(depth_real_img.astype("float32"), frame=camera_intr.frame)
       color_im = ColorImage(np.zeros([depth_im.height, depth_im.width,
                                    3]).astype(np.uint8),
                          frame=camera_intr.frame)



       hsv = cv2.cvtColor(color_to_depth_img, cv2.COLOR_BGR2HSV)
       lower_red = np.array([50, 50, 50])
       upper_red = np.array([180, 180, 180])
       mask = cv2.inRange(color_to_depth_img, lower_red, upper_red)
    
       res = cv2.bitwise_and(color_to_depth_img,color_to_depth_img,mask= mask)




       mask2 = np.mean(color_to_depth_img,axis=2)
       mask2[mask2>=50] = 255
       mask2[mask2<50] = 0
       mask2[mask2==255] = 1
       mask2[mask2==0] = 255
       mask2[mask2==1] = 0
       mask2 = np.uint8(mask2)

       mask[mask==255] = 1
       mask[mask==0] = 255
       mask[mask==1] = 0

       segmask_img  = depth_real_img.astype("float32")
       threshold = np.mean(segmask_img)*0.8
       #segmask_img[0:160,:] =  segmask_img[0:160,:]*1.1
       segmask_img[segmask_img<threshold] = 0
       segmask_img[segmask_img>threshold] = 255
       segmask_img[segmask_img==0] = 254
       segmask_img[segmask_img==255] = 0
       segmask_img[segmask_img==254] = 255

       #segmask_img[:130,:] = 0
       #segmask_img[257:,:] = 0
       #segmask_img[:,:37] = 0
       #segmask_img[:,282:] = 0
       segmask_img = np.uint8(segmask_img)

       try: 


       #yolo------------------


        try:
          label = ""
          for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = r_class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            #draw = ImageDraw.Draw(r_image)
            #label_size = draw.textsize(label, font)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(r_image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(r_image.size[0], np.floor(right + 0.5).astype('int32'))


            #scale = 0.7
            #center_x = (right-left)/2
            #center_y = (top-bottom)/2

            width = (right-left)
            #height = (top-bottom)*scale

            #right = int(center_x + width/2)
            #left = int(center_x - width/2)
            #top = int(center_y + height/2)
            #botom = int(center_y - height/2)

            label_name,label_percent =label.split(" ", maxsplit=2)
            if float(label_percent)>0.2 :
              color_to_depth_img=cv2.rectangle(color_to_depth_img, (left, top),(right, bottom),class_color[label_name], 1)
              if label_name =='gagi':
                 color_to_depth_img = cv2.putText(color_to_depth_img , "eggplant   "+str(label_percent), (left, top-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,class_color[label_name] ,1) 
              elif label_name =='bae':
                 color_to_depth_img = cv2.putText(color_to_depth_img , "pear   "+str(label_percent), (left, top-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,class_color[label_name] ,1) 
              else:
                 color_to_depth_img = cv2.putText(color_to_depth_img , label, (left, top-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,class_color[label_name] ,1) 
              #print(label, (left, top), (right, bottom))
        except:
            print("YOLO EXCEPTION KIIII")













        r = 40

        print("ddddddddddddddddddddddddddddddddd1")
        color_to_depth_img = cv2.putText(color_to_depth_img , "Q VALUE : "+str(round(prev_q_value,3)) , (200,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,255) ,1)
        if state == 1 and no_valid_grasp_count <1:
           color_to_depth_img = cv2.putText(color_to_depth_img , "FIND VALID GRASP", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(220,0,0) ,2) 
           color_to_depth_img = cv2.putText(color_to_depth_img , "FIND VALID GRASP", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,100,100) ,1)
          
           color_to_depth_img = cv2.line(color_to_depth_img, (int(center[0]-96-r*math.cos(angle/180*math.pi)),int(center[1]-96-r*math.sin(angle/180*math.pi))), (int(center[0]-96+r*math.cos(angle/180*math.pi)),int(center[1]-96+r*math.sin(angle/180*math.pi))), (143,143,143), 2)
           color_to_depth_img = cv2.line(color_to_depth_img, (int(center[0]-96),int(center[1]-96)), (int(center[0]-96+1),int(center[1]-96)) , (0,0,1), 3)
           color_to_depth_img = cv2.line(color_to_depth_img, (int(best_center[0]-96-r*math.cos(best_angle/180*math.pi)),int(best_center[1]-96-r*math.sin(best_angle/180*math.pi))), (int(best_center[0]-96+r*math.cos(best_angle/180*math.pi)),int(best_center[1]-96+r*math.sin(best_angle/180*math.pi))), (0,0,255), 2)
           color_to_depth_img = cv2.line(color_to_depth_img, (int(best_center[0]-96),int(best_center[1]-96)), (int(best_center[0]-96+1),int(best_center[1]-96)) , (0,136,255), 3)

           color_to_depth_img = cv2.putText(color_to_depth_img , "SECOND", (int(center[0]-96+5),int(center[1]-96+35)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0) ,2)     
           color_to_depth_img = cv2.putText(color_to_depth_img , "SECOND", (int(center[0]-96+5),int(center[1]-96+35)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(230,230,230) ,1)     
           #color_to_depth_img = cv2.putText(color_to_depth_img , "DEPTH : "+str(prev_z) , (int(center[0]-96+5),int(center[1]-96+50)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0) ,2)       
           #color_to_depth_img = cv2.putText(color_to_depth_img , "DEPTH : "+str(prev_z) , (int(center[0]-96+5),int(center[1]-96+50)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(230,230,230) ,1)       
           #color_to_depth_img = cv2.putText(color_to_depth_img , "DEPTH : "+str(z) , (int(best_center[0]-96+5),int(best_center[1]-96+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,255,255) ,2)
           #color_to_depth_img = cv2.putText(color_to_depth_img , "DEPTH : "+str(z) , (int(best_center[0]-96+5),int(best_center[1]-96+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,255) ,1)
           prev_z = z
           color_to_depth_img = cv2.putText(color_to_depth_img , "BEST", (int(best_center[0]-96+5),int(best_center[1]-96)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,255,255) ,2) 
           color_to_depth_img = cv2.putText(color_to_depth_img , "BEST", (int(best_center[0]-96+5),int(best_center[1]-96)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,255) ,1) 
        elif no_valid_grasp_count >0 and state==FIND_LOC:
           if no_valid_move_y > 0 :
              color_to_depth_img = cv2.putText(color_to_depth_img , "NO VALID GRASP <<"+str(no_valid_grasp_count ), (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255) ,2) 
              color_to_depth_img = cv2.putText(color_to_depth_img , "NO VALID GRASP <<"+str(no_valid_grasp_count ), (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,255) ,1) 
           elif no_valid_move_y == 0 :
              color_to_depth_img = cv2.putText(color_to_depth_img , "NO VALID GRASP |"+str(no_valid_grasp_count ), (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255) ,2) 
              color_to_depth_img = cv2.putText(color_to_depth_img , "NO VALID GRASP |"+str(no_valid_grasp_count ), (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,255) ,1)
           elif no_valid_move_y < 0 :
              color_to_depth_img = cv2.putText(color_to_depth_img , "NO VALID GRASP >>"+str(no_valid_grasp_count ), (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255) ,2) 
              color_to_depth_img = cv2.putText(color_to_depth_img , "NO VALID GRASP >>"+str(no_valid_grasp_count ), (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,255) ,1)
        elif grip_success ==1  and state == GRIP :
           color_to_depth_img = cv2.putText(color_to_depth_img , "SUCCESS GRASP", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,144,0) ,2) 
           color_to_depth_img = cv2.putText(color_to_depth_img , "SUCCESS GRASP", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,0) ,1) 
        elif grip_success ==0 and state == MOVE_TARGET:
           color_to_depth_img = cv2.putText(color_to_depth_img , "MOVING ", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,144) ,2) 
           color_to_depth_img = cv2.putText(color_to_depth_img , "MOVING", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255) ,1) 
        if state == ALL_RESET:
           color_to_depth_img = cv2.putText(color_to_depth_img , "RESET", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,200,200) ,2) 
           color_to_depth_img = cv2.putText(color_to_depth_img , "RESET", (0,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,255) ,1) 
        if robot_collided==1:
           color_to_depth_img = cv2.putText(color_to_depth_img , "COLLISION", (270,10), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(100,100,100) ,2) 
           color_to_depth_img = cv2.putText(color_to_depth_img , "COLLISION", (270,10), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(150,150,150) ,1)              


       except:
        pass

       res_mask =  cv2.bitwise_or(segmask_img,mask)
       res_mask = cv2.bitwise_or(res_mask,mask2)
       #res_mask[:52,:] = 0
       #res_mask[res_mask>100] = 255
       #res_mask[res_mask<=100] = 0
       
       #res_mask[0:73,:] = 0
       #res_mask[242:,:] = 0
       #res_mask[:,0:27] = 0
       #res_mask[:,282:] = 0

       segmask = BinaryImage(res_mask)
       depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_real_img*150, alpha=2), 1)
       #print("MEAN IMAGE")
       try:
          sector = color_to_depth_img.get()
          mean_sector = np.mean(sector[265:270,150:180])
          #print(mean_sector)
       except:
          sector = color_to_depth_img
          mean_sector = np.mean(sector[265:270,150:180])
          #print(mean_sector)
          pass
       #ddddddfdfdf= np.array(color_img_raw.get())
      # mean_img = np.array(color_img_raw[256:276,143:180],dtype=float)
       cv2.imshow('COLOR_TO_DEPTH',color_to_depth_img )
       cv2.imshow('DEPTH_REAL',np.uint8( depth_colormap ))
       cv2.imshow('SEGMENTATION',np.uint8( res_mask ))
       cv2.waitKey(1)





       time.sleep(0.01)
     except:
       print("KINECT_EXEPTION")
       time.sleep(0.01)
         



def open_gripper_():
    run_gripper(-1)
def close_gripper_():
    if(indy.is_collided() == 0):
        run_gripper(1)
def open_gripper():
    global direction
    direction = -1
    #t1 = threading.Thread(target=open_gripper_)
    #t1.start()
    #t1.join()
def close_gripper():
    global direction
    if(indy.is_collided() == 0):
        direction = 1
    #t1 = threading.Thread(target=close_gripper_)
    #t1.start()
    #t1.join()
def open_motor():
    global direction_motor
    if(indy.is_collided() == 0):
        direction_motor = -1
def close_motor():
    global direction_motor
    if(indy.is_collided() == 0):
        direction_motor = 1
def refresh_motor():
    global direction_motor
    if(indy.is_collided() == 0):
        direction_motor = 0

def reset_robot_():
    if(indy.is_in_resetting()==0):
        indy.reset_robot()
    while(indy.is_in_resetting()==0):
        if(indy.is_robot_ready()==1):
            break;
        time.sleep(0.1)
   

def reset_robot():
    t1 = threading.Thread(target=reset_robot_)
    t1.start()
    t1.join()

def move_home_():
    if(indy.is_in_resetting()==0 and indy.is_robot_ready()==1):
         indy.go_home()
    while 1:
        if(indy.is_move_finished()==1 or indy.is_collided()==1 or indy.is_in_resetting() ==1):
            check_collided()
            break

        time.sleep(0.1)
    
def move_home():
    t1 = threading.Thread(target=move_home_)
    t1.start()
    t1.join()

def move_xyz_(x,y,z,t1,t2,t3):
    if(indy.is_in_resetting()==0 and indy.is_robot_ready()==1):
         indy.task_move_to([x,y,z,t1,t2,t3])
    while 1:
        if(indy.is_move_finished()==1 or indy.is_collided()==1 or indy.is_in_resetting() ==1):
            check_collided()
            break

        time.sleep(0.1)
    
def move_xyz(x,y,z,t1,t2,t3):
    print("___________________MOVE______JOINT__________________ \t :"+str(x)+"   "+str(y)+"   "+str(z))
    t1 = threading.Thread(target=move_xyz_,args=(x,y,z,t1,t2,t3))
    t1.start()
    t1.join()
def move_joint_(t1,t2,t3,t4,t5,t6):
    if(indy.is_in_resetting()==0 and indy.is_robot_ready()==1):
         indy.joint_move_to([t1,t2,t3,t4,t5,t6])
    while 1:
        if(indy.is_move_finished()==1 or indy.is_collided()==1 or indy.is_in_resetting() ==1):
            check_collided()
            break

        time.sleep(0.1)
    
def move_joint(t1,t2,t3,t4,t5,t6):
    print("___________________MOVE______JOINT__________________ \t")
    t1 = threading.Thread(target=move_joint_,args=(t1,t2,t3,t4,t5,t6))
    t1.start()
    t1.join()

def move_xyz_by_(x,y,z,t1,t2,t3):
    if(indy.is_in_resetting()==0 and indy.is_robot_ready()==1):
         indy.task_move_by([x,y,z,t1,t2,t3])
    while 1:
        if(indy.is_move_finished()==1 or indy.is_collided()==1 or indy.is_in_resetting() ==1):
            check_collided()
            break

        time.sleep(0.1)
    
def move_xyz_by(x,y,z,t1,t2,t3):
    t1 = threading.Thread(target=move_xyz_by_,args=(x,y,z,t1,t2,t3))
    t1.start()
    t1.join()
def check_collided():
       global robot_collided
       global state
       if(indy.is_collided()==0):
           robot_collided = 1;
           print("-----------------------COLLIDED-----------------------")
           #state = FIND_LOC
           reset_robot()
           robot_collided = 0;
           time.sleep(0.1)

def move_indy():
    global no_valid_grasp_count
    global depth_im
    global color_im
    global toggle
    global segmask
    global policy
    global image
    global im1
    global ax1
    global best_center
    global t
    global x
    global y
    global z
    global best_angle
    global prev_q_value
    global depth_img
    global num_find_loc
    global grip_time
    global state
    global no_find
    global grip_success
    global no_valid_move_y 
    global robot_collided
    grip_success=0
    robot_collided = 0;
    state = INITIAL
    num_find_loc = 0
    grip_time = 0
    no_valid_move_y =0;
    indy.reset_robot()
    while 1:
      if toggle == -1 :
             raise ValueError("invalid thread id")
      try:
       if state == INITIAL:
          prev_q_value=0
          best_angle=0
          num_find_loc = 0
          x=0.0
          y=0.5
          z = 0.2

          depth_im = 0
          color_im =0
          segmask = 0
          grip_success = 0
          print("INITIAL STATE")
          open_gripper()
          reset_robot()
          move_xyz(0.4,0.0,0.5,0.0,-180,0)
          #move_joint(0.0,-1.73,80.17,0.0,101.56,0.0)
          state = FIND_LOC
       elif state == FIND_LOC:
          if(num_find_loc==0):
              prev_q_value=0
              move_xyz(0.4,no_valid_move_y ,0.5,0.0,-180,0)
              time.sleep(1)
          #print("FIND_LOC STATE : "+str(num_find_loc))
          if(num_find_loc > 3):
             if no_valid_grasp_count <=5 :
                state = MOVE_TARGET
             num_find_loc = 0
          #print("no valid grasp count : "+str(no_valid_grasp_count))
          if no_valid_grasp_count > 12 :
             print("----------ALL RESET-------------")
             num_find_loc = 0
             no_valid_move_y =0;
             state=ALL_RESET
             time.sleep(0.1)
          elif no_valid_grasp_count > 8 and no_valid_grasp_count <=12:
             print("-------FIND MOVE RIGHT-------")
             num_find_loc = 0
             no_valid_move_y = -0.04
             state = FIND_LOC
             time.sleep(0.3)
          elif no_valid_grasp_count > 5 and no_valid_grasp_count <=8:
             print("---------FIND MOVE LEFT---------")
             num_find_loc = 0
             no_valid_move_y = 0.07
             state = FIND_LOC
             time.sleep(0.3)
        #  else:
          #   num_find_loc = 0
        #     no_valid_move_y = 0
        #     state = FIND_LOC
        #     time.sleep(0.3)
         

          time.sleep(0.01)
       elif state == MOVE_TARGET:

          time.sleep(1)
          print("MOVE_TARGET STATE")
          inv_best_angle = best_angle*(-1)
          print("\t\t\t\t\t\t\tbest angle     :"+str(inv_best_angle))
          if x < -0.160:
             x = -0.160
          if x > 0.190:
             x = 0.190
          print("MOVE_TARGET STATE")
          if y < 0.28:
             y = 0.28
          if y < 0.28:
             y = 0.28
          if y > 0.53:
             y = 0.53
          print("MOVE_TARGET STATE")
          move_z = 0.5-z
          if move_z>0.4 :
             move_z = 0.4
             prev_q_value =0
             state = FIND_LOC
             continue

          go_down = 0.125
          grip_z = move_z +0.19-go_down 

          if grip_z <0.18 :
             grip_z = 0.18
          try:
           if(abs(inv_best_angle)>90):
             if(inv_best_angle<=0):

                  temp_x = x
                  temp_y = y
                  print("MOVE_TARGET STATE2-1")
                  move_xyz(temp_y,temp_x, move_z +0.19,0.0,-180,inv_best_angle+180)
                  move_xyz(temp_y,temp_x, grip_z,0.0,-180,inv_best_angle+180)
             elif(inv_best_angle>0):
                  temp_x = x
                  temp_y = y
                  print("MOVE_TARGET STATE2-2")
                  move_xyz(temp_y,temp_x, move_z +0.19 ,0.0,-180,inv_best_angle-180)
                  move_xyz(temp_y,temp_x, grip_z,0.0,-180,inv_best_angle-180)
           elif(abs(inv_best_angle)<=90):
                  temp_x = x
                  temp_y = y
                  print("MOVE_TARGET STATE2-3")
                  move_xyz(temp_y,temp_x, move_z  +0.19,0.0,-180,inv_best_angle)
                  move_xyz(temp_y,temp_x, grip_z,0.0,-180,inv_best_angle)
           state = GRIP
          except:
             print("MOVE FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
             prev_q_value =0
             state = INITIAL
          print("MOVE_TARGET STATE3")



       elif state == GRIP:
          prev_q_value =0
          best_angle = 0
          close_gripper()
          time.sleep(1)
          move_xyz_by(0,0,go_down,0,0,0)
          if grip_success == 0 :
             state = INITIAL
             continue;
          else:           
             if grip_success == 1 :
                 if grip_success == 0 :
                    state = INITIAL
                    continue;
                 move_xyz(0.4,-0.351,0.511,-180,0,180)
                 if grip_success == 0 :
                    state = INITIAL
                    continue;
             if grip_success == 1 :
                    
               # refresh_motor()
               # time.sleep(0.2)
                #open_motor()
                #time.sleep(1)
                #close_motor()
                if grip_success == 0 :
                   state = INITIAL
                   continue;
                state = MOVE_HOME
       elif state == MOVE_HOME:
          print("MOVE_HOME STATE")

          prev_q_value =0
          #random_angle = np.random.rand(1)*90-45
          #move_xyz_by(0,0,-0.2,0,0,random_angle)
          #move_home()

          if grip_success == 0 :
             state = INITIAL
          else :
          #move_joint(-50.29,10.47,-117.12,-1.01,-74.62,-50.06)
             if grip_success == 1 :
                if grip_success == 0 :
                   state = INITIAL
                   continue;
                rand_y= np.random.rand(1)*1/10-0.05
                rand_x= np.random.rand(1)*1/10-0.05
                move_xyz_by(rand_x,rand_y,-0.1,0,0,0)
                if grip_success == 0 :
                   state = INITIAL
                   continue;
                open_gripper()
                if grip_success == 0 :
                   state = INITIAL
                   continue;
                move_xyz_by(0,0,0.1,0,0,0)
                if grip_success == 0 :
                   state = INITIAL
                   continue;
   

          state = INITIAL
          #state = GRIP_BOX

       elif state == ALL_RESET:
          no_valid_grasp_count = 0
          no_valid_move_y = 0
          open_motor()
          time.sleep(2)
          close_motor()
          state = INITIAL

       #plt.pause(0.01)
      except:
         pass
    toggle=-1
 
if __name__ == '__main__':
    global depth_im
    global color_im
    global segmask
    global policy
    global im1
    global ax1
    global x
    global y
    global best_angle
    global depth_img
    global num_find_loc
    global grip_time
    global grip_success
    yolo=YOLO()

    model_name = "GQCNN-4.0-PJ"
    depth_im_filename = "dframe1.npy"
    segmask_filename = "segmask1.jpg"
    camera_intr_filename = "kinect.intr"
    config_filename = None
    fully_conv = 0
    maskrcnn_run = 0
    
    # Set model if provided.
    model_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"/home/sung/gqcnn/models")
    model_path = os.path.join(model_dir, model_name)

    # Get configs.
    model_config = json.load(open(os.path.join(model_path, "config.json"),
                                  "r"))

    try:
        gqcnn_config = model_config["gqcnn"]
        gripper_mode = gqcnn_config["gripper_mode"]
    except KeyError:
        gqcnn_config = model_config["gqcnn_config"]
        input_data_mode = gqcnn_config["input_data_mode"]
        if input_data_mode == "tf_image":
            gripper_mode = GripperMode.LEGACY_PARALLEL_JAW
        elif input_data_mode == "tf_image_suction":
            gripper_mode = GripperMode.LEGACY_SUCTION
        elif input_data_mode == "suction":
            gripper_mode = GripperMode.SUCTION
        elif input_data_mode == "multi_suction":
            gripper_mode = GripperMode.MULTI_SUCTION
        elif input_data_mode == "parallel_jaw":
            gripper_mode = GripperMode.PARALLEL_JAW
        else:
            raise ValueError(
                "Input data mode {} not supported!".format(input_data_mode))
    # Set config.
    if config_filename is None:
        if (gripper_mode == GripperMode.LEGACY_PARALLEL_JAW
                or gripper_mode == GripperMode.PARALLEL_JAW):
            if fully_conv:
                config_filename = os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), "..",
                    "/home/sung/gqcnn/cfg/examples/fc_gqcnn_pj.yaml")
            else:
                config_filename = os.path.join(
                    "gqcnn_pj_kinect.yaml")
        elif (gripper_mode == GripperMode.LEGACY_SUCTION
              or gripper_mode == GripperMode.SUCTION):
            if fully_conv:
                config_filename = os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), "..",
                    "/home/sung/gqcnn/cfg/examples/fc_gqcnn_suction.yaml")
            else:
                config_filename = os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), "..",
                    "/home/sung/gqcnn/cfg/examples/gqcnn_suction.yaml")


   # Read config.
    config = YamlConfig(config_filename)
    inpaint_rescale_factor = config["inpaint_rescale_factor"]
    policy_config = config["policy"]

    # Make relative paths absolute.
    if "gqcnn_model" in policy_config["metric"]:
        policy_config["metric"]["gqcnn_model"] = model_path
        if not os.path.isabs(policy_config["metric"]["gqcnn_model"]):
            policy_config["metric"]["gqcnn_model"] = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "..",
                policy_config["metric"]["gqcnn_model"])

    # Setup sensor.
    camera_intr = CameraIntrinsics.load(camera_intr_filename)

    # Read images.
    depth_data =np.load(depth_im_filename)
    #depth_data = np.array((depth_data-depth_data.min())/(depth_data.max()-depth_data.min()),dtype=float)
    depth_data = np.array((depth_data[:,:,0]-depth_data[:,:,0].min())/(depth_data[:,:,0].max()-depth_data[:,:,0].min()),dtype=float)
    depth_im = DepthImage(depth_data, frame=camera_intr.frame)
    color_im = ColorImage(np.zeros([depth_im.height, depth_im.width,
                                    3]).astype(np.uint8),
                          frame=camera_intr.frame)

    # Optionally read a segmask.
    segmask = None
    if segmask_filename is not None:
        segmask = BinaryImage.open(segmask_filename)
    valid_px_mask = depth_im.invalid_pixel_mask().inverse()
    if segmask is None:
        segmask = valid_px_mask
    else:
        segmask = segmask.mask_binary(valid_px_mask)
   # print("----------------------------------")


     # Set input sizes for fully-convolutional policy.
    if fully_conv:
         policy_config["metric"]["fully_conv_gqcnn_config"][
             "im_height"] = depth_im.shape[0]
         policy_config["metric"]["fully_conv_gqcnn_config"][
             "im_width"] = depth_im.shape[1]

     # Init policy.
    if fully_conv:
         # TODO(vsatish): We should really be doing this in some factory policy.
         if policy_config["type"] == "fully_conv_suction":
             policy = FullyConvolutionalGraspingPolicySuction(policy_config)
         elif policy_config["type"] == "fully_conv_pj":
             policy = FullyConvolutionalGraspingPolicyParallelJaw(policy_config)
         else:
             raise ValueError(
                "Invalid fully-convolutional policy type: {}".format(
                    policy_config["type"]))
    else:
         policy_type = "cem"
         if "type" in policy_config:
            policy_type = policy_config["type"]
         if policy_type == "ranking":
            policy = RobustGraspingPolicy(policy_config)
         elif policy_type == "cem":
            policy = CrossEntropyRobustGraspingPolicy(policy_config)
         else:
            raise ValueError("Invalid policy type: {}".format(policy_type))
    t6 = threading.Thread(target=run_gripper)
    t6.start()

    t0 = threading.Thread(target=st)
    t0.start()
    t1 = threading.Thread(target=run_kinect)
    t1.start()
    t3 = threading.Thread(target=run_gqcnn)
    t3.start()

    t4 = threading.Thread(target=move_indy)
    t4.start()

    if maskrcnn_run == 1 :
      t5 = threading.Thread(target=run_maskrcnn)
      t5.start()
    close_motor()
    #ax1 = plt.subplot(111)
    temp = np.ones([240,320,3])*255
    #im1 = ax1.imshow(temp)
    #plt.ion()
  

    plt.ion()

    global color_to_depth_raw
    global r_image
    global out_classes
    global out_boxes
    global r_class_names
    global out_scores
    color_to_depth_raw = np.zeros((320,320),dtype=np.uint8)
    while True:
        img = cv2.cvtColor(color_to_depth_raw.copy(),cv2.COLOR_BGR2RGB)
        img= cv2.resize(img,(320,320))
        imgc = img.copy()
        img = Image.fromarray(img)
        try:
          result = yolo.detect_image(img)
          r_image = result[0] 
          out_classes = result[1]
          r_class_names = result[2]
          out_boxes = result[3]
          out_scores = result[4]

 


            #if top - label_size[1] >= 0:
               # text_origin = np.array([left, top - label_size[1]])
           # else:
               # text_origin = np.array([left, top + 1])
            #print(text_origin)

         # plt.subplot(1,1,1)
         # plt.axis('off')
         # plt.imshow(r_image)
         
         # plt.show()
         # plt.pause(0.0001)
         #  plt.clf()

        except:
          print("YOLO EXCEPTION")
          pass
        time.sleep(0.01) 
    yolo.close_session()   
    plt.ioff()
    end()
