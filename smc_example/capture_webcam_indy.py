import cv2
import numpy as np
import time
import os
from indydcp_client import IndyDCPClient
bind_ip = "192.168.0.6"   
server_ip = "192.168.0.7"
robot_name = "NRMK-Indy7"
indy = IndyDCPClient(bind_ip, server_ip, robot_name) 
indy.connect()
print("INDY CONNECTION : "+str(indy.is_connected()))
# Create a VideoCapture object
cap = cv2.VideoCapture(0)
 
# Check if camera opened successfully
if (cap.isOpened() == False): 
  print("Unable to read camera feed")
 
# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
 
# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
n = 0;
angle = 180;
indy.go_home()
time.sleep(3)
print("Number : ")
input_num = input()
print('test'+str(input_num))
try:
 os.mkdir('test'+str(input_num))
except:
 os.chdir('test'+str(input_num))
 pass
os.chdir('test'+str(input_num))
toggle = 1;
while(True):
  ret, frame = cap.read()
  angle = angle-toggle;
  if angle<0 : 
     angle = 0
     time.sleep(5)
     toggle = toggle*(-1)
  if angle>180 : 
     angle = 180
     time.sleep(5)
     toggle = toggle*(-1)
  randz = np.random.rand(1)*0.15-0.05
  randx = np.random.rand(1)*0.1
  randy = np.random.rand(1)*0.2-0.1
  indy.task_move_to([0.4+randx,0+randy,0.5+randz,-180,0,angle])
  if ret == True: 
     
    # Write the frame into the file 'output.avi'
    out.write(frame)
 
    # Display the resulting frame    
    cv2.imshow('frame',frame)
    filename = str(n)+".png"
    cv2.imwrite(filename,frame)
    n=n+1
    # Press Q on keyboard to stop recording
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else:
    indy.go_home()
    time.sleep(3)
    break 
indy.go_home()
time.sleep(3)
# When everything done, release the video capture and video write objects
cap.release()
out.release()
 
# Closes all the frames
cv2.destroyAllWindows()


