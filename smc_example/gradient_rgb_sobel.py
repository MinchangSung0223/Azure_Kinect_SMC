import cv2
import numpy as np
import math
import os

classname ="glue"
classnumber = 4
dir_path = os.getcwd()
os.mkdir(dir_path+"/"+classname)
os.chdir(classname)
capture = cv2.VideoCapture("../glue3.mp4")
n= 0
while True:
    if(capture.get(cv2.CAP_PROP_POS_FRAMES) == capture.get(cv2.CAP_PROP_FRAME_COUNT)):
        capture.open("Image/Star.mp4")
    ret, frame = capture.read()

    img = np.array(frame)
    gaussian_img = cv2.GaussianBlur(img, (3,3), 0)
    gray_gaussian_img = cv2.cvtColor( gaussian_img,cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray_gaussian_img,cv2.CV_8U,1,0,ksize=3)
    sobely = cv2.Sobel(gray_gaussian_img,cv2.CV_8U,0,1,ksize=3)
    gradient_img = sobelx+sobely;
    print(img.shape)
    width =img.shape[0]
    height = img.shape[1]

    cv2.imshow('rgb',img)
    #cv2.imshow('sobelx',sobelx)
    #cv2.imshow('sobely',sobely)
    cv2.imshow('gradient',gradient_img)


    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sensitivity = 140
    lower_white = np.array([0,0,255-sensitivity], dtype=np.uint8)
    upper_white = np.array([255,sensitivity,255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    mask = ~mask
    res = cv2.bitwise_and(img,img, mask= mask)
    
    ret, img_binary = cv2.threshold(np.uint8(mask.copy()), 127, 255, 0)
    contours, hierachy =cv2.findContours(img_binary.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    area_threshold = 5000
    max_cnt = 0
    for cnt in contours:
       M = cv2.moments(cnt)
       if max_area < M['m00']: 
           max_area = M['m00']
           max_cnt = cnt
    c0 = max_cnt
    x, y, w, h = cv2.boundingRect(c0)
    x_ = float(x+w/2)/width
    y_ = float(y+h/2)/height
    w_ = float(w)/width
    h_ = float(h)/height
    contour_img = cv2.drawContours(img.copy(), contours, -1, (255,0,0), 2)
    box_img = cv2.rectangle(img.copy(), (x, y), (x+w, y+h), (255,0,0), 2)

    #cv2.imshow('rgb',res)
    #cv2.imshow('mask',mask)

    cv2.imshow('contour_img ',contour_img )
    cv2.imshow('box_img ',box_img )

    #cv2.imshow("VideoFrame", frame)
    # save

    filename = classname+str(n)+".png"
    txt_filename = classname+str(n)+".txt"
    cv2.imwrite(filename, frame) 
    try:
        f = open(txt_filename, 'w')

        txt_data = str(classnumber)+" "+ str(x_)+" "+ str(y_)+" "+ str(w_)+" "+ str(h_)
        f.write(txt_data)
        f.close()
    except:
        f.close()
    n = n+1
    if cv2.waitKey(33) > 0: break

capture.release()
cv2.destroyAllWindows()


img = cv2.imread('rgb.jpg')

