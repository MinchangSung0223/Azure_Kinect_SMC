import sys
import argparse
from PIL import Image
from primesense import openni2  # , nite2
from primesense import _openni2 as c_api
from matplotlib import pyplot as plt
import time
import numpy as np
import cv2
import threading
from ctypes import cdll
import ctypes
from numpy.ctypeslib import ndpointer
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


def detect_img():

    n = 0
    while True:
      try:
        try:
          color_data = np.array(dataread_color(),dtype=np.uint8)
          color_img = color_data[:,:,0:3]
          img = cv2.cvtColor(color_img.copy(),cv2.COLOR_BGR2RGB)
          img = cv2.cvtColor(img.copy(),cv2.COLOR_RGB2BGR)
          x = input()
          filename = "kiwi_"+str(n)+".png"
          cv2.imwrite(filename, img)
          n = n+1
          print("SAVE "+filename)
          if n>31 : 
            end()
            break;
        except:
          pass

      except:
        end()
        yolo.close_session()

FLAGS = None

if __name__ == '__main__':
    t1 = threading.Thread(target=detect_img)
    t1.start()

