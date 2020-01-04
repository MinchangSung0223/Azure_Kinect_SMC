import sys
import argparse
from PIL import Image
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
          color_to_depth_data = np.array(dataread_color_to_depth(),dtype=np.uint8)
          color_to_depth_img = color_to_depth_data[96:512-96,96:512-96,0:3]
          img = cv2.cvtColor(color_to_depth_img.copy(),cv2.COLOR_BGR2RGB)
          img = cv2.cvtColor(img.copy(),cv2.COLOR_RGB2BGR)
          cv2.imshow("RGB", img)
          cv2.waitKey(1)
        except:
          pass

      except:
        end()
        yolo.close_session()

FLAGS = None

if __name__ == '__main__':
    t1 = threading.Thread(target=detect_img)
    t1.start()

