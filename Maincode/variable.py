import pandas as pd
import numpy as np
import time 
import math
import ctypes as ct
import ctypes.wintypes as wt
import cv2 as cv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import drawnow
import csv

from math import *
from multiprocessing import shared_memory

# Real time variable ---------------------------------------

time_curr  = 0
time_cnt   = 0
time_stime = 0 
time_ts    = 0.2
time_final = 5000

# sundry variable ------------------------------------------

colorYellow    = (25, 255, 255)
colorWhite     = (255, 255, 255)
colorRed       = (0, 0, 255)
colorBlue      = (255, 0, 0)
colorGreen     = (0, 255, 0)
userFont       = cv.FONT_HERSHEY_COMPLEX
D2R            = pi/180
R2D            = 180/pi

# Calibration variable -------------------------------------

Alpha          = 90 * D2R
Beta           = 0  * D2R
Gamma          = 0  * D2R

nLidar         = 16000
camHeight      = 0.2
camRecede      = 0.3
focalLen       = 0.00367
imgWidth       = 640
imgHeight      = 480
fovX           = 60.92   *  D2R
fovY           = 53.1432 *  D2R
ox             = imgWidth/2                                              
oy             = imgHeight/2
sx             = focalLen * math.tan(0.5 * fovX)/(0.5 * imgWidth) 
sy             = focalLen * math.tan(0.5 * fovY)/(0.5 * imgHeight)

rotX = np.array([[1   ,          0        ,         0        ], 
                 [0   ,   np.cos(Alpha)   ,   -np.sin(Alpha) ], 
                 [0   ,   np.sin(Alpha)   ,    np.cos(Alpha) ]])   

rotY = np.array([[np.cos(Beta)       ,   0   ,    np.sin(Beta) ], 
                 [    0              ,   1   ,        0        ], 
                 [-np.sin(Beta)      ,   0   ,    np.cos(Beta) ]])

rotZ = np.array([[np.cos(Gamma)      ,   -np.sin(Gamma)  ,    0 ], 
                 [np.sin(Gamma)      ,   np.cos(Gamma)   ,    0 ], 
                 [    0              ,        0          ,    1 ]])        
        
rotMat   = rotZ @ rotY @ rotX

transMat = np.array([[      0       ],
                     [   camHeight  ],
                     [   camRecede  ]])
        
extMat = np.hstack((rotMat, transMat))

intMat = np.array([[ focalLen/sx    ,        0      ,    ox ],        
                   [      0         ,  focalLen/sy  ,    oy ],
                   [      0         ,        0      ,     1 ]])  
        
projectionMat = intMat @ extMat