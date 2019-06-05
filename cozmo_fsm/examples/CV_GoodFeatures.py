"""
  CV_GoodFeatures demonstrates the Shi and Tomasi (1994) feature
  extractor built in to OpenCV.
"""

import cv2
import numpy as np
from cozmo_fsm import *

class CV_GoodFeatures(StateMachineProgram):
    def __init__(self):
        super().__init__(aruco=False, cam_viewer=False, annotate_sdk=False)

    def start(self):
        self.colors = np.random.randint(0,255,(101,3),dtype=np.int)
        dummy = numpy.array([[0]],dtype='uint8')
        super().start()

        cv2.namedWindow('features')
        cv2.imshow('features',dummy)
        cv2.createTrackbar('maxCorners','features',50,100,lambda self: None)
        cv2.createTrackbar('qualityLevel','features',10,1000,lambda self: None)
        cv2.createTrackbar('minDistance','features',5,50,lambda self: None)

    def user_image(self,image,gray):
        cv2.waitKey(1)
        maxCorners = max(1,cv2.getTrackbarPos('maxCorners','features'))
        quality = max(1,cv2.getTrackbarPos('qualityLevel','features'))
        cv2.setTrackbarPos('qualityLevel', 'features', quality) # don't allow zero
        minDist = max(1,cv2.getTrackbarPos('minDistance','features'))
        cv2.setTrackbarPos('minDistance', 'features', minDist) # don't allow zero
        qualityLevel = quality / 1000
        corners = cv2.goodFeaturesToTrack(gray, maxCorners, qualityLevel, minDist)
        (x,y,_) = image.shape
        image = cv2.resize(image,(y*2,x*2))
        i = 0
        for corner in corners:
            x,y = corner.ravel()
            x = int(x*2); y = int(y*2)
            color_index = (x+y) % self.colors.shape[0]
            color = self.colors[color_index].tolist()
            cv2.circle(image, (x, y), 4, color, -1)
            i += 1
        cv2.imshow('features',image)
