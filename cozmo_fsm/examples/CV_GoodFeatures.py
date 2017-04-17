"""
  CV_GoodFeatures demonstrates the Shi and Tomasi (1994) feature
  extractor built in to OpenCV.
"""

import cv2
import numpy as np
from cozmo_fsm import *

class CV_GoodFeatures(StateMachineProgram):
    def __init__(self):
        super().__init__(aruco=False, cam_viewer=True, annotate_cube = False)

    def start(self):
        cv2.namedWindow('features')
        dummy = numpy.array([[0]*320])
        cv2.imshow('features',dummy)

        cv2.createTrackbar('maxFeatures','features',50,100,lambda self: None)

        cv2.createTrackbar('qualityLevel','features',10,1000,lambda self: None)

        cv2.createTrackbar('minDistance','features',5,50,lambda self: None)

        self.colors = np.random.randint(0,255,(101,3),dtype=np.int)

        super().start()

    def user_image(self,image,gray):
        maxFeat = cv2.getTrackbarPos('maxFeatures','features')
        quality = max(1,cv2.getTrackbarPos('qualityLevel','features'))
        cv2.setTrackbarPos('qualityLevel', 'features', quality) # don't allow zero
        minDist = max(1,cv2.getTrackbarPos('minDistance','features'))
        cv2.setTrackbarPos('minDistance', 'features', minDist) # don't allow zero
        qualityLevel = quality / 1000
        self.corners = cv2.goodFeaturesToTrack(gray, maxFeat, qualityLevel, minDist)

    def user_annotate(self,image):
        if self.corners is None: return image
        i = 0
        for corner in self.corners:
            x,y = corner.ravel()
            x = int(x); y = int(y)
            color_index = (x+y) % self.colors.shape[0]
            color = self.colors[color_index].tolist()
            cv2.circle(image, (2*x,2*y), 3, color, -1)
            i += 1
        return image
