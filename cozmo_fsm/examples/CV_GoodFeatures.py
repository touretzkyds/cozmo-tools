"""
  CV_GoodFeatures demonstrates the Shi and Tomasi (1994) feature
  extractor built in to OpenCV.
"""

import cv2
import numpy as np
from cozmo_fsm import *

class CV_GoodFeatures(StateMachineProgram):
    def __init__(self):
        super().__init__(aruco=True, particle_filter=False, cam_viewer=True,
                         annotate_cube = False)

    def start(self):
        cv2.namedWindow('features')
        dummy = numpy.array([[0]*320])
        cv2.imshow('features',dummy)

        cv2.createTrackbar('maxFeatures','features',1,100,lambda self: None)
        cv2.setTrackbarPos('maxFeatures', 'features', 50)

        cv2.createTrackbar('qualityLevel','features',1,100,lambda self: None)
        cv2.setTrackbarPos('qualityLevel', 'features', 10)

        cv2.createTrackbar('minDistance','features',0,50,lambda self: None)
        cv2.setTrackbarPos('minDistance', 'features', 5)

        self.colors = np.random.randint(0,255,(101,3))

        super().start()

    def user_image(self,image,gray):
        maxFeat = cv2.getTrackbarPos('maxFeatures','features')
        quality = cv2.getTrackbarPos('qualityLevel','features') / 1000
        minDist = cv2.getTrackbarPos('minDistance','features')
        self.corners = cv2.goodFeaturesToTrack(gray, maxFeat, quality, minDist)

    def user_annotate(self,image):
        i = 0
        for corner in self.corners:
            x,y = corner.ravel()
            color = [int(c) for c in self.colors[(x+y)%self.colors.shape[0]]]
            # Must write x+x instead of 2*x to maintain type float32:
            cv2.circle(image, (x+x,y+y), 3, color, -1)
            i += 1
        return image
