"""
  CV_Thresh demonstrates image thresholding in OpenCV, and
  independently, the Canny edge detector.
"""

import cv2
import numpy as np
from cozmo_fsm import *

class CV_Thresh(StateMachineProgram):
    def __init__(self):
        super().__init__(aruco=False, particle_filter=False, cam_viewer=True,
                         annotate_sdk = False)

    def start(self):
        cv2.namedWindow('edges')
        cv2.namedWindow('threshold')
        dummy = numpy.array([[0]])
        cv2.imshow('threshold',dummy)
        cv2.imshow('edges',dummy)

        cv2.createTrackbar('thresh','threshold',0,255,lambda self: None)
        cv2.setTrackbarPos('thresh', 'threshold', 100)

        cv2.createTrackbar('thresh1','edges',0,255,lambda self: None)
        cv2.createTrackbar('thresh2','edges',0,255,lambda self: None)
        cv2.setTrackbarPos('thresh1', 'edges', 50)
        cv2.setTrackbarPos('thresh2', 'edges', 150)

        super().start()

    def user_image(self,image,gray):
        # Thresholding
        self.thresh = cv2.getTrackbarPos('thresh','threshold')
        ret, self.im_thresh = cv2.threshold(gray, self.thresh, 255, cv2.THRESH_BINARY)

        # Canny edge detection
        self.thresh1 = cv2.getTrackbarPos('thresh1','edges')
        self.thresh2 = cv2.getTrackbarPos('thresh2','edges')
        self.im_edges = cv2.Canny(gray, self.thresh1, self.thresh2, apertureSize=3)

    def user_annotate(self,image):
        cv2.imshow('threshold',self.im_thresh)
        cv2.imshow('edges',self.im_edges)
        return image
