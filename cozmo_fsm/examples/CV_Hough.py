"""
  CV_Hough demonstrates OpenCV's HoughLines and probabilistic HoughLinesP
  primitives. The 'edges' window displays the results of a Canny edge operator
  that is the input to the Hough transform. The 'Hough' window shows the
  output of HoughLines with the given settings of the r and theta tolerances
  and minimum bin count (threshold). The 'HoughP' window shows the output of
  HoughLinesP using the r and theta values from the Hough window, plus the
  minLineLength and maxLineGap parameters and its own bin count threshold.
"""

import cv2
import numpy as np
from cozmo_fsm import *

class CV_Hough(StateMachineProgram):
    def __init__(self):
        super().__init__(aruco=False, particle_filter = False, cam_viewer=True,
                         annotate_sdk = False)

    def start(self):
        cv2.namedWindow('edges')
        cv2.namedWindow('Hough')
        cv2.namedWindow('HoughP')
        dummy = numpy.array([[0]])
        cv2.imshow('edges',dummy)
        cv2.imshow('Hough',dummy)
        cv2.imshow('HoughP',dummy)

        self.h_lines = None
        self.p_lines = None

        cv2.createTrackbar('thresh1','edges',0,255,lambda self: None)
        cv2.createTrackbar('thresh2','edges',0,255,lambda self: None)
        cv2.setTrackbarPos('thresh1','edges',50)
        cv2.setTrackbarPos('thresh2','edges',150)

        cv2.createTrackbar('r_tol','Hough',1,10,lambda self: None)
        cv2.createTrackbar('deg_tol','Hough',1,18,lambda self: None)
        cv2.createTrackbar('h_thresh','Hough',1,250,lambda self: None)
        cv2.createTrackbar('h_main','Hough',0,1,lambda self: None)
        cv2.setTrackbarPos('r_tol','Hough',2)
        cv2.setTrackbarPos('deg_tol','Hough',2)
        cv2.setTrackbarPos('h_thresh','Hough',120)
        cv2.setTrackbarPos('h_main','Hough',0)

        cv2.createTrackbar('minLineLength','HoughP',1,80,lambda self: None)
        cv2.createTrackbar('maxLineGap','HoughP',1,50,lambda self: None)
        cv2.createTrackbar('p_thresh','HoughP',1,250,lambda self: None)
        cv2.setTrackbarPos('minLineLength','HoughP',40)
        cv2.setTrackbarPos('maxLineGap','HoughP',20)
        cv2.setTrackbarPos('p_thresh','HoughP',20)
        cv2.createTrackbar('p_main','HoughP',0,1,lambda self: None)
        cv2.setTrackbarPos('p_main','HoughP',0)
        super().start()

    def user_image(self,image,gray):
        # Canny edge detector
        self.thresh1 = cv2.getTrackbarPos('thresh1','edges')
        self.thresh2 = cv2.getTrackbarPos('thresh2','edges')
        self.edges = cv2.Canny(gray, self.thresh1, self.thresh2, apertureSize=3)

        # regular Hough
        self.r_tol = max(0.1, cv2.getTrackbarPos('r_tol','Hough'))
        self.deg_tol = max(0.1, cv2.getTrackbarPos('deg_tol','Hough'))
        self.h_thresh = cv2.getTrackbarPos('h_thresh','Hough')
        self.h_lines = cv2.HoughLines(self.edges, self.r_tol,
                                      self.deg_tol/180.*np.pi,
                                      self.h_thresh)
        # probabilistic Hough
        self.p_thresh = cv2.getTrackbarPos('p_thresh','HoughP')
        self.minLineLength = cv2.getTrackbarPos('minLineLength','HoughP')
        self.maxLineGap = cv2.getTrackbarPos('maxLineGap','HoughP')
        self.p_lines = cv2.HoughLinesP(self.edges, self.r_tol, self.deg_tol/180.*np.pi,
                                       self.p_thresh, None,
                                       self.minLineLength, self.maxLineGap)

    def user_annotate(self,image):
        cv2.imshow('edges',self.edges)
        if self.h_lines is not None:
            hough_image = cv2.cvtColor(self.edges,cv2.COLOR_GRAY2BGR)
            h_main = cv2.getTrackbarPos('h_main','Hough')
            for line in self.h_lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*a)
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*a)
                cv2.line(hough_image,(x1,y1),(x2,y2),(0,255,0),1)
                if h_main:
                    cv2.line(image,(2*x1,2*y1),(2*x2,2*y2),(0,255,0),2)
            cv2.imshow('Hough',hough_image)
        if self.p_lines is not None:
            houghp_image = cv2.cvtColor(self.edges,cv2.COLOR_GRAY2BGR)
            p_main = cv2.getTrackbarPos('p_main','HoughP')
            for line in self.p_lines:
                x1,y1,x2,y2 = line[0]
                cv2.line(houghp_image,(x1,y1),(x2,y2),(255,0,0),1)
                if p_main:
                    cv2.line(image,(2*x1,2*y1),(2*x2,2*y2),(255,0,0),2)
            cv2.imshow('HoughP',houghp_image)
        return image
