import cv2
import numpy as np
from cozmo_fsm import *

class CV_Contour(StateMachineProgram):
    def __init__(self):
        self.colors = [(0,0,255), (0,255,0), (255,0,0),
                       (255,255,0), (255,0,255), (0,255,255),
                       (0,0,128), (0,128,0), (128,0,0),
                       (128,128,0), (0,128,128), (128,0,128),
                       (255,255,255)]
        super().__init__(aruco=False, particle_filter=False, cam_viewer=False,
                         force_annotation=True, annotate_sdk=False)

    def start(self):
        super().start()
        dummy = numpy.array([[0]*320], dtype='uint8')
        cv2.namedWindow('contour')
        cv2.imshow('contour',dummy)

        cv2.createTrackbar('thresh1','contour',0,255,lambda self: None)
        cv2.setTrackbarPos('thresh1','contour',100)

        cv2.createTrackbar('minArea','contour',1,1000,lambda self: None)
        cv2.setTrackbarPos('minArea','contour',50)

    def user_image(self,image,gray):
        thresh1 = cv2.getTrackbarPos('thresh1','contour')
        ret, thresholded = cv2.threshold(gray, thresh1, 255, 0)
        #cv2.imshow('contour',thresholded)
        if cv2.__version__[0] >= '4':
            contours, hierarchy = \
                cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:  # in OpenCV 3.x there was an additional return value
            dummy, contours, hierarchy = \
                cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        areas = [(i, cv2.contourArea(contours[i])) for i in range(len(contours))]
        areas.sort(key=lambda x: x[1])
        areas.reverse()
        self.areas = areas
        self.contours = contours
        self.hierarchy = hierarchy

    def user_annotate(self,annotated_image):
        minArea = cv2.getTrackbarPos('minArea','contour')
        scale = self.annotated_scale_factor
        for area_entry in self.areas:
            if area_entry[1] < minArea:
                break
            temp = index = area_entry[0]
            depth = -1
            while temp != -1 and depth < len(self.colors)-1:
                depth += 1
                temp = self.hierarchy[0,temp,3]
            contour = scale * self.contours[index]
            cv2.drawContours(annotated_image, [contour], 0, self.colors[depth], 2)
        cv2.imshow('contour',annotated_image)
        cv2.waitKey(1)
        return annotated_image
