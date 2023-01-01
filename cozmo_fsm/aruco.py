try: import cv2
except: pass

import math
from numpy import sqrt, arctan2, array, multiply

ARUCO_MARKER_SIZE = 44

class ArucoMarker(object):
    def __init__(self, aruco_parent, marker_id, bbox, translation, rotation):
        self.id = marker_id
        self.id_string = 'Aruco-' + str(marker_id)
        self.bbox = bbox
        self.aruco_parent = aruco_parent

        # OpenCV Pose information
        self.opencv_translation = translation
        self.opencv_rotation = (180/math.pi)*rotation

        # Marker coordinates in robot's camera reference frame
        self.camera_coords = (-translation[0], -translation[1], translation[2])

        # Distance in the x-y plane; particle filter ignores height so don't include it
        self.camera_distance = math.sqrt(translation[0]*translation[0] +
                                         # translation[1]*translation[1] +
                                         translation[2]*translation[2])
        # Conversion to euler angles
        self.euler_rotation = self.rotationMatrixToEulerAngles(
                                        cv2.Rodrigues(rotation)[0])*(180/math.pi)

    def __str__(self):
        return "<ArucoMarker id=%d trans=(%d,%d,%d) rot=(%d,%d,%d) erot=(%d,%d,%d)>" % \
                (self.id, *self.opencv_translation, *self.opencv_rotation, *self.euler_rotation)

    def __repr__(self):
        return self.__str__()

    @staticmethod
    def rotationMatrixToEulerAngles(R) :
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular:
            x = arctan2(R[2,1] , R[2,2])
            y = arctan2(-R[2,0], sy)
            z = arctan2(R[1,0], R[0,0])
        else:
            x = arctan2(-R[1,2], R[1,1])
            y = arctan2(-R[2,0], sy)
            z = 0

        return array([x, y, z])

class Aruco(object):
    def __init__(self, robot, arucolibname, marker_size=ARUCO_MARKER_SIZE, disabled_ids=[]):
        self.arucolibname = arucolibname
        if arucolibname is not None:
            self.aruco_lib = cv2.aruco.getPredefinedDictionary(arucolibname)
            self.aruco_params = cv2.aruco.DetectorParameters()
        self.seen_marker_ids = []
        self.seen_marker_objects = dict()
        self.disabled_ids = disabled_ids  # disable markers with high false detection rates
        self.ids = []
        self.corners = []

        if robot.camera is None: return  # robot is a SimRobot

        # Added for pose estimation
        self.marker_size = marker_size #these units will be pose est units!!
        self.image_size = (320,240)
        focal_len = robot.camera._config._focal_length
        self.camera_matrix = \
            array([[focal_len.x ,  0,            self.image_size[0]/2],
                         [0,             -focal_len.y, self.image_size[1]/2],
                         [0,             0,            1]]).astype(float)
        self.distortion_array = array([[0,0,0,0,0]]).astype(float)

    def process_image(self,gray):
        self.seen_marker_ids = []
        self.seen_marker_objects = dict()
        (self.corners,self.ids,_) = \
            cv2.aruco.detectMarkers(gray,self.aruco_lib,parameters=self.aruco_params)
        if self.ids is None: return

        # Estimate poses
        # Warning: OpenCV 3.2 estimate returns a pair; 3.3 returns a triplet
        estimate = \
            cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                self.marker_size,
                                                self.camera_matrix,
                                                self.distortion_array)

        self.rvecs = estimate[0]
        self.tvecs = estimate[1]
        for i in range(len(self.ids)):
            id = int(self.ids[i][0])
            if id in self.disabled_ids: continue
            tvec = self.tvecs[i][0]
            rvec = self.rvecs[i][0]
            if rvec[2] > math.pi/2 or rvec[2] < -math.pi/2:
                # can't see a marker facing away from us, so bogus
                print('Marker rejected! id=', id, 'tvec=', tvec, 'rvec=', rvec)
                continue
            marker = ArucoMarker(self, id,
                                 self.corners[i], self.tvecs[i][0], self.rvecs[i][0])
            self.seen_marker_ids.append(marker.id)
            self.seen_marker_objects[marker.id] = marker

    def annotate(self, image, scale_factor):
        scaled_corners = [ multiply(corner, scale_factor) for corner in self.corners ]
        displayim = cv2.aruco.drawDetectedMarkers(image, scaled_corners, self.ids)

        #add poses currently fails since image is already scaled. How to scale camMat?
        #if(self.ids is not None):
        #    for i in range(len(self.ids)):
        #      displayim = cv2.aruco.drawAxis(displayim,self.cameraMatrix,
        #                    self.distortionArray,self.rvecs[i],self.tvecs[i]*scale_factor,self.axisLength*scale_factor)
        return displayim
