import cv2, numpy, math

class ArucoMarker(object):
  def __init__(self,marker_id,bbox,translation,rotation):
    self.id = marker_id
    self.bbox = bbox

    # OpenCV Pose information
    self.opencv_translation = translation
    self.opencv_rotation = (180/math.pi)*rotation

    # Cozmo coordinates in camera reference frame
    self.camera_coords = (-translation[0], -translation[1], translation[2])
    self.camera_distance = math.sqrt(translation[0]*translation[0] +
                                     translation[2]*translation[2])

  def __str__(self):
    return "<ArucoMarker id=%d trans=(%d,%d,%d) rot=(%d,%d,%d)>" % \
              (self.id, *self.opencv_translation, *self.opencv_rotation)

  def __repr__(self):
    return self.__str__()


class Aruco(object):
    def __init__(self,arucolibname, marker_size=50):
        self.arucolibname = arucolibname
        self.aruco_lib = cv2.aruco.Dictionary_get(arucolibname)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.seen_marker_ids = []
        self.seen_marker_objects = dict()
        self.ids = []
        self.corners = []

        #added for pose estimation
        self.marker_size = marker_size #these units will be pose est units!!
        self.image_size = (320,240)
        self.focal_len = 290
        self.camera_matrix = \
            numpy.array([[self.focal_len, 0,               self.image_size[0]/2],
                         [0,              -self.focal_len, self.image_size[1]/2],
                         [0,              0,               1]]).astype(float)
        self.distortion_array = numpy.array([[0,0,0,0,0]]).astype(float)

    def process_image(self,gray):
        self.seen_marker_ids = []
        self.seen_marker_objects = dict()
        (self.corners,self.ids,_) = \
            cv2.aruco.detectMarkers(gray,self.aruco_lib,parameters=self.aruco_params)
        if self.ids is None: return

        #estimate poses
        (self.rvecs,self.tvecs) = \
            cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                self.marker_size,
                                                self.camera_matrix,
                                                self.distortion_array)

        for i in range(len(self.ids)):
            marker = ArucoMarker(self.ids[i][0], self.corners[i],self.tvecs[i][0],self.rvecs[i][0])
            self.seen_marker_ids.append(marker.id)
            self.seen_marker_objects[marker.id] = marker

    def annotate(self, image, scale_factor):
        scaled_corners = [ numpy.multiply(corner, scale_factor) for corner in self.corners ]
        displayim = cv2.aruco.drawDetectedMarkers(image, scaled_corners, self.ids)

        #add poses #currently fails since image is already scaled. How to scale camMat?
        #if(self.ids is not None):
        #    for i in range(len(self.ids)):
        #      displayim = cv2.aruco.drawAxis(displayim,self.cameraMatrix,
        #                    self.distortionArray,self.rvecs[i],self.tvecs[i]*scale_factor,self.axisLength*scale_factor)
        return displayim
