import cv2, numpy, math

class ArucoMarker(object):
  def __init__(self,markerID,bbox,translation,rotation):
    self.id = markerID
    self.bbox = bbox

    #pose information
    self.translation = translation
    self.rotation = (180/math.pi)*rotation

  def __str__(self):
    return "<ArucoMarker id=%d trans=(%d,%d,%d) rot=(%d,%d,%d)>" % (self.id,
                  self.translation[0],self.translation[1],self.translation[2],
                           self.rotation[0],self.rotation[1],self.rotation[2])

  def __repr__(self):
    return self.__str__()


class Aruco(object):
    def __init__(self,arucolibname,markerSize = 50):
        self.arucolibname = arucolibname
        self.aruco_lib = cv2.aruco.Dictionary_get(arucolibname)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.seenMarkers = dict()
        self.seenMarkerObjects = []

        #added for pose estimation
        self.markerSize = markerSize #these units will be pose est units!!
        self.axisLength = markerSize*0.5
        self.image_size = (240,320)
        self.focal_len = 290
        self.cameraMatrix = numpy.array([[self.focal_len,0,self.image_size[0]/2],
		                                 [0,self.focal_len,self.image_size[1]/2],
										 [0,0,1]]).astype(float)
        self.distortionArray = numpy.array([[0,0,0,0,0]]).astype(float)

    def process_image(self,gray):
        self.seenMarkers = []
        self.seenMarkerObjects = dict()
        (self.corners,self.ids,_) = \
            cv2.aruco.detectMarkers(gray,self.aruco_lib,parameters=self.aruco_params)
        if self.ids is None: return

        #estimate poses
        (self.rvecs,self.tvecs) = cv2.aruco.estimatePoseSingleMarkers(self.corners, \
                                    self.markerSize,self.cameraMatrix,self.distortionArray)

        for i in range(len(self.ids)):
            marker = ArucoMarker(self.ids[i][0], self.corners[i],self.tvecs[i][0],self.rvecs[i][0])
            self.seenMarkers.append(marker.id)
            self.seenMarkerObjects[marker.id] = marker

    def annotate(self, image, scale_factor):
        scaled_corners = [ numpy.multiply(corner, scale_factor) for corner in self.corners ]
        displayim = cv2.aruco.drawDetectedMarkers(image, scaled_corners, self.ids)

        #add poses #currently fails since image is already scaled. How to scale camMat?
        #if(self.ids is not None):
        #    for i in range(len(self.ids)):
        #      displayim = cv2.aruco.drawAxis(displayim,self.cameraMatrix,
        #                    self.distortionArray,self.rvecs[i],self.tvecs[i]*scale_factor,self.axisLength*scale_factor)
        return displayim
