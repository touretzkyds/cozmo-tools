import cv2

class ArucoMarker(object):
  def __init__(self,markerID,bbox):
    self.id = markerID
    self.bbox = bbox

  def __str__(self):
    return "marker %d" % (self.id)

  def __repr__(self):
    return self.__str__()

  def annotated(self,im):
    return cv2.aruco.drawDetectedMarkers(im,[self.bbox],[self.id])

class Aruco():
    def __init__(self,arucolibname):
        self.arucolibname = arucolibname
        self.aruco_lib = cv2.aruco.Dictionary_get(arucolibname)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.seenMarkers = []
        self.seenMarkerObjects = []

    def process_image(self,gray):
        self.seenMarkers = []
        self.seenMarkerObjects = []
        (self.corners,self.ids,_) = \
            cv2.aruco.detectMarkers(gray,self.aruco_lib,parameters=self.aruco_params)
        if self.ids is None: return
        for i in range(len(self.ids)):
            marker = ArucoMarker(self.ids[i][0], self.corners[i])
            self.seenMarkers.append(marker.id)
            self.seenMarkerObjects.append(marker)

    def annotate(self,image):
        displayim = cv2.aruco.drawDetectedMarkers(image, self.corners, self.ids)
        return displayim
