import cv2

class ArucoTag(object):
  def __init__(self,tagID,bbox):
    self.id = tagID
    self.bbox = bbox

  def __str__(self):
    return "tag %d" % (self.id)

  def __repr__(self):
    return self.__str__()

  def annotated(self,im):
    return cv2.aruco.drawDetectedMarkers(im,[self.bbox],[self.id])

class Aruco():
    def __init__(self,arucolibname):
        self.arucolibname = arucolibname
        self.aruco_lib = cv2.aruco.Dictionary_get(arucolibname)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.seenTags = []
        self.seenTagObjects = []

    def process_image(self,gray):
        (corners,ids,_) =
            cv2.aruco.detectMarkers(gray,self.aruco_lib,parameters=self.aruco_params)
        self.corners = corners
        self.ids = ids
        if ids is None: return
        for i in range(len(ids)):
            tag = ArucoTag(ids[i][0], corners[i])
            if tag.id not in self.seenTags:
                self.seenTags.append(tag.id)
                self.seenTagObjects.append(tag)

    def annotate(self,image):
        displayim = cv2.aruco.drawDetectedMarkers(image, self.corners, self.ids)
        return displayim
