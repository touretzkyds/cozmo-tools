from numpy import matrix, array, ndarray, sqrt, arctan2, pi
import threading
from time import sleep
from .geometry import wrap_angle

try:
    import cv2
    import cv2.aruco as aruco
except: pass

# Known camera parameters

# Microsoft HD ( Calibrated to death )
microsoft_HD_webcam_cameraMatrix = matrix([[1148.00,       -3,    641.0],
                               [0.000000,   1145.0,    371.0],
                               [0.000000, 0.000000, 1.000000]])
microsoft_HD_webcam_distCoeffs = array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])

class Cam():
    def __init__(self,cap,x,y,z,phi, theta):
        self.cap = cap
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.theta = theta

    def __repr__(self):
        return '<Cam (%.2f, %.2f, %.2f)> @ %.2f' % \
               (self.x, self.y, self.z,self.phi*180/pi)

class PerchedCameraThread(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot
        self.use_perched_cameras = False
        self.perched_cameras = []
        # Set camera parameters. (Current code assumes same parameters for all cameras connected to a computer.)
        self.cameraMatrix = microsoft_HD_webcam_cameraMatrix
        self.distCoeffs = microsoft_HD_webcam_distCoeffs
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        # camera landmarks from local cameras
        self.cameras = {}
        # camera landamrks from network (sent from server)
        self.camera_pool = {}

    def run(self):
        while(True):
            if self.use_perched_cameras:
                self.process_image()
                # Computer overloaded if not given break
                sleep(0.01)
            else:
                break

    def start_perched_camera_thread(self,cameras=[]):
        if not isinstance(cameras,list):
            cameras = [cameras]

        if self.robot.aruco_id == -1:
            self.robot.aruco_id = int(input("Please enter the aruco id of the robot:"))
            self.robot.world.server.camera_landmark_pool[self.robot.aruco_id]={}
        self.use_perched_cameras=True
        self.perched_cameras = []
        for x in cameras:
            cap = cv2.VideoCapture(x)
            if cap.isOpened():
                self.perched_cameras.append(cap)
            else:
               raise RuntimeError("Could not open camera %s." % repr(x))
        for cap in self.perched_cameras:
            # hack to set highest resolution
            cap.set(3,4000)
            cap.set(4,4000)
        self.robot.world.particle_filter.sensor_model.use_perched_cameras = True
        print("Particle filter now using perched cameras")
        self.start()

    def stop_perched_camera_thread(self):
        self.use_perched_cameras=False
        sleep(1)
        for cap in self.perched_cameras:
            cap.release()
        self.robot.world.particle_filter.sensor_model.use_perched_cameras = False
        print("Particle filter stopped using perched cameras")

    def check_camera(self,camera):
        cap = cv2.VideoCapture(camera)
        for j in range(10):
            # hack to clear buffer
            for i in range(5):
                cap.grab()
            ret, frame = cap.read()
            if not ret:
                print('Failed to get camera frame from camera %s.' % camera )
                return
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            gray = cv2.aruco.drawDetectedMarkers(gray, corners, ids)
            cv2.imshow("Camera:"+str(camera),gray)
            cv2.waitKey(1)
        cap.release()
        cv2.destroyAllWindows()

    def rotationMatrixToEulerAngles(self, R) :
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = arctan2(R[2,1] , R[2,2])
            y = arctan2(-R[2,0], sy)
            z = arctan2(R[1,0], R[0,0])
        else :
            x = arctan2(-R[1,2], R[1,1])
            y = arctan2(-R[2,0], sy)
            z = 0
     
        return array([x, y, z])

    def process_image(self):
        # Dict with key: aruco id with values as cameras that can see the marker
        self.temp_cams = {}     # Necessary, else self.cameras is empty most of the time
        for cap in self.perched_cameras:
            # Clearing Buffer by grabbing five frames
            for i in range(5):
                cap.grab()
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if type(ids) is ndarray:
                vecs = aruco.estimatePoseSingleMarkers(corners, 50, self.cameraMatrix, self.distCoeffs)
                rvecs, tvecs = vecs[0], vecs[1]
                for i in range(len(ids)):
                    rotationm, jcob = cv2.Rodrigues(rvecs[i])
                    # transform to robot coordinate frame
                    transformed = matrix(rotationm).T*(-matrix(tvecs[i]).T)
                    phi = self.rotationMatrixToEulerAngles(rotationm.T)
                    if ids[i][0] in self.temp_cams:
                        self.temp_cams[ids[i][0]][str(cap)]=Cam(str(cap),transformed[0][0,0],
                            transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))
                    else:
                        self.temp_cams[ids[i][0]]={str(cap):Cam(str(cap),transformed[0][0,0],
                            transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))}
        self.cameras = self.temp_cams

        # Only server clears the pool
        if self.robot.world.is_server:
            self.camera_pool = self.temp_cams
