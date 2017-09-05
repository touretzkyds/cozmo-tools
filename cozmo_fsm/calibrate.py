import time
import inspect
import random
import cv2
import cv2.aruco as aruco
from scipy.optimize import fsolve
from math import sqrt, sin, asin
import numpy as np 
import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees, Distance, Angle

from .base import *
from .events import *
from .nodes import *
from .cozmo_kin import wheelbase
from .worldmap import *


#________________ Action Nodes ________________

"""
class LocateCam1(ActionNode):
    def __init__(self, camera_number=1, abort_on_stop=True, **action_kwargs):
        if 'should_play_anim' not in action_kwargs:
            action_kwargs['should_play_anim'] = False
        self.action_kwargs = action_kwargs
        self.focus = 1140     # Set according to camera
        self.camera_number = camera_number # Set according to camera
        self.camera_width = 1280
        self.camera_height = 720
        self.cap = cv2.VideoCapture(self.camera_number) # Camera_capture Object
        self.cap.set(3,self.camera_width)
        self.cap.set(4,self.camera_height)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        # super's init must come last because it checks self.action_kwargs
        super().__init__(abort_on_stop)

    def getframe(self):
        for i in range(5):
            self.cap.grab()
        ret, self.frame = self.cap.read()
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        return self.gray

    def getcorners(self):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def orient(self, corners):
        coordinates = (np.mean(corners[0][0][:,0]),0)
        center = (int(np.mean(corners[0][0,:,0])),int(np.mean(corners[0][0,:,1])))
        topa = (int(2*np.mean(corners[0][0,:2,0])-center[0]),int(2*np.mean(corners[0][0,:2,1])-center[1]))
        vecc = (topa[0]-center[0],topa[1]-center[1])
        vecc = (vecc[0]/np.linalg.norm(vecc),vecc[1]/np.linalg.norm(vecc))

        rvecc = (coordinates[0] - center[0], coordinates[1] -center[1])
        rvecc = (rvecc[0]/np.linalg.norm(rvecc),rvecc[1]/np.linalg.norm(rvecc))
        direc = (vecc[1]*rvecc[0] - vecc[0]*rvecc[1])
        self.ang = direc*180*np.arccos(np.inner(vecc,rvecc))/(3.14*abs(direc))

        if abs(self.ang) > 5:
            self.robot.turn_in_place(degrees(self.ang)).wait_for_completed()

            time.sleep(3)
            return True
        return False

    def distance_to_aruco(self, corners):
        return 5*np.sqrt(self.focus**2 + (np.mean(corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[0][0][:,1])-self.camera_height/2)**2 )/np.linalg.norm(corners[0][:][0][0] - corners[0][:][0][1])


    def f(self, theta):
        return np.tan(theta+self.omega2) - np.tan(theta + self.omega1) - 10/(self.distance1*np.cos(theta+self.omega1))


    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Distance):
            self.distance = event.data
        super().start(event)

    def action_launcher(self):
        while(True):
            self.getframe()
            #cv2.imshow('Gray',self.gray)
            self.corners1, self.ids1 = self.getcorners()
            if type(self.ids1) is np.ndarray:
                if(self.orient(self.corners1)):
                    continue
                self.distance1 = self.distance_to_aruco(self.corners1)
                self.omega1 = np.arctan(( self.camera_height/2 - np.mean(self.corners1[0][0][:,1]) )/self.focus)
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.robot.drive_straight(distance_mm(100), speed_mmps(100)).wait_for_completed()

        time.sleep(5)

        while(True):
            self.getframe()
            #cv2.imshow('Gray',self.gray)
            self.corners2, self.ids2 = self.getcorners()
            
            if type(self.ids2) is np.ndarray:
                if(self.orient(self.corners2)):
                    continue
                self.distance2 = self.distance_to_aruco(self.corners2)
                self.omega2 = np.arctan(( self.camera_height/2 - np.mean(self.corners2[0][0][:,1]) )/self.focus)
                break

        self.cap.release()

        self.theta = fsolve(self.f, np.pi/4)
        self.height = np.cos(self.theta + self.omega1)*self.distance1


        self.robot.world.world_map.objects['Cam1'] = CameraObj(1, self.robot.pose.position._x - 10*self.height*np.tan(self.theta + self.omega2), self.robot.pose.position._y, self.robot.pose.position._z + 10*self.height, self.theta )

        return self.robot.drive_straight(distance_mm(0), speed_mmps(100),**self.action_kwargs)





class LocateCam2(ActionNode):
    def __init__(self, camera_number=1, abort_on_stop=True, **action_kwargs):
        if 'should_play_anim' not in action_kwargs:
            action_kwargs['should_play_anim'] = False
        self.action_kwargs = action_kwargs
        self.focus = 1140     # Set according to camera
        self.camera_number = camera_number # Set according to camera
        self.camera_width = 1280
        self.camera_height = 720
        self.cap = cv2.VideoCapture(self.camera_number) # Camera_capture Object
        self.cap.set(3,self.camera_width)
        self.cap.set(4,self.camera_height)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()
        # super's init must come last because it checks self.action_kwargs
        super().__init__(abort_on_stop)

    def getframe(self):
        for i in range(5):
            self.cap.grab()
        ret, self.frame = self.cap.read()
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        return self.gray

    def getcorners(self):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def orient(self, corners):
        coordinates = (np.mean(corners[0][0][:,0]),0)
        center = (int(np.mean(corners[0][0,:,0])),int(np.mean(corners[0][0,:,1])))
        topa = (int(2*np.mean(corners[0][0,:2,0])-center[0]),int(2*np.mean(corners[0][0,:2,1])-center[1]))
        vecc = (topa[0]-center[0],topa[1]-center[1])
        vecc = (vecc[0]/np.linalg.norm(vecc),vecc[1]/np.linalg.norm(vecc))

        rvecc = (coordinates[0] - center[0], coordinates[1] -center[1])
        rvecc = (rvecc[0]/np.linalg.norm(rvecc),rvecc[1]/np.linalg.norm(rvecc))
        direc = (vecc[1]*rvecc[0] - vecc[0]*rvecc[1])
        self.ang = direc*180*np.arccos(np.inner(vecc,rvecc))/(3.14*abs(direc))

        if abs(self.ang) > 5:
            self.robot.turn_in_place(degrees(self.ang)).wait_for_completed()

            time.sleep(3)
            return True
        return False

    def distance_to_aruco(self, corners):
        return 5*np.sqrt(self.focus**2 + (np.mean(corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[0][0][:,1])-self.camera_height/2)**2 )/np.linalg.norm(corners[0][:][0][0] - corners[0][:][0][1])


    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Distance):
            self.distance = event.data
        super().start(event)

    def action_launcher(self):
        while(True):
            self.getframe()
            #cv2.imshow('Gray',self.gray)
            self.corners, self.ids1 = self.getcorners()
            if type(self.ids1) is np.ndarray:
                (x0, y0) = self.corners[0][0][0]
                (x1, y1) = self.corners[0][0][1]
                (x2, y2) = self.corners[0][0][2]
                (x3, y3) = self.corners[0][0][3]


                omega_x = np.arctan((360 - np.mean(self.corners[0][0][:,1]))/self.focus)
                omega_y = np.arctan((640 - np.mean(self.corners[0][0][:,0]))/self.focus)

                A = (x0-x3)/np.cos(omega_y)
                C = (x1-x0)/np.cos(omega_y)
                B = abs(y0-y3)
                
                C2 = abs(x0-x3)/np.cos(omega_y)
                B2 = abs(y0-y1)
                
                C3 = abs(x0-x2)/np.cos(omega_y)
                B3 = abs(y1-y3)
                
                C4 = abs(x1-x3)/np.cos(omega_y)
                B4 = abs(y0-y2)

                phi = np.arctan2(A,C)
                
                if -30 < phi*180/np.pi < 30:
                    theta = np.arccos(B/abs(C)) - omega_x
                elif 30 <= phi*180/np.pi < 60:
                    theta = np.arccos(B3/C3) - omega_x
                elif -60 <= phi*180/np.pi <= -30:
                    theta = np.arccos(B4/C4) - omega_x
                else:
                    theta = np.arccos(B2/C2) - omega_x
                                      

                l = np.sqrt(A**2 + C**2)                
                r = np.sqrt(self.focus*self.focus + (np.mean(self.corners[0][0][:,0])-640)**2 + (np.mean(self.corners[0][0][:,1])-360)**2 )
                R1 = 50*r/l
                
                Y = (np.mean(self.corners[0][0][:,0])-640)*R1/r
                X = (360 - np.mean(self.corners[0][0][:,1]))*R1/(r*np.cos( theta + omega_x ))
                
                height = np.sqrt( R1*R1 - X*X )*np.cos(theta+omega_x)             


                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()

        self.theta = theta
        self.height = height + 100

        self.phi = self.robot.pose.rotation.angle_z.radians + phi
        

        camera_x = self.robot.pose.position._x - (self.height*np.tan(self.theta + omega_x)*np.cos(self.phi) + Y*np.sin(self.phi))
        camera_y = self.robot.pose.position._y - ( -self.height*np.tan(self.theta + omega_x)*np.sin(self.phi) + Y*np.cos(self.phi))


        print(self.phi*180/np.pi, "Camera at: ",camera_x, camera_y)
        
        self.robot.world.world_map.objects['Cam1'] = CameraObj(1, camera_x, -camera_y, self.height, self.theta, self.phi )
        self.robot.world.world_map.objects['Ghost1'] = RobotGhostObj(1, self.robot.pose.position._x, self.robot.pose.position._y, self.robot.pose.position._z, self.robot.pose.rotation.angle_z.radians)
        
        return self.robot.drive_straight(distance_mm(0), speed_mmps(100),**self.action_kwargs)
        """