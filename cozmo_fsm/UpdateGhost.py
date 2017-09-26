import cv2
import socket
import pickle
import threading
import random
from cozmo_fsm import *

class LocateCam(StateNode):
    """ Locates Camera1."""
    def __init__(self, camera_number=1, focus=1140):
        self.camera_number = camera_number
        self.owner = socket.gethostname()
        self.focus = focus
        super().__init__()

    def getframe(self):
        for i in range(5):
            self.cap.grab()
        ret, self.frame = self.cap.read()
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        return self.gray

    def getcorners(self):
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def distance_to_aruco(self, corners):
        return 5*np.sqrt(self.focus**2 + (np.mean(corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[0][0][:,1])-self.camera_height/2)**2 )/np.linalg.norm(corners[0][:][0][0] - corners[0][:][0][1])

    def start(self, event=None):
        super().start(event)

        if self.camera_number not in self.parent.caplist:
            self.parent.caplist[self.camera_number] = cv2.VideoCapture(self.camera_number)
            self.parent.caplist[self.camera_number].set(3,4000)
            self.parent.caplist[self.camera_number].set(4,4000)

        self.cap = self.parent.caplist[self.camera_number]
        self.camera_width = self.cap.get(3)
        self.camera_height = self.cap.get(4)

        print("Camera"+str(self.camera_number)+" resolution", self.camera_width,'x', self.camera_height)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        tries = 0

        for i in range(100):
            ret, self.frame = self.cap.read()
            cv2.imshow('Make sure Cozmo is visible',self.frame)
            cv2.waitKey(1)

        cv2.destroyAllWindows()

        while(True):
            tries += 1
            print('Try',tries)

            if tries >5:
                print("Cannot find cozmo from Camera"+str(self.camera_number%10))
                self.post_completion()
                return

            self.getframe()
            self.corners, self.ids1 = self.getcorners()
            print(self.ids1)


            if type(self.ids1) is np.ndarray:
                (x0, y0) = self.corners[0][0][0]
                (x1, y1) = self.corners[0][0][1]
                (x2, y2) = self.corners[0][0][2]
                (x3, y3) = self.corners[0][0][3]

                omega_x = np.arctan((self.camera_height/2 - np.mean(self.corners[0][0][:,1]))/self.focus)
                omega_y = np.arctan((self.camera_width/2 - np.mean(self.corners[0][0][:,0]))/self.focus)

                A = (x0-x3)#/np.cos(omega_y)
                C = (x1-x0)#/np.cos(omega_y)
                phi = np.arctan2(-A,-C)

                comparr =  phi*180/np.pi

                if -30 < comparr < 30 or 150 < comparr < 180 or -180 < comparr < -150:
                    theta = np.mean( (np.arccos((y3-y0)/(x1-x0)), np.arccos((y2-y1)/(x2-x3))) ) - omega_x
                elif 30 <= comparr < 60 or -150 < comparr <= -120:
                    theta = np.arccos((y2-y0)/(x1-x3)) - omega_x
                elif -60 <= comparr <= -30 or 120 <= comparr <= 150:
                    theta = np.arccos((y3-y1)/(x2-x0)) - omega_x
                else:
                    theta = np.mean( (np.arccos((y2-y3)/(x0-x3)), np.arccos((y1-y0)/(x1-x2))) ) - omega_x

                l = np.sqrt(A**2 + C**2)
                r = np.sqrt(self.focus*self.focus + (np.mean(self.corners[0][0][:,0])-self.camera_width/2)**2 + (np.mean(self.corners[0][0][:,1])-self.camera_height/2)**2 )
                R1 = 50*r/l

                Y = (np.mean(self.corners[0][0][:,0])-self.camera_width/2)*R1/r
                X = (self.camera_height/2 - np.mean(self.corners[0][0][:,1]))*R1/(r*np.cos( theta ))
                height = np.sqrt( R1*R1 - X*X )*np.cos(theta+omega_x)
                print('Calibrated on',self.ids1[0][0])
                break

        self.theta = theta
        self.height = height + 100
        self.phi = self.robot.pose.rotation.angle_z.radians + phi

        camera_x = self.robot.pose.position._x - (self.height*np.tan(self.theta + omega_x)*np.cos(self.phi) + Y*np.sin(self.phi))
        camera_y = self.robot.pose.position._y - ( -self.height*np.tan(self.theta + omega_x)*np.sin(self.phi) + Y*np.cos(self.phi))
        print("Theta:",self.theta*180/np.pi,"Phi",self.phi*180/np.pi, "Camera" +str(self.camera_number)+ " at: ",camera_x, camera_y)
        self.robot.world.world_map.objects['Cam-'+self.owner+'-'+str(self.camera_number)] = CameraObj(random.randint(0,255), camera_x, -camera_y, self.height, self.theta, self.phi, (X,Y), self.ids1[0][0] )
        self.parent.supreme_cozmo_id = self.ids1[0][0]
        print("World Coordinate frame is the current frame of Cozmo",self.parent.supreme_cozmo_id)
        self.post_completion()


class Findcubes(StateNode):
    def __init__(self, camera_number=1, focus=1140):
        self.camera_number = camera_number # Set according to camera
        self.focus = focus
        super().__init__()

    def fra(self):
        for i in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        return frame[:,:,2].astype(np.int16, copy=False)

    def twink(self, par, cube_location):
        coun = 0
        if par==0:
            A = cozmo.lights.red_light
            B = cozmo.lights.off_light
            C = cozmo.lights.off_light
            D = cozmo.lights.off_light
        elif par==1:
            B = cozmo.lights.red_light
            A = cozmo.lights.off_light
            C = cozmo.lights.off_light
            D = cozmo.lights.off_light
        elif par==2:
            C = cozmo.lights.red_light
            B = cozmo.lights.off_light
            A = cozmo.lights.off_light
            D = cozmo.lights.off_light
        elif par==3:
            D = cozmo.lights.red_light
            B = cozmo.lights.off_light
            C = cozmo.lights.off_light
            A = cozmo.lights.off_light
        else:
            print("Error")

        for i in range(20):
            red1 = self.fra()
            cube1.set_light_corners(A, B, C, D)
            red2 = self.fra()
            cube1.set_lights(cozmo.lights.off_light)

            a = abs(red1-red2)
            possib = (np.argmax(a)%1280,int(np.argmax(a)/1280))

            if abs(np.sum( np.subtract( possib , cube_location[par]) )) < 10:
                coun+=1
            else:
                cube_location[par] = possib
                coun = 0

            if coun==3:
                return cube_location

        print("Not found")


    def start(self, event=None):
        super().start(event)
        print("Finding Cubes")
        self.cap = cv2.VideoCapture(self.camera_number) # Camera_capture Object
        self.cap.set(3,4000)
        self.cap.set(4,4000)
        self.camera_width = self.cap.get(3)
        self.camera_height = self.cap.get(4)


        cube_location =[(0,0), (0,0), (0,0), (0,0)]

        cube_location =  self.twink(0, cube_location)
        cube_location =  self.twink(1, cube_location)
        cube_location =  self.twink(2, cube_location)
        cube_location =  self.twink(3, cube_location)

        phi = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].phi
        angle = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].theta
        initial_position = self.robot.world.world_map.objects['Cam'+str(self.camera_number)].initial_position

        self.robot.world.world_map.objects['CubeGhost'+str(self.camera_number)] = LightCubeGhostObj(self.camera_number, cube_location[0][0], cube_location[0][1], 0, 0, True)
        self.cap.release()
        print("Done")
        self.post_completion()


class ProcessImage(StateNode):
    def __init__(self, camera_number=1, focus=1140):
        self.camera_number = camera_number
        self.owner = socket.gethostname()
        self.focus = focus
        self.cap = None
        super().__init__()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters =  cv2.aruco.DetectorParameters_create()
        self.focus = 1140
        self.flag=0
        self.seen=dict(dict())

    def uncertainity(self, corners):
        return(abs(np.mean(corners[0][0][:,1])/self.camera_height -0.5) + abs(np.mean(corners[0][0][:,0])/self.camera_width -0.5))

    def start(self, event=None):

        if self.camera_number not in self.parent.caplist:
            print("HEER")
            self.parent.caplist[self.camera_number] = cv2.VideoCapture(self.camera_number)
            self.parent.caplist[self.camera_number].set(3,4000)

        self.cap = self.parent.caplist[self.camera_number]
        self.camera_width = self.cap.get(3)
        self.camera_height = self.cap.get(4)

        super().start(event)

        #Update Ghost
        for i in range(5):
            self.cap.grab()
        ret, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        for key in self.seen:
            self.seen[key]=False

        if type(ids) is np.ndarray:
            for id in range(len(ids)):
                (x0, y0) = corners[id][0][0]
                (x1, y1) = corners[id][0][1]
                (x2, y2) = corners[id][0][2]
                (x3, y3) = corners[id][0][3]

                omega_x = np.arctan((self.camera_height/2 - np.mean(corners[id][0][:,1]))/self.focus)

                A = (x0-x3)#/np.cos(omega_x)
                C = (x1-x0)#/np.cos(omega_x)

                gphi = np.arctan2(-A,-C)

                try:
                    Cam = self.robot.world.world_map.objects['Cam-'+self.owner+'-'+str(self.camera_number)]
                    phi = Cam.phi
                    theta = Cam.theta
                    initial_position = Cam.initial_position

                    l = np.sqrt(A**2 + C**2)
                    r = np.sqrt(self.focus*self.focus + (np.mean(corners[id][0][:,0])-self.camera_width/2)**2 + (np.mean(corners[id][0][:,1])-self.camera_height/2)**2 )
                    R1 = 50*r/l

                    X = (self.camera_height/2 - np.mean(corners[id][0][:,1]))*R1/(r*np.cos(theta)) - initial_position[0]
                    Y = -( ((np.mean(corners[id][0][:,0])-self.camera_width/2)*R1/r) - initial_position[1] )

                    gname = 'Ghost'+str(self.camera_number)

                    if (gname,str(ids[id][0])) in self.robot.world.world_map.temp_ghosts:
                        self.robot.world.world_map.temp_ghosts[gname,str(ids[id][0])].update(X*cos(phi) - Y*sin(phi), X*sin(phi) + Y*cos(phi), 0, -gphi + phi, self.uncertainity(corners))
                        self.seen[gname,str(ids[id][0])]=True
                    else:
                        print(gname+'-'+str(ids[id][0]))
                        self.robot.world.world_map.temp_ghosts[gname,str(ids[id][0])] = RobotGhostObj(Cam.id, ids[id][0], X*cos(phi) - Y*sin(phi), X*sin(phi) + Y*cos(phi), 0, -gphi + phi, True, self.uncertainity(corners), Cam.cozmo_number)
                        self.seen[gname,str(ids[id][0])]=True
                except:
                    print('Camera not found')

        for key in self.seen:
            self.robot.world.world_map.temp_ghosts[key].is_visible = self.seen[key]
        #else:
        #    self.robot.world.world_map.objects['Ghost'+str(self.camera_number)+'-'+str(id)].is_visible = False
        self.post_completion()

class FindTransforms(StateNode):
    def start(self, event=None):
        super().start(event)
        self.supreme_cozmo_id = self.parent.supreme_cozmo_id
        for key, value in self.robot.world.world_map.temp_ghosts.items():
            if value.cozmo_number != self.supreme_cozmo_id and value.cozmo_number == value.cozmo_id:
                for k, v in self.robot.world.world_map.temp_ghosts.items():
                    if v.cozmo_number == self.supreme_cozmo_id and v.cozmo_id == value.cozmo_id:
                        self.supreme = v
                        break

                theta = value.theta - self.supreme.theta

                X = self.supreme.x -( cos(theta)*value.x + sin(theta)*value.y )
                Y = self.supreme.y -( cos(theta)*value.y - sin(theta)*value.x )
                self.parent.tranforms[(value.cozmo_number,self.supreme_cozmo_id)] = (theta,X,Y)

                print("Added Transform from",value.cozmo_number,"to",self.supreme_cozmo_id)

        self.post_completion()

class Set_current_cozmo(StateNode):
    def transform_ghost(self, ghost):
        theta, X, Y = self.parent.tranforms[ghost.cozmo_number,self.supreme_cozmo_id]
        x = X + cos(theta)*ghost.x + sin(theta)*ghost.y
        y = Y + cos(theta)*ghost.y - sin(theta)*ghost.x
        ghost.x=x
        ghost.y=y
        ghost.theta-=theta
        ghost.cozmo_number = self.supreme_cozmo_id

    def start(self, event=None):
        super().start(event)
        self.supreme_cozmo_id = self.parent.supreme_cozmo_id
        minn = {}
        for key, value in self.robot.world.world_map.temp_ghosts.items():
            if value.is_visible and int(key[1]) in self.parent.cozmo_map:
                if self.parent.cozmo_map[int(key[1])] == socket.gethostname():
                    if key[1] in minn:
                        if minn[key[1]] > value.uncertainity:
                            minn[key[1]] = value.uncertainity
                            if value.cozmo_number != self.supreme_cozmo_id:
                                self.transform_ghost(value)
                            self.robot.world.particle_filter.set_pose(value.x, value.y, value.theta)
                    else:
                        minn[key[1]] = value.uncertainity
                        if value.cozmo_number != self.supreme_cozmo_id:
                            self.transform_ghost(value)
                        self.robot.world.particle_filter.set_pose(value.x, value.y, value.theta)
        self.post_completion()

class Correct_cam(StateNode):
    def trans(self, cam):
        theta, X, Y = self.parent.tranforms[cam.cozmo_number,self.supreme_cozmo_id]
        x = X + cos(theta)*cam.x + sin(theta)*cam.y
        y = Y + cos(theta)*cam.y - sin(theta)*cam.x
        cam.x=x
        cam.y=y
        cam.phi+=theta
        cam.cozmo_number = self.supreme_cozmo_id

    def start(self, event=None):
        super().start(event)
        self.supreme_cozmo_id = self.parent.supreme_cozmo_id
        for key, value in self.robot.world.world_map.temp_cams.items():
            if 'Cam' in key and value.cozmo_number != self.supreme_cozmo_id:
                self.trans(value)

            self.robot.world.world_map.objects[key] = value

        self.post_completion()

class Fusion(StateNode):
    def transform_ghost(self, ghost):
        theta, X, Y = self.parent.tranforms[ghost.cozmo_number,self.supreme_cozmo_id]
        x = X + cos(theta)*ghost.x + sin(theta)*ghost.y
        y = Y + cos(theta)*ghost.y - sin(theta)*ghost.x
        ghost.x=x
        ghost.y=y
        ghost.theta-=theta
        ghost.cozmo_number = self.supreme_cozmo_id

    def start(self, event=None):
        super().start(event)
        self.supreme_cozmo_id = self.parent.supreme_cozmo_id
        minn = {}
        for key, value in self.robot.world.world_map.temp_ghosts.items():
            if value.is_visible and int(key[1]) in self.parent.cozmo_map:
                if self.parent.cozmo_map[int(key[1])] != socket.gethostname():
                    if key[1] in minn:
                        if minn[key[1]] > value.uncertainity:
                            minn[key[1]] = value.uncertainity
                            if value.cozmo_number != self.supreme_cozmo_id:
                                self.transform_ghost(value)
                            self.robot.world.world_map.objects['Ghost'+key[1]]=value
                    else:
                        minn[key[1]] = value.uncertainity
                        if value.cozmo_number != self.supreme_cozmo_id:
                            self.transform_ghost(value)
                        self.robot.world.world_map.objects['Ghost'+key[1]]=value

        self.post_completion()

class Client(object):
    def __init__(self, robot):
        self.port = None
        self.socket = None #not running until startClient is called
        self.ipaddr = None
        self.robot= robot

    def startClient(self,ipaddr="",port=1800):
        self.port = port
        self.ipaddr = ipaddr
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True)
        print("Attempting to connect to %s at port %d" % (ipaddr,port))
        self.socket.connect((ipaddr,port))
        print("Connected.")
        return self #lets user to call client = Client().startClient()

    def sendMessage(self,msg):
        self.socket.recv(1024)
        if type(msg) == str:
            self.socket.sendall((msg).encode()) #send as byte string
        else:
            self.socket.sendall(pickle.dumps(msg))


class Server(object):
    def __init__(self, robot):
        self.port = None
        self.socket = None #not running until startServer is called
        self.robot= robot

    def startServer(self,port=42):
        self.port = port
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setblocking(True) #lets select work properly I think
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True) #enables easy server restart
        self.socket.bind(("",port)) #binds to any address the computer can be accessed by
        self.socket.listen() #start awaiting connections
        print("running server on %d" % (self.port))
        self.c, addr = self.socket.accept()
        print('Got connection from', addr)
        return self #enables user to call server = Server().startServer()

    def getMessage(self):
        self.c.send(b'Thank you for connecting')
        ghosts = pickle.loads(self.c.recv(4096))

        for key, value in ghosts.items():
            if 'Cam' in key:
                self.robot.world.world_map.temp_cams[key]=value
            else:
                self.robot.world.world_map.temp_ghosts[key]=value


class Send(StateNode):
    def start(self,event=None):
        super().start(event)
        ghosts = dict(dict())
        for key, value in self.robot.world.world_map.objects.items():
            if type(key)==str:
                ghosts[key] = value

        for key, value in self.robot.world.world_map.temp_ghosts.items():
            ghosts[key] = value

        self.parent.client.sendMessage(ghosts)
        self.post_completion()


class Recieve(StateNode):
    def start(self,event=None):
        super().start(event)
        self.parent.server.getMessage()
        self.post_completion()


class UpdateGhost(StateMachineProgram):
    def start(self):
        super().__init__(cam_viewer=False)
        print("Enter your server's ip address:",end='')
        ipaddr = '' #input().strip() # get user input to be server ip
        if ipaddr not in ["None",""]:
            print("ipaddr is "+ipaddr)
            self.client = Client(self.robot).startClient(ipaddr=ipaddr,port=1800)
        else:
            print("Launching Server...")
            self.server = Server(self.robot).startServer(port=1800)
        self.cozmo_map = { 1:"a", 2:"tekkotsu2" }
        self.tranforms = {}
        self.caplist = {}
        self.supreme_cozmo_id = 1
        super().start()

    def setup(self):
        """
            #launch:  Recieve() =C=> process
            launch:  LocateCam(1,1140) =C=> setup
            setup :  ProcessImage(1,1140) =C=> Recieve() =C=> FindTransforms() =C=> Set_current_cozmo() =C=> process
            #process: Recieve() =C=> process
            process: ProcessImage(1,1140) =C=> Recieve() =C=> Correct_cam()=C=> Fusion() =C=> process
        """
        
        # Code generated by genfsm on Mon Sep 25 14:20:36 2017:
        
        launch = LocateCam(1,1140) .set_name("launch") .set_parent(self)
        setup = ProcessImage(1,1140) .set_name("setup") .set_parent(self)
        recieve1 = Recieve() .set_name("recieve1") .set_parent(self)
        findtransforms1 = FindTransforms() .set_name("findtransforms1") .set_parent(self)
        set_current_cozmo1 = Set_current_cozmo() .set_name("set_current_cozmo1") .set_parent(self)
        process = ProcessImage(1,1140) .set_name("process") .set_parent(self)
        recieve2 = Recieve() .set_name("recieve2") .set_parent(self)
        correct_cam1 = Correct_cam() .set_name("correct_cam1") .set_parent(self)
        fusion1 = Fusion() .set_name("fusion1") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(launch) .add_destinations(setup)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(setup) .add_destinations(recieve1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(recieve1) .add_destinations(findtransforms1)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(findtransforms1) .add_destinations(set_current_cozmo1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(set_current_cozmo1) .add_destinations(process)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(process) .add_destinations(recieve2)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(recieve2) .add_destinations(correct_cam1)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(correct_cam1) .add_destinations(fusion1)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(fusion1) .add_destinations(process)
        
        return self

