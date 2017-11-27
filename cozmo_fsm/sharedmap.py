import cv2
import socket
import pickle
import threading
from time import sleep
from numpy import inf, arctan2, pi, cos, sin
from .worldmap import RobotForeignObj, LightCubeForeignObj, WallObj
from .transform import wrap_angle
from cozmo.objects import LightCube
from copy import deepcopy

class ServerThread(threading.Thread):
    def __init__(self, robot, port=1800):
        threading.Thread.__init__(self)
        self.port = port
        self.socket = None #not running until startServer is called
        self.robot= robot
        self.camera_landmark_pool = {} # used to find transforms
        self.poses = {}
        self.started = False
        self.foreign_objects = {} # foreign walls and cubes

    def run(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setblocking(True) 
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True) #enables server restart
        self.socket.bind(("",self.port)) 
        self.threads =[]
        print("Server started")
        self.started = True
        self.fusion.start()
        self.robot.world.is_server = True

        for i in range(100): # Limit of 100 clients
            self.socket.listen(5)   # Now wait for client connection.
            c, addr = self.socket.accept()    # Establish connection with client.
            print('Got connection from', addr)
            self.threads.append(ClientHandlerThread(i, c, self.robot))
            self.threads[i].start()

    def start_server_thread(self):
        if self.robot.aruco_id == -1:
            self.robot.aruco_id = int(input("Please enter the aruco id of the robot:"))
        self.robot.world.server.camera_landmark_pool[self.robot.aruco_id]={}
        # try to get transforms from camera_landmark_pool
        self.fusion = FusionThread(self.robot)
        self.start()

class ClientHandlerThread(threading.Thread):
    def __init__(self, threadID, client, robot):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.c = client
        self.robot = robot
        self.c.sendall(pickle.dumps("Hello"))
        self.aruco_id = int(pickle.loads(self.c.recv(1024)))
        self.name = "Client-"+str(self.aruco_id)
        self.robot.world.server.camera_landmark_pool[self.aruco_id]={}
        self.to_send={}
        print("Started thread for",self.name)

    def run(self):
        # Send from server to clients
        while(True):
            for key, value in self.robot.world.world_map.objects.items():
                if isinstance(key,LightCube):
                    self.to_send["LightCubeForeignObj-"+str(value.id)]= LightCubeForeignObj(id=value.id, x=value.x, y=value.y, z=value.z, theta=value.theta)
                elif isinstance(key,str):
                    # Send walls and cameras
                    self.to_send[key] = value         # Fix case when object removed from shared map
                else:
                    pass                              # Nothing else in sent
            # append 'end' to end to mark end
            self.c.sendall(pickle.dumps([self.robot.world.perched.camera_pool,self.to_send])+b'end')
            # hack to recieve variable size data without crashing
            data = b''
            while True:
                data += self.c.recv(1024)
                if data[-3:]==b'end':
                    break

            cams, landmarks, foreign_objects, pose = pickle.loads(data[:-3])
            for key, value in cams.items():
                if key in self.robot.world.perched.camera_pool:
                    self.robot.world.perched.camera_pool[key].update(value)
                else:
                    self.robot.world.perched.camera_pool[key]=value
            self.robot.world.server.camera_landmark_pool[self.aruco_id].update(landmarks)
            self.robot.world.server.poses[self.aruco_id] = pose
            self.robot.world.server.foreign_objects[self.aruco_id] = foreign_objects

class FusionThread(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot
        self.aruco_id = self.robot.aruco_id
        self.accurate = {}
        self.transforms = {}

    def run(self):
        while(True):
            # adding local camera landmarks into camera_landmark_pool
            self.robot.world.server.camera_landmark_pool[self.aruco_id].update( \
                {k:self.robot.world.particle_filter.sensor_model.landmarks[k] for k in \
                [x for x in self.robot.world.particle_filter.sensor_model.landmarks.keys()\
                if isinstance(x,str) and "Video" in x]})
            flag = False
            # Choose accurate camera
            for key1, value1 in self.robot.world.server.camera_landmark_pool.items():
                for key2, value2 in self.robot.world.server.camera_landmark_pool.items():
                    if key1 == key2:
                        continue
                    for cap, lan in value1.items():
                        if cap in value2:
                            varsum = lan[2].sum()+value2[cap][2].sum()
                            if varsum < self.accurate.get((key1,key2),(inf,None))[0]:
                                self.accurate[(key1,key2)] = (varsum,cap)
                                flag = True
            # Find transform
            if flag:
                for key, value in self.accurate.items():
                    x1,y1 = self.robot.world.server.camera_landmark_pool[key[0]][value[1]][0]
                    h1,p1,t1 = self.robot.world.server.camera_landmark_pool[key[0]][value[1]][1]
                    x2,y2 = self.robot.world.server.camera_landmark_pool[key[1]][value[1]][0]
                    h2,p2,t2 = self.robot.world.server.camera_landmark_pool[key[1]][value[1]][1]
                    theta_t = wrap_angle(p1 - p2)
                    x_t = x2 - ( x1*cos(theta_t) + y1*sin(theta_t))
                    y_t = y2 - (-x1*sin(theta_t) + y1*cos(theta_t))
                    self.transforms[key] = (x_t, y_t, theta_t, value[1])
            self.update_foreign_robot()
            self.update_foreign_objects()
            sleep(0.01)

    def update_foreign_robot(self):
        for key, value in self.transforms.items():
            if key[1] == self.robot.aruco_id:
                x_t, y_t, theta_t, cap = value
                x, y, theta = self.robot.world.server.poses[key[0]]
                x2 =  x*cos(theta_t) + y*sin(theta_t) + x_t
                y2 = -x*sin(theta_t) + y*cos(theta_t) + y_t
                # improve using update function instead of new obj everytime
                self.robot.world.world_map.objects["Foreign-"+str(key[0])]=RobotForeignObj(cozmo_id=key[0],
                                     x=x2, y=y2, z=0, theta=wrap_angle(theta-theta_t), camera_id = int(cap[-2]))

    def update_foreign_objects(self):
        for key, value in self.transforms.items():
            if key[1] == self.robot.aruco_id:
                x_t, y_t, theta_t, cap = value
                for k, v in self.robot.world.server.foreign_objects[key[0]].items():
                    x2 =  v.x*cos(theta_t) + v.y*sin(theta_t) + x_t
                    y2 = -v.x*sin(theta_t) + v.y*cos(theta_t) + y_t
                    if isinstance(k,str) and "Wall" in k:
                        # update wall
                        if k in self.robot.world.world_map.objects:
                            if self.robot.world.world_map.objects[k].foreign:
                                self.robot.world.world_map.objects[k].update(x=x2, y=y2, theta=wrap_angle(v.theta-theta_t))
                        else:
                            copy_obj = deepcopy(v)
                            copy_obj.x = x2
                            copy_obj.y = y2
                            copy_obj.theta = wrap_angle(v.theta-theta_t)
                            copy_obj.foreign = True
                            self.robot.world.world_map.objects[k]=copy_obj
                    elif isinstance(k,str) and "Cube" in k and not self.robot.world.light_cubes[v.id].is_visible:
                        # update cube
                        if k in self.robot.world.world_map.objects:
                            if self.robot.world.world_map.objects[k].foreign:
                                self.robot.world.world_map.objects[k].update(x=x2, y=y2, theta=wrap_angle(v.theta-theta_t))
                        else:
                            copy_obj = deepcopy(v)
                            copy_obj.x = x2
                            copy_obj.y = y2
                            copy_obj.theta = wrap_angle(v.theta-theta_t)
                            copy_obj.foreign = True
                            self.robot.world.world_map.objects[k]=copy_obj


class ClientThread(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.port = None
        self.socket = None #not running until startClient is called
        self.ipaddr = None
        self.robot= robot
        self.to_send = {}

    def start_client_thread(self,ipaddr="",port=1800):
        if self.robot.aruco_id == -1:
            self.robot.aruco_id = int(input("Please enter the aruco id of the robot:"))
            self.robot.world.server.camera_landmark_pool[self.robot.aruco_id]={}
        self.port = port
        self.ipaddr = ipaddr
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,True)
        while True:
            try:
                print("Attempting to connect to %s at port %d" % (ipaddr,port))
                self.socket.connect((ipaddr,port))
                data = pickle.loads(self.socket.recv(1024))
                break
            except:
                print("No server found, make sure the address is correct, retrying in 10 seconds")
                sleep(10)
        print("Connected.")
        self.socket.sendall(pickle.dumps(self.robot.aruco_id))
        self.robot.world.is_server = False
        self.start()

    def use_shared_map(self):
        # currently affects only worldmap_viewer
        # uses robot.world.world_map.shared_objects instead of robot.world.world_map.objects
        self.robot.use_shared_map = True

    def use_local_map(self):
        self.robot.use_shared_map = False

    def run(self):
        # Send from client to server
        while(True):
            # hack to recieve variable size data without crashing
            data = b''
            while True:
                data += self.socket.recv(1024)
                if data[-3:]==b'end':
                    break
            self.robot.world.perched.camera_pool, self.robot.world.world_map.shared_objects = pickle.loads(data[:-3])

            for key, value in self.robot.world.world_map.objects.items():
                if isinstance(key,LightCube):
                    self.to_send["LightCubeForeignObj-"+str(value.id)]= LightCubeForeignObj(id=value.id, cozmo_id=self.robot.aruco_id, x=value.x, y=value.y, z=value.z, theta=value.theta)
                elif isinstance(key,str) and 'Wall' in key:
                    # Send walls
                    self.to_send[key] = value         # Fix case when object removed from shared map
                else:
                    pass    

            # send cameras, landmarks, objects and pose
            self.socket.sendall(pickle.dumps([self.robot.world.perched.cameras,
                {k:self.robot.world.particle_filter.sensor_model.landmarks[k] for k in 
                [x for x in self.robot.world.particle_filter.sensor_model.landmarks.keys() 
                if isinstance(x,str) and "Video" in x]},
                self.to_send,
                self.robot.world.particle_filter.pose])+b'end')
