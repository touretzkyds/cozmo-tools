"""
Create a dummy robot and world so we can use cozmo-tools
classes without having to connect to a real robot.
"""

import asyncio

try:
    import cv2
    ARUCO_DICT_4x4_100 = cv2.aruco.DICT_4X4_100
except:
    ARUCO_DICT_4x4_100 = None

import cozmo
from cozmo.util import Distance, Angle, Pose

from .cozmo_kin import CozmoKinematics
from .evbase import EventRouter
from .aruco import Aruco
from .particle import SLAMParticleFilter
from .rrt import RRT, RRTNode
from .worldmap import WorldMap

class SimWorld():
    def __init__(self):
        self.path_viewer = None
        self.particle_viewer = None
        self.worldmap_viewer = None

class SimServer():
    def __init__(self):
        self.started = False

class SimRobot():
    def __init__(self, run_in_cloud=False):
        robot = self

        robot.loop = asyncio.get_event_loop()

        if not run_in_cloud:
            robot.erouter = EventRouter()
            robot.erouter.robot = robot
            robot.erouter.start()

        robot.head_angle = Angle(radians=0)
        robot.shoulder_angle = Angle(radians=0)
        robot.lift_height = Distance(distance_mm=0)
        robot.pose = Pose(0,0,0,angle_z=Angle(degrees=0))
        robot.camera = None
        robot.carrying = None

        robot.world = SimWorld()
        robot.world.aruco = Aruco(robot, ARUCO_DICT_4x4_100)
        robot.world.light_cubes = dict()
        robot.world._faces = dict()
        robot.world.charger = None
        robot.world.server = SimServer()
        robot.world.path_viewer = None

        robot.world.particle_filter = SLAMParticleFilter(robot)
        robot.kine = CozmoKinematics(robot)  # depends on particle filter
        robot.world.rrt = RRT(robot) # depends on kine
        robot.world.world_map = WorldMap(robot)
