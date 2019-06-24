"""
Create a dummy robot and world so we can use cozmo-tools
classes without having to connect to a real robot.
"""

import cozmo

from cozmo.util import Distance, Angle, Pose
from .cozmo_kin import CozmoKinematics

from .particle import SLAMParticleFilter
from .rrt import RRT
from .worldmap import WorldMap

class SimWorld(): pass

class SimRobot():
    def __init__(self):
        robot = self

        robot.head_angle = Angle(radians=0)
        robot.shoulder_angle = Angle(radians=0)
        robot.lift_height = Distance(distance_mm=0)
        robot.pose = Pose(0,0,0,angle_z=Angle(degrees=0))

        robot.world = SimWorld()
        robot.world.particle_filter = SLAMParticleFilter(robot)
        robot.kine = CozmoKinematics(robot)  # depends on particle filter
        robot.world.rrt = RRT(robot) # depends on kine
        robot.world.world_map = WorldMap(robot)

