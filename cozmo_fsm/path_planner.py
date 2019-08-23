"""
Path planner using RRT and Wavefront algorithms.
"""

from math import pi, sin, cos
from multiprocessing import Process
import cv2

from .nodes import LaunchProcess
from .events import DataEvent, PilotEvent
from .pilot0 import NavPlan
from .worldmap import WorldObject, LightCubeObj, ChargerObj, CustomMarkerObj, RoomObj, DoorwayObj
from .rrt import RRT, RRTNode, StartCollides, GoalCollides, GoalUnreachable
from .wavefront import WaveFront
from .geometry import wrap_angle

from . import rrt

class PathPlanner(LaunchProcess):

    def start(self, event=None):
        if not isinstance(event,DataEvent):
            raise ValueError('PathPlanner node must be invoked with a DataEvent for the goal.')
        goal_object = event.data
        if not isinstance(goal_object, WorldObject):
            raise ValueError('Path planner goal %s is not a WorldObject' % goal_object)
        self.goal_object = goal_object
        super().start(event)

    def create_process(self,reply_token):
        goal_object = self.goal_object
        # Fat obstacles and narrow doorways for WaveFront
        obstacle_inflation = 20  # must be << pilot's escape_distance
        passageway_adjustment = -40  # narrow doorways for WaveFront
        self.robot.world.rrt.generate_obstacles(obstacle_inflation, passageway_adjustment)
        fat_obstacles = self.robot.world.rrt.obstacles

        # Skinny obstacles and wide doorways for RRT
        obstacle_inflation = 10
        passageway_adjustment = +77  # widen doorways for RRT
        self.robot.world.rrt.generate_obstacles(obstacle_inflation, passageway_adjustment)
        skinny_obstacles = self.robot.world.rrt.obstacles

        (pose_x, pose_y, pose_theta) = self.robot.world.particle_filter.pose
        start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)

        if isinstance(goal_object, (LightCubeObj,ChargerObj)):
            goal_shape = RRT.generate_cube_obstacle(goal_object)
        elif isinstance(goal_object, CustomMarkerObj):
            goal_shape = RRT.generate_marker_obstacle(goal_objject)
        elif isinstance(goal_object, RoomObj):
            goal_shape = RRT.generate_room_obstacle(goal_object)
        else:
            raise ValueError("Can't convert path planner goal %s to shape." % goal_object)

        robot_parts = self.robot.world.rrt.make_robot_parts(self.robot)
        bbox = self.robot.world.rrt.compute_bounding_box()
        doorway_list = self.robot.world.world_map.generate_doorway_list()
        need_tree = self.robot.world.path_viewer is not None

        p = Process(target=self.__class__.process_workhorse,
                    args = [reply_token,
                            start_node, goal_shape, robot_parts, bbox,
                            fat_obstacles, skinny_obstacles, doorway_list, need_tree])
        return p

    @staticmethod
    def process_workhorse(reply_token, start_node, goal_shape, robot_parts, bbox,
                          fat_obstacles, skinny_obstacles, doorway_list, need_tree):
        cv2.startWindowThread()
        rrt = RRT(robot_parts=robot_parts, bbox=bbox)
        rrt.obstacles = skinny_obstacles
        start_escape_move = None

        collider = rrt.collides(start_node)
        if collider:
            escape_distance = 50 # mm
            escape_headings = (0, +30/180.0*pi, -30/180.0*pi, pi, pi/2, -pi/2)
            for phi in escape_headings:
                if phi != pi:
                    new_q = wrap_angle(start_node.q + phi)
                else:
                    new_q = start_node.q
                new_start = RRTNode(x=start_node.x + escape_distance*cos(new_q),
                                    y=start_node.y + escape_distance*sin(new_q),
                                    q=new_q)
                if not rrt.collides(new_start):
                    start_escape_move = (phi, start_node, new_start)
                    start_node = new_start
                    break
            if start_escape_move is None:
                print('PathPlanner: Start collides!', collider)
                PathPlanner.post_event(reply_token, PilotEvent(StartCollides,collider))
                return

        # Run the wavefront path finder
        rrt.obstacles = fat_obstacles
        wf = WaveFront(bbox=rrt.bbox)
        PathPlanner.wf = wf
        for obstacle in fat_obstacles:
            wf.add_obstacle(obstacle)
        wf.set_goal_shape(goal_shape)
        wf_start = (start_node.x, start_node.y)
        goal_found = wf.propagate(*wf_start)
        wf.display_grid()
        if goal_found is None:
            print('PathPlanner: goal unreachable!')
            PathPlanner.post_event(reply_token, PilotEvent(GoalUnreachable))
            return

        # Extract and smooth the path
        coords_pairs = wf.extract(goal_found)
        rrt.path = rrt.coords_to_path(coords_pairs)
        rrt.obstacles = skinny_obstacles
        rrt.smooth_path()

        # Construct the navigation plan
        navplan = NavPlan.from_path(rrt.path, doorway_list)

        # Insert the StartCollides escape move if there is one
        if start_escape_move:
            phi, start, new_start = start_escape_move
            if phi == pi:
                escape_step = NavStep(NavStep.BACKUP, RRTNode(x=new_start.x, y=new_start.y))
                navplan.steps.insert(0, escape_step)
            elif navplan.steps[0].type == NavStep.DRIVE:
                navplan.steps[0].param.insert(0, RRTNode(x=start.x, y=start.y))
            else:
                # Shouldn't get here, but just in case
                print("Shouldn't end up here!", navplan.steps[0])
                escape_step = NavStep(NavStep.DRIVE,
                                      (RRTNode(x=start.x, y=start.y),
                                       RRTNode(x=new_start.x, y=new_start.y)))
                navplan.steps.insert(0, escape_step)

        # Return the navigation plan
        result = (navplan, rrt.path)
        PathPlanner.post_event(reply_token, DataEvent(result))
