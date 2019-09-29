"""
Path planner using RRT and Wavefront algorithms.
"""

from math import pi, sin, cos
from multiprocessing import Process
import cv2

from .nodes import LaunchProcess
from .events import DataEvent, PilotEvent
from .pilot0 import NavPlan, NavStep
from .worldmap import WorldObject, LightCubeObj, ChargerObj, CustomMarkerObj, RoomObj, DoorwayObj
from .rrt import RRT, RRTNode, StartCollides, GoalCollides, GoalUnreachable
from .wavefront import WaveFront
from .geometry import wrap_angle

from . import rrt

class PathPlanner():
    """This path planner can be called directly, or it can be used inside
    a PathPlannerProcess node that runs the heavy lifting portion of
    the algorithm in a child process.  Because child processes in
    Windows don't share memory with the parent, we must transmit
    certain data to the child as parameters during process creation.
    But only structures that are pickle-able can be sent.  The
    setup_problem() method sets up those structures, and do_planning()
    uses them to do the work.  If we don't want to run in a separate
    process then we can use plan_path_this_process() to call both
    methods in the main process and return the result.
    """

    @staticmethod
    def plan_path_this_process(goal_object, robot, use_doorways=False):
        # Get pickle-able data structures
        (start_node, goal_shape, robot_parts, bbox,
         fat_obstacles, skinny_obstacles, doorway_list, need_grid_display) = \
            __class__.setup_problem(goal_object, robot, use_doorways)
        # Do the actual path planning
        result = \
            __class__.do_planning(robot.world.rrt, start_node, goal_shape,
                                  fat_obstacles, skinny_obstacles, doorway_list,
                                  need_grid_display)
        return result

    @staticmethod
    def setup_problem(goal_object, robot, use_doorways):
        """Calculate values from world map in main process since the map won't
        be available in the child process."""

        # Fat obstacles and narrow doorways for WaveFront
        obstacle_inflation = 20  # must be << pilot's escape_distance
        passageway_adjustment = -40  # narrow doorways for WaveFront
        robot.world.rrt.generate_obstacles(obstacle_inflation, passageway_adjustment)
        fat_obstacles = robot.world.rrt.obstacles

        # Skinny obstacles and wide doorways for RRT
        obstacle_inflation = 10
        passageway_adjustment = 0  # normal doorways for RRT
        robot.world.rrt.generate_obstacles(obstacle_inflation, passageway_adjustment)
        skinny_obstacles = robot.world.rrt.obstacles

        (pose_x, pose_y, pose_theta) = robot.world.particle_filter.pose
        start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)

        if isinstance(goal_object, (LightCubeObj,ChargerObj)):
            goal_shape = RRT.generate_cube_obstacle(goal_object)
        elif isinstance(goal_object, CustomMarkerObj):
            goal_shape = RRT.generate_marker_obstacle(goal_objject)
        elif isinstance(goal_object, RoomObj):
            goal_shape = RRT.generate_room_obstacle(goal_object)
        else:
            raise ValueError("Can't convert path planner goal %s to shape." % goal_object)

        robot_parts = robot.world.rrt.make_robot_parts(robot)
        bbox = robot.world.rrt.compute_bounding_box()

        if use_doorways:
            doorway_list = robot.world.world_map.generate_doorway_list()
        else:
            doorway_list = []  # ignore doorways in simulator

        need_grid_display = robot.world.path_viewer is not None

        return (start_node, goal_shape, robot_parts, bbox,
                fat_obstacles, skinny_obstacles, doorway_list, need_grid_display)

    @staticmethod
    def do_planning(rrt_instance, start_node, goal_shape,
                    fat_obstacles, skinny_obstacles, doorway_list, need_grid_display):
        """Does the heavy lifting; may be called in a child process."""

        escape_options = (
                           # angle       distance(mm)
                           (0,            40),
                           (+30/180*pi,   50),
                           (-30/180*pi,   50),
                           (pi,           40),
                           (+60/180*pi,   80),
                           (-60/180*pi,   80),
                           (+pi/2,        70),
                           (-pi/2,        70)
        )

        rrt_instance.obstacles = skinny_obstacles
        start_escape_move = None

        wf = WaveFront(bbox=rrt_instance.bbox)
        for obstacle in fat_obstacles:
            wf.add_obstacle(obstacle)

        collider = rrt_instance.collides(start_node)
        if not collider:
            collider = wf.check_start_collides(start_node.x, start_node.y)

        if collider:
            q = start_node.q
            for (phi, escape_distance) in escape_options:
                if phi != pi:
                    new_q = wrap_angle(q + phi)
                else:
                    new_q = q
                new_start = RRTNode(x=start_node.x + escape_distance*cos(q+phi),
                                    y=start_node.y + escape_distance*sin(q+phi),
                                    q=new_q)
                collider2 = rrt_instance.collides(new_start)
                print('trying escape', new_start, collider2)
                if not collider2  and \
                   not wf.check_start_collides(new_start.x,new_start.y):
                    start_escape_move = (phi, start_node, new_start)
                    start_node = new_start
                    break
            if start_escape_move is None:
                print('PathPlanner: Start collides!', collider)
                return PilotEvent(StartCollides,collider)

        # Run the wavefront path finder
        rrt_instance.obstacles = fat_obstacles
        wf.set_goal_shape(goal_shape)
        wf_start = (start_node.x, start_node.y)
        goal_found = wf.propagate(*wf_start)
        grid_display = None if not need_grid_display else wf.make_grid_display()
        if goal_found is None:
            print('PathPlanner: goal unreachable!')
            return PilotEvent(GoalUnreachable, grid_display)

        # Extract and smooth the path
        coords_pairs = wf.extract(goal_found)
        rrt_instance.path = rrt_instance.coords_to_path(coords_pairs)
        rrt_instance.obstacles = skinny_obstacles
        rrt_instance.smooth_path()

        # Construct the navigation plan
        navplan = NavPlan.from_path(rrt_instance.path, doorway_list)

        # Insert the StartCollides escape move if there is one
        if start_escape_move:
            phi, start, new_start = start_escape_move
            if phi == pi:
                escape_step = NavStep(NavStep.BACKUP, (RRTNode(x=new_start.x, y=new_start.y),))
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
        result = (navplan, grid_display)
        return DataEvent(result)

#----------------------------------------------------------------

class PathPlannerProcess(LaunchProcess):
    def start(self, event=None):
        if not isinstance(event,DataEvent):
            raise ValueError('PathPlanner node must be invoked with a DataEvent for the goal.')
        goal_object = event.data
        if not isinstance(goal_object, WorldObject):
            raise ValueError('Path planner goal %s is not a WorldObject' % goal_object)
        self.goal_object = goal_object
        super().start(event)  # will call create_process

    def create_process(self, reply_token):
        use_doorways = True   # assume we're running on the robot
        (start_node, goal_shape, robot_parts, bbox,
         fat_obstacles, skinny_obstacles, doorway_list, need_grid_display) = \
            PathPlanner.setup_problem(self.goal_object, self.robot, user_doorways)
        p = Process(target=self.__class__.process_workhorse,
                    args = [reply_token,
                            start_node, goal_shape, robot_parts, bbox,
                            fat_obstacles, skinny_obstacles, doorway_list,
                            need_grid_display])
        return p

    @staticmethod
    def process_workhorse(reply_token, start_node, goal_shape, robot_parts, bbox,
                          fat_obstacles, skinny_obstacles, doorway_list, need_grid_display):
        cv2.startWindowThread()
        rrt_instance = RRT(robot_parts=robot_parts, bbox=bbox)
        result = \
            PathPlanner.do_planning(rrt_instance, start_node, goal_shape,
                                    fat_obstacles, skinny_obstacles, doorway_list,
                                    need_grid_display)
        __class__.post_event(reply_token, result)

