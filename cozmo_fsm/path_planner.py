"""
Path planner using RRT and Wavefront algorithms.
"""

from math import pi, sin, cos

from .base import StateNode
from .events import DataEvent
from .pilot0 import NavPlan
from .worldmap import WorldObject, LightCubeObj, ChargerObj, CustomMarkerObj, RoomObj, DoorwayObj
from .rrt import RRT, RRTNode
from .wavefront import WaveFront
from .transform import wrap_angle

from . import rrt

class PathPlanner(StateNode):

    def __init__(self):
        super().__init__()

    def start(self, event=None):
        super().start(event)
        goal_object = event.data if isinstance(event,DataEvent) else None
        if not isinstance(goal_object, WorldObject):
            raise ValueError('Path planner goal %s is not a WorldObject' % goal_object)

        # Fat obstacles and narrow doorways for WaveFront
        obstacle_inflation = 10  # must be << pilot's escape_distance
        passageway_adjustment = -40  # narrow doorways for WaveFront
        self.robot.world.rrt.generate_obstacles(obstacle_inflation, passageway_adjustment)
        fat_obstacles = self.robot.world.rrt.obstacles

        # Skinny obstacles and wide doorways for RRT
        obstacle_inflation = 5
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

        doorway_list = self.generate_doorway_list()

        need_tree = self.robot.world.path_viewer is not None

        result = \
            PathPlanner.plan_path(start_node, goal_shape, robot_parts, bbox,
                                  fat_obstacles, skinny_obstacles, doorway_list, need_tree)
        return result

    def generate_doorway_list(self):
        doorways = []
        for (key,obj) in self.robot.world.world_map.objects.items():
            if isinstance(obj,DoorwayObj):
                w = obj.door_width
                door_theta = obj.theta + pi/2
                dx = w * sin(door_theta)
                dy = w * cos(door_theta)
                doorways.append((obj, ((obj.x-dx, obj.y-dy), (obj.x+dx, obj.y+dy))))
        return doorways

    @staticmethod
    def plan_path(start_node, goal_shape, robot_parts, bbox,
                  fat_obstacles, skinny_obstacles, doorway_list, need_tree):
        rrt = RRT(robot_parts=robot_parts)
        rrt.bbox = bbox

        start_escape_move = None
        rrt.obstacles = skinny_obstacles
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
                print('PilotPlanner: Start collides!',e)
                raise NotImplementedError('path planner report failure')
                self.parent.post_event(PilotEvent(StartCollides, e.args))
                self.parent.post_failure()
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
        if goal_found is None:
            raise NotImplementedError('wavefront could not reach goal')

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
                escape_step = NavStep(NavStep.BACKUP, (new_start.x, new_start.y))
                navplan.steps.insert(0, escape_step)
            elif navplan.steps[0].type == NavStep.DRIVE:
                navplan.steps[0].param.insert(0, (start.x, start.y))
            else:
                # Shouldn't get here, but just in case
                escape_step = NavStep(NavStep.DRIVE, ((start.x,start.y), (new_start.x,new_start.y)))
                navplan.steps.insert(0, escape_step)

        # Return the navigation plan
        return navplan
