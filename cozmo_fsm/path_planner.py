"""
Path planner using RRT and Wavefront algorithms.
"""

from .base import StateNode
from .events import DataEvent
from .worldmap import WorldObject
from .rrt import RRT
from .wavefront import WaveFront

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
            goal_shape = rrt.generate_cube_obstacle(obj)
        elif isinstance(goal_object, CustomMarkerObj):
            goal_shape = rrt.generate_marker_obstacle(obj)
        elif isinstance(goal_object, RoomObj):
            goal_shape = rrt.generate_room_obstacle(obj)
        else:
            raise ValueError("Can't convert path planner goal %s to shape." % obj)

        doorway_list = self.generate_doorway_list()

        need_tree = self.robot.path_viewer is not None

        PathPlanner.plan_path(start_node, goal_shape, fat_obstacles, skinny_obstacles, doorway_list, need_tree)

    def generate_doorway_list(self):
        doorways = []
        for (key,obj) in self.robot.world.world_map.objects.items():
            if isinstance(obj,DoorwayObj):
                w = obj.door_width
                dx = w * sin(obj.theta)
                dy = w * cos(obj.theta)
                doorways.append((obj, ((obj.x-dx, obj.y-dy), (obj.x+dx, obj.y+dy))))
        return doorways

    @staticmethod
    def plan_path(start_node, goal_shape, fat_obstacles, skinny_obstacles, doorway_list, need_tree):
        rrt = RRT()

        start_escape_move = None
        rrt.obstacles = skinny_obstacles
        collider = rrt.collides(start_node)
        if collider:
            escape_headings = (0, +30/180.0*pi, -30/180.0*pi, pi, pi/2, -pi/2)
            for phi in escape_headings:
                if phi != pi:
                    new_q = wrap_angle(start_node.q + phi)
                else:
                    new_q = start_node.q
                    new_start = RRTNode(x=start_node.x + escape_distance*cos(new_q),
                                        y=start_node.y + escape_distance*sin(new_q),
                                        q=new_q)
                if not self.robot.world.rrt.collides(new_start):
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
            rrt.compute_bounding_box()
            wf = WaveFront(bbox=rrt.bbox)
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

