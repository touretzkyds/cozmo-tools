from .base import *
from .rrt import *
from .nodes import Turn, Forward
import time

from cozmo.util import distance_mm, speed_mmps

class NavStep():
    FORWARD = "forward"
    BACKWARD = "backward"
    HEADING = "heading"
    ARC = "arc"

    def __init__(self, type, params):
        self.type = type
        self.params = params

    def __repr__(self):
        return '<NavStep %.1f,%.1f>' % self.params

class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    @staticmethod
    def from_path(path):
        steps = []
        for node in path:
            steps.append(NavStep(NavStep.FORWARD,
                                 (node.x, node.y, node.q)))
        if path[-2].x == path[-1].x and path[-2].y == path[-1].y:
            steps[-1].type = NavStep.HEADING
        return NavPlan(steps)

class PilotBase(StateNode):
    def __init__(self):
        super().__init__()

    def planner(self):
        raise ValueError('No planner specified')

class PilotToPose(PilotBase):
    def __init__(self,pose):
        super().__init__()
        self.target_pose = pose

    def planner(self,start_node,goal_node):
        return self.robot.world.rrt.plan_path(start_node,goal_node)

    def start(self,event=None):
        super().start(event)
        (pose_x, pose_y, pose_theta) = self.robot.world.particle_filter.pose
        start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)
        tpose = self.target_pose
        goal_node = RRTNode(x=tpose.position.x, y=tpose.position.y,
                            q=tpose.rotation.angle_z.radians)

        try:
            (treeA, treeB, path) = self.planner(start_node, goal_node)                    
        except StartCollides:
            print('Start collides!')
            self.post_failure()
            return
        except GoalCollides:
            print('Goal collides!')
            self.post_failure()
            return
        except MaxIterations:
            print('Max iteration exceeded!')
            self.post_failure()
            return

        print(len(treeA)+len(treeB),'nodes')
        if self.robot.world.path_viewer:
            self.robot.world.path_viewer.add_tree(path, (1,0,0,0.75))

        # construct nav plan
        print('path=',path)
        self.plan = NavPlan.from_path(path)
        self.robot.loop.create_task(self.execute_plan())

    async def execute_plan(self):
        for step in self.plan.steps[1:]:
            if not self.running: return
            (targ_x, targ_y, targ_hdg) = step.params
            self.robot.world.particle_filter.variance_estimate()
            (cur_x,cur_y,cur_hdg) = self.robot.world.particle_filter.pose
            if step.type == NavStep.HEADING:
                turn_angle = wrap_angle(targ_hdg - cur_hdg)
                await self.robot.turn_in_place(cozmo.util.radians(turn_angle)).wait_for_completed()
                continue
            # FORWARD step
            print('cur = ', (cur_x,cur_y), ' @ ', cur_hdg*180/pi, 'deg.')
            dx = targ_x - cur_x
            dy = targ_y - cur_y
            dist = sqrt(dx**2 + dy**2)
            course = atan2(dy,dx)
            turn_angle = wrap_angle(course - cur_hdg)
            print('targ =', (targ_x,targ_y), 'turn_angle=',turn_angle*180/pi)
            await self.robot.turn_in_place(cozmo.util.radians(turn_angle)).wait_for_completed()
            if not self.running: return
            print('dist=',dist)
            await self.robot.drive_straight(distance_mm(dist),
                                            speed_mmps(50)).wait_for_completed()
        print('done executing')
        self.post_completion()

class PilotPushToPose(PilotToPose):
    def planner(self,start_node,goal_node):
        return self.robot.world.rrt.plan_push_chip(start_node,goal_node)
