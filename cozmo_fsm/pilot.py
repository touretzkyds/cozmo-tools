import math
import time
import asyncio

from .base import *
from .rrt import *
from .nodes import DriveArc
from .cozmo_kin import wheelbase

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
            if node.radius == 0:
                steps.append(NavStep(NavStep.FORWARD,
                                     (node.x, node.y, node.q)))
            else:
                steps.append(NavStep(NavStep.ARC,
                                     (node.x, node.y, node.q, node.radius)))
        if path[-2].x == path[-1].x and path[-2].y == path[-1].y:
            steps[-1].type = NavStep.HEADING
        return NavPlan(steps)

class PilotBase(StateNode):
    def __init__(self):
        super().__init__()
        self.handle = None
        self.arc_radius = 40
        self.max_turn = pi

    def stop(self):
        if self.handle:
            self.handle.cancel()
            self.handle = None
        super().stop()

    def planner(self):
        raise ValueError('No planner specified')

    def calculate_arc(self, cur_x, cur_y, cur_q, dest_x, dest_y):
        # Compute arc node parameters to get us on a heading toward node_j.
        direct_turn_angle = wrap_angle(atan2(dest_y-cur_y, dest_x-cur_x) - cur_q)
        # find center of arc we'll be moving along
        dir = +1 if direct_turn_angle >=0 else -1
        cx = cur_x + self.arc_radius * cos(cur_q + dir*pi/2)
        cy = cur_y + self.arc_radius * sin(cur_q + dir*pi/2)
        dx = cx - dest_x 
        dy = cy - dest_y
        center_dist = sqrt(dx*dx + dy*dy)
        if center_dist < self.arc_radius:  # turn would be too wide: punt
            print('*** TURN TOO WIDE ***, center_dist =',center_dist)
            center_dist = self.arc_radius
        # tangent points on arc: outer tangent formula from Wikipedia with r=0
        gamma = atan2(dy, dx)
        beta = asin(self.arc_radius / center_dist)
        alpha1 = gamma + beta
        tang_x1 = cx + self.arc_radius * cos(alpha1 + pi/2)
        tang_y1 = cy + self.arc_radius * sin(alpha1 + pi/2)
        tang_q1 = (atan2(tang_y1-cy, tang_x1-cx) + dir*pi/2)
        turn1 = tang_q1 - cur_q
        if dir * turn1 < 0:
            turn1 += dir * 2 * pi
        alpha2 = gamma - beta
        tang_x2 = cx + self.arc_radius * cos(alpha2 - pi/2)
        tang_y2 = cy + self.arc_radius * sin(alpha2 - pi/2)
        tang_q2 = (atan2(tang_y2-cy, tang_x2-cx) + dir*pi/2)
        turn2 = tang_q2 - cur_q
        if dir * turn2 < 0:
            turn2 += dir * 2 * pi
        # Correct tangent point has shortest turn.
        if abs(turn1) < abs(turn2):
            (tang_x,tang_y,tang_q,turn) = (tang_x1,tang_y1,tang_q1,turn1)
        else:
            (tang_x,tang_y,tang_q,turn) = (tang_x2,tang_y2,tang_q2,turn2)
        return (dir*self.arc_radius, turn)

    async def drive_arc(self,radius,angle):
        speed = 50
        l_wheel_speed = speed * (1 - wheelbase / radius)
        r_wheel_speed = speed * (1 + wheelbase / radius)
        last_heading = self.robot.pose.rotation.angle_z.degrees
        traveled = 0
        cor = self.robot.drive_wheels(l_wheel_speed, r_wheel_speed)
        self.handle = self.robot.loop.create_task(cor)
        while abs(traveled) < abs(angle):
            await asyncio.sleep(0.05)
            p0 = last_heading
            p1 = self.robot.pose.rotation.angle_z.degrees
            last_heading = p1
            diff = p1 - p0
            if diff  < -90.0:
                diff += 360.0
            elif diff > 90.0:
                diff -= 360.0
            traveled += diff
        self.handle.cancel()
        self.handle = None
        self.robot.stop_all_motors()
        print('drive_arc angle=',angle,'deg.,  traveled=',traveled,'deg.')
        
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
        except StartCollides as e:
            print('Start collides!',e)
            self.post_failure()
            return
        except GoalCollides as e:
            print('Goal collides!',e)
            self.post_failure()
            return
        except MaxIterations:
            print('Max iteration exceeded!')
            self.post_failure()
            return

        print(len(treeA)+len(treeB),'nodes')
        if self.robot.world.path_viewer:
            self.robot.world.path_viewer.add_tree(path, (1,0,0,0.75))

        # Construct and execute nav plan
        [print(x) for x in path]
        self.plan = NavPlan.from_path(path)
        self.robot.loop.create_task(self.execute_plan())

    async def execute_plan(self):
        print('-------- Executing Nav Plan --------')
        for step in self.plan.steps[1:]:
            if not self.running: return
            self.robot.world.particle_filter.variance_estimate()
            (cur_x,cur_y,cur_hdg) = self.robot.world.particle_filter.pose
            if step.type == NavStep.HEADING:
                (targ_x, targ_y, targ_hdg) = step.params
                turn_angle = wrap_angle(targ_hdg - cur_hdg)
                await self.robot.turn_in_place(cozmo.util.radians(turn_angle)).wait_for_completed()
                continue
            elif step.type == NavStep.FORWARD:
                (targ_x, targ_y, targ_hdg) = step.params
                dx = targ_x - cur_x
                dy = targ_y - cur_y
                course = atan2(dy,dx)
                turn_angle = wrap_angle(course - cur_hdg)
                print('FORWARD: cur=(%.1f,%.1f) @ %.1f deg.,  targ=(%.1f,%.1f) @ %.1f deg, turn_angle=%.1f deg.' %
                      (cur_x,cur_y,cur_hdg*180/pi,
                       targ_x,targ_y,targ_hdg*180/pi,turn_angle*180/pi))
                if abs(turn_angle) > self.max_turn:
                    turn_angle = self.max_turn if turn_angle > 0 else -self.max_turn
                    print('  ** TURN ANGLE SET TO', turn_angle*180/pi)
                # *** HACK: skip node if it requires unreasonable turn
                if abs(turn_angle) < 2*pi/180 or wrap_angle(course-targ_hdg) > pi/2:
                    print('  ** SKIPPED TURN **')
                else:
                    await self.robot.turn_in_place(cozmo.util.radians(turn_angle)).wait_for_completed()
                if not self.running: return
                (cur_x,cur_y,cur_hdg) = self.robot.world.particle_filter.pose
                dx = targ_x - cur_x
                dy = targ_y - cur_y
                dist = sqrt(dx**2 + dy**2)
                await self.robot.drive_straight(distance_mm(dist),
                                                speed_mmps(50)).wait_for_completed()
            elif step.type == NavStep.ARC:
                (targ_x, targ_y, targ_hdg, radius) = step.params
                print('ARC: cur=(%.1f,%.1f) @ %.1f deg.,  targ=(%.1f,%.1f), targ_hdg=%.1f deg., radius=%.1f' %
                      (cur_x,cur_y,cur_hdg*180/pi,targ_x,targ_y,targ_hdg*180/pi,radius))
                (actual_radius, actual_angle) = \
                                self.calculate_arc(cur_x, cur_y, cur_hdg, targ_x, targ_y)
                print(' ** actual_radius =', actual_radius, '  actual_angle=', actual_angle*180/pi)
                await self.drive_arc(actual_radius, math.degrees(abs(actual_angle)))
            else:
                raise ValueError('Invalid NavStep',step)
        print('done executing')
        self.post_completion()

class PilotPushToPose(PilotToPose):
    def __init__(self,pose):
        super().__init__(pose)
        self.max_turn = 20*(pi/180)

    def planner(self,start_node,goal_node):
        return self.robot.world.rrt.plan_push_chip(start_node,goal_node)
