import math
import time
import sys
import asyncio

from .base import *
from .rrt import *
from .nodes import ParentFails, ParentCompletes, DriveArc, DriveContinuous, Forward
from .events import PilotEvent
from .transitions import CompletionTrans, FailureTrans, SuccessTrans, DataTrans
from .cozmo_kin import wheelbase, center_of_rotation_offset
from .worldmap import DoorwayObj
from .transform import segment_intersect_test
from .doorpass import DoorPass
from .pilot0 import *

from cozmo.util import Pose, distance_mm, radians, degrees, speed_mmps

#---------------- Pilot Exceptions and Events ----------------

class PilotException(Exception):
    def __str__(self):
        return self.__repr__()

class InvalidPose(PilotException): pass
class CollisionDetected(PilotException): pass

# Note: StartCollides, GoalCollides, and MaxIterations exceptions are defined in rrt.py.

class ParentPilotEvent(StateNode):
    """Receive a PilotEvent and repost it from the receiver's parent. This allows
     derived classes that use the Pilot to make its PilotEvents visible."""
    def start(self,event):
        super().start(event)
        if not isinstance(event,PilotEvent):
            raise TypeError("ParentPilotEvent must be invoked with a PilotEvent, not %s" % event)
        self.parent.post_event(PilotEvent(event.status))

#---------------- Navigation Plan ----------------

class NavStep():
    DRIVE = "drive"
    DOORPASS = "doorpass"
    BACKUP = "backup"

    def __init__(self, type, param):
        self.type = type
        self.param = param

    def __repr__(self):
        if self.type == NavStep.DOORPASS:
            pstring = self.param.id
        elif self.type == NavStep.DRIVE:
            psteps = [(round(x,1),round(y,1)) for (x,y) in self.param]
            pstring = repr(psteps)
        else:
            pstring = repr(self.param)
            if len(pstring) > 40:
                pstring = pstring[0:20] + ' ...' + pstring[-20:]
        return '<NavStep %s %s>' % (self.type, pstring)


class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    @staticmethod
    def intersects_doorway(p1, p2, doorways):
        for door in doorways:
            if segment_intersect_test(p1, p2, door[1][0], door[1][1]):
                return door[0]
        return None

    @staticmethod
    def from_path(path, doorways):
        steps = []
        door = None
        pt1 = path[0]
        for i in range(1, len(path)):
            pt2 = path[i]
            door = NavPlan.intersects_doorway(pt1,pt2,doorways)
            #print('i=',i,'pt1=',pt1,'pt2=',pt2,'door=',door)
            if door:
                i -= 1
                break
            pt1 = pt2
        new_path = path[0:i+1]
        step1 = NavStep(NavStep.DRIVE, new_path)
        steps.append(step1)
        if door:
            step2 = NavStep(NavStep.DOORPASS, door)
            steps.append(step2)
        plan = NavPlan(steps)
        return plan

    def __repr__(self):
        steps = [(('doorpass(%s)' % s.param.id) if s.type == NavStep.DOORPASS else s.type) for s in self.steps]
        return '<NavPlan %s>' % repr(steps)

#---------------- PilotToPose ----------------

class PilotToPose(StateNode):
    def __init__(self, target_pose=None, verbose=False, max_iter=RRT.DEFAULT_MAX_ITER, use_wf=False):
        super().__init__()
        self.target_pose = target_pose
        self.verbose = verbose
        self.max_iter = max_iter
        self.use_wf = use_wf

    def start(self, event=None):
        self.robot.world.rrt.max_iter = self.max_iter
        super().start(self)

    class PilotPlanner(StateNode):
        def planner(self,start_node,goal_node):
            return self.robot.world.rrt.plan_path(start_node,goal_node, use_wf=self.parent.use_wf)

        def start(self,event=None):
            super().start(event)
            tpose = self.parent.target_pose
            if tpose is None or (tpose.position.x == 0 and tpose.position.y == 0 and
                                 tpose.rotation.angle_z.radians == 0 and not tpose.is_valid):
                print("Pilot: target pose is invalid: %s" % tpose)
                self.parent.post_event(PilotEvent(InvalidPose, tpose))
                self.parent.post_failure()
                return
            (pose_x, pose_y, pose_theta) = self.robot.world.particle_filter.pose
            start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)
            goal_node = RRTNode(x=tpose.position.x, y=tpose.position.y,
                                q=tpose.rotation.angle_z.radians)

            if self.robot.world.path_viewer:
                self.robot.world.path_viewer.clear()

            start_escape_move = None
            try:
                (treeA, treeB, path) = self.planner(start_node, goal_node)

            except StartCollides as e:
                # See if we can escape the start collision using canned headings.
                # This could be made more sophisticated, e.g., using arcs.
                #print('planner',e,'start',start_node)
                escape_distance = 25 # mm
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
                    self.parent.post_event(PilotEvent(StartCollides, e.args))
                    self.parent.post_failure()
                    return
                #print('escape=', start_escape_move)
                try:
                    (treeA, treeB, path) = self.planner(start_node, goal_node)
                except GoalCollides as e:
                    print('PilotPlanner: Goal collides!',e)
                    self.parent.post_event(PilotEvent(GoalCollides, e.args))
                    self.parent.post_failure()
                    return
                except MaxIterations as e:
                    print('PilotPlanner: Max iterations %d exceeded!' % e.args[0])
                    self.parent.post_event(PilotEvent(MaxIterations, e.args))
                    self.parent.post_failure()
                    return
                #print('replan',path)

            except GoalCollides as e:
                print('PilotPlanner: Goal collides!',e)
                self.parent.post_event(PilotEvent(GoalCollides, e.args))
                self.parent.post_failure()
                return
            except MaxIterations as e:
                print('PilotPlanner: Max iterations %d exceeded!' % e.args[0])
                self.parent.post_event(PilotEvent(MaxIterations, e.args))
                self.parent.post_failure()
                return

            if self.parent.verbose:
                print('Path planner generated',len(treeA)+len(treeB),'nodes.')
            if self.parent.robot.world.path_viewer:
                self.parent.robot.world.path_viewer.clear()
                self.parent.robot.world.path_viewer.add_tree(path, (1,0,0,0.75))

            # Construct the nav plan
            if self.parent.verbose:
                [print(' ',x) for x in path]
            cpath = [(node.x,node.y) for node in path]
            if self.parent.verbose:
                print('cpath =', cpath)

            doors = self.generate_doorway_list()
            navplan = NavPlan.from_path(cpath, doors)

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

            #print('finalnavplan steps:', navplan.steps)

            # If no doorpass, we're good to go
            last_step = navplan.steps[-1]
            if last_step.type != NavStep.DOORPASS:
                self.post_data(navplan)
                return

            # We planned for a doorpass; replan to the outer gate
            door = last_step.param
            gate = DoorPass.calculate_gate(self.robot, door, DoorPass.OUTER_GATE_DISTANCE)
            goal_node = RRTNode(x=gate[0], y=gate[1], q=gate[2])
            print('new goal is', goal_node)
            try:
                (_, _, path) = self.planner(start_node, goal_node)
            except Exception as e:
                print('Pilot replanning for door gateway failed!', e.args)
            cpath = [(node.x,node.y) for node in path]
            navplan = NavPlan.from_path(cpath, [])
            navplan.steps.append(last_step)  # Add the doorpass step
            self.post_data(navplan)

        def generate_doorway_list(self):
            doorways = []
            for (key,obj) in self.robot.world.world_map.objects.items():
                if isinstance(obj,DoorwayObj):
                    w = obj.door_width
                    dx = w * sin(obj.theta)
                    dy = w * cos(obj.theta)
                    doorways.append((obj, ((obj.x-dx, obj.y-dy), (obj.x+dx, obj.y+dy))))
            return doorways

        # ----- End of PilotPlanner -----

    class PilotExecutePlan(StateNode):
        def start(self, event=None):
            if not isinstance(event, DataEvent) and isinstance(event.data, NavPlan):
                raise ValueError(event)
            self.navplan = event.data
            self.index = 0
            super().start(event)

        class DispatchStep(StateNode):
            def start(self, event=None):
                super().start(event)
                step = self.parent.navplan.steps[self.parent.index]
                print('nav step', step)
                self.post_event(DataEvent(self,step.type))

        class ExecuteDrive(DriveContinuous):
            def start(self, event=None):
                step = self.parent.navplan.steps[self.parent.index]
                super().start(DataEvent(None,step.param))

        class ExecuteDoorPass(DoorPass):
            def start(self, event=None):
                step = self.parent.navplan.steps[self.parent.index]
                super().start(DataEvent(None,step.param))

        class ExecuteBackup(Forward):
            def start(self, event=None):
                step = self.parent.navplan.steps[self.parent.index]
                dx = step.param[0] - self.robot.world.particle_filter.pose[0]
                dy = step.param[1] - self.robot.world.particle_filter.pose[1]
                self.distance = distance_mm(- sqrt(dx*dx + dy*dy))
                super().start(event)

        class NextStep(StateNode):
            def start(self, event=None):
                super().start(event)
                self.parent.index += 1
                if self.parent.index < len(self.parent.navplan.steps):
                    self.post_success()
                else:
                    self.post_completion()

        def setup(self):
            """
                dispatch: self.DispatchStep()
                dispatch =D(NavStep.DRIVE)=> drive
                dispatch =D(NavStep.DOORPASS)=> doorpass
                dispatch =D(NavStep.BACKUP)=> backup
    
                drive: self.ExecuteDrive()
                drive =C=> next
                drive =F=> ParentFails()
    
                doorpass: self.ExecuteDoorPass()
                doorpass =C=> next
                doorpass =F=> ParentFails()
    
                backup: self.ExecuteBackup()
                backup =C=> next
                backup =F=> ParentFails()
    
                next: self.NextStep()
                next =S=> dispatch
                next =C=> ParentCompletes()
            """
            
            # Code generated by genfsm on Tue Jul 23 02:36:50 2019:
            
            dispatch = self.DispatchStep() .set_name("dispatch") .set_parent(self)
            drive = self.ExecuteDrive() .set_name("drive") .set_parent(self)
            parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
            doorpass = self.ExecuteDoorPass() .set_name("doorpass") .set_parent(self)
            parentfails2 = ParentFails() .set_name("parentfails2") .set_parent(self)
            backup = self.ExecuteBackup() .set_name("backup") .set_parent(self)
            parentfails3 = ParentFails() .set_name("parentfails3") .set_parent(self)
            next = self.NextStep() .set_name("next") .set_parent(self)
            parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
            
            datatrans1 = DataTrans(NavStep.DRIVE) .set_name("datatrans1")
            datatrans1 .add_sources(dispatch) .add_destinations(drive)
            
            datatrans2 = DataTrans(NavStep.DOORPASS) .set_name("datatrans2")
            datatrans2 .add_sources(dispatch) .add_destinations(doorpass)
            
            datatrans3 = DataTrans(NavStep.BACKUP) .set_name("datatrans3")
            datatrans3 .add_sources(dispatch) .add_destinations(backup)
            
            completiontrans1 = CompletionTrans() .set_name("completiontrans1")
            completiontrans1 .add_sources(drive) .add_destinations(next)
            
            failuretrans1 = FailureTrans() .set_name("failuretrans1")
            failuretrans1 .add_sources(drive) .add_destinations(parentfails1)
            
            completiontrans2 = CompletionTrans() .set_name("completiontrans2")
            completiontrans2 .add_sources(doorpass) .add_destinations(next)
            
            failuretrans2 = FailureTrans() .set_name("failuretrans2")
            failuretrans2 .add_sources(doorpass) .add_destinations(parentfails2)
            
            completiontrans3 = CompletionTrans() .set_name("completiontrans3")
            completiontrans3 .add_sources(backup) .add_destinations(next)
            
            failuretrans3 = FailureTrans() .set_name("failuretrans3")
            failuretrans3 .add_sources(backup) .add_destinations(parentfails3)
            
            successtrans1 = SuccessTrans() .set_name("successtrans1")
            successtrans1 .add_sources(next) .add_destinations(dispatch)
            
            completiontrans4 = CompletionTrans() .set_name("completiontrans4")
            completiontrans4 .add_sources(next) .add_destinations(parentcompletes1)
            
            return self

        # End of PilotExecutePlan

    class CheckArrival(StateNode):
        def start(self, event=None):
            super().start(event)
            pf_pose = self.robot.world.particle_filter.pose
            current_pose = Pose(pf_pose[0], pf_pose[1], 0, angle_z=radians(pf_pose[2]))
            pose_diff = current_pose - self.parent.target_pose
            distance = (pose_diff.position.x**2 + pose_diff.position.y**2) ** 0.5
            MAX_TARGET_DISTANCE = 50.0 # mm
            if distance <= MAX_TARGET_DISTANCE:
                self.post_success()
            else:
                self.post_failure()


    def setup(self):
        """
            planner: self.PilotPlanner() =D=> exec
    
            exec: self.PilotExecutePlan()
            exec =C=> check
            exec =F=> ParentFails()
    
            check: self.CheckArrival()
            check =S=> ParentCompletes()
            check =F=> planner
        """
        
        # Code generated by genfsm on Tue Jul 23 02:36:50 2019:
        
        planner = self.PilotPlanner() .set_name("planner") .set_parent(self)
        exec = self.PilotExecutePlan() .set_name("exec") .set_parent(self)
        parentfails4 = ParentFails() .set_name("parentfails4") .set_parent(self)
        check = self.CheckArrival() .set_name("check") .set_parent(self)
        parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
        
        datatrans4 = DataTrans() .set_name("datatrans4")
        datatrans4 .add_sources(planner) .add_destinations(exec)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(exec) .add_destinations(check)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(exec) .add_destinations(parentfails4)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(check) .add_destinations(parentcompletes2)
        
        failuretrans5 = FailureTrans() .set_name("failuretrans5")
        failuretrans5 .add_sources(check) .add_destinations(planner)
        
        return self


class PilotPushToPose(PilotToPose):
    def __init__(self,pose):
        super().__init__(pose)
        self.max_turn = 20*(pi/180)

    def planner(self,start_node,goal_node):
        self.robot.world.rrt.step_size=20
        return self.robot.world.rrt.plan_push_chip(start_node,goal_node)


"""
class NavStep():
    FORWARD = "forward"
    BACKWARD = "backward"
    HEADING = "heading"
    ARC = "arc"

    def __init__(self, type, params):
        self.type = type
        self.params = params

    def __repr__(self):
        return '<NavStep %s %.1f,%.1f @ %d deg.>' % \
               (self.type, *self.params[0:2], round(self.params[2]*180/pi))

class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    @staticmethod
    def from_path(path):
        steps = []
        last_node = path[0]
        for node in path:
            if (not node.radius) or (node.radius == 0):
                dist = sqrt((node.x-last_node.x)**2 + (node.y-last_node.y)**2)
                max_step = 50 # mm
                for d in range(max_step,math.ceil(dist),max_step):
                    steps.append(NavStep(NavStep.FORWARD,
                                         (last_node.x + d*cos(node.q),
                                          last_node.y + d*sin(node.q),
                                          node.q)))
                steps.append(NavStep(NavStep.FORWARD,
                                     (node.x, node.y, node.q)))
            else:
                steps.append(NavStep(NavStep.ARC,
                                     (node.x, node.y, node.q, node.radius)))
            last_node = node
        if path[-1].radius == 0:
            steps[-1].type = NavStep.HEADING
        return NavPlan(steps)

class PilotBase(StateNode):
    def __init__(self, verbose=False):
        super().__init__()
        self.verbose = verbose
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
            if self.verbose:
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
        if self.verbose:
            print('drive_arc angle=',angle,'deg.,  traveled=',traveled,'deg.')

class PilotToPoseOld(PilotBase):
    def __init__(self, target_pose=None, verbose=False):
        super().__init__(verbose)
        self.target_pose = target_pose

    def planner(self,start_node,goal_node):
        return self.robot.world.rrt.plan_path(start_node,goal_node)

    def start(self,event=None):
        super().start(event)
        if self.target_pose is None:
            self.post_failure()
            return
        (pose_x, pose_y, pose_theta) = self.robot.world.particle_filter.pose
        start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)
        tpose = self.target_pose
        goal_node = RRTNode(x=tpose.position.x, y=tpose.position.y,
                            q=tpose.rotation.angle_z.radians)

        if self.robot.world.path_viewer:
            self.robot.world.path_viewer.clear()
        try:
            (treeA, treeB, path) = self.planner(start_node, goal_node)
        except StartCollides as e:
            print('Start collides!',e)
            self.post_event(PilotEvent(StartCollides, e.args))
            self.post_failure()
            return
        except GoalCollides as e:
            print('Goal collides!',e)
            self.post_event(PilotEvent(GoalCollides, e.args))
            self.post_failure()
            return
        except MaxIterations as e:
            print('Max iterations %d exceeded!' % e.args[0])
            self.post_event(PilotEvent(MaxIterations, e.args))
            self.post_failure()
            return

        if self.verbose:
            print(len(treeA)+len(treeB),'nodes')
        if self.robot.world.path_viewer:
            self.robot.world.path_viewer.add_tree(path, (1,0,0,0.75))

        # Construct and execute nav plan
        if self.verbose:
            [print(x) for x in path]
        self.plan = NavPlan.from_path(path)
        if self.verbose:
            print('Navigation Plan:')
            [print(y) for y in self.plan.steps]
        self.robot.loop.create_task(self.execute_plan())

    async def execute_plan(self):
        print('-------- Executing Nav Plan --------')
        for step in self.plan.steps[1:]:
            if not self.running: return
            self.robot.world.particle_filter.variance_estimate()
            (cur_x,cur_y,cur_hdg) = self.robot.world.particle_filter.pose
            if step.type == NavStep.HEADING:
                (targ_x, targ_y, targ_hdg) = step.params
                # Equation of the line y=ax+c through the target pose
                a = min(1000, max(-1000, math.tan(targ_hdg)))
                c = targ_y - a * targ_x
                # Equation of the line y=bx+d through the present pose
                b = min(1000, max(-1000, math.tan(cur_hdg)))
                d = cur_y - b * cur_x
                # Intersection point
                int_x = (d-c) / (a-b) if abs(a-b) > 1e-5 else math.nan
                int_y = a * int_x + c
                dx = int_x - cur_x
                dy = int_y - cur_y
                dist = sqrt(dx*dx + dy*dy)
                if abs(wrap_angle(atan2(dy,dx) - cur_hdg)) > pi/2:
                    dist = - dist
                dist += -center_of_rotation_offset
                if self.verbose:
                    print('PRE-TURN: cur=(%.1f,%.1f) @ %.1f deg.,  int=(%.1f, %.1f)  dist=%.1f' %
                          (cur_x, cur_y, cur_hdg*180/pi, int_x, int_y, dist))
                if abs(dist) < 2:
                    if self.verbose:
                        print('  ** SKIPPED **')
                else:
                    await self.robot.drive_straight(distance_mm(dist),
                                                    speed_mmps(50)).wait_for_completed()
                (cur_x,cur_y,cur_hdg) = self.robot.world.particle_filter.pose
                turn_angle = wrap_angle(targ_hdg - cur_hdg)
                if self.verbose:
                    print('TURN: cur=(%.1f,%.1f) @ %.1f deg.,  targ=(%.1f,%.1f) @ %.1f deg, turn_angle=%.1f deg.' %
                          (cur_x, cur_y, cur_hdg*180/pi,
                           targ_x, targ_y, targ_hdg*180/pi, turn_angle*180/pi))
                await self.robot.turn_in_place(cozmo.util.radians(turn_angle)).wait_for_completed()
                continue
            elif step.type == NavStep.FORWARD:
                (targ_x, targ_y, targ_hdg) = step.params
                dx = targ_x - cur_x
                dy = targ_y - cur_y
                course = atan2(dy,dx)
                turn_angle = wrap_angle(course - cur_hdg)
                if self.verbose:
                    print('FWD: cur=(%.1f,%.1f)@%.1f\N{degree sign} targ=(%.1f,%.1f)@%.1f\N{degree sign} turn=%.1f\N{degree sign}' %
                          (cur_x,cur_y,cur_hdg*180/pi,
                           targ_x,targ_y,targ_hdg*180/pi,turn_angle*180/pi),
                          end='')
                    sys.stdout.flush()
                if abs(turn_angle) > self.max_turn:
                    turn_angle = self.max_turn if turn_angle > 0 else -self.max_turn
                    if self.verbose:
                        print('  ** TURN ANGLE SET TO', turn_angle*180/pi)
                # *** HACK: skip node if it requires unreasonable turn
                if abs(turn_angle) < 2*pi/180 or abs(wrap_angle(course-targ_hdg)) > pi/2:
                    if self.verbose:
                        print('  ** SKIPPED TURN **')
                else:
                    await self.robot.turn_in_place(cozmo.util.radians(turn_angle)).wait_for_completed()
                if not self.running: return
                (cur_x,cur_y,cur_hdg) = self.robot.world.particle_filter.pose
                dx = targ_x - cur_x
                dy = targ_y - cur_y
                dist = sqrt(dx**2 + dy**2)
                if self.verbose:
                    print(' dist=%.1f' % dist)
                await self.robot.drive_straight(distance_mm(dist),
                                                speed_mmps(50)).wait_for_completed()
            elif step.type == NavStep.ARC:
                (targ_x, targ_y, targ_hdg, radius) = step.params
                if self.verbose:
                    print('ARC: cur=(%.1f,%.1f) @ %.1f deg.,  targ=(%.1f,%.1f), targ_hdg=%.1f deg., radius=%.1f' %
                          (cur_x,cur_y,cur_hdg*180/pi,targ_x,targ_y,targ_hdg*180/pi,radius))
                (actual_radius, actual_angle) = \
                                self.calculate_arc(cur_x, cur_y, cur_hdg, targ_x, targ_y)
                if self.verbose:
                    print(' ** actual_radius =', actual_radius, '  actual_angle=', actual_angle*180/pi)
                await self.drive_arc(actual_radius, math.degrees(abs(actual_angle)))
            else:
                raise ValueError('Invalid NavStep',step)
        if self.verbose:
            print('done executing')
        self.post_completion()
"""
