from cozmo.util import Pose
from cozmo.objects import LightCube

from .nodes import *
from .transitions import *
from .geometry import wrap_angle, line_equation, line_intersection
from .geometry import ORIENTATION_UPRIGHT, ORIENTATION_INVERTED, ORIENTATION_SIDEWAYS, ORIENTATION_TILTED
from .geometry import get_orientation_state, same_orientation, get_pattern_number
from .cozmo_kin import center_of_rotation_offset
from .pilot import PilotToPose, PilotCheckStart, ParentPilotEvent, InvalidPose
from .rrt import StartCollides, GoalCollides, MaxIterations
from .worldmap import LightCubeObj
from .trace import tracefsm

from math import sin, cos, atan2, pi, sqrt
import copy


class GoToCube(StateNode):
    def __init__(self, cube=None):
        self.object = cube
        super().__init__()
        self.side_index = 0
        self.try_number = 0
        self.try_max = 3
        self.roll_cube = False
        self.roll_target = None

    def start(self, event=None):
        # self.object will normally be set up by the parent of this node
        if isinstance(self.object, LightCube):
            self.wmobject = self.object.wm_obj
        elif isinstance(self.object, LightCubeObj):
            self.wmobject = self.object
            self.object = self.object.sdk_obj
        else:
            raise ValueError(self.object)
        self.children['looker'].object = self.object
        self.sides = []
        self.side = None
        self.side_index = 0
        self.try_number = 0
        self.try_max = 3
        super().start(event)
        if self.wmobject.pose_confidence < 0:
            print('GoToCube: cube has invalid pose!', self.wmobject, self.object.pose)
            self.post_event(PilotEvent(InvalidPose))
            self.post_failure()

    def pick_side(self, offset, use_world_map):
        # If use_world_map is True, use the LightCubeObj pose.
        # If False, use the LightCube's pose as determined by vision.
        cube = self.object
        if use_world_map:
            wmobj = self.object.wm_obj
            x = wmobj.x
            y = wmobj.y
            ang = wmobj.theta
            orientation = wmobj.orientation
            print('Pick side from', self.object.wm_obj)
        else:
            x = cube.pose.position.x
            y = cube.pose.position.y
            orientation, _, _, z = get_orientation_state(cube.pose.rotation.q0_q1_q2_q3)
            if orientation == ORIENTATION_SIDEWAYS:
                ang = wrap_angle(z)
            else:
                ang = cube.pose.rotation.angle_z.radians
        (rx, ry, rtheta) = self.get_robot_cor(use_world_map=use_world_map)
        dist = LightCubeObj.light_cube_size[0]/2 + offset
        side1 = [ (x + cos(ang)*dist), (y + sin(ang)*dist), wrap_angle(ang + pi)   ]
        side2 = [ (x - cos(ang)*dist), (y - sin(ang)*dist), wrap_angle(ang)    ]
        side3 = [ (x + sin(ang)*dist), (y - cos(ang)*dist), wrap_angle(ang + pi/2) ]
        side4 = [ (x - sin(ang)*dist), (y + cos(ang)*dist), wrap_angle(ang - pi/2) ]
        # sides = (side1, side2, side3, side4)
        sides = (side2, side3, side1, side4)

        if orientation == ORIENTATION_SIDEWAYS:
            if self.roll_cube:
                # if the orientation is sideways, only two valid sides to roll cube to upright/inverted
                if self.roll_target == ORIENTATION_UPRIGHT or self.roll_target == ORIENTATION_INVERTED:
                    self.try_max = 1
                    sorted_sides = (side2, side1) if self.roll_target == ORIENTATION_UPRIGHT else (side1, side2)
                    self.side_index = self.side_index%2
                else:
                    sides12 = sorted((side1, side2), key=lambda pt: ((pt[0]-rx)**2 + (pt[1]-ry)**2))
                    sides34 = sorted((side3, side4), key=lambda pt: ((pt[0]-rx)**2 + (pt[1]-ry)**2))
                    sorted_sides = sides12 + sides34
            else:
                # if the orientation is sideways, only one valid side to pickup cube
                self.try_max = 0
                side = side2
                d = sqrt((side[0]-rx)**2 + (side[1]-ry)**2)
                print('  side 0: %5.1f mm   %5.1f , %5.1f @ %5.1f deg.' %
                      (d, side[0], side[1], side[2]*180/pi))
                return side
        else:
            sorted_sides = sorted(sides, key=lambda pt: ((pt[0]-rx)**2 + (pt[1]-ry)**2))

        for i in range(len(sorted_sides)):
            side = sorted_sides[i]
            d = sqrt((side[0]-rx)**2 + (side[1]-ry)**2)
            print('  side %d: %5.1f mm   %5.1f , %5.1f @ %5.1f deg.' %
                  (i, d, side[0], side[1], side[2]*180/pi))
        print('Go to side %i' % self.side_index)
        return sorted_sides[self.side_index]

    def pick_another_side(self):
        # put the code from ChooseAnotherSide here
        pass
    

    def get_robot_pose(self, use_world_map):
        if use_world_map:
            rx = self.robot.world.particle_filter.pose[0]
            ry = self.robot.world.particle_filter.pose[1]
            rtheta = self.robot.world.particle_filter.pose[2]
        else:
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            rtheta = self.robot.pose.rotation.angle_z.radians
        return (rx, ry, rtheta)

    def get_robot_cor(self, use_world_map):
        "Get robot center of rotation and current heading"
        (rx, ry, rtheta) = self.get_robot_pose(use_world_map=use_world_map)
        cx = rx + center_of_rotation_offset*cos(rtheta)
        cy = ry + center_of_rotation_offset*sin(rtheta)
        return (cx, cy, rtheta)

    def get_robot_line(self, use_world_map):
        (rx, ry, rtheta) = self.get_robot_pose(use_world_map)
        (cx, cy, ctheta) = self.get_robot_cor(use_world_map)
        return line_equation((rx,ry), (cx,cy))

    def get_cube_line(self, use_world_map):
        if use_world_map:
            ox = self.parent.wmobject.x
            oy = self.parent.wmobject.y
        else:
            ox = self.parent.object.pose.position.x
            oy = self.parent.object.pose.position.y
        (sx, sy, stheta) = self.side
        return line_equation((ox,oy), (sx,sy))

    def measure_dockedness(self, side, use_world_map):
        """Returns distance and relative angle to specified docking pose."""
        (rx, ry, rtheta) = self.get_robot_cor(use_world_map)
        (ox, oy, otheta) = side
        dist = sqrt((rx-ox)**2 + (ry-oy)**2)
        relative_angle = abs(wrap_angle(rtheta-otheta) % (pi/2)) * (180/pi)
        return (dist, relative_angle)

    class PilotToSide(PilotToPose):
        def __init__(self):
            super().__init__(None, verbose=True)

        def start(self, event=None):
            cube = self.parent.object
            (x, y, theta) = self.parent.pick_side(100, use_world_map=True)
            self.target_pose = Pose(x, y, self.robot.pose.position.z,
                                    angle_z=Angle(radians=wrap_angle(theta)))
            (px,py,pq) = self.robot.world.particle_filter.pose
            print('PilotToSide: planned path from (%.1f, %.1f) @ %.1f deg. to pickup point (%.1f, %.1f) @ %.1f deg.' %
                  (px, py, pq*180/pi,
                   self.target_pose.position.x, self.target_pose.position.y,
                   self.target_pose.rotation.angle_z.degrees))
            super().start(event)


    class ReportPosition(StateNode):
        def __init__(self,id=None):
            super().__init__()
            self.id_string = id + ': ' if id else ''

        def start(self,event=None):
            super().start(event)
            cube = self.parent.object
            vis = 'visible' if cube.is_visible else 'not visible'
            cx = cube.pose.position.x
            cy = cube.pose.position.y
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            dx = cx - rx
            dy = cy - ry
            dist = sqrt(dx*dx + dy*dy)
            bearing = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) * 180/pi
            print('%scube %s at (%5.1f,%5.1f)  robot at (%5.1f,%5.1f)  dist=%5.1f  rel. brg=%5.1f' %
                  (self.id_string, vis, cx, cy, rx, ry, dist, bearing))


    class TurnToCube(Turn):
        def __init__(self, check_vis=False):
            self.check_vis = check_vis
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            if self.check_vis and not cube.is_visible:
                print('** TurnToCube %s could not see the cube.' % self.name)
                self.angle = None
                super().start(event)
                self.post_failure()
                print('TurnToCube %s posted failure' % self.name)
            else:
                (sx, sy, _) = self.parent.pick_side(0, use_world_map=False)
                (cx, cy, ctheta) = self.parent.get_robot_cor(False)
                dx = sx - cx
                dy = sy - cy
                dist = sqrt(dx*dx + dy*dy)
                self.angle = Angle(degrees = wrap_angle(atan2(dy,dx) - ctheta) * 180/pi)
                if abs(self.angle.degrees) <= 2:
                    self.angle = degrees(0)
                if abs(self.angle.degrees) > 60:
                    print('********>> BIG TURN in',self)
                print('TurnToCube %s: cube at (%5.1f,%5.1f)  robot cor at (%5.1f,%5.1f)  dist=%5.1f  turn angle=%5.1f' %
                      (self.name, sx, sy, cx, cy, dist, self.angle.degrees))
                super().start(event)


    class CheckAlmostDocked(StateNode):
        # *** TODO: convert to iterate through all feasible sides
        def start(self, event=None):
            if self.running: return
            super().start(event)
            cube = self.parent.object
            side = self.parent.pick_side(0, use_world_map=True)
            self.parent.side = side
            (dist, relative_angle) = self.parent.measure_dockedness(side,True)
            max_distance_from_dock_point = 150 # millimeters
            max_angle_from_dock_heading = 10 # degrees
            max_angle_for_sideways = 135 # degrees

            if isinstance(cube, LightCube):
                orientation = cube.wm_obj.orientation
            elif isinstance(cube, LightCubeObj):
                orientation = cube.sdk_obj.orientation

            # Re-calculate relative_angle for sideways cube
            if orientation == ORIENTATION_SIDEWAYS:
                (rx, ry, rtheta) = self.parent.get_robot_cor(True)
                ctheta = side[2]
                dtheta = (rtheta - ctheta) if (rtheta>ctheta) else (ctheta-rtheta)
                relative_angle = abs(dtheta/pi*180)
                # print('robot: %.1f deg; cube: %.1f deg; delta: %.1f deg.' %(rtheta/pi*180, ctheta/pi*180, relative_angle))

            if dist < max_distance_from_dock_point:
                if relative_angle < max_angle_from_dock_heading and cube.is_visible:
                    print('CheckAlmostDocked is satisfied.  dist=%.1f mm  angle=%.1f deg.' %
                          (dist, relative_angle))
                    self.post_completion()
                else:
                    if orientation == ORIENTATION_SIDEWAYS:
                        if relative_angle < max_angle_for_sideways and cube.is_visible:
                            print('CheckAlmostDocked: bad angle.  (dist=%.1f mm)  angle=%.1f deg.' %
                                  (dist, relative_angle))
                            self.post_success()
                        else:
                            print('CheckAlmostDocked: use the Pilot to path plan. orientation=%s. dist=%.1f mm.  angle=%.1f deg.' %
                                  (orientation, dist, relative_angle))
                            self.post_failure()
                    elif not cube.is_visible:
                        print('CheckAlmostDocked: cube not visible')
                        self.post_success()
                    else:
                        print('CheckAlmostDocked: bad angle.  (dist=%.1f mm)  angle=%.1f deg.' %
                              (dist, relative_angle))
                        self.post_success()
            else:
                print('CheckAlmostDocked: too far away.  dist=%.1f mm.  (angle=%.1f deg.)' %
                      (dist, relative_angle))
                self.post_failure()


    class ForwardToCube(Forward):
        def __init__(self, offset):
            self.offset = offset
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            dx = cube.pose.position.x - self.robot.pose.position.x
            dy = cube.pose.position.y - self.robot.pose.position.y
            dist = sqrt(dx*dx + dy*dy) - self.offset
            if (dist < 0):
                print('***** ForwardToCube %s negative distance: %.1f mm' % (self.name,dist))
            self.distance = Distance(dist)
            print('ForwardToCube %s: distance %.1f mm' % (self.name, self.distance.distance_mm))
            super().start(event)


    class ManualDock1(Forward):
        def report(self,rx,ry,rtheta,sx,sy,stheta,intx,inty,int_brg):
            print('ManualDock1: robot cor at %.1f , %.1f @ %.1f deg.  side at %.1f , %.1f @ %.1f deg.' %
                  (rx, ry, 180*rtheta/pi, sx, sy, stheta*180/pi))
            print('  int at %.1f , %.1f   bearing=%.1f deg.  dist=%.1f mm ' %
                  (intx,inty,int_brg*180/pi,self.distance.distance_mm))

        def start(self,event=None):
            rline = self.parent.get_robot_line(use_world_map=True)
            cline = self.parent.get_cube_line(use_world_map=True)
            (intx, inty) = line_intersection(rline, cline)
            (rx, ry,rtheta) = self.parent.get_robot_cor(use_world_map=True)
            # Is intersection point ahead of or behind us?
            intersection_bearing = wrap_angle(atan2(inty-ry, intx-rx)-rtheta)
            (sx, sy, stheta) = self.parent.side
            if abs(intersection_bearing) > pi/2:  # Intersection is behind us
                print('ManualDock1: Intersection is behind us.')
                dist = min(75, sqrt((rx-intx)**2 + (ry-inty)**2))
                self.distance = distance_mm(-dist)
                self.report(rx,ry,rtheta,sx,sy,stheta,intx,inty,intersection_bearing)
                super().start(event)
                return
            else:  # Intersection is ahead of us
                dx = sx - intx
                dy = sy - inty
                dtheta = abs(wrap_angle(atan2(dy,dx) - stheta))
                dist_to_side = sqrt(dx**2 + dy**2)
                min_dist_to_side = 60 # mm from cor
                max_dist_to_side = 120 # mm from cor
                print('ManualDock1: Intersection ahead is %.1f mm from side and dtheta=%.1f deg.' %
                      (dist_to_side, dtheta*180/pi))
                alignment_threshold = 5 # degrees
                aligned = abs(wrap_angle(rtheta-stheta)) < alignment_threshold*pi/180
                if ((dist_to_side >= min_dist_to_side) or aligned) and \
                   (dist_to_side <= max_dist_to_side) and \
                   (dtheta < pi/20):   # make sure intersection is on near side of cube
                    # Intersection ahead is at an acceptable distance from the chosen side
                    print('ManualDock1: move forward to intersection.')
                    self.distance = distance_mm(sqrt((rx-intx)**2 + (ry-inty)**2))
                    self.report(rx,ry,rtheta,sx,sy,stheta,intx,inty,intersection_bearing)
                    super().start(event)
                    return
                else:
                    # Intersection ahead is past the target, or too close or too far from it, so
                    # pick a new point on cline at a reasonable distance and turn to that
                    print('ManualDock: pick new intersection point')
                    good_dist = 70 # mmm from cor
                    tx = sx + good_dist * cos(stheta+pi)
                    ty = sy + good_dist * sin(stheta+pi)
                    turn_angle = wrap_angle(atan2(ty-ry,tx-rx)-rtheta)
                    min_turn_angle = 2 * pi/180
                    if abs(turn_angle) > min_turn_angle:
                        self.distance = distance_mm(0)
                        self.report(rx,ry,rtheta,sx,sy,stheta,intx,inty,intersection_bearing)
                        print('ManualDock1: turn to point at %.1f , %.1f   turn_angle=%.1f deg.' %
                              (tx, ty, turn_angle*180/pi))
                        super().start(event)
                        self.post_data(Angle(radians=turn_angle))
                        return
                    else:
                        dist = sqrt((rx-tx)**2 + (ry-ty)**2)
                        self.distance = distance_mm(dist)
                        self.report(rx,ry,rtheta,sx,sy,stheta,intx,inty,intersection_bearing)
                        print('ManualDock1: Alignment is close enough.')
                        super().start(event)
                        return

    class ManualDock2(Turn):
        def start(self,event=None):
            (rx,ry,rtheta) = self.parent.get_robot_cor(use_world_map=True)
            (ox,oy,otheta) = self.parent.side
            #bearing = atan2(oy-ry, ox-rx)
            #turn_angle = wrap_angle(bearing-rtheta)
            turn_angle = wrap_angle(otheta-rtheta)
            self.angle = Angle(radians=turn_angle)
            print('ManualDock2: otheta=%.1f deg.  heading=%.1f deg  turn_angle=%.1f deg.' %
                  (otheta*180/pi, rtheta*180/pi, turn_angle*180/pi))
            super().start(event)


    class InvalidatePose(StateNode):
        def start(self,event=None):
            super().start(event)
            self.parent.wmobject.pose_confidence = -1


    class ChooseAnotherSide(StateNode):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            super().start(event)
            # Replace the code below with:
            #  if self.parent.pick_another_side():
            #     self.post_success();
            #  else:
            #     self.post_failure()
            if self.parent.try_number >= self.parent.try_max:
                # Have tried all sides, reset counter
                print('Have tried all sides.')
                self.parent.try_number = 0
                self.parent.side_index = 0
                self.parent.try_max = 3
                self.post_failure()
            else:
                self.parent.try_number += 1
                self.parent.side_index = (self.parent.side_index + 1) % 4
                print('Haven\'t tried out all sides of cube. Going to try the side', self.parent.side_index)
                self.post_success()


    class CheckCubePoseValid(StateNode):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            super().start(event)

            if isinstance(self.parent.object, LightCube):
                cube_id = self.parent.object.wm_obj.id
            elif isinstance(self.parent.object, LightCubeObj):
                cube_id = self.parent.object.id
            else:
                raise ValueError(self.parent.object)

            try:
                wmobject = self.robot.world.world_map.objects[cube_id]
            except: # worldmap was cleared just before we were called
                self.post_failure()
                return
            if self.parent.robot.is_picked_up:
                self.parent.robot.stop_all_motors()
                print('CheckCubePoseValid: robot is picked up.')
                self.post_failure()
            elif wmobject.pose_confidence < 0:
                print('CheckCubePoseValid: %s has invalid pose!' % cube_id)
                self.post_failure()
            else:
                self.post_completion()

    class ReadyToRoll(StateNode):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            super().start(event)
            if self.parent.roll_cube:
                print('get ready to roll')
                self.post_success()
            else:
                self.post_failure()


    def setup(self):
        """
            # GoToCube machine
    
            droplift: SetLiftHeight(0)
            droplift =C=> Print('droplift succeeded') =N=> {looker, waitlift, check_cube_pose_valid}
            droplift =F=> Print('droplift failed') =N=> {looker, waitlift, check_cube_pose_valid}   # lift motion fails if on charger
    
            looker: LookAtObject()
    
            waitlift: StateNode() =T(1)=>    # allow time for vision to set up world map
               check_almost_docked
    
            check_cube_pose_valid: self.CheckCubePoseValid()
            check_cube_pose_valid =C=> check_cube_pose_valid
            check_cube_pose_valid =F=> ParentFails()
    
            check_almost_docked: self.CheckAlmostDocked()   # sets up parent.side
            check_almost_docked =C=> turn_to_cube2       # we're good to dock right now
            check_almost_docked =S=> Forward(-10) =C=> manual_dock1   # we're close: skip the Pilot and set up dock manually
            check_almost_docked =F=> pilot_check_start   # not close: use the Pilot to path plan
    
            manual_dock1: self.ManualDock1()
            manual_dock1 =D=> Turn() =C=> Print('turned. wait...') =N=> manual_dock1
            manual_dock1 =C=> Print('wait...') =N=> manual_dock2
            manual_dock1 =T(10)=> Print('Cannot manual dock from here') =N=> pilot_check_start  # temporary
    
            manual_dock2: self.ManualDock2() =C=> Print('wait...') =N=> turn_to_cube1
    
            pilot_check_start: PilotCheckStart()
            pilot_check_start =S=> Print('Start collision check passed.') =N=> go_side
            # TODO: instead of blindly backing up, find the best direction to move.
            pilot_check_start =F=> Print('Backing up to escape start collision...') =N=>
               Forward(-80) =C=> StateNode() =T(0.5)=> pilot_check_start2
    
            # Second chance to avoid StartCollides.  There is no third chance.
            pilot_check_start2: PilotCheckStart()
            pilot_check_start2 =S=> Print('Start collision re-check passed.') =N=> go_side
            pilot_check_start2 =PILOT(StartCollides)=> check_start2_pilot: ParentPilotEvent()
            pilot_check_start2 =F=> failure
    
            go_side: self.PilotToSide()
            go_side =PILOT(GoalCollides)=> failure
            go_side =PILOT(MaxIterations)=> failure
            go_side =PILOT=> go_side_pilot: ParentPilotEvent()
            go_side =F=> failure
            go_side =C=> self.ReportPosition('go_side_deccel')
                =T(0.75)=> self.ReportPosition('go_side_stopped')
                =N=> turn_to_cube1
    
            turn_to_cube1: self.TurnToCube(check_vis=True) =C=>
                self.ReportPosition('turn_to_cube1_deccel')
                =T(0.75)=> self.ReportPosition('turn_to_cube1_stopped')
                =N=> Print('wait to approach...') =N=> approach
            turn_to_cube1 =F=> recover1
            turn_to_cube1 =T(5)=> recover1  # failsafe in case the failure event is missed??
    
            recover1: Forward(-50) =C=> StateNode() =T(1)=> turn_to_cube2
    
            approach: self.ForwardToCube(60) =C=> StateNode() =T(0.75)=>
                self.ReportPosition('approach') =T(0.75)=>
                self.ReportPosition('approach') =N=> Print('wait...') =N=>
                turn_to_cube_1a: self.TurnToCube(check_vis=False) =C=> Print('wait...') =N=>
                exit_to_roll
    
            exit_to_roll: self.ReadyToRoll()
            exit_to_roll =F=> forward_to_cube_1a
            exit_to_roll =S=> Forward(-5) =C=> SetLiftHeight(1) =C=> forward_to_cube_1a
    
            forward_to_cube_1a: self.ForwardToCube(15) =C=> success
    
            turn_to_cube2: self.TurnToCube(check_vis=True)
            turn_to_cube2 =F=> Print("TurnToCube2: Cube Lost") =N=> self.InvalidatePose() =N=> failure
            turn_to_cube2 =C=> forward_to_cube2
    
            forward_to_cube2: self.ForwardToCube(60) =C=> turn_to_cube3
    
            turn_to_cube3: self.TurnToCube(check_vis=False)   # can't fail
            turn_to_cube3 =C=> exit_to_roll2
    
            exit_to_roll2: self.ReadyToRoll()
            exit_to_roll2 =F=> forward_to_cube3
            exit_to_roll2 =S=> Forward(-5) =C=> SetLiftHeight(1) =C=> forward_to_cube3
    
            forward_to_cube3: self.ForwardToCube(20) =C=> success
    
            success: Print('GoToSide has succeeded.') =N=> ParentCompletes()
    
            failure: Print('GoToSide has failed.') =N=> check_cube_pose
    
            check_cube_pose: self.CheckCubePoseValid()
            check_cube_pose =C=> choose_another_side
            check_cube_pose =F=> ParentFails()
    
            choose_another_side: self.ChooseAnotherSide()
            choose_another_side =F=> ParentFails()
            choose_another_side =S=> droplift
        """
        
        # Code generated by genfsm on Fri Dec 20 18:08:35 2019:
        
        droplift = SetLiftHeight(0) .set_name("droplift") .set_parent(self)
        print1 = Print('droplift succeeded') .set_name("print1") .set_parent(self)
        print2 = Print('droplift failed') .set_name("print2") .set_parent(self)
        looker = LookAtObject() .set_name("looker") .set_parent(self)
        waitlift = StateNode() .set_name("waitlift") .set_parent(self)
        check_cube_pose_valid = self.CheckCubePoseValid() .set_name("check_cube_pose_valid") .set_parent(self)
        parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
        check_almost_docked = self.CheckAlmostDocked() .set_name("check_almost_docked") .set_parent(self)
        forward1 = Forward(-10) .set_name("forward1") .set_parent(self)
        manual_dock1 = self.ManualDock1() .set_name("manual_dock1") .set_parent(self)
        turn1 = Turn() .set_name("turn1") .set_parent(self)
        print3 = Print('turned. wait...') .set_name("print3") .set_parent(self)
        print4 = Print('wait...') .set_name("print4") .set_parent(self)
        print5 = Print('Cannot manual dock from here') .set_name("print5") .set_parent(self)
        manual_dock2 = self.ManualDock2() .set_name("manual_dock2") .set_parent(self)
        print6 = Print('wait...') .set_name("print6") .set_parent(self)
        pilot_check_start = PilotCheckStart() .set_name("pilot_check_start") .set_parent(self)
        print7 = Print('Start collision check passed.') .set_name("print7") .set_parent(self)
        print8 = Print('Backing up to escape start collision...') .set_name("print8") .set_parent(self)
        forward2 = Forward(-80) .set_name("forward2") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        pilot_check_start2 = PilotCheckStart() .set_name("pilot_check_start2") .set_parent(self)
        print9 = Print('Start collision re-check passed.') .set_name("print9") .set_parent(self)
        check_start2_pilot = ParentPilotEvent() .set_name("check_start2_pilot") .set_parent(self)
        go_side = self.PilotToSide() .set_name("go_side") .set_parent(self)
        go_side_pilot = ParentPilotEvent() .set_name("go_side_pilot") .set_parent(self)
        reportposition1 = self.ReportPosition('go_side_deccel') .set_name("reportposition1") .set_parent(self)
        reportposition2 = self.ReportPosition('go_side_stopped') .set_name("reportposition2") .set_parent(self)
        turn_to_cube1 = self.TurnToCube(check_vis=True) .set_name("turn_to_cube1") .set_parent(self)
        reportposition3 = self.ReportPosition('turn_to_cube1_deccel') .set_name("reportposition3") .set_parent(self)
        reportposition4 = self.ReportPosition('turn_to_cube1_stopped') .set_name("reportposition4") .set_parent(self)
        print10 = Print('wait to approach...') .set_name("print10") .set_parent(self)
        recover1 = Forward(-50) .set_name("recover1") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        approach = self.ForwardToCube(60) .set_name("approach") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        reportposition5 = self.ReportPosition('approach') .set_name("reportposition5") .set_parent(self)
        reportposition6 = self.ReportPosition('approach') .set_name("reportposition6") .set_parent(self)
        print11 = Print('wait...') .set_name("print11") .set_parent(self)
        turn_to_cube_1a = self.TurnToCube(check_vis=False) .set_name("turn_to_cube_1a") .set_parent(self)
        print12 = Print('wait...') .set_name("print12") .set_parent(self)
        exit_to_roll = self.ReadyToRoll() .set_name("exit_to_roll") .set_parent(self)
        forward3 = Forward(-5) .set_name("forward3") .set_parent(self)
        setliftheight1 = SetLiftHeight(1) .set_name("setliftheight1") .set_parent(self)
        forward_to_cube_1a = self.ForwardToCube(15) .set_name("forward_to_cube_1a") .set_parent(self)
        turn_to_cube2 = self.TurnToCube(check_vis=True) .set_name("turn_to_cube2") .set_parent(self)
        print13 = Print("TurnToCube2: Cube Lost") .set_name("print13") .set_parent(self)
        invalidatepose1 = self.InvalidatePose() .set_name("invalidatepose1") .set_parent(self)
        forward_to_cube2 = self.ForwardToCube(60) .set_name("forward_to_cube2") .set_parent(self)
        turn_to_cube3 = self.TurnToCube(check_vis=False) .set_name("turn_to_cube3") .set_parent(self)
        exit_to_roll2 = self.ReadyToRoll() .set_name("exit_to_roll2") .set_parent(self)
        forward4 = Forward(-5) .set_name("forward4") .set_parent(self)
        setliftheight2 = SetLiftHeight(1) .set_name("setliftheight2") .set_parent(self)
        forward_to_cube3 = self.ForwardToCube(20) .set_name("forward_to_cube3") .set_parent(self)
        success = Print('GoToSide has succeeded.') .set_name("success") .set_parent(self)
        parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
        failure = Print('GoToSide has failed.') .set_name("failure") .set_parent(self)
        check_cube_pose = self.CheckCubePoseValid() .set_name("check_cube_pose") .set_parent(self)
        parentfails2 = ParentFails() .set_name("parentfails2") .set_parent(self)
        choose_another_side = self.ChooseAnotherSide() .set_name("choose_another_side") .set_parent(self)
        parentfails3 = ParentFails() .set_name("parentfails3") .set_parent(self)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(droplift) .add_destinations(print1)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(print1) .add_destinations(looker,waitlift,check_cube_pose_valid)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(droplift) .add_destinations(print2)
        
        nulltrans2 = NullTrans() .set_name("nulltrans2")
        nulltrans2 .add_sources(print2) .add_destinations(looker,waitlift,check_cube_pose_valid)
        
        timertrans1 = TimerTrans(1) .set_name("timertrans1")
        timertrans1 .add_sources(waitlift) .add_destinations(check_almost_docked)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(check_cube_pose_valid) .add_destinations(check_cube_pose_valid)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(check_cube_pose_valid) .add_destinations(parentfails1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(check_almost_docked) .add_destinations(turn_to_cube2)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(check_almost_docked) .add_destinations(forward1)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(forward1) .add_destinations(manual_dock1)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(check_almost_docked) .add_destinations(pilot_check_start)
        
        datatrans1 = DataTrans() .set_name("datatrans1")
        datatrans1 .add_sources(manual_dock1) .add_destinations(turn1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(turn1) .add_destinations(print3)
        
        nulltrans3 = NullTrans() .set_name("nulltrans3")
        nulltrans3 .add_sources(print3) .add_destinations(manual_dock1)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(manual_dock1) .add_destinations(print4)
        
        nulltrans4 = NullTrans() .set_name("nulltrans4")
        nulltrans4 .add_sources(print4) .add_destinations(manual_dock2)
        
        timertrans2 = TimerTrans(10) .set_name("timertrans2")
        timertrans2 .add_sources(manual_dock1) .add_destinations(print5)
        
        nulltrans5 = NullTrans() .set_name("nulltrans5")
        nulltrans5 .add_sources(print5) .add_destinations(pilot_check_start)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(manual_dock2) .add_destinations(print6)
        
        nulltrans6 = NullTrans() .set_name("nulltrans6")
        nulltrans6 .add_sources(print6) .add_destinations(turn_to_cube1)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(pilot_check_start) .add_destinations(print7)
        
        nulltrans7 = NullTrans() .set_name("nulltrans7")
        nulltrans7 .add_sources(print7) .add_destinations(go_side)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(pilot_check_start) .add_destinations(print8)
        
        nulltrans8 = NullTrans() .set_name("nulltrans8")
        nulltrans8 .add_sources(print8) .add_destinations(forward2)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(forward2) .add_destinations(statenode1)
        
        timertrans3 = TimerTrans(0.5) .set_name("timertrans3")
        timertrans3 .add_sources(statenode1) .add_destinations(pilot_check_start2)
        
        successtrans3 = SuccessTrans() .set_name("successtrans3")
        successtrans3 .add_sources(pilot_check_start2) .add_destinations(print9)
        
        nulltrans9 = NullTrans() .set_name("nulltrans9")
        nulltrans9 .add_sources(print9) .add_destinations(go_side)
        
        pilottrans1 = PilotTrans(StartCollides) .set_name("pilottrans1")
        pilottrans1 .add_sources(pilot_check_start2) .add_destinations(check_start2_pilot)
        
        failuretrans5 = FailureTrans() .set_name("failuretrans5")
        failuretrans5 .add_sources(pilot_check_start2) .add_destinations(failure)
        
        pilottrans2 = PilotTrans(GoalCollides) .set_name("pilottrans2")
        pilottrans2 .add_sources(go_side) .add_destinations(failure)
        
        pilottrans3 = PilotTrans(MaxIterations) .set_name("pilottrans3")
        pilottrans3 .add_sources(go_side) .add_destinations(failure)
        
        pilottrans4 = PilotTrans() .set_name("pilottrans4")
        pilottrans4 .add_sources(go_side) .add_destinations(go_side_pilot)
        
        failuretrans6 = FailureTrans() .set_name("failuretrans6")
        failuretrans6 .add_sources(go_side) .add_destinations(failure)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(go_side) .add_destinations(reportposition1)
        
        timertrans4 = TimerTrans(0.75) .set_name("timertrans4")
        timertrans4 .add_sources(reportposition1) .add_destinations(reportposition2)
        
        nulltrans10 = NullTrans() .set_name("nulltrans10")
        nulltrans10 .add_sources(reportposition2) .add_destinations(turn_to_cube1)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(turn_to_cube1) .add_destinations(reportposition3)
        
        timertrans5 = TimerTrans(0.75) .set_name("timertrans5")
        timertrans5 .add_sources(reportposition3) .add_destinations(reportposition4)
        
        nulltrans11 = NullTrans() .set_name("nulltrans11")
        nulltrans11 .add_sources(reportposition4) .add_destinations(print10)
        
        nulltrans12 = NullTrans() .set_name("nulltrans12")
        nulltrans12 .add_sources(print10) .add_destinations(approach)
        
        failuretrans7 = FailureTrans() .set_name("failuretrans7")
        failuretrans7 .add_sources(turn_to_cube1) .add_destinations(recover1)
        
        timertrans6 = TimerTrans(5) .set_name("timertrans6")
        timertrans6 .add_sources(turn_to_cube1) .add_destinations(recover1)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(recover1) .add_destinations(statenode2)
        
        timertrans7 = TimerTrans(1) .set_name("timertrans7")
        timertrans7 .add_sources(statenode2) .add_destinations(turn_to_cube2)
        
        completiontrans12 = CompletionTrans() .set_name("completiontrans12")
        completiontrans12 .add_sources(approach) .add_destinations(statenode3)
        
        timertrans8 = TimerTrans(0.75) .set_name("timertrans8")
        timertrans8 .add_sources(statenode3) .add_destinations(reportposition5)
        
        timertrans9 = TimerTrans(0.75) .set_name("timertrans9")
        timertrans9 .add_sources(reportposition5) .add_destinations(reportposition6)
        
        nulltrans13 = NullTrans() .set_name("nulltrans13")
        nulltrans13 .add_sources(reportposition6) .add_destinations(print11)
        
        nulltrans14 = NullTrans() .set_name("nulltrans14")
        nulltrans14 .add_sources(print11) .add_destinations(turn_to_cube_1a)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(turn_to_cube_1a) .add_destinations(print12)
        
        nulltrans15 = NullTrans() .set_name("nulltrans15")
        nulltrans15 .add_sources(print12) .add_destinations(exit_to_roll)
        
        failuretrans8 = FailureTrans() .set_name("failuretrans8")
        failuretrans8 .add_sources(exit_to_roll) .add_destinations(forward_to_cube_1a)
        
        successtrans4 = SuccessTrans() .set_name("successtrans4")
        successtrans4 .add_sources(exit_to_roll) .add_destinations(forward3)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(forward3) .add_destinations(setliftheight1)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(setliftheight1) .add_destinations(forward_to_cube_1a)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(forward_to_cube_1a) .add_destinations(success)
        
        failuretrans9 = FailureTrans() .set_name("failuretrans9")
        failuretrans9 .add_sources(turn_to_cube2) .add_destinations(print13)
        
        nulltrans16 = NullTrans() .set_name("nulltrans16")
        nulltrans16 .add_sources(print13) .add_destinations(invalidatepose1)
        
        nulltrans17 = NullTrans() .set_name("nulltrans17")
        nulltrans17 .add_sources(invalidatepose1) .add_destinations(failure)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(turn_to_cube2) .add_destinations(forward_to_cube2)
        
        completiontrans18 = CompletionTrans() .set_name("completiontrans18")
        completiontrans18 .add_sources(forward_to_cube2) .add_destinations(turn_to_cube3)
        
        completiontrans19 = CompletionTrans() .set_name("completiontrans19")
        completiontrans19 .add_sources(turn_to_cube3) .add_destinations(exit_to_roll2)
        
        failuretrans10 = FailureTrans() .set_name("failuretrans10")
        failuretrans10 .add_sources(exit_to_roll2) .add_destinations(forward_to_cube3)
        
        successtrans5 = SuccessTrans() .set_name("successtrans5")
        successtrans5 .add_sources(exit_to_roll2) .add_destinations(forward4)
        
        completiontrans20 = CompletionTrans() .set_name("completiontrans20")
        completiontrans20 .add_sources(forward4) .add_destinations(setliftheight2)
        
        completiontrans21 = CompletionTrans() .set_name("completiontrans21")
        completiontrans21 .add_sources(setliftheight2) .add_destinations(forward_to_cube3)
        
        completiontrans22 = CompletionTrans() .set_name("completiontrans22")
        completiontrans22 .add_sources(forward_to_cube3) .add_destinations(success)
        
        nulltrans18 = NullTrans() .set_name("nulltrans18")
        nulltrans18 .add_sources(success) .add_destinations(parentcompletes1)
        
        nulltrans19 = NullTrans() .set_name("nulltrans19")
        nulltrans19 .add_sources(failure) .add_destinations(check_cube_pose)
        
        completiontrans23 = CompletionTrans() .set_name("completiontrans23")
        completiontrans23 .add_sources(check_cube_pose) .add_destinations(choose_another_side)
        
        failuretrans11 = FailureTrans() .set_name("failuretrans11")
        failuretrans11 .add_sources(check_cube_pose) .add_destinations(parentfails2)
        
        failuretrans12 = FailureTrans() .set_name("failuretrans12")
        failuretrans12 .add_sources(choose_another_side) .add_destinations(parentfails3)
        
        successtrans6 = SuccessTrans() .set_name("successtrans6")
        successtrans6 .add_sources(choose_another_side) .add_destinations(droplift)
        
        return self

class SetCarrying(StateNode):
    def __init__(self,objparam=None):
        self.objparam = objparam
        self.object = None
        super().__init__()

    def start(self, event=None):
        if self.objparam is not None:
            self.object = self.objparam
        else:
            self.object = self.parent.object
        if isinstance(self.object, LightCube):
            self.wmobject = self.object.wm_obj
        elif isinstance(self.object, LightCubeObj):
            self.wmobject = self.object
            self.object = self.object.sdk_obj
        else:
            raise ValueError(self.object)
        self.robot.carrying = self.wmobject
        self.robot.fetching = None
        self.wmobject.update_from_sdk = False
        self.wmobject.pose_confidence = +1
        super().start(event)
        self.post_completion()

class SetNotCarrying(StateNode):
    def start(self,event=None):
        self.robot.carrying = None
        self.robot.fetching = None
        super().start(event)
        self.post_completion()

class CheckCarrying(StateNode):
    def start(self, event=None):
        super().start(event)
        if self.robot.carrying:
            self.post_success()
        else:
            self.post_failure()

class SetFetching(StateNode):
    "Prevents pose invalidation if we bump the cube while trying to pick it up."
    def __init__(self,objparam=None):
        self.objparam = objparam
        self.object = None
        super().__init__()

    def start(self, event=None):
        if self.objparam is not None:
            self.object = self.objparam
        else:
            self.object = self.parent.object
        if isinstance(self.object, LightCube):
            self.wmobject = self.object.wm_obj
        elif isinstance(self.object, LightCubeObj):
            self.wmobject = self.object
            self.object = self.object.sdk_obj
        else:
            raise ValueError(self.object)
        self.robot.fetching = self.wmobject
        super().start(event)
        self.post_completion()


class SetNotFetching(StateNode):
    def start(self,event=None):
        super().start(event)
        self.robot.fetching = None
        self.post_completion()


class PickUpCube(StateNode):
    """Pick up a cube using our own dock and verify routines.
    Set self.object to indicate the cube to be picked up."""

    class VerifyPickup(StateNode):
        def probe_column(self, im, col, row_start, row_end):
            """
            Probe one column of the image, looking for the top horizontal
            black bar of the cube marker.  This bar should be 23-32 pixels
            thick.  Use adaptive thresholding by sorting the pixels and
            finding the darkest ones to set the black threshold.
            """
            pixels = [float(im[r,col,0]) for r in range(row_start,row_end)]
            #print('Column ',col,':',sep='')
            #[print('%4d' % i,end='') for i in pixels]
            pixels.sort()
            npix = len(pixels)
            bindex = 1
            bsum = pixels[0]
            bmax = pixels[0]
            bcnt = 1
            windex = npix-2
            wsum = pixels[npix-1]
            wmin = pixels[npix-1]
            wcnt = 1
            while bindex < windex:
                if abs(bmax-pixels[bindex]) < abs(wmin-pixels[windex]):
                    i = bindex
                    bindex += 1
                else:
                    i = windex
                    windex -= 1
                bmean = bsum / bcnt
                wmean = wsum / wcnt
                val = pixels[i]
                if abs(val-bmean) < abs(val-wmean):
                    bsum += val
                    bcnt += 1
                    bmax = max(bmax,val)
                else:
                    wsum += val
                    wcnt +=1
                    wmin = min(wmin,val)
            black_thresh = bmax
            index = row_start
            nrows = im.shape[0]
            black_run_length = 0
            # initial white run
            while index < nrows and im[index,col,0] > black_thresh:
                index += 1
            if index == nrows: return -1
            while index < nrows and im[index,col,0] <= black_thresh:
                black_run_length += 1
                index +=1
            if index >= nrows-5:
                retval = -1
            else:
                retval = black_run_length
            #print('  col=%3d wmin=%5.1f wmean=%5.1f bmean=%5.1f black_thresh=%5.1f run_length=%d' %
            #      (col, wmin, wmean, bmean, black_thresh, black_run_length))
            return retval

        def start(self,event=None):
            super().start(event)
            im = np.array(self.robot.world.latest_image.raw_image)
            min_length = 20
            max_length = 32
            bad_runs = 0
            print('Verifying pickup.  hangle=%4.1f deg.  langle=%4.1f deg.  lheight=%4.1f mm' %
                  (self.robot.head_angle.degrees, self.robot.lift_angle.degrees,
                   self.robot.lift_height.distance_mm))
            columns = (100, 110, 120, 200, 210, 220)  # avoid the screw in the center
            for col in columns: # range(100,220,20):
                run_length = self.probe_column(im, col, 8, 100)
                if run_length < min_length or run_length > max_length:
                    bad_runs += 1
            print('  Number of bad_runs:', bad_runs)
            if bad_runs < 2:
                self.post_success()
            else:
                self.post_failure()

        # end of class VerifyPickup

    # PickUpCube methods

    def __init__(self, cube=None):
        self.cube = cube
        super().__init__()

    def picked_up_handler(self):
        print("PickUpCube aborting because robot was picked up.")
        self.post_failure()
        self.stop()

    def start(self, event=None):
        if isinstance(self.cube, LightCube):
            self.object = self.cube
            self.wmobject = self.object.wm_obj
        elif isinstance(self.cube, LightCubeObj):
            self.wmobject = self.cube
            self.object = self.cube.sdk_obj
        elif isinstance(self.object, LightCube):
            self.wmobject = self.object.wm_obj
        elif isinstance(self.object, LightCubeObj):
            self.wmobject = self.object
            self.object = self.object.sdk_obj
        else:
            raise ValueError(self.object)
        self.children['goto_cube'].object = self.object
        print('Picking up',self.wmobject)
        super().start(event)

    def setup(self):
        """
            fetch: SetFetching() =C=> check_carry
    
            check_carry: CheckCarrying()
            check_carry =S=> DropObject() =C=> goto_cube
            check_carry =F=> goto_cube
    
            goto_cube: GoToCube()
            goto_cube =PILOT=> goto_cube_pilot: ParentPilotEvent() =N=> fail
            goto_cube =F=> fail
            goto_cube =C=> AbortHeadAction() =T(0.1) => # clear head track
               {raise_lift, raise_head}
    
            raise_lift: SetLiftHeight(0.4)
    
            raise_head: SetHeadAngle(5)
            raise_head =C=> wait: StateNode() =T(0.1)=> raise_head2
            raise_head =F=> wait
            raise_head2: SetHeadAngle(0, num_retries=2)
            raise_head2 =F=> verify
    
            {raise_lift, raise_head2} =C=> verify
    
            verify: self.VerifyPickup()
            verify =S=> set_carrying
            verify =F=> StateNode() =T(0.5)=> verify2
            verify2: self.VerifyPickup()
            verify2 =S=> set_carrying
            verify2 =F=> frustrated # was StateNode() =T(0.5)=> verify3
    
            # verify3 is dead code
            verify3: self.VerifyPickup()
            verify3 =S=> set_carrying
            verify3 =F=> frustrated
    
            set_carrying: SetCarrying() =N=> satisfied
    
            satisfied: AnimationTriggerNode(trigger=cozmo.anim.Triggers.ReactToBlockPickupSuccess,
                                            ignore_body_track=True,
                                            ignore_head_track=True,
                                            ignore_lift_track=True)
            satisfied =C=> {final_raise, drop_head}
    
            final_raise: SetLiftHeight(1.0)
            drop_head: SetHeadAngle(0)
    
            {final_raise, drop_head} =C=> ParentCompletes()
    
            frustrated: StateNode() =N=> AnimationTriggerNode(trigger=cozmo.anim.Triggers.FrustratedByFailure,
                                             ignore_body_track=True,
                                             ignore_head_track=True,
                                             ignore_lift_track=True) =C=>
            missed_cube: SetNotCarrying() =C=> Forward(-5) =C=> {drop_lift, drop_head_low}
    
            drop_lift: SetLiftHeight(0)
            drop_lift =C=> backupmore
            drop_lift =F=> backupmore
    
            backupmore: Forward(-5)
    
            drop_head_low: SetHeadAngle(-20)
    
            {backupmore, drop_head_low} =C=> fail
    
            fail: SetNotFetching() =C=> ParentFails()
    
        """
        
        # Code generated by genfsm on Fri Dec 20 18:08:35 2019:
        
        fetch = SetFetching() .set_name("fetch") .set_parent(self)
        check_carry = CheckCarrying() .set_name("check_carry") .set_parent(self)
        dropobject1 = DropObject() .set_name("dropobject1") .set_parent(self)
        goto_cube = GoToCube() .set_name("goto_cube") .set_parent(self)
        goto_cube_pilot = ParentPilotEvent() .set_name("goto_cube_pilot") .set_parent(self)
        abortheadaction1 = AbortHeadAction() .set_name("abortheadaction1") .set_parent(self)
        raise_lift = SetLiftHeight(0.4) .set_name("raise_lift") .set_parent(self)
        raise_head = SetHeadAngle(5) .set_name("raise_head") .set_parent(self)
        wait = StateNode() .set_name("wait") .set_parent(self)
        raise_head2 = SetHeadAngle(0, num_retries=2) .set_name("raise_head2") .set_parent(self)
        verify = self.VerifyPickup() .set_name("verify") .set_parent(self)
        statenode4 = StateNode() .set_name("statenode4") .set_parent(self)
        verify2 = self.VerifyPickup() .set_name("verify2") .set_parent(self)
        verify3 = self.VerifyPickup() .set_name("verify3") .set_parent(self)
        set_carrying = SetCarrying() .set_name("set_carrying") .set_parent(self)
        satisfied = AnimationTriggerNode(trigger=cozmo.anim.Triggers.ReactToBlockPickupSuccess,
                                        ignore_body_track=True,
                                        ignore_head_track=True,
                                        ignore_lift_track=True) .set_name("satisfied") .set_parent(self)
        final_raise = SetLiftHeight(1.0) .set_name("final_raise") .set_parent(self)
        drop_head = SetHeadAngle(0) .set_name("drop_head") .set_parent(self)
        parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
        frustrated = StateNode() .set_name("frustrated") .set_parent(self)
        animationtriggernode1 = AnimationTriggerNode(trigger=cozmo.anim.Triggers.FrustratedByFailure,
                                         ignore_body_track=True,
                                         ignore_head_track=True,
                                         ignore_lift_track=True) .set_name("animationtriggernode1") .set_parent(self)
        missed_cube = SetNotCarrying() .set_name("missed_cube") .set_parent(self)
        forward5 = Forward(-5) .set_name("forward5") .set_parent(self)
        drop_lift = SetLiftHeight(0) .set_name("drop_lift") .set_parent(self)
        backupmore = Forward(-5) .set_name("backupmore") .set_parent(self)
        drop_head_low = SetHeadAngle(-20) .set_name("drop_head_low") .set_parent(self)
        fail = SetNotFetching() .set_name("fail") .set_parent(self)
        parentfails4 = ParentFails() .set_name("parentfails4") .set_parent(self)
        
        completiontrans24 = CompletionTrans() .set_name("completiontrans24")
        completiontrans24 .add_sources(fetch) .add_destinations(check_carry)
        
        successtrans7 = SuccessTrans() .set_name("successtrans7")
        successtrans7 .add_sources(check_carry) .add_destinations(dropobject1)
        
        completiontrans25 = CompletionTrans() .set_name("completiontrans25")
        completiontrans25 .add_sources(dropobject1) .add_destinations(goto_cube)
        
        failuretrans13 = FailureTrans() .set_name("failuretrans13")
        failuretrans13 .add_sources(check_carry) .add_destinations(goto_cube)
        
        pilottrans5 = PilotTrans() .set_name("pilottrans5")
        pilottrans5 .add_sources(goto_cube) .add_destinations(goto_cube_pilot)
        
        nulltrans20 = NullTrans() .set_name("nulltrans20")
        nulltrans20 .add_sources(goto_cube_pilot) .add_destinations(fail)
        
        failuretrans14 = FailureTrans() .set_name("failuretrans14")
        failuretrans14 .add_sources(goto_cube) .add_destinations(fail)
        
        completiontrans26 = CompletionTrans() .set_name("completiontrans26")
        completiontrans26 .add_sources(goto_cube) .add_destinations(abortheadaction1)
        
        timertrans10 = TimerTrans(0.1) .set_name("timertrans10")
        timertrans10 .add_sources(abortheadaction1) .add_destinations(raise_lift,raise_head)
        
        completiontrans27 = CompletionTrans() .set_name("completiontrans27")
        completiontrans27 .add_sources(raise_head) .add_destinations(wait)
        
        timertrans11 = TimerTrans(0.1) .set_name("timertrans11")
        timertrans11 .add_sources(wait) .add_destinations(raise_head2)
        
        failuretrans15 = FailureTrans() .set_name("failuretrans15")
        failuretrans15 .add_sources(raise_head) .add_destinations(wait)
        
        failuretrans16 = FailureTrans() .set_name("failuretrans16")
        failuretrans16 .add_sources(raise_head2) .add_destinations(verify)
        
        completiontrans28 = CompletionTrans() .set_name("completiontrans28")
        completiontrans28 .add_sources(raise_lift,raise_head2) .add_destinations(verify)
        
        successtrans8 = SuccessTrans() .set_name("successtrans8")
        successtrans8 .add_sources(verify) .add_destinations(set_carrying)
        
        failuretrans17 = FailureTrans() .set_name("failuretrans17")
        failuretrans17 .add_sources(verify) .add_destinations(statenode4)
        
        timertrans12 = TimerTrans(0.5) .set_name("timertrans12")
        timertrans12 .add_sources(statenode4) .add_destinations(verify2)
        
        successtrans9 = SuccessTrans() .set_name("successtrans9")
        successtrans9 .add_sources(verify2) .add_destinations(set_carrying)
        
        failuretrans18 = FailureTrans() .set_name("failuretrans18")
        failuretrans18 .add_sources(verify2) .add_destinations(frustrated)
        
        successtrans10 = SuccessTrans() .set_name("successtrans10")
        successtrans10 .add_sources(verify3) .add_destinations(set_carrying)
        
        failuretrans19 = FailureTrans() .set_name("failuretrans19")
        failuretrans19 .add_sources(verify3) .add_destinations(frustrated)
        
        nulltrans21 = NullTrans() .set_name("nulltrans21")
        nulltrans21 .add_sources(set_carrying) .add_destinations(satisfied)
        
        completiontrans29 = CompletionTrans() .set_name("completiontrans29")
        completiontrans29 .add_sources(satisfied) .add_destinations(final_raise,drop_head)
        
        completiontrans30 = CompletionTrans() .set_name("completiontrans30")
        completiontrans30 .add_sources(final_raise,drop_head) .add_destinations(parentcompletes2)
        
        nulltrans22 = NullTrans() .set_name("nulltrans22")
        nulltrans22 .add_sources(frustrated) .add_destinations(animationtriggernode1)
        
        completiontrans31 = CompletionTrans() .set_name("completiontrans31")
        completiontrans31 .add_sources(animationtriggernode1) .add_destinations(missed_cube)
        
        completiontrans32 = CompletionTrans() .set_name("completiontrans32")
        completiontrans32 .add_sources(missed_cube) .add_destinations(forward5)
        
        completiontrans33 = CompletionTrans() .set_name("completiontrans33")
        completiontrans33 .add_sources(forward5) .add_destinations(drop_lift,drop_head_low)
        
        completiontrans34 = CompletionTrans() .set_name("completiontrans34")
        completiontrans34 .add_sources(drop_lift) .add_destinations(backupmore)
        
        failuretrans20 = FailureTrans() .set_name("failuretrans20")
        failuretrans20 .add_sources(drop_lift) .add_destinations(backupmore)
        
        completiontrans35 = CompletionTrans() .set_name("completiontrans35")
        completiontrans35 .add_sources(backupmore,drop_head_low) .add_destinations(fail)
        
        completiontrans36 = CompletionTrans() .set_name("completiontrans36")
        completiontrans36 .add_sources(fail) .add_destinations(parentfails4)
        
        return self

class DropObject(StateNode):

    class SetObject(StateNode):
        def start(self,event=None):
            super().start(event)
            self.parent.object = self.robot.carrying

    class CheckCubeVisible(StateNode):
        def start(self,event=None):
            super().start(event)
            for cube in self.robot.world.light_cubes.values():
                if cube and cube.is_visible:
                    self.post_completion()
                    return
            self.post_failure()

    def setup(self):
        """
            SetLiftHeight(0) =C=> check_carrying
    
            check_carrying: CheckCarrying()
            check_carrying =F=> {backup, lookdown}
            check_carrying =S=> self.SetObject() =N=>
              SetNotCarrying() =N=> SetFetching() =N=> {backup, lookdown}
    
            backup: Forward(-15)
    
            # Robots differ on head angle alignment, so try a shallow angle,
            # and if we don't see the cube, try a steeper one.
            lookdown: SetHeadAngle(-12)
            lookdown =F=> head_angle_wait  # Shouldn't fail, but just in case
    
            {backup, lookdown} =C=> head_angle_wait
    
            head_angle_wait: StateNode() =T(0.5)=> check_visible
    
            check_visible: self.CheckCubeVisible()
            check_visible =C=> wrap_up
            check_visible =F=> lookdown2
    
            # Try a lower head angle, but keep going even if we don't see the object
            lookdown2: SetHeadAngle(-20)
            lookdown2 =F=> wrap_up  # Shouldn't fail, but just in case
            lookdown2 =T(0.5)=> wrap_up
    
            wrap_up: SetNotFetching() =N=> ParentCompletes()
        """
        
        # Code generated by genfsm on Fri Dec 20 18:08:35 2019:
        
        setliftheight3 = SetLiftHeight(0) .set_name("setliftheight3") .set_parent(self)
        check_carrying = CheckCarrying() .set_name("check_carrying") .set_parent(self)
        setobject1 = self.SetObject() .set_name("setobject1") .set_parent(self)
        setnotcarrying1 = SetNotCarrying() .set_name("setnotcarrying1") .set_parent(self)
        setfetching1 = SetFetching() .set_name("setfetching1") .set_parent(self)
        backup = Forward(-15) .set_name("backup") .set_parent(self)
        lookdown = SetHeadAngle(-12) .set_name("lookdown") .set_parent(self)
        head_angle_wait = StateNode() .set_name("head_angle_wait") .set_parent(self)
        check_visible = self.CheckCubeVisible() .set_name("check_visible") .set_parent(self)
        lookdown2 = SetHeadAngle(-20) .set_name("lookdown2") .set_parent(self)
        wrap_up = SetNotFetching() .set_name("wrap_up") .set_parent(self)
        parentcompletes3 = ParentCompletes() .set_name("parentcompletes3") .set_parent(self)
        
        completiontrans37 = CompletionTrans() .set_name("completiontrans37")
        completiontrans37 .add_sources(setliftheight3) .add_destinations(check_carrying)
        
        failuretrans21 = FailureTrans() .set_name("failuretrans21")
        failuretrans21 .add_sources(check_carrying) .add_destinations(backup,lookdown)
        
        successtrans11 = SuccessTrans() .set_name("successtrans11")
        successtrans11 .add_sources(check_carrying) .add_destinations(setobject1)
        
        nulltrans23 = NullTrans() .set_name("nulltrans23")
        nulltrans23 .add_sources(setobject1) .add_destinations(setnotcarrying1)
        
        nulltrans24 = NullTrans() .set_name("nulltrans24")
        nulltrans24 .add_sources(setnotcarrying1) .add_destinations(setfetching1)
        
        nulltrans25 = NullTrans() .set_name("nulltrans25")
        nulltrans25 .add_sources(setfetching1) .add_destinations(backup,lookdown)
        
        failuretrans22 = FailureTrans() .set_name("failuretrans22")
        failuretrans22 .add_sources(lookdown) .add_destinations(head_angle_wait)
        
        completiontrans38 = CompletionTrans() .set_name("completiontrans38")
        completiontrans38 .add_sources(backup,lookdown) .add_destinations(head_angle_wait)
        
        timertrans13 = TimerTrans(0.5) .set_name("timertrans13")
        timertrans13 .add_sources(head_angle_wait) .add_destinations(check_visible)
        
        completiontrans39 = CompletionTrans() .set_name("completiontrans39")
        completiontrans39 .add_sources(check_visible) .add_destinations(wrap_up)
        
        failuretrans23 = FailureTrans() .set_name("failuretrans23")
        failuretrans23 .add_sources(check_visible) .add_destinations(lookdown2)
        
        failuretrans24 = FailureTrans() .set_name("failuretrans24")
        failuretrans24 .add_sources(lookdown2) .add_destinations(wrap_up)
        
        timertrans14 = TimerTrans(0.5) .set_name("timertrans14")
        timertrans14 .add_sources(lookdown2) .add_destinations(wrap_up)
        
        nulltrans26 = NullTrans() .set_name("nulltrans26")
        nulltrans26 .add_sources(wrap_up) .add_destinations(parentcompletes3)
        
        return self


class RollCube(StateNode):
    def __init__(self):
        super().__init__()

    def start(self, event=None):
        super().start(event)

    def setup(self):
        """
            put_arm: SetLiftHeight(0.68)
            put_arm =C=> {drop, back}
            put_arm =F=> ParentFails()
    
            drop: SetLiftHeight(0.28)
            back: Forward(-60)
    
            drop =F=> ParentFails()
            back =F=> ParentFails()
    
            {drop, back} =C=> reset
            {drop, back} =F=> ParentFails()
    
            reset: SetLiftHeight(0)
            reset =C=> Forward(-10) =C=> ParentCompletes()
            reset =F=> ParentFails()
        """
        
        # Code generated by genfsm on Fri Dec 20 18:08:35 2019:
        
        put_arm = SetLiftHeight(0.68) .set_name("put_arm") .set_parent(self)
        parentfails5 = ParentFails() .set_name("parentfails5") .set_parent(self)
        drop = SetLiftHeight(0.28) .set_name("drop") .set_parent(self)
        back = Forward(-60) .set_name("back") .set_parent(self)
        parentfails6 = ParentFails() .set_name("parentfails6") .set_parent(self)
        parentfails7 = ParentFails() .set_name("parentfails7") .set_parent(self)
        parentfails8 = ParentFails() .set_name("parentfails8") .set_parent(self)
        reset = SetLiftHeight(0) .set_name("reset") .set_parent(self)
        forward6 = Forward(-10) .set_name("forward6") .set_parent(self)
        parentcompletes4 = ParentCompletes() .set_name("parentcompletes4") .set_parent(self)
        parentfails9 = ParentFails() .set_name("parentfails9") .set_parent(self)
        
        completiontrans40 = CompletionTrans() .set_name("completiontrans40")
        completiontrans40 .add_sources(put_arm) .add_destinations(drop,back)
        
        failuretrans25 = FailureTrans() .set_name("failuretrans25")
        failuretrans25 .add_sources(put_arm) .add_destinations(parentfails5)
        
        failuretrans26 = FailureTrans() .set_name("failuretrans26")
        failuretrans26 .add_sources(drop) .add_destinations(parentfails6)
        
        failuretrans27 = FailureTrans() .set_name("failuretrans27")
        failuretrans27 .add_sources(back) .add_destinations(parentfails7)
        
        completiontrans41 = CompletionTrans() .set_name("completiontrans41")
        completiontrans41 .add_sources(drop,back) .add_destinations(reset)
        
        failuretrans28 = FailureTrans() .set_name("failuretrans28")
        failuretrans28 .add_sources(drop,back) .add_destinations(parentfails8)
        
        completiontrans42 = CompletionTrans() .set_name("completiontrans42")
        completiontrans42 .add_sources(reset) .add_destinations(forward6)
        
        completiontrans43 = CompletionTrans() .set_name("completiontrans43")
        completiontrans43 .add_sources(forward6) .add_destinations(parentcompletes4)
        
        failuretrans29 = FailureTrans() .set_name("failuretrans29")
        failuretrans29 .add_sources(reset) .add_destinations(parentfails9)
        
        return self


class RollingCube(StateNode):
    def __init__(self, object=None, orientation=None, old_object=None):
        super().__init__()
        self.object = object
        self.target_orientation = orientation
        self.old_object = old_object
        self.try_roll_number = 0


    def start(self, event=None):
        if isinstance(self.object, LightCube):
            self.wmobject = self.object.wm_obj
        elif isinstance(self.object, LightCubeObj):
            self.wmobject = self.object
            self.object = self.object.sdk_obj
        else:
            return
        self.children['looker'].object = self.object
        self.children['goto_cube'].object = self.object
        self.children['goto_cube'].roll_cube = True
        self.children['goto_cube'].roll_target = self.target_orientation
        self.children['roll1'].object = self.object
        self.children['roll2'].object = self.object
        self.children['roll3'].object = self.object
        super().start(event)
        if self.wmobject.pose_confidence < 0:
            print('RollingCube: cube has invalid pose!', self.wmobject, self.object.pose)
            self.post_event(PilotEvent(InvalidPose))
            self.post_failure()


    class CheckOrientation(StateNode):
        def start(self,event=None):
            self.parent.orientation, _, _, _ = get_orientation_state(self.parent.object.pose.rotation.q0_q1_q2_q3)
            super().start(event)
            if self.parent.target_orientation == self.parent.orientation != None:
                print('roll it successfully to', self.parent.target_orientation)
                self.post_completion()
            elif not self.parent.target_orientation:
                if not self.parent.old_object:
                    self.parent.old_object = copy.copy(self.parent.object)
                    self.post_failure()
                elif same_orientation(self.parent.old_object, self.parent.object):
                    print('still the same orientation')
                    self.post_failure()
                else:
                    self.parent.old_object = copy.copy(self.parent.object)
                    print('roll it successfully')
                    self.post_completion()
            else:
                self.post_failure()

    class CheckCubePoseValidOnce(StateNode):
        def __init__(self, check_vis=False, reset=False):
            self.check_vis = check_vis
            self.reset = reset
            super().__init__()

        def start(self, event=None):
            super().start(event)

            if isinstance(self.parent.object, LightCube):
                cube_id = self.parent.object.wm_obj.id
            elif isinstance(self.parent.object, LightCubeObj):
                cube_id = self.parent.object.id
            else:
                raise ValueError(self.parent.object)

            if 'Cube' not in str(cube_id):
                cube_id = 'Cube-' + str(cube_id)
            wmobject = self.robot.world.world_map.objects[cube_id]

            if self.check_vis and not self.parent.object.is_visible:
                if self.reset:
                    self.parent.wmobject.pose_confidence = -1
                    print('CheckCubePoseValidOnce: %s has invalid pose!' % cube_id)
                self.post_failure()
            elif self.parent.wmobject.pose_confidence < 0:
                print('pose_confidence', self.parent.wmobject.pose_confidence)
                self.post_failure()
            else:
                self.post_completion()

    class CheckCounts(StateNode):
        def start(self,event=None):
            SIDEWAYS_FRONT = 'front'
            SIDEWAYS_BACK = 'back'
            SIDEWAYS_SIDE = 'side'

            super().start(event)
            if not self.parent.target_orientation:
                print('no target_orientation')
                self.post_data(1)
            elif self.parent.orientation == ORIENTATION_UPRIGHT or self.parent.orientation == ORIENTATION_INVERTED:
                if self.parent.target_orientation == ORIENTATION_SIDEWAYS:
                    self.post_data(1)
                elif self.parent.orientation == self.parent.target_orientation:
                    self.post_data(3)
                else:
                    self.post_data(2)
            else:   # ORIENTATION_SIDEWAYS
                x, y, z = self.parent.object.pose.rotation.euler_angles
                robot_angle = self.robot.pose.rotation.angle_z.radians
                pattern = get_pattern_number(self.parent.object.pose.rotation.euler_angles)
                if pattern == 1:
                    cube_rotate_angle = wrap_angle(x - pi/2)
                    new_robot_angle = wrap_angle(robot_angle + cube_rotate_angle)
                elif pattern == 2:
                    cube_rotate_angle = wrap_angle(y + pi)
                    new_robot_angle = wrap_angle(robot_angle + cube_rotate_angle)
                elif pattern == 3:
                    cube_rotate_angle = wrap_angle(- (x + pi/2))
                    new_robot_angle = wrap_angle(robot_angle + cube_rotate_angle)
                elif pattern == 4:
                    cube_rotate_angle = y
                    new_robot_angle = wrap_angle(robot_angle + cube_rotate_angle)
                else:
                    print('Unrecognized pattern.')
                    cube_rotate_angle = 0
                    new_robot_angle = 0

                possible_angles = [-pi, -pi/2, 0, pi/2, pi]
                facing_side = None
                new_robot_angle = min(possible_angles, key=lambda val:abs(val-new_robot_angle))
                if new_robot_angle == 0:
                    facing_side = SIDEWAYS_FRONT
                elif abs(new_robot_angle) == pi:
                    facing_side = SIDEWAYS_BACK
                else:
                    facing_side = SIDEWAYS_SIDE

                # print('new_robot_angle: %.2f' % new_robot_angle)
                print('Robot is facing', facing_side)

                if facing_side == SIDEWAYS_SIDE:    # left/right
                    if self.parent.target_orientation == ORIENTATION_SIDEWAYS:
                        self.post_data(1)
                    else:
                        self.post_data(0)
                elif facing_side == SIDEWAYS_FRONT:  # front
                    if self.parent.target_orientation == ORIENTATION_INVERTED:
                        self.post_data(1)
                    elif self.parent.target_orientation == ORIENTATION_SIDEWAYS:
                        self.post_data(2)
                    else:
                        self.post_data(3)
                elif facing_side == SIDEWAYS_BACK:  # back
                    if self.parent.target_orientation == ORIENTATION_UPRIGHT:
                        self.post_data(1)
                    elif self.parent.target_orientation == ORIENTATION_SIDEWAYS:
                        self.post_data(2)
                    else:
                        self.post_data(3)


    class TryAgain(StateNode):
        def start(self, event=None):
            super().start(event)

            if self.parent.try_roll_number > 2:
                self.parent.try_roll_number = 0
                self.post_failure()
            else:
                self.parent.try_roll_number += 1
                print('try again', self.parent.try_roll_number)
                self.post_success()

    class ForwardToCube(Forward):
        def __init__(self, offset):
            self.offset = offset
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            dx = cube.pose.position.x - self.robot.pose.position.x
            dy = cube.pose.position.y - self.robot.pose.position.y
            dist = sqrt(dx*dx + dy*dy) - self.offset
            if (dist < 0):
                print('***** ForwardToCube %s negative distance: %.1f mm' % (self.name,dist))
            self.distance = Distance(dist)
            print('ForwardToCube %s: distance %.1f mm' % (self.name, self.distance.distance_mm))
            super().start(event)


    def setup(self):
        """
            start: SetFetching() =C=> {looker, check_cube_pose}
    
            looker: LookAtObject()
    
            check_cube_pose: self.CheckCubePoseValidOnce(check_vis=False)
            check_cube_pose =C=> check_orientation
            check_cube_pose =F=> ParentFails()
    
            check_orientation: self.CheckOrientation()
            check_orientation =C=> ParentCompletes()
            check_orientation =F=> goto_cube
    
            goto_cube: GoToCube()
            goto_cube =F=> Print('goto_cube has failed.') =N=> try_again
            goto_cube =C=> Print('goto_cube has succeeded.') =N=> decide_roll_counts
    
            decide_roll_counts: self.CheckCounts()
            decide_roll_counts =D(0)=> Print('No way to achieve from this side') =C=> check_cube_pose_valid
            decide_roll_counts =D(1)=> Print('Roll once') =C=> roll1
            decide_roll_counts =D(2)=> Print('Roll twice') =C=> roll2
            decide_roll_counts =D(3)=> Print('Roll thrice') =C=> roll3
    
            check_cube_pose_valid: self.CheckCubePoseValidOnce(check_vis=True)
            check_cube_pose_valid =C=> check_orientation
            check_cube_pose_valid =F=> Forward(-30) =C=> Print('backing up...') =T(1)=> check_cube_pose_valid2
    
            check_cube_pose_valid2:self.CheckCubePoseValidOnce(check_vis=True, reset=True)
            check_cube_pose_valid2 =C=> check_orientation
            check_cube_pose_valid2 =F=> fail
    
            roll1: RollCube()
            roll1 =C=> Forward(-30) =C=> Print('Checking the new orientation...') =T(1)=> check_cube_pose_valid
            roll1 =F=> try_again
    
            roll2: RollCube()
            roll2 =C=> SetLiftHeight(1) =C=> self.ForwardToCube(15) =C=> roll1
            roll2 =F=> try_again
    
            roll3: RollCube()
            roll3 =C=> SetLiftHeight(1) =C=> self.ForwardToCube(15) =C=> roll2
            roll3 =F=> try_again
    
            try_again: self.TryAgain()
            try_again =S=> check_cube_pose_valid
            try_again =F=> fail
    
            fail: SetNotFetching() =C=> ParentFails()
        """
        
        # Code generated by genfsm on Fri Dec 20 18:08:35 2019:
        
        start = SetFetching() .set_name("start") .set_parent(self)
        looker = LookAtObject() .set_name("looker") .set_parent(self)
        check_cube_pose = self.CheckCubePoseValidOnce(check_vis=False) .set_name("check_cube_pose") .set_parent(self)
        parentfails10 = ParentFails() .set_name("parentfails10") .set_parent(self)
        check_orientation = self.CheckOrientation() .set_name("check_orientation") .set_parent(self)
        parentcompletes5 = ParentCompletes() .set_name("parentcompletes5") .set_parent(self)
        goto_cube = GoToCube() .set_name("goto_cube") .set_parent(self)
        print14 = Print('goto_cube has failed.') .set_name("print14") .set_parent(self)
        print15 = Print('goto_cube has succeeded.') .set_name("print15") .set_parent(self)
        decide_roll_counts = self.CheckCounts() .set_name("decide_roll_counts") .set_parent(self)
        print16 = Print('No way to achieve from this side') .set_name("print16") .set_parent(self)
        print17 = Print('Roll once') .set_name("print17") .set_parent(self)
        print18 = Print('Roll twice') .set_name("print18") .set_parent(self)
        print19 = Print('Roll thrice') .set_name("print19") .set_parent(self)
        check_cube_pose_valid = self.CheckCubePoseValidOnce(check_vis=True) .set_name("check_cube_pose_valid") .set_parent(self)
        forward7 = Forward(-30) .set_name("forward7") .set_parent(self)
        print20 = Print('backing up...') .set_name("print20") .set_parent(self)
        check_cube_pose_valid2 = self.CheckCubePoseValidOnce(check_vis=True, reset=True) .set_name("check_cube_pose_valid2") .set_parent(self)
        roll1 = RollCube() .set_name("roll1") .set_parent(self)
        forward8 = Forward(-30) .set_name("forward8") .set_parent(self)
        print21 = Print('Checking the new orientation...') .set_name("print21") .set_parent(self)
        roll2 = RollCube() .set_name("roll2") .set_parent(self)
        setliftheight4 = SetLiftHeight(1) .set_name("setliftheight4") .set_parent(self)
        forwardtocube1 = self.ForwardToCube(15) .set_name("forwardtocube1") .set_parent(self)
        roll3 = RollCube() .set_name("roll3") .set_parent(self)
        setliftheight5 = SetLiftHeight(1) .set_name("setliftheight5") .set_parent(self)
        forwardtocube2 = self.ForwardToCube(15) .set_name("forwardtocube2") .set_parent(self)
        try_again = self.TryAgain() .set_name("try_again") .set_parent(self)
        fail = SetNotFetching() .set_name("fail") .set_parent(self)
        parentfails11 = ParentFails() .set_name("parentfails11") .set_parent(self)
        
        completiontrans44 = CompletionTrans() .set_name("completiontrans44")
        completiontrans44 .add_sources(start) .add_destinations(looker,check_cube_pose)
        
        completiontrans45 = CompletionTrans() .set_name("completiontrans45")
        completiontrans45 .add_sources(check_cube_pose) .add_destinations(check_orientation)
        
        failuretrans30 = FailureTrans() .set_name("failuretrans30")
        failuretrans30 .add_sources(check_cube_pose) .add_destinations(parentfails10)
        
        completiontrans46 = CompletionTrans() .set_name("completiontrans46")
        completiontrans46 .add_sources(check_orientation) .add_destinations(parentcompletes5)
        
        failuretrans31 = FailureTrans() .set_name("failuretrans31")
        failuretrans31 .add_sources(check_orientation) .add_destinations(goto_cube)
        
        failuretrans32 = FailureTrans() .set_name("failuretrans32")
        failuretrans32 .add_sources(goto_cube) .add_destinations(print14)
        
        nulltrans27 = NullTrans() .set_name("nulltrans27")
        nulltrans27 .add_sources(print14) .add_destinations(try_again)
        
        completiontrans47 = CompletionTrans() .set_name("completiontrans47")
        completiontrans47 .add_sources(goto_cube) .add_destinations(print15)
        
        nulltrans28 = NullTrans() .set_name("nulltrans28")
        nulltrans28 .add_sources(print15) .add_destinations(decide_roll_counts)
        
        datatrans2 = DataTrans(0) .set_name("datatrans2")
        datatrans2 .add_sources(decide_roll_counts) .add_destinations(print16)
        
        completiontrans48 = CompletionTrans() .set_name("completiontrans48")
        completiontrans48 .add_sources(print16) .add_destinations(check_cube_pose_valid)
        
        datatrans3 = DataTrans(1) .set_name("datatrans3")
        datatrans3 .add_sources(decide_roll_counts) .add_destinations(print17)
        
        completiontrans49 = CompletionTrans() .set_name("completiontrans49")
        completiontrans49 .add_sources(print17) .add_destinations(roll1)
        
        datatrans4 = DataTrans(2) .set_name("datatrans4")
        datatrans4 .add_sources(decide_roll_counts) .add_destinations(print18)
        
        completiontrans50 = CompletionTrans() .set_name("completiontrans50")
        completiontrans50 .add_sources(print18) .add_destinations(roll2)
        
        datatrans5 = DataTrans(3) .set_name("datatrans5")
        datatrans5 .add_sources(decide_roll_counts) .add_destinations(print19)
        
        completiontrans51 = CompletionTrans() .set_name("completiontrans51")
        completiontrans51 .add_sources(print19) .add_destinations(roll3)
        
        completiontrans52 = CompletionTrans() .set_name("completiontrans52")
        completiontrans52 .add_sources(check_cube_pose_valid) .add_destinations(check_orientation)
        
        failuretrans33 = FailureTrans() .set_name("failuretrans33")
        failuretrans33 .add_sources(check_cube_pose_valid) .add_destinations(forward7)
        
        completiontrans53 = CompletionTrans() .set_name("completiontrans53")
        completiontrans53 .add_sources(forward7) .add_destinations(print20)
        
        timertrans15 = TimerTrans(1) .set_name("timertrans15")
        timertrans15 .add_sources(print20) .add_destinations(check_cube_pose_valid2)
        
        completiontrans54 = CompletionTrans() .set_name("completiontrans54")
        completiontrans54 .add_sources(check_cube_pose_valid2) .add_destinations(check_orientation)
        
        failuretrans34 = FailureTrans() .set_name("failuretrans34")
        failuretrans34 .add_sources(check_cube_pose_valid2) .add_destinations(fail)
        
        completiontrans55 = CompletionTrans() .set_name("completiontrans55")
        completiontrans55 .add_sources(roll1) .add_destinations(forward8)
        
        completiontrans56 = CompletionTrans() .set_name("completiontrans56")
        completiontrans56 .add_sources(forward8) .add_destinations(print21)
        
        timertrans16 = TimerTrans(1) .set_name("timertrans16")
        timertrans16 .add_sources(print21) .add_destinations(check_cube_pose_valid)
        
        failuretrans35 = FailureTrans() .set_name("failuretrans35")
        failuretrans35 .add_sources(roll1) .add_destinations(try_again)
        
        completiontrans57 = CompletionTrans() .set_name("completiontrans57")
        completiontrans57 .add_sources(roll2) .add_destinations(setliftheight4)
        
        completiontrans58 = CompletionTrans() .set_name("completiontrans58")
        completiontrans58 .add_sources(setliftheight4) .add_destinations(forwardtocube1)
        
        completiontrans59 = CompletionTrans() .set_name("completiontrans59")
        completiontrans59 .add_sources(forwardtocube1) .add_destinations(roll1)
        
        failuretrans36 = FailureTrans() .set_name("failuretrans36")
        failuretrans36 .add_sources(roll2) .add_destinations(try_again)
        
        completiontrans60 = CompletionTrans() .set_name("completiontrans60")
        completiontrans60 .add_sources(roll3) .add_destinations(setliftheight5)
        
        completiontrans61 = CompletionTrans() .set_name("completiontrans61")
        completiontrans61 .add_sources(setliftheight5) .add_destinations(forwardtocube2)
        
        completiontrans62 = CompletionTrans() .set_name("completiontrans62")
        completiontrans62 .add_sources(forwardtocube2) .add_destinations(roll2)
        
        failuretrans37 = FailureTrans() .set_name("failuretrans37")
        failuretrans37 .add_sources(roll3) .add_destinations(try_again)
        
        successtrans12 = SuccessTrans() .set_name("successtrans12")
        successtrans12 .add_sources(try_again) .add_destinations(check_cube_pose_valid)
        
        failuretrans38 = FailureTrans() .set_name("failuretrans38")
        failuretrans38 .add_sources(try_again) .add_destinations(fail)
        
        completiontrans63 = CompletionTrans() .set_name("completiontrans63")
        completiontrans63 .add_sources(fail) .add_destinations(parentfails11)
        
        return self


"""
class PickUpCubeForeign(StateNode):

    # *** THIS IS OLD CODE AND NEEDS TO BE UPDATED ***

    def __init__(self, cube_id=None):
        self.object_id = cube_id
        super().__init__()

    def start(self, event=None):
        # self.object will be set up by the parent of this node
        self.object = self.robot.world.light_cubes[self.object_id]
        self.foreign_cube_id = 'LightCubeForeignObj-'+str(self.object_id)
        super().start(event)

    def pick_side(self, dist, use_world_map):
        # NOTE: This code is only correct for upright cubes
        cube = self.foreign_cube_id
        wobj = self.robot.world.world_map.objects[cube]
        x = wobj.x
        y = wobj.y
        ang = wobj.theta
        rx = self.robot.world.particle_filter.pose[0]
        ry = self.robot.world.particle_filter.pose[1]

        side1 = (x + cos(ang) * dist, y + sin(ang) * dist, ang + pi)
        side2 = (x - cos(ang) * dist, y - sin(ang) * dist, ang)
        side3 = (x + sin(ang) * dist, y - cos(ang) * dist, ang + pi/2)
        side4 = (x - sin(ang) * dist, y + cos(ang) * dist, ang - pi/2)
        sides = [side1, side2, side3, side4]
        sorted_sides = sorted(sides, key=lambda pt: (pt[0]-rx)**2 + (pt[1]-ry)**2)
        return sorted_sides[0]

    class GoToSide(WallPilotToPose):
        def __init__(self):
            super().__init__(None)

        def start(self, event=None):
            cube = self.parent.foreign_cube_id
            print('Selected cube',self.robot.world.world_map.objects[cube])
            (x, y, theta) = self.parent.pick_side(200, True)
            self.target_pose = Pose(x, y, self.robot.pose.position.z,
                                    angle_z=Angle(radians = wrap_angle(theta)))
            print('pickup.GoToSide: traveling to (%.1f, %.1f) @ %.1f deg.' %
                  (self.target_pose.position.x, self.target_pose.position.y,
                   self.target_pose.rotation.angle_z.degrees))
            super().start(event)

    class Pick(PickUpCube):
        def __init__(self):
            super().__init__(None)

        def start(self, event=None):
            self.object = self.parent.object
            super().start(event)

    #setup{  # PickUpCube machine
        goto_cube: self.GoToSide() =C=> one

        one: self.Pick() =C=> end
        end: Say('Done') =C=> ParentCompletes()
        }
"""
