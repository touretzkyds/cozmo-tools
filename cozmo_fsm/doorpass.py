from cozmo.util import Pose
from numpy import matrix, tan, arctan2
from math import sin, cos, atan2, pi, sqrt

try: from cv2 import Rodrigues
except: pass

from .nodes import *
from .transitions import *
from .geometry import wrap_angle
from .pilot0 import *
from .worldmap import WallObj, DoorwayObj
from time import sleep

class DoorPass(StateNode):
    """Pass through a doorway. Assumes the doorway is nearby and unobstructed."""

    OUTER_GATE_DISTANCE = 150 # mm
    INNER_GATE_DISTANCE =  70 # mm

    def __init__(self, door=None):
        self.door = door
        super().__init__()

    def start(self, event=None):
        door = self.door
        if isinstance(event,DataEvent):
            door = event.data
        if isinstance(door, int):
            door ='Doorway-%d' % door
        if isinstance(door, str):
            doorway = self.robot.world.world_map.objects.get(door)
        elif isinstance(door, DoorwayObj):
            doorway = door
        if isinstance(doorway, DoorwayObj):
            self.object = doorway
        else:
            print("Error in DoorPass: no doorway named %s" % repr(door))
            raise ValueError(door,doorway)
        super().start(event)


    @staticmethod
    def calculate_gate(start_point, door, offset):
        """Returns closest gate point (gx, gy)"""
        (rx,ry) = start_point
        dx = door.x
        dy = door.y
        dtheta = door.theta
        pt1x = dx + offset * cos(dtheta)
        pt1y = dy + offset * sin(dtheta)
        pt2x = dx + offset * cos(dtheta+pi)
        pt2y = dy + offset * sin(dtheta+pi)
        dist1sq = (pt1x-rx)**2 + (pt1y-ry)**2
        dist2sq = (pt2x-rx)**2 + (pt2y-ry)**2
        if dist1sq < dist2sq:
            return (pt1x, pt1y, wrap_angle(dtheta+pi))
        else:
            return (pt2x, pt2y, dtheta)


    class AdjustLiftHeight(SetLiftHeight):
        # TODO: If lift is high, push it higher (we're carrying something).
        # If lift isn't high, drop it to zero
        def start(self, event=None):
            self.height = 0
            super().start(event)


    class TurnToGate(Turn):
        """Turn to the approach gate, or post success if we're already there."""
        def __init__(self,offset):
            self.offset = offset
            super().__init__(speed=Angle(radians=2.0))

        def start(self,event=None):
            (rx, ry, rtheta) = self.robot.world.particle_filter.pose_estimate()
            (gate_x, gate_y, _) = DoorPass.calculate_gate((rx,ry), self.parent.object, self.offset)
            bearing = atan2(gate_y-ry, gate_x-rx)
            turn = wrap_angle(bearing - rtheta)
            print('^^ TurnToGate: gate=(%.1f, %.1f)  offset=%.1f rtheta=%.1f  bearing=%.1f  turn=%.1f' %
                  (gate_x, gate_y, self.offset, rtheta*180/pi, bearing*180/pi, turn*180/pi))
            if False and abs(turn) < 0.1:
                self.angle = Angle(0)
                super().start(event)
                self.post_success()
            else:
                self.angle = Angle(radians=turn)
                super().start(event)


    class ForwardToGate(Forward):
        """Travel forward to reach the approach gate."""
        def __init__(self,offset):
            self.offset = offset
            super().__init__()

        def start(self,event=None):
            (rx, ry, rtheta) = self.robot.world.particle_filter.pose_estimate()
            (gate_x, gate_y, _) = DoorPass.calculate_gate((rx,ry), self.parent.object, self.offset)
            dist = sqrt((gate_x-rx)**2 + (gate_y-ry)**2)
            self.distance = distance_mm(dist)
            self.speed = speed_mmps(50)
            super().start(event)

    class TurnToMarker(Turn):
        """Use camera image and native pose to center the door marker."""
        def start(self,event=None):
            marker_ids = self.parent.object.marker_ids
            marker = self.robot.world.aruco.seen_marker_objects.get(marker_ids[0], None) or \
                     self.robot.world.aruco.seen_marker_objects.get(marker_ids[1], None)
            if not marker:
                self.angle = Angle(0)
                super().start(event)
                print("TurnToMarker failed to find marker %s or %s!" % marker_ids)
                self.post_failure()
                return
            else:
                print('TurnToMarker saw marker', marker)
            sensor_dist = marker.camera_distance
            sensor_bearing = atan2(marker.camera_coords[0],
                                   marker.camera_coords[2])
            x = self.robot.pose.position.x
            y = self.robot.pose.position.y
            theta = self.robot.pose.rotation.angle_z.radians
            direction = theta + sensor_bearing
            dx = sensor_dist * cos(direction)
            dy = sensor_dist * sin(direction)
            turn = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians)
            if abs(turn) < 0.5*pi/180:
                self.angle = Angle(0)
            else:
                self.angle = Angle(radians=turn)
            print("TurnToMarker %s turning by %.1f degrees" % (self.name, self.angle.degrees))
            super().start(event)

    class CheckCarrying(SetLiftHeight):
        def __init__(self):
            super().__init__()

        def start(self,event=None):
            if self.robot.carrying:
                self.height = 0.4 if self.robot.lift_ratio > 0.5 else 1
            super().start(event)

    class DriveThrough(Forward):
        """Travel forward to drive through the gate."""
        def __init__(self):
            super().__init__()

        def start(self,event=None):
            (rx, ry, rtheta) = self.robot.world.particle_filter.pose_estimate()
            (gate_x, gate_y, gate_theta) = DoorPass.calculate_gate((rx,ry), self.parent.object, 5)
            dist = sqrt((gate_x-rx)**2 + (gate_y-ry)**2)
            offset = 120
            delta_theta = wrap_angle(rtheta-(gate_theta+pi/2))
            delta_dist = abs(offset/sin(delta_theta))
            dist += delta_dist
            self.distance = distance_mm(dist)
            self.speed = speed_mmps(50)
            super().start(event)

    def setup(self):
        """
            droplift: self.AdjustLiftHeight() =N=>
                SetHeadAngle(0) =T(0.2)=> check_start  # Time for vision to process
    
            check_start: PilotCheckStart()
            check_start =S=> turn_to_gate1
            check_start =F=> Forward(-80) =C=> check_start2
    
            check_start2: PilotCheckStart()
            check_start2 =S=> turn_to_gate1
            check_start2 =F=> ParentFails()
    
            turn_to_gate1: self.TurnToGate(DoorPass.OUTER_GATE_DISTANCE) =C=>
                StateNode() =T(0.2)=> forward_to_gate1
    
            forward_to_gate1: self.ForwardToGate(DoorPass.OUTER_GATE_DISTANCE) =C=>
                StateNode() =T(0.2)=> {look_up, turn_to_gate2}
    
            # If we're carrying a cube, we lower the lift so we can see
            look_up: SetHeadAngle(35)
    
            turn_to_gate2: self.TurnToGate(DoorPass.INNER_GATE_DISTANCE) =C=>
                StateNode() =T(0.2)=> self.CheckCarrying() =C=> turn_to_marker1
    
            turn_to_marker1: self.TurnToMarker()
            turn_to_marker1 =C=> marker_forward1
            turn_to_marker1 =F=> marker_forward1
    
            marker_forward1: self.ForwardToGate(DoorPass.INNER_GATE_DISTANCE) =C=>
                SetHeadAngle(40) =C=> StateNode() =T(0.2)=> turn_to_marker2
    
            turn_to_marker2: self.TurnToMarker()
            turn_to_marker2 =C=> marker_forward2
            turn_to_marker2 =F=> marker_forward2
    
            marker_forward2: StateNode() =T(0.2)=>  {lower_head, through_door}
    
            lower_head: SetHeadAngle(0)
    
            through_door: self.DriveThrough()
    
            {lower_head, through_door} =C=> self.CheckCarrying() =C=> ParentCompletes()
    
        """
        
        # Code generated by genfsm on Tue Feb  4 21:25:16 2020:
        
        droplift = self.AdjustLiftHeight() .set_name("droplift") .set_parent(self)
        setheadangle1 = SetHeadAngle(0) .set_name("setheadangle1") .set_parent(self)
        check_start = PilotCheckStart() .set_name("check_start") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        check_start2 = PilotCheckStart() .set_name("check_start2") .set_parent(self)
        parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
        turn_to_gate1 = self.TurnToGate(DoorPass.OUTER_GATE_DISTANCE) .set_name("turn_to_gate1") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        forward_to_gate1 = self.ForwardToGate(DoorPass.OUTER_GATE_DISTANCE) .set_name("forward_to_gate1") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        look_up = SetHeadAngle(35) .set_name("look_up") .set_parent(self)
        turn_to_gate2 = self.TurnToGate(DoorPass.INNER_GATE_DISTANCE) .set_name("turn_to_gate2") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        checkcarrying1 = self.CheckCarrying() .set_name("checkcarrying1") .set_parent(self)
        turn_to_marker1 = self.TurnToMarker() .set_name("turn_to_marker1") .set_parent(self)
        marker_forward1 = self.ForwardToGate(DoorPass.INNER_GATE_DISTANCE) .set_name("marker_forward1") .set_parent(self)
        setheadangle2 = SetHeadAngle(40) .set_name("setheadangle2") .set_parent(self)
        statenode4 = StateNode() .set_name("statenode4") .set_parent(self)
        turn_to_marker2 = self.TurnToMarker() .set_name("turn_to_marker2") .set_parent(self)
        marker_forward2 = StateNode() .set_name("marker_forward2") .set_parent(self)
        lower_head = SetHeadAngle(0) .set_name("lower_head") .set_parent(self)
        through_door = self.DriveThrough() .set_name("through_door") .set_parent(self)
        checkcarrying2 = self.CheckCarrying() .set_name("checkcarrying2") .set_parent(self)
        parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(droplift) .add_destinations(setheadangle1)
        
        timertrans1 = TimerTrans(0.2) .set_name("timertrans1")
        timertrans1 .add_sources(setheadangle1) .add_destinations(check_start)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(check_start) .add_destinations(turn_to_gate1)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(check_start) .add_destinations(forward1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(forward1) .add_destinations(check_start2)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(check_start2) .add_destinations(turn_to_gate1)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(check_start2) .add_destinations(parentfails1)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(turn_to_gate1) .add_destinations(statenode1)
        
        timertrans2 = TimerTrans(0.2) .set_name("timertrans2")
        timertrans2 .add_sources(statenode1) .add_destinations(forward_to_gate1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(forward_to_gate1) .add_destinations(statenode2)
        
        timertrans3 = TimerTrans(0.2) .set_name("timertrans3")
        timertrans3 .add_sources(statenode2) .add_destinations(look_up,turn_to_gate2)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(turn_to_gate2) .add_destinations(statenode3)
        
        timertrans4 = TimerTrans(0.2) .set_name("timertrans4")
        timertrans4 .add_sources(statenode3) .add_destinations(checkcarrying1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(checkcarrying1) .add_destinations(turn_to_marker1)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(turn_to_marker1) .add_destinations(marker_forward1)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(turn_to_marker1) .add_destinations(marker_forward1)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(marker_forward1) .add_destinations(setheadangle2)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(setheadangle2) .add_destinations(statenode4)
        
        timertrans5 = TimerTrans(0.2) .set_name("timertrans5")
        timertrans5 .add_sources(statenode4) .add_destinations(turn_to_marker2)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(turn_to_marker2) .add_destinations(marker_forward2)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(turn_to_marker2) .add_destinations(marker_forward2)
        
        timertrans6 = TimerTrans(0.2) .set_name("timertrans6")
        timertrans6 .add_sources(marker_forward2) .add_destinations(lower_head,through_door)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(lower_head,through_door) .add_destinations(checkcarrying2)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(checkcarrying2) .add_destinations(parentcompletes1)
        
        return self
