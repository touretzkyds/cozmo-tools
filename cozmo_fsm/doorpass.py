from cozmo.util import Pose
from cv2 import Rodrigues
from numpy import matrix, tan, arctan2

from .nodes import *
from .transitions import *
from .transform import wrap_angle
from .pilot0 import *
from .worldmap import WallObj, DoorwayObj
from time import sleep

from math import sin, cos, atan2, pi, sqrt

class DoorPass(StateNode):
    """Pass through a doorway. Assumes the doorway is nearby and unobstructed."""
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
        elif instance(door, DoorwayObj):
            doorway = door
        if isinstance(doorway, DoorwayObj):
            self.object = doorway
        else:
            raise ValueError(door,doorway)
        super().start(event)

    def calculate_gate(self,offset):
        """Returns closest gate point (gx, gy)"""
        (rx,ry,_) = self.robot.world.particle_filter.pose
        dx = self.object.x
        dy = self.object.y
        dtheta = self.object.theta
        pt1x = dx + offset * cos(dtheta)
        pt1y = dy + offset * sin(dtheta)
        pt2x = dx + offset * cos(dtheta+pi)
        pt2y = dy + offset * sin(dtheta+pi)
        dist1sq = (pt1x-rx)**2 + (pt1y-ry)**2
        dist2sq = (pt2x-rx)**2 + (pt2y-ry)**2
        if dist1sq < dist2sq:
            return (pt1x, pt1y)
        else:
            return (pt2x, pt2y)

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
            (gate_x, gate_y) = self.parent.calculate_gate(self.offset)
            bearing = atan2(gate_y-ry, gate_x-rx)
            turn = wrap_angle(bearing - rtheta)
            print('^^ TurnToGate: gate=(%.1f, %.1f)  rtheta=%.1f  bearing=%.1f  turn=%.1f' %
                  (gate_x, gate_y, rtheta*180/pi, bearing*180/pi, turn*180/pi))
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
            (gate_x, gate_y) = self.parent.calculate_gate(self.offset)
            dist = sqrt((gate_x-rx)**2 + (gate_y-ry)**2)
            self.distance = distance_mm(dist)
            self.speed = speed_mmps(50)
            super().start(event)

    class TurnToMarker(Turn):
        """Use camera image and native pose to center the door marker."""
        def start(self,event=None):
            marker_id = self.parent.object.marker_id
            marker = self.robot.world.aruco.seen_marker_objects.get(marker_id, None)
            if not marker:
                self.angle = Angle(0)
                super().start(event)
                print("TurnToMarker failed to find marker %s!" % marker_id)
                self.post_failure()
                return
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
            print("TurnTOMarker %s turning by %.1f degrees" % (self.name, self.angle.degrees))
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
    
            turn_to_gate1: self.TurnToGate(150) =C=> StateNode() =T(0.2)=> forward_to_gate1
            forward_to_gate1: self.ForwardToGate(150) =C=> StateNode() =T(0.2)=> {look_up, turn_to_gate2}
    
            # If we're carrying a cube, should we lower the lift so we can see?
            look_up: SetHeadAngle(35)
    
            turn_to_gate2: self.TurnToGate(70) =C=> StateNode() =T(0.2)=> turn_to_marker1
    
            turn_to_marker1: self.TurnToMarker() =C=>
                self.ForwardToGate(70) =C=> SetHeadAngle(40) =C=> StateNode() =T(0.2)=> turn_to_marker2
    
            turn_to_marker2: self.TurnToMarker() =C=> StateNode() =T(0.2)=>  {lower_head, through_door}
    
            lower_head: SetHeadAngle(0)
    
            through_door: Forward(150)
    
            {lower_head, through_door} =C=> ParentCompletes()
    
        """
        
        # Code generated by genfsm on Wed May  1 05:40:41 2019:
        
        droplift = self.AdjustLiftHeight() .set_name("droplift") .set_parent(self)
        setheadangle1 = SetHeadAngle(0) .set_name("setheadangle1") .set_parent(self)
        check_start = PilotCheckStart() .set_name("check_start") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        check_start2 = PilotCheckStart() .set_name("check_start2") .set_parent(self)
        parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
        turn_to_gate1 = self.TurnToGate(150) .set_name("turn_to_gate1") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        forward_to_gate1 = self.ForwardToGate(150) .set_name("forward_to_gate1") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        look_up = SetHeadAngle(35) .set_name("look_up") .set_parent(self)
        turn_to_gate2 = self.TurnToGate(70) .set_name("turn_to_gate2") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        turn_to_marker1 = self.TurnToMarker() .set_name("turn_to_marker1") .set_parent(self)
        forwardtogate1 = self.ForwardToGate(70) .set_name("forwardtogate1") .set_parent(self)
        setheadangle2 = SetHeadAngle(40) .set_name("setheadangle2") .set_parent(self)
        statenode4 = StateNode() .set_name("statenode4") .set_parent(self)
        turn_to_marker2 = self.TurnToMarker() .set_name("turn_to_marker2") .set_parent(self)
        statenode5 = StateNode() .set_name("statenode5") .set_parent(self)
        lower_head = SetHeadAngle(0) .set_name("lower_head") .set_parent(self)
        through_door = Forward(150) .set_name("through_door") .set_parent(self)
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
        timertrans4 .add_sources(statenode3) .add_destinations(turn_to_marker1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(turn_to_marker1) .add_destinations(forwardtogate1)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(forwardtogate1) .add_destinations(setheadangle2)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(setheadangle2) .add_destinations(statenode4)
        
        timertrans5 = TimerTrans(0.2) .set_name("timertrans5")
        timertrans5 .add_sources(statenode4) .add_destinations(turn_to_marker2)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(turn_to_marker2) .add_destinations(statenode5)
        
        timertrans6 = TimerTrans(0.2) .set_name("timertrans6")
        timertrans6 .add_sources(statenode5) .add_destinations(lower_head,through_door)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(lower_head,through_door) .add_destinations(parentcompletes1)
        
        return self

