from cozmo.util import Pose
from cv2 import Rodrigues
from numpy import matrix, tan, arctan2

from .nodes import *
from .transitions import *
from .geometry import wrap_angle
from .pilot import PilotToPose, PilotCheckStart
from .worldmap import WallObj
from time import sleep


class GoToWall(StateNode):
    def __init__(self, wall=None, door=-1):
        super().__init__()
        self.object = wall
        if isinstance(wall, int):
            self.wall_name = 'Wall-'+str(wall)
        elif isinstance(wall, str):
            self.wall_name = wall
        elif wall is None:
            self.wall_name = None
        else:
            raise ValueError('wall should be an integer, string, or None, not %r' % wall)
        self.door_id = door

    def start(self,event=None):
        if self.wall_name is None:
            self.wall_name = self.find_closest_wall()

        if self.wall_name in self.robot.world.world_map.objects:
            self.wobj = self.robot.world.world_map.objects[self.wall_name]
        else:
            print("GoToWall: %s is not in the map." % self.wall_name)
            super().start(event)
            self.post_failure()
            return

        if self.wobj != -1:
            if self.door_id == -1:
                self.door_coordinates = -1
                print("Going to closest door")
                super().start(event)
            elif self.door_id in self.wobj.door_ids:
                self.door_coordinates = self.wobj.markers[self.door_id][1]
                print(self.door_coordinates)
                super().start(event)
            else:
                print(self.door_id,"is not a door")
                super().start(event)
                self.post_failure()

    def find_closest_wall(self):
        (x,y,theta) = self.robot.world.particle_filter.pose
        walls = []
        for obj in self.robot.world.world_map.objects.values():
            if isinstance(obj,WallObj):
                distsq = (x-obj.x)**2 + (y-obj.y)**2
                walls.append((distsq,obj))
        walls = sorted(walls, key=lambda x: x[0])
        return 'Wall-%d' % walls[0][1].id if walls else None

    def pick_side(self, dist):
        wall = self.object
        door_coordinates = self.door_coordinates
        x = self.wobj.x 
        y = self.wobj.y
        ang = self.wobj.theta
        rx = self.robot.world.particle_filter.pose[0]
        ry = self.robot.world.particle_filter.pose[1]
        l = self.wobj.length/2
        if door_coordinates == -1:
            door_ids = self.wobj.door_ids
            sides = []
            for id in door_ids:
               door_coordinates = self.wobj.markers[id][1]
               s = self.wobj.markers[id][0]
               sides.append((x - s*cos(ang)*dist - sin(ang)*(l - door_coordinates[0]),
                             y - s*sin(ang)*dist + cos(ang)*( l - door_coordinates[0]),
                             wrap_angle(ang+(1-s)*pi/2), id))

            sorted_sides = sorted(sides, key=lambda pt: (pt[0]-rx)**2 + (pt[1]-ry)**2)
            self.door_id = sorted_sides[0][3]
            self.door_coordinates = self.wobj.markers[self.door_id][1]
            print("Going to door", self.door_id )
            shortest = sorted_sides[0][0:3]
        else:
            side1 = (x + cos(ang)*dist - sin(ang)*(self.wobj.length/2 - door_coordinates[0]),
                     y + sin(ang)*dist + cos(ang)*( self.wobj.length/2 - door_coordinates[0]),
                     wrap_angle(ang+pi))
            side2 = (x - cos(ang)*dist - sin(ang)*(self.wobj.length/2 - door_coordinates[0]),
                     y - sin(ang)*dist + cos(ang)*( self.wobj.length/2 - door_coordinates[0]),
                     wrap_angle(ang))
            sides = [side1, side2]
            sorted_sides = sorted(sides, key=lambda pt: (pt[0]-rx)**2 + (pt[1]-ry)**2)
            shortest = sorted_sides[0]
        return shortest

    class TurnToSide(Turn):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            wall = self.parent.object
            wobj = self.parent.wobj
            (x, y, ang) = self.parent.pick_side(150)
            dtheta = wrap_angle(ang - self.robot.world.particle_filter.pose_estimate()[2])
            if abs(dtheta) > 0.1:
                self.angle = Angle(dtheta)
                super().start(event)
            else:
                self.angle = Angle(0)
                super().start(event)
                self.post_success()


    class GoToSide(PilotToPose):
        def __init__(self):
            super().__init__(None)

        def start(self, event=None):
            wall = self.parent.object
            print('Selected wall',self.parent.wobj)
            (x, y, theta) = self.parent.pick_side(150)

            self.target_pose = Pose(x, y, self.robot.pose.position.z,
                                    angle_z=Angle(radians = wrap_angle(theta)))
            print('Traveling to',self.target_pose)
            super().start(event)


    class ReportPosition(StateNode):
        def start(self,event=None):
            super().start(event)
            wall = self.parent.object
            wobj = self.parent.wobj
            cx = wobj.x
            cy = wobj.y
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            dx = cx - rx
            dy = cy - ry
            dist = math.sqrt(dx*dx + dy*dy)
            bearing = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) * 180/pi
            print('wall at (%5.1f,%5.1f)  robot at (%5.1f,%5.1f)  dist=%5.1f  brg=%5.1f' %
                  (cx, cy, rx, ry, dist, bearing))


    class TurnToWall(Turn):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            door_id = self.parent.door_id

            for i in range(4):
                if door_id not in self.robot.world.aruco.seen_marker_ids:
                    #Check three times that the marker is not visible
                    if i > 2:
                        self.angle = Angle(degrees=0)
                        super().start(event)
                        self.post_failure()
                        break
                    else:
                        sleep(0.1)
                        continue
                else:
                    while True:
                        rx = self.robot.pose.position.x
                        ry = self.robot.pose.position.y
                        rt = self.robot.pose.rotation.angle_z.radians

                        marker = self.robot.world.aruco.seen_marker_objects.get(door_id,0)
                        if marker!=0:
                            break

                    sensor_dist = marker.camera_distance
                    sensor_bearing = atan2(marker.camera_coords[0],
                                           marker.camera_coords[2])
                    sensor_orient = - marker.opencv_rotation[1] * (pi/180)

                    direction = rt + sensor_bearing
                    dx = sensor_dist * cos(direction)
                    dy = sensor_dist * sin(direction)
                    cx = rx + dx
                    cy = ry + dy
                    dist = math.sqrt(dx*dx + dy*dy)
                    self.angle = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) \
                                 * 180/pi
                    if abs(self.angle) < 2:
                        self.angle = 0
                    self.angle = Angle(degrees=self.angle)
                    #print("TurnToWall", self.angle)
                    super().start(event)
                    break


    class ForwardToWall(Forward):
        def __init__(self, offset):
            self.offset = offset
            super().__init__()

        def start(self, event=None):
            if self.running: return
            door_id = self.parent.door_id
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            rt = self.robot.pose.rotation.angle_z.radians
            if door_id in self.robot.world.aruco.seen_marker_objects:
                marker = self.robot.world.aruco.seen_marker_objects[door_id]
                sensor_dist = marker.camera_distance
                sensor_bearing = atan2(marker.camera_coords[0],
                                       marker.camera_coords[2])
                sensor_orient = - marker.opencv_rotation[1] * (pi/180)

                direction = rt + sensor_bearing
                dx = sensor_dist * cos(direction)
                dy = sensor_dist * sin(direction)
                cx = rx + dx
                cy = ry + dy
                dist = math.sqrt(dx*dx + dy*dy)
                self.distance = Distance(sqrt(dx*dx + dy*dy) - self.offset)
                super().start(event)
            else:
                self.distance = Distance(0)
                super().start(event)
                self.post_failure()


    class FindWall(SetHeadAngle):
        def __init__(self):
            super().__init__()

        def start(self, event=None):
            if self.running: return
            door_id = self.parent.door_id
            if door_id not in self.robot.world.aruco.seen_marker_ids:
                #print('Looking higher for wall')
                if self.robot.head_angle.degrees < 40:
                    self.angle = Angle(self.robot.head_angle.radians + 0.15)
                    super().start(event)
                else:
                    self.angle = self.robot.head_angle
                    super().start(event)
            else:
                self.angle = self.robot.head_angle
                super().start(event)

    # GoToWall state machine
    def setup(self):
        """
            droplift: SetLiftHeight(0) =T(0.5)=> check_start    # time for vision to set up world map
    
            check_start: PilotCheckStart()
            check_start =S=> SetHeadAngle(0) =C=> turn_to_side
            check_start =F=> Forward(-80) =C=> check_start
    
            turn_to_side: self.TurnToSide()
            turn_to_side =C=> turn_to_side
            turn_to_side =S=> self.ReportPosition() =N=> go_side
    
            go_side: self.GoToSide() =C=> self.TurnToSide() =C=> lookup
    
            lookup:  SetHeadAngle(35) =C=> find
    
            find: self.TurnToWall() =C=>approach
            find =F=> Forward(-80) =C=> StateNode() =T(1)=> find2
    
            find2: self.TurnToWall() =C=>approach
            find2 =F=> Forward(-80) =C=> Say("No Door trying again") =C=> turn_to_side
    
            approach: self.ForwardToWall(100) =C=> self.FindWall() =C=>
                self.TurnToWall() =C=> self.FindWall() =C=>
                self.ForwardToWall(70) =C=> self.FindWall() =C=>
                self.TurnToWall()=C=> end
            approach =F=> end
    
            end: SetHeadAngle(0) =C=> Forward(150) =C=> ParentCompletes()
        """
        
        # Code generated by genfsm on Fri Apr  6 04:49:50 2018:
        
        droplift = SetLiftHeight(0) .set_name("droplift") .set_parent(self)
        check_start = PilotCheckStart() .set_name("check_start") .set_parent(self)
        setheadangle1 = SetHeadAngle(0) .set_name("setheadangle1") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        turn_to_side = self.TurnToSide() .set_name("turn_to_side") .set_parent(self)
        reportposition1 = self.ReportPosition() .set_name("reportposition1") .set_parent(self)
        go_side = self.GoToSide() .set_name("go_side") .set_parent(self)
        turntoside1 = self.TurnToSide() .set_name("turntoside1") .set_parent(self)
        lookup = SetHeadAngle(35) .set_name("lookup") .set_parent(self)
        find = self.TurnToWall() .set_name("find") .set_parent(self)
        forward2 = Forward(-80) .set_name("forward2") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        find2 = self.TurnToWall() .set_name("find2") .set_parent(self)
        forward3 = Forward(-80) .set_name("forward3") .set_parent(self)
        say1 = Say("No Door trying again") .set_name("say1") .set_parent(self)
        approach = self.ForwardToWall(100) .set_name("approach") .set_parent(self)
        findwall1 = self.FindWall() .set_name("findwall1") .set_parent(self)
        turntowall1 = self.TurnToWall() .set_name("turntowall1") .set_parent(self)
        findwall2 = self.FindWall() .set_name("findwall2") .set_parent(self)
        forwardtowall1 = self.ForwardToWall(70) .set_name("forwardtowall1") .set_parent(self)
        findwall3 = self.FindWall() .set_name("findwall3") .set_parent(self)
        turntowall2 = self.TurnToWall() .set_name("turntowall2") .set_parent(self)
        end = SetHeadAngle(0) .set_name("end") .set_parent(self)
        forward4 = Forward(150) .set_name("forward4") .set_parent(self)
        parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
        
        timertrans1 = TimerTrans(0.5) .set_name("timertrans1")
        timertrans1 .add_sources(droplift) .add_destinations(check_start)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(check_start) .add_destinations(setheadangle1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(setheadangle1) .add_destinations(turn_to_side)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(check_start) .add_destinations(forward1)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(forward1) .add_destinations(check_start)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(turn_to_side) .add_destinations(turn_to_side)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(turn_to_side) .add_destinations(reportposition1)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(reportposition1) .add_destinations(go_side)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(go_side) .add_destinations(turntoside1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(turntoside1) .add_destinations(lookup)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(lookup) .add_destinations(find)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(find) .add_destinations(approach)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(find) .add_destinations(forward2)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(forward2) .add_destinations(statenode1)
        
        timertrans2 = TimerTrans(1) .set_name("timertrans2")
        timertrans2 .add_sources(statenode1) .add_destinations(find2)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(find2) .add_destinations(approach)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(find2) .add_destinations(forward3)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(forward3) .add_destinations(say1)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(say1) .add_destinations(turn_to_side)
        
        completiontrans12 = CompletionTrans() .set_name("completiontrans12")
        completiontrans12 .add_sources(approach) .add_destinations(findwall1)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(findwall1) .add_destinations(turntowall1)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(turntowall1) .add_destinations(findwall2)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(findwall2) .add_destinations(forwardtowall1)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(forwardtowall1) .add_destinations(findwall3)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(findwall3) .add_destinations(turntowall2)
        
        completiontrans18 = CompletionTrans() .set_name("completiontrans18")
        completiontrans18 .add_sources(turntowall2) .add_destinations(end)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(approach) .add_destinations(end)
        
        completiontrans19 = CompletionTrans() .set_name("completiontrans19")
        completiontrans19 .add_sources(end) .add_destinations(forward4)
        
        completiontrans20 = CompletionTrans() .set_name("completiontrans20")
        completiontrans20 .add_sources(forward4) .add_destinations(parentcompletes1)
        
        return self

class Exploren(StateNode):
    
    def __init__(self):
        self.current_wall = None
        self.to_do_wall = []
        self.done_wall = []
        super().__init__()

    class Think(StateNode):
        def start(self,event=None):
            super().start(event)
            for key, val in self.robot.world.world_map.objects.items():
                if isinstance(val,WallObj) and val.id not in self.parent.done_wall and val.id not in self.parent.to_do_wall:
                    self.parent.to_do_wall.append(val)
                    print(val.id)

            if len(self.parent.to_do_wall) > 0:
                wall = self.parent.to_do_wall.pop()
                self.parent.current_wall = wall.id
                self.parent.done_wall.append(wall.id)
                print(self.parent.to_do_wall,self.parent.current_wall,self.parent.done_wall)
                self.post_failure()
            else:
                self.post_success()

    class Go(GoToWall):
        def __init__(self):
            super().__init__()
            
        def start(self,event=None):
            self.object = self.parent.current_wall
            self.wall_name = 'Wall-'+str(self.object)
            self.door_id = -1
            super().start(event)

    # Explore state machine
    def setup(self):
        """
            look: LookAroundInPlace(stop_on_exit=False) =T(5)=> StopBehavior() =C=> think
    
            think: self.Think()
            think =F=> go
            think =S=> end
    
            go: self.Go() =C=> look
    
            end: Say("Done") =C=> ParentCompletes()
        """
        
        # Code generated by genfsm on Fri Apr  6 04:49:50 2018:
        
        look = LookAroundInPlace(stop_on_exit=False) .set_name("look") .set_parent(self)
        stopbehavior1 = StopBehavior() .set_name("stopbehavior1") .set_parent(self)
        think = self.Think() .set_name("think") .set_parent(self)
        go = self.Go() .set_name("go") .set_parent(self)
        end = Say("Done") .set_name("end") .set_parent(self)
        parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
        
        timertrans3 = TimerTrans(5) .set_name("timertrans3")
        timertrans3 .add_sources(look) .add_destinations(stopbehavior1)
        
        completiontrans21 = CompletionTrans() .set_name("completiontrans21")
        completiontrans21 .add_sources(stopbehavior1) .add_destinations(think)
        
        failuretrans5 = FailureTrans() .set_name("failuretrans5")
        failuretrans5 .add_sources(think) .add_destinations(go)
        
        successtrans3 = SuccessTrans() .set_name("successtrans3")
        successtrans3 .add_sources(think) .add_destinations(end)
        
        completiontrans22 = CompletionTrans() .set_name("completiontrans22")
        completiontrans22 .add_sources(go) .add_destinations(look)
        
        completiontrans23 = CompletionTrans() .set_name("completiontrans23")
        completiontrans23 .add_sources(end) .add_destinations(parentcompletes2)
        
        return self
