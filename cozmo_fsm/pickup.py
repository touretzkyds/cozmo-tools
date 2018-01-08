from cozmo.util import Pose

from .nodes import *
from .transitions import *
from .transform import wrap_angle
from .pilot import PilotToPose, PilotCheckStart
from .worldmap import LightCubeObj
from .doorpass import WallPilotToPose
from .trace import tracefsm

from math import sin, cos, atan2, pi, sqrt

class GoToCube(StateNode):

    def __init__(self, cube=None):
        self.object = cube
        super().__init__()

    def start(self, event=None):
        # self.object will be set up by the parent of this node
        if isinstance(self.object, LightCubeObj):
            self.object = self.object.sdk_obj
        self.children['looker'].object = self.object
        tracefsm(1)
        super().start(event)

    def pick_side(self, dist, use_world_map):
        "NOTE: This code is only correct for upright cubes"
        cube = self.object
        if use_world_map:
            cobj = self.robot.world.world_map.objects[cube]
            x = cobj.x
            y = cobj.y
            ang = cobj.theta
            rx = self.robot.world.particle_filter.pose[0]
            ry = self.robot.world.particle_filter.pose[1]
        else:
            x = cube.pose.position.x
            y = cube.pose.position.y
            ang = cube.pose.rotation.angle_z.radians
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
        side1 = [ (x + cos(ang)*dist), (y + sin(ang)*dist), ang + pi   ]
        side2 = [ (x - cos(ang)*dist), (y - sin(ang)*dist), ang        ]
        side3 = [ (x + sin(ang)*dist), (y - cos(ang)*dist), ang + pi/2 ]
        side4 = [ (x - sin(ang)*dist), (y + cos(ang)*dist), ang - pi/2 ]
        sides = (side1, side2, side3, side4)
        sorted_sides = sorted(sides, key=lambda pt: (pt[0]-rx)**2 + (pt[1]-ry)**2)
        return sorted_sides[0]

    def almost_docked(self, side, use_world_map):
        """Returns True if we're almost docked with the cube so we don't
        need to check for collisions."""
        if use_world_map:
            rx = self.robot.world.particle_filter.pose[0]
            ry = self.robot.world.particle_filter.pose[1]
            rtheta = self.robot.world.particle_filter.pose[2]
        else:
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            rtheta = self.robot.pose.rotation.angle_z.radians
        dist = math.sqrt((rx-side[0])**2 + (ry-side[1])**2)
        relative_angle = abs(wrap_angle(rtheta-side[2]) % (pi/2)) * (180/pi)
        return (dist < 100) and (relative_angle < 10)

    class GoToSide(PilotToPose):
        def __init__(self):
            super().__init__(None)

        def start(self, event=None):
            cube = self.parent.object
            print('Selected cube',self.robot.world.world_map.objects[cube])
            (x, y, theta) = self.parent.pick_side(100, use_world_map=True)
            self.target_pose = Pose(x, y, self.robot.pose.position.z,
                                    angle_z=Angle(radians = wrap_angle(theta)))
            print('pickup.GoToSide: traveling to (%.1f, %.1f) @ %.1f deg.' %
                  (self.target_pose.position.x, self.target_pose.position.y,
                   self.target_pose.rotation.angle_z.degrees))
            super().start(event)

    class ReportPosition(StateNode):
        def __init__(self,id=None):
            super().__init__()
            self.id_string = id + ': ' if id else ''

        def start(self,event=None):
            super().start(event)
            cube = self.parent.object
            if cube.is_visible:
                vis = 'visible'
            else:
                vis = 'not visible'
            cx = cube.pose.position.x
            cy = cube.pose.position.y
            rx = self.robot.pose.position.x
            ry = self.robot.pose.position.y
            dx = cx - rx
            dy = cy - ry
            dist = math.sqrt(dx*dx + dy*dy)
            bearing = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) * 180/pi
            print('%scube %s at (%5.1f,%5.1f)  robot at (%5.1f,%5.1f)  dist=%5.1f  brg=%5.1f' %
                  (self.id_string, vis, cx, cy, rx, ry, dist, bearing))

    class TurnToCube(SmallTurn):
        def __init__(self, offset=0, check_vis=False):
            self.offset = offset
            self.check_vis = check_vis
            super().__init__()

        def start(self, event=None):
            if self.running: return
            cube = self.parent.object
            if self.check_vis and not cube.is_visible:
                print('** TurnToCube could not see the cube.')
                self.angle = None # Angle(0)
                super().start(event)
                self.post_failure()
            else:
                (cx, cy, _) = self.parent.pick_side(self.offset, False)
                rx = self.robot.pose.position.x
                ry = self.robot.pose.position.y
                dx = cx - rx
                dy = cy - ry
                dist = math.sqrt(dx*dx + dy*dy)
                self.angle = wrap_angle(atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians) \
                             * 180/pi
                if abs(self.angle) < 2:
                    self.angle = 0
                print('TurnToCube: cube at (%5.1f,%5.1f)  robot at (%5.1f,%5.1f)  dist=%5.1f  angle=%5.1f' %
                      (cx, cy, rx, ry, dist, self.angle))
                super().start(event)

    class CheckAlmostDocked(StateNode):
        def start(self, event=None):
            if self.running: return
            super().start(event)
            cube = self.parent.object
            if not cube.is_visible:
                self.post_failure()
            side = self.parent.pick_side(25, False)
            if self.parent.almost_docked(side,False):
                self.post_success()
            else:
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
            self.distance = Distance(sqrt(dx*dx + dy*dy) - self.offset)
            super().start(event)

    def setup(self):
        """
            droplift: SetLiftHeight(0) =T(0.5)=>    # allow time for vision to set up world map
               {looker, check_almost_docked_0}
    
            looker: LookAtObject()
    
            check_almost_docked_0: StateNode() =T(1)=> check_almost_docked
            
            check_almost_docked: self.CheckAlmostDocked()
            check_almost_docked =S=> go_cube2
            check_almost_docked =F=> check_start
    
            check_start: PilotCheckStart()
            check_start =S=> go_side
            check_start =F=> Forward(-80) =C=> StateNode() =T(0.5)=> check_start
    
            go_side: self.GoToSide()
            go_side =F=> ParentFails()
            go_side =C=> self.ReportPosition('go_side')
                =T(0.75)=> self.ReportPosition('go_side')
                =T(0.75)=> self.ReportPosition('go_side')
                =N=> go_cube1
    
            go_cube1: self.TurnToCube(0,True) =C=>
                self.ReportPosition('go_cube1') =T(0.75)=> self.ReportPosition('go_cube1')
                =T(0.75)=> self.ReportPosition('go_cube1') =N=> approach
            go_cube1 =F=> Forward(-80) =C=> StateNode() =T(1)=> go_cube2
    
            approach: self.ForwardToCube(60) =C=>
                self.ReportPosition('approach') =T(0.75)=> self.ReportPosition('approach') =T(0.75)=>
                self.ReportPosition('approach') =N=>
                self.TurnToCube(0,False) =C=> self.ForwardToCube(20) =C=> end
    
            go_cube2: self.TurnToCube(0,True)
            go_cube2 =F=> Print("Cube Lost") =N=> ParentFails()
            go_cube2 =C=> self.ForwardToCube(60) =C=>
                self.TurnToCube(0,False) =C=> self.ForwardToCube(20) =C=> end
    
            end: ParentCompletes()
        """
        
        # Code generated by genfsm on Sat Jan  6 14:15:07 2018:
        
        droplift = SetLiftHeight(0) .set_name("droplift") .set_parent(self)
        looker = LookAtObject() .set_name("looker") .set_parent(self)
        check_almost_docked_0 = StateNode() .set_name("check_almost_docked_0") .set_parent(self)
        check_almost_docked = self.CheckAlmostDocked() .set_name("check_almost_docked") .set_parent(self)
        check_start = PilotCheckStart() .set_name("check_start") .set_parent(self)
        forward1 = Forward(-80) .set_name("forward1") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        go_side = self.GoToSide() .set_name("go_side") .set_parent(self)
        parentfails1 = ParentFails() .set_name("parentfails1") .set_parent(self)
        reportposition1 = self.ReportPosition('go_side') .set_name("reportposition1") .set_parent(self)
        reportposition2 = self.ReportPosition('go_side') .set_name("reportposition2") .set_parent(self)
        reportposition3 = self.ReportPosition('go_side') .set_name("reportposition3") .set_parent(self)
        go_cube1 = self.TurnToCube(0,True) .set_name("go_cube1") .set_parent(self)
        reportposition4 = self.ReportPosition('go_cube1') .set_name("reportposition4") .set_parent(self)
        reportposition5 = self.ReportPosition('go_cube1') .set_name("reportposition5") .set_parent(self)
        reportposition6 = self.ReportPosition('go_cube1') .set_name("reportposition6") .set_parent(self)
        forward2 = Forward(-80) .set_name("forward2") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        approach = self.ForwardToCube(60) .set_name("approach") .set_parent(self)
        reportposition7 = self.ReportPosition('approach') .set_name("reportposition7") .set_parent(self)
        reportposition8 = self.ReportPosition('approach') .set_name("reportposition8") .set_parent(self)
        reportposition9 = self.ReportPosition('approach') .set_name("reportposition9") .set_parent(self)
        turntocube1 = self.TurnToCube(0,False) .set_name("turntocube1") .set_parent(self)
        forwardtocube1 = self.ForwardToCube(20) .set_name("forwardtocube1") .set_parent(self)
        go_cube2 = self.TurnToCube(0,True) .set_name("go_cube2") .set_parent(self)
        print1 = Print("Cube Lost") .set_name("print1") .set_parent(self)
        parentfails2 = ParentFails() .set_name("parentfails2") .set_parent(self)
        forwardtocube2 = self.ForwardToCube(60) .set_name("forwardtocube2") .set_parent(self)
        turntocube2 = self.TurnToCube(0,False) .set_name("turntocube2") .set_parent(self)
        forwardtocube3 = self.ForwardToCube(20) .set_name("forwardtocube3") .set_parent(self)
        end = ParentCompletes() .set_name("end") .set_parent(self)
        
        timertrans1 = TimerTrans(0.5) .set_name("timertrans1")
        timertrans1 .add_sources(droplift) .add_destinations(looker,check_almost_docked_0)
        
        timertrans2 = TimerTrans(1) .set_name("timertrans2")
        timertrans2 .add_sources(check_almost_docked_0) .add_destinations(check_almost_docked)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(check_almost_docked) .add_destinations(go_cube2)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(check_almost_docked) .add_destinations(check_start)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(check_start) .add_destinations(go_side)
        
        failuretrans2 = FailureTrans() .set_name("failuretrans2")
        failuretrans2 .add_sources(check_start) .add_destinations(forward1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(forward1) .add_destinations(statenode1)
        
        timertrans3 = TimerTrans(0.5) .set_name("timertrans3")
        timertrans3 .add_sources(statenode1) .add_destinations(check_start)
        
        failuretrans3 = FailureTrans() .set_name("failuretrans3")
        failuretrans3 .add_sources(go_side) .add_destinations(parentfails1)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(go_side) .add_destinations(reportposition1)
        
        timertrans4 = TimerTrans(0.75) .set_name("timertrans4")
        timertrans4 .add_sources(reportposition1) .add_destinations(reportposition2)
        
        timertrans5 = TimerTrans(0.75) .set_name("timertrans5")
        timertrans5 .add_sources(reportposition2) .add_destinations(reportposition3)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(reportposition3) .add_destinations(go_cube1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(go_cube1) .add_destinations(reportposition4)
        
        timertrans6 = TimerTrans(0.75) .set_name("timertrans6")
        timertrans6 .add_sources(reportposition4) .add_destinations(reportposition5)
        
        timertrans7 = TimerTrans(0.75) .set_name("timertrans7")
        timertrans7 .add_sources(reportposition5) .add_destinations(reportposition6)
        
        nulltrans2 = NullTrans() .set_name("nulltrans2")
        nulltrans2 .add_sources(reportposition6) .add_destinations(approach)
        
        failuretrans4 = FailureTrans() .set_name("failuretrans4")
        failuretrans4 .add_sources(go_cube1) .add_destinations(forward2)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(forward2) .add_destinations(statenode2)
        
        timertrans8 = TimerTrans(1) .set_name("timertrans8")
        timertrans8 .add_sources(statenode2) .add_destinations(go_cube2)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(approach) .add_destinations(reportposition7)
        
        timertrans9 = TimerTrans(0.75) .set_name("timertrans9")
        timertrans9 .add_sources(reportposition7) .add_destinations(reportposition8)
        
        timertrans10 = TimerTrans(0.75) .set_name("timertrans10")
        timertrans10 .add_sources(reportposition8) .add_destinations(reportposition9)
        
        nulltrans3 = NullTrans() .set_name("nulltrans3")
        nulltrans3 .add_sources(reportposition9) .add_destinations(turntocube1)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(turntocube1) .add_destinations(forwardtocube1)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(forwardtocube1) .add_destinations(end)
        
        failuretrans5 = FailureTrans() .set_name("failuretrans5")
        failuretrans5 .add_sources(go_cube2) .add_destinations(print1)
        
        nulltrans4 = NullTrans() .set_name("nulltrans4")
        nulltrans4 .add_sources(print1) .add_destinations(parentfails2)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(go_cube2) .add_destinations(forwardtocube2)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(forwardtocube2) .add_destinations(turntocube2)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(turntocube2) .add_destinations(forwardtocube3)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(forwardtocube3) .add_destinations(end)
        
        return self

class SetCarrying(StateNode):
    def __init__(self,object=None):
        self.object = object
        super().__init__()
        
    def start(self, event=None):
        self.robot.carrying = self.object
        self.object.update_from_sdk = False
        super().start(event)
        self.post_completion()

class SetNotCarrying(StateNode):
    def start(self,event=None):
        self.robot.carrying = None
        super().start(event)
        self.post_completion()

class PickUpCube(StateNode):

    class StoreImagePatch(StateNode):
        def __init__(self,params,attr_name):
            self.params = params
            self.attr_name = attr_name
            super().__init__()

        def start(self,event=None):
            array = np.array(self.robot.world.latest_image.raw_image)
            row_index = self.params[0]
            row = array[row_index,:,0]
            setattr(self.parent,  self.attr_name, row)
            super().start(event)

    class OldVerifyPickUp(StateNode):
        def start(self,event=None):
            super().start(event)
            before = self.parent.before
            bsum = int(before.sum())
            after = self.parent.after
            asum = int(after.sum())
            diff = abs(asum-bsum)
            print('>>> Verify: before:',bsum,' after:', asum, ' diff=',diff)
            if diff > 15000:
                self.post_success()
            else:
                self.post_failure()

    class VerifyPickup(StateNode):
        def probe_column(self, im, col, row_start, row_end):
            """
            Probe one column of the image, looking for the top horizontal
            black bar of the cube marker.  This bar should be 22-24 pixels
            thick.  Use adaptive thresholding by sorting the pixels and
            finding the darkest ones to set the black threshold.
            """
            pixels = [im[r,col,0] for r in range(row_start,row_end)]
            # print('Column ',col,':',sep='')
            # [print('%4d' % i,end='') for i in pixels]
            pixels.sort()
            black_index = 20  # start with the 20th darkest pixel
            black_thresh = pixels[black_index] + 4
            last_pixel = len(pixels) - 1
            while black_index < last_pixel and pixels[black_index+1] <= black_thresh:
                black_index += 1
                black_thresh = pixels[black_index] + 4
            white_thresh = pixels[black_index] + 20
            # print('\nblack_thresh=',black_thresh,'  black_index=',black_index, '  white_thresh=',white_thresh)
            # initial white segment
            row = row_start
            while row <= row_end:
                if im[row,col,0] < white_thresh: break
                row += 1
            # gray transition
            for i in range(5):
                if im[row,col,0] <= black_thresh: break
                row += 1
            # black segment
            black_start = row
            run_length = 0
            while row <= row_end:
                if im[row,col,0] > black_thresh: break
                run_length += 1
                row += 1
            if row > row_end:  # black to the end; no final white
                return -1
            run_length = row - black_start
            # gray transition
            for i in range(5):
                if im[row,col,0] >= white_thresh: break
                row += 1
            # final white segment
            while row <= row_end:
                if im[row,col,0] < white_thresh:
                    #print('col=',col,'  row=',row,'  im =',im[row,col],
                    #      ' white_thresh=',white_thresh,' black_thresh=',black_thresh,'  run_length=',run_length)
                    return -2
                row += 1
            return run_length
                    
        def start(self,event=None):
            super().start(event)
            im = np.array(self.robot.world.latest_image.raw_image)
            min_length = 20
            max_length = 26
            bad_runs = 0
            print('Verifying.  Runs: ',end='')
            for col in range(100,220,20):
                run_length = self.probe_column(im, col, 10, 100)
                print(run_length,' ',end='')
                if run_length < min_length or run_length > max_length:
                    bad_runs += 1
            print('  bad_runs:', bad_runs)
            if bad_runs < 2:
                self.post_success()
            else:
                self.post_failure()                

    def __init__(self, cube=None):
        self.object = cube
        super().__init__()

    def start(self, event=None):
        self.children['goto_cube'].object = self.object
        self.children['set_carrying'].object = self.object
        super().start(event)

    def setup(self):
        """
            goto_cube: GoToCube()
            goto_cube =F=> ParentFails()
            goto_cube =C=> AbortAllActions() =C=> {raise_lift, raise_head}
    
            raise_lift: SetLiftHeight(0.5)
            raise_head: SetHeadAngle(10)
    
            {raise_lift, raise_head} =C=> StateNode() =T(0.5)=> verify
    
    
    
            verify: self.VerifyPickup()
            verify =S=> satisfied
            verify =F=> frustrated
    
            satisfied: AnimationTriggerNode(trigger=cozmo.anim.Triggers.FistBumpSuccess,
                                            ignore_body_track=True,
                                            ignore_lift_track=True) =C=>
            have_cube: SetLiftHeight(1.0) =C=>
              set_carrying: SetCarrying() =C=> ParentCompletes()
    
            frustrated: AnimationTriggerNode(trigger=cozmo.anim.Triggers.FrustratedByFailure,
                                             ignore_body_track=True,
                                             ignore_lift_track=True) =C=>
            missed_cube: Forward(-5) =C=>
              droplift: SetLiftHeight(0) =C=> ParentFails()
    
        """
        
        # Code generated by genfsm on Sat Jan  6 14:15:07 2018:
        
        goto_cube = GoToCube() .set_name("goto_cube") .set_parent(self)
        parentfails3 = ParentFails() .set_name("parentfails3") .set_parent(self)
        abortallactions1 = AbortAllActions() .set_name("abortallactions1") .set_parent(self)
        raise_lift = SetLiftHeight(0.5) .set_name("raise_lift") .set_parent(self)
        raise_head = SetHeadAngle(10) .set_name("raise_head") .set_parent(self)
        statenode3 = StateNode() .set_name("statenode3") .set_parent(self)
        verify = self.VerifyPickup() .set_name("verify") .set_parent(self)
        satisfied = AnimationTriggerNode(trigger=cozmo.anim.Triggers.FistBumpSuccess,
                                        ignore_body_track=True,
                                        ignore_lift_track=True) .set_name("satisfied") .set_parent(self)
        have_cube = SetLiftHeight(1.0) .set_name("have_cube") .set_parent(self)
        set_carrying = SetCarrying() .set_name("set_carrying") .set_parent(self)
        parentcompletes1 = ParentCompletes() .set_name("parentcompletes1") .set_parent(self)
        frustrated = AnimationTriggerNode(trigger=cozmo.anim.Triggers.FrustratedByFailure,
                                         ignore_body_track=True,
                                         ignore_lift_track=True) .set_name("frustrated") .set_parent(self)
        missed_cube = Forward(-5) .set_name("missed_cube") .set_parent(self)
        droplift = SetLiftHeight(0) .set_name("droplift") .set_parent(self)
        parentfails4 = ParentFails() .set_name("parentfails4") .set_parent(self)
        
        failuretrans6 = FailureTrans() .set_name("failuretrans6")
        failuretrans6 .add_sources(goto_cube) .add_destinations(parentfails3)
        
        completiontrans12 = CompletionTrans() .set_name("completiontrans12")
        completiontrans12 .add_sources(goto_cube) .add_destinations(abortallactions1)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(abortallactions1) .add_destinations(raise_lift,raise_head)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(raise_lift,raise_head) .add_destinations(statenode3)
        
        timertrans11 = TimerTrans(0.5) .set_name("timertrans11")
        timertrans11 .add_sources(statenode3) .add_destinations(verify)
        
        successtrans3 = SuccessTrans() .set_name("successtrans3")
        successtrans3 .add_sources(verify) .add_destinations(satisfied)
        
        failuretrans7 = FailureTrans() .set_name("failuretrans7")
        failuretrans7 .add_sources(verify) .add_destinations(frustrated)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(satisfied) .add_destinations(have_cube)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(have_cube) .add_destinations(set_carrying)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(set_carrying) .add_destinations(parentcompletes1)
        
        completiontrans18 = CompletionTrans() .set_name("completiontrans18")
        completiontrans18 .add_sources(frustrated) .add_destinations(missed_cube)
        
        completiontrans19 = CompletionTrans() .set_name("completiontrans19")
        completiontrans19 .add_sources(missed_cube) .add_destinations(droplift)
        
        completiontrans20 = CompletionTrans() .set_name("completiontrans20")
        completiontrans20 .add_sources(droplift) .add_destinations(parentfails4)
        
        return self

class DropObject(StateNode):
    def __init__(self):
        self.object = None
        super().__init__()

    def setup(self):
        """
            Print('DropObject...') =N=> SetLiftHeight(0) =C=> SetNotCarrying() =N=> Forward(-10) =C=> ParentCompletes()
        """
        
        # Code generated by genfsm on Sat Jan  6 14:15:07 2018:
        
        print2 = Print('DropObject...') .set_name("print2") .set_parent(self)
        setliftheight1 = SetLiftHeight(0) .set_name("setliftheight1") .set_parent(self)
        setnotcarrying1 = SetNotCarrying() .set_name("setnotcarrying1") .set_parent(self)
        forward3 = Forward(-10) .set_name("forward3") .set_parent(self)
        parentcompletes2 = ParentCompletes() .set_name("parentcompletes2") .set_parent(self)
        
        nulltrans5 = NullTrans() .set_name("nulltrans5")
        nulltrans5 .add_sources(print2) .add_destinations(setliftheight1)
        
        completiontrans21 = CompletionTrans() .set_name("completiontrans21")
        completiontrans21 .add_sources(setliftheight1) .add_destinations(setnotcarrying1)
        
        nulltrans6 = NullTrans() .set_name("nulltrans6")
        nulltrans6 .add_sources(setnotcarrying1) .add_destinations(forward3)
        
        completiontrans22 = CompletionTrans() .set_name("completiontrans22")
        completiontrans22 .add_sources(forward3) .add_destinations(parentcompletes2)
        
        return self


class PickUpCubeForeign(StateNode):

    def __init__(self, cube_id=None):
        self.object_id = cube_id
        super().__init__()

    def start(self, event=None):
        # self.object will be set up by the parent of this node
        self.object = self.robot.world.light_cubes[self.object_id]
        self.foreign_cube_id = 'LightCubeForeignObj-'+str(self.object_id)
        super().start(event)

    def pick_side(self, dist, use_world_map):
        "NOTE: This code is only correct for upright cubes"
        cube = self.foreign_cube_id
        cobj = self.robot.world.world_map.objects[cube]
        x = cobj.x
        y = cobj.y
        ang = cobj.theta
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

    def setup(self):
        """
            goto_cube: self.GoToSide() =C=> one
    
            one: self.Pick() =C=> end
            end: Say("Done") =C=> ParentCompletes()
        """
        
        # Code generated by genfsm on Sat Jan  6 14:15:07 2018:
        
        goto_cube = self.GoToSide() .set_name("goto_cube") .set_parent(self)
        one = self.Pick() .set_name("one") .set_parent(self)
        end = Say("Done") .set_name("end") .set_parent(self)
        parentcompletes3 = ParentCompletes() .set_name("parentcompletes3") .set_parent(self)
        
        completiontrans23 = CompletionTrans() .set_name("completiontrans23")
        completiontrans23 .add_sources(goto_cube) .add_destinations(one)
        
        completiontrans24 = CompletionTrans() .set_name("completiontrans24")
        completiontrans24 .add_sources(one) .add_destinations(end)
        
        completiontrans25 = CompletionTrans() .set_name("completiontrans25")
        completiontrans25 .add_sources(end) .add_destinations(parentcompletes3)
        
        return self
