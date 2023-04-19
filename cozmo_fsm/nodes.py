import time
import asyncio
import inspect
import types
import random
import numpy as np

try: import cv2
except: pass

from math import pi, sqrt, atan2, inf, nan
from multiprocessing import Process, Queue

import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees, Distance, Angle, Pose

from . import evbase
from .base import *
from .events import *
from .cozmo_kin import wheelbase
from .geometry import wrap_angle
from .worldmap import WorldObject, FaceObj, CustomMarkerObj

#________________ Ordinary Nodes ________________

class ParentCompletes(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to complete' % (self, self.parent))
        if self.parent:
            self.parent.post_completion()

class ParentSucceeds(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to succeed' % (self, self.parent))
        if self.parent:
            self.parent.post_success()

class ParentFails(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to fail' % (self, self.parent))
        if self.parent:
            self.parent.post_failure()

class Iterate(StateNode):
    """Iterates over an iterable, posting DataEvents.  Completes when done."""
    def __init__(self,iterable=None):
        super().__init__()
        self.iterable = iterable

    class NextEvent(Event): pass

    def start(self,event=None):
        if self.running: return
        super().start(event)
        if isinstance(event, DataEvent):
            self.iterable = event.data
        if isinstance(self.iterable, int):
            self.iterable = range(self.iterable)
        if self.iterable is None:
            raise ValueError('~s has nothing to iterate on.' % repr(self))
        if not isinstance(event, self.NextEvent):
            self.iterator = self.iterable.__iter__()
        try:
            value = next(self.iterator)
        except StopIteration:
            self.post_completion()
            return
        self.post_data(value)

class MoveLift(StateNode):
    "Move lift at specified speed."
    def __init__(self,speed):
        super().__init__()
        self.speed = speed

    def start(self,event=None):
        if self.running: return
        super().start(event)
        # Temporary hack supplied by Mark Wesley at Anki
        msg = cozmo._clad._clad_to_engine_iface.EnableLiftPower(True)
        self.robot.conn.send_msg(msg)
        self.robot.move_lift(self.speed)

    def stop(self):
        if not self.running: return
        self.robot.move_lift(0)
        super().stop()

class RelaxLift(StateNode):
    def start(self,event=None):
        if self.running: return
        super().start(event)
        # Temporary hack supplied by Mark Wesley at Anki
        msg = cozmo._clad._clad_to_engine_iface.EnableLiftPower(False)
        self.robot.conn.send_msg(msg)

class SetLights(StateNode):
    def __init__(self, object, light):
        super().__init__()
        self.object = object
        self.light = light

    def start(self,event=None):
        super().start(event)
        if self.object is not self.robot:
            self.object.set_lights(self.light)
        else:
            if self.light.on_color.int_color & 0x00FFFF00 == 0: # no green or blue component
                self.robot.set_all_backpack_lights(self.light)
            else:
                self.robot.set_backpack_lights_off()
                self.robot.set_center_backpack_lights(self.light)
        self.post_completion()

class DriveContinuous(StateNode):
    def __init__(self,path=[]):
        self.path = path
        self.polling_interval = 0.05
        super().__init__()

    def start(self,event=None):
        if isinstance(event, DataEvent) and isinstance(event.data,(list,tuple)):
            self.path = event.data
        if len(self.path) == 0:
            raise ValueError('Node %s has a null path' % repr(self))
        self.path_index = 0
        self.cur = self.path[self.path_index]
        self.prev = None
        self.last_dist = -1
        self.target_q = None
        self.reached_dist = False
        self.mode = None
        self.pause_counter = 0
        super().start(event)

    def stop(self):
        self.robot.stop_all_motors()
        super().stop()

    def poll(self):
        if not self.running: return
        # Quit if the robot is picked up.
        if self.robot.really_picked_up():
            print('** Robot was picked up.')
            self.robot.stop_all_motors()
            self.poll_handle.cancel()
            self.path_index = None
            #print('<><><>', self, 'punting')
            self.post_failure()
            return
        # See where we are
        x = self.robot.world.particle_filter.pose[0]
        y = self.robot.world.particle_filter.pose[1]
        q = self.robot.world.particle_filter.pose[2]
        dist = sqrt((self.cur.x-x)**2 + (self.cur.y-y)**2)
        delta_q = wrap_angle(q - self.target_q) if self.target_q is not None else inf

        # If we're pausing, print our position and exit
        if self.pause_counter > 0:
            #print('%x p%1d. x: %5.1f  y: %5.1f  q:%6.1f     dist: %5.1f' %
            #      (self.__hash__() & 0xffffffffffffffff, self.pause_counter, x, y, q*180/pi, dist))
            self.pause_counter -= 1
            return

        # See if we've passed the closest approach to the waypoint,
        # i.e., distance to the waypoint is consistently INCREASING
        # or waypoint is behind us.
        if not self.reached_dist:
            self.reached_dist = \
                abs(delta_q) > 135*pi/180 or \
                (dist - self.last_dist) > 0.1 and \
                ( (self.mode == 'x' and np.sign(x-self.cur.x) == np.sign(self.cur.x-self.prev.x)) or
                  (self.mode == 'y' and np.sign(y-self.cur.y) == np.sign(self.cur.y-self.prev.y)) )
        self.last_dist = dist

        # Is it time to switch to the next waypoint?
        reached_waypoint = (self.path_index == 0) or \
                           (self.reached_dist and \
                            (self.path_index < len(self.path)-1 or \
                             abs(delta_q) < 5*pi/180))

        # Advance to next waypoint if indicated
        if reached_waypoint:
            self.path_index += 1
            print('DriveContinuous: current position is (%.1f, %.1f) @ %.1f deg.' %
                  (x, y, q*180/pi))
            print('   path index advanced to %d' % self.path_index, end='')
            if self.path_index == len(self.path):
                print('\nDriveContinous: path complete.  Stopping.')
                self.robot.stop_all_motors()
                self.post_completion()
                return
            elif self.path_index > len(self.path):
                # uncaught completion event
                print('\nDriveContinuous: uncaught completion! Stopping.')
                self.stop()
                return
            self.prev = self.cur
            self.cur = self.path[self.path_index]
            self.last_dist = inf
            self.reached_dist = False
            self.target_q = atan2(self.cur.y-self.prev.y, self.cur.x-self.prev.x)
            print(': [%.1f, %.1f] tgtQ is %.1f deg.' % (self.cur.x, self.cur.y, self.target_q*180/pi))

            # Is the target behind us?
            delta_dist = sqrt((self.cur.x-x)**2 + (self.cur.y-y)**2)
            if delta_q < inf and  abs(delta_q) > 135*pi/180:
                #self.target_q = wrap_angle(self.target_q + pi)
                print('New waypoint is behind us --> delta_q = %.1f deg., new target_q = %.1f deg., dist = %.1f' %
                      (delta_q*180/pi, self.target_q*180/pi, delta_dist))
                self.drive_direction = +1  # was -1
            else:
                self.drive_direction = +1

            # Heading determines whether we're solving y=f(x) or x=f(y)
            if abs(self.target_q) < pi/4 or abs(abs(self.target_q)-pi) < pi/4:
                self.mode = 'x'    # y = m*x + b
                xdiff = self.cur.x - self.prev.x
                xdiv = xdiff if xdiff != 0 else 0.01
                self.m = (self.cur.y-self.prev.y) / xdiv
                self.b = self.cur.y - self.m * self.cur.x
                #print('   y =', self.m, ' * x +', self.b)
            else:
                self.mode = 'y'    # x = m*y + b
                ydiff = self.cur.y - self.prev.y
                ydiv = ydiff if ydiff != 0 else 0.01
                self.m = (self.cur.x-self.prev.x) / ydiv
                self.b = self.cur.x - self.m * self.cur.y
                #print('   x =', self.m, ' * y +', self.b)

            # Do we need to turn in place before setting off toward new waypoint?
            if abs(wrap_angle(q-self.target_q)) > 30*pi/180:
                self.saved_mode = self.mode
                self.mode = 'q'
                print('DriveContinuous: turning to %.1f deg. before driving to waypoint.' %
                      (self.target_q*180/pi))

            # If we were moving, come to a full stop before trying to change direction
            if self.path_index > 1:
                self.robot.stop_all_motors()
                self.pause_counter = 5
                return

        # Haven't reached waypoint yet
        elif self.reached_dist:
            # But we have traveled far enough, and this is the last waypoint, so
            # come to a stop and then fix heading
            if self.mode != 'q':
                if abs(wrap_angle(q-self.target_q)) > 5:
                    self.robot.stop_all_motors()
                    self.robot.pause_counter = 5
                    self.mode = 'q'
                    print('DriveContinuous: final waypoint reached; adjusting heading to %.1f deg.' %
                          (self.target_q*180/pi))
        elif self.mode == 'q' and abs(wrap_angle(q-self.target_q)) < 10*pi/180:
            # If within 10 degrees, cut motors and let inertia carry us the rest of the way.
            print('DriveContinuous: turn to heading complete: heading is %.1f deg.' %
                  (q*180/pi))
            self.robot.stop_all_motors()
            self.mode = self.saved_mode
            self.pause_counter = 5
            return

        # Calculate error and correction based on present x/y/q position
        q_error = wrap_angle(q - self.target_q)
        intercept_distance = 100 # was 25
        if self.mode == 'x':      # y = m*x + b
            target_y = self.m * x + self.b
            d_error = (y - target_y) * np.sign(pi/2 - abs(self.target_q))  # *** CHECK THIS
            correcting_q = - 0.8*q_error - 0.8*atan2(d_error,intercept_distance)
        elif self.mode == 'y':    # x = m*y + b
            target_x = self.m * y + self.b
            d_error = (x - target_x) * np.sign(pi/2 - abs(self.target_q-pi/2))  # *** CHECK THIS
            correcting_q = - 0.8*q_error - 0.8*atan2(-d_error,intercept_distance)
        elif self.mode == 'q':
            d_error = nan
            correcting_q = - nan # 0.8*q_error
        else:
            print("Bad mode value '%s'" % repr(self.mode))
            return

        # Calculate wheel speeds based on correction value
        if self.mode == 'q' or abs(q_error)*180/pi >= 15:
            # For large heading error, turn in place
            flag = "<>"
            speed = 0
            qscale = 150 # was 50
            correcting_q = nan #  - 1.0 * np.sign(q_error) * min(abs(q_error), 25*pi/180)
            speedinc = qscale * 15*pi/180 * -np.sign(q_error)
        elif abs(q_error)*180/pi > 5 and abs(d_error) < 100:
            # For moderate heading error where  distance error isn't huge,
            # slow down and turn more slowly
            flag = "* "
            speed = 50 # was 20
            qscale = 150
            speedinc = qscale * correcting_q
        else:
            # We're doing pretty well; go fast and make minor corrections
            flag = "  "
            speed = 100
            qscale = 150 # was 150
            speedinc = qscale * correcting_q
        lspeed = self.drive_direction * (speed - self.drive_direction*speedinc)
        rspeed = self.drive_direction * (speed + self.drive_direction*speedinc)

        if self.mode == 'x': display_target = target_y
        elif self.mode == 'y': display_target = target_x
        elif self.mode == 'q': display_target = self.target_q*180/pi
        elif self.mode == 'p': display_target = inf
        else: display_target = nan
        """
        print('%x %s x: %5.1f  y: %5.1f  q:%6.1f  tgt:%6.1f   derr: %5.1f  qerr:%6.1f  corq: %5.1f  inc: %5.1f  dist: %5.1f   speeds: %4.1f/%4.1f' %
              (self.__hash__() & 0xffffffffffffffff,
               self.mode+flag, x, y, q*180/pi, display_target, d_error, q_error*180/pi,
               correcting_q*180/pi, speedinc, dist, lspeed, rspeed))
        """
        self.robot.drive_wheel_motors(lspeed, rspeed, 500, 500)

class LookAtObject(StateNode):
    "Continuously adjust head angle to fixate object."
    def __init__(self):
        super().__init__()
        self.object = None
        self.handle = None

    def start(self,event=None):
        self.set_polling_interval(0.1)
        self.handle = None
        super().start()

    def stop(self):
        if self.handle:
            self.handle.cancel()
        super().stop()

    def poll(self):
        if not self.running: return
        if isinstance(self.object, FaceObj) or isinstance(self.object, CustomMarkerObj):
            image_box =  self.object.sdk_obj.last_observed_image_box
            camera_center = self.robot.camera.config.center.y
            delta = image_box.top_left_y + image_box.height/2 - camera_center
            adjust_level = 0.1
            if self.robot.left_wheel_speed.speed_mmps != 0 and self.robot.right_wheel_speed.speed_mmps != 0:
                adjust_level = 0.2
            if delta > 15:
                angle = self.robot.head_angle.radians - adjust_level
            elif delta < -15:
                angle = self.robot.head_angle.radians + adjust_level
            else:
                angle = self.robot.head_angle.radians
            angle = cozmo.robot.MAX_HEAD_ANGLE.radians if angle > cozmo.robot.MAX_HEAD_ANGLE.radians else angle
            angle = cozmo.robot.MIN_HEAD_ANGLE.radians if angle < cozmo.robot.MIN_HEAD_ANGLE.radians else angle
        else:
            if isinstance(self.object, WorldObject):
                rpose = self.robot.world.particle_filter.pose
                dx = self.object.x - rpose[0]
                dy = self.object.y - rpose[1]
            else:
                opos = self.object.pose.position
                rpos = self.robot.pose.position
                dx = opos.x - rpos.x
                dy = opos.y - rpos.y
            dist = sqrt(dx**2 + dy**2)
            if dist < 60:
                angle = -0.4
            elif dist < 80:
                angle = -0.3
            elif dist < 100:
                angle = -0.2
            elif dist < 140:
                angle = -0.1
            elif dist < 180:
                angle = 0
            else:
                angle = 0.1
        if abs(self.robot.head_angle.radians - angle) > 0.03:
            self.handle = self.robot.loop.call_soon(self.move_head, angle)

    def move_head(self,angle):
        try:
            self.robot.set_head_angle(cozmo.util.radians(angle), in_parallel=True, num_retries=2)
        except cozmo.exceptions.RobotBusy:
            print("LookAtObject: robot busy; can't move head to",angle)
            pass


class SetPose(StateNode):
    def __init__(self, pose=Pose(0,0,0,angle_z=degrees(0))):
        super().__init__()
        self.pose = pose

    def start(self, event=None):
        super().start(event)
        if isinstance(event, DataEvent) and isinstance(event.data, Pose):
            pose = event.data
        else:
            pose = self.pose
        self.robot.world.particle_filter.set_pose(self.pose.x, self.pose.y, self.pose.angle_z.radians)


class Print(StateNode):
    "Argument can be a string, or a function to be evaluated at print time."
    def __init__(self,spec=None):
        super().__init__()
        self.spec = spec

    def start(self,event=None):
        super().start(event)
        if isinstance(self.spec, types.FunctionType):
            text = self.spec()
        else:
            text = self.spec
        if text is None and isinstance(event, DataEvent):
            text = repr(event.data)
        print(text)
        self.post_completion()


class AbortAllActions(StateNode):
    def start(self,event=None):
        super().start(event)
        self.robot.abort_all_actions()
        self.post_completion()


class AbortHeadAction(StateNode):
    def start(self,event=None):
        super().start(event)
        actionType = cozmo._clad._clad_to_engine_cozmo.RobotActionType.UNKNOWN
        msg = cozmo._clad._clad_to_engine_iface.CancelAction(actionType=actionType)
        self.robot.conn.send_msg(msg)
        self.post_completion()


class StopAllMotors(StateNode):
    def start(self,event=None):
        super().start(event)
        self.robot.stop_all_motors()
        self.post_completion()


#________________ Color Images ________________

class ColorImageBase(StateNode):

    def is_color(self,image):
        raw = image.raw_image
        for i in range(0, raw.height, 15):
            pixel = raw.getpixel((i,i))
            if pixel[0] != pixel[1]:
                return True
        return False


class ColorImageEnabled(ColorImageBase):
    """Turn color images on or off and post completion when setting has taken effect."""
    def __init__(self,enabled=True):
        self.enabled = enabled
        super().__init__()

    def start(self,event=None):
        super().start(event)
        if self.robot.camera.color_image_enabled == self.enabled:
            self.post_completion()
        else:
            self.robot.camera.color_image_enabled = self.enabled
            self.robot.world.add_event_handler(cozmo.world.EvtNewCameraImage, self.new_image)

    def new_image(self,event,**kwargs):
        is_color = self.is_color(event.image)
        if is_color:
            self.robot.world.latest_color_image = event.image
        if is_color == self.enabled:
            self.robot.world.remove_event_handler(cozmo.world.EvtNewCameraImage, self.new_image)
            self.post_completion()


class GetColorImage(ColorImageBase):
    """Post one color image as a data event; leave color mode unchanged."""

    def start(self,event=None):
        super().start(event)
        self.save_enabled = self.robot.camera.color_image_enabled
        if not self.save_enabled:
            self.robot.camera.color_image_enabled = True
        self.robot.world.add_event_handler(cozmo.world.EvtNewCameraImage, self.new_image)

    def new_image(self,event,**kwargs):
        if self.is_color(event.image):
            self.robot.world.latest_color_image = event.image
            self.robot.camera.color_image_enabled = self.save_enabled
            try:
                self.robot.world.remove_event_handler(cozmo.world.EvtNewCameraImage, self.new_image)
            except: pass
            self.post_data(event.image)

class SaveImage(StateNode):
    "Save an image to a file."

    def __init__(self, filename="image", filetype="jpg", counter=0, verbose=True):
        super().__init__()
        self.filename = filename
        self.filetype = filetype
        self.counter = counter
        self.verbose = verbose

    def start(self,event=None):
        super().start(event)
        fname = self.filename
        if isinstance(self.counter, int):
            fname = fname + str(self.counter)
            self.counter = self.counter + 1
        fname = fname + "." + self.filetype
        image = np.array(self.robot.world.latest_image.raw_image)
        cv2.imwrite(fname, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        if self.verbose:
            print('Wrote',fname)


#________________ Coroutine Nodes ________________

class CoroutineNode(StateNode):
    def __init__(self):
        super().__init__()
        self.handle = None
        self.abort_launch = False

    def start(self, event=None):
        super().start(event)
        if self.abort_launch:
            self.handle = None
            return
        cor = self.coroutine_launcher()
        if inspect.iscoroutine(cor):
            self.handle = self.robot.loop.create_task(cor)
        elif cor is False:
            self.handle = None
        else:
            print('cor=',cor,'type=',type(cor))
            raise ValueError("Result of %s launch_couroutine() is %s, not a coroutine." %
                             (self,cor))

    def coroutine_launcher(self):
        raise Exception('%s lacks a coroutine_launcher() method' % self)

    def post_when_complete(self):
        "Call this from within start() if the coroutine will signal completion."
        self.robot.loop.create_task(self.wait_for_completion())

    async def wait_for_completion(self):
        await self.handle
        if TRACE.trace_level >= TRACE.await_satisfied:
            print('TRACE%d:' % TRACE.await_satisfied, self,
                  'await satisfied:', self.handle)
        self.post_completion()

    def stop(self):
        if not self.running: return
        if self.handle: self.handle.cancel()
        super().stop()


class DriveWheels(CoroutineNode):
    def __init__(self,l_wheel_speed,r_wheel_speed,**kwargs):
        super().__init__()
        self.l_wheel_speed = l_wheel_speed
        self.r_wheel_speed = r_wheel_speed
        self.kwargs = kwargs

    def start(self,event=None):
        if (isinstance(event,DataEvent) and isinstance(event.data,(list,tuple)) and
                len(event.data) == 2):
            (lspeed,rspeed) = event.data
            if isinstance(lspeed,(int,float)) and isinstance(rspeed,(int,float)):
                self.l_wheel_speed = lspeed
                self.r_wheel_speed = rspeed
        self.abort_launch = False
        if self.robot.really_picked_up():
            self.abort_launch = True
            super().start(event)
            self.post_failure()
            return
        super().start(event)

    def coroutine_launcher(self):
        return self.robot.drive_wheels(self.l_wheel_speed,self.r_wheel_speed,**self.kwargs)

    def stop_wheels(self):
        try:
            driver = self.robot.drive_wheels(0,0)
            # driver is either a co-routine or None
            if driver: driver.send(None)  # will raise StopIteration
        except StopIteration: pass

    def stop(self):
        if not self.running: return
        self.stop_wheels()
        super().stop()


class DriveForward(DriveWheels):
    def __init__(self, distance=50, speed=50, **kwargs):
        if isinstance(distance, cozmo.util.Distance):
            distance = distance.distance_mm
        if isinstance(speed, cozmo.util.Speed):
            speed = speed.speed_mmps
        if distance < 0:
            distance = -distance
            speed = -speed
        self.distance = distance
        self.speed = speed
        self.kwargs = kwargs
        super().__init__(speed,speed,**self.kwargs)
        self.polling_interval = 0.1

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Distance):
            self.distance = event.data.distance_mm
        self.start_position = self.robot.pose.position
        super().start(event)

    def poll(self):
        if not self.running: return
        """See how far we've traveled"""
        p0 = self.start_position
        p1 = self.robot.pose.position
        diff = (p1.x - p0.x, p1.y - p0.y)
        dist = sqrt(diff[0]*diff[0] + diff[1]*diff[1])
        if dist >= self.distance:
            self.poll_handle.cancel()
            self.stop_wheels()
            self.post_completion()

class SmallTurn(CoroutineNode):
    """Estimates how many polling cycles to run the wheels; doesn't use odometry."""
    def __init__(self, angle=5):
        self.angle = angle
        self.counter = 0
        self.polling_interval = 0.025
        super().__init__()

    def start(self,event=None):
        # constants were determined empirically for speed 50
        self.counter = round((abs(self.angle) + 5) / 1.25) if self.angle else 0
        self.abort_launch = False
        if self.robot.really_picked_up():
            self.abort_launch = True
            super().start(event)
            self.post_failure()
            return
        super().start(event)

    def coroutine_launcher(self):
        if self.angle:
            speed = 50 if self.angle < 0 else -50
            return self.robot.drive_wheels(speed,-speed,500,500)
        else:
            self.robot.stop_all_motors()
            return False

    def poll(self):
        if not self.running: return
        self.counter -= 1
        if self.counter <= 0:
            self.poll_handle.cancel()
            self.robot.stop_all_motors()
            self.post_completion()

class DriveTurn(DriveWheels):
    def __init__(self, angle=90, speed=50, **kwargs):
        if isinstance(angle, cozmo.util.Angle):
            angle = angle.degrees
        if isinstance(speed, cozmo.util.Speed):
            speed = speed.speed_mmps
        if speed <= 0:
            raise ValueError('speed parameter must be positive')
        self.angle = angle
        self.speed = speed
        self.kwargs = kwargs
        self.polling_interval = 0.05
        super().__init__(0,0,**self.kwargs)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Angle):
            self.angle = event.data.degrees
        if self.angle > 0:
            self.l_wheel_speed = -self.speed
            self.r_wheel_speed = self.speed
        else:
            self.l_wheel_speed = self.speed
            self.r_wheel_speed = -self.speed
        self.last_heading = self.robot.pose.rotation.angle_z.degrees
        self.traveled = 0
        super().start(event)

    def poll(self):
        if not self.running: return
        """See how far we've traveled"""
        p0 = self.last_heading
        p1 = self.robot.pose.rotation.angle_z.degrees
        self.last_heading = p1
        # Assume we're polling quickly enough that diff will be small;
        # typically only about 1 degree.  So diff will be large only
        # if the heading has passed through 360 degrees since the last
        # call to poll().  Use 90 degrees as an arbitrary large threshold.
        diff = p1 - p0
        if diff  < -90.0:
            diff += 360.0
        elif diff > 90.0:
            diff -= 360.0
        self.traveled += diff
        if abs(self.traveled) > abs(self.angle):
            self.poll_handle.cancel()
            self.stop_wheels()
            self.post_completion()


class DriveArc(DriveWheels):
    """Negative radius means right turn; negative angle means drive
    backwards.  This node can be passed a DataEvent with a dict
    containing any of the arguments accepted by __init__: radius,
    angle, distance, speed, and angspeed.  Values must already be in
    the appropriate units (degrees, mm, deg/sec, or mm/sec)."""
    def __init__(self, radius=0, angle=None, distance=None,
                 speed=None, angspeed=None, **kwargs):
        if isinstance(radius, cozmo.util.Distance):
            radius = radius.distance_mm
        if isinstance(angle, cozmo.util.Angle):
            angle = angle.degrees
        if isinstance(speed, cozmo.util.Speed):
            speed = speed.speed_mmps
        if isinstance(angspeed, cozmo.util.Angle):
            angspeed = angspeed.degrees
        self.calculate_wheel_speeds(radius, angle, distance, speed, angspeed)
        super().__init__(self.l_wheel_speed, self.r_wheel_speed, **kwargs)
        # Call parent init before setting polling interval.
        self.polling_interval = 0.05

    def calculate_wheel_speeds(self, radius=0, angle=None, distance=None,
                               speed=None, angspeed=None):
        if radius != 0:
            if angle is not None:
                pass
            elif distance is not None:
                angle = self.dist2ang(distance, radius)
            else:
                raise ValueError('DriveArc requires an angle or distance.')

            if  speed is not None:
                pass
            elif angspeed is not None:
                speed = self.ang2dist(angspeed, radius)
            else:
                speed = 40 # degrees/second
            if angle < 0:
                speed = - speed

            self.angle = angle
            self.l_wheel_speed = speed * (1 - wheelbase / radius)
            self.r_wheel_speed = speed * (1 + wheelbase / radius)

        else:  # radius is 0
            if angspeed is None:
                angspeed = 40 # degrees/second
            s = angspeed
            if angle < 0:
                s = -s
            self.angle = angle
            self.l_wheel_speed = -s
            self.r_wheel_speed = s

    def ang2dist(self, angle, radius):
        return (angle / 360) * 2 * pi * abs(radius)

    def dist2ang(self, distance, radius):
        return (distance / abs(2 * pi * radius)) * 360

    def start(self,event=None):
        if self.running: return
        if isinstance(event,DataEvent) and isinstance(event.data,dict):
            self.calculate_wheel_speeds(**event.data)
        self.last_heading = self.robot.pose.rotation.angle_z.degrees
        self.traveled = 0
        super().start(event)

    def poll(self):
        if not self.running: return
        """See how far we've traveled"""
        p0 = self.last_heading
        p1 = self.robot.pose.rotation.angle_z.degrees
        self.last_heading = p1
        # Assume we're polling quickly enough that diff will be small;
        # typically only about 1 degree.  So diff will be large only
        # if the heading has passed through 360 degrees since the last
        # call to poll().  Use 90 degrees as an arbitrary large threshold.
        diff = p1 - p0
        if diff  < -90.0:
            diff += 360.0
        elif diff > 90.0:
            diff -= 360.0
        self.traveled += diff

        if abs(self.traveled) > abs(self.angle):
            self.poll_handle.cancel()
            self.stop_wheels()
            self.post_completion()


#________________ Cube Disconnect/Reconnect ________________

class DisconnectFromCubes(StateNode):
    def start(self, event=None):
        super().start(event)
        self.robot.world.disconnect_from_cubes()


class ConnectToCubes(CoroutineNode):
    def start(self, event=None):
        super().start(event)
        self.post_when_complete()

    def coroutine_launcher(self):
        return self.robot.world.connect_to_cubes()


#________________ Action Nodes ________________

class ActionNode(StateNode):
    relaunch_delay = 0.050 # 50 milliseconds

    def __init__(self, abort_on_stop=True):
        """Call this method only after the subclass __init__ has set
        up self.action_kwargs"""
        self.abort_on_stop = abort_on_stop
        super().__init__()
        if 'in_parallel' not in self.action_kwargs:
            self.action_kwargs['in_parallel'] = True
        if 'num_retries' not in self.action_kwargs:
            self.action_kwargs['num_retries'] = 2
        self.cozmo_action_handle = None
        self.abort_launch = False

    def start(self,event=None):
        super().start(event)
        self.retry_count = 0
        if not self.abort_launch:
            self.launch_or_retry()

    def launch_or_retry(self):
        try:
            result = self.action_launcher()
        except cozmo.exceptions.RobotBusy:
            if TRACE.trace_level >= TRACE.statenode_startstop:
                print('TRACE%d:' % TRACE.statenode_startstop, self, 'launch_action raised RobotBusy')
            self.handle = self.robot.loop.call_later(self.relaunch_delay, self.launch_or_retry)
            return
        if isinstance(result, cozmo.action.Action):
            self.cozmo_action_handle = result
        elif result is None: # Aborted
            return
        else:
            raise ValueError("Result of %s launch_action() is %s, not a cozmo.action.Action." %
                             (self,result))
        self.post_when_complete()

    def action_launcher(self):
        raise Exception('%s lacks an action_launcher() method' % self)

    def post_when_complete(self):
       self.robot.loop.create_task(self.wait_for_completion())

    async def wait_for_completion(self):
        async_task = self.cozmo_action_handle.wait_for_completed()
        await async_task
        if TRACE.trace_level >= TRACE.await_satisfied:
            print('TRACE%d:' % TRACE.await_satisfied, self,
                  'await satisfied:', self.cozmo_action_handle)
        # check status for 'completed'; if not, schedule relaunch or post failure
        if self.running:
            if self.cozmo_action_handle.state == 'action_succeeded':
                self.post_completion()
            elif self.cozmo_action_handle.failure_reason[0] == 'cancelled':
                print('CANCELLED: ***>',self,self.cozmo_action_handle)
                self.post_completion()
            elif self.cozmo_action_handle.failure_reason[0] == 'retry':
                if self.retry_count < self.action_kwargs['num_retries']:
                    print("*** ACTION %s of %s FAILED WITH CODE 'retry': TRYING AGAIN" %
                          (self.cozmo_action_handle, self.name))
                    self.retry_count += 1
                    self.launch_or_retry()
                else:
                    print("*** %s ACTION RETRY COUNT EXCEEDED: FAILING" % self.name)
                    self.post_failure(self.cozmo_action_handle)
            else:
                print("*** ACTION NODE %s %s FAILED DUE TO %s AND CAN'T BE RETRIED." %
                      (self.name,
                       self.cozmo_action_handle,
                       self.cozmo_action_handle.failure_reason[0] or 'unknown reason'))
                self.post_failure(self.cozmo_action_handle)

    def stop(self):
        if not self.running: return
        if self.cozmo_action_handle and self.abort_on_stop and \
                self.cozmo_action_handle.is_running:
            self.cozmo_action_handle.abort()
        super().stop()


class Say(ActionNode):
    """Speaks some text, then posts a completion event."""

    class SayDataEvent(Event):
        def __init__(self,text=None):
            self.text = text

    def __init__(self, text="I'm speechless",
                 abort_on_stop=False, **action_kwargs):
        self.text = text
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, self.SayDataEvent):
            utterance = event.text
        else:
            utterance = self.text
        if isinstance(utterance, (list,tuple)):
            utterance = random.choice(utterance)
        if not isinstance(utterance, str):
            utterance = repr(utterance)
        self.utterance = utterance
        print("Speaking: '",utterance,"'",sep='')
        super().start(event)

    def action_launcher(self):
        if self.utterance.rstrip() == '':
            # robot.say_text() action would fail on empty string
            self.post_completion()
            return None
        else:
            return self.robot.say_text(self.utterance, **self.action_kwargs)


class Forward(ActionNode):
    """ Moves forward a specified distance. Can accept a Distance as a Dataevent."""
    def __init__(self, distance=distance_mm(50),
                 speed=speed_mmps(50), abort_on_stop=True, **action_kwargs):
        if isinstance(distance, (int,float)):
            distance = distance_mm(distance)
        elif not isinstance(distance, cozmo.util.Distance):
            raise ValueError('%s distance must be a number or a cozmo.util.Distance' % self)
        if isinstance(speed, (int,float)):
            speed = speed_mmps(speed)
        elif not isinstance(speed, cozmo.util.Speed):
            raise ValueError('%s speed must be a number or a cozmo.util.Speed' % self)
        self.distance = distance
        self.speed = speed
        if 'should_play_anim' not in action_kwargs:
            action_kwargs['should_play_anim'] = False
        self.action_kwargs = action_kwargs
        # super's init must come last because it checks self.action_kwargs
        super().__init__(abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Distance):
            self.distance = event.data
        self.abort_launch = False
        if self.robot.really_picked_up():
            self.abort_launch = True
            super().start(event)
            self.post_failure()
            return
        super().start(event)

    def action_launcher(self):
        return self.robot.drive_straight(self.distance, self.speed,
                                         **self.action_kwargs)


class Turn(ActionNode):
    """Turns by a specified angle. Can accept an Angle as a DataEvent."""
    def __init__(self, angle=degrees(90), abort_on_stop=True, **action_kwargs):
        if isinstance(angle, (int,float)):
            angle = degrees(angle)
        elif angle is None:
            pass
        elif not isinstance(angle, cozmo.util.Angle):
            raise ValueError('%s angle must be a number or a cozmo.util.Angle' % self)
        self.angle = angle
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Angle):
            self.angle = event.data
        self.abort_launch = False
        if self.robot.really_picked_up():
            self.abort_launch = True
            super().start(event)
            self.post_failure()
            return
        super().start(event)

    def action_launcher(self):
        if self.angle is None:
            return None
        elif not isinstance(self.angle, cozmo.util.Angle):
            print("*** WARNING: node", self.name, "self.angle =", self.angle, "is not an instance of cozmo.util.Angle")
            self.angle = degrees(self.angle) if isinstance(self.angle, (int,float)) else degrees(0)
        return self.robot.turn_in_place(self.angle, **self.action_kwargs)

class GoToPose(ActionNode):
    "Uses SDK's go_to_pose method."
    def __init__(self, pose, abort_on_stop=True, **action_kwargs):
        self.pose = pose
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop)

    def action_launcher(self):
        return self.robot.go_to_pose(self.pose, **self.action_kwargs)

class SetHeadAngle(ActionNode):
    def __init__(self, angle=degrees(0), abort_on_stop=True, **action_kwargs):
        if isinstance(angle, (int,float)):
            angle = degrees(angle)
        elif not isinstance(angle, cozmo.util.Angle):
            raise ValueError('%s angle must be a number or a cozmo.util.Angle' % self)
        self.angle = angle
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Angle):
            self.angle = event.data
        super().start(event)

    def action_launcher(self):
        return self.robot.set_head_angle(self.angle, **self.action_kwargs)

class SetLiftHeight(ActionNode):
    def __init__(self, height=0, abort_on_stop=True, **action_kwargs):
        """height is a percentage from 0 to 1"""
        self.height = height
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop)

    def action_launcher(self):
        # Temporary hack supplied by Mark Wesley at Anki
        msg = cozmo._clad._clad_to_engine_iface.EnableLiftPower(True)
        self.robot.conn.send_msg(msg)
        return self.robot.set_lift_height(self.height, **self.action_kwargs)

class SetLiftAngle(SetLiftHeight):
    def __init__(self, angle, abort_on_stop=True, **action_kwargs):

        #def get_theta(height):
        #   return math.asin((height-45)/66)

        if isinstance(angle, cozmo.util.Angle):
            angle = angle.degrees
        self.angle = angle
        super().__init__(0, abort_on_stop=abort_on_stop, **action_kwargs)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and isinstance(event.data, cozmo.util.Angle):
            self.angle = event.data.degrees
        min_theta = cozmo.robot.MIN_LIFT_ANGLE.degrees
        max_theta = cozmo.robot.MAX_LIFT_ANGLE.degrees
        angle_range = max_theta - min_theta
        self.height = (self.angle - min_theta) / angle_range
        super().start(event)


class SdkDockWithCube(ActionNode):
    "Uses SDK's dock_with_cube method."
    def __init__(self, object=None, abort_on_stop=False, **action_kwargs):
        self.object = object
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop=abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and \
                isinstance(event.data,cozmo.objects.LightCube):
            self.object = event.data
        super().start(event)

    def action_launcher(self):
        if self.object is None:
            raise ValueError('No cube to dock with')
        return self.robot.dock_with_cube(self.object, **self.action_kwargs)


class SdkPickUpObject(ActionNode):
    "Uses SDK's pick_up_object method."
    def __init__(self, object=None, abort_on_stop=False, **action_kwargs):
        self.object = object
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop=abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and \
                isinstance(event.data,cozmo.objects.LightCube):
            self.object = event.data
        super().start(event)

    def action_launcher(self):
        if self.object is None:
            raise ValueError('No object to pick up')
        return self.robot.pickup_object(self.object, **self.action_kwargs)


class SdkPlaceObjectOnGroundHere(ActionNode):
    "Uses SDK's place_object_on_ground_here method."
    def __init__(self, object=None, abort_on_stop=False, **action_kwargs):
        self.object = object
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop=abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and \
                isinstance(event.data,cozmo.objects.LightCube):
            self.object = event.data
        super().start(event)

    def action_launcher(self):
        if self.object is None:
            raise ValueError('No object to place')
        return self.robot.place_object_on_ground_here(self.object, **self.action_kwargs)

class SdkPlaceOnObject(ActionNode):
    "Uses SDK's place_on_object method."
    def __init__(self, object=None, abort_on_stop=False, **action_kwargs):
        self.object = object
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop=abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and \
                isinstance(event.data,cozmo.objects.LightCube):
            self.object = event.data
        super().start(event)

    def action_launcher(self):
        if self.object is None:
            raise ValueError('No object to place')
        return self.robot.place_on_object(self.object, **self.action_kwargs)


class SdkRollCube(ActionNode):
    "Uses SDK's roll_cube method."
    def __init__(self, object=None, abort_on_stop=True, **action_kwargs):
        self.object = object
        self.action_kwargs = action_kwargs
        super().__init__(abort_on_stop=abort_on_stop)

    def start(self,event=None):
        if self.running: return
        if isinstance(event, DataEvent) and \
                isinstance(event.data,cozmo.objects.LightCube):
            self.object = event.data
        super().start(event)

    def action_launcher(self):
        if self.object is None:
            raise ValueError('No object to roll')
        return self.robot.roll_cube(self.object, **self.action_kwargs)

# Note: additional nodes for object manipulation are in pickup.fsm.

#________________ Animations ________________

class AnimationNode(ActionNode):
    def __init__(self, anim_name='anim_bored_01', **kwargs):
        self.anim_name = anim_name
        self.action_kwargs = kwargs
        super().__init__()

    def action_launcher(self):
        return self.robot.play_anim(self.anim_name, **self.action_kwargs)

class AnimationTriggerNode(ActionNode):
    def __init__(self, trigger=cozmo.anim.Triggers.CubePouncePounceNormal, **kwargs):
        if not isinstance(trigger, cozmo.anim._AnimTrigger):
            raise TypeError('%s is not an instance of cozmo.anim._AnimTrigger' %
                            repr(trigger))
        self.trigger = trigger
        self.action_kwargs = kwargs
        super().__init__()

    def action_launcher(self):
        return self.robot.play_anim_trigger(self.trigger, **self.action_kwargs)

#________________ Behaviors ________________

class StartBehavior(StateNode):
    def __init__(self, behavior=None, stop_on_exit=True):
        if not isinstance(behavior, cozmo.behavior._BehaviorType):
            raise ValueError("'%s' isn't an instance of cozmo.behavior._BehaviorType" %
                             repr(behavior))
        self.behavior = behavior
        self.behavior_handle = None
        self.stop_on_exit = stop_on_exit
        super().__init__()

    def __repr__(self):
        if self.behavior_handle:
            return '<%s %s active=%s>' % \
                   (self.__class__.__name__, self.name, self.behavior_handle.is_active)
        else:
            return super().__repr__()

    def start(self,event=None):
        if self.running: return
        super().start(event)
        try:
            if self.robot.behavior_handle:
                self.robot.behavior_handle.stop()
        except: pass
        finally:
            self.robot.behavior_handle = None
        self.behavior_handle = self.robot.start_behavior(self.behavior)
        self.robot.behavior_handle = self.behavior_handle
        self.post_completion()

    def stop(self):
        if not self.running: return
        if self.stop_on_exit and self.behavior_handle is self.robot.behavior_handle:
            self.robot.behavior_handle.stop()
            self.robot.behavior_handle = None
        super().stop()

class StopBehavior(StateNode):
    def start(self,event=None):
        if self.running: return
        super().start(event)
        try:
            if self.robot.behavior_handle:
                self.robot.behavior_handle.stop()
        except: pass
        self.robot.behavior_handle = None
        self.post_completion()

class FindFaces(StartBehavior):
    def __init__(self,stop_on_exit=True):
        super().__init__(cozmo.robot.behavior.BehaviorTypes.FindFaces,stop_on_exit)

class KnockOverCubes(StartBehavior):
    def __init__(self,stop_on_exit=True):
        super().__init__(cozmo.robot.behavior.BehaviorTypes.KnockOverCubes,stop_on_exit)

class LookAroundInPlace(StartBehavior):
    def __init__(self,stop_on_exit=True):
        super().__init__(cozmo.robot.behavior.BehaviorTypes.LookAroundInPlace,stop_on_exit)

class PounceOnMotion(StartBehavior):
    def __init__(self,stop_on_exit=True):
        super().__init__(cozmo.robot.behavior.BehaviorTypes.PounceOnMotion,stop_on_exit)

class RollBlock(StartBehavior):
    def __init__(self,stop_on_exit=True):
        super().__init__(cozmo.robot.behavior.BehaviorTypes.RollBlock,stop_on_exit)

class StackBlocks(StartBehavior):
    def __init__(self,stop_on_exit=True):
        super().__init__(cozmo.robot.behavior.BehaviorTypes.StackBlocks,stop_on_exit)

#________________ Multiprocessing ________________

class LaunchProcess(StateNode):

    def __init__(self):
        super().__init__()
        self.process = None

    @staticmethod
    def process_workhorse(reply_token):
        """
        Override this static method with the code to do your computation.
        The method must be static because we can't pickle methods of StateNode
        instances.
        """
        print('*** Failed to override process_workhorse for LaunchProcess node ***')
        print('Sleeping for 2 seconds...')
        time.sleep(2)
        # A process returns its result to the caller as an event.
        result = 42

        LaunchProcess.post_event(reply_token,DataEvent(result))  # source must be None for pickling
        LaunchProcess.post_event(reply_token,CompletionEvent()) # we can post more than one event

    @staticmethod
    def post_event(reply_token,event):
        id,queue = reply_token
        event_pair = (id, event)
        queue.put(event_pair)

    def create_process(self, reply_token):
        p = Process(target=self.__class__.process_workhorse,
                    args=[reply_token])
        return p

    def start(self, event=None):
        super().start(event)
        reply_token = (id(self), self.robot.erouter.interprocess_queue)
        self.process = self.create_process(reply_token)
        self.robot.erouter.add_process_node(self)
        self.process.start()
        print('Launched', self.process)

    def stop(self):
        if self.process:
            print('Exiting',self.process,self.process.is_alive())
            self.process = None
        super().stop()
        self.robot.erouter.delete_process_node(self)

