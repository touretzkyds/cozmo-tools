from math import sqrt

import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees

from .base import *
from .events import *

#________________ Ordinary Nodes ________________

class ParentCompletes(StateNode):
    def start(self,event=None):
        super().start(event)
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  '%s is causing %s to complete' % (self, self.parent))
        if parent:
            self.parent.post_completion()


class DriveWheels(StateNode):
    def __init__(self,l_wheel_speed,r_wheel_speed):
        super().__init__()
        self.l_wheel_speed = l_wheel_speed
        self.r_wheel_speed = r_wheel_speed

    def start(self,event=None):
        if self.running: return
        super().start(event)
        cor = self.robot.drive_wheels(self.l_wheel_speed, self.r_wheel_speed)
        self.handle = self.robot.loop.create_task(cor)

    def stop(self):
        if not self.running: return
        self.handle.cancel()
        cor = self.robot.drive_wheels(0, 0)
        self.handle = self.robot.loop.create_task(cor)
        super().stop()


class DriveForward(DriveWheels):
    def __init__(self, distance=50, speed=50):
        if distance < 0:
            distance = -distance
            speed = -speed
        super().__init__(speed,speed)
        self.distance = distance
        self.polling_interval = 0.1

    def start(self,event=None):
        if self.running: return
        super().start(event)
        self.start_position = self.robot.pose.position

    def poll(self):
        """See how far we've traveled"""
        p0 = self.start_position
        p1 = self.robot.pose.position
        diff = (p1.x - p0.x, p1.y - p0.y)
        dist = sqrt(diff[0]*diff[0] + diff[1]*diff[1])
        if dist >= self.distance:
            self.post_completion()
        else:
            self.next_poll()


#________________ Action Nodes ________________

class ActionNode(StateNode):
    relaunch_delay = 0.050 # 50 milliseconds

    def __init__(self):
        super().__init__()
        self.cozmo_action_handle = None

    def start(self,event=None):
        super().start(event)
        self.launch_or_retry()

    def launch_or_retry(self):
        try:
            result = self.action_launcher(self.robot)
        except cozmo.exceptions.RobotBusy:
            if TRACE.trace_level >= TRACE.statenode_startstop:
                print('TRACE%d:' % TRACE.statenode_startstop, self, 'launch_action raised RobotBusy')
            self.handle = self.robot.loop.call_later(self.relaunch_delay, self.launch_or_retry)
            return
        if isinstance(result, cozmo.action.Action):
            self.cozmo_action_handle = result
        else:
            raise ValueError("Result of %s launch_action() is not a cozmo.action.Action", self)
        self.post_when_complete()

    def action_launcher(self,robot):
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
                return
            else:
                self.post_failure(self.cozmo_action_handle)

    def stop(self):
        if not self.running: return
        if self.cozmo_action_handle and self.cozmo_action_handle.is_running:
            self.cozmo_action_handle.abort()
        super().stop()


class Say(ActionNode):
    """Speaks some text, then posts a completion event."""
    def __init__(self, text="I'm speechless", **kwargs):
        super().__init__()
        self.text = text
        self.kwargs = kwargs

    def start(self,event=None):
        if self.running: return
        super().start(event)
        print("Speaking: '",self.text,"'",sep='')

    def action_launcher(self,robot):
        return robot.say_text(self.text,**self.kwargs)


class Forward(ActionNode):
    def __init__(self, distance=distance_mm(50),
                 speed=speed_mmps(50), **kwargs):
        super().__init__()
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
        self.kwargs = kwargs

    def action_launcher(self,robot):
        return robot.drive_straight(self.distance, self.speed, **self.kwargs)


class Turn(ActionNode):
    def __init__(self, angle=degrees(90), **kwargs):
        super().__init__()
        if isinstance(angle, (int,float)):
            angle = degrees(angle)
        elif not isinstance(angle, cozmo.util.Angle):
            raise ValueError('%s angle must be a number or a cozmo.util.Angle' % self)
        self.angle = angle
        self.kwargs = kwargs

    def action_launcher(self,robot):
        return self.robot.turn_in_place(self.angle, **self.kwargs)


class AnimationNode(StateNode):
    pass  # TODO


class BehaviorNode(StateNode):
    pass # TODO


