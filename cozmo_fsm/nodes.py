from .events import *
from .base import *

from cozmo.util import distance_mm, speed_mmps, degrees

class ActionNode(StateNode):
    def __init__(self,name):
        super().__init__(name)
        self.cozmo_action_handle = None

    def post_when_complete(self):
       get_robot().loop.create_task(self.wait_for_completion())

    async def wait_for_completion(self):
        async_task = self.cozmo_action_handle.wait_for_completed()
        await async_task
        self.post_completion()

    def stop(self):
        if self.cozmo_action_handle and self.cozmo_action_handle.is_running:
            self.cozmo_action_handle.abort()
        super().stop()


class Forward(ActionNode):
    def __init__(self, name, distance=distance_mm(50),
                 speed=speed_mmps(50), **kwargs):
        super().__init__(name)
        if isinstance(distance,int) or isinstance(distance,float):
            distance = distance_mm(distance)
        if isinstance(speed,int) or isinstance(speed,float):
            speed = speed_mmps(speed)
        self.distance = distance
        self.speed = speed
        self.kwargs = kwargs

    def start(self,event=None):
        super().start(event)
        self.cozmo_action_handle = \
            get_robot().drive_straight(self.distance,self.speed,**self.kwargs)
        self.post_when_complete()


class Turn(ActionNode):
    def __init__(self, name, angle=degrees(90)):
        if isinstance(angle,int) or isinstance(angle,float):
            angle = degrees(angle)
        super().__init__(name)
        self.angle = angle

    def start(self,event=None):
        super().start(event)
        self.cozmo_action_handle = get_robot().turn_in_place(self.angle)
        self.post_when_complete()


class Say(ActionNode):
    """Speaks some text, then posts a completion event."""
    def __init__(self, name, text, **kwargs):
        super().__init__(name)
        self.text = text
        self.kwargs = kwargs

    def start(self,event=None):
        super().start(event)
        print("Speaking: '",self.text,"'",sep='')
        self.cozmo_action_handle = get_robot().say_text(self.text,**self.kwargs)
        self.post_when_complete()


class AnimationNode(StateNode):
    pass  # TODO


class BehaviorNode(StateNode):
    pass # TODO


