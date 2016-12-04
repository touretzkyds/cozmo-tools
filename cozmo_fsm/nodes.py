from .events import *
from .base import *

from cozmo.util import distance_mm, speed_mmps, degrees

class ActionNode(StateNode):
    def __init__(self,name):
        super().__init__(name)
        self.handle = None

    def post_when_complete(self):
       get_robot().loop.create_task(self.wait_for_completion())

    async def wait_for_completion(self):
        await self.handle.wait_for_completed()
        self.post_completion()


class Forward(ActionNode):
    def __init__(self, name, distance=distance_mm(50), speed=speed_mmps(50), **kwargs):
        super().__init__(name)
        self.distance = distance
        self.speed = speed
        self.kwargs = kwargs

    def start(self,event=None):
        super().start(event)
        self.handle = get_robot().drive_straight(self.distance,self.speed,**self.kwargs)
        self.post_when_complete()


class Turn(ActionNode):
    def __init__(self, name, angle=degrees(90)):
        super().__init__(name)
        self.angle = angle

    def start(self,event=None):
        super().start(event)
        self.handle = get_robot().turn_in_place(self.angle)
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
        self.handle = get_robot().say_text(self.text,**self.kwargs)
        self.post_when_complete()

