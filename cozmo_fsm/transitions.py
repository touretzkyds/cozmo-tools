import random

from .base import *
from .events import *
from .nodes import Say, Iterate

class NullTrans(Transition):
    """Transition fires immediately; does not require an event to trigger it."""
    def start(self,event=None):
        if self.running: return
        super().start(event)
        # Don't fire immediately on start because the source node(s) may
        # have other startup calls to make. Give them time to finish.
        self.robot.loop.call_soon(self.fire)


class CSFEventBase(Transition):
    """Base class for Completion, Success, and Failure Events"""
    def __init__(self,event_type,count=None):
        super().__init__()
        self.event_type = event_type
        self.count = count

    def start(self,event=None):
        if self.running: return
        super().start(event)
        self.observed_sources = set()
        for source in self.sources:
            self.robot.erouter.add_listener(self, self.event_type, source)

    def handle_event(self,event):
        if not self.running: return
        if TRACE.trace_level >= TRACE.listener_invocation:
            print('TRACE%d: %s is handling %s' %
                  (TRACE.listener_invocation, self,event))
        super().handle_event(event)
        if isinstance(event, self.event_type):
            self.observed_sources.add(event.source)
            if len(self.observed_sources) >= (self.count or len(self.sources)):
                self.fire(event)
        else:
            raise ValueError("%s can't handle %s" % (self.event_type, event))

class CompletionTrans(CSFEventBase):
    """Transition fires when source nodes complete."""
    def __init__(self,count=None):
        super().__init__(CompletionEvent,count)

class SuccessTrans(CSFEventBase):
    """Transition fires when source nodes succeed."""
    def __init__(self,count=None):
        super().__init__(SuccessEvent,count)

class FailureTrans(CSFEventBase):
    """Transition fires when source nodes fail."""
    def __init__(self,count=None):
        super().__init__(FailureEvent,count)

class CNextTrans(CSFEventBase):
    """Transition fires when source nodes complete."""
    def __init__(self,count=None):
        super().__init__(CompletionEvent,count)

    def fire(self, event=None):
        super().fire(Iterate.NextEvent())


class NextTrans(Transition):
    """Transition sends a NextEvent to its target nodes to advance an iterator."""
    def start(self, event=None):
        super().start(event)
        self.fire(Iterate.NextEvent())


class SayDataTrans(Transition):
    """Converts a DataEvent to Say.SayDataEvent so we can speak the data."""
    def start(self,event=None):
        if self.running: return
        super().start(event)
        for source in self.sources:
            self.robot.erouter.add_listener(self, DataEvent, source)
            self.robot.erouter.add_listener(self, Say.SayDataEvent, source)

    def handle_event(self,event):
        super().handle_event(event)
        if isinstance(event, Say.SayDataEvent):
            say_data_event = event
        elif isinstance(event, DataEvent):
            say_data_event = Say.SayDataEvent(event.data)
        else:
            return
        self.fire(say_data_event)            


class TimerTrans(Transition):
    """Transition fires when the timer has expired."""
    def __init__(self,duration=None):
        if not isinstance(duration, (int, float)) or duration < 0:
            raise ValueError("TimerTrans requires a positive number for duration, not %s" % duration)
        super().__init__()
        self.set_polling_interval(duration)

    def poll(self):
        self.fire()


class TapTrans(Transition):
    """Transition fires when a cube is tapped."""
    def __init__(self,cube=None):
        super().__init__()
        self.cube = cube

    def start(self,event):
        if self.running: return
        super().start(event)
        self.robot.erouter.add_listener(self,TapEvent,self.cube)

    def handle_event(self,event):
        if self.cube:
            self.fire(event)
        else:
            self.handle = \
                self.robot.loop.call_later(Transition.default_value_delay, self.fire, event)

class DataTrans(Transition):
    """Transition fires when data matches."""
    def __init__(self, data=None):
        super().__init__()
        self.data = data

    def start(self,event=None):
        if self.running: return
        super().start(event)
        for source in self.sources:
            self.robot.erouter.add_listener(self, DataEvent, source)

    def handle_event(self,event):
        super().handle_event(event)
        if isinstance(event,DataEvent):
            if self.data is None or \
               (isinstance(self.data, type) and isinstance(event.data, self.data)):
                    self.fire(event)
            else:
                try:
                    if self.data == event.data:    # error if == barfs
                        self.fire(event)
                except TypeError: pass
        else:
            raise TypeError('%s is not a DataEvent' % event)

class ArucoTrans(Transition):
    """Fires if one of the specified markers is visible"""
    def __init__(self,markers=None):
        super().__init__()
        self.polling_interval = 0.1
        self.markers = markers

    def poll(self,event=None):
        if self.markers is None:
            if self.robot.world.aruco.seenMarkers != []:
                self.fire()
        elif isinstance(self.markers,set):
            if self.markers.intersection(self.robot.world.aruco.seenMarkers) != set():
                self.fire()
        elif self.markers in self.robot.world.aruco.seenMarkers:
            self.fire()


class TextMsgTrans(Transition):
    """Transition fires when message matches."""
    def __init__(self,message=None):
        super().__init__()
        self.source = message

    def start(self,event=None):
        if self.running: return
        super().start(event)
        # The 'source' is the message text.
        self.robot.erouter.add_listener(self, TextMsgEvent, self.source)

    def handle_event(self,event):
        super().handle_event(event)
        if isinstance(event,TextMsgEvent): # erouter checked for match
            self.fire(event)
        else:
            raise TypeError('%s is not a TextMsgEvent' % event)

class RandomTrans(Transition):
    """Picks a destination node at random."""
    def start(self,event=None):
        if self.running: return
        super().start(event)
        # Don't fire immediately on start because the source node(s) may
        # have other startup calls to make. Give them time to finish.
        self.robot.loop.call_soon(self.fire)  # okay to use Transition.fire

    def fire2(self,event):
        """Overrides Transition.fire2 to only start one randomly-chosen destination node."""
        dest = random.choice(self.destinations)
        dest.start(event)
