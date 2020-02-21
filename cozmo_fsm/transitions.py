import random
import re

from .base import *
from .events import *
from .nodes import Say, Iterate

class NullTrans(Transition):
    """Transition fires immediately; does not require an event to trigger it."""
    def start(self):
        if self.running: return
        super().start()
        # Don't fire immediately on start because the source node(s) may
        # have other startup calls to make. Give them time to finish.
        self.handle = self.robot.loop.call_soon(self.fire)

    def stop(self):
        if self.handle:
            print(self, 'cancelling', self.handle)
            self.handle.cancel()
            self.handle = None
        super().stop()

    def fire(self, event=None):
        self.handle = None
        super().fire(event)


class CSFEventBase(Transition):
    """Base class for Completion, Success, and Failure Events"""
    def __init__(self,event_type,count=None):
        super().__init__()
        self.event_type = event_type
        self.count = count

    def start(self):
        if self.running: return
        super().start()
        self.observed_sources = set()
        for source in self.sources:
            self.robot.erouter.add_listener(self, self.event_type, source)

    def handle_event(self,event):
        if not self.running:
            print('***',self,'got an event ', event, ' while not running!')
            return
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
        super().start()
        self.fire(Iterate.NextEvent())


class SayDataTrans(Transition):
    """Converts a DataEvent to Say.SayDataEvent so we can speak the data."""
    def start(self):
        if self.running: return
        super().start()
        for source in self.sources:
            self.robot.erouter.add_listener(self, DataEvent, source)
            self.robot.erouter.add_listener(self, Say.SayDataEvent, source)

    def handle_event(self,event):
        if not self.running: return
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
        if not self.running: return
        self.fire()


class TapTrans(Transition):
    """Transition fires when a cube is tapped."""
    def __init__(self,cube=None):
        super().__init__()
        self.cube = cube

    def start(self):
        if self.running: return
        super().start()
        self.robot.erouter.add_listener(self,TapEvent,self.cube)

    def handle_event(self,event):
        if self.cube:
            self.fire(event)
        else:
            self.handle = \
                self.robot.loop.call_later(Transition.default_value_delay, self.fire, event)


class ObservedMotionTrans(Transition):
    """Transition fires when motion is observed in the camera image."""
    def start(self):
        if self.running: return
        super().start()
        self.robot.erouter.add_listener(self,ObservedMotionEvent,None)

    def handle_event(self,event):
        if not self.running: return
        super().handle_event(event)
        self.fire(event)


class UnexpectedMovementTrans(Transition):
    """Transition fires when unexpected movement is detected."""
    def start(self):
        if self.running: return
        super().start()
        self.robot.erouter.add_listener(self,UnexpectedMovementEvent,None)

    def handle_event(self,event):
        if not self.running: return
        super().handle_event(event)
        self.fire(event)


class DataTrans(Transition):
    """Transition fires when data matches."""
    def __init__(self, data=None):
        super().__init__()
        self.data = data

    def start(self):
        if self.running: return
        super().start()
        for source in self.sources:
            if self.data is None or isinstance(self.data, type):
                self.robot.erouter.add_wildcard_listener(self, DataEvent, source)
            else:
                self.robot.erouter.add_listener(self, DataEvent, source)

    def handle_event(self,event):
        if not self.running: return
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
    def __init__(self,marker_ids=None):
        super().__init__()
        self.polling_interval = 0.1
        if isinstance(marker_ids,(list,tuple)):
            marker_ids = set(marker_ids)
        self.marker_ids = marker_ids

    def poll(self,event=None):
        if not self.running: return
        if self.marker_ids is None:
            if self.robot.world.aruco.seen_marker_ids != []:
                self.fire()
        elif isinstance(self.marker_ids,set):
            if self.marker_ids.intersection(self.robot.world.aruco.seen_marker_ids) != set():
                self.fire()
        elif self.marker_ids in self.robot.world.aruco.seen_marker_ids:
            self.fire()


class PatternMatchTrans(Transition):
    wildcard = re.compile('.*')
    
    def __init__(self, pattern=None, event_type=None):
        super().__init__()
        if pattern:
            pattern = re.compile(pattern)
        self.pattern = pattern
        self.event_type = event_type
    
    def start(self):
        if self.running: return
        super().start()
        if self.pattern is None:
            self.robot.erouter.add_wildcard_listener(self, self.event_type, None)
        else:
            self.robot.erouter.add_listener(self, self.event_type, None)

    def handle_event(self,event):
        if not self.running: return
        super().handle_event(event)
        if self.pattern is None:
            result = self.wildcard.match(event.string)
        else:
            result = self.pattern.match(event.string)
        if result:
            match_event = self.event_type(event.string,event.words,result)
            self.fire(match_event)

class TextMsgTrans(PatternMatchTrans):
    """Transition fires when text message event matches pattern."""
    def __init__(self,pattern=None):
        super().__init__(pattern,TextMsgEvent)


class HearTrans(PatternMatchTrans):
    """Transition fires if speech event matches pattern."""
    def __init__(self,pattern=None):
        super().__init__(pattern,SpeechEvent)


class PilotTrans(Transition):
    """Fires if a matching PilotEvent is observed."""
    def __init__(self,status=None):
        super().__init__()
        self.status = status

    def start(self):
        if self.running: return
        super().start()
        for source in self.sources:
            if self.status == None:
                self.robot.erouter.add_wildcard_listener(self, PilotEvent, source)
            else:
                self.robot.erouter.add_listener(self, PilotEvent, source)

    def handle_event(self,event):
        if not self.running: return
        super().handle_event(event)
        if self.status == None or self.status == event.status:
            self.fire(event)
        

class RandomTrans(Transition):
    """Picks a destination node at random."""
    def start(self):
        if self.running: return
        super().start()
        # Don't fire immediately on start because the source node(s) may
        # have other startup calls to make. Give them time to finish.
        self.handle = self.robot.loop.call_soon(self.fire)  # okay to use Transition.fire

    def stop(self):
        if self.handle:
            self.handle.cancel()
            self.handle = None
        super().stop()

    def fire2(self,event):
        """Overrides Transition.fire2 to only start one randomly-chosen destination node."""
        dest = random.choice(self.destinations)
        dest.start(event)
