from .erouter import get_robot
from .base import *
from .events import *

class NullTrans(Transition):
    """Transition fires immediately; does not require an event to trigger it."""
    def __init__(self):
        super().__init__()

    def start(self,event=None):
        if self.running: return
        super().start(event)
        # Don't fire immediately on start because the source node(s) may
        # have other startup calls to make. Give them time to finish.
        get_robot().loop.call_soon(self.fire)


class CompletionTrans(Transition):
    """Transition fires when a source node completes."""
    def __init__(self,count=None):
        super().__init__()
        self.count = count

    def start(self,event=None):
        if self.running: return
        super().start(event)
        self.completed_sources = set()
        for source in self.sources:
            erouter.add_listener(self,CompletionEvent,source)

    def handle_event(self,event):
        if not self.running: return
        if TRACE.trace_level >= TRACE.listener_invocation:
            print('TRACE:',self,'is handling',event)
        super().handle_event(event)
        if isinstance(event,CompletionEvent):
            self.completed_sources.add(event.source)
            if len(self.completed_sources) >= (self.count or len(self.sources)):
                self.fire(event)
        else:
            raise ValueError("CompletionTrans can't handle %s" % event)


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
        erouter.add_listener(self,TapEvent,self.cube)

    def handle_event(self,event):
        if self.cube:
            self.fire(event)
        else:
            self.handle = get_robot().loop.call_later(0.1, self.fire, event)


class SignalTrans(Transition):
    """Transition fires when value matches."""
    def __init__(self,value=None):
        super().__init__()
        self.value = value

    def start(self):
        if self.running: return
        super().start()
        for source in self.sources:
            erouter.add_listener(self,DataEvent,self.value)

    def handle_event(self,event):
        super().handle_event(event)
        if isinstance(event,DataEvent):
            if self.value is not None:
                self.fire(event)
            else: # wildcard case: fire only if nothing else does
                self.handle = get_robot().loop.call_later(0.1, self.fire, event)
        else:
            raise TypeError('%s is not a DataEvent' % event)
