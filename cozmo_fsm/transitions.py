from .events import *
from .base import *

class NullTrans(Transition):
    def __init__(self):
        super().__init__()

    def start(self):
        super().start()
        get_robot().loop.call_soon(self.fire)


class CompletionTrans(Transition):
    """Transition fires when a source node completes."""
    def __init__(self,count=None):
        super().__init__()
        self.count = count

    def __repr__(self):
        srcs = ','.join(node.name for node in self.sources)
        dests = ','.join(node.name for node in self.destinations)
        return '<%s %s: %s=>%s >' % \
            (self.__class__.__name__, self.name, srcs, dests)

    def start(self):
        if self.running: return
        super().start()
        self.completed = set()
        for source in self.sources:
            erouter.add_listener(self,CompletionEvent,source)

    def handle_event(self,event):
        print(self,'is handling',event)
        if not self.running: return
        super().handle_event(event)
        if isinstance(event,CompletionEvent):
            self.completed.add(event.source)
            if len(self.completed) >= (self.count or len(self.sources)):
                self.fire(event)
        else:
            raise ValueError("CompletionTrans can't handle %s" % event)


class TimerTrans(Transition):
    """Transition fires when the timer has expired."""
    def __init__(self,duration):
        super().__init__()
        self.duration = duration

    def start(self):
        super().start()
        self.handle = get_robot().loop.call_later(self.duration, self.fire)

    def stop(self):
        super().stop()

    def fire(self):
        print(self.name,'trying to fire')
        if not self.running: return
        super().fire()


class SignalTrans(Transition):
    """Transition fires when value matches."""
    def __init__(self,value=None):
        super().__init__()
        self.value = value

    def start(self):
        super().start()
        for source in self.sources:
            erouter.add_listener(self,DataEvent,self.value)

    def stop(self):
        for source in self.sources:
            erouter.remove_listener(self,DataEvent,source)
        super.stop()

    def handle_event(self,event):
        super().handle_event(event)
        if isinstance(event,DataEvent):
            if self.value is not None:
                self.fire(event)
            else: # wildcard case: fire only if nothing else does
                self.handle = get_robot().loop.call_later(0.1, self.fire, event)
        else:
            raise TypeError('%s is not a DataEvent' % event)
