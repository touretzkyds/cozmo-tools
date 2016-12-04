from .events import *
from .base import *


class NullTrans(Transition):
    def start(self):
        super.start()
        pass   ### TODO: finish this


class CompletionTrans(Transition):
    """Transition fires when a source node completes."""
    def start(self):
        super().start()
        for source in self.sources:
            erouter.add_listener(self,CompletionEvent,source)

    def stop(self):
        super().stop()
        for source in self.sources:
            erouter.remove_listener(self,CompletionEvent,source)

    def handle_event(self,event):
        print('CompletionTrans',self,'got',event)
        self.fire(event)


class TimerTrans(Transition):
    """Transition fires when the timer has expired."""
    def __init__(self,name,duration):
        super().__init__(name)
        self.duration = duration

    def start(self):
        super().start()
        self.handle = robot.loop.call_later(self.duration, self.fire)

    def stop(self):
        super().stop()

    def fire(self):
        print(self.name,'trying to fire')
        if not self.running: return
        super().fire()


class SignalTrans(Transition):
    """Transition fires when value matches."""
    def __init__(self,name,value=None):
        super().__init__(name)
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
        if isinstance(event,DataEvent):
            if self.value is not None
                self.fire(event)
            else: # wildcard case: fire only if nothing else does
                self.handle = robot.loop.call_later(0.1, self.fire, event)
        else:
            raise TypeError('%s is not a DataEvent' % event)
