"""
    The base Event class is imported from erouter.py.
    All other events are defined here.
"""

from .erouter import Event

class CompletionEvent(Event):
    """Signals completion of a state node's action."""
    def __init__(self,source):
        super().__init__()
        self.source = source

    def __repr__(self):
        return '<%s for %s>' % (self.__class__.__name__, self.source.name)

class FailureEvent(Event):
    """Signals failure of a state node's action."""
    def __init__(self,source,details):
        super().__init__()
        self.source = source
        self.details = details

    def __repr__(self):
        if isinstance(self.details, cozmo.action.Action):
            reason = details.failure_reason[0]
        else:
            reason = details
        return '<%s for %s: %s>' % (self.__class__.__name__, self.source.name, reason)

class DataEvent(Event):
    """Signals a data item broadcase by the node."""
    def __init__(self,value):
        super().__init__()
        self.source = value

class TapEvent(Event):
    """Signals detection of a tap on a light cube."""
    def __init__(self,cube,intensity,duration):
        super().__init__()
        self.source = cube
        self.intensity = intensity
        self.duration = duration
