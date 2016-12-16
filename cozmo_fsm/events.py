"""
    The base Event class is imported from erouter.py.
    All other events are defined here.
"""

import cozmo

from .evbase import Event

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
    """Signals a data item broadcasted by the node."""
    def __init__(self,source,value):
        super().__init__()
        self.source = value

#________________ Cozmo-generated events ________________

class CozmoGeneratedEvent(Event):
    def __init__(self,source,params):
        super().__init__()
        self.source = source
        self.params = params
    # Note regarding generator(): we're going to curry this function
    # to supply EROUTER and EVENT_CLASS as the first two arguments.
    def generator(EROUTER, EVENT_CLASS, cozmo_event, obj, **kwargs):
        our_event = EVENT_CLASS(obj,kwargs)
        EROUTER.post(our_event)

class TapEvent(CozmoGeneratedEvent):
    cozmo_evt_type = cozmo.objects.EvtObjectTapped

class FaceEvent(CozmoGeneratedEvent):
    cozmo_evt_type = cozmo.faces.EvtFaceAppeared
