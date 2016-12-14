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

# TODO: make an @cozmo_event(evt_type) decorator to auto generate?

class TapEvent(Event):
    """Signals detection of a tap on an object."""
    cozmo_evt_type = cozmo.objects.EvtObjectTapped
    def __init__(self,source,params):
        super().__init__()
        self.source = source
        self.params = params

    # Note that the argument list has SELF as the second arg because
    # we're going to curry this function to supply the erouter as the
    # first arg.
    def generator(erouter,SELF,obj,**kwargs):
        our_event = TapEvent(obj,kwargs)
        erouter.post(our_event)
