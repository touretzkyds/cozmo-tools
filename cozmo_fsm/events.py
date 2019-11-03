"""
    The base Event class is imported from evbase.py.
    All other events are defined here.
"""

import cozmo

from .evbase import Event

class CompletionEvent(Event):
    """Signals completion of a state node's action."""
    pass


class SuccessEvent(Event):
    """Signals success of a state node's action."""
    def __init__(self,details=None):
        super().__init__()
        self.details = details


class FailureEvent(Event):
    """Signals failure of a state node's action."""
    def __init__(self,details=None):
        super().__init__()
        self.details = details

    def __repr__(self):
        if isinstance(self.details, cozmo.action.Action):
            reason = self.details.failure_reason[0]
        else:
            reason = self.details
        return '<%s for %s: %s>' % (self.__class__.__name__, self.source.name, reason)


class DataEvent(Event):
    """Signals a data item broadcasted by the node."""
    def __init__(self,data):
        super().__init__()
        self.data = data


class TextMsgEvent(Event):
    """Signals a text message broadcasted to the state machine."""
    def __init__(self,string,words=None,result=None):
        super().__init__()
        self.string = string
        self.words = words or string.split(None)
        self.result = result

class SpeechEvent(Event):
    """Results of speech recognition process."""
    def __init__(self,string,words=None,result=None):
        super().__init__()
        self.string = string
        self.words = words
        self.result = result

class PilotEvent(Event):
    """Results of a pilot request."""
    def __init__(self,status,*args):
        super().__init__()
        self.status = status
        self.args = args

    def __repr__(self):
        try:
            src_string = self.source.name
        except:
            src_string = repr(self.source)
        return '<%s %s from %s>' % (self.__class__.__name__, self.status.__name__, src_string)


#________________ Cozmo-generated events ________________

class CozmoGeneratedEvent(Event):
    def __init__(self,source,params):
        super().__init__()
        self.source = source
        self.params = params
    # Note regarding generator(): we're going to curry this function
    # to supply EROUTER and EVENT_CLASS as the first two arguments.
    def generator(EROUTER, EVENT_CLASS, cozmo_event, obj=None, **kwargs):
        our_event = EVENT_CLASS(obj,kwargs)
        EROUTER.post(our_event)

class TapEvent(CozmoGeneratedEvent):
    cozmo_evt_type = cozmo.objects.EvtObjectTapped

class FaceEvent(CozmoGeneratedEvent):
    cozmo_evt_type = cozmo.faces.EvtFaceAppeared

class ObservedMotionEvent(CozmoGeneratedEvent):
    cozmo_evt_type = cozmo.camera.EvtRobotObservedMotion

    def __repr__(self):
        top = self.params['has_top_movement']
        left = self.params['has_left_movement']
        right = self.params['has_right_movement']
        movement = ''
        if top:
            pos = self.params['top_img_pos']
            movement = movement + ('' if (movement=='') else ' ') + \
                       ('top:(%d,%d)' % (pos.x,pos.y))
        if left:
            pos = self.params['left_img_pos']
            movement = movement + ('' if (movement=='') else ' ') + \
                       ('left:(%d,%d)' % (pos.x,pos.y))
        if right:
            pos = self.params['right_img_pos']
            movement = movement + ('' if (movement=='') else ' ') + \
                       ('right:(%d,%d)' % (pos.x,pos.y))
        if movement == '':
            pos = self.params['img_pos']
            movement = movement + ('' if (movement=='') else ' ') + \
                       ('broad:(%d,%d)' % (pos.x,pos.y))
        return '<%s %s>' % (self.__class__.__name__, movement)


class UnexpectedMovementEvent(CozmoGeneratedEvent):
    cozmo_evt_type = cozmo.robot.EvtUnexpectedMovement

    def __repr__(self):
        side = self.params['movement_side']
        # side.id == 0 means the movement_side is "unknown"
        # Occurs when reaction triggers are disabled (as is normally the case).
        side_string = ' '+side.name if side.id > 0 else ''
        return '<%s %s%s>' % (self.__class__.__name__,
                              self.params['movement_type'].name,
                              side_string)

