"""
  Events and the Event Router

  This file implements an event router scheme modeled after the
  one in Tekkotsu.

"""

global erouter

class Event:
    """Base class for all events."""
    def __init__(self):
        self.source = None


class CompletionEvent(Event):
    """Signals completion of a state node's action."""
    def __init__(self,source):
        super().__init__()
        self.source = source


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


################

class EventRouter:
    """An event router drives the state machine."""
    def __init__(self):
        self.dispatch_table = dict(
               {
                CompletionEvent : dict() ,
        TapEvent : dict()
               }
            )

    def start(self):
        pass
        ### TODO: add a handler for cube tap API events

    def stop(self):
        pass
        ### TODO: remove all handlers for API events

    def add_listener(self, listener, event_type, source):
        if not issubclass(event_type, Event):
            raise TypeError('% is not an Event' % event_type)
        source_dict = self.dispatch_table.get(event_type, dict())
        handlers = source_dict.get(source, [])
        handlers.append(listener.handle_event)
        source_dict[source] = handlers
        self.dispatch_table[event_type] = source_dict

    def remove_listener(self, listener, event_type, source):
        if not issubclass(event_type, Event):
            raise TypeError('% is not an Event' % event_type)
        source_dict = self.dispatch_table.get(event_type, None)
        if source_dict is None: return
        handlers = source_dict.get(source, None)
        if handlers is None: return
        try:
            handlers.remove(listener.handle_event)
        except: pass

    def _get_listeners(self,event):
        source_dict = self.dispatch_table.get(type(event), None)
        if source_dict is None:  # no listeners for this event type
            return []
        matches = source_dict.get(event.source, [])
        wildcards = source_dict.get(None, [])
        if not wildcards:
            return matches
        else:  # append specific listeners and wildcard listeners
            matches = matches.copy()
            matches.append(wildcards)
            return matches

    def post(self,event):
        if not isinstance(event,Event):
            raise TypeError('%s is not an Event' % event)
        for listener in self._get_listeners(event):
            print('scheduling',listener,'on',event)
            robot.loop.call_soon(listener,event)

erouter = EventRouter()

################

class EventListener:
    """Parent class for both StateNode and Transition."""
    def __init__(self):
        self.running = False

    def start(self):
        self.running = True
        print(self,'running')

    def stop(self):
        self.running = False
        print(self,'stopped')

    def handle_event(self, event):
        if not running:
            print('Handler',self,'not running, but received',event)
        else:
            pass

