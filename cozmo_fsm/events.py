"""
  Events and the Event Router

  This file implements an event router scheme modeled after the
  one in Tekkotsu.

"""

def set_robot(_robot):
    global robot
    robot = _robot

def get_robot():
    if robot:
        return robot
    else:
        raise Exception("Don't have a robot.")

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

    def __repr__(self):
        return '<%s for %s>' % (self.__class__.__name__, self.source.name)

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
        self.listener_registry = dict()   # indexed by listener

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
        reg_entry = self.listener_registry.get(listener,[])
        reg_entry.append((event_type,source))
        self.listener_registry[listener] = reg_entry

    def remove_listener(self, listener, event_type, source):
        if not issubclass(event_type, Event):
            raise TypeError('% is not an Event' % event_type)
        source_dict = self.dispatch_table.get(event_type, None)
        if source_dict is None: return
        handlers = source_dict.get(source, None)
        if handlers is None: return
        try:
            # print('erouter removing',listener,'for',event_type,source)
            handlers.remove(listener.handle_event)
        except: pass

    def remove_all_listener_entries(self, listener):
        for event_type, source in erouter.listener_registry.get(listener,[]):
            erouter.remove_listener(listener, event_type, source)
        try:
            del erouter.listener_registry[listener]
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
            # print('scheduling',listener,'on',event)
            get_robot().loop.call_soon(listener,event)

erouter = EventRouter()

################

class EventListener:
    """Parent class for both StateNode and Transition."""
    def __init__(self):
        self.running = False
        rep = object.__repr__(self)
        self.name = rep[1+rep.rfind(' '):-1]  # name defaults to hex address
        self.polling_interval = None
        self.poll_handle = None

    def set_name(self,name):
        self.name = name

    def start(self):
        self.running = True
        if self.polling_interval:
            self.next_poll()
        #print(self,'running')

    def stop(self):
        if not self.running: return
        self.running = False
        if self.poll_handle: self.poll_handle.cancel()
        erouter.remove_all_listener_entries(self)
        #print(self,'stopped')

    def handle_event(self, event):
        pass

    def set_polling_interval(self,interval):
        if isinstance(interval, (int,float)):
            self.polling_interval = interval
        else:
            raise TypeError('interval must be a number')

    def next_poll(self):
        """Call this to schedule the next polling interval."""
        self.poll_handle = \
            get_robot().loop.call_later(self.polling_interval, self.poll)

    def poll(self):
        """Dummy polling function in case sublass neglects to supply one."""
        print('%s has no poll() method' % self)
