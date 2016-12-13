"""
  The Event Router and Event Listener

  This file implements an event router scheme modeled after the
  one in Tekkotsu.

"""

import cozmo

from .trace import TRACE

def set_robot(_robot):
    global robot
    robot = _robot

def get_robot():
    if robot:
        return robot
    else:
        raise Exception("Don't have a robot.")

#________________ Event base class ________________

class Event:
    """Base class for all events."""
    def __init__(self):
        self.source = None
    cozmo_evt_type = None
    def generator(self,cozmo_evt): pass


#________________ Event Router ________________

class EventRouter:
    """An event router drives the state machine."""
    def __init__(self):
        # dispatch_table: event_class -> (source,listener)...
        self.dispatch_table = dict()
        # listener_registry: listener -> (event_class, source)...
        self.listener_registry = dict()

    def add_listener(self, listener, event_type, source):
        if not issubclass(event_type, Event):
            raise TypeError('% is not an Event' % event_type)
        source_dict = self.dispatch_table.get(event_type)
        if source_dict is None:
            source_dict = dict()
            # start a cozmo event handler if this event type requires one
            if event_type.cozmo_evt_type:
                coztype = event_type.cozmo_evt_type
                if not issubclass(coztype, cozmo.event.Event):
                    raise ValueError('%s cozmo_evt_type %s not a subclass of cozmo.event.Event' % (event_type, coztype))
                world = get_robot().world
                world.add_event_handler(coztype, event_type.generator)
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
        source_dict = self.dispatch_table.get(event_type)
        if source_dict is None: return
        handlers = source_dict.get(source)
        if handlers is None: return
        try:
            # print('erouter removing',listener,'for',event_type,source)
            handlers.remove(listener.handle_event)
        except: pass
        if handlers == []:
            del source_dict[source]
        if len(source_dict) == 0:   # no one listening for this event
            del self.dispatch_table[event_type]
            # remove the cozmo event handler if there was one
            if event_type.cozmo_evt_type:
                world = get_robot().world
                coztype = event_type.cozmo_evt_type
                world.remove_event_handler(coztype, event_type.generator)

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
            if TRACE.trace_level >= TRACE.listener_invocation:
                print('TRACE%d:' % TRACE.listener_invocation, self, 'receiving', event)
            get_robot().loop.call_soon(listener,event)

erouter = EventRouter()

#________________ Event Listener ________________

class EventListener:
    """Parent class for both StateNode and Transition."""
    def __init__(self):
        self.running = False
        rep = object.__repr__(self)
        self.name = rep[1+rep.rfind(' '):-1]  # name defaults to hex address
        self.polling_interval = None
        self.poll_handle = None

    def __repr__(self):
        return '<%s %s>' % (self.__class__.__name__, self.name)

    def set_name(self,name):
        if not isinstance(name,str):
            raise ValueError('name must be a string, not %s' % name)
        self.name = name
        return self

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
        if TRACE.trace_level >= TRACE.polling:
            print('TRACE%d: polling' % TRACE.polling, self)
        print('%s has no poll() method' % self)
