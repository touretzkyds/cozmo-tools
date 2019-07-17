"""
  The Event Router and Event Listener

  This file implements an event router scheme modeled after the
  one in Tekkotsu.

"""

import functools

import cozmo

from .trace import TRACE

#________________ Event base class ________________

class Event:
    """Base class for all events."""
    def __init__(self, source=None):
        self.source = source

    cozmo_evt_type = None

    def generator(self,erouter,cozmo_evt): pass

    def __repr__(self):
        try:
            src_string = self.source.name
        except:
            src_string = repr(self.source)
        return '<%s for %s>' % (self.__class__.__name__, src_string)

#________________ Event Router ________________

class EventRouter:
    """An event router drives the state machine."""
    def __init__(self):
        # dispatch_table: event_class -> (source,listener)...
        self.dispatch_table = dict()
        # listener_registry: listener -> (event_class, source)...
        self.listener_registry = dict()
        # wildcard registry: true if listener is a wildcard (should run last)
        self.wildcard_registry = dict()
        # event generator objects
        self.event_generators = dict()
        # running processes
        self.processes = []

    def start(self):
        self.clear()
        self.poll_processes()

    def clear(self):
        self.dispatch_table.clear()
        self.listener_registry.clear()
        self.wildcard_registry.clear()
        self.event_generators.clear()
        self.processes = []

    def add_listener(self, listener, event_class, source):
        if not issubclass(event_class, Event):
            raise TypeError('% is not an Event' % event_type)
        source_dict = self.dispatch_table.get(event_class)
        if source_dict is None:
            source_dict = dict()
            # start a cozmo event handler if this event type requires one
            if event_class.cozmo_evt_type:
                coztype = event_class.cozmo_evt_type
                if not issubclass(coztype, cozmo.event.Event):
                    raise ValueError('%s cozmo_evt_type %s not a subclass of cozmo.event.Event' % (event_type, coztype))
                world = self.robot.world
                # supply the erouter and event type
                gen = functools.partial(event_class.generator, self, event_class)
                self.event_generators[event_class] = gen
                world.add_event_handler(coztype,gen)                                
        handlers = source_dict.get(source, [])
        handlers.append(listener.handle_event)
        source_dict[source] = handlers
        self.dispatch_table[event_class] = source_dict
        reg_entry = self.listener_registry.get(listener,[])
        reg_entry.append((event_class,source))
        self.listener_registry[listener] = reg_entry

    # Transitions like =Hear('foo')=> must use None as the source
    # value because they do the matching themselves instead of relying
    # on the event router. So to distinguish a wildcard =Hear=>
    # transition, which must be invoked last, from all the other Hear
    # transitions, we must register it specially.
    def add_wildcard_listener(self, listener, event_class, source):
        self.add_listener(listener, event_class, source)
        self.wildcard_registry[listener.handle_event] = True

    def remove_listener(self, listener, event_class, source):
        try:
            del self.wildcard_registry[listener.handle_event]
        except: pass
        if not issubclass(event_class, Event):
            raise TypeError('% is not an Event' % event_class)
        source_dict = self.dispatch_table.get(event_class)
        if source_dict is None: return
        handlers = source_dict.get(source)
        if handlers is None: return
        try:
            handlers.remove(listener.handle_event)
        except: pass
        if handlers == []:
            del source_dict[source]
        if len(source_dict) == 0:   # no one listening for this event
            del self.dispatch_table[event_class]
            # remove the cozmo SDK event handler if there was one
            if event_class.cozmo_evt_type:
                coztype = event_class.cozmo_evt_type
                world = self.robot.world
                gen = self.event_generators[event_class]
                world.remove_event_handler(coztype, gen)
                del self.event_generators[event_class]

    def remove_all_listener_entries(self, listener):
        for event_class, source in self.listener_registry.get(listener,[]):
            self.remove_listener(listener, event_class, source)
        try:
            del self.listener_registry[listener]
        except: pass

    def _get_listeners(self,event):
        source_dict = self.dispatch_table.get(type(event), None)
        if source_dict is None:  # no listeners for this event type
            return []
        matches = source_dict.get(event.source, [])
        if event.source is None:
            none_matches = matches
            matches = []
        else:
            none_matches = source_dict.get(None, [])
        wildcards = []
        for handler in none_matches:
            if self.wildcard_registry.get(handler,False) is True:
                wildcards.append(handler)
            else:
                matches.append(handler)
        # wildcard handlers must come last in the list
        return matches + wildcards

    def post(self,event):
        if not isinstance(event,Event):
            raise TypeError('%s is not an Event' % event)
        listeners = self._get_listeners(event)
        cnt = 0
        for listener in listeners:
            cnt += 1
            if TRACE.trace_level >= TRACE.listener_invocation:
                print('TRACE%d:' % TRACE.listener_invocation, listener.__class__, 'receiving', event)
            self.robot.loop.call_soon(listener,event)
    
    def add_process(self, node):
        self.processes.append(node)

    def delete_process(self, node):
        if node in self.processes:
            self.processes.remove(node)

    POLLING_INTERVAL = 0.1

    def poll_processes(self):
        for node in self.processes:
            if not node.queue.empty():
                event = node.queue.get()
                self.delete_process(node)
                event.source = node
                print(event)
                self.post(event)
        self.robot.loop.call_later(self.POLLING_INTERVAL, self.poll_processes)

#________________ Event Listener ________________

class EventListener:
    """Parent class for both StateNode and Transition."""
    def __init__(self):
        rep = object.__repr__(self)
        self.name = rep[1+rep.rfind(' '):-1]  # name defaults to hex address
        self.running = False
        if not hasattr(self,'polling_interval'):
            self.polling_interval = None
        self.poll_handle = None
        self._robot = robot_for_loading

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
            self.poll_handle = \
                self.robot.loop.call_later(self.polling_interval, self._next_poll)

    def stop(self):
        if not self.running: return
        self.running = False
        if self.poll_handle: self.poll_handle.cancel()
        self.robot.erouter.remove_all_listener_entries(self)

    def handle_event(self, event):
        pass

    def set_polling_interval(self,interval):
        if isinstance(interval, (int,float)):
            self.polling_interval = interval
        else:
            raise TypeError('interval must be a number')

    def _next_poll(self):
        """Called to poll the node and then schedule the next polling interval."""
        if self.running and self.polling_interval:
            self.poll_handle = \
                self.robot.loop.call_later(self.polling_interval, self._next_poll)
        # schedule the next poll first because self.poll may cancel it
        self.poll()

    def poll(self):
        """Dummy polling function in case sublass neglects to supply one."""
        if TRACE.trace_level >= TRACE.polling:
            print('TRACE%d: polling' % TRACE.polling, self)
        print('%s has no poll() method' % self)
