"""
    Base classes StateNode for nodes.py and Transition for
    transitions.py are placed here due to circular dependencies.
    Their parent class EventListener is imported from evbase.py.

"""

import cozmo

from .trace import TRACE
from .evbase import Event, EventListener
from .events import CompletionEvent, SuccessEvent, FailureEvent, DataEvent

class StateNode(EventListener):
    """Base class for state nodes; does nothing."""
    def __init__(self):
        super().__init__()
        self.parent = b'No parent defined yet.'
        self.children = {}
        self.transitions = []
        self.start_node = None
        self.setup()
        self.setup2()

    # Cache 'robot' in the instance because we could have two state
    # machine instances controlling different robots.
    @property
    def robot(self):
        return self._robot

    def setup(self):
        """Redefine this to set up a child state machine."""
        pass

    def setup2(self):
        """Redefine this if post-setup processing is required."""
        pass

    def start(self,event=None):
        if self.running: return
        if TRACE.trace_level >= TRACE.statenode_start:
            print('TRACE%d:' % TRACE.statenode_start, self, 'starting')
        super().start()
        # Start transitions before children, because children
        # may post an event that we're listening for (such as completion).
        for t in self.transitions:
            t.start()
        if self.start_node:
            if TRACE.trace_level >= TRACE.statenode_start:
                print('TRACE%d:' % TRACE.statenode_start, self, 'starting child', self.start_node)
            self.start_node.start()

    def stop(self):
        # If this node was stopped by an outgoing transition firing,
        # and then its parent tries to stop it, we need to cancel the
        # pending fire2 call.
        if self.running:
            if TRACE.trace_level >= TRACE.statenode_startstop:
                print('TRACE%d:' % TRACE.statenode_startstop, self, 'stopping')
            super().stop()
            self.stop_children()
        # Stop transitions even if we're not running, because a firing
        # transition could have stopped us and left a fire2 pending.
        for t in self.transitions:
            t.stop()

    def stop_children(self):
        if self.children == {}:
            return
        if TRACE.trace_level >= TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop, self, 'is stopping its children')
        for child in self.children.values():
            child.stop()

    def add_transition(self, trans):
        if not isinstance(trans, Transition):
            raise TypeError('%s is not a Transition' % trans)
        self.transitions.append(trans)

    def set_parent(self, parent):
        if not isinstance(parent, StateNode):
            raise TypeError('%s is not a StateNode' % parent)
        try:
            if isinstance(self.parent, StateNode):
                raise Exception('parent already set')
        except AttributeError:
            raise Exception("It appears %s's __init__ method did not call super().__init__"
                            % self.__class__.__name__)
        self.parent = parent
        parent.children[self.name] = self
        # First-declared child is the default start node.
        if not parent.start_node:
            parent.start_node = self
        return self

    def post_event(self,event):
        if not isinstance(event,Event):
            raise ValuError('post_event given a non-Event argument:',event)
        if event.source is None:
            event.source = self
        if TRACE.trace_level >= TRACE.event_posted:
            print('TRACE%d:' % TRACE.event_posted, self, 'posting event',event)
        self.robot.erouter.post(event)

    def post_completion(self):
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop, self, 'posting completion')
        self.robot.erouter.post(CompletionEvent(self))

    def post_success(self,details=None):
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  self, 'posting success, details=%s' % details)
        self.robot.erouter.post(SuccessEvent(self,details))

    def post_failure(self,details=None):
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  self, 'posting failure, details=%s' % details)
        self.robot.erouter.post(FailureEvent(self,details))

    def post_data(self,value):
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  self, 'posting data', value)
        self.robot.erouter.post(DataEvent(self,value))

    def now(self):
        """Use now() to execute this node from the command line instead of as part of a state machine."""
        if not self.robot:
            raise ValueError('Node %s has no robot designated.' % self)
        # 'program' is inserted into this module by __init__ to avoid circular importing
        program.running_fsm.children = dict()
        program.running_fsm.children[self.name] = self
        self.robot.loop.call_soon(self.start)
        return self


class Transition(EventListener):
    """Base class for transitions: does nothing."""
    def __init__(self):
        super().__init__()
        self.sources = []
        self.destinations = []
        self.handle = None

    def __repr__(self):
        srcs = ','.join(node.name for node in self.sources)
        dests = ','.join(node.name for node in self.destinations)
        return '<%s %s: %s=>%s >' % \
            (self.__class__.__name__, self.name, srcs, dests)

    @property
    def robot(self):
        return self._robot

    def _sibling_check(self,node):
        for sibling in self.sources + self.destinations:
            if sibling.parent is not node.parent:
                raise ValueError("All source/destination nodes must have the same parent.")

    def add_sources(self, *nodes):
        for node in nodes:
            if not isinstance(node, StateNode):
                raise TypeError('%s is not a StateNode' % node)
            self._sibling_check(node)
            node.add_transition(self)
            self.sources.append(node)
        return self

    def add_destinations(self, *nodes):
        for node in nodes:
            if not isinstance(node, StateNode):
                raise TypeError('%s is not a StateNode' % node)
            self._sibling_check(node)
            self.destinations.append(node)
        return self

    def start(self):
        if self.running: return
        self.handle = None
        if TRACE.trace_level >= TRACE.transition_startstop:
            print('TRACE%d:' % TRACE.transition_startstop, self, 'starting')
        super().start()

    def stop(self):
        if self.running:
            # don't stop if we still have a live source
            for src in self.sources:
                if src.running:
                    if TRACE.trace_level >= TRACE.transition_startstop:
                        print('TRACE%d:' % TRACE.transition_startstop,self,'saved from stopping by',src)
                        return
            if TRACE.trace_level >= TRACE.transition_startstop:
                print('TRACE%d:' % TRACE.transition_startstop, self, 'stopping')
            super().stop()
        # stop pending fire2 if fire already stopped this transition
        if self.handle:
            if TRACE.trace_level >= TRACE.task_cancel:
                print('TRACE%d:' % TRACE.task_cancel, self.handle, 'cancelled')
            self.handle.cancel()
            self.handle = None

    def fire(self,event=None):
        """Shut down source nodes and schedule start of destination nodes.
        Lets the stack unwind by returning before destinations are started.
        Delay also gives time for Cozmo action cancellation to take effect."""
        if not self.running: return
        if TRACE.trace_level >= TRACE.transition_fire:
            if event == None:
                evt_desc = ''
            else:
                evt_desc = ' on %s' % event
            print('TRACE%d:' % TRACE.transition_fire, self, 'firing'+evt_desc)
        for src in self.sources:
            src.stop()
        self.stop()
        action_cancel_delay = 0.01  # wait for source node action cancellations to take effect
        self.handle = self.robot.loop.call_later(action_cancel_delay, self.fire2, event)

    def fire2(self,event):
        if not self.handle:
            print('@ @ @ @ @ HANDLE GONE: I SHOULD BE DEAD', self, event)
        for dest in self.destinations:
            if TRACE.trace_level >= TRACE.transition_fire:
                print('TRACE%d: ' % TRACE.transition_fire, self, 'starting', dest)
            dest.start(event)

    default_value_delay = 0.1  # delay before wildcard match will fire
