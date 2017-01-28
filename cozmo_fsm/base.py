"""
    Base classes StateNode for nodes.py and Transition for
    transitions.py are placed here due to circular dependencies.
    Their parent class EventListener is imported from evbase.py.

"""

import cozmo

from .trace import TRACE
from .evbase import EventListener
from .events import CompletionEvent, SuccessEvent, FailureEvent, DataEvent

class StateNode(EventListener):
    """Base class for state nodes; does nothing."""
    def __init__(self):
        super().__init__()
        self.parent = None
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
            t.start(event)
        if self.start_node:
            self.start_node.start()

    def stop(self):
        if not self.running: return
        if TRACE.trace_level >= TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop, self, 'stopping')
        super().stop()
        # Stop children before transitions, because a child's stop()
        # method could post an event we want to handle.
        for c in self.children.values(): c.stop()
        for t in self.transitions: t.stop()

    def add_transition(self, trans):
        if not isinstance(trans, Transition):
            raise TypeError('%s is not a Transition' % trans)
        self.transitions.append(trans)

    def set_parent(self, parent):
        if not isinstance(parent, StateNode):
            raise TypeError('%s is not a StateNode' % parent)
        try:
            if self.parent:
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
        """if not self._robot:
            if len(self.sources) > 0:
                self._robot = self.sources.robot
                return self._robot
            else:
                raise Exception('Transition %s has no sources.' % self)"""

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

    def start(self,event):
        if self.running: return
        if TRACE.trace_level >= TRACE.transition_startstop:
            print('TRACE%d:' % TRACE.transition_startstop, self, 'starting')
        super().start()
        self.handle = None

    def stop(self):
        if not self.running: return
        # don't stop if we still have a live source
        for src in self.sources:
            if src.running:
                # print(self,'saved from stopping by',src)
                return
        if TRACE.trace_level >= TRACE.transition_startstop:
            print('TRACE%d:' % TRACE.transition_startstop, self, 'stopping')
        if self.handle:
            if True or TRACE.trace_level >= TRACE.task_cancel:
                print('TRACE%d:' % TRACE.task_cancel, self.handle, 'cancelled')
            self.handle.cancel()
        super().stop()

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
        self.robot.loop.call_later(action_cancel_delay, self.fire2,event)

    def fire2(self,event):
        for dest in self.destinations:
            dest.start(event)

    default_value_delay = 0.1  # delay before wildcard match will fire

