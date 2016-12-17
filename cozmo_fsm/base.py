"""
    Base classes StateNode for nodes.py and Transition for
    transitions.py are placed here due to circular dependencies.
    Their parent class EventListener is imported from erouter.py.

"""

import cozmo

from .trace import TRACE
from .evbase import EventListener
from .events import CompletionEvent, FailureEvent

class StateNode(EventListener):
    """Base class for state nodes; does nothing."""
    def __init__(self):
        super().__init__()
        self.parent = None
        self.children = []
        self.transitions = []
        self.setup()

    # Cache 'robot' in the instance because we could have two state
    # machine instances controlling different robots.
    @property
    def robot(self):
        """if not self._robot:
            if self.parent:
                self._robot = self.parent.robot  # recursive call
            else:
                self._robot = erouter.robot_for_loading
            if not isinstance(self._robot, cozmo.robot.Robot):
                raise ValueError('%s _robot attribute is %s, not a cozmo.robot.Robot' %
                                 (self, self._robot))"""
        return self._robot

    def start(self,event=None):
        if self.running: return
        if TRACE.trace_level >= TRACE.statenode_start:
            print('TRACE%d:' % TRACE.statenode_start, self, 'starting')
        super().start()
        # Start transitions before children, because children
        # may post an event that we're listening for (like completion).
        for t in self.transitions:
            # print(self,'starting',t)
            t.start(event)
        if self.children:
            self.children[0].start()

    def stop(self):
        if not self.running: return
        if TRACE.trace_level >= TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop, self, 'stopping')
        super().stop()
        # Stop children before transitions, because a child's stop()
        # method could post an event we want to handle.
        for c in self.children: c.stop()
        for t in self.transitions: t.stop()

    def add_transition(self, trans):
        if not isinstance(trans, Transition):
            raise TypeError('%s is not a Transition' % trans)
        self.transitions.append(trans)

    def set_parent(self, parent):
        if not isinstance(parent, StateNode):
            raise TypeError('%s is not a StateNode' % parent)
        if self.parent:
            raise Exception('parent already set')
        self.parent = parent
        parent.children.append(self)
        return self

    def post_completion(self):
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop, self, 'posting completion')
        self.robot.erouter.post(CompletionEvent(self))

    def post_failure(self,details=None):
        if TRACE.trace_level > TRACE.statenode_startstop:
            print('TRACE%d:' % TRACE.statenode_startstop,
                  self, 'posting failure', details)
        self.robot.erouter.post(FailureEvent(self,details))

    def now(self):
        """Use now() to execute this node from the command line instead of as part of a state machine."""
        if not self.robot:
            raise ValueError('Node %s has no robot designated.' % self)
        self.robot.loop.call_soon(self.start)


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

    def add_source(self, node):
        if not isinstance(node, StateNode):
            raise TypeError('%s is not a StateNode' % node)
        self._sibling_check(node)
        node.add_transition(self)
        self.sources.append(node)

    def add_destination(self, node):
        if not isinstance(node, StateNode):
            raise TypeError('%s is not a StateNode' % node)
        self._sibling_check(node)
        self.destinations.append(node)

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
            if TRACE.trace_level >= TRACE.task_cancel:
                print('TRACE%d:' % TRACE.task_cancel, self.handle, 'cancelled')
            self.handle.cancel()
        super().stop()

    def fire(self,event=None):
        if TRACE.trace_level >= TRACE.transition_fire:
            if event == None:
                evt_desc = ''
            else:
                evt_desc = ' on %s' % event
            print('TRACE%d:' % TRACE.transition_fire, self, 'firing'+evt_desc)
        for src in self.sources:
            src.stop()
        self.stop()
        action_cancel_delay = 0.001  # wait for source node action cancellations to take effect
        self.robot.loop.call_later(action_cancel_delay, self.fire2,event)

    def fire2(self,event):
        for dest in self.destinations:
            # print(self,'fire2 is starting',dest)
            dest.start(event)

    default_value_delay = 0.1  # delay before wildcard match will fire

