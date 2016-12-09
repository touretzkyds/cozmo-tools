"""
    Base classes StateNode for nodes.py and Transition for
    transitions.py are placed here due to circular dependencies.

"""

from .events import *


class StateNode(EventListener):
    """Base class for state nodes; does nothing."""
    def __init__(self):
        super().__init__()
        self.transitions = []
        self.parent = None
        self.children = []

    def start(self,event=None):
        if self.running: return
        super().start()
        # Start transitions before children, because children
        # may post an event that we're listening for (like completion).
        for t in self.transitions: t.start()
        if self.children:
            self.children[0].start()

    def stop(self):
        if not self.running: return
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

    def post_completion(self):
        erouter.post(CompletionEvent(self))


class Transition(EventListener):
    """Base class for transitions: does nothing."""
    def __init__(self):
        super().__init__()
        self.sources = []
        self.destinations = []
        self.handle = None

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

    def start(self):
        if self.running: return
        super().start()
        self.handle = None

    def stop(self):
        if not self.running: return
        if self.handle is not None:
            print(self.name,'cancelling',self.handle)
            self.handle.cancel()
        super().stop()

    def fire(self,event=None):
        self.stop()
        for src in self.sources: src.stop()
        for dest in self.destinations: dest.start(event=event)
