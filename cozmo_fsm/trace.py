"""
  Constants for defining tracing levels.
"""

class TRACE:
    def __init__(self):
        self._trace_level = 0

    @property
    def trace_level(self): return TRACE._trace_level
    @trace_level.setter
    def trace_level(self,val):
        TRACE._trace_level = val
        
    @property
    def no_tracing(self): return 0
    @property
    def statenode_start(self): return 1
    @property
    def statenode_startstop(self): return 2
    @property
    def transition_fire(self): return 3
    @property
    def transition_startstop(self): return 4
    @property
    def listener_invocation(self): return 5
    @property
    def polling(self): return 6
    @property
    def await_satisfied(self): return 7
    @property
    def event_posted(self): return 8

TRACE = TRACE()
