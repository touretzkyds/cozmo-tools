"""
  The Nested demo shows the use of nested state machines. We define a
  new node class DingDong that has a three-node state machine inside
  it. We then define the main class, Nested, whose state machine
  contains two instances of DingDong.  DingDong uses a ParentCompletes
  node to cause DinDong to post a completion event, which allows
  Nested's first DingDong instance 'dd1' to move on to the next state,
  which is 'bridge'. (The 'dd2' instance also posts a completion
  event, but nothing is listening for it.)

  Behavior: Cozmo says 'ding', then 'dong', then 'once again' (that's
  the bridge), then 'ding', and then 'dong'.

  Shorthand for DingDong:
    ding: Say('ding') =C=> dong: Say('dong') =C=> ParentCompletes()

  Shorthand for Nested:
    dd1: DingDong() =C=> bridge: Say('once again') =C=> dd2: DingDong()
"""

from cozmo_fsm import *

class DingDong(StateNode):
    def setup(self):
        ding = Say("ding")
        ding.set_name('ding')
        ding.set_parent(self)

        dong = Say("dong")
        dong.set_name('dong')
        dong.set_parent(self)

        done = ParentCompletes()
        done.set_name('done')
        done.set_parent(self)

        ct1 = CompletionTrans()
        ct1.set_name('ct1')
        ct1.add_source(ding)
        ct1.add_destination(dong)

        ct2 = CompletionTrans()
        ct2.set_name('ct2')
        ct2.add_source(dong)
        ct2.add_destination(done)

        return self


class Nested(StateNode):
    def setup(self):
        self.name = 'Nested'

        dd1 = DingDong()
        dd1.set_name('dd1')
        dd1.set_parent(self)

        bridge = Say("once again")
        bridge.set_name('bridge')
        bridge.set_parent(self)

        dd2 = DingDong()
        dd2.set_name('dd2')
        dd2.set_parent(self)

        ct1 = CompletionTrans()
        ct1.set_name('ct1')
        ct1.add_source(dd1)
        ct1.add_destination(bridge)

        ct2 = CompletionTrans()
        ct2.set_name('ct2')
        ct2.add_source(bridge)
        ct2.add_destination(dd2)

        return self
