"""
  The TapSpeak demo shows Cozmo responding to cube tap events. A
  TapTrans transition is used to set up a handler for taps. The
  example also illustrates how the TapTrans transition does wildcard
  matching if not given an argument. By passing a cube as an argument
  to the TapTrans constructor can use it to look for taps on a
  specific cube.

  Behavior: Cozmo starts out by saying 'Tap a cube'. Then, every time
  a cube is tapped, Cozmo says the cube name and goes back to
  listening for more tap events.

  Shorthand version:
    intro: Say('Tap a cube.') =C=> wait
    wait: StateNode() =Tap()=> speak
    speak: SayCube() =C=> wait
"""

from cozmo_fsm import *

class SayCube(Say):
    def start(self, event=None, \
              cube_names = ['paperclip', 'anglepoise lamp', 'deli slicer']):
        cube_number = next(k for k,v in self.robot.world.light_cubes.items() \
                               if v == event.source)
        self.text = cube_names[cube_number-1]
        super().start(event)

class TapSpeak(StateNode):
    def setup(self):
        self.set_name("tapspeak")

        intro = Say("Tap a cube")
        intro.set_name("intro")
        intro.set_parent(self)

        wait = StateNode()
        wait.set_name("wait")
        wait.set_parent(self)

        speak = SayCube()
        speak.set_name("speak")
        speak.set_parent(self)

        t1 = CompletionTrans()
        t1.set_name("t1")
        t1.add_source(intro)
        t1.add_destination(wait)

        t2 = TapTrans()
        t2.set_name("t2")
        t2.add_source(wait)
        t2.add_destination(speak)

        t3 = CompletionTrans()
        t3.set_name("t3")
        t3.add_source(speak)
        t3.add_destination(wait)

        return self
