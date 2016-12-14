"""
  The TapSpeak demo shows Cozmo responding to cube tap events. A
  TapTrans transition is used to set up an event listener for taps on
  cube 1. Cozmo starts out by saying 'Tap cube 1'. Then, every time
  the cube is tapped, Cozmo says 'Tap!' and goes back to listening for
  more tap events.

  Shorthand version:
    intro: Say('Tap cube 1') =C=> wait
    wait: StateNode() =Tap(self.cube1)=> speak
    speak: Say('tap',duration_scalar=0.8,voice_pitch=1) =C=> wait
"""

try:
    from cozmo_fsm import *
except:
    raise ImportError("Can't find the cozmo_fsm package. Check your search path.")

class TapSpeak(EventListener):
    def setup(self):
        self.set_name("tapspeak")

        intro = Say("Tap cube 1")
        intro.set_name("intro")
        intro.set_parent(self)

        wait = StateNode()
        wait.set_name("wait")
        wait.set_parent(self)

        speak = Say("tap",duration_scalar=0.8,voice_pitch=1)
        speak.set_name("speak")
        speak.set_parent(self)

        t1 = CompletionTrans()
        t1.set_name("t1")
        t1.add_source(intro)
        t1.add_destination(wait)

        t2 = TapTrans(self.cube1)   # cube1 is a property inherited from EventListener
        t2.set_name("t2")
        t2.add_source(wait)
        t2.add_destination(speak)

        t3 = CompletionTrans()
        t3.set_name("t3")
        t3.add_source(speak)
        t3.add_destination(wait)

        return self
