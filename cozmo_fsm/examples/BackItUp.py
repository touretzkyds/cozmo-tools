"""
  The BackItUp demo illustrates the use of fork/join to launch
  parallel actions and synchronize them again. The fork is performed
  by the NullTrans transition with two destinations, while the join is
  performed by the CompletionTrans transition with two sources.

  The demo causes Cozmo to back up by 100 mm while simultaneously
  beeping. It uses DriveForward instead of Forward to avoid conflict
  with the Say action. When he's done backing up, he stops beeping and
  says"Safety first".

  Shorthand version:
    launcher: StateNode() =N=> {speaker, driver}
    speaker: Say('beep',duration_scalar=0.8)
    driver: DriveForward(-100,10)
    {speaker,driver} =C=> finisher: Say('Safety first!')
"""

try:
    from cozmo_fsm import *
except ImportError:
    raise ImportError("Can't find the cozmo_fsm package. Check your search path.")

class BackItUp(StateNode):
    def setup(self):
        self.set_name('BackItUp')

        launcher = StateNode()
        launcher.set_name("launcher")
        launcher.set_parent(self)

        speaker = Say("beep",duration_scalar=0.8)
        speaker.set_name("speaker")
        speaker.set_parent(self)

        driver = DriveForward(-100,10)
        driver.set_name("driver")
        driver.set_parent(self)

        finisher = Say("Safety first!")
        finisher.set_name("finisher")
        finisher.set_parent(self)

        nt1 = NullTrans()
        nt1.set_name("nt1")
        nt1.add_source(launcher)
        nt1.add_destination(speaker)
        nt1.add_destination(driver)

        ct1 = CompletionTrans()
        ct1.set_name("ct1")
        ct1.add_source(speaker)
        ct1.add_destination(speaker)

        ct2 = CompletionTrans()
        ct2.set_name("ct2")
        ct2.add_source(speaker)
        ct2.add_source(driver)
        ct2.add_destination(finisher)

        return self
