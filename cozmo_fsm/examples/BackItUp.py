"""
  This FSM demo causes Cozmo to back up by 100 mm while simultaneously
  beeping. When he's done backing up, he says "Safety first".
  The demo illustrates the use of fork/join to launch parallel actions and
  synchronize them again. The fork is performed by the NullTrans with
  two destinations, while the join is performed by the CompletionTrans
  with two sources.
"""

try:
    from cozmo_fsm import *
except:
    raise ImportError("Can't find the cozmo_fsm package. Check your search path.")

def setup_fsm(robot):
    # Begin boilerplate
    set_robot(robot)
    cube1 = robot.world.light_cubes[1]
    cube2 = robot.world.light_cubes[2]
    cube3 = robot.world.light_cubes[3]
    charger = robot.world.charger
    # End boilerplate

    backitup = StateNode()
    backitup.set_name("backitup")

    launcher = StateNode()
    launcher.set_name("launcher")
    launcher.set_parent(backitup)

    speaker = Say("beep",duration_scalar=0.8)
    speaker.set_name("speaker")
    speaker.set_parent(backitup)

    driver = DriveForward(-100,10)
    driver.set_name("driver")
    driver.set_parent(backitup)

    finisher = Say("Safety first!")
    finisher.set_name("finisher")
    finisher.set_parent(backitup)

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

    return backitup
