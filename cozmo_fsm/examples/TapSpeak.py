"""
  This FSM demo shows Cozmo responding to cube tap events. The
  TapTrans transition is used to set up an event listener for taps.
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

    tapspeak = StateNode()
    tapspeak.set_name("tapspeak")

    intro = Say("Tap cube 1")
    intro.set_name("intro")
    intro.set_parent(tapspeak)

    wait = StateNode()
    wait.set_name("wait")
    wait.set_parent(tapspeak)

    speak = Say("tap",duration_scalar=0.8,voice_pitch=1)
    speak.set_name("speak")
    speak.set_parent(tapspeak)

    t1 = CompletionTrans()
    t1.set_name('t1')
    t1.add_source(intro)
    t1.add_destination(wait)

    t2 = TapTrans(cube1)
    t2.set_name("t2")
    t2.add_source(wait)
    t2.add_destination(speak)

    t3 = CompletionTrans()
    t3.set_name("t3")
    t3.add_source(speak)
    t3.add_destination(wait)

    return tapspeak
