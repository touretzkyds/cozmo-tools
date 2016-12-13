"""
  This FSM demo makes Cozmo say 'Greetings, human!'  After he's
  finished speaking, he waits 5 seconds, and then say 'Bye-bye now'.
  This demonstrates the CompletionTrans and TimerTrans transitions.
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

    greet = StateNode()
    greet.set_name("greet")

    say1 = Say("Greetings, human!")
    say1.set_name('say1')
    say1.set_parent(greet)

    wait = StateNode()
    wait.set_name('wait')
    wait.set_parent(greet)

    say2 = Say("Bye-bye now.")
    say2.set_name('say2')
    say2.set_parent(greet)

    ct1 = CompletionTrans()
    ct1.set_name('ct1')
    ct1.add_source(say1)
    ct1.add_destination(wait)

    timer1 = TimerTrans(5)
    timer1.set_name('timer1')
    timer1.add_source(wait)
    timer1.add_destination(say2)

    return greet
