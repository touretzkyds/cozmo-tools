try:
    from cozmo_fsm import *
except:
    raise ImportError("Can't find the cozmo_fsm package. Check your search path.")

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
timer2.add_destination(say2)

def run(robot):
    set_robot(robot)
    greet.start()

