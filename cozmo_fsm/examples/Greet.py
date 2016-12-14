"""
  The Greet demo illustrates the use of CompletionTrans and TimerTrans
  transitions.

  Behavior: Cozmo starts out by saying 'Greetings, human!'. After his
  speech has completed, he waits 5 seconds, then says 'Bye-bye now'.

  Shorthand version (using the chaining-style notation):
    say: Say('Greetings, human!') =C=>
      wait: StateNode() =T(5)=>
        say2: Say('Bye-bye now.')
"""

try:
    from cozmo_fsm import *
except:
    raise ImportError("Can't find the cozmo_fsm package. Check your search path.")

class Greet(StateNode):
    def setup(self):
        self.set_name("greet")

        say1 = Say("Greetings, human!")
        say1.set_name('say1')
        say1.set_parent(self)

        wait = StateNode()
        wait.set_name("wait")
        wait.set_parent(self)

        say2 = Say("Bye-bye now.")
        say2.set_name("say2")
        say2.set_parent(self)

        ct1 = CompletionTrans()
        ct1.set_name("ct1")
        ct1.add_source(say1)
        ct1.add_destination(wait)

        timer1 = TimerTrans(5)
        timer1.set_name("timer1")
        timer1.add_source(wait)
        timer1.add_destination(say2)

        return self
