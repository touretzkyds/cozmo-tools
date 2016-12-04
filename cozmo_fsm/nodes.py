from .events import *
from .base import *


class Forward(StateNode):
    pass


class Turn(StateNode):
    pass


class Say(StateNode):
    """Speaks some text, then posts a completion event."""
    def __init__(self, _name, _text):
        super().__init__(_name)
        text = _text

    def start(self):
        super().start()
        print("Speaking: '",text,"'",sep='')
        robot.say_text(text).wait_for_completion()
        self.post_completion()

