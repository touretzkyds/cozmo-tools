try:
    from cozmo_fsm import *
except:
    raise ImportError("Can't find the cozmo_fsm package. Check your search path.")

class InitFaceCounts(StateNode):
    def start(self,event=None):
        super().start(event)
        Peekaboo.amountOfFacesNotFound = 0
        Peekaboo.amountOfFacesFound = 0

class FaceChecker(StateNode):
    def start(self,event=None):
        super().start(event)
        numberOfFaces = self.robot.world.visible_face_count()
        if numberOfFaces == 0:
            Peekaboo.amountOfFacesNotFound += 1
        else:
            Peekaboo.amountOfFacesFound += 1

class GreetIfPlayer(StateNode):
    def start(self,event=None):
        super().start(event)
        numberOfFaces = self.robot.world.visible_face_count()
        if numberOfFaces > 0 and Peekaboo.amountOfFacesFound > 3:
            post_success();

class CheckPlayerHidden(StateNode):
    def start(self,event=None):
        super().start(event)
        numberOfFaces = self.robot.world.visible_face_count()
        if numberOfFaces == 0 and Peekaboo.amountOfFacesNotFound > 2:
            post_success();
        

class Peekaboo(StateNode):
    face_delay = 0.2
