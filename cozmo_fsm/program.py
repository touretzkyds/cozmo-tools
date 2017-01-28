import cv2

from .base import StateNode
from .aruco import *

class StateMachineProgram(StateNode):
    def __init__(self,
                 viewer=True,
                 aruco=True,
                 arucolibname=cv2.aruco.DICT_4X4_250):
        super().__init__()

        self.viewer = viewer
        self.aruco = aruco
        if self.aruco:
            world.aruco = Aruco(arucolibname)

    def start(self):
        # Launch viewer
        if self.viewer:
            self.windowName = "streaming"
            cv2.namedWindow(self.windowName)
            cv2.startWindowThread() #call cv2.destroyWindow(self.windowName) to close
        else:
            self.windowName = None

        # Request camera image stream
        if self.viewer or self.aruco:
            robot.camera.image_stream_enabled = True
            while(robot.world.latest_image is None):
                time.sleep(0.2)
            robot.world.add_event_handler(cozmo.world.EvtNewCameraImage,
                                          self.process_image)

        # Call parent's start() to launch the state machine
        super().start()

    def stop(self):
        super().stop()
        robot.world.remove_event_handler(cozmo.world.EvtNewCameraImage,self.process_image)
        if self.windowName is not None:
            cv2.destroyWindow(self.windowName)

    def process_image(self,event,**kwargs):
        curim = numpy.array(event.image.raw_image).astype(numpy.uint8) #cozmo-raw image
        gray = cv2.cvtColor(curim,cv2.COLOR_BGR2GRAY)

        # Aruco image processing
        if self.aruco:
            world.aruco.process_image(gray)
        # Other image processors can run here if the user supplies them.

        # Annotate and display image if stream enabled.
        if self.windowName is not None:
            annotated_im = numpy.array(event.image.annotate_image()).astype(numpy.uint8) #cozmo-annotated image
            # Aruco annotation
            if world.aruco.ids is not None:
                annotated_im = world.aruco.annotate(annotated_im)
            # Other annotators can run here if the user supplies them.
            cv2.imshow(self.windowName,annotated_im)
