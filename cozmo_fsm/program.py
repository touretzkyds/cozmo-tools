import time
import numpy
import cv2

import cozmo

from .base import StateNode
from .aruco import *
from .particle import *
from .cozmo_kin import *

class StateMachineProgram(StateNode):
    def __init__(self,
                 kine_class=CozmoKinematics,
                 viewer=True,
                 aruco=True,
                 arucolibname=cv2.aruco.DICT_4X4_250,
                 particle_filter = True
                 ):
        super().__init__()

        self.kine_class = kine_class
        self.windowName = None
        self.viewer = viewer
        self.aruco = aruco
        if self.aruco:
            self.robot.world.aruco = Aruco(arucolibname)
        self.particle_filter = particle_filter

    def start(self):
        # Set up kinematics
        self.robot.kine = self.kine_class(self.robot)
        self.set_polling_interval(0.050)
        # Create a particle filter
        if self.particle_filter:
            if self.particle_filter is True:
                self.particle_filter = ParticleFilter(self.robot)
            self.robot.world.particle_filter = self.particle_filter
        else:
            self.robot.world.particle_filter = None

        # Launch viewer
        if self.viewer:
            self.windowName = self.name
            cv2.namedWindow(self.windowName)
            cv2.startWindowThread()
            # Display a dummy image to prevent glibc complaints when a camera
            # image doesn't arrive quickly enough after the window opens.
            dummy = numpy.array([[0]])
            cv2.imshow(self.windowName,dummy)
        else:
            self.windowName = None

        # Request camera image stream
        if self.viewer or self.aruco:
            self.robot.camera.image_stream_enabled = True
            self.robot.world.add_event_handler(cozmo.world.EvtNewCameraImage,
                                               self.process_image)

        # Call parent's start() to launch the state machine
        super().start()

    def stop(self):
        super().stop()
        try:
            self.robot.world.remove_event_handler(cozmo.world.EvtNewCameraImage,
                                                  self.process_image)
        except: pass
        if self.windowName is not None:
            cv2.destroyWindow(self.windowName)

    def poll(self):
        self.robot.kine.get_pose()
        if self.robot.world.particle_filter:
            self.robot.world.particle_filter.move()

    def process_image(self,event,**kwargs):
        curim = numpy.array(event.image.raw_image).astype(numpy.uint8) #cozmo-raw image
        gray = cv2.cvtColor(curim,cv2.COLOR_BGR2GRAY)

        # Aruco image processing
        if self.aruco:
            self.robot.world.aruco.process_image(gray)
        # Other image processors can run here if the user supplies them.
        #  ...
        # Done with image processing

        # Annotate and display image if stream enabled.
        if self.windowName is not None:
            scale_factor = 2
            # Cozmo's anootations
            self.robot.img = event.image
            coz_ann = event.image.annotate_image(scale=scale_factor)
            annotated_im = numpy.array(coz_ann).astype(numpy.uint8)
            # Aruco annotation
            if self.robot.world.aruco.seenMarkers is not None:
                annotated_im = self.robot.world.aruco.annotate(annotated_im,scale_factor)
            # Other annotators can run here if the user supplies them.
            #  ...
            # Done with nnotation
            cv2.imshow(self.windowName,annotated_im)
