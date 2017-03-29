import time
import numpy
import cv2

import cozmo

import world_viewer

from .base import StateNode
from .aruco import *
from .particle import *
from .cozmo_kin import *
from .particle_viewer import ParticleViewer
from .speech import SpeechListener, Thesaurus
from . import opengl

class StateMachineProgram(StateNode):
    def __init__(self,
                 kine_class=CozmoKinematics,
                 world_viewer=False,
                 cam_viewer=True,
                 particle_viewer = False,
                 particle_viewer_scale = 1.0,
                 path_viewer = False,
                 annotate_cube = True,
                 aruco=True,
                 arucolibname=cv2.aruco.DICT_4X4_250,
                 particle_filter = True,
                 speech = False,
                 thesaurus = Thesaurus()
                 ):
        super().__init__()
        self.name = self.__class__.__name__.lower()

        self.kine_class = kine_class
        self.windowName = None
        self.world_viewer = world_viewer
        self.cam_viewer = cam_viewer
        self.particle_viewer = particle_viewer
        self.particle_viewer_scale = particle_viewer_scale
        self.annotate_cube = annotate_cube
        self.aruco = aruco
        if self.aruco:
            self.robot.world.aruco = Aruco(arucolibname)
        self.particle_filter = particle_filter
        self.speech = speech
        self.thesaurus = thesaurus

    def start(self):
        self.robot.loop.create_task(self.robot.world.delete_all_custom_objects())
        # Set up kinematics
        self.robot.kine = self.kine_class(self.robot)
        self.set_polling_interval(0.050)
        # Create a particle filter
        if self.particle_filter:
            if self.particle_filter is True:
                self.particle_filter = SLAMParticleFilter(self.robot)
            pf = self.particle_filter
            pf.primed = False  # haven't processed a camera image yet
            self.robot.world.particle_filter = pf
        else:
            self.robot.world.particle_filter = None

        # Launch viewers
        if self.world_viewer:
            world_viewer.viewer(self.robot)

        if self.cam_viewer:
            self.windowName = self.name
            cv2.namedWindow(self.windowName)
            cv2.startWindowThread()
            # Display a dummy image to prevent glibc complaints when a camera
            # image doesn't arrive quickly enough after the window opens.
            dummy = numpy.array([[0]])
            cv2.imshow(self.windowName,dummy)
        else:
            self.windowName = None

        if self.particle_viewer:
            if self.particle_viewer is True:
                self.particle_viewer = \
                    ParticleViewer(self.robot, scale=self.particle_viewer_scale)
            self.particle_viewer.start_thread()

        # Request camera image stream
        self.robot.camera.image_stream_enabled = True
        self.robot.world.add_event_handler(cozmo.world.EvtNewCameraImage,
                                           self.process_image)

        # Start speech recognition
        if self.speech:
            self.speech_listener = SpeechListener(self.robot,self.thesaurus)
            self.speech_listener.start()

        # Call parent's start() to launch the state machine
        super().start()

        if opengl.INIT_DONE:
            opengl.launch_main_loop()

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

    def user_image(self,image,gray): pass

    def user_annotate(self,image):
        return image

    def process_image(self,event,**kwargs):
        curim = numpy.array(event.image.raw_image).astype(numpy.uint8) #cozmo-raw image
        gray = cv2.cvtColor(curim,cv2.COLOR_BGR2GRAY)

        # Aruco image processing
        if self.aruco:
            self.robot.world.aruco.process_image(gray)
        # Other image processors can run here if the user supplies them.
        self.user_image(curim,gray)
        # Done with image processing

        # Annotate and display image if stream enabled.
        if self.windowName is not None:
            scale_factor = 2
            # Cozmo's annotations
            self.robot.img = event.image
            if self.annotate_cube:
                coz_ann = event.image.annotate_image(scale=scale_factor)
                annotated_im = numpy.array(coz_ann).astype(numpy.uint8)
            else:
                im = numpy.array(event.image.raw_image).astype(numpy.uint8)
                shape = im.shape
                dsize = (2*shape[1], 2*shape[0])
                annotated_im = cv2.resize(im, dsize)
            # Aruco annotation
            if self.aruco and \
                   len(self.robot.world.aruco.seen_marker_ids) > 0:
                annotated_im = self.robot.world.aruco.annotate(annotated_im,scale_factor)
            # Other annotators can run here if the user supplies them.
            annotated_im = self.user_annotate(annotated_im)
            # Done with annotation
            cv2.imshow(self.windowName,annotated_im)
        pf = self.robot.world.particle_filter
        if pf and not pf.primed:
            pf.look_for_new_landmarks()
