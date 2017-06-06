import time
import functools
import numpy
import cv2

import cozmo

from .base import StateNode
from .aruco import *
from .particle import *
from .cozmo_kin import *
from .particle_viewer import ParticleViewer
from .worldmap import WorldMap
from .rrt import RRT
from .path_viewer import PathViewer
from .worldmap_viewer import WorldMapViewer
from .speech import SpeechListener, Thesaurus
from . import opengl
from . import custom_objs

class StateMachineProgram(StateNode):
    def __init__(self,
                 kine_class=CozmoKinematics,
                 cam_viewer=True,
                 particle_viewer = False,
                 particle_viewer_scale = 1.0,
                 annotate_cube = True,
                 aruco=True,
                 arucolibname=cv2.aruco.DICT_4X4_250,
                 particle_filter = True,
                 world_map = None,
                 worldmap_viewer=False,
                 rrt = None,
                 path_viewer = False,
                 speech = False,
                 speech_debug = False,
                 thesaurus = Thesaurus()
                 ):
        super().__init__()
        self.name = self.__class__.__name__.lower()

        self.robot.loop.create_task(custom_objs.declare_objects(self.robot))
        time.sleep(0.5)  # need time for custom objects to be transmitted
        
        self.kine_class = kine_class
        self.windowName = None
        self.cam_viewer = cam_viewer
        self.particle_viewer = particle_viewer
        self.particle_viewer_scale = particle_viewer_scale
        self.annotate_cube = annotate_cube
        self.aruco = aruco
        if self.aruco:
            self.robot.world.aruco = Aruco(arucolibname)
        self.particle_filter = particle_filter
        self.world_map = world_map
        self.worldmap_viewer = worldmap_viewer
        self.rrt = rrt
        self.path_viewer = path_viewer
        self.speech = speech
        self.speech_debug = speech_debug
        self.thesaurus = thesaurus

    async def daco(self):
        await self.robot.world.undefine_all_custom_marker_objects()
        await self.robot.world.delete_all_custom_objects()

    def start(self):
        self.robot.loop.create_task(self.daco())

        # Create a particle filter
        if not isinstance(self.particle_filter,ParticleFilter):
            self.particle_filter = SLAMParticleFilter(self.robot)
        pf = self.particle_filter
        pf.primed = False  # haven't processed a camera image yet
        self.robot.world.particle_filter = pf

        # Set up kinematics
        self.robot.kine = self.kine_class(self.robot)
        self.set_polling_interval(0.025)  # for kine and motion model update
        # World map and path planner
        self.robot.world.world_map = \
                self.world_map or WorldMap(self.robot)
        self.robot.world.rrt = self.rrt or RRT(self.robot)

        # Launch viewers
        opengl.init()
        opengl.launch_event_loop()

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
            self.particle_viewer.start()
        self.robot.world.particle_viewer = self.particle_viewer

        if self.path_viewer:
            if self.path_viewer is True:
                self.path_viewer = PathViewer(self.robot.world.rrt)
            self.path_viewer.start()
        self.robot.world.path_viewer = self.path_viewer

        if self.worldmap_viewer:
            if self.worldmap_viewer is True:
                self.worldmap_viewer = WorldMapViewer(self.robot)
            self.worldmap_viewer.start()
        self.robot.world.worldmap_viewer = self.worldmap_viewer

        # Request camera image and object streams
        self.robot.camera.image_stream_enabled = True
        self.robot.world.add_event_handler(cozmo.world.EvtNewCameraImage,
                                           self.process_image)
        self.robot.world.add_event_handler(
            cozmo.objects.EvtObjectObserved,
            self.robot.world.world_map.handle_object_observed)

        # Start speech recognition if requested
        if self.speech:
            self.speech_listener = SpeechListener(self.robot,self.thesaurus,debug=self.speech_debug)
            self.speech_listener.start()

        # Call parent's start() to launch the state machine.
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

        # Finally update the world map
        self.robot.world.world_map.update_map()
