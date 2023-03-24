import asyncio
import functools
import inspect
import os
import time

import numpy
import numpy as np

try:
    import cv2
    ARUCO_DICT_4x4_100 = cv2.aruco.DICT_4X4_100
except:
    ARUCO_DICT_4x4_100 = None

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
from .evbase import EventRouter
from .base import StateNode
from .aruco import *
from .particle import *
from .cozmo_kin import *
from .particle_viewer import ParticleViewer
from .worldmap import WorldMap
from .rrt import RRT
from .path_viewer import PathViewer
from .worldmap_viewer import WorldMapViewer
from .cam_viewer import CamViewer
from .speech import SpeechListener, Thesaurus
from . import opengl
from . import custom_objs
from .perched import *
from .sharedmap import *
from .cam_viewer import CamViewer

running_fsm = None
charger_warned = False

class StateMachineProgram(StateNode):
    def __init__(self,
                 kine_class = CozmoKinematics,
                 cam_viewer = True,
                 force_annotation = False,   # set to True for annotation even without cam_viewer
                 annotate_sdk = True,        # include SDK's own image annotations
                 annotated_scale_factor = 2, # set to 1 to avoid cost of resizing images
                 viewer_crosshairs = False,  # set to True to draw viewer crosshairs

                 particle_filter = True,
                 landmark_test = SLAMSensorModel.is_solo_aruco_landmark,
                 particle_viewer = False,
                 particle_viewer_scale = 1.0,

                 aruco = True,
                 arucolibname = ARUCO_DICT_4x4_100,
                 aruco_disabled_ids = (17, 37),
                 aruco_marker_size = ARUCO_MARKER_SIZE,

                 perched_cameras = False,

                 world_map = None,
                 worldmap_viewer = False,

                 rrt = None,
                 path_viewer = False,

                 speech = False,
                 speech_debug = False,
                 thesaurus = Thesaurus(),

                 simple_cli_callback = None
                 ):
        super().__init__()
        self.name = self.__class__.__name__.lower()
        self.parent = None
        self.simple_cli_callback = simple_cli_callback

        if not hasattr(self.robot, 'erouter'):
            self.robot.erouter = EventRouter()
            self.robot.erouter.robot = self.robot
            self.robot.erouter.start()
        else:
            self.robot.erouter.clear()

        # Reset custom objects
        cor = self.robot.world.undefine_all_custom_marker_objects()
        if inspect.iscoroutine(cor):
            asyncio.ensure_future(cor)
        self.robot.loop.create_task(custom_objs.declare_objects(self.robot))
        time.sleep(0.25)  # need time for custom objects to be transmitted

        self.kine_class = kine_class

        self.cam_viewer = cam_viewer
        self.viewer = None
        self.annotate_sdk = annotate_sdk
        self.force_annotation = force_annotation
        self.annotated_scale_factor = annotated_scale_factor
        self.viewer_crosshairs = viewer_crosshairs

        self.particle_filter = particle_filter
        self.landmark_test = landmark_test
        self.particle_viewer = particle_viewer
        self.particle_viewer_scale = particle_viewer_scale
        self.picked_up_callback = self.robot_picked_up
        self.put_down_handler = self.robot_put_down

        self.aruco = aruco
        self.aruco_marker_size = aruco_marker_size
        if self.aruco:
            self.robot.world.aruco = \
                Aruco(self.robot, arucolibname, aruco_marker_size, aruco_disabled_ids)

        self.perched_cameras = perched_cameras
        if self.perched_cameras:
            self.robot.world.perched = PerchedCameraThread(self.robot)

        self.robot.aruco_id = -1
        self.robot.use_shared_map = False
        self.robot.world.server = ServerThread(self.robot)
        self.robot.world.client = ClientThread(self.robot)
        self.robot.world.is_server = True # Writes directly into perched.camera_pool

        self.world_map = world_map
        self.worldmap_viewer = worldmap_viewer

        self.rrt = rrt
        self.path_viewer = path_viewer

        self.speech = speech
        self.speech_debug = speech_debug
        self.thesaurus = thesaurus

    def start(self):
        global running_fsm
        running_fsm = self
        # Create a particle filter
        if not isinstance(self.particle_filter,ParticleFilter):
            self.particle_filter = SLAMParticleFilter(self.robot, landmark_test=self.landmark_test)
        elif isinstance(self.particle_filter,SLAMParticleFilter):
            self.particle_filter.clear_landmarks()
        pf = self.particle_filter
        self.robot.world.particle_filter = pf

        # Set up kinematics
        self.robot.kine = self.kine_class(self.robot)
        self.robot.was_picked_up = False
        self.robot.carrying = None
        self.robot.fetching = None

        # robot.is_picked_up uses just the cliff detector, and can be fooled.
        # robot.pose.rotation does not encode pitch or roll, only yaw.
        # So use accelerometer data as our backup method.
        self.robot.really_picked_up = \
            (lambda robot :
             (lambda :
              robot.is_picked_up
              or (not robot.is_moving
                  and (robot.accelerometer.z < 5000
                       or robot.accelerometer.z > 13000))))(self.robot)
#                  and (robot.accelerometer.z < 5000
#                       or robot.accelerometer.z > 10300))))(self.robot)

        # World map and path planner
        self.robot.enable_facial_expression_estimation(True)
        self.robot.world.world_map = \
                self.world_map or WorldMap(self.robot)
        self.robot.world.world_map.clear()
        self.robot.world.rrt = self.rrt or RRT(self.robot)

        if self.simple_cli_callback:
            self.make_cubes_available()

        # Polling
        self.set_polling_interval(0.025)  # for kine and motion model update

        # Launch viewers
        if self.cam_viewer:
            if self.cam_viewer is True:
                self.cam_viewer = CamViewer(self.robot)
            self.cam_viewer.start()
        self.robot.world.cam_viewer = self.cam_viewer

        if self.particle_viewer:
            if self.particle_viewer is True:
                self.particle_viewer = \
                    ParticleViewer(self.robot, scale=self.particle_viewer_scale)
            self.particle_viewer.start()
        self.robot.world.particle_viewer = self.particle_viewer

        if self.path_viewer:
            if self.path_viewer is True:
                self.path_viewer = PathViewer(self.robot, self.robot.world.rrt)
            else:
                self.path_viewer.set_rrt(self.robot.world.rrt)
            self.path_viewer.start()
        self.robot.world.path_viewer = self.path_viewer

        if self.worldmap_viewer:
            if self.worldmap_viewer is True:
                self.worldmap_viewer = WorldMapViewer(self.robot)
            self.worldmap_viewer.start()
        self.robot.world.worldmap_viewer = self.worldmap_viewer

        # Request camera image and object recognition streams
        self.robot.camera.image_stream_enabled = True
        self.robot.world.add_event_handler(cozmo.world.EvtNewCameraImage,
                                           self.process_image)

        self.robot.world.add_event_handler(
            cozmo.objects.EvtObjectObserved,
            self.robot.world.world_map.handle_object_observed)

        # Set up cube motion detection
        cubes = self.robot.world.light_cubes
        for i in cubes:
            cubes[i].movement_start_time = None

        self.robot.world.add_event_handler(
            cozmo.objects.EvtObjectMovingStarted,
            self.robot.world.world_map.handle_object_move_started)

        self.robot.world.add_event_handler(
            cozmo.objects.EvtObjectMovingStopped,
            self.robot.world.world_map.handle_object_move_stopped)

        # Start speech recognition if requested
        if self.speech:
            self.speech_listener = SpeechListener(self.robot,self.thesaurus,debug=self.speech_debug)
            self.speech_listener.start()

        # Call parent's start() to launch the state machine by invoking the start node.
        super().start()

    def make_cubes_available(self):
        # Make worldmap cubes and charger accessible to simple_cli
        cubes = self.robot.world.light_cubes
        wc1 = wc2 = wc3 = wcharger = None
        if 1 in cubes:
            wc1 = self.robot.world.world_map.update_cube(cubes[1])
        if 2 in cubes:
            wc2 = self.robot.world.world_map.update_cube(cubes[2])
        if 3 in cubes:
            wc3 = self.robot.world.world_map.update_cube(cubes[3])
        if self.robot.world.charger is not None:
            wcharger = self.robot.world.world_map.update_charger()
        self.simple_cli_callback(wc1, wc2, wc3, wcharger)

    def robot_picked_up(self):
        print('** Robot was picked up!', self.robot.accelerometer)
        self.robot.stop_all_motors()
        self.run_picked_up_handler(self)

    def run_picked_up_handler(self,node):
        """Complex state machines use a picked_up_handler to abort the machine
        gracefully, usually by posting a failure event from the parent."""
        if node.running and hasattr(node,'picked_up_handler'):
            node.picked_up_handler()
        else:
            for child in node.children.values():
                self.run_picked_up_handler(child)

    def robot_put_down(self):
        print('** Robot was put down.')
        pf = self.robot.world.particle_filter
        pf.delocalize()

    def stop(self):
        super().stop()
        self.robot.erouter.clear()
        try:
            self.robot.world.remove_event_handler(cozmo.world.EvtNewCameraImage,
                                                  self.process_image)
        except: pass

    def poll(self):
        global charger_warned
        # Invalidate cube pose if cube has been moving and isn't seen
        move_duration_regular_threshold = 0.5 # seconds
        move_duration_fetch_threshold = 1 # seconds
        cubes = self.robot.world.light_cubes
        now = None
        for i in cubes:
            cube = cubes[i]
            if self.robot.carrying and self.robot.carrying.sdk_obj is cube:
                continue
            if cube.movement_start_time is not None and not cube.is_visible:
                now = now or time.time()
                if self.robot.fetching and self.robot.fetching.sdk_obj is cube:
                    threshold = move_duration_fetch_threshold
                else:
                    threshold = move_duration_regular_threshold
                if (now - cube.movement_start_time) > threshold:
                    cube_id = 'Cube-' + str(i)
                    wcube = self.robot.world.world_map.objects[cube_id]
                    print('Invalidating pose of', wcube)
                    wcube.pose_confidence = -1
                    cube.movement_start_time = None

        if self.simple_cli_callback:
            self.make_cubes_available()

        # Update robot kinematic description
        self.robot.kine.get_pose()

        # Handle robot being picked up or put down
        if self.robot.really_picked_up():
            # robot is in the air
            if self.robot.was_picked_up:
                pass  # we already knew that
            else:
                self.picked_up_callback()
        else:  # robot is on the ground
            pf = self.robot.world.particle_filter
            if pf:
                if self.robot.was_picked_up:
                    self.put_down_handler()
                else:
                    pf.move()
        self.robot.was_picked_up = self.robot.really_picked_up()

        # Handle robot being placed on the charger
        if self.robot.is_on_charger:
            if not charger_warned:
                print("\n** On charger. Type robot.drive_off_charger_contacts() to enable motion.")
                charger_warned = True
        else:
            charger_warned = False

    def user_image(self,image,gray): pass

    def user_annotate(self,image):
        return image

    def process_image(self,event,**kwargs):
        if self.cam_viewer:
            # if show cam_viewer, run the process_image under cam_viewer
            pass
        else:
            curim = numpy.array(event.image.raw_image) #cozmo-raw image
            gray = cv2.cvtColor(curim,cv2.COLOR_BGR2GRAY)

            # Aruco image processing
            if self.aruco:
                self.robot.world.aruco.process_image(gray)
            # Other image processors can run here if the user supplies them.
            self.user_image(curim,gray)
            # Done with image processing

            # Annotate and display image if requested
            if self.force_annotation or self.viewer is not None:
                scale = self.annotated_scale_factor
                # Apply Cozmo SDK annotations and rescale.
                if self.annotate_sdk:
                    coz_ann = event.image.annotate_image(scale=scale)
                    annotated_im = numpy.array(coz_ann)
                elif scale != 1:
                    shape = curim.shape
                    dsize = (scale*shape[1], scale*shape[0])
                    annotated_im = cv2.resize(curim, dsize)
                else:
                    annotated_im = curim
                # Aruco annotation
                if self.aruco and \
                       len(self.robot.world.aruco.seen_marker_ids) > 0:
                    annotated_im = self.robot.world.aruco.annotate(annotated_im,scale)
                # Other annotators can run here if the user supplies them.
                annotated_im = self.user_annotate(annotated_im)
                # Done with annotation
                annotated_im = cv2.cvtColor(annotated_im,cv2.COLOR_RGB2BGR)

        # Use this heartbeat signal to look for new landmarks
        pf = self.robot.world.particle_filter
        if pf and not self.robot.really_picked_up():
            pf.look_for_new_landmarks()

        # Finally update the world map
        self.robot.world.world_map.update_map()
