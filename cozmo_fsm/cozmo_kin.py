from math import pi

import cozmo

from .kine import *
from cozmo_fsm import transform
from .transform import tprint
from .rrt_shapes import *

# ================ Constants ================

wheelbase = 45 # millimeters
center_of_rotation_offset = -19 # millimeters

# ================================================================

class CozmoKinematics(Kinematics):
    def __init__(self,robot):
        base_frame = Joint('base',
                           description='Base frame: the root of the kinematic tree')

        # cor is center of rotation
        cor_frame = Joint('cor', parent=base_frame,
                          description='Center of rotation',
                          r=-19.,
                          collision_model=Rectangle(transform.point(),
                                                    dimensions=(95,60)))

        # Use link instead of joint for world_frame
        world_frame = Joint('world', parent=base_frame, type='world', getter=self.get_world,
                            description='World origin in base frame coordinates',
                            qmin=None, qmax=None)

        front_axle_frame = Joint('front_axle', parent=base_frame,
                                 description='Center of the front axle',
                                 alpha=pi/2)
        back_axle_frame = Joint('back_axle', parent=base_frame, r=-46., alpha=pi/2)

        # This frame is on the midline.  Could add separate left and right shoulders.
        # Positive angle is up, so z must point to the right.
        # x is forward, y points up.
        shoulder_frame = Joint('shoulder', parent=base_frame,
                               type='revolute', getter=self.get_shoulder,
                               description='Rotation axis of the lift; z points to the right',
                               qmin=cozmo.robot.MIN_LIFT_ANGLE.radians,
                               qmax=cozmo.robot.MAX_LIFT_ANGLE.radians,
                               d=21., r=-39., alpha=pi/2)

        lift_attach_frame = \
            Joint('lift_attach', parent=shoulder_frame, type='revolute',
                  description='Tip of the lift, where cubes attach; distal end of four-bar linkage',
                  getter=self.get_lift_attach, r=66.,
                  qmax = - cozmo.robot.MIN_LIFT_ANGLE.radians,
                  qmin = - cozmo.robot.MAX_LIFT_ANGLE.radians,
                  collision_model=Circle(transform.point(), radius=10))

        # Positive head angle is up, so z must point to the right.
        # With x pointing forward, y must point up.
        head_frame = Joint('head', parent=base_frame, type='revolute',
                           getter=self.get_head,
                           description='Axis of head rotation; z points to the right',
                           qmin=cozmo.robot.MIN_HEAD_ANGLE.radians,
                           qmax=cozmo.robot.MAX_HEAD_ANGLE.radians,
                           d=35., r=-10., alpha=pi/2)

        # Dummy joint located below head joint at level of the camera frame,
        # and x axis points down, z points forward, y points left
        camera_dummy = Joint('camera_dummy', parent=head_frame,
                             description='Dummy joint below the head, at the level of the camera frame',
                             theta=-pi/2, r=-7.5, alpha=-pi/2)
        # x axis points right, y points down, z points forward
        camera_frame = Joint('camera', parent=camera_dummy,
                             description='Camera reference frame; y is down and z is outward',
                             d=15., theta=-pi/2)

        joints = [base_frame, world_frame, cor_frame,
                  front_axle_frame, back_axle_frame,
                  shoulder_frame, lift_attach_frame,
                  head_frame, camera_dummy, camera_frame]

        super().__init__(joints,robot)

    def get_head(self):
        return self.robot.head_angle.radians

    def get_shoulder(self):
        # Formula supplied by Mark Wesley at Anki
        # Check SDK documentation for new lift-related calls that might replace this
        return math.asin( (self.robot.lift_height.distance_mm-45.0) / 66.0)

    def get_lift_attach(self):
        return -self.get_shoulder()

    def get_world(self):
        return self.robot.world.particle_filter.pose_estimate()
