from math import pi

from .kine import *
from cozmo_fsm import transform
from .transform import tprint
from .shapes import *

class CozmoKinematics(Kinematics):
    def __init__(self,robot):
        base_frame = Joint('base', collision_model=Circle(transform.point(), radius=50))

        # cor is center of rotation
        cor_frame = Joint('cor', parent=base_frame, r=-20.)

        world_frame = Joint('world', parent=base_frame, type='world', getter=self.get_world)

        front_axle_frame = Joint('front_axle', parent=base_frame, alpha=pi/2)
        back_axle_frame = Joint('back_axle', parent=base_frame, r=-46., alpha=pi/2)

        # This frame is on the midline.  Could add separate left and right shoulders.
        # Positive angle is up, so z must point to the right.
        # x is forward, y points up.
        shoulder_frame = Joint('shoulder', parent=base_frame,
                               type='revolute', getter=self.get_shoulder,
                               d=21., r=-39., alpha=pi/2)
        lift_attach_frame = Joint('lift_attach', parent=shoulder_frame, type='revolute',
                                  getter=self.get_lift_attach, r=66.)

        # Positive head angle is up, so z must point to the right.
        # With x pointing forward, y must point up.
        head_frame = Joint('head', parent=base_frame, type='revolute',
                           getter=self.get_head,
                           d=35., r=-10., alpha=pi/2)

        # Dummy joint located below head joint at level of the camera frame,
        # and x axis points down, z points forward, y points left
        camera_dummy = Joint('camera_dummy', parent=head_frame,
                             theta=-pi/2, r=-7.5, alpha=-pi/2)
        # x axis points right, y points down, z points forward
        camera_frame = Joint('camera', parent=camera_dummy, d=15., theta=-pi/2)

        joints = [base_frame, cor_frame,
                  front_axle_frame, back_axle_frame,
                  shoulder_frame, lift_attach_frame,
                  head_frame, camera_dummy, camera_frame]

        super().__init__(joints,robot)

    def get_head(self):
        return self.robot.head_angle.radians

    def get_shoulder(self):
        # Formula supplied by Mark Wesley at Anki
        return math.asin( (self.robot.lift_height.distance_mm-45.0) / 66.0)

    def get_lift_attach(self):
        return -self.get_shoulder()

    def get_world(self):
        return self.robot.world.particle_filter.pose_estimate()
