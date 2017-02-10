from math import pi

from .kine import *
from .transform import tprint

class CozmoKinematics(Kinematics):
    def __init__(self,robot):
        base_frame = Joint('base')
        cor_frame = Joint('cor', base_frame, r=-20.)

        front_axle_frame = Joint('front_axle', parent=base_frame, alpha=pi/2)
        back_axle_frame = Joint('back_axle', parent=base_frame, r=-46., alpha=pi/2)

        shoulder_frame = Joint('shoulder', parent=base_frame,
                               type='revolute', getter=self.get_shoulder,
                               d=-21., r=39., alpha = pi/2)
        lift_attach_frame = Joint('lift_attach', parent=shoulder_frame, type='revolute',
                                  getter=self.get_lift_attach, r=-66.)

        head_frame = Joint('head', parent=base_frame, type='revolute',
                           getter=self.get_head,
                           d=35., r=-10., alpha=pi/2)
        camera_dummy = Joint('camera_dummy', parent=head_frame,
                             theta=-pi/2, r=7.5, alpha=pi/2)
        camera_frame = Joint('camera', parent=camera_dummy, d=15.)

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
