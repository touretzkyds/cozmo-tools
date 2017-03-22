from cozmo_fsm import *
from cozmo.util import degrees, Pose

class PF_Cube(StateMachineProgram):
    def __init__(self):
        landmarks = {
            cube1 : Pose( 55, 160, 0, angle_z=degrees(90)),
            cube2 : Pose(160,  55, 0, angle_z=degrees( 0)),
            cube3 : Pose(160, -55, 0, angle_z=degrees( 0))
        }
        pf = ParticleFilter(robot,
                            landmarks = landmarks,
                            sensor_model = CubeSensorModel(robot))
        super().__init__(particle_filter=pf, particle_viewer=True)
