"""
PF_Aruco demonstrates a particle filter using ArUco markers.
There are three sensor models provided:
  ArucoDistanceSensorModel -- distances only
  ArucoBearingSensorModel -- bearings only
  ArucoCombinedSensorModel -- combined distances + bearings

In the particle viewer window:
   the WASD keys move the robot
  'e' forces an evaluation step
  'r' forces a resampling
  'v' displays the weight statistics
  'z' re-randomizes the particles.
"""

from cozmo_fsm import *
from cozmo.util import degrees, Pose

class PF_Aruco(StateMachineProgram):
    def __init__(self):
        landmarks = {
            0 : Pose(-55, 160, 0, angle_z=degrees(90)),
            1 : Pose( 55, 160, 0, angle_z=degrees(90)),
            2 : Pose(160,  55, 0, angle_z=degrees( 0)),
            3 : Pose(160, -55, 0, angle_z=degrees( 0))
        }
        pf = ParticleFilter(robot,
                            landmarks = landmarks,
                            sensor_model = ArucoCombinedSensorModel(robot))
        super().__init__(particle_filter=pf, particle_viewer=True)

