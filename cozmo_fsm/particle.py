"""
Particle filter localization.
"""

import math, array, random
import numpy as np
from math import pi, sqrt, sin, cos, atan2, exp

class Particle():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.log_weight = 0
        self.weight = 1

    def __repr__(self):
        return '<Particle (%5.2f,%5.2f) %4.1f deg. log_wt=%f>' % \
               (self.x, self.y, self.theta*180/pi, self.log_weight)

#================ Particle Initializers ================

class ParticleInitializer(): pass

class RandomWithinRadius(ParticleInitializer):
    """ Normally distribute particles within a radius, with random heading. """
    def __init__(self,radius=200):
        self.radius = radius

    def initialize(self, particles):
        for p in particles:
            qangle = random.random()*2*pi
            r = random.gauss(0, self.radius/2) + self.radius/1.5
            p.x = r * cos(qangle)
            p.y = r * sin(qangle)
            p.theta = random.random()*2*pi
            p.log_weight = 0.0
            p.weight = 1.0

class RobotPosition(ParticleInitializer):
    """ Initialize all particles to the robot's current position; the motion model will jitter tjem. """
    def __init__(self, x=None, y=None, theta=None):
        self.x = x
        self.y = y
        self.theta = theta
        p.log_weight = 0.0
        p.weight = 1.0
        
    def initialize(self,particles):
        if self.x == None:
            self.x = self.robot.pose.position.x
            self.y = self.robot.pose.position.y
            self.theta = self.robot.pose.rotation.angle_z.radians
        for p in particles:
            p.x = self.x
            p.y = self.y
            p.theta = self.theta
    

#================ Motion Model ================

class MotionModel():
    def __init__(self, robot):
        self.robot = robot

class DefaultMotionModel(MotionModel):
    def __init__(self, robot, trans_var=0.1, rot_var=0.1):
        super().__init__(robot)
        self.trans_var = trans_var
        self.rot_var = rot_var
        self.old_pose = robot.pose

    def move(self, particles):
        old_pose = self.old_pose
        new_pose = self.robot.pose
        old_xyz = old_pose.position.x_y_z
        new_xyz = new_pose.position.x_y_z
        old_hdg = old_pose.rotation.angle_z.radians
        new_hdg = new_pose.rotation.angle_z.radians
        turn_angle = wrap_angle(new_hdg - old_hdg)
        dx = new_xyz[0] - old_xyz[0]
        dy = new_xyz[1] - old_xyz[1]
        dist = sqrt(dx*dx + dy*dy)
        # If we didn't move much, ignore the motion
        if dist < 1 and abs(turn_angle) < 0.05:
            return
        self.old_pose = new_pose
        # Did we drive forward or backward?
        fwd_xy = (old_xyz[0] + dist * cos(old_hdg+turn_angle/2),
                  old_xyz[1] + dist * sin(old_hdg+turn_angle/2))
        rev_xy = (old_xyz[0] - dist * cos(old_hdg+turn_angle/2),
                  old_xyz[1] - dist * sin(old_hdg+turn_angle/2))
        fwd_dx = fwd_xy[0] - new_xyz[0]
        fwd_dy = fwd_xy[1] - new_xyz[1]
        rev_dx = rev_xy[0] - new_xyz[0]
        rev_dy = rev_xy[1] - new_xyz[1]
        if (fwd_dx*fwd_dx + fwd_dy*fwd_dy) >  (rev_dx*rev_dx + rev_dy*rev_dy):
            dist = - dist
        for p in particles:
            if abs(dist) >= 1:
                pdist = dist + random.gauss(0, self.trans_var)
            else:
                pdist = 0
            if abs(turn_angle) > 0.05:
                pturn = turn_angle + random.gauss(0, self.rot_var)
            else:
                pturn = 0
            p.theta += pturn/2
            p.x = p.x + cos(p.theta)*pdist
            p.y = p.y + sin(p.theta)*pdist
            p.theta += pturn/2
            p.theta = wrap_angle(p.theta)

#================ Sensor Model ================

class SensorModel():
    def __init__(self, robot, landmarks=dict()):
        self.robot = robot
        self.set_landmarks(landmarks)
        self.last_evaluate_pose = robot.pose

    def set_landmarks(self,landmarks):
        self.landmarks = landmarks

    def compute_robot_motion(self):
        # How much did we move since last evaluation?
        dx = self.robot.pose.position.x - self.last_evaluate_pose.position.x
        dy = self.robot.pose.position.y - self.last_evaluate_pose.position.y
        dist = sqrt(dx*dx + dy*dy)
        turn_angle = wrap_angle(self.robot.pose.rotation.angle_z.radians -
                                self.last_evaluate_pose.rotation.angle_z.radians)
        return (dx,dy,dist,turn_angle)

class ArucoDistanceSensorModel(SensorModel):
    """Sensor model using only landmark distances."""
    def __init__(self, robot, landmarks=dict(), distance_variance=100):
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dx,dy,dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seenMarkerObjects because vision is in another thread.
        seenMarkerObjects = self.robot.world.aruco.seenMarkerObjects
        # Process each seen marker:
        for id in seenMarkerObjects:
            if id in self.landmarks:
                sensor_dist = seenMarkerObjects[id].camera_distance
                landmark_spec = self.landmarks[id]
                lm_x = landmark_spec.position.x
                lm_y = landmark_spec.position.y
                for p in particles:
                    dx = lm_x - p.x
                    dy = lm_y - p.y
                    predicted_dist = sqrt(dx*dx + dy*dy)
                    error = sensor_dist - predicted_dist
                    p.log_weight -= (error*error)/self.distance_variance
        return True

class ArucoBearingSensorModel(SensorModel):
    """Sensor model using only landmark bearings."""
    def __init__(self, robot, landmarks=dict(), bearing_variance=0.1):
        super().__init__(robot,landmarks)
        self.bearing_variance = bearing_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dx,dy,dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seenMarkerObjects because vision is in another thread.
        seenMarkerObjects = self.robot.world.aruco.seenMarkerObjects
        # Process each seen marker:
        for id in seenMarkerObjects:
            if id in self.landmarks:
                sensor_coords = seenMarkerObjects[id].camera_coords
                sensor_bearing = atan2(sensor_coords[0], sensor_coords[2])
                landmark_spec = self.landmarks[id]
                lm_x = landmark_spec.position.x
                lm_y = landmark_spec.position.y
                for p in particles:
                    dx = lm_x - p.x
                    dy = lm_y - p.y
                    predicted_bearing = wrap_angle(atan2(dy,dx) - p.theta)
                    error = wrap_angle(sensor_bearing - predicted_bearing)
                    p.log_weight -= (error * error) / self.bearing_variance
        return True

class ArucoCombinedSensorModel(SensorModel):
    """Sensor model using combined distance and bearing information."""
    def __init__(self, robot, landmarks=dict(), distance_variance=200):
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dx,dy,dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seenMarkerObjects because vision is in another thread.
        seenMarkerObjects = self.robot.world.aruco.seenMarkerObjects
        # Process each seen marker:
        for id in seenMarkerObjects:
            if id in self.landmarks:
                sensor_dist = seenMarkerObjects[id].camera_distance
                sensor_coords = seenMarkerObjects[id].camera_coords
                sensor_bearing = atan2(sensor_coords[0], sensor_coords[2])
                landmark_spec = self.landmarks[id]
                lm_x = landmark_spec.position.x
                lm_y = landmark_spec.position.y
                for p in particles:
                    # Use sensed bearing and distance to get particle's
                    # estimate of landmark position on the world map.
                    predicted_pos_x = p.x + sensor_dist * cos(p.theta + sensor_bearing)
                    predicted_pos_y = p.y + sensor_dist * sin(p.theta + sensor_bearing)
                    dx = lm_x - predicted_pos_x
                    dy = lm_y - predicted_pos_y
                    error_sq = dx*dx + dy*dy
                    p.log_weight -= error_sq / self.distance_variance
        return True

#================ Particle Filter ================

class ParticleFilter():
    def __init__(self, robot, num_particles=500,
                 initializer = RandomWithinRadius(),
                 motion_model = "default",
                 sensor_model = "default",
                 landmarks = dict()):
        self.robot = robot
        self.num_particles = num_particles
        self.initializer = initializer
        if motion_model == "default":
            motion_model = DefaultMotionModel(robot)
        if sensor_model == "default":
            sensor_model = ArucoDistanceSensorModel(robot)
        if sensor_model:
            sensor_model.set_landmarks(landmarks)
        self.motion_model = motion_model
        self.sensor_model = sensor_model
        self.particles = [Particle() for i in range(num_particles)]
        self.min_log_weight = -300  # prevent floating point underflow in exp()
        self.initializer.initialize(self.particles)
        self.exp_weights = np.empty(self.num_particles)
        self.new_x = np.empty(self.num_particles)
        self.new_y = np.empty(self.num_particles)
        self.new_theta = np.empty(self.num_particles)
        self.cdf = np.empty(self.num_particles)

    def move(self):
        self.motion_model.move(self.particles)
        if self.sensor_model.evaluate(self.particles):  # true if log_weights changed
            var = self.update_weights()
            if var > 0:
                print('resample')
                self.resample()

    def pose_estimate(self):
        cx = 0.0; cy = 0.0
        hsin = 0.0; hcos = 0.0
        weight_sum = 0.0
        for p in self.particles:
            cx += p.weight * p.x
            cy += p.weight * p.y
            hsin += sin(p.theta) * p.weight
            hcos += cos(p.theta) * p.weight
            weight_sum += p.weight
        cx /= weight_sum
        cy /= weight_sum
        return (cx, cy, atan2(hsin,hcos))

    def update_weights(self):
        # Clip the log_weight values and calculate the new weights.
        if max(p.log_weight for p in self.particles) >= self.min_log_weight:
            wt_inc = 0.0
        else:
            wt_inc = - self.min_log_weight / 2.0
        exp_weights = self.exp_weights
        particles = self.particles
        for i in range(self.num_particles):
            p = particles[i]
            p.log_weight += wt_inc            
            exp_weights[i] = p.weight = exp(p.log_weight)
        variance = np.var(exp_weights)
        return variance

    def resample(self):
        # Compute and normalize the cdf; make local pointers for faster access.
        num_particles = self.num_particles;
        exp_weights = self.exp_weights
        cdf = self.cdf
        cdf[0] = exp_weights[0]
        for i in range(1,num_particles):
            cdf[i] = cdf[i-1] + exp_weights[i]
        cumsum = cdf[-1]
        np.divide(cdf,cumsum,cdf)

        # Prepare for resampling
        dist_var = 2 # mm squared
        hdg_var = 0.01 # radians squared
        x_jitter = np.random.normal(0, dist_var, size=self.num_particles)
        y_jitter = np.random.normal(0, dist_var, size=self.num_particles)
        theta_jitter = np.random.normal(0, hdg_var, size=self.num_particles)

        u = random.random() * (1/num_particles)
        index = 0

        # Resampling loop
        new_x = self.new_x; new_y = self.new_y; new_theta = self.new_theta
        particles = self.particles
        for j in range(num_particles):
            while u > cdf[index]:
                index += 1
            p = particles[index]
            new_x[j] = p.x + x_jitter[j]
            new_y[j] = p.y + y_jitter[j]
            new_theta[j] = wrap_angle(p.theta + theta_jitter[j])
            u += 1/num_particles

        # Copy the new particle values into the old particles and
        # clear the weights.
        for i in range(num_particles):
            p = particles[i]
            p.x = new_x[i]
            p.y = new_y[i]
            p.theta = new_theta[i]
            p.log_weight = 0.0
            p.weight = 1.0

def wrap_angle(angle_rads):
    """Keep angle between -pi and pi."""
    if angle_rads <= -pi:
        return 2*pi + angle_rads
    elif angle_rads > pi:
        return angle_rads - 2*pi
    else:
        return angle_rads

