"""
Particle filter localization.
"""

import math, numpy, array, random
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
        self.force_eval = False  # for debugging via jviewer

    def set_landmarks(self,landmarks):
        self.landmarks = landmarks

class ArucoDistanceSensorModel(SensorModel):
    def __init__(self, robot, landmarks=dict(), dist_variance=100):
        super().__init__(robot,landmarks)
        self.dist_variance = dist_variance

    def evaluate(self,particles):
        # Returns true if particles were evaluated.

        # How much did we move since last evaluation?
        dx = self.robot.pose.position.x - self.last_evaluate_pose.position.x
        dy = self.robot.pose.position.y - self.last_evaluate_pose.position.y
        dist = sqrt(dx*dx + dy*dy)
        turn_angle = wrap_angle(self.robot.pose.rotation.angle_z.radians -
                                self.last_evaluate_pose.rotation.angle_z.radians)

        # Only evaluate if the robot moved enough for evaluation to be worthwhile,
        # or the user requested an evaluation (for debugging via force_eval).
        if not self.force_eval and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.force_eval = False  # was set by jviewer
        self.last_evaluate_pose = self.robot.pose
        eta = 0.01   # scale the weights
        for (id,coords) in self.landmarks.items():
            if id in self.robot.world.aruco.seenMarkers:
                sensor_dist = self.robot.world.aruco.seenMarkerObjects[id].translation[2]
                for p in particles:
                    dx = p.x - coords[0]
                    dy = p.y - coords[1]
                    predicted_dist = sqrt(dx*dx + dy*dy)
                    error = sensor_dist - predicted_dist
                    p.log_weight -= eta * (error * error) / self.dist_variance
                    p.weight = exp(p.log_weight)
        return True

#================ Particle Filter ================

class ParticleFilter():
    def __init__(self, robot, num_particles=500, radius=200,
                 motion_model = "default",
                 sensor_model = "default",
                 landmarks = None):
        self.robot = robot
        self.num_particles = num_particles
        if motion_model == "default":
            motion_model = DefaultMotionModel(robot)
        if sensor_model == "default":
            sensor_model = ArucoDistanceSensorModel(robot)
        if sensor_model:
            sensor_model.set_landmarks(landmarks)
        self.motion_model = motion_model
        self.sensor_model = sensor_model
        self.particles = [Particle() for i in range(num_particles)]
        if radius is None:
            self.set_particles_to_position()
        else:
            self.randomize_particles(radius)

    def randomize_particles(self,radius):
        for p in self.particles:
            qangle = random.random()*2*pi
            r = random.gauss(0,radius/2) + radius/1.5
            p.x = r * cos(qangle)
            p.y = r * sin(qangle)
            p.theta = random.random()*2*pi

    def set_particles_to_position(self):
        for p in self.particles:
            p.x = self.robot.pose.position.x
            p.y = self.robot.pose.position.y
            p.theta = self.robot.pose.rotation.angle_z.radians

    def move(self):
        self.motion_model.move(self.particles)
        if self.sensor_model.evaluate(self.particles):
            var = self.weight_variance()
            if var > 0:
                self.resample()

    def pose_estimate(self):
        cx = 0
        cy = 0
        hsin = 0
        hcos = 0
        for p in self.particles:
            cx += p.weight * p.x
            cy += p.weight * p.y
            hsin += sin(p.theta)
            hcos += cos(p.theta)
        cx *= 1/self.num_particles
        cy *= 1/self.num_particles
        return (cx, cy, atan2(hsin,hcos))

    def weight_variance(self):
        self.log_weights = array.array('f',[p.log_weight for p in self.particles])
        self.exp_weights = numpy.exp(self.log_weights)
        variance = numpy.var(self.exp_weights)
        return variance

    def resample(self):
        # Compute and normalize the cdf
        exp_weights = self.exp_weights
        cdf = exp_weights.__copy__()
        cdf[0] = exp_weights[0]
        for i in range(1,self.num_particles):
            cdf[i] = cdf[i-1] + exp_weights[i]
        cumsum = cdf[-1]
        cdf = numpy.divide(cdf, cumsum)

        # Prepare for resampling
        new_x = exp_weights.__copy__()
        new_y = exp_weights.__copy__()
        new_theta = exp_weights.__copy__()
        u = random.random() * (1/self.num_particles)
        index = 1

        # Resampling loop
        for j in range(self.num_particles):
            while u > cdf[index]:
                index += 1
            p = self.particles[index]
            new_x[j] = p.x
            new_y[j] = p.y
            new_theta[j] = p.theta
            u += 1/self.num_particles

        # Copy the new particle values into the old particles while jittering
        for i in range(self.num_particles):
            p = self.particles[i]
            p.x = random.gauss(new_x[i], 2)
            p.y = random.gauss(new_y[i], 2)
            p.theta = wrap_angle(random.gauss(new_theta[i], 0.01))
            p.log_weight = 0
            p.weight = 1

def wrap_angle(angle_rads):
    if angle_rads < -pi:
        return 2*pi + angle_rads
    elif angle_rads > pi:
        return angle_rads - 2*pi
    else:
        return angle_rads

