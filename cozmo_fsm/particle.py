"""
Particle filter localization.
"""

import math, array, random
import numpy as np
from math import pi, sqrt, sin, cos, atan2, exp
import cv2
import cozmo

from .transform import wrap_angle, wrap_selected_angles, tprint
from .aruco import ArucoMarker
from .cozmo_kin import center_of_rotation_offset
from .worldmap import WorldObject, WallObj, wall_marker_dict, ArucoMarkerObj
from .perched import Cam

class Particle():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.log_weight = 0
        self.weight = 1

    def __repr__(self):
        return '<Particle (%.2f, %.2f) %.1f deg. log_wt=%f>' % \
               (self.x, self.y, self.theta*80/pi, self.log_weight)

#================ Particle Initializers ================

class ParticleInitializer():
    def __init__(self):
        self.pf = None   # must be filled in after creation

class RandomWithinRadius(ParticleInitializer):
    """ Normally distribute particles within a radius, with random heading. """
    def __init__(self,radius=200):
        super().__init__()
        self.radius = radius

    def initialize(self, robot):
        for p in self.pf.particles:
            qangle = random.random()*2*pi
            r = random.gauss(0, self.radius/2) + self.radius/1.5
            p.x = r * cos(qangle)
            p.y = r * sin(qangle)
            p.theta = random.random()*2*pi
            p.log_weight = 0.0
            p.weight = 1.0
        self.pf.pose = (0, 0, 0)
        self.pf.motion_model.old_pose = robot.pose

class RobotPosition(ParticleInitializer):
    """ Initialize all particles to the robot's current position or a constant;
    the motion model will jitter them. """
    def __init__(self, x=None, y=None, theta=None):
        super().__init__()
        self.x = x
        self.y = y
        self.theta = theta
        
    def initialize(self, robot):
        if self.x == None:
            x = robot.pose.position.x
            y = robot.pose.position.y
            theta = robot.pose.rotation.angle_z.radians
        else:
            x = self.x
            y = self.y
            theta = self.theta
        for p in self.pf.particles:
            p.x = x
            p.y = y
            p.theta = theta
            p.log_weight = 0.0
            p.weight = 1.0
        self.pf.pose = (x, y, theta)
        self.pf.motion_model.old_pose = robot.pose
    

#================ Motion Model ================

class MotionModel():
    def __init__(self, robot):
        self.robot = robot

class DefaultMotionModel(MotionModel):
    def __init__(self, robot, sigma_trans=0.1, sigma_rot=0.01):
        super().__init__(robot)
        self.sigma_trans = sigma_trans
        self.sigma_rot = sigma_rot
        self.old_pose = robot.pose

    def move(self, particles):
        old_pose = self.old_pose
        new_pose = self.robot.pose
        self.old_pose = new_pose
        if not new_pose.is_comparable(old_pose):
            return  # can't path integrate if the robot switched reference frames
        old_xyz = old_pose.position.x_y_z
        new_xyz = new_pose.position.x_y_z
        old_hdg = old_pose.rotation.angle_z.radians
        new_hdg = new_pose.rotation.angle_z.radians
        turn_angle = wrap_angle(new_hdg - old_hdg)
        cor = center_of_rotation_offset
        old_rx = old_xyz[0] + cor * cos(old_hdg)
        old_ry = old_xyz[1] + cor * sin(old_hdg)
        new_rx = new_xyz[0] + cor * cos(new_hdg)
        new_ry = new_xyz[1] + cor * sin(new_hdg)
        dist = sqrt((new_rx-old_rx)**2 + (new_ry-old_ry)**2)
        # Did we drive forward, or was it backward?
        fwd_xy = (old_xyz[0] + dist * cos(old_hdg+turn_angle/2),
                  old_xyz[1] + dist * sin(old_hdg+turn_angle/2))
        rev_xy = (old_xyz[0] - dist * cos(old_hdg+turn_angle/2),
                  old_xyz[1] - dist * sin(old_hdg+turn_angle/2))
        fwd_dx = fwd_xy[0] - new_xyz[0]
        fwd_dy = fwd_xy[1] - new_xyz[1]
        rev_dx = rev_xy[0] - new_xyz[0]
        rev_dy = rev_xy[1] - new_xyz[1]
        if (fwd_dx*fwd_dx + fwd_dy*fwd_dy) >  (rev_dx*rev_dx + rev_dy*rev_dy):
            dist = - dist    # we drove backward
        rot_var = 0 if abs(turn_angle) < 0.001 else self.sigma_rot
        for p in particles:
            pdist = dist * (1 + random.gauss(0, self.sigma_trans))
            pturn = random.gauss(turn_angle, rot_var)
            # Correct for the center of rotation being behind the base frame
            # (xc,yc) is the center of rotation
            xc = p.x + cor * cos(p.theta)
            yc = p.y + cor * sin(p.theta)
            # Make half the turn, translate, then complete the turn
            p.theta = p.theta + pturn/2
            p.x = xc + cos(p.theta) * pdist
            p.y = yc + sin(p.theta) * pdist
            p.theta = wrap_angle(p.theta + pturn/2)
            # Move from center of rotation back to (rotated) base frame
            p.x = p.x - cor * cos(p.theta)
            p.y = p.y - cor * sin(p.theta)

#================ Sensor Model ================

class SensorModel():
    def __init__(self, robot, landmarks=None):
        self.robot = robot
        if landmarks is None:
            landmarks = dict()
        self.set_landmarks(landmarks)
        self.last_evaluate_pose = robot.pose

    def set_landmarks(self,landmarks):
        self.landmarks = landmarks

    def compute_robot_motion(self):
        # How much did we move since last evaluation?
        if self.robot.pose.is_comparable(self.last_evaluate_pose):
            dx = self.robot.pose.position.x - self.last_evaluate_pose.position.x
            dy = self.robot.pose.position.y - self.last_evaluate_pose.position.y
            dist = sqrt(dx*dx + dy*dy)
            turn_angle = wrap_angle(self.robot.pose.rotation.angle_z.radians -
                                    self.last_evaluate_pose.rotation.angle_z.radians)
        else:
            dist = 0
            turn_angle = 0
            print('** Robot origin_id changed from', self.last_evaluate_pose.origin_id,
                  'to', self.robot.pose.origin_id)
            self.last_evaluate_pose = self.robot.pose
        return (dist,turn_angle)

class ArucoDistanceSensorModel(SensorModel):
    """Sensor model using only landmark distances."""
    def __init__(self, robot, landmarks=None, distance_variance=100):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dist,turn_angle) = self.compute_robot_motion()
        if (not force) and (dist < 5) and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seen_marker_objects because vision is in another thread.
        seen_marker_objects = self.robot.world.aruco.seen_marker_objects
        # Process each seen marker:
        for id in seen_marker_objects:
            if id in self.landmarks:
                sensor_dist = seen_marker_objects[id].camera_distance
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
    def __init__(self, robot, landmarks=None, bearing_variance=0.1):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.bearing_variance = bearing_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seen_marker_objects because vision is in another thread.
        seen_marker_objects = self.robot.world.aruco.seen_marker_objects
        # Process each seen marker:
        for id in seen_marker_objects:
            if id in self.landmarks:
                sensor_coords = seen_marker_objects[id].camera_coords
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
    def __init__(self, robot, landmarks=None, distance_variance=200):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        # Cache seen_marker_objects because vision is in another thread.
        seen_marker_objects = self.robot.world.aruco.seen_marker_objects
        # Process each seen marker:
        for id in seen_marker_objects:
            if id in self.landmarks:
                sensor_dist = seen_marker_objects[id].camera_distance
                sensor_coords = seen_marker_objects[id].camera_coords
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

class CubeOrientSensorModel(SensorModel):
    """Sensor model using only orientation information."""
    def __init__(self, robot, landmarks=None, distance_variance=200):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        seenCubes = [cube for cube in self.robot.world.light_cubes.values()
                     if cube.is_visible]
        # Process each seen cube if it's a landmark:
        for cube in seenCubes:
            if cube in self.landmarks:
                sensor_dx = cube.pose.position.x - self.robot.pose.position.x
                sensor_dy = cube.pose.position.y - self.robot.pose.position.y
                sensor_dist = sqrt(sensor_dx*sensor_dx + sensor_dy*sensor_dy)
                angle = atan2(sensor_dy,sensor_dx)
                sensor_bearing = \
                    wrap_angle(angle - self.robot.pose.rotation.angle_z.radians)
                #sensor_orient = wrap_angle(robot.pose.rotation.angle_z.radians -
                #                           cube.pose.rotation.angle_z.radians +
                #                           sensor_bearing)
                # simplifies to...
                sensor_orient = wrap_angle(angle - cube.pose.rotation.angle_z.radians)

                landmark_spec = self.landmarks[cube]
                lm_x = landmark_spec.position.x
                lm_y = landmark_spec.position.y
                lm_orient = landmark_spec.rotation.angle_z.radians

                for p in particles:
                    # ... Orientation error:
                    #predicted_bearing = wrap_angle(atan2(lm_y-p.y, lm_x-p.x) - p.theta)
                    #predicted_orient = wrap_angle(p.theta - lm_orient + predicted_bearing)
                    # simplifies to...
                    predicted_orient = wrap_angle(atan2(lm_y-p.y, lm_x-p.x) - lm_orient)
                    error_sq = ((predicted_orient - sensor_orient)*sensor_dist)**2
                    p.log_weight -= error_sq / self.distance_variance
        return True

class CubeSensorModel(SensorModel):
    """Sensor model using combined distance, bearing, and orientation information."""
    def __init__(self, robot, landmarksNone, distance_variance=200):
        if landmarks is None:
            landmarks = dict()
        super().__init__(robot,landmarks)
        self.distance_variance = distance_variance

    def evaluate(self,particles,force=False):
        # Returns true if particles were evaluated.
        # Called with force=True from particle_viewer to force evaluation.

        # Only evaluate if the robot moved enough for evaluation to be worthwhile.
        (dist,turn_angle) = self.compute_robot_motion()
        if not force and dist < 5 and abs(turn_angle) < math.radians(5):
            return False
        self.last_evaluate_pose = self.robot.pose
        seenCubes = [cube for cube in world.light_cubes.values() if cube.is_visible]
        # Process each seen cube if it's a landmark:
        for cube in seenCubes:
            if cube in self.landmarks:
                sensor_dx = cube.pose.position.x - robot.pose.position.x
                sensor_dy = cube.pose.position.y - robot.pose.position.y
                sensor_dist = sqrt(sensor_dx*sensor_dx + sensor_dy*sensor_dy)
                angle = atan2(sensor_dy,sensor_dx)
                sensor_bearing = wrap_angle(angle - robot.pose.rotation.angle_z.radians)
                #sensor_orient = wrap_angle(robot.pose.rotation.angle_z.radians -
                #                           cube.pose.rotation.angle_z.radians +
                #                           sensor_bearing)
                # simplifies to...
                sensor_orient = wrap_angle(angle - cube.pose.rotation.angle_z.radians)

                landmark_spec = self.landmarks[cube]
                lm_x = landmark_spec.position.x
                lm_y = landmark_spec.position.y
                lm_orient = landmark_spec.rotation.angle_z.radians

                for p in particles:
                    # ... Bearing and distance errror:
                    # Use sensed bearing and distance to get particle's
                    # prediction of landmark position on the world map.
                    predicted_pos_x = p.x + sensor_dist * cos(p.theta + sensor_bearing)
                    predicted_pos_y = p.y + sensor_dist * sin(p.theta + sensor_bearing)
                    dx = lm_x - predicted_pos_x
                    dy = lm_y - predicted_pos_y
                    error1_sq = dx*dx + dy*dy
                    # ... Orientation error:
                    #predicted_bearing = wrap_angle(atan2(lm_y-p.y, lm_x-p.x) - p.theta)
                    #predicted_orient = wrap_angle(p.theta - lm_orient + predicted_bearing)
                    # simplifies to...
                    predicted_orient = wrap_angle(atan2(lm_y-p.y, lm_x-p.x) - lm_orient)
                    error2_sq = (sensor_dist*wrap_angle(predicted_orient - sensor_orient))**2

                    error_sq = error1_sq + error2_sq
                    p.log_weight -= error_sq / self.distance_variance
        return True


#================ Particle Filter ================

class ParticleFilter():
    def __init__(self, robot, num_particles=500,
                 initializer = RandomWithinRadius(),
                 motion_model = "default",
                 sensor_model = "default",
                 particle_factory = Particle,
                 landmarks = None):
        if landmarks is None:
            landmarks = dict()   # don't do this in the argument list!
        self.robot = robot
        self.num_particles = num_particles
        self.initializer = initializer
        self.initializer.pf = self
        if motion_model == "default":
            motion_model = DefaultMotionModel(robot)
        if sensor_model == "default":
            sensor_model = ArucoCombinedSensorModel(robot)
        if sensor_model:
            sensor_model.set_landmarks(landmarks)
        self.motion_model = motion_model
        self.sensor_model = sensor_model
        self.particle_factory = particle_factory
        self.particles = [particle_factory() for i in range(num_particles)]
        self.best_particle = self.particles[0]
        self.best_particle_index = 0
        self.min_log_weight = -300  # prevent floating point underflow in exp()
        self.initializer.initialize(robot)
        self.exp_weights = np.empty(self.num_particles)
        self.dist_jitter = 2 # mm
        self.hdg_jitter = 0.01 # radians
        self.new_indices = np.empty(self.num_particles, dtype=np.int)
        self.new_x = np.empty(self.num_particles)
        self.new_y = np.empty(self.num_particles)
        self.new_theta = np.empty(self.num_particles)
        self.cdf = np.empty(self.num_particles)
        self.pose = (0., 0., 0.)
        self.variance = (np.array([[0,0],[0,0]]), 0.)

    def move(self):
        self.motion_model.move(self.particles)
        if self.sensor_model.evaluate(self.particles):  # true if log_weights changed
            var = self.update_weights()
            if var > 0:
                #print('resample')
                self.resample()
        if self.robot.carrying:
            self.robot.world.world_map.update_carried_object(self.robot.carrying)

    def pose_estimate(self):
        cx = 0.0; cy = 0.0
        hsin = 0.0; hcos = 0.0
        weight_sum = 0.0
        best_particle_index = 0
        best_particle = self.particles[0]
        for i in range(len(self.particles)):
            p = self.particles[i]
            p.weight = exp(p.log_weight)
            if p.weight > best_particle.weight:
                best_particle = p
                best_particle_index = i
            cx += p.weight * p.x
            cy += p.weight * p.y
            hsin += sin(p.theta) * p.weight
            hcos += cos(p.theta) * p.weight
            weight_sum += p.weight
        if weight_sum == 0:
            weight_sum = 1
        cx /= weight_sum
        cy /= weight_sum
        self.pose = (cx, cy, atan2(hsin,hcos))
        self.best_particle = best_particle
        self.best_particle_index = best_particle_index
        return self.pose

    def variance_estimate(self):
        weight = var_xx = var_xy = var_yy = r_sin = r_cos = 0
        (mu_x, mu_y, mu_theta) = self.pose_estimate()
        for p in self.particles:
            dx = (p.x - mu_x)
            dy = (p.y - mu_y)
            var_xx += dx * dx * p.weight
            var_xy += dx * dy * p.weight
            var_yy += dy * dy * p.weight
            r_sin += sin(p.theta) * p.weight
            r_cos += cos(p.theta) * p.weight
            weight += p.weight
        if weight == 0:
            print('*** weight is zero in variance_estimate() !!!')
            weight = self.num_particles
        xy_var = np.array([[var_xx, var_xy],
                           [var_xy, var_yy]]) / weight
        Rsq = r_sin**2 + r_cos**2
        Rav = sqrt(Rsq) / weight
        theta_var = max(0, 1 - Rav)
        self.variance = (xy_var, theta_var)
        return self.variance

    def update_weights(self):
        # Clip the log_weight values and calculate the new weights.
        max_weight = max(p.log_weight for p in self.particles)
        if max_weight >= self.min_log_weight:
            wt_inc = 0.0
        else:
            wt_inc = - self.min_log_weight / 2.0
            print('wt_inc',wt_inc,'applied for max_weight',max_weight)
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
        exp_weights = self.exp_weights
        cdf = self.cdf
        cumsum = 0
        for i in range(self.num_particles):
            cumsum += exp_weights[i]
            cdf[i] = cumsum
        np.divide(cdf,cumsum,cdf)

        # Resampling loop: choose particles to spawn
        uincr = 1/self.num_particles
        u = random.random() * uincr
        index = 0
        new_indices = self.new_indices
        for j in range(self.num_particles):
            while u > cdf[index]:
                index += 1
            new_indices[j] = index
            u += uincr

        # Now jitter the new particles and copy into the old ones
        self.jitter_new_particles()
        self.install_new_particles()

    def jitter_new_particles(self):
        x_jitter = 0 * np.random.normal(0, self.dist_jitter, size=self.num_particles)
        y_jitter = 0 * np.random.normal(0, self.dist_jitter, size=self.num_particles)
        theta_jitter = 0 * np.random.normal(0, self.hdg_jitter, size=self.num_particles)

        new_x = self.new_x; new_y = self.new_y; new_theta = self.new_theta
        particles = self.particles
        j = 0
        for i in self.new_indices:
            p = particles[i]
            new_x[j] = p.x + x_jitter[j]
            new_y[j] = p.y + y_jitter[j]
            new_theta[j] = wrap_angle(p.theta + theta_jitter[j])
            j += 1

    def install_new_particles(self):
        particles = self.particles
        new_x = self.new_x; new_y = self.new_y; new_theta = self.new_theta
        for i in range(self.num_particles):
            p = particles[i]
            p.x = new_x[i]
            p.y = new_y[i]
            p.theta = new_theta[i]
            p.log_weight = 0.0
            p.weight = 1.0

    def increase_variance(self):
        print('Particle filter: increasing variance.')
        particles = self.particles
        x_jitter = np.random.normal(0, 10*self.dist_jitter, size=self.num_particles)
        y_jitter = np.random.normal(0, 10*self.dist_jitter, size=self.num_particles)
        theta_jitter = np.random.normal(0, 20*self.hdg_jitter, size=self.num_particles)
        for i in range(self.num_particles):
            p = particles[i]
            p.x += x_jitter[i]
            p.y += y_jitter[i]
            p.theta = wrap_angle(p.theta + theta_jitter[i])
            p.log_weight = 0.0
            p.weight = 1.0
        self.variance_estimate()

    def set_pose(self,x,y,theta):
        for i in range(self.num_particles):
            p = self.particles[i]
            p.x = x
            p.y = y
            p.theta = theta
            p.log_weight = 0.0
            p.weight = 1.0
        self.variance_estimate()

    def look_for_new_landmarks(self): pass  # SLAM only

    def clear_landmarks(self):
        print('Not SLAM.  Landmarks are fixed in this particle filter.')

#================ Particle SLAM ================

class SLAMParticle(Particle):
    def __init__(self):
        super().__init__()
        self.landmarks = dict()

    def __repr__(self):
        return '<SLAMParticle (%.2f, %.2f) %.1f deg. log_wt=%f, %d-lm>' % \
               (self.x, self.y, self.theta*180/pi, self.log_weight, len(self.landmarks))

    sigma_r = 50
    sigma_alpha = 15 * (pi/180)
    sigma_phi = 15 * (pi/180)
    sigma_theta =  15 * (pi/180)
    sigma_z = 50
    landmark_sensor_variance_Qt = np.array([[sigma_r**2, 0             , 0],
                                            [0         , sigma_alpha**2, 0],
                                            [0         , 0             , sigma_phi**2]])
    # variance of camera location (cylindrical coordinates)
    # phi is the angle around the Z axis of the robot
    # theta is the angle around the X axis of the camera (pitch)
    camera_sensor_variance_Qt = np.array([[sigma_r**2 , 0             , 0          ,0           , 0],
                                          [0          , sigma_alpha**2, 0          ,0           , 0],
                                          [0          , 0             , sigma_z**2 ,0           , 0],
                                          [0          , 0             , 0          ,sigma_phi**2, 0],
                                          [0          , 0             , 0          ,0           , sigma_theta**2]])

    @staticmethod
    def sensor_jacobian_H(dx, dy, dist):
        """Jacobian of sensor values (r, alpha) wrt particle state x,y
           where (dx,dy) is vector from particle to lm, and
           r = sqrt(dx**2 + dy**2), alpha = atan2(dy,dx), phi = phi"""
        q = dist**2
        sqr_q = dist
        return np.array([[dx/sqr_q, dy/sqr_q, 0],
                         [-dy/q   , dx/q    , 0],
                         [0       , 0       , 1]])

    @staticmethod
    def sensor_jacobian_H_cam(dx, dy, dist):
        """Jacobian of sensor values (r, alpha) wrt particle state x,y
           where (dx,dy) is vector from particle to lm, and
           r = sqrt(dx**2 + dy**2), alpha = atan2(dy,dx), z = z, phi = phi, theta = theta"""
        q = dist**2
        sqr_q = dist
        return np.array([[dx/sqr_q, dy/sqr_q, 0, 0, 0],
                         [-dy/q   , dx/q    , 0, 0, 0],
                         [0       , 0       , 1, 0, 0],
                         [0       , 0       , 0, 1, 0],
                         [0       , 0       , 0, 0, 1],])

    def add_landmark(self, lm_id, sensor_dist, sensor_bearing, sensor_orient):
        direction = self.theta + sensor_bearing
        dx = sensor_dist * cos(direction)
        dy = sensor_dist * sin(direction)
        lm_x = self.x + dx
        lm_y = self.y + dy

        if isinstance(lm_id, cozmo.objects.LightCube):
            lm_orient = sensor_orient
        else:  # AruCo marker
            lm_orient = sensor_orient + self.theta

        lm_mu =  np.array([[lm_x], [lm_y]])
        H = self.sensor_jacobian_H(dx, dy, sensor_dist)
        Hinv = np.linalg.inv(H)
        Q = self.landmark_sensor_variance_Qt
        lm_sigma = Hinv.dot(Q.dot(Hinv.T))
        self.landmarks[lm_id] = (lm_mu, lm_orient, lm_sigma)

    def update_landmark(self, id, sensor_dist, sensor_bearing, sensor_orient,
                        dx, dy, I=np.eye(3)):
        # (dx,dy) is vector from particle to SENSOR position of lm
        (old_mu, old_orient, old_sigma) = self.landmarks[id]
        H = self.sensor_jacobian_H(dx, dy, sensor_dist)
        Ql =  H.dot(old_sigma.dot(H.T)) + self.landmark_sensor_variance_Qt
        Ql_inv = np.linalg.inv(Ql)
        K = old_sigma.dot((H.T).dot(Ql_inv))
        z = np.array([[sensor_dist], [sensor_bearing], [sensor_orient]])
        # (ex,ey) is vector from particle to MAP position of lm
        ex = old_mu[0,0] - self.x
        ey = old_mu[1,0] - self.y
        h = np.array([ [sqrt(ex**2+ey**2)],
                       [wrap_angle(atan2(ey,ex) - self.theta)],
                       [wrap_angle(old_orient - self.theta)] ])
        delta_sensor = wrap_selected_angles(z-h, [1,2])
        if abs(delta_sensor[1,0]) > 0.1 or abs(delta_sensor[0,0]) > 50:
            # The landmark must have moved, so reset our estimate.
            #print('id=%s  dist=%5.1f  brg=%5.1f  orient=%5.1f' %
            #      (id, sensor_dist, sensor_bearing*180/pi, sensor_orient*180/pi))
            #print('delta id=', id, delta_sensor)
            new_mu = np.array([[self.x + sensor_dist*cos(sensor_bearing+self.theta)],
                               [self.y + sensor_dist*sin(sensor_bearing+self.theta)],
                               [sensor_orient]])
            Hinv = np.linalg.inv(H)
            Q = self.landmark_sensor_variance_Qt
            new_sigma = Hinv.dot(Q.dot(Hinv.T))
        else:
            new_mu = np.append(old_mu,[old_orient]).reshape([3,1]) + K.dot(delta_sensor)
            new_sigma = (I - K.dot(H)).dot(old_sigma)
        # [ [x,y], [orient], covariance_matrix]
        self.landmarks[id] = (new_mu[0:2], new_mu[2], new_sigma)

    def add_landmark_cam(self, lm_id, sensor_dist, sensor_bearing, sensor_height, sensor_phi, sensor_theta):
        direction = self.theta + sensor_bearing
        dx = sensor_dist * cos(direction)
        dy = sensor_dist * sin(direction)
        lm_x = self.x + dx
        lm_y = self.y + dy

        lm_height = (sensor_height,wrap_angle(sensor_phi+self.theta), sensor_theta)

        lm_mu =  np.array([[lm_x], [lm_y]])
        H = self.sensor_jacobian_H_cam(dx, dy, sensor_dist)
        Hinv = np.linalg.inv(H)
        Q = self.camera_sensor_variance_Qt
        lm_sigma = Hinv.dot(Q.dot(Hinv.T))
        # [ [x,y], [z,orient,pitch], covarience_matrix]
        self.landmarks[lm_id] = (lm_mu, lm_height, lm_sigma)

    def update_landmark_cam(self, id, sensor_dist, sensor_bearing, sensor_height, sensor_phi, sensor_theta,
                        dx, dy, I=np.eye(5)):
        # (dx,dy) is vector from particle to SENSOR position of lm
        (old_mu, old_height, old_sigma) = self.landmarks[id]
        H = self.sensor_jacobian_H_cam(dx, dy, sensor_dist)
        Ql =  H.dot(old_sigma.dot(H.T)) + self.camera_sensor_variance_Qt
        Ql_inv = np.linalg.inv(Ql)
        K = old_sigma.dot((H.T).dot(Ql_inv))
        z = np.array([[sensor_dist], [sensor_bearing], [sensor_height], [wrap_angle(sensor_phi+self.theta)], [sensor_theta]])
        # (ex,ey) is vector from particle to MAP position of lm
        ex = old_mu[0,0] - self.x
        ey = old_mu[1,0] - self.y
        h = np.array([[sqrt(ex**2+ey**2)],
                      [wrap_angle(atan2(ey,ex) - self.theta)],
                      [old_height[0]],
                      [old_height[1]],
                      [old_height[2]]])
        new_mu = np.append(old_mu,[old_height]).reshape([5,1]) + K.dot(wrap_selected_angles(z - h,[1,3,4]))
        new_sigma = (I - K.dot(H)).dot(old_sigma)
        # [ [x,y], [z,orient,pitch], covariance_matrix]
        self.landmarks[id] = (new_mu[0:2], new_mu[2:5], new_sigma)


class SLAMSensorModel(SensorModel):
    @staticmethod
    def is_cube(x):
        return isinstance(x, cozmo.objects.LightCube)

    @staticmethod
    def is_aruco(x):
        return isinstance(x, ArucoMarker)
    
    def __init__(self, robot, landmark_test=None, landmarks=None,
                 distance_variance=200):
        if landmarks is None:
            landmarks = dict()
        if landmark_test is None:
            landmark_test = self.is_cube
        self.landmark_test = landmark_test
        self.distance_variance = distance_variance
        self.candidate_landmarks = dict()
        self.use_perched_cameras = False
        super().__init__(robot,landmarks)

    def rotationMatrixToEulerAngles(self, R) :
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = atan2(R[2,1] , R[2,2])
            y = atan2(-R[2,0], sy)
            z = atan2(R[1,0], R[0,0])
        else :
            x = atan2(-R[1,2], R[1,1])
            y = atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    def infer_wall(self, id, markers):
        # Find rotation and translation vector from camera frame using SolvePnP
        world_points = []
        image_points = []
        marker_size = self.robot.world.aruco.marker_size
        wall_spec = wall_marker_dict.get(markers[0][0], None)
        if wall_spec is None: return  # spurious marker
        for key, value in markers:
            (s, (cx, cy)) = wall_spec.markers[key]

            world_points.append((cx-s*marker_size/2, cy+marker_size/2, s))
            world_points.append((cx+s*marker_size/2, cy+marker_size/2, s))
            world_points.append((cx+s*marker_size/2, cy-marker_size/2, s))
            world_points.append((cx-s*marker_size/2, cy-marker_size/2, s))

            image_points.append(value[0])
            image_points.append(value[1])
            image_points.append(value[2])
            image_points.append(value[3])

        (success,rvecs, tvecs) = cv2.solvePnP(np.array(world_points),
                                              np.array(image_points),
                                              self.robot.world.aruco.camera_matrix,
                                              self.robot.world.aruco.distortion_array)
        rotationm, jcob = cv2.Rodrigues(rvecs)
        # Change to marker frame
        transformed = np.matrix(rotationm).T*(-np.matrix(tvecs))
        an = self.rotationMatrixToEulerAngles(rotationm)
        # euler angle flip when back of wall is seen
        if an[2] > pi/2:
            wall_orient = wrap_angle(-(an[1]-pi))
        elif an[2] >= -pi/2 and an[2] <= pi/2:
            wall_orient = wrap_angle((an[1]))
        else:
            wall_orient = wrap_angle(-(an[1]+pi))

        wall_x = -transformed[2]*cos(wall_orient) + (transformed[0]-wall_spec.length/2)*sin(wall_orient)
        wall_y = (transformed[0]-wall_spec.length/2)*cos(wall_orient) - -transformed[2]*sin(wall_orient)

        wall = WallObj(id=wall_spec.id, x=wall_x, y=wall_y, theta=-wall_orient)
        return wall

    def generate_walls_from_markers(self, seen_marker_objects, good_markers):
        if self.robot.is_moving: return []
        walls = []
        wall_markers = dict()
        for id in good_markers:
            value = seen_marker_objects[id]
            wall_spec = wall_marker_dict.get(id,None)
            if wall_spec is None: continue  # marker not part of a known wall
            wall_id = wall_spec.id
            markers = wall_markers.get(wall_id, list())
            markers.append((id,value.bbox[0]))
            wall_markers[wall_id] = markers
            # Now infer the walls from the markers
        for (id,markers) in wall_markers.items():
            if len(markers) > 1:
                walls.append(self.infer_wall(id,markers))
        return walls

    def evaluate(self, particles, force=False, just_looking=False):
        # Returns true if particles were evaluated.
        # Call with force=True from particle_viewer to skip distance traveled check.
        # Call with just_looking=True to just look for new landmarks; no evaluation.
        evaluated = False

        # Unless forced, only evaluate if the robot moved enough
        # for evaluation to be worthwhile.
        if not force:
            (dist,turn_angle) = self.compute_robot_motion()
            if (dist < 5) and abs(turn_angle) < 2*pi/180:
                return False
        if not just_looking:
            self.last_evaluate_pose = self.robot.pose
        # Cache seen marker objects because vision is in another thread.
        try:
            seen_marker_objects = self.robot.world.aruco.seen_marker_objects
        except:
            seen_marker_objects = dict()
        seen_landmarks = \
            [cube for cube in self.robot.world.light_cubes.values()
                 if cube.is_visible and self.landmark_test(cube)] + \
            [marker.id for marker in seen_marker_objects.values()
                 if self.landmark_test(marker)]
        for id in seen_landmarks:
            evaluated = self.process_landmark(id, just_looking, seen_marker_objects) or evaluated
        # Good markers have been seen consistently enough to be deemed reliable.
        good_markers = [id for id in seen_marker_objects
                        if id in self.landmarks]  # *** relies on landmark id being an int
        walls = self.generate_walls_from_markers(seen_marker_objects, good_markers)
        for id in walls:
            self.process_landmark(id, just_looking, seen_marker_objects)
        if self.use_perched_cameras:
            # add cammeras that can see the robot as landmarks
            perched = list(self.robot.world.perched.camera_pool.get(self.robot.aruco_id,{}).values())
            for id in perched:
                evaluated = self.process_landmark(id, just_looking, seen_marker_objects) or evaluated
        if evaluated:
            wmax = - np.inf
            for p in particles:
                wmax = max(wmax, p.log_weight)
            min_log_weight = self.robot.world.particle_filter.min_log_weight
            if wmax < min_log_weight:
                wt_inc = min_log_weight - wmax
                # print('wmax=',wmax,'wt_inc=',wt_inc)
                for p in particles:
                    p.log_weight += wt_inc
            self.robot.world.particle_filter.variance_estimate()

        # Update the candidate landmarks and delete any losers
        for id in tuple(self.candidate_landmarks.keys()):
            self.candidate_landmarks[id] -= 1
            if self.candidate_landmarks[id] <= 0:
                # print('*** DELETING CANDIDATE LANDMARK', id)
                del self.candidate_landmarks[id]

        self.robot.world.particle_filter.variance_estimate()
        return evaluated
    
    def process_landmark(self, id, just_looking, seen_marker_objects):
        particles = self.robot.world.particle_filter.particles
        if isinstance(id, cozmo.objects.LightCube):
            # sdk values are in SDK's coordinate system, not ours
            sdk_dx = id.pose.position.x - self.robot.pose.position.x
            sdk_dy = id.pose.position.y - self.robot.pose.position.y
            sensor_dist = sqrt(sdk_dx**2 + sdk_dy**2)
            sdk_bearing = atan2(sdk_dy,sdk_dx)
            # sensor_bearing is lm bearing relative to robot centerline
            sensor_bearing = \
                wrap_angle(sdk_bearing-self.robot.pose.rotation.angle_z.radians)
            # sensor_orient is lm bearing relative to cube's North
            sensor_orient = \
                wrap_angle(sdk_bearing - id.pose.rotation.angle_z.radians)
        elif isinstance(id, WallObj):
            # Turning to polar coordinates
            sensor_dist = sqrt(id.x**2 + id.y**2)
            sensor_bearing = atan2(id.y,id.x)
            sensor_orient = id.theta
            id = id.id
        elif isinstance(id, Cam):
            # turning to cylindrical coordinates
            sensor_dist = sqrt(id.x**2 + id.y**2)
            sensor_bearing = atan2(id.y,id.x)
            sensor_height = id.z
            sensor_phi = id.phi
            sensor_theta = id.theta
            if sensor_height<0:
                print("FLIP!!!")
            # Using str instead of capture object as new object is added by perched_cam everytime
            id = id.cap
        else:  # lm is an AruCo marker
            marker = seen_marker_objects[id]
            sensor_dist = marker.camera_distance
            sensor_bearing = atan2(marker.camera_coords[0],
                                   marker.camera_coords[2])
            # Rotation about Y axis of marker. Fix sign.
            sensor_orient = - marker.euler_rotation[1] * (pi/180)
        if id not in self.landmarks:
            # Not checking for spurious wall as it is very unlikely to see
            # two or more spurious markers simultaneously
            if isinstance(id, int):  # must be ArUco marker
                seen_count = self.candidate_landmarks.get(id,0)
                if seen_count < 5:
                    # add 2 because we're going to subtract 1 at the end
                    self.candidate_landmarks[id] = seen_count + 2
                    return False
            print('  *** ADDING LANDMARK %s at:  distance=%6.1f  bearing=%5.1f deg.' %
                  (id, sensor_dist, sensor_bearing*180/pi))
            for p in particles:
                if isinstance(id, str) and 'Video' in id:
                    # special function for cameras as landmark list has more variables
                    p.add_landmark_cam(id, sensor_dist, sensor_bearing, sensor_height, sensor_phi, sensor_theta)
                else:
                    p.add_landmark(id, sensor_dist, sensor_bearing, sensor_orient)
            # Add new landmark to sensor model's landmark list so worldmap can reference it
            self.landmarks[id] = self.robot.world.particle_filter.particles[0].landmarks[id]
            # Delete new landmark from tentative candidate list; it's established now.
            if isinstance(id, int):
                del self.candidate_landmarks[id]
            return False
        # If we reach here, we're seeing a familiar landmark, so evaluate
        if just_looking:
            # We can't afford to update all the particles on each
            # camera frame so we'll just update particle 0 and use
            # that to update the sensor model.
            pp = [particles[0]]
            evaluated = False
        else:
            # We've moved a bit, so we should update every particle.
            pp = particles
            evaluated = True

        if id in self.robot.world.world_map.objects:
            obj = self.robot.world.world_map.objects[id]
            should_update_landmark = not obj.is_fixed
        else:
            should_update_landmark = True
        landmark_is_camera =  isinstance(id, str) and 'Video' in id

        for p in pp:
            # Use sensed bearing and distance to get
            # particle's prediction of landmark position in
            # the world.  Compare to its stored map position.
            sensor_direction = p.theta + sensor_bearing
            dx = sensor_dist * cos(sensor_direction)
            dy = sensor_dist * sin(sensor_direction)
            predicted_lm_x = p.x + dx
            predicted_lm_y = p.y + dy
            (lm_mu, lm_orient, lm_sigma) = p.landmarks[id]
            map_lm_x = lm_mu[0,0]
            map_lm_y = lm_mu[1,0]
            error_x = map_lm_x - predicted_lm_x
            error_y = map_lm_y - predicted_lm_y
            error1_sq = error_x**2 + error_y**2
            error2_sq = 0 # *** (sensor_dist * wrap_angle(sensor_orient - lm_orient))**2
            p.log_weight -= (error1_sq + error2_sq) / self.distance_variance
            # Update landmark in this particle's map
            if should_update_landmark:
                if landmark_is_camera:
                    # special function for cameras as landmark list has more variables
                    p.update_landmark_cam(id, sensor_dist, sensor_bearing,
                                          sensor_height, sensor_phi, sensor_theta, dx, dy)
                else:
                    p.update_landmark(id, sensor_dist, sensor_bearing,
                                      sensor_orient, dx, dy)
        return evaluated

class SLAMParticleFilter(ParticleFilter):
    def __init__(self, robot, landmark_test=SLAMSensorModel.is_aruco, **kwargs):
        if 'sensor_model' not in kwargs or kwargs['sensor_model'] == 'default':
            kwargs['sensor_model'] = SLAMSensorModel(robot, landmark_test=landmark_test)
        if 'particle_factory' not in kwargs:
            kwargs['particle_factory'] = SLAMParticle
        if 'initializer' not in kwargs:
            kwargs['initializer'] = RobotPosition(0,0,0)
        super().__init__(robot, **kwargs)
        self.initializer.pf = self
        self.new_landmarks = [None] * self.num_particles
        if len(self.sensor_model.landmarks) > 0: screwed()

    def clear_landmarks(self):
        for p in self.particles:
            p.landmarks.clear()
        self.sensor_model.landmarks.clear()

    def add_fixed_landmark(self,landmark):
        mu = np.array([[landmark.x], [landmark.y]])
        theta = np.array([landmark.theta])
        sigma = np.zeros([3,3])
        mu_theta_sigma = (mu, theta, sigma)
        for p in self.particles:
            p.landmarks[landmark.id] = mu_theta_sigma
        self.sensor_model.landmarks[landmark.id] = mu_theta_sigma

    def update_weights(self):
        var = super().update_weights()
        best_particle = self.particles[self.exp_weights.argmax()]
        #self.sensor_model.landmarks = best_particle.landmarks
        self.sensor_model.landmarks = self.particles[0].landmarks
        return var

    def jitter_new_particles(self):
        super().jitter_new_particles()
        particles = self.particles  # make local for faster access
        new_indices = self.new_indices
        new_landmarks = self.new_landmarks
        for i in range(self.num_particles):
            new_landmarks[i] = particles[new_indices[i]].landmarks.copy()

    def install_new_particles(self):
        super().install_new_particles()
        particles = self.particles  # make local for faster access
        new_landmarks = self.new_landmarks
        for i in range(self.num_particles):
            particles[i].landmarks = new_landmarks[i]

    def look_for_new_landmarks(self):
        """Calls evaluate() to find landmarks and add them to the maps.
        Also updates existing landmarks."""
        self.sensor_model.evaluate(self.particles, force=True, just_looking=True)
        self.sensor_model.landmarks = self.particles[0].landmarks
        return
        for key,value in self.particles[0].landmarks.items():
            if (key not in self.sensor_model.landmarks) or \
               not self.sensor_model.landmarks[key].is_fixed:            
                self.sensor_model.landmarks[key] = self.particles[0].landmarks[key]
