"""
Particle filter localization.
"""

import math, array, random
import numpy as np
from math import pi, sqrt, sin, cos, atan2, exp

import cozmo

from .transform import wrap_angle
from .aruco import ArucoMarker

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
        
    def initialize(self,particles):
        if self.x == None:
            self.x = self.robot.pose.position.x
            self.y = self.robot.pose.position.y
            self.theta = self.robot.pose.rotation.angle_z.radians
        for p in particles:
            p.x = self.x
            p.y = self.y
            p.theta = self.theta
            p.log_weight = 0.0
            p.weight = 1.0
    

#================ Motion Model ================

class MotionModel():
    def __init__(self, robot):
        self.robot = robot

class DefaultMotionModel(MotionModel):
    def __init__(self, robot, sigma_trans=0.1, sigma_rot=0.1):
        super().__init__(robot)
        self.sigma_trans = sigma_trans
        self.sigma_rot = sigma_rot
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
        if dist < 5 and abs(turn_angle) < 0.05:
            return
        self.old_pose = new_pose
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
            dist = - dist
        for p in particles:
            pdist = dist * (1 + random.gauss(0, self.sigma_trans))
            pturn = turn_angle * (1 + random.gauss(0, self.sigma_rot))
            p.theta = wrap_angle(p.theta + pturn/2)
            p.x = p.x + cos(p.theta)*pdist
            p.y = p.y + sin(p.theta)*pdist
            p.theta = wrap_angle(p.theta + pturn/2)

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
        return (dist,turn_angle)

class ArucoDistanceSensorModel(SensorModel):
    """Sensor model using only landmark distances."""
    def __init__(self, robot, landmarks=dict(), distance_variance=100):
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
    def __init__(self, robot, landmarks=dict(), bearing_variance=0.1):
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
    def __init__(self, robot, landmarks=dict(), distance_variance=200):
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
    def __init__(self, robot, landmarks=dict(), distance_variance=200):
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
    def __init__(self, robot, landmarks=dict(), distance_variance=200):
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
                 landmarks = dict()):
        self.robot = robot
        self.num_particles = num_particles
        self.initializer = initializer
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
        self.min_log_weight = -300  # prevent floating point underflow in exp()
        self.initializer.initialize(self.particles)
        self.exp_weights = np.empty(self.num_particles)
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
                print('resample')
                self.resample()

    def pose_estimate(self):
        cx = 0.0; cy = 0.0
        hsin = 0.0; hcos = 0.0
        weight_sum = 0.0
        best_particle = self.particles[0]
        for p in self.particles:
            p.weight = exp(p.log_weight)
            if p.weight > best_particle.weight:
                best_particle = p
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
        cdf[0] = exp_weights[0]
        for i in range(1,self.num_particles):
            cdf[i] = cdf[i-1] + exp_weights[i]
        cumsum = cdf[-1]
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
        dist_jitter = 2 # mm
        hdg_jitter = 0.01 # radians

        x_jitter = np.random.normal(0, dist_jitter, size=self.num_particles)
        y_jitter = np.random.normal(0, dist_jitter, size=self.num_particles)
        theta_jitter = np.random.normal(0, hdg_jitter, size=self.num_particles)

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
    sensor_variance_Qt = np.array([[sigma_r**2, 0             ],
                                [0,          sigma_alpha**2]])

    @staticmethod
    def sensor_jacobian_H(dx, dy, dist):
        """Jacobian of sensor values (r, alpha) wrt particle state x,y
           where (dx,dy) is vector from particle to lm, and
           r = sqrt(dx**2 + dy**2), alpha = atan2(dy,dx)"""
        q = dist**2
        sqr_q = dist
        return np.array([[dx/sqr_q, dy/sqr_q],
                         [-dy/q    , dx/q     ]])

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
        Q = self.sensor_variance_Qt
        lm_sigma = Hinv.dot(Q.dot(Hinv.T))
        self.landmarks[lm_id] = (lm_mu, lm_orient, lm_sigma)

    def update_landmark(self, id, sensor_dist, sensor_bearing, sensor_orient,
                        dx, dy, I=np.eye(2)):
        # (dx,dy) is vector from particle to SENSOR position of lm
        (old_mu, old_orient, old_sigma) = self.landmarks[id]
        H = self.sensor_jacobian_H(dx, dy, sensor_dist)
        Ql =  H.dot(old_sigma.dot(H.T)) + self.sensor_variance_Qt
        Ql_inv = np.linalg.inv(Ql)
        K = old_sigma.dot((H.T).dot(Ql_inv))
        z = np.array([[sensor_dist], [sensor_bearing]])
        # (ex,ey) is vector from particle to MAP position of lm
        ex = old_mu[0,0] - self.x
        ey = old_mu[1,0] - self.y
        h = np.array([[sqrt(ex**2+ey**2)], [wrap_angle(atan2(ey,ex) - self.theta)]])
        new_mu = old_mu + K.dot(z - h)
        new_sigma = (I - K.dot(H)).dot(old_sigma)
        # Should be deriving H for (x,y,phi) instead of (x,y), to update orient
        # For now, just do exponential averaging to update phi
        computed_orient = sensor_orient + self.theta
        ori_c = 9*cos(old_orient) + cos(computed_orient)
        ori_s = 9*sin(old_orient) + sin(computed_orient)
        new_orient = atan2(ori_s, ori_c)
        self.landmarks[id] = (new_mu, new_orient, new_sigma)

class SLAMSensorModel(SensorModel):
    @staticmethod
    def is_cube(x):
        return isinstance(x, cozmo.objects.LightCube)

    @staticmethod
    def is_aruco(x):
        return isinstance(x, ArucoMarker)
    
    def __init__(self, robot, landmark_test=None, landmarks=dict(),
                 distance_variance=200):
        if landmark_test is None:
            landmark_test = self.is_cube
        self.landmark_test = landmark_test
        self.distance_variance = distance_variance
        self.candidate_landmarks = dict()
        super().__init__(robot,landmarks)

    def evaluate(self, particles, force=False, just_looking=False):
        # Returns true if particles were evaluated.
        # Call with force=True from particle_viewer to skip distance traveled check.
        # Call with just_looking=True to just look for new landmarks; no evaluation.
        evaluated = False

        (dist,turn_angle) = self.compute_robot_motion()
        # Unless forced, only evaluate if the robot moved enough
        # for evaluation to be worthwhile.
        if (not force) and (dist < 1) and abs(turn_angle) < math.radians(1):
            return evaluated
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
        # Process each seen landmark:
        for id in seen_landmarks:
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
            else:  # lm is an AruCo marker
                marker = seen_marker_objects[id]
                sensor_dist = marker.camera_distance
                sensor_bearing = atan2(marker.camera_coords[0],
                                       marker.camera_coords[2])
                # For AruCo, sensor_orient is a relative value that must
                # be converted to asbolute for each particle separately since
                # we don't have an independent coordinate system to measure by.
                # Also, OpenCV's rotation has the wrong sign, so we must fix it.
                sensor_orient = - marker.opencv_rotation[1] * (pi/180)
            if id not in particles[0].landmarks:
                seen_count = self.candidate_landmarks.get(id,0)
                if seen_count < 5:
                    # add 2 because we're going to subtract 1 at the end
                    self.candidate_landmarks[id] = seen_count + 2
                    continue
                print('  *** ADDING LANDMARK ', id)
                for p in particles:
                    p.add_landmark(id, sensor_dist, sensor_bearing, sensor_orient)
                del self.candidate_landmarks[id]
                continue
            if just_looking:
                continue
            # If we reach here, we're seeing a familiar landmark, so evaluate
            evaluated = True
            for p in particles:
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
                error2_sq = (sensor_dist * wrap_angle(sensor_orient - lm_orient))**2
                p.log_weight -= (error1_sq + error2_sq) / self.distance_variance
                # Update landmark in this particle's map
                p.update_landmark(id, sensor_dist, sensor_bearing, sensor_orient,
                                  dx, dy)
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
                del self.candidate_landmarks[id]

        self.robot.world.particle_filter.variance_estimate()
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
        self.new_landmarks = [None] * self.num_particles

    def clear_landmarks(self):
        for p in self.particles:
            p.landmarks.clear()
        self.sensor_model.landmarks.clear()

    def update_weights(self):
        var = super().update_weights()
        best_particle = self.particles[self.exp_weights.argmax()]
        self.sensor_model.landmarks = best_particle.landmarks
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
        """Calls evaluate() to find landmarks and add them to the maps."""
        self.sensor_model.evaluate(self.particles, force=True, just_looking=True)
        self.sensor_model.landmarks = self.best_particle.landmarks
