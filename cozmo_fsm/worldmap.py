from math import pi, inf, sin, cos, tan, atan2, sqrt
import time

from cozmo.faces import Face
from cozmo.objects import LightCube, CustomObject
from cozmo.util import Pose

from . import evbase
from . import transform
from . import custom_objs
from .transform import wrap_angle, quat2rot, quaternion_to_euler_angle, get_orientation_state

import math
import numpy as np

class WorldObject():
    def __init__(self, id=None, x=0, y=0, z=0, is_visible=None):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.is_fixed = False   # True for walls and markers in predefined maps
        self.is_obstacle = True
        if is_visible is not None:
            self.is_visible = is_visible
        self.sdk_obj = None
        self.update_from_sdk = False
        self.is_foreign = False
        if is_visible:
            self.pose_confidence = +1
        else:
            self.pose_confidence = -1

class LightCubeObj(WorldObject):
    light_cube_size = (44., 44., 44.)
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0):
        if id is None:
            id = 'Cube-' + str(sdk_obj.cube_id)
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
        if sdk_obj:
            self.sdk_obj.wm_obj = self
            self.update_from_sdk = True
            self.orientation, _, _, self.theta = get_orientation_state(self.sdk_obj.pose.rotation.q0_q1_q2_q3)
        else:
            self.theta = theta
            self.orientation = transform.ORIENTATION_UPRIGHT
        self.size = self.light_cube_size

    @property
    def is_visible(self):
        return self.sdk_obj.is_visible

    def __repr__(self):
        if self.pose_confidence >= 0:
            vis = ' visible' if self.sdk_obj and self.is_visible else ''
            return '<LightCubeObj %s: (%.1f, %.1f, %.1f) @ %d deg.%s %s>' % \
                (self.id, self.x, self.y, self.z, self.theta*180/pi, vis, self.orientation)
        else:
            return '<LightCubeObj %s: position unknown>' % self.id


class ChargerObj(WorldObject):
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0):
        if id is None:
            id = 'Charger'
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
        if sdk_obj:
            self.sdk_obj.wm_obj = self
            self.update_from_sdk = True
        self.orientation = transform.ORIENTATION_UPRIGHT
        self.theta = theta
        self.size = (104, 98, 10)
        if self.sdk_obj and self.sdk_obj.pose:
            self.orientation, _, _, self.theta = get_orientation_state(self.sdk_obj.pose.rotation.q0_q1_q2_q3)

    @property
    def is_visible(self):
        return self.sdk_obj.is_visible

    def __repr__(self):
        if self.pose_confidence >= 0:
            vis = ' visible' if self.sdk_obj and self.is_visible else ''
            return '<ChargerObj: (%.1f, %.1f, %.1f) @ %d deg.%s %s>' % \
                (self.x, self.y, self.z, self.theta*180/pi, vis, self.orientation)
        else:
            return '<ChargerObj: position unknown>'


class CustomMarkerObj(WorldObject):
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0):
        if id is None:
            custom_type = sdk_obj.object_type.name[-2:]
            id = 'CustomMarkerObj-' + str(custom_type)
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
        self.marker_number = int(id[-2:])
        if self.sdk_obj:
            self.orientation, self.theta, _, _ = get_orientation_state(self.sdk_obj.pose.rotation.q0_q1_q2_q3, True)
        else:
            self.theta = theta
            self.orientation = transform.ORIENTATION_UPRIGHT

    @property
    def is_visible(self):
        return self.sdk_obj.is_visible

    def __repr__(self):
        if self.sdk_obj:
            vis = ' visible' if self.is_visible else ''
            return '<CustomMarkerObj-%s %d: (%.1f,%.1f)%s %s>' % \
                (self.sdk_obj.object_type.name[-2:], self.sdk_obj.object_id,
                 self.x, self.y, vis, self.orientation)
        else:
            return '<CustomMarkerObj %s (%.1f,%.1f) %s>' % \
                (self.id, self.x, self.y, self.orientation)


class CustomCubeObj(WorldObject):
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0, size=None):
        custom_type = sdk_obj.object_type.name[-2:]
        if id is None:
            id = 'CustomCubeObj-' + str(custom_type)
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
        self.update_from_sdk = True
        self.theta = theta
        self.custom_type = custom_type
        if (size is None) and isinstance(id, CustomObject):
            self.size = (id.x_size_mm, id.y_size_mm, id.z_size_mm)
        elif size:
            self.size = size
        else:
            self.size = (50., 50., 50.)

    @property
    def is_visible(self):
        return self.sdk_obj.is_visible

    def __repr__(self):
        vis = ' visible' if self.sdk_obj and self.is_visible else ''
        return '<CustomCubeObj-%s %d: (%.1f,%.1f, %.1f) @ %d deg.%s>' % \
               (self.sdk_obj.object_type.name[-2:], self.sdk_obj.object_id,
                self.x, self.y, self.z, self.theta*180/pi, vis)


class ArucoMarkerObj(WorldObject):
    def __init__(self, aruco_parent, marker_number, id=None, x=0, y=0, z=0, theta=0):
        if id is None:
            id = 'Aruco-' + str(marker_number)
        super().__init__(id,x,y,z)
        self.aruco_parent = aruco_parent
        self.marker_number = marker_number
        self.theta = theta
        self.pose_confidence = +1

    @property
    def is_visible(self):
        return self.marker_number in self.aruco_parent.seen_marker_ids

    def __repr__(self):
        if self.pose_confidence >= 0:
            vis = ' visible' if self.is_visible else ''
            fix = ' fixed' if self.is_fixed else ''
            return '<ArucoMarkerObj %d: (%.1f, %.1f, %.1f) @ %d deg.%s%s>' % \
                (self.marker_number, self.x, self.y, self.z, self.theta*180/pi, fix, vis)
        else:
            return '<ArucoMarkerObj %d: position unknown>' % self.marker_number


class WallObj(WorldObject):
    def __init__(self, id=None, x=0, y=0, theta=0, length=100, height=150,
                 door_width=75, door_height=105, marker_specs=dict(),
                 doorways=[], door_ids=[], is_foreign=False, is_fixed=False,
                 wall_spec=None, spec_id=None):
        if wall_spec:
            spec_id = wall_spec.spec_id
            length = wall_spec.length
            height = wall_spec.height
            door_width = wall_spec.door_width
            door_height = wall_spec.door_height
            marker_specs = wall_spec.marker_specs.copy()
            doorways = wall_spec.doorways.copy()
            door_ids = wall_spec.door_ids.copy()
        if id:
            self.wall_label = id[1+id.rfind('-'):]
        else:
            if len(marker_specs) > 0:
                k = list(marker_specs.keys())
                k.sort()
                self.wall_label = k[0][1+k[0].rfind('-'):]
                id = 'Wall-%s' % self.wall_label
            elif wall_spec and wall_spec.label:
                self.wall_label = wall_spec.label
                id = 'Wall-%s' % wall_spec.label
            else:
                raise ValueError('id (e.g., "A") must be supplied if wall has no markers')
        super().__init__(id, x, y)
        self.z = height/2
        self.theta = theta
        self.spec_id = spec_id
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height
        self.marker_specs = marker_specs
        self.doorways = doorways
        self.door_ids = door_ids
        self.is_foreign = is_foreign
        self.is_fixed = is_fixed
        self.pose_confidence = +1

    def update(self, world_map, x=0, y=0, theta=0):
        # Used instead of making new object for efficiency
        self.x = x
        self.y = y
        self.theta = theta

    def make_doorways(self, world_map):
        index = 0
        for index in range(len(self.doorways)):
            doorway = DoorwayObj(self, index)
            doorway.pose_confidence = +1
            world_map.objects[doorway.id] = doorway

    def make_arucos(self, world_map):
        "Called by add_fixed_landmark to make fixed aruco markers."
        for key,value in self.marker_specs.items():
            # Project marker onto the wall; move marker if it already exists
            marker = world_map.objects.get(key, None)
            if marker is None:
                marker_number = int(key[1+key.rfind('-'):])
                marker = ArucoMarkerObj(world_map.robot.world.aruco, marker_number=marker_number)
                world_map.objects[marker.id] = marker
            wall_xyz = transform.point(self.length/2 - value[1][0], 0, value[1][1])
            s = 0 if value[0] == +1 else pi
            rel_xyz = transform.aboutZ(self.theta+s).dot(wall_xyz)
            marker.x = self.x + rel_xyz[1][0]
            marker.y = self.y + rel_xyz[0][0]
            marker.z = rel_xyz[2][0]
            marker.theta = wrap_angle(self.theta + s)
            marker.is_fixed = self.is_fixed

    @property
    def is_visible(self):
        try:
            seen_marker_keys = [('Aruco-%d' % id) for id in evbase.robot_for_loading.world.aruco.seen_marker_ids]
        except:
            return True
        for m in self.marker_specs.keys():
            if m in seen_marker_keys:
                return True
        return False

    def __repr__(self):
        if self.pose_confidence >= 0:
            vis = ' visible' if self.is_visible else ''
            fix = ' fixed' if self.is_fixed else ''
            return '<WallObj %s: (%.1f,%.1f) @ %d deg. for %.1f%s%s>' % \
                (self.id, self.x, self.y, self.theta*180/pi, self.length, fix, vis)
        else:
            return '<WallObj %s: position unknown>' % self.id

class DoorwayObj(WorldObject):
    def __init__(self, wall, index):
        self.marker_ids = wall.door_ids[index]
        id = 'Doorway-' + str(self.marker_ids[0])
        super().__init__(id,0,0)
        self.theta = wall.theta
        self.wall = wall
        self.door_width = wall.door_width
        self.index = index  # which doorway is this?  0, 1, ...
        self.is_obstacle = False
        self.update()

    def update(self):
        bignum = 1e6
        self.theta = self.wall.theta
        m = max(-bignum, min(bignum, tan(self.theta+pi/2)))
        b = self.wall.y - m*self.wall.x
        dy =  (self.wall.length/2 - self.wall.doorways[self.index][0]) * cos(self.theta)
        self.y = self.wall.y - dy
        if abs(m) > 1/bignum:
            self.x = (self.y - b) / m
        else:
            self.x = self.wall.x
        self.pose_confidence = self.wall.pose_confidence

    def __repr__(self):
        if self.pose_confidence >= 0:
            return '<DoorwayObj %s: (%.1f,%.1f) @ %d deg.>' % \
                (self.id, self.x, self.y, self.theta*180/pi)
        else:
            return '<DoorwayObj %s: position unknown>' % self.id


class RoomObj(WorldObject):
    def __init__(self, name,
                 points=np.resize(np.array([0,0,0,1]),(4,4)).transpose(),
                 floor=1, door_ids=[], connections=[]):
        "points should be four points in homogeneous coordinates forming a convex polygon"
        id = 'Room-' + name
        self.name = name
        x,y,z,s = points.mean(1)
        super().__init__(id,x,y)
        self.points = points
        self.floor = floor
        self.door_ids = door_ids
        self.connections = connections
        self.is_obstacle = False
        self.is_fixed = True

    def __repr__(self):
        return '<RoomObj %s: (%.1f,%.1f) floor=%s>' % (self.id, self.x, self.y, self.floor)


class ChipObj(WorldObject):
    def __init__(self, id, x, y, z=0, radius=25/2, thickness=4):
        super().__init__(id,x,y,z)
        self.radius = radius
        self.thickness = thickness

    def __repr__(self):
        return '<ChipObj (%.1f,%.1f) radius %.1f>' % \
               (self.x, self.y, self.radius)

class FaceObj(WorldObject):
    def __init__(self, sdk_obj, id, x, y, z, name):
        super().__init__(id, x, y, z)
        self.sdk_obj = sdk_obj
        self.is_obstacle = False
        self.expression = 'unknown'

    @property
    def name(self):
        return self.sdk_obj.name

    @property
    def is_visible(self):
        return self.sdk_obj.is_visible

    def __repr__(self):
        return "<FaceObj name:'%s' expression:%s (%.1f, %.1f, %.1f) vis:%s>" % \
               (self.name, self.expression, self.x, self.y, self.z, self.is_visible)


class CameraObj(WorldObject):
    camera_size = (44., 44., 44.)
    def __init__(self, id=None, x=0, y=0, z=0, theta=0, phi = 0):
        super().__init__(id,x,y,z)
        self.size = self.camera_size
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi

    def update(self,x=0, y=0, z=0, theta = 0, phi = 0):
        # Used instead of making new object for efficiency
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi

    def __repr__(self):
        return '<CameraObj %d: (%.1f, %.1f, %.1f) @ %f.>\n' % \
               (self.id, self.x, self.y, self.z, self.phi*180/pi)

class RobotForeignObj(WorldObject):
    def __init__(self, cozmo_id=None, x=0, y=0, z=0, theta=0, camera_id = -1 ):
        super().__init__(id,x,y,z)
        self.cozmo_id = cozmo_id
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.size = (120., 90., 100.)
        self.camera_id = camera_id

    def __repr__(self):
        return '<RobotForeignObj %d: (%.1f, %.1f, %.1f) @ %f.> from camera %f\n' % \
               (self.cozmo_id, self.x, self.y, self.z, self.theta*180/pi, self.camera_id)

    def update(self, x=0, y=0, z=0, theta=0, camera_id=-1):
        # Used instead of making new object for efficiency
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.camera_id = camera_id


class LightCubeForeignObj(WorldObject):
    light_cube_size = (44., 44., 44.)
    def __init__(self, id=None, cozmo_id=None, x=0, y=0, z=0, theta=0, is_visible=False):
        super().__init__(id,x,y,z)
        self.theta = theta
        self.cozmo_id = cozmo_id
        self.size = self.light_cube_size
        self.is_visible = is_visible

    def __repr__(self):
        return '<LightCubeForeignObj %d: (%.1f, %.1f, %.1f) @ %d deg.> by cozmo %d \n' % \
               (self.id, self.x, self.y, self.z, self.theta*180/pi, self.cozmo_id)

    def update(self, x=0, y=0, z=0, theta=0):
        # Used instead of making new object for efficiency
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta


#================ WorldMap ================

class WorldMap():
    vision_z_fudge = 10  # Cozmo underestimates object z coord by about this much

    def __init__(self,robot):
        self.robot = robot
        self.objects = dict()
        self.shared_objects = dict()

    def add_fixed_landmark(self,landmark):
        landmark.is_fixed = True
        self.objects[landmark.id] = landmark
        if isinstance(landmark,WallObj):
            wall = landmark
            if wall.marker_specs:
                self.robot.world.particle_filter.add_fixed_landmark(landmark)
            wall.make_doorways(self)
            wall.make_arucos(self)
        else:
            self.robot.world.particle_filter.add_fixed_landmark(landmark)

    def delete_wall(self,wall_id):
        "Delete a wall, its markers, and its doorways, so we can predefine a new one."
        wall = self.objects.get(wall_id,None)
        if wall is None: return
        marker_ids = [('Aruco-'+str(id)) for id in wall.marker_specs.keys()]
        door_ids = [('Doorway-'+str(id)) for id in wall.door_ids]
        landmarks = self.robot.world.particle_filter.sensor_model.landmarks
        del self.objects[wall_id]
        if wall_id in landmarks:
            del landmarks[wall_id]
        for marker_id in marker_ids:
            if marker_id in self.objects:
                del self.objects[marker_id]
            if marker_id in landmarks:
                del landmarks[marker_id]
        for door_id in door_ids:
            if door_id in self.objects:
                del self.objects[door_id]

    def update_map(self):
        """Called to update the map after every camera image, after
        object_observed and object_moved events, and just before the
        path planner runs.
        """
        for (id,cube) in self.robot.world.light_cubes.items():
            self.update_cube(cube)
        if self.robot.world.charger: self.update_charger()
        for face in self.robot.world._faces.values():
            if face.face_id == face.updated_face_id:
                self.update_face(face)
            else:
                if face in self.robot.world.world_map.objects:
                    del  self.robot.world.world_map.objects[face]
        self.update_aruco_landmarks()
        self.update_walls()
        self.update_doorways()
        self.update_perched_cameras()

    def update_cube(self, cube):
        cube_id = 'Cube-' + str(cube.cube_id)
        if cube_id in self.objects:
            foreign_id = "LightCubeForeignObj-"+str(cube.cube_id)
            if foreign_id in self.objects:
                # remove foreign cube when local cube seen
                del self.objects[foreign_id]
            wmobject = self.objects[cube_id]
            wmobject.sdk_obj = cube  # In case created before seen
            if self.robot.carrying is wmobject:
                if cube.is_visible: # we thought we were carrying it, but we're wrong
                    self.robot.carrying = None
                    return self.update_cube(cube)
                else:  # we do appear to be carrying it
                    self.update_carried_object(wmobject)
        elif cube.pose is None:  # not in contact with cube
            return None
        else:
            # Cube is not in the worldmap, so add it.
            wmobject = LightCubeObj(cube)
            self.objects[cube_id] = wmobject
        if cube.is_visible:
            wmobject.update_from_sdk = True  # In case we've just dropped it; now we see it
            wmobject.pose_confidence = +1
        elif (cube.pose is None) or not cube.pose.is_comparable(self.robot.pose): # Robot picked up or cube moved
            wmobject.update_from_sdk = False
        else:       # Robot re-localized so cube came back
            pass  # skip for now due to SDK bug
            # wmobject.update_from_sdk = True
            # wmobject.pose_confidence = max(0, wmobject.pose_confidence)
        if wmobject.update_from_sdk:  # True unless if we've dropped it and haven't seen it yet
            self.update_coords_from_sdk(wmobject, cube)
            wmobject.orientation, _, _, wmobject.theta = get_orientation_state(cube.pose.rotation.q0_q1_q2_q3)
        return wmobject

    def update_charger(self):
        charger = self.robot.world.charger
        if charger is None: return
        charger_id = 'Charger'
        wmobject = self.objects.get(charger_id, None)
        if wmobject is None:
            wmobject = ChargerObj(charger)
            self.objects[charger_id] = wmobject
        wmobject.sdk_obj = charger  # In case we created charger before seeing it
        if self.robot.is_on_charger:
            wmobject.update_from_sdk = False
            theta = wrap_angle(self.robot.world.particle_filter.pose[2] + pi)
            charger_offset = np.array([[-30], [0], [0], [1]])
            offset = transform.aboutZ(theta).dot(charger_offset)
            wmobject.x = self.robot.world.particle_filter.pose[0] + offset[0,0]
            wmobject.y = self.robot.world.particle_filter.pose[1] + offset[1,0]
            wmobject.theta = theta
            wmobject.pose_confidence = +1
        elif charger.is_visible:
            wmobject.update_from_sdk = True
            wmobject.pose_confidence = +1
        elif (charger.pose is None) or not charger.pose.is_comparable(self.robot.pose):
            wmobject.update_from_sdk = False
            wmobject.pose_confidence = -1
        else:       # Robot re-localized so charger came back
            pass  # skip for now due to SDK bug
            # wmobject.update_from_sdk = True
            # wmobject.pose_confidence = max(0, wmobject.pose_confidence)
        if wmobject.update_from_sdk:  # True unless pose isn't comparable
            self.update_coords_from_sdk(wmobject, charger)
            wmobject.orientation, _, _, wmobject.theta = get_orientation_state(charger.pose.rotation.q0_q1_q2_q3)
        return wmobject

    def update_aruco_landmarks(self):
        try:
            seen_marker_objects = self.robot.world.aruco.seen_marker_objects.copy()
        except:
            return
        aruco_parent = self.robot.world.aruco
        for (key,value) in seen_marker_objects.items():
            marker_id = value.id_string
            wmobject = self.objects.get(marker_id, None)
            if wmobject is None:
                # TODO: wait to see marker several times before adding.
                wmobject = ArucoMarkerObj(aruco_parent,key)  # coordinates will be filled in below
                self.objects[marker_id] = wmobject
                landmark_spec = None
            else:
                landmark_spec = self.robot.world.particle_filter.sensor_model.landmarks.get(marker_id, None)
            wmobject.pose_confidence = +1
            if isinstance(landmark_spec, tuple):  # Particle filter is tracking this marker
                wmobject.x = landmark_spec[0][0][0]
                wmobject.y = landmark_spec[0][1][0]
                wmobject.theta = landmark_spec[1]
                elevation = atan2(value.camera_coords[1], value.camera_coords[2])
                cam_pos = transform.point(0,
                                          value.camera_distance * sin(elevation),
                                          value.camera_distance * cos(elevation))
                base_pos = self.robot.kine.joint_to_base('camera').dot(cam_pos)
                wmobject.z = base_pos[2,0]
                wmobject.elevation = elevation
                wmobject.cam_pos = cam_pos
                wmobject.base_pos = base_pos
            elif isinstance(landmark_spec, Pose):  # Hard-coded landmark pose for laboratory exercises
                wmobject.x = landmark_spec.position.x
                wmobject.y = landmark_spec.position.y
                wmobject.theta = landmark_spec.rotation.angle_z.radians
                wmobject.is_fixed = True
            else:
                # Non-landmark: convert aruco sensor values to pf coordinates and update
                elevation = atan2(value.camera_coords[1], value.camera_coords[2])
                cam_pos = transform.point(0,
                                          value.camera_distance * sin(elevation),
                                          value.camera_distance * cos(elevation))
                base_pos = self.robot.kine.joint_to_base('camera').dot(cam_pos)
                wmobject.x = base_pos[0,0]
                wmobject.y = base_pos[1,0]
                wmobject.z = base_pos[2,0]
                wmobject.theta = wrap_angle(self.robot.world.particle_filter.pose[2] +
                                            value.euler_rotation[1]*(pi/180) + pi)
                wmobject.elevation = elevation
                wmobject.cam_pos = cam_pos
                wmobject.base_pos = base_pos

    def update_walls(self):
        for key, value in self.robot.world.particle_filter.sensor_model.landmarks.items():
            if key.startswith('Wall-'):
                if key in self.objects:
                    wall = self.objects[key]
                    if not wall.is_fixed and not wall.is_foreign:
                        wall.update(self,x=value[0][0][0], y=value[0][1][0], theta=value[1])
                else:
                    print('Creating new wall in worldmap:',key)
                    wall_spec = wall_marker_dict[key]
                    wall = WallObj(id=key,
                                   x=value[0][0][0],
                                   y=value[0][1][0],
                                   theta=value[1],
                                   length=wall_spec.length,
                                   height=wall_spec.height,
                                   door_width=wall_spec.door_width,
                                   door_height=wall_spec.door_height,
                                   marker_specs=wall_spec.marker_specs,
                                   doorways=wall_spec.doorways,
                                   door_ids=wall_spec.door_ids,
                                   is_foreign=False,
                                   spec_id=key)
                    self.objects[key] = wall
                    wall.pose_confidence = +1
                    # Make the doorways
                    wall.make_doorways(self.robot.world.world_map)
                # Relocate the aruco markers to their predefined positions
                spec = wall_marker_dict.get(wall.id, None)
                if spec is None: return
                for key,value in spec.marker_specs.items():
                    if key in self.robot.world.world_map.objects:
                        aruco_marker = self.robot.world.world_map.objects[key]
                        dir = value[0]    # +1 for front side or -1 for back side
                        s = 0 if dir == +1 else pi
                        aruco_marker.theta = wrap_angle(wall.theta + s)
                        wall_xyz = transform.point(-dir*(wall.length/2 - value[1][0]), 0, value[1][1])
                        rel_xyz = transform.aboutZ(aruco_marker.theta + pi/2).dot(wall_xyz)
                        aruco_marker.x = wall.x + rel_xyz[0][0]
                        aruco_marker.y = wall.y + rel_xyz[1][0]
                        aruco_marker.z = rel_xyz[2][0]
                        aruco_marker.is_fixed = wall.is_fixed

    def update_doorways(self):
        for key,value in self.robot.world.world_map.objects.items():
            if key.startswith('Doorway'):
                value.update()

    def lookup_face_obj(self,face):
        "Look up face by name, not by Face instance."
        for (key,value) in self.robot.world.world_map.objects.items():
            if isinstance(value, FaceObj) and value.name == face.name:
                if value.sdk_obj is not face and face.is_visible:
                    # Older Face object with same name: replace it with new one
                    value.sdk_obj = face
                    value.id = face.face_id
                return value
        return None

    def update_face(self,face):
        if face.pose is None:
            return
        pos = face.pose.position
        face_obj = self.lookup_face_obj(face)
        if face_obj is None:
            face_obj = FaceObj(face, face.face_id, pos.x, pos.y, pos.z,
                               face.name)
            if len(face.name) == 0:
                key = 'Face:unknown'
            else:
                key = 'Face:' + face.name
            self.robot.world.world_map.objects[key] = face_obj
        else:
            face_obj.sdk_obj = face  # in case face.updated_id changed
        # now update the face
        if face.is_visible:
            face_obj.x = pos.x
            face_obj.y = pos.y
            face_obj.z = pos.z
            face_obj.expression = face.expression
            self.update_coords_from_sdk(face_obj, face)

    def update_custom_object(self, sdk_obj):
        if not sdk_obj.pose.is_comparable(self.robot.pose):
            print('Should never get here:',sdk_obj.pose,self.robot.pose)
            return
        id = 'CustomMarkerObj-' + str(sdk_obj.object_type.name[-2:])
        if id in self.objects:
            wmobject = self.objects[id]
            wmobject.sdk_obj = sdk_obj  # In case created marker before seeing it
        else:
            type = sdk_obj.object_type
            if type in custom_objs.custom_marker_types:
                wmobject = CustomMarkerObj(sdk_obj,id)
            elif type in custom_objs.custom_cube_types:
                wmobject = CustomCubeObj(sdk_obj,id)
            self.objects[id] = wmobject
        wmobject.pose_confidence = +1
        self.update_coords_from_sdk(wmobject, sdk_obj)
        if isinstance(wmobject, CustomMarkerObj):
            wmobject.orientation, wmobject.theta, _, _ = get_orientation_state(sdk_obj.pose.rotation.q0_q1_q2_q3, True)
        elif isinstance(wmobject, CustomCubeObj):
            wmobject.orientation, _, _, wmobject.theta = get_orientation_state(sdk_obj.pose.rotation.q0_q1_q2_q3)

    def update_carried_object(self, wmobject):
        #print('Updating carried object ',wmobject)
        # set x,y based on robot's pose
        # need to cache initial orientation relative to robot:
        #   grasped_orient = wmobject.theta - robot.pose.rotation.angle_z
        world_frame = self.robot.kine.joints['world']
        lift_attach_frame = self.robot.kine.joints['lift_attach']
        tmat = self.robot.kine.base_to_link(world_frame).dot(self.robot.kine.joint_to_base(lift_attach_frame))
        # *** HACK *** : depth calculation only works for cubes; need to handle custom obj, chips
        half_depth = wmobject.size[0] / 2
        new_pose = tmat.dot(transform.point(half_depth,0))
        theta = self.robot.world.particle_filter.pose[2]
        wmobject.x = new_pose[0,0]
        wmobject.y = new_pose[1,0]
        wmobject.z = new_pose[2,0]
        wmobject.theta = theta

    def update_coords_from_sdk(self, wmobject, sdk_obj):
        dx = sdk_obj.pose.position.x - self.robot.pose.position.x
        dy = sdk_obj.pose.position.y - self.robot.pose.position.y
        alpha = atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians
        r = sqrt(dx*dx + dy*dy)
        (rob_x,rob_y,rob_theta) = self.robot.world.particle_filter.pose
        wmobject.x = rob_x + r * cos(alpha + rob_theta)
        wmobject.y = rob_y + r * sin(alpha + rob_theta)
        wmobject.z = sdk_obj.pose.position.z
        orient_diff = wrap_angle(rob_theta - self.robot.pose.rotation.angle_z.radians)
        wmobject.theta = wrap_angle(sdk_obj.pose.rotation.angle_z.radians + orient_diff)

    def update_perched_cameras(self):
        if self.robot.world.server.started:
            pool = self.robot.world.server.camera_landmark_pool
            for key, val in pool.get(self.robot.aruco_id,{}).items():
                if key.startswith('Video'):
                    if key in self.objects:
                        self.objects[key].update(x=val[0][0,0], y=val[0][1,0], z=val[1][0],
                                                 theta=val[1][2], phi=val[1][1])
                    else:
                        # last digit of capture id as camera key
                        self.objects[key] = \
                            CameraObj(id=int(key[-2]), x=val[0][0,0], y=val[0][1,0],
                                      z=val[1][0], theta=val[1][2], phi=val[1][1])
        else:
            for key, val in self.robot.world.particle_filter.sensor_model.landmarks.items():
                if key.startswith('Video'):
                    if key in self.objects:
                        self.objects[key].update(x=val[0][0,0], y=val[0][1,0], z=val[1][0],
                                                 theta=val[1][2], phi=val[1][1])
                    else:
                        # last digit of capture id as camera key
                        self.objects[key] = \
                            CameraObj(id=int(key[-2]), x=val[0][0,0], y=val[0][1,0],
                                      z=val[1][0], theta=val[1][2], phi=val[1][1])

    def invalidate_poses(self):
        # *** This medthod is not currently used. ***
        for wmobj in self.robot.world.world_map.objects.values():
            if not wmobj.is_fixed:
                wmobj.pose_confidence = -1

    def show_objects(self):
        objs = self.objects
        print('%d object%s in the world map:' %
              (len(objs), '' if len(objs) == 1 else 's'))
        basics = ['Charger', 'Cube-1', 'Cube-2', 'Cube-3']
        ordered_keys = []
        for key in basics:
            if key in objs:
                ordered_keys.append(key)
        customs = []
        arucos = []
        walls = []
        misc = []
        for (key,value) in objs.items():
            if key in basics:
                pass
            elif isinstance(value, CustomMarkerObj):
                customs.append(key)
            elif isinstance(value, ArucoMarkerObj):
                arucos.append(key)
            elif isinstance(value, WallObj):
                walls.append(key)
            else:
                misc.append(key)
        arucos.sort()
        walls.sort()
        ordered_keys = ordered_keys + customs + arucos + walls + misc
        for key in ordered_keys:
            print('  ', objs[key])
        print()

#================ Event Handlers ================

    def handle_object_observed(self, evt, **kwargs):
        if isinstance(evt.obj, LightCube):
            # print('observed: ',evt.obj)
            self.update_cube(evt.obj)
        elif isinstance(evt.obj, CustomObject):
            self.update_custom_object(evt.obj)
        elif isinstance(evt.obj, Face):
            self.update_face(evt.obj)

    def handle_object_move_started(self, evt, **kwargs):
        cube = evt.obj
        if self.robot.carrying and self.robot.carrying.sdk_obj is cube:
            return
        if self.robot.fetching and self.robot.fetching.sdk_obj is cube:
            return
        cube.movement_start_time = time.time()
        cube_id = 'Cube-' + str(cube.cube_id)
        wmobject = self.robot.world.world_map.objects[cube_id]
        wmobject.pose_confidence = min(0, wmobject.pose_confidence)

    def handle_object_move_stopped(self, evt, **kwargs):
        cube = evt.obj
        cube.movement_start_time = None

#================ Wall Specification  ================

# WallSpec is used in wall_defs.py

wall_marker_dict = dict()

class WallSpec():
    def __init__(self, label=None, length=100, height=210, door_width=77, door_height=105,
                 marker_specs=dict(), doorways=[], door_ids=[]):
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height
        self.marker_specs = marker_specs
        self.doorways = doorways
        self.door_ids = door_ids
        marker_id_numbers = [int(marker_id[1+marker_id.rfind('-'):]) for marker_id in marker_specs.keys()]
        if label and len(marker_id_numbers) == 0:
            self.spec_id = 'Wall-' + label  # 'Wall-A' for 'A'
            label = 'Wall-' + label
        elif len(marker_id_numbers) > 0 and not label:
            lowest_marker_id = 'Aruco-%d' % min(marker_id_numbers)
            self.spec_id = 'Wall-%d' % min(marker_id_numbers)
            label = self.spec_id  # 'Wall-37' for 'Aruco-37'
        else:
            raise ValueError("Don't know how to label wall '%s'" % label)
        self.label = label
        global wall_marker_dict
        for id in marker_specs.keys():
            wall_marker_dict[id] = self
        wall_marker_dict[label] = self
