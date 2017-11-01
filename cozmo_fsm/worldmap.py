from math import pi, inf, sin, cos, atan2, sqrt
from cozmo.faces import Face
from cozmo.objects import CustomObject, LightCube

from . import transform
from .transform import wrap_angle

class WorldObject():
    def __init__(self, id=None, x=0, y=0, z=0, is_visible=False):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.obstacle = True
        self.is_visible = is_visible
        self.update_from_sdk = False

class WallObj(WorldObject):
    def __init__(self, id=None, x=0, y=0, theta=0, length=100, height=150,
                 door_width=75, door_height=105, markers=[]):
        super().__init__(id,x,y)
        self.z = height/2
        self.theta = theta
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height
        self.markers = markers

    def __repr__(self):
        return '<WallObj %d: (%.1f,%.1f) @ %d deg. for %.1f>' % \
               (self.id, self.x, self.y, self.theta*180/pi, self.length)

class MarkerObj(WorldObject):
    def __init__(self, id=None, x=0, y=0):
        super().__init__(id,x,y)

    def __repr__(self):
        return '<MarkerObj %d: (%.1f,%.1f)' % \
               (self.id, self.x, self.y)
        
class LightCubeObj(WorldObject):
    light_cube_size = (44., 44., 44.)
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0):
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
        self.update_from_sdk = True
        self.theta = theta
        self.size = self.light_cube_size
        self.is_visible = sdk_obj.is_visible

    def __repr__(self):
        return '<LightCubeObj %d: (%.1f, %.1f, %.1f) @ %d deg.>' % \
               (self.id, self.x, self.y, self.z, self.theta*180/pi)

class CustomCubeObj(WorldObject):
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0, size=None):
        # id is a CustomObjecType
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
        self.update_from_sdk = True
        self.theta = theta
        if (size is None) and isinstance(id, CustomObject):
            self.size = (id.x_size_mm, id.y_size_mm, id.z_size_mm)
        elif size:
            self.size = size
        else:
            self.size = (50., 50., 50.)
        self.is_visible = sdk_obj.is_visible

    def __repr__(self):
        return '<CustomCubeObj %s: (%.1f,%.1f, %.1f) @ %d deg.>' % \
               (self.sdk_obj.object_type, self.x, self.y, self.z, self.theta*180/pi)

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
        super().__init__(id, x, y, z, is_visible=sdk_obj.is_visible)
        self.sdk_obj = sdk_obj
        self.obstacle = False

    # Faces can be renamed, so check the Face object
    @property
    def name(self):
        return self.sdk_obj.name

    def __repr__(self):
        return "<FaceObj name:'%s' expr:%s (%.1f, %.1f, %.1f) vis:%s>" % \
               (self.name, self.expression, self.x, self.y, self.z, self.sdk_obj.is_visible)

class CameraObj(WorldObject):
    camera_size = (44., 44., 44.)
    def __init__(self, name='NULL', id=None, x=0, y=0, z=0, theta=0, phi = 0, rotm = None, tvecs = None, calibration_number= None):
        super().__init__(id,x,y,z)
        self.size = self.camera_size
        self.id = id
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi
        self.rotm = rotm
        self.tvecs = tvecs
        self.calibration_number=calibration_number

    def __repr__(self):
        return '<CameraObj %d: (%.1f, %.1f, %.1f) @ %f.> Calibrated on %d\n' % \
               (self.id, self.x, self.y, self.z, self.phi*180/3.14, self.calibration_number)


class WallGhostObj(WorldObject):
    def __init__(self, id=None, x=0, y=0, theta=0, length=100, height=150,
                 door_width=75, door_height=105, markers=[],calibration_number = 1):
        super().__init__(id,x,y)
        self.z = height/2
        self.theta = theta
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height
        self.markers = markers
        self.calibration_number = calibration_number

    def __repr__(self):
        return '<WallGhostObj %d: (%.1f,%.1f) @ %d deg. for %.1f> Calibrated on %d\n' % \
               (self.id, self.x, self.y, self.theta*180/pi, self.length, self.calibration_number)

class RobotGhostObj(WorldObject):
    def __init__(self, camera_id=None, name = 'NULL', cozmo_id=None, x=0, y=0, z=0, theta=0, is_visible=True, uncertainity=0, calibration_number = 1):
        super().__init__(id,x,y,z)
        self.camera_id = camera_id
        self.name = name
        self.cozmo_id = cozmo_id
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.uncertainity = uncertainity
        self.is_visible = is_visible
        self.calibration_number = calibration_number
        self.size = (120., 70., 100.)

    def __repr__(self):
        return '<RobotGhostObj %s-%d: (%.1f, %.1f, %.1f) @ %f.> Calibrated on %d \n' % \
               (self.name, self.cozmo_id, self.x, self.y, self.z, self.theta*180/3.14, self.calibration_number)

    def update(self, x=0, y=0, z=0, theta=0, uncertainity=0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.uncertainity = uncertainity

class LightCubeGhostObj(WorldObject):
    light_cube_size = (44., 44., 44.)
    def __init__(self, id=None, cozmo_id=None, x=0, y=0, z=0, theta=0, is_visible= False):
        super().__init__(id,x,y,z)
        self.theta = theta
        self.cozmo_id = cozmo_id
        self.size = self.light_cube_size
        self.is_visible = is_visible

    def __repr__(self):
        return '<LightCubeGhostObj %d: (%.1f, %.1f, %.1f) @ %d deg.>\n' % \
               (self.id, self.x, self.y, self.z, self.theta*180/pi)

    def update(self, x=0, y=0, z=0, theta=0, distance=0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.distance = distance

#================ WorldMap ================

class WorldMap():
    vision_z_fudge = 10  # Cozmo underestimates object z coord by about this much

    def __init__(self,robot):
        self.robot = robot
        self.objects = dict()
        self.shared_objects = dict()
        self.temp_cams = dict()
        self.temp_cubes = dict()
        self.temp_walls = dict()
        self.temp_ghosts = dict(dict())
        
    def update_map(self):
        """Called to update the map just before the path planner runs.  Cubes,
        custom objects, and faces are updated automatically in reponse
        to observation events, but we update them here to get the
        freshest possible value.  Walls must be generated fresh as
        they have no observation events."""
        self.generate_walls_from_markers()
        for (id,cube) in self.robot.world.light_cubes.items():
            self.update_cube(cube)
        for face in self.robot.world._faces.values():
            self.update_face(face)

    def generate_walls_from_markers(self):
        for key, value in self.robot.world.particle_filter.sensor_model.landmarks.items():
            if isinstance(key,str):
                id = int(key[-(len(key)-5):])
                wall_spec = wall_marker_dict[id]
                self.objects[key] = WallObj(id, x=value[0][0][0], y=value[0][1][0], theta=value[1], length=wall_spec.length, height=wall_spec.height,
                 door_width=wall_spec.door_width, door_height=wall_spec.door_height, markers=wall_spec.markers)
            
    def update_cube(self, cube):
        if cube in self.objects:
            world_obj = self.objects[cube]
            if self.robot.carrying is world_obj:
                self.update_carried_object(world_obj)
                return
        elif cube.pose is None or not cube.pose.is_comparable(self.robot.pose):
            return
        else:
            id = tuple(key for (key,value) in self.robot.world.light_cubes.items() if value == cube)[0]
            world_obj = LightCubeObj(cube, id)
            self.objects[cube] = world_obj
        if cube.is_visible:
            world_obj.update_from_sdk = True  # In case we've dropped it; now we see it
        if world_obj.update_from_sdk:  # True unless if we've dropped it and haven't seen it yet
            self.update_coords(world_obj, cube)

    def lookup_face_obj(self,face):
        "Look up face by name, not by Face instance."
        for (key,value) in self.robot.world.world_map.objects.items():
            if isinstance(key, Face) and key.name == face.name:
                if key is not face and face.is_visible:
                    # Older Face object with same name: replace it with new one
                    self.robot.world.world_map.objects.pop(key)
                    self.robot.world.world_map.objects[face] = value
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
            self.robot.world.world_map.objects[face] = face_obj
        # now update the face
        face_obj.is_visible = face.is_visible
        if face.is_visible:
            face_obj.x = pos.x
            face_obj.y = pos.y
            face_obj.z = pos.z
            face_obj.expression = face.expression
            self.update_coords(face_obj, face)

    def update_custom_object(self, sdk_obj):
        if not sdk_obj.pose.is_comparable(self.robot.pose):
            return
        if sdk_obj in self.objects:
            world_obj = self.objects[sdk_obj]
        else:
            id = sdk_obj.object_type
            world_obj = CustomCubeObj(sdk_obj,id)
            self.objects[sdk_obj] = world_obj
        self.update_coords(world_obj, sdk_obj)

    def update_carried_object(self, world_obj):
        #print('Updating carried object ',world_obj)
        # set x,y based on robot's pose
        # need to cache initial orientation relative to robot:
        #   grasped_orient = world_obj.theta - robot.pose.rotation.angle_z
        world_frame = self.robot.kine.joints['world']
        lift_attach_frame = self.robot.kine.joints['lift_attach']
        tmat = self.robot.kine.base_to_link(world_frame).dot(self.robot.kine.joint_to_base(lift_attach_frame))
        # *** HACK *** : width calculation only works for cubes; need to handle custom obj, chips
        half_width = 22 # world_obj.size[0] / 2
        new_pose = tmat.dot(transform.point(half_width,0))
        theta = self.robot.world.particle_filter.pose[2]
        world_obj.x = new_pose[0,0]
        world_obj.y = new_pose[1,0]
        world_obj.theta = theta

    def update_coords(self, world_obj, sdk_obj):
        dx = sdk_obj.pose.position.x - self.robot.pose.position.x
        dy = sdk_obj.pose.position.y - self.robot.pose.position.y
        alpha = atan2(dy,dx) - self.robot.pose.rotation.angle_z.radians
        r = sqrt(dx*dx + dy*dy)
        (rob_x,rob_y,rob_theta) = self.robot.world.particle_filter.pose
        world_obj.x = rob_x + r * cos(alpha + rob_theta)
        world_obj.y = rob_y + r * sin(alpha + rob_theta)
        world_obj.z = sdk_obj.pose.position.z
        orient_diff = wrap_angle(rob_theta - self.robot.pose.rotation.angle_z.radians)
        world_obj.theta = wrap_angle(sdk_obj.pose.rotation.angle_z.radians + orient_diff)
        world_obj.is_visible = sdk_obj.is_visible

    def handle_object_observed(self, evt, **kwargs):
        if isinstance(evt.obj, LightCube):
            self.update_cube(evt.obj)
        elif isinstance(evt.obj, CustomObject):
            self.update_custom_object(evt.obj)
        elif isinstance(evt.obj, Face):
            self.update_face(evt.obj)

#================ Wall Specification  ================

wall_marker_dict = dict()

class WallSpec():
    def __init__(self, length=100, height=210, door_width=75, door_height=105,
                 markers={}, doorways=[]):
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height
        self.markers = markers
        self.doorways = doorways
        ids = list(markers.keys())
        self.id = min(ids)
        global wall_marker_dict
        for id in ids:
            wall_marker_dict[id] = self
