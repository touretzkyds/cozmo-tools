from math import pi, inf, sin, cos, atan2, sqrt
from cozmo.objects import CustomObject, LightCube

from .transform import wrap_angle

class WorldObject():
    def __init__(self, id=None, x=0, y=0, z=0, is_visible=False):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.obstacle = True
        self.is_visible = is_visible

class WallObj(WorldObject):
    def __init__(self, id=None, x=0, y=0, theta=0, length=100, height=150,
                 door_width=75, door_height=105):
        super().__init__(id,x,y)
        self.z = height/2
        self.theta = theta
        self.length = length
        self.height = height
        self.door_width = door_width
        self.door_height = door_height

    def __repr__(self):
        return '<WallObj %d: (%.1f,%.1f) @ %d deg. for %.1f>' % \
               (self.id, self.x, self.y, self.theta*180/pi, self.length)
        
class LightCubeObj(WorldObject):
    light_cube_size = (44., 44., 44.)
    def __init__(self, sdk_obj, id=None, x=0, y=0, z=0, theta=0):
        super().__init__(id,x,y,z)
        self.sdk_obj = sdk_obj
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
        self.name = name
        self.obstacle = False

    def __repr__(self):
        return "<Face '%s' %s (%.1f, %.1f, %.1f)>" % \
               (self.name, self.expression, self.x, self.y, self.z)

class CameraObj(WorldObject):
    camera_size = (44., 44., 44.)
    def __init__(self, id=None, x=0, y=0, z=0, theta=0, phi =0, initial_position =(0,0)):
        super().__init__(id,x,y,z)
        self.size = self.camera_size
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi
        self.initial_position = initial_position
        #self.is_visible = sdk_obj.is_visible

    def __repr__(self):
        return '<CameraObj %d: (%.1f, %.1f, %.1f).>' % \
               (self.id, self.x, self.y, self.z)

class RobotGhostObj(WorldObject):
    def __init__(self, camera_id=None, cozmo_id=None, x=0, y=0, z=0, theta=0, is_visible=True):
        super().__init__(id,x,y,z)
        self.camera_id = camera_id
        self.cozmo_id = cozmo_id
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.is_visible = is_visible

    def __repr__(self):
        return '<RobotGhostObj %d: (%.1f, %.1f, %.1f).>' % \
               (self.id, self.x, self.y, self.z)

    def update(self, x=0, y=0, z=0, theta=0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

class LightCubeGhostObj(WorldObject):
    light_cube_size = (44., 44., 44.)
    def __init__(self, id=None, x=0, y=0, z=0, theta=0):
        super().__init__(id,x,y,z)
        self.theta = theta
        self.size = self.light_cube_size

    def __repr__(self):
        return '<LightCubeGhostObj %d: (%.1f, %.1f, %.1f) @ %d deg.>' % \
               (self.id, self.x, self.y, self.z, self.theta*180/pi)

#================ WorldMap ================

class WorldMap():
    vision_z_fudge = 10  # Cozmo underestimates object z coord by about this much

    def __init__(self,robot):
        self.robot = robot
        self.objects = dict()
        
    def update_map(self):
        """Called to update the map just before the path planner runs.
        Cubes and faces are updated automatically in reponse to observation events, but
        we update them here to get the freshest possible value.  Walls must be
        generated fresh as they have no observation events."""
        self.generate_walls_from_markers()
        for (id,cube) in self.robot.world.light_cubes.items():
            self.update_cube(cube)
        for face in self.robot.world._faces.values():
            self.update_face(face)

    def generate_walls_from_markers(self):
        landmarks = self.robot.world.particle_filter.sensor_model.landmarks
        seen_markers = dict()
        # Distribute markers to wall ids
        for (id,spec) in landmarks.items():
            wall_spec = wall_marker_dict.get(id,None)
            if wall_spec is None: continue  # marker not part of a known wall
            wall_id = wall_spec.id
            markers = seen_markers.get(wall_id, list())
            markers.append((id,spec))
            seen_markers[wall_id] = markers
        # Now infer the walls from the markers
        for (id,markers) in seen_markers.items():
            self.objects[id] = self.infer_wall(id,markers)

    def infer_wall(self,id,markers):
        # Just use one marker for now; should really do least squares fit
        for (m_id, m_spec) in markers:
            wall_spec = wall_marker_dict.get(m_id,None)
            if wall_spec is None: continue  # spurious marker
            (m_mu, m_orient, m_sigma) = m_spec
            m_x = m_mu[0,0]
            m_y = m_mu[1,0]
            dist = wall_spec.length/2 - wall_spec.markers[m_id][1][0]
            wall_orient = m_orient # simple for now
            wall_x = m_x + dist*cos(wall_orient-pi/2)
            wall_y = m_y + dist*sin(wall_orient-pi/2)
            return WallObj(id=wall_spec.id, x=wall_x, y=wall_y, theta=wall_orient,
                           length=wall_spec.length, height=wall_spec.height,
                           door_height=wall_spec.door_height)
        
    def update_cube(self, cube):
        if cube.pose is None or not cube.pose.is_comparable(self.robot.pose):
            return
        if cube in self.objects:
            world_obj = self.objects[cube]
        else:
            id = tuple(key for (key,value) in self.robot.world.light_cubes.items() if value == cube)[0]
            world_obj = LightCubeObj(cube, id)
            self.objects[cube] = world_obj
        self.update_coords(world_obj, cube)

    def update_face(self,face):
        pos = face.pose.position
        if face in self.robot.world.world_map.objects:
            face_obj = self.robot.world.world_map.objects[face]
        else:
            face_obj = FaceObj(face, face.face_id, pos.x, pos.y, pos.z,
                               face.name)
            self.robot.world.world_map.objects[face] = face_obj
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
        elif isinstance(evt.obj, cozmo.faces.Face):
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

