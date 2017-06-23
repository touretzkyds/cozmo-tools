from math import pi, inf, sin, cos, atan2, sqrt
from cozmo.objects import CustomObject

from .transform import wrap_angle

class WorldObject():
    def __init__(self, id=0, x=0, y=0, z=0):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.obstacle = True

class WallObj(WorldObject):
    def __init__(self, id=0, x=0, y=0, theta=0, length=100, height=150,
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
    def __init__(self, id, x, y, z, theta):
        super().__init__(id,x,y,z)
        self.theta = theta
        self.size = self.light_cube_size

    def __repr__(self):
        return '<LightCubeObj %d: (%.1f, %.1f, %.1f) @ %d deg.>' % \
               (self.id, self.x, self.y, self.z, self.theta*180/pi)

class CustomCubeObj(WorldObject):
    def __init__(self, id, x, y, z, theta, size=None):
        # id is a CustomObjecType
        super().__init__(id,x,y,z)
        self.type = type
        self.theta = theta
        if (size is None) and isinstance(id, CustomObject):
            self.size = (id.x_size_mm, id.y_size_mm, id.z_size_mm)
        elif size:
            self.size = size
        else:
            self.size = (50., 50., 50.)

    def __repr__(self):
        return '<CustomCubeObj %s: (%.1f,%.1f, %.1f) @ %d deg.>' % \
               (self.type, self.x, self.y, self.z, self.theta*180/pi)

class ChipObj(WorldObject):
    def __init__(self, id, x, y, z=0, radius=25/2, thickness=4):
        super().__init__(id,x,y)
        self.radius = radius
        self.thickness = thickness

    def __repr__(self):
        return '<ChipObj (%.1f,%.1f) radius %.1f>' % \
               (self.x, self.y, self.radius)

#================ WorldMap ================

class WorldMap():
    vision_z_fudge = 10  # Cozmo underestimates object z coord by about this much

    def __init__(self,robot):
        self.robot = robot
        self.objects = dict()
        
    def update_map(self):
        self.generate_walls_from_markers()
        self.add_cubes()

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
        
    def add_cubes(self):
        for (id,cube) in self.robot.world.light_cubes.items():
            if cube.pose.is_comparable(self.robot.pose):
                diff = cube.pose - self.robot.pose
                (dx,dy,_) = diff.position.x_y_z
                (rob_x,rob_y,rob_theta) = self.robot.world.particle_filter.pose
                orient_diff = wrap_angle(rob_theta - self.robot.pose.rotation.angle_z.radians)
                osin = sin(orient_diff)
                ocos = cos(orient_diff)
                wx = dx * ocos + dy * osin
                wy = dx * -osin + dy * ocos
                world_x = rob_x + wx
                world_y = rob_y + wy
                world_z = cube.pose.position.z + self.vision_z_fudge
                world_orient = wrap_angle(cube.pose.rotation.angle_z.radians + orient_diff)
                self.objects[cube] = LightCubeObj(id, world_x, world_y, world_z, world_orient)
            
    def update_object(self,obj):
        if isinstance(obj, CustomObject):
            diff = obj.pose - self.robot.pose
            (dx,dy,_) = diff.position.x_y_z
            dist = sqrt(dx*dx + dy*dy)
            bearing = atan2(dy,dx)
            (rob_x,rob_y,rob_theta) = self.robot.world.particle_filter.pose
            world_x = rob_x + dist * cos(bearing)
            world_y = rob_y + dist * sin(bearing)
            world_z = obj.pose.position.z + self.vision_z_fudge
            world_orient = rob_theta + diff.rotation.angle_z.radians
            self.objects[obj] = CustomCubeObj(obj, world_x, world_y, world_z, world_orient)

    def handle_object_observed(self, evt, **kwargs):
        if isinstance(evt.obj, CustomObject):
            self.update_object(evt.obj)

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

