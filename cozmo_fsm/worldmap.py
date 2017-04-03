from math import pi, inf, sin, cos, atan2, sqrt

from .transform import wrap_angle

class Wall():
    def __init__(self, id=0, x=0, y=0, theta=0, length=100):
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta
        self.length = length

    def __repr__(self):
        return '<Wall %d: (%.1f,%.1f) @ %d deg. for %.1f>' % \
               (self.id, self.x, self.y, self.theta*180/pi, self.length)
        
class LightCubeObst():
    def __init__(self, id, x, y, theta):
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta
        self.size = (44., 44., 44.)

    def __repr__(self):
        return '<LightCubeObst %d: (%.1f,%.1f) @ %d deg.>' % \
               (self.id, self.x, self.y, self.theta)

#================ WorldMap ================

class WorldMap():
    def __init__(self,robot):
        self.robot = robot
        self.objects = []
        
    def generate_map(self):
        self.generate_walls()
        self.add_cubes()

    def generate_walls(self):
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
        # Delete any pre-existing versions of these walls
        for obj in self.objects.copy():
            if isinstance(obj,Wall) and obj.id in seen_markers:
                self.objects.remove(obj)
        # Now infer the walls from the markers
        for (id,markers) in seen_markers.items():
            self.objects.append(self.infer_wall(id,markers))

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
            return Wall(id=wall_spec.id, x=wall_x, y=wall_y, theta=wall_orient,
                        length=wall_spec.length)
        
    def add_cubes(self):
        for (id,cube) in self.robot.world.light_cubes.items():
            if cube.is_visible:
                diff = cube.pose - self.robot.pose
                (dx,dy,_) = diff.position.x_y_z
                dist = sqrt(dx*dx + dy*dy)
                bearing = atan2(dy,dx)
                (rob_x,rob_y,rob_theta) = self.robot.world.particle_filter.pose
                world_bearing = wrap_angle(rob_theta + bearing)
                world_x = rob_x + dist * cos(world_bearing)
                world_y = rob_y + dist * sin(world_bearing)
                world_orient = rob_theta + diff.rotation.angle_z.radians
                self.objects.append(LightCubeObst(id, world_x, world_y, world_orient))
            

#================ Actual Walls ================

wall_marker_dict = dict()

class WallSpec():
    def __init__(self, length=100, markers={}, doorways=[]):
        self.length = length
        self.markers = markers
        self.doorways = doorways
        ids = list(markers.keys())
        self.id = min(ids)
        global wall_marker_dict
        for id in ids:
            wall_marker_dict[id] = self

def make_walls():
    w1 = WallSpec(length=620,
                  markers={ 53 : (+1, ( 70.,50.)),  # +1 = front markers
                            65 : (+1, (235.,50.)),
                            59 : (+1, (375.,50.)),
                            55 : (+1, (544.,50.)),
                            45 : (-1, ( 70.,50.)),  # -1 = back markers
                            64 : (-1, (238.,50.)),
                            37 : (-1, (380.,50.)),
                            67 : (-1, (544.,50.)) },
                  doorways = [ (160., 77.), (470., 77.) ])  # (center, width)
    w2 = WallSpec(length=620,
                  markers={ 41 : (+1, ( 70.,50.)),  # +1 = front markers
                            56 : (+1, (235.,50.)),
                            80 : (+1, (375.,50.)),
                            68 : (+1, (544.,50.)),
                            44 : (-1, ( 70.,50.)),  # -1 = back markers
                            62 : (-1, (238.,50.)),
                            38 : (-1, (380.,50.)),
                            50 : (-1, (544.,50.)) },
                  doorways = [ (160., 77.), (470., 77.) ])  # (center, width)
    w3 = WallSpec(length=215,
                  markers={ 0 : (+1, ( 45.,50.)),
                            1 : (+1, (145.,50.)) },
                  doorways = [ (105., 77.) ])  # (center, width)

make_walls()
