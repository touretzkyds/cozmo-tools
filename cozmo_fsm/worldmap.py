from math import pi, inf, sin, cos, atan2
from .rrt_shapes import *

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
    w2 = WallSpec(length=215,
                  markers={ 0 : (+1, ( 45.,50.)),
                            1 : (+1, (145.,50.)) },
                  doorways = [ (105., 77.) ])  # (center, width)

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
        
class WorldMap():
    def __init__(self,robot):
        self.robot = robot
        self.objects = []
        
    def generate_map(self):
        landmarks = self.robot.world.particle_filter.sensor_model.landmarks
        seen_markers = dict()
        # Distribute markers to wall ids
        for (id,spec) in landmarks.items():
            wall_spec = wall_marker_dict.get(id,None)
            if wall_spec is None: continue
            wall_id = wall_spec.id
            markers = seen_markers.get(wall_id, list())
            markers.append((id,spec))
            seen_markers[wall_id] = markers
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
        
    def generate_wall_obstacle(self,wall):
        wall_spec = wall_marker_dict[wall.id]
        half_length = wall.length / 2
        widths = []
        last_x = -half_length
        edges = [ [0, -half_length, 0., 1.] ]
        for (center,width) in wall_spec.doorways:
            left_edge = center-width/2 - half_length
            edges.append([0., left_edge, 0., 1.])
            widths.append(left_edge - last_x)
            right_edge = center + width/2 - half_length
            edges.append([0., right_edge, 0., 1.])
            last_x = right_edge
        edges.append([0., half_length, 0., 1.])
        widths.append(half_length-last_x)
        edges = np.array(edges).T
        edges = transform.aboutZ(wall.theta).dot(edges)
        edges = transform.translate(wall.x,wall.y).dot(edges)
        transform.tprint(edges)
        obst = []
        for i in range(0,len(widths)):
            center = edges[:,2*i:2*i+1].mean(1).reshape(4,1)
            dimensions=(4.0, widths[i])
            r = Rectangle(center=center,
                          dimensions=dimensions,
                          orient=wall.theta )
            print(r)
            obst.append(r)
        return obst
            

    def generate_obstacles(self):
        result = []
        for obj in self.objects:
            if isinstance(obj, Wall):
                result = result + self.generate_wall_obstacle(obj)
        return result

make_walls()
