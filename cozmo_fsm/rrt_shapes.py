from cozmo_fsm import transform
from math import sqrt, pi, atan2
import numpy as np

class Shape():
    def __init__(self):
        self.center = transform.point()
        self.rotmat = transform.identity()
        self.obstacle = None
    
    def __repr__(self):
        return "<%s >" % (self.__class__.__name__)

    def collides(self, shape):
        if isinstance(shape, Rectangle):
            return self.collides_rect(shape)
        elif isinstance(shape, Polygon):
            return self.collides_poly(shape)
        elif isinstance(shape, Circle):
            return self.collides_circle(shape)
        elif isinstance(shape, Compound):
            return shape.collides(self)
        else:
            raise Exception("%s has no collides() method defined for %s." % (self, shape))

#================ Basic Shapes ================

class Circle(Shape):
    def __init__(self, center=transform.point(), radius=25/2):
        super().__init__()
        self.center = center
        self.radius = radius
        self.orient = 0.

    def __repr__(self):
        return '<Circle (%.1f,%.1f) r=%.1f>' % \
               (self.center[0,0], self.center[1,0], self.radius)

    def instantiate(self, tmat):
        return Circle(center=tmat.dot(self.center), radius=self.radius)        

    def collides_rect(self,rect):
        return rect.collides_circle(self)
        
    def collides_poly(self,poly):
        return poly.collides(self)

    def collides_circle(self,circle):
        dx = self.center[0,0] - circle.center[0,0]
        dy = self.center[1,0] - circle.center[1,0]
        dist = sqrt(dx*dx + dy*dy)
        return dist < (self.radius + circle.radius)
        
class Polygon(Shape):
    def __init__(self, vertices=None):
      self.vertices = vertices
      N = vertices.shape[1]
      self.edges = tuple( (vertices[:,i:i+1], vertices[:,(i+1)%N:((i+1)%N)+1])
                          for i in range(N) )
      center = vertices.mean(1).resize(4,1)

    def collides_poly(poly): pass

    def collides_circle(circle):
        raise ValueError()

class Rectangle(Polygon):
    def __init__(self, center=None, dimensions=None, orient=0):
        self.center = center
        self.dimensions = dimensions
        self.orient = orient
        dx2 = dimensions[0]/2
        dy2 = dimensions[1]/2
        vertices = np.array([[-dx2,  dx2, dx2, -dx2 ],
                             [-dy2, -dy2, dy2,  dy2 ],
                             [  0,    0,   0,    0  ],
                             [  1,    1,   1,    1  ]])
        self.unrot = transform.aboutZ(-orient)
        center_ex = self.unrot.dot(center)
        extents = transform.translate(center_ex[0],center_ex[1]).dot(vertices)
        # Extents measured along the rectangle's axes, not world axes
        self.min_Ex = min(extents[0,:])
        self.max_Ex = max(extents[0,:])
        self.min_Ey = min(extents[1,:])
        self.max_Ey = max(extents[1,:])
        world_vertices = transform.aboutZ(orient).dot(vertices)
        world_vertices = transform.translate(center[0],center[1]).dot(world_vertices)
        super().__init__(vertices=world_vertices)

    def __repr__(self):
        return '<Rectangle (%.1f,%.1f) %.1fx%.1f %.1f deg>' % \
               (self.center[0,0],self.center[1,0],*self.dimensions,
                self.orient*(180/pi))

    def instantiate(self, tmat):
        dimensions = (self.max_Ex-self.min_Ex, self.max_Ey-self.min_Ey)
        rot = atan2(tmat[1,0], tmat[0,0])
        return Rectangle(center = tmat.dot(self.center),
                         orient = rot + self.orient,
                         dimensions = dimensions)

    def collides_rect(self,other):
        # Test others edges in our reference frame
        o_verts = self.unrot.dot(other.vertices)
        o_min_x = min(o_verts[0,:])
        o_max_x = max(o_verts[0,:])
        o_min_y = min(o_verts[1,:])
        o_max_y = max(o_verts[1,:])
        if o_max_x <= self.min_Ex or self.max_Ex <= o_min_x or \
               o_max_y <= self.min_Ey or self.max_Ey <= o_min_y:
            return False

        if self.orient == other.orient: return True

        # Test our edges in other's reference frame
        s_verts = other.unrot.dot(self.vertices)
        s_min_x = min(s_verts[0,:])
        s_max_x = max(s_verts[0,:])
        s_min_y = min(s_verts[1,:])
        s_max_y = max(s_verts[1,:])
        if s_max_x <= other.min_Ex or other.max_Ex <= s_min_x or  \
               s_max_y <= other.min_Ey or other.max_Ey <= s_min_y:
            return False
        return True
            
    def collides_circle(self,circle):
        p = self.unrot.dot(circle.center)[0:2,0]
        pmin = p - circle.radius
        pmax = p + circle.radius
        if pmax[0] <= self.min_Ex or self.max_Ex <= pmin[0] or \
           pmax[1] <= self.min_Ey or self.max_Ey <= pmin[1]:
            return False
        # Need corner tests here
        return True

#================ Compound Shapes ================

class Compound(Shape):
    def __init__(self, shapes=[]):
        self.shapes = shapes

    def collides(self,shape):
        for s in self.shapes:
            if s.collides(shape):
                return True
        return False

