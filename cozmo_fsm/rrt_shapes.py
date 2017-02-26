from cozmo_fsm import transform
from math import sqrt

class Shape():
    def __init__(self):
        self.center = transform.point()
        self.rotmat = transform.identity()
    
    def __repr__(self):
        return "<%s >" % (self.__class.__name)

    def collides(self, shape):
        if isinstance(shape, Polygon):
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

    def __repr__(self):
        return '<Circle (%s,%s) r=%s>' % \
               ( ('%5.1f' % self.center[0,0]).strip(),
                 ('%5.1f' % self.center[1,0]).strip(),
                 ('%5.1f' % self.radius).strip() )

    def instantiate(self, tmat):
        return Circle(center=tmat.dot(self.center), radius=self.radius)        

    def collides_poly(self,poly):
        return poly.collides(self)

    def collides_circle(self,circle):
        dx = self.center[0,0] - circle.center[0,0]
        dy = self.center[1,0] - circle.center[1,0]
        dist = sqrt(dx*dx + dy*dy)
        return dist < (self.radius + circle.radius)
        
class Rectangle(Shape):
    def __init__(self, corners=None, dimensions=None, orient=0):
        if corners:
            pt0 = pt1 = corners[0]
            pt2 = pt3 = corners[1]
            # fix pt1 and pt3 ...
        elif dimensions:
            pt0 = transform.point(0,0,0).dot(aboutZ(orient))
            pt1 = transform.point(dimensions[0], 0, 0).dot(aboutZ(orient))
            pt2 = transform.point(dimensions[0], dimensions[1], 0).dot(aboutZ(orient))
            pt3 = transform.point(0, dimensions[1], 0).dot(aboutZ(orient))
        super().__init__(vertices=(pt0,pt1,pt2,pt3))

class Polygon(Shape):
    def __init__(self, vertices=None):
      self.vertices = tuple(vertices)
      N = len(vertices)
      self.edges = tuple( (vertices[i], vertices[(i+1) % N]) for i in range(N) )
      center_x = mean(v[0,0] for v in vertices)
      center_y = mean(v[1,0] for v in vertices)
      center_z = mean(v[2,0] for v in vertices)
      self.center = transform.point(center_x, center_y, center_z, 1.0)

    def collides_poly(poly): pass

    def collides_circle(circle): pass


#================ Compound Shapes ================

class Compound(Shape):
    def __init__(self, shapes=[]):
        self.shapes = shapes

    def collides(self,shape):
        for s in self.shapes:
            if s.collides(shape):
                return True
        return False

