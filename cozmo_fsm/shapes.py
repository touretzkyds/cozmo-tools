import transform

class Shape():
    def __init__(self):
        self.center = transform.point()
        self.rotmat = transform.identity()
    
    def _repr_(self):
        return "<%s >" % (self.__class.__name)

    def collides(self, shape):
        if isinstance(shape, Polyon):
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

    def instantiate(self, tmat, x, y, q):
        return Circle(center=transform.translate(x,y).dot(tmat.dot(self.center)), radius=self.radius)        

    def collides_poly(poly):
        return poly.collides(self)

    def collides_circle(circle):
        dist = transform.norm(self.center - circle.center)
        return dist < (self.radius + circle.radius)
        
class Rectangle(Polygon):
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

