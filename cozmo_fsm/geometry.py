"""
Geometry calculations, including transformation matrices for
kinematics.

"""

import numpy as np
from math import sin, cos, tan, pi, atan2, asin, sqrt, floor, ceil
from fractions import Fraction
import copy

def point(x=0,y=0,z=0):
    return np.array([ [x], [y], [z], [1.] ])

def norm(pt):
    return pt[0][0:3].norm()

def aboutX(theta):
    c = cos(theta)
    s = sin(theta)
    return np.array([
        [ 1,  0,  0, 0],
        [ 0,  c, -s, 0],
        [ 0,  s,  c, 0],
        [ 0,  0,  0, 1]])

def aboutY(theta):
    c = cos(theta)
    s = sin(theta)
    return np.array([
        [ c,  0,  s, 0],
        [ 0,  1,  0, 0],
        [-s,  0,  c, 0],
        [ 0,  0,  0, 1]])

def aboutZ(theta):
    c = cos(theta)
    s = sin(theta)
    return np.array([
        [ c, -s,  0, 0],
        [ s,  c,  0, 0],
        [ 0,  0,  1, 0],
        [ 0,  0,  0, 1.]])

def translate(x,y,z=0):
    return np.array([
        [ 1, 0, 0, x],
        [ 0, 1, 0, y],
        [ 0, 0, 1, z],
        [ 0, 0, 0, 1.]])

def normalize(v):
    s = v[3,0]
    if s == 0:
        return v
    else:
        return v/s

def identity():
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1.]])

def dh_matrix(d,theta,r,alpha):
    """Denavit-Hartenberg transformation from joint i to joint i+1."""
    return aboutX(alpha).dot(translate(r,0,d).dot(aboutZ(theta)))

def translation(t):
    return np.array([ [t[0,3]], [t[1,3]], [t[2,3]], [t[3,3]] ])

def wrap_angle(angle_rads):
    """Keep angle between -pi and pi."""
    if isinstance(angle_rads, np.ndarray):
        raise ValueError("Argument not a scalar: %s", angle_rads)
    while angle_rads <= -pi:
        angle_rads += 2*pi
    while angle_rads > pi:
        angle_rads -= 2*pi
    return angle_rads

def wrap_selected_angles(angle_rads, index):
    """Keep angle between -pi and pi for column vector of angles"""
    for i in index:
       angle_rads[i,0] =  wrap_angle(angle_rads[i,0])
    return angle_rads

def tprint(t):
    number_format = "%7.3f"
    def tprint_vector(t):
        for i in range(t.shape[0]):
            if i == 0:
                print('[ ',end='')
            else:
                print('  ',end='')
            print(number_format % t[i],end='')
            if i+1 == t.shape[0]:
                print(' ]')
            else:
                print()
    def tprint_matrix(t):
        for i in range(t.shape[0]):
            if i == 0:
                print('[ ',end='')
            else:
                print('  ',end='')
            for j in range(t.shape[1]):
                if j>0: print('  ',end='')
                print(number_format % t[i][j], end='')
            if i+1 == t.shape[0]:
                print(' ]')
            else:
                print()
    if isinstance(t, np.ndarray) and t.ndim == 1:
        tprint_vector(t)
    elif isinstance(t, np.ndarray) and t.ndim == 2:
        tprint_matrix(t)
    elif isinstance(t, (int,float)):
        print(number_format % t)
    else:
        print(t)

def rotate_point(point, center, angle):
    pointX, pointY = point
    centerX, centerY = center
    rotatedX = cos(angle) * (pointX - centerX) - sin(angle) * (pointY-centerY) + centerX
    rotatedY = sin(angle) * (pointX - centerX) + cos(angle) * (pointY - centerY) + centerY
    return rotatedX, rotatedY

#---------------- Quaternions ----------------

def quat2rot(q0,q1,q2,q3):
    # formula from http://stackoverflow.com/questions/7938373/from-quaternions-to-opengl-rotations
    q0_sq = q0*q0; q1_sq = q1*q1; q2_sq = q2*q2; q3_sq = q3*q3
    t_q0q1 = 2. * q0 * q1
    t_q0q2 = 2. * q0 * q2
    t_q0q3 = 2. * q0 * q3
    t_q1q2 = 2. * q1 * q2
    t_q1q3 = 2. * q1 * q3
    t_q2q3 = 2. * q2 * q3
    return np.array([
        [ q0_sq+q1_sq-q2_sq-q3_sq, t_q1q2-t_q0q3,           t_q1q3+t_q0q2,           0. ],
        [ t_q1q2+t_q0q3,           q0_sq-q1_sq+q2_sq-q3_sq, t_q2q3-t_q0q1,           0. ],
        [ t_q1q3-t_q0q2,           t_q2q3+t_q0q1,           q0_sq-q1_sq-q2_sq+q3_sq, 0. ],
        [             0.,                     0.,                      0.,           1.  ]])

def quat2rot33(q0,q1,q2,q3):
    # formula from http://stackoverflow.com/questions/7938373/from-quaternions-to-opengl-rotations
    q0_sq = q0*q0; q1_sq = q1*q1; q2_sq = q2*q2; q3_sq = q3*q3
    t_q0q1 = 2. * q0 * q1
    t_q0q2 = 2. * q0 * q2
    t_q0q3 = 2. * q0 * q3
    t_q1q2 = 2. * q1 * q2
    t_q1q3 = 2. * q1 * q3
    t_q2q3 = 2. * q2 * q3
    return np.array([
        [ q0_sq+q1_sq-q2_sq-q3_sq, t_q1q2-t_q0q3,           t_q1q3+t_q0q2,         ],
        [ t_q1q2+t_q0q3,           q0_sq-q1_sq+q2_sq-q3_sq, t_q2q3-t_q0q1,         ],
        [ t_q1q3-t_q0q2,           t_q2q3+t_q0q1,           q0_sq-q1_sq-q2_sq+q3_sq]])


def quaternion_to_euler_angle(quaternion):
    # source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    w, x, y, z = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = atan2(t3, t4)

    return X, Y, Z


#---------------- Orientation state from quaternion ----------------

ORIENTATION_UPRIGHT = 'upright'
ORIENTATION_INVERTED = 'inverted'
ORIENTATION_SIDEWAYS = 'sideways'
ORIENTATION_TILTED = 'tilted'
ORIENTATION_LEFT = 'left'
ORIENTATION_RIGHT = 'right'

def get_orientation_state(quaternion, isPlanar=False):
    """Utility used by light cubes, charger, and custom markers."""
    q0, q1, q2, q3 = quaternion
    mat_arr = quat2rot(q0, q1, q2, q3)
    z_vec = np.array([0, 0, 1, 1])
    z_dot = mat_arr.dot(z_vec)[:3]
    dot_product = np.round(z_dot.dot(np.array([0, 0, 1])), decimals=2)
    x, y, z = quaternion_to_euler_angle(quaternion)
    if isPlanar:
        perpendicular = True if -0.5 < y < 0.5 else False
        if not perpendicular:
            dot_product = np.round(z_dot.dot(np.array([1, 0, 0])), decimals=2)
            x, y, z = quaternion_to_euler_angle([q0, q2, q3, q1])
            x = -y if x>0 else y+pi
            x = x if x < pi else (x - 2*pi)
    if dot_product >= 0.9:
        orientation = ORIENTATION_UPRIGHT
    elif dot_product <= -0.9:
        orientation = ORIENTATION_INVERTED
        z -= pi
    elif -0.15 <= dot_product <= 0.15:
        if isPlanar:
            # Markers
            if 0 < x < pi:
                orientation = ORIENTATION_RIGHT
            else:
                orientation = ORIENTATION_LEFT
        else:
            # Cubes
            isSideways = abs(y) < 0.2 or abs(abs(y)-pi/2) < 0.2
            orientation = ORIENTATION_SIDEWAYS if isSideways else ORIENTATION_TILTED
            if round(y, 1) == 0:
                z = z-pi/2 if x>0 else z+pi/2
            else:
                w, x, y, z = quaternion
                x, y, z = quaternion_to_euler_angle([w, y, x, z])
                z = -y if x>0 else y+pi
    else:
        orientation = ORIENTATION_TILTED

    return orientation, x, y, wrap_angle(z)

def same_orientation(old_object, new_object):
    q1 = old_object.pose.rotation.q0_q1_q2_q3
    q2 = new_object.pose.rotation.q0_q1_q2_q3
    old_orientation, _, _, _ = get_orientation_state(q1)
    new_orientation, _, _, _ = get_orientation_state(q2)
    if old_orientation != new_orientation:
        return False
    elif old_orientation == ORIENTATION_SIDEWAYS:
        old_pattern_number = get_pattern_number(old_object.pose.rotation.euler_angles)
        new_pattern_number = get_pattern_number(new_object.pose.rotation.euler_angles)
        if old_pattern_number == new_pattern_number:
            return True
        else:
            return False
    else:
        return True

def get_pattern_number(eulerAngles):
    x, y, z = eulerAngles
    pattern = -1
    z = min([pi/2, 0, -pi/2], key=lambda val:abs(val-z))
    if z == -pi/2:
        pattern = 1
    elif z == pi/2:
        pattern = 3
    else:
        if min([0, -pi, pi], key=lambda val:abs(val-x)) == 0:
            pattern = 2
        else:
            pattern = 4
    return pattern


#---------------- General Geometric Calculations ----------------

def project_to_line(x0,y0,theta0,x1,y1):
    """Returns the projection of the point (x1,y1) onto the
    line through (x0,y0) with orientation theta0."""
    bigvalue = 1e6
    m0 = max(-bigvalue, min(bigvalue, tan(theta0)))
    if abs(m0) < 1/bigvalue:
        return (x1,y0)
    m1 = -1 / m0
    b0 = y0 - m0*x0
    b1 = y1 - m1*x1
    x2 = (b0-b1) / (m1-m0)
    y2 = m0 * x2 + b0
    return (x2,y2)

def line_equation(p1, p2):
    "Returns the line equation used by line_intersection."
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return (A, B, -C)

def line_extrapolate(L, x):
    (A,B,C) = L
    s = +1 if B > 0 else -1
    return C if B == 0 else (-A/B)*x + C*s

def line_intersection(L1,L2):
    "Intersection point of two lines defined by line equations"
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    if D == 0: return False
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    x = Dx / D
    y = Dy / D
    return (x,y)

def segment_intersect_test(p1, p2, p3, p4):
    """Returns True if the line segment from p1 to p2
    intersects the line segment from p3 to p4. Formula from
    http://www.cs.swan.ac.uk/~cssimon/line_intersection.html"""
    (x1,y1) = p1
    (x2,y2) = p2
    (x3,y3) = p3
    (x4,y4) = p4
    denom = (x4-x3)*(y1-y2) - (x1-x2)*(y4-y3)
    if abs(denom) < 0.0001:
        return False
    numa = (y3-y4)*(x1-x3) + (x4-x3)*(y1-y3)
    numb = (y1-y2)*(x1-x3) + (x2-x1)*(y1-y3)
    ta = numa / denom
    tb = numb / denom
    if (0 <= ta <= 1) and (0 <= tb <= 1):
        return True
    else:
        return False


def rotation_matrix_to_euler_angles(R):
    "Input R is a 3x3 rotation matrix."
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


def polygon_fill(polygon, offset):
    """
    Implement the scanline polygon fill algorithm
    Input a polygon (rrt shape) and return points inside the polygon
    """
    class Edge:
        def __init__(self, ymax, x, sign, dx, dy, sum):
            self.ymax = ymax
            self.xval = x
            self.sign = sign
            self.dx = dx
            self.dy = dy
            self.sum = sum
        def __repr__(self):
            return '<Edge (ymax= %s, xval= %s, sign= %s, dx= %s, dy= %s, sum= %s )>' % \
                   (self.ymax, self.xval, self.sign, self.dx, self.dy, self.sum)

    [xCenter, yCenter, _, _] = polygon.vertices.mean(1)
    edges = polygon.edges
    ((xmin,ymin), (xmax,ymax)) = polygon.get_bounding_box()
    xmin, ymin, xmax, ymax = floor(xmin), floor(ymin), ceil(xmax), ceil(ymax)
    xdelta = abs(xmin) if xmin < 0 else 0
    xmin += xdelta
    xmax += xdelta
    xCenter += xdelta
    ydelta = abs(ymin) if ymin < 0 else 0
    ymin += ydelta
    ymax += ydelta
    yCenter += ydelta
    edge_table = [[] for i in range(ymax+1)]
    active_list, points = [], []

    for edge in edges:
        ([[p1x], [p1y], _, _], [[p2x], [p2y], _, _]) = edge
        if (p1y-p2y) != 0:  # Don't need to consider horizontal edges
            p1x, p1y, p2x, p2y = p1x+xdelta, p1y+ydelta, p2x+xdelta, p2y+ydelta
            end_points = [[p1x, p1y], [p2x, p2y]]
            end_points = sorted(end_points, key = lambda pt: pt[1])   # Sort on y value
            _xval, _ymin, _ymax = int(round(end_points[0][0])), int(round(end_points[0][1])), int(round(end_points[1][1]))
            slope = Fraction((p1x-p2x)/(p1y-p2y)).limit_denominator(10)
            _dx = slope.numerator
            _dy = slope.denominator
            _sign = 1 if (_dx > 0) == (_dy > 0) else -1
            _edge = Edge(_ymax, _xval, _sign, abs(_dx), abs(_dy), 0)
            edge_table[_ymin].append(_edge)

    for scanline in range(ymin, ymax+1):
        # Add match (ymin==scanline) edges to the active_list
        if len(edge_table[scanline]) > 0:
            for edge in edge_table[scanline]:
                active_list.append(edge)
        if len(active_list) > 0:
            y_lower_bound = (ymin - offset) if (offset < 0) else (yCenter - offset)
            y_upper_bound = (ymax + offset) if (offset < 0) else (yCenter + offset)
            if y_lower_bound < scanline < y_upper_bound:
                # Sort active_list on x value; if same x value, sort on slope (1/m)
                active_list = sorted(active_list, key = lambda x: (x.xval, x.sign*x.dx/x.dy))
                for _x in range(active_list[0].xval, active_list[1].xval):
                    x_lower_bound = (active_list[0].xval - offset) if (offset < 0) else (xCenter - offset)
                    x_upper_bound = (active_list[1].xval + offset) if (offset < 0) else (xCenter + offset)
                    if x_lower_bound < _x < x_upper_bound:
                        points.append([_x-xdelta, scanline-ydelta])
        if len(active_list) > 3:
            y_lower_bound = (ymin - offset) if (offset < 0) else (yCenter - offset)
            y_upper_bound = (ymax + offset) if (offset < 0) else (yCenter + offset)
            if y_lower_bound < scanline < y_upper_bound:
                for _x in range(active_list[2].xval, active_list[3].xval):
                    x_lower_bound = (active_list[2].xval - offset) if (offset < 0) else (xCenter - offset)
                    x_upper_bound = (active_list[3].xval + offset) if (offset < 0) else (xCenter + offset)
                    if x_lower_bound < _x < x_upper_bound:
                        points.append([_x-xdelta, scanline-ydelta])
        # Remove form active_list if edge.ymax = scanline
        active_list = [edge for edge in active_list if scanline < edge.ymax]
        # Increase x-value
        for edge in active_list:
            # Add dx to sum
            edge.sum += edge.dx
            # While sum ≥ dy, adjust x, subtract dy from sum
            while edge.sum >= edge.dy:
                edge.xval += edge.sign
                edge.sum -= edge.dy

    return points

def check_concave(polygon):
    """
    Input a polygon (rrt shape)
    Return a boolean(is concave or not) and
           a list of triangle vertices divided from the concave polygon
    """
    # Currently only works for quadrilateral
    vertices = np.transpose(polygon.vertices).tolist()
    edges = [[p1x-p2x, p1y-p2y] for [[p1x], [p1y], _, _], [[p2x], [p2y], _, _] in polygon.edges]
    crossProducts = [np.cross(edges[i], edges[i-1]) > 0 for i in range(len(edges))]
    if all(crossProducts) or not any(crossProducts):
        return False, None
    else:
        trues = [i for i in range(len(crossProducts)) if crossProducts[i] == True]
        falses = [i for i in range(len(crossProducts)) if crossProducts[i] == False]
        idx = trues[0] if len(trues) < len(falses) else falses[0]
        vertices += vertices
        tri1 = vertices[:][idx:idx+3]
        tri2 = vertices[:][idx+2:idx+5]
        return True, [np.transpose(tri1), np.transpose(tri2)]
