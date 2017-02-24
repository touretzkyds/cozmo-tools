"""
Transformation matrices for kinematics calculations.
"""

import numpy as np
from math import sin, cos, pi

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

def tprint(t):
    number_format = "%7.3f"
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
    if isinstance(t, np.ndarray):
        tprint_matrix(t)
    elif isinstance(t, (int,float)):
        print(number_format % t)
    else:
        print(t)

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
