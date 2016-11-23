'''

OpenGL viewer for Cozmo's world map.
====================================

Run it by typing:  viewer(robot)

Keyboard commands:
  a          Translate left
  d          Translate right
  w          Translate forward
  s          Translate backward
  page-up    Translate up
  page-down  Translate down

  left-arrow   Rotate left
  right-arrow  Rotate right
  up-arrow     Rotate upward
  down-arrow   Rotate downward

  z          Reset view

  
Author:  David S. Touretzky, Carnegie Mellon University
=======

Change Log:
===========

* Created.

To Do:
======
    * Add charger object
    * Add user-generated barriers
    * Different appearance for each light cube to tell them apart
    * Add the robot's lift
    * Replace cubes and robot body with texture-mapped images
    * Add a floor to the world

'''

from math import sin, cos, radians
import sys
import threading
import array

try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    print("Can't find required OpenGL package.  Do 'pip3 install PyOpenGL'")
    raise

import cozmo

exited = False

initial_window_size = (500, 500)

cube_vertices = array.array('f', \
    [-0.5, -0.5, +0.5, \
     -0.5, +0.5, +0.5, \
     +0.5, +0.5, +0.5, \
     +0.5, -0.5, +0.5, \
     -0.5, -0.5, -0.5, \
     -0.5, +0.5, -0.5, \
     +0.5, +0.5, -0.5, \
     +0.5, -0.5, -0.5 ])

cube_colors_0 = array.array('f', \
    [0.8, 0.0, 0.0, \
     0.8, 0.0, 0.0, \
     0.0, 0.8, 0.0, \
     0.0, 0.8, 0.0, \
     0.0, 0.0, 0.8, \
     0.0, 0.0, 0.8, \
     0.5, 0.5, 0.5, \
     0.5, 0.5, 0.5 ])

cube_colors_1 = array.array('f', \
    [0.8, 0.8, 0.0, \
     0.8, 0.8, 0.0, \
     0.0, 0.8, 0.8, \
     0.0, 0.8, 0.8, \
     0.8, 0.0, 0.8, \
     0.8, 0.0, 0.8, \
     0.9, 0.9, 0.9, \
     0.9, 0.9, 0.9 ])
cube_cIndices = array.array('B', \
    [0, 3, 2, 1, \
     2, 3, 7, 6, \
     0, 4, 7, 3, \
     1, 2, 6, 5, \
     4, 5, 6, 7, \
     0, 1, 5, 4 ])

light_cube_size_mm = 44.3
robot_body_size_mm = (56, 30, 70)
robot_head_size_mm = (39.4, 39, 36)
robot_head_offset_mm = (20, 0, 38)
wscale = 0.02  # millimeters to graphics coordinates

def make_cube(xsize=1, ysize=1, zsize=1, vis=False):
    """Make a cube centered on the origin"""
    glEnableClientState(GL_COLOR_ARRAY)
    glEnableClientState(GL_VERTEX_ARRAY)
    if vis:
        glColorPointer(3, GL_FLOAT, 0, cube_colors_1.tostring())
    else:
        glColorPointer(3, GL_FLOAT, 0, cube_colors_0.tostring())
    verts = cube_vertices * 1;
    for i in range(0,24,3):
        verts[i  ] *= xsize * wscale
        verts[i+1] *= ysize * wscale
        verts[i+2] *= zsize * wscale
    glVertexPointer(3, GL_FLOAT, 0, verts.tostring())
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tostring())
    glDisableClientState(GL_COLOR_ARRAY)
    glDisableClientState(GL_VERTEX_ARRAY)

def make_light_cube(lcube):
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    p = lcube.pose.position.x_y_z
    if p != (0.0, 0.0, 0.0):
        glPushMatrix()
        glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
        glRotatef(lcube.pose.rotation.angle_z.degrees, 0, 1, 0)
        s = light_cube_size_mm
        make_cube(s, s, s, lcube.is_visible)
        glPopMatrix()
    glEndList()
    return c

def make_cozmo_robot():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    p = robot.pose.position.x_y_z
    glPushMatrix()
    glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
    glRotatef(robot.pose.rotation.angle_z.degrees, 0, 1, 0)
    make_cube(*robot_body_size_mm)
    h = robot_head_offset_mm
    glTranslatef(-h[1]*wscale, h[2]*wscale, -h[0]*wscale)
    glRotatef(robot.head_angle.degrees, 1, 0, 0)
    make_cube(*robot_head_size_mm)
    glPopMatrix()
    glEndList()
    return c

def make_shapes():
    global cube1, cube2, cube3, cozmo_robot
    # cube 1
    cube1 = make_light_cube(robot.world.light_cubes[1])
    cube2 = make_light_cube(robot.world.light_cubes[2])
    cube3 = make_light_cube(robot.world.light_cubes[3])
    # cozmo robot
    cozmo_robot = make_cozmo_robot()

def del_shapes():
    glDeleteLists(cube1,1)
    glDeleteLists(cube2,1)
    glDeleteLists(cube3,1)
    glDeleteLists(cozmo_robot,1)

initial_view_translate = [1.1, 2.5, -0.5]
initial_view_rotate = [25, 25, 0]

view_translate = initial_view_translate.copy()
view_rotate = initial_view_rotate.copy()

view_dist = 7.0

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    #glOrtho(-10, 10, -10, 10, -10, 10)
    gluPerspective(50.0, 1.0, 1.0, 19.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    fixation_point = view_translate
    hdg = view_rotate[1]
    camera_loc = [view_dist * sin(radians(hdg)) + view_translate[0],
                  view_dist * sin(radians(view_rotate[0])) + view_translate[1],
                  view_dist * cos(radians(hdg)) + view_translate[2]]
    gluLookAt(*camera_loc, *fixation_point, 0.0, 1.0, 0.0)
    make_shapes()
    glCallList(cube1)
    glCallList(cube2)
    glCallList(cube3)
    glCallList(cozmo_robot)
    glutSwapBuffers()
    del_shapes()

def idle():
    glutPostRedisplay()

def keyboard(key, x, y):
    global exited
    if ord(key) == 27:
        exited = True
        glutDestroyWindow(1)
        sys.exit(0)
    global view_translate, view_rotate, view_dist
    hdg = -view_rotate[1]
    if key == b'a':
        view_translate[0] -= 0.1 * cos(radians(hdg))
        view_translate[2] -= 0.1 * sin(radians(hdg))
    elif key == b'd':
        view_translate[0] += 0.1 * cos(radians(hdg))
        view_translate[2] += 0.1 * sin(radians(hdg))
    elif key == b's':
        view_dist += 0.1
    elif key == b'w':
        view_dist -= 0.1
    elif key == b'j':
        view_rotate[1] += 5
    elif key == b'l':
        view_rotate[1] -= 5
    elif key == b'k':
        view_rotate[0] += 5
    elif key == b'i':
        view_rotate[0] -= 5
    elif key == b'z':
        view_translate = initial_view_translate.copy()
        view_rotate = initial_view_rotate.copy()
    #print(view_translate, view_rotate)
    glutPostRedisplay()

def special(key, x, y):
    global view_translate, view_rotate, view_dist
    hdg = -view_rotate[1]
    if key == GLUT_KEY_LEFT:
      view_rotate[1] += 5
    elif key == GLUT_KEY_RIGHT:
      view_rotate[1] -= 5
    elif key == GLUT_KEY_UP:
        view_rotate[0] -= 5
    elif key == GLUT_KEY_DOWN:
        view_rotate[0] += 5
    elif key == GLUT_KEY_PAGE_UP:
      view_translate[1] += 0.1
    elif key == GLUT_KEY_PAGE_DOWN:
      view_translate[1] -= -.1
    glutPostRedisplay()

def reshape(width, height):
    glViewport(0, 0, width, height)

def visible(vis):
    if vis == GLUT_VISIBLE:
        glutIdleFunc(idle)
    else:
        glutIdleFunc(None)

def init_display():
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(*initial_window_size)
    # glutInitWindowPosition(100, 100)
    glutInit()
    glutCreateWindow("Cozmo's World")
    glClearColor(0, 0, 0, 0)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)
    glutDisplayFunc(display)
    glutIdleFunc(idle)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    glutSpecialFunc(special)
    glutVisibilityFunc(visible)
    glutMainLoop()

def viewer(_robot):
    global robot
    robot = _robot
    th = threading.Thread(None,init_display)
    th.start()

