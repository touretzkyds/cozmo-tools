'''

OpenGL world viewer for Cozmo's world map.
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
* Various improvements.
    Dave Touretzky
        - Add was_visible attribute to determine whether an object has ever been
          visible.  If not, we can't trust its coordinates, and while they will
          initially be (0,0,0) for lightcubes, that is not true for the charger.
          Only objects with was_visible == True are displayed; they are
          highlighted if is_visible == True.
        - Fix aspect ratio in gluPespective call if the window is resized.
        - Some variable renaming and comments added to improve readability.
        - Added safety checks for viewer() function.

* Display improvements.
    Dave Touretzky
        - Added the charger.
        - Draw edges in black.
        - Draw light cubes as solid colors.
        - Add position offsets for robot and charger, since the robot's
          origin is between the front wheels and the charger's origin is
          at the front lip.

* Created.

To Do:
======
    * Give cubes distinctive faces so we can see their orientation
    * Use quaternion instead of angle_z so we can fully rotate the cube
    * Add representation of custom (user-generated) barriers
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
    print("Can't find required OpenGL package.  Do 'pip3 install PyOpenGL PyOpenGL_accelerate'")
    raise

import cozmo

exited = False
GLwindow = None

global window_width, window_height
window_width = window_height = 500

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

color_black = (0., 0., 0.)
color_red   = (1., 0., 0.)
color_green = (0., 1., 0.)
color_blue  = (0., 0., 1.0)


cube_cIndices = array.array('B', \
    [0, 3, 2, 1, \
     2, 3, 7, 6, \
     0, 4, 7, 3, \
     1, 2, 6, 5, \
     4, 5, 6, 7, \
     0, 1, 5, 4 ])

light_cube_size_mm = 44.3

robot_body_size_mm = (56, 30, 70)
robot_body_offset_mm = (0, 0, 30)
robot_head_size_mm = (39.4, 39, 36)
robot_head_offset_mm = (20, 0, 38)

charger_size = (98, 34.75, 104)

wscale = 0.02  # millimeters to graphics window coordinates

def make_cube(xsize=1, ysize=1, zsize=1, vis=False, color=None):
    """Make a cube centered on the origin"""
    glEnableClientState(GL_VERTEX_ARRAY)
    if color is None:
        glEnableClientState(GL_COLOR_ARRAY)
        if vis:
            glColorPointer(3, GL_FLOAT, 0, cube_colors_1.tostring())
        else:
            glColorPointer(3, GL_FLOAT, 0, cube_colors_0.tostring())
    else:
        if not vis:
            s = 0.5   # scale down the brightness
            color = (color[0]*s, color[1]*s, color[2]*s)
        glColor3f(*color)
    verts = cube_vertices * 1;
    for i in range(0,24,3):
        verts[i  ] *= xsize * wscale
        verts[i+1] *= ysize * wscale
        verts[i+2] *= zsize * wscale
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    glVertexPointer(3, GL_FLOAT, 0, verts.tostring())
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tostring())
    # begin wireframe
    for i in range(0,24):
        verts[i] *= 1.0
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    glVertexPointer(3, GL_FLOAT, 0, verts.tostring())
    glDisableClientState(GL_COLOR_ARRAY)
    glColor3f(*color_black)
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tostring())
    # end wireframe
    glDisableClientState(GL_COLOR_ARRAY)
    glDisableClientState(GL_VERTEX_ARRAY)

def make_light_cube(cube_number):
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    lcube = robot.world.light_cubes[cube_number]
    lcube.was_visible = lcube.was_visible or lcube.is_visible
    if lcube.was_visible:
        p = lcube.pose.position.x_y_z
        glPushMatrix()
        glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
        glRotatef(lcube.pose.rotation.angle_z.degrees, 0, 1, 0)
        s = light_cube_size_mm
        color = (None, color_red, color_green, color_blue)[cube_number]
        make_cube(s, s, s, lcube.is_visible, color)
        glPopMatrix()
    glEndList()
    return c

def make_charger():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    charger = robot.world.charger
    charger.was_visible = charger.was_visible or charger.is_visible
    if charger.was_visible:
        p = charger.pose.position.x_y_z
        glPushMatrix()
        glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
        glTranslatef(0, 0, -charger_size[2]/2*wscale)
        glRotatef(charger.pose.rotation.angle_z.degrees, 0, 1, 0)
        make_cube(*charger_size, charger.is_visible)
        glPopMatrix()
    glEndList()
    return c

def make_cozmo_robot():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    p = robot.pose.position.x_y_z
    glPushMatrix()
    glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
    glTranslatef(0, 0, robot_body_offset_mm[2]*wscale)
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
    global cube1, cube2, cube3, charger, cozmo_robot
    # light cubes
    cube1 = make_light_cube(1)
    cube2 = make_light_cube(2)
    cube3 = make_light_cube(3)
    # charger
    charger = make_charger()
    # cozmo robot
    cozmo_robot = make_cozmo_robot()

def del_shapes():
    glDeleteLists(cube1,1)
    glDeleteLists(cube2,1)
    glDeleteLists(cube3,1)
    glDeleteLists(charger,1)
    glDeleteLists(cozmo_robot,1)

initial_fixation_point = [1.1, 2.5, -0.5]
initial_camera_rotation = [25, 25, 0]
initial_camera_distance = 7.0

fixation_point = initial_fixation_point.copy()
camera_rotation = initial_camera_rotation.copy()
camera_distance = initial_camera_distance

global window_width, window_height

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    field_of_view = 50 # degrees
    aspect_ratio = window_width / window_height
    near_clip = 0.5
    far_clip = 20.0
    gluPerspective(field_of_view, aspect_ratio, near_clip, far_clip)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    # Model transformation is identity matrix so all objects start out at the origin.
    # View transformation moves the camera, keeping it pointed at the fixation point.
    # Keyboard commands: translations move the fixation point, rotations orbit the camera.
    heading = camera_rotation[1]
    pitch = camera_rotation[0]
    camera_loc = [camera_distance * sin(radians(heading)) + fixation_point[0],
                  camera_distance * sin(radians(pitch)) + fixation_point[1],
                  camera_distance * cos(radians(heading)) + fixation_point[2]]
    gluLookAt(*camera_loc, *fixation_point, 0.0, 1.0, 0.0)
    make_shapes()
    glCallList(cube1)
    glCallList(cube2)
    glCallList(cube3)
    glCallList(charger)
    glCallList(cozmo_robot)
    glutSwapBuffers()
    del_shapes()

def idle():
    if exited:
        glutDestroyWindow(GLwindow)
        return
    glutPostRedisplay()

def keyboard(key, x, y):
    global exited
    if ord(key) == 27:
        print("Use 'exit' to quit.")
        #exited = True
        #return
    global fixation_point, camera_rotation, camera_distance
    heading = -camera_rotation[1]
    if key == b'a':
        fixation_point[0] -= 0.1 * cos(radians(heading))
        fixation_point[2] -= 0.1 * sin(radians(heading))
    elif key == b'd':
        fixation_point[0] += 0.1 * cos(radians(heading))
        fixation_point[2] += 0.1 * sin(radians(heading))
    elif key == b's':
        camera_distance += 0.1
    elif key == b'w':
        camera_distance -= 0.1
    elif key == b'j':
        camera_rotation[1] += 5
    elif key == b'l':
        camera_rotation[1] -= 5
    elif key == b'k':
        camera_rotation[0] += 5
    elif key == b'i':
        camera_rotation[0] -= 5
    elif key == b'z':
        fixation_point = initial_fixation_point.copy()
        camera_rotation = initial_camera_rotation.copy()
    #print(fixation_point, camera_rotation)
    glutPostRedisplay()

def special(key, x, y):
    global fixation_point, camera_rotation, camera_distance
    heading = -camera_rotation[1]
    if key == GLUT_KEY_LEFT:
        camera_rotation[1] += 5
    elif key == GLUT_KEY_RIGHT:
        camera_rotation[1] -= 5
    elif key == GLUT_KEY_UP:
        camera_rotation[0] -= 5
    elif key == GLUT_KEY_DOWN:
        camera_rotation[0] += 5
    elif key == GLUT_KEY_PAGE_UP:
        fixation_point[1] += 0.1
    elif key == GLUT_KEY_PAGE_DOWN:
        fixation_point[1] -= -.1
    glutPostRedisplay()

def reshape(width, height):
    global window_width, window_height
    window_width = width
    window_height = height
    glViewport(0, 0, width, height)

def visible(vis):
    if vis == GLUT_VISIBLE:
        glutIdleFunc(idle)
    else:
        glutIdleFunc(None)

def init_display():
    global GLwindow
    
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(window_width, window_height)
    # glutInitWindowPosition(100, 100)
    GLwindow = glutCreateWindow("Cozmo's World")
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

global RUNNING
RUNNING = False

def viewer(_robot):
    global RUNNING
    if not isinstance(_robot,cozmo.robot.Robot):
        raise TypeError('Argument must be a cozmo.robot.Robot instance.')
    if RUNNING:
        print('Viewer is already running!\n')
        return
    else:
        RUNNING = True
    global robot
    robot = _robot
    for obj in robot.world._objects.values():
        try:
            obj.was_visible = obj.is_visible
        except:
            pass
    th = threading.Thread(target=init_display)
    th.start()

