"""

OpenGL world viewer for Cozmo's world map.
====================================

Run it by typing:  viewer(robot)

  
Author:  David S. Touretzky, Carnegie Mellon University
=======

Change Log:
===========
* 12/22/2016: Update for SDK release 0.10
    Dave Touretzky
        - Make use of the new Pose.is_valid and Pose.is_comparable() features.

* 12/7/2016: More enhancements.
    Dave Touretzky
        - Display coordinate axes and show the gazepoint as a yellow dot.
        - 'x' command toggles axes; 'h' prints help, 'v' displays viewpoint.
        - 'w' and 's' now translate; '<' and '>' zoom.
        - Up/down arrow motion was reversed; fixed it.
        - Page up/page down only moved in one direction; fixed it.
        - Charger now displayed in two pieces: bed and back wall.
        - Objects are displayed in wireframe if their pose is no longer valid.
        - Use proper height offsets for robot and charger.
        - Rename was_visible to is_present since charger may never be seen.

* 12/4/2016: Various improvements.
    Dave Touretzky
        - Add was_visible attribute to determine whether an object has ever been
          visible.  If not, we can't trust its coordinates, and while they will
          initially be (0,0,0) for lightcubes, that is not true for the charger.
          Only objects with was_present == True are displayed; they are
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
* Move the scaling and axis swap into the model view so all
    the rest of the code can work in Cozmo world coordinates
    (x-forward, z-up) and millimeter units.
* Give cubes distinctive faces so we can see their orientation
* Use quaternion instead of angle_z so we can fully rotate the cube
* Add representation of custom (user-generated) barriers
* Add the robot's lift
* Replace cubes and robot body with texture-mapped images
* Add a floor to the world

"""

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

help_text = """
World viewer keyboard commands:
  a            Translate gazepoint left
  d            Translate gazepoint right
  w            Translate gazepoint forward
  s            Translate gazepoint backward
  <            Zoom in
  >            Zoom out
  page-up      Translate gazepoint up
  page-down    Translate gazepoint down

  left-arrow   Orbit camera left
  right-arrow  Orbit camera right
  up-arrow     Orbit camera upward
  down-arrow   Orbit camera downward

  x            Toggle axes
  z            Reset to initial view
  v            Display viewing parameters
  h            Print help
"""

exited = False
GLwindow = None

window_width = window_height = 500

cube_vertices = array.array('f', [ \
     -0.5, -0.5, +0.5, \
     -0.5, +0.5, +0.5, \
     +0.5, +0.5, +0.5, \
     +0.5, -0.5, +0.5, \
     -0.5, -0.5, -0.5, \
     -0.5, +0.5, -0.5, \
     +0.5, +0.5, -0.5, \
     +0.5, -0.5, -0.5  \
     ])

cube_colors_0 = array.array('f', [ \
     0.6, 0.6, 0.0, \
     0.6, 0.6, 0.0, \
     0.0, 0.0, 0.7, \
     0.0, 0.0, 0.7, \
     0.7, 0.0, 0.0, \
     0.7, 0.0, 0.0, \
     0.0, 0.7, 0.0, \
     0.0, 0.7, 0.0, \
     ])

cube_colors_1 = array.array('f', [x/0.7 for x in cube_colors_0])

cube_colors_2 = array.array('f', \
    [0.8, 0.8, 0.0, \
     0.8, 0.8, 0.0, \
     0.0, 0.8, 0.8, \
     0.0, 0.8, 0.8, \
     0.8, 0.0, 0.8, \
     0.8, 0.0, 0.8, \
     0.9, 0.9, 0.9, \
     0.9, 0.9, 0.9 ])

color_black = (0., 0., 0.)
color_white = (1., 1., 1.)
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

robot_body_size_mm =   (56,   30, 70)
robot_body_offset_mm = ( 0,   15, 30)
robot_head_size_mm =   (39.4, 39, 36)
robot_head_offset_mm = (20,    0, 38)

#charger_size_mm = (98, 34.75, 104)
charger_bed_size_mm = (98, 10, 104)
charger_back_size_mm = (90, 34.75, 5)

wscale = 0.02  # millimeters to graphics window coordinates

def make_cube(size=(1,1,1), highlight=False, color=None, body=True, edges=True):
    """Make a cube centered on the origin"""
    glEnableClientState(GL_VERTEX_ARRAY)
    if color is None:
        glEnableClientState(GL_COLOR_ARRAY)
        if highlight:
            glColorPointer(3, GL_FLOAT, 0, cube_colors_1.tostring())
        else:
            glColorPointer(3, GL_FLOAT, 0, cube_colors_0.tostring())
    else:
        if not highlight:
            s = 0.5   # scale down the brightness if necessary
            color = (color[0]*s, color[1]*s, color[2]*s)
        glColor3f(*color)
    verts = cube_vertices * 1; # copy the array
    for i in range(0,24,3):
        verts[i  ] *= size[0] * wscale
        verts[i+1] *= size[1] * wscale
        verts[i+2] *= size[2] * wscale
    if body:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glVertexPointer(3, GL_FLOAT, 0, verts.tostring())
        glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tostring())
    if edges:
        # begin wireframe
        for i in range(0,24): verts[i] *= 1.01
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glVertexPointer(3, GL_FLOAT, 0, verts.tostring())
        glDisableClientState(GL_COLOR_ARRAY)
        if body:
            if highlight:
                glColor3f(*color_white)
            else:
                glColor3f(*color_black)
        else:
            if highlight:
                glColor3f(*color)
            else:
                s = 0.7   # scale down the brightness if necessary
                glColor3f(color[0]*s, color[1]*s, color[2]*s)
        glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tostring())
        # end wireframe
    glDisableClientState(GL_COLOR_ARRAY)
    glDisableClientState(GL_VERTEX_ARRAY)

def make_light_cube(cube_number):
    lcube = robot.world.light_cubes[cube_number]
    if not lcube.pose.is_valid: return None
    p = lcube.pose.position.x_y_z
    s = light_cube_size_mm
    color = (None, color_red, color_green, color_blue)[cube_number]
    valid_pose = (lcube.pose.origin_id == robot.pose.origin_id)
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
    glRotatef(lcube.pose.rotation.angle_z.degrees, 0, 1, 0)
    if lcube.pose.is_comparable(robot.pose):
        # make solid cube and highlight if visible
        make_cube((s,s,s), highlight=lcube.is_visible, color=color)
    else:
        # make wireframe cube if coords no longer valid
        make_cube((s,s,s), body=False, highlight=True, color=color)
    glPopMatrix()
    glEndList()
    return c

def make_charger():
    charger = robot.world.charger
    if not charger.pose.is_valid: return None
    # Bug: charger.pose.origin_id doesn't always update when on charger
    comparable = charger.pose.is_comparable(robot.pose)
    high = charger.is_visible or (robot.is_on_charger and comparable)
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    p = charger.pose.position.x_y_z
    glPushMatrix()
    glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
    glRotatef(charger.pose.rotation.angle_z.degrees, 0, 1, 0)
    glTranslatef(0., charger_bed_size_mm[1]/2*wscale,
                    -charger_bed_size_mm[2]/2*wscale)
    glRotatef(180, 0, 1, 0) # charger "front" is opposite robot "front"
    if comparable:
        make_cube(charger_bed_size_mm, highlight=high)
    else:
        make_cube(charger_bed_size_mm, body=False, \
                  highlight=False, color=color_white)
    glTranslatef(0., \
           (charger_back_size_mm[1]-charger_bed_size_mm[1])/2*wscale, \
           charger_bed_size_mm[2]/2*wscale)
    if comparable:
        make_cube(charger_back_size_mm, highlight=high)
    else:
        make_cube(charger_back_size_mm, body=False, \
                  highlight=False, color=color_white)
    glPopMatrix()
    glEndList()
    return c

def make_cozmo_robot():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    p = robot.pose.position.x_y_z
    glTranslatef(-p[1]*wscale, p[2]*wscale, -p[0]*wscale)
    glTranslatef(0, robot_body_offset_mm[1]*wscale, robot_body_offset_mm[2]*wscale)
    glRotatef(robot.pose.rotation.angle_z.degrees, 0, 1, 0)
    make_cube(robot_body_size_mm, highlight=robot.is_on_charger)
    h = robot_head_offset_mm
    glTranslatef(-h[1]*wscale, h[2]*wscale, -h[0]*wscale)
    glRotatef(robot.head_angle.degrees, 1, 0, 0)
    make_cube(robot_head_size_mm, highlight=robot.is_on_charger)
    glPopMatrix()
    glEndList()
    return c

axis_length = 100
axis_width = 1
show_axes = True

def make_axes():
    if not show_axes: return None
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    l = axis_length
    w = axis_width
    glTranslate(0, 0, -l/2 * wscale)
    make_cube((w,w,l), highlight=True, color=color_red, edges=False)
    glPopMatrix()
    glPushMatrix()
    glTranslate(-l/2 * wscale, 0, 0)
    make_cube((l,w,w), highlight=True, color=color_green, edges=False)
    glPopMatrix()
    glPushMatrix()
    glTranslate(0, l/2 * wscale, 0)
    make_cube((w,l,w), highlight=True, color=color_blue, edges=False)
    glPopMatrix()
    glEndList()
    return c

def make_gazepoint():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    glTranslate(fixation_point[0], fixation_point[1], \
                fixation_point[2])
    s = 3.
    make_cube((s,s,s), highlight=True, color=(1.0, 0.9, 0.1), edges=False)
    glPopMatrix()
    glEndList()
    return c

def make_shapes():
    global axes, gazepoint, cube1, cube2, cube3, charger, cozmo_robot
    # axes
    axes = make_axes()
    # gaze point
    gazepoint = make_gazepoint()
    # light cubes
    cube1 = make_light_cube(cozmo.objects.LightCube1Id)
    cube2 = make_light_cube(cozmo.objects.LightCube2Id)
    cube3 = make_light_cube(cozmo.objects.LightCube3Id)
    # charger
    charger = make_charger()
    # cozmo robot
    cozmo_robot = make_cozmo_robot()

def del_shapes():
    if gazepoint: glDeleteLists(gazepoint,1)
    if axes: glDeleteLists(axes,1)
    if cube1: glDeleteLists(cube1,1)
    if cube2: glDeleteLists(cube2,1)
    if cube3: glDeleteLists(cube3,1)
    if charger: glDeleteLists(charger,1)
    glDeleteLists(cozmo_robot,1)

initial_fixation_point = [1.1, 0, -0.5]
initial_camera_rotation = [25, 25, 0]
initial_camera_distance = 10.0

fixation_point = initial_fixation_point.copy()
camera_rotation = initial_camera_rotation.copy()
camera_distance = initial_camera_distance

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
    if axes: glCallList(axes)
    if gazepoint: glCallList(gazepoint)
    if cube1: glCallList(cube1)
    if cube2: glCallList(cube2)
    if cube3: glCallList(cube3)
    if charger: glCallList(charger)
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
    global fixation_point, camera_rotation, camera_distance, show_axes
    heading = -camera_rotation[1]
    if key == b'a':
        fixation_point[0] -= 0.1 * cos(radians(heading))
        fixation_point[2] -= 0.1 * sin(radians(heading))
    elif key == b'd':
        fixation_point[0] += 0.1 * cos(radians(heading))
        fixation_point[2] += 0.1 * sin(radians(heading))
    elif key == b'w':
        fixation_point[0] -= 0.1 * cos(radians(heading+90))
        fixation_point[2] -= 0.1 * sin(radians(heading+90))
    elif key == b's':
        fixation_point[0] += 0.1 * cos(radians(heading+90))
        fixation_point[2] += 0.1 * sin(radians(heading+90))
    elif key == b'>':
        camera_distance += 0.1
    elif key == b'<':
        camera_distance -= 0.1
    elif key == b'j':
        camera_rotation[1] += 5
    elif key == b'l':
        camera_rotation[1] -= 5
    elif key == b'k':
        camera_rotation[0] += 5
    elif key == b'i':
        camera_rotation[0] -= 5
    elif key == b'x':
        show_axes = not show_axes
    elif key == b'h':
        print(help_text)
    elif key == b'v':
        # permute gazepoint coords to match robot instead of OpenGL
        print('gazepoint[%.1f, %.1f, %.1f]'
              '  camera[pan=%d, tilt=%d]  distance=%.1f' %
              (fixation_point[2] / wscale, \
               -fixation_point[0] / wscale, \
               fixation_point[1] / wscale, \
               camera_rotation[1], camera_rotation[0], \
               camera_distance/wscale))
    elif key == b'z':
        fixation_point = initial_fixation_point.copy()
        camera_rotation = initial_camera_rotation.copy()
        camera_distance = initial_camera_distance
    display()

def special(key, x, y):
    global fixation_point, camera_rotation, camera_distance
    heading = -camera_rotation[1]
    if key == GLUT_KEY_LEFT:
        camera_rotation[1] = (camera_rotation[1] - 5) % 360
    elif key == GLUT_KEY_RIGHT:
        camera_rotation[1] = (camera_rotation[1] + 5) % 360
    elif key == GLUT_KEY_UP:
        camera_rotation[0] += 5
    elif key == GLUT_KEY_DOWN:
        camera_rotation[0] -= 5
    elif key == GLUT_KEY_PAGE_UP:
        fixation_point[1] += 0.1
    elif key == GLUT_KEY_PAGE_DOWN:
        fixation_point[1] -= 0.1
    display()

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
    global RUNNING, robot
    if not isinstance(_robot,cozmo.robot.Robot):
        raise TypeError('Argument must be a cozmo.robot.Robot instance.')
    if RUNNING:
        print('Viewer is already running!\n')
        return
    else:
        RUNNING = True
    robot = _robot
    th = threading.Thread(target=init_display)
    th.start()

