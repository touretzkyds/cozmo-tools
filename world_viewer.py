"""

OpenGL world viewer for Cozmo's world map.
====================================

Run it by typing:  viewer(robot)

  
Author:  David S. Touretzky, Carnegie Mellon University
=======

Change Log:
===========
* 12/27/2016
    Dave Touretzky
        - Add suport for FixedCustomObject (yellow cuboids) and CustomObject (orange cuboids)
        - Add a floor with 100 mm grid lines
        - Add full 3D rotations using an object's pose quaternion
        - Add numeric text label to top of cube to break rotational symmetry
        - Change "v" command to toggle display of camera parameters.

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
* Give cubes distinctive faces so we can better see their orientation.
* Add the robot's lift.
* Replace cubes and robot body with texture-mapped images.

"""

from math import sin, cos, atan2, pi, radians
import sys
import threading
import array

try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    print("Can't find required OpenGL package.  Do 'pip3 install PyOpenGL PyOpenGL_accelerate'")
    print("  and also 'sudo apt-get install freeglut3'/ or 'brew install freeglut'")
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
  v            Toggle display of viewing parameters
  h            Print help
"""

exited = False
GLwindow = None

window_width = window_height = 500

cube_vertices = array.array('f', [
     -0.5, -0.5, +0.5,
     -0.5, +0.5, +0.5,
     +0.5, +0.5, +0.5,
     +0.5, -0.5, +0.5,
     -0.5, -0.5, -0.5,
     -0.5, +0.5, -0.5,
     +0.5, +0.5, -0.5,
     +0.5, -0.5, -0.5 
     ])

cube_colors_0 = array.array('f', [
     0.6, 0.6, 0.0,
     0.6, 0.6, 0.0,
     0.0, 0.0, 0.7,
     0.0, 0.0, 0.7,
     0.7, 0.0, 0.0,
     0.7, 0.0, 0.0,
     0.0, 0.7, 0.0,
     0.0, 0.7, 0.0,
     ])

cube_colors_1 = array.array('f', [x/0.7 for x in cube_colors_0])

cube_colors_2 = array.array('f',
    [0.8, 0.8, 0.0,
     0.8, 0.8, 0.0,
     0.0, 0.8, 0.8,
     0.0, 0.8, 0.8,
     0.8, 0.0, 0.8,
     0.8, 0.0, 0.8,
     0.9, 0.9, 0.9,
     0.9, 0.9, 0.9 ])

color_black  = (0., 0., 0.)
color_white  = (1., 1., 1.)
color_red    = (1., 0., 0.)
color_green  = (0., 1., 0.)
color_blue   = (0., 0., 1.0)
color_yellow = (1., .93, 0.)
color_orange = (1., 0.5, .063)
color_gray =   (0.5, 0.5, 0.5)
color_light_gray =   (0.65, 0.65, 0.65)

cube_cIndices = array.array('B',
    [0, 3, 2, 1,
     2, 3, 7, 6,
     0, 4, 7, 3,
     1, 2, 6, 5,
     4, 5, 6, 7,
     0, 1, 5, 4 ])

light_cube_size_mm = 44.3

robot_body_size_mm =   ( 70, 56,   30)
robot_body_offset_mm = (-30,  0,   15)
robot_head_size_mm =   ( 36, 39.4, 36)
robot_head_offset_mm = ( 20,  0,   36)

charger_bed_size_mm =  (104, 98, 10 )
charger_back_size_mm = (  5, 90, 35 )

wscale = 0.02  # millimeters to graphics window coordinates

def quat2rot(q0,q1,q2,q3):
    # formula from http://stackoverflow.com/questions/7938373/from-quaternions-to-opengl-rotations
    q0_sq = q0*q0; q1_sq = q1*q1; q2_sq = q2*q2; q3_sq = q3*q3
    t_q0q1 = 2. * q0 * q1
    t_q0q2 = 2. * q0 * q2
    t_q0q3 = 2. * q0 * q3
    t_q1q2 = 2. * q1 * q2
    t_q1q3 = 2. * q1 * q3
    t_q2q3 = 2. * q2 * q3
    # result must be in column-major order for glMultMatrixf
    return (
        q0_sq+q1_sq-q2_sq-q3_sq, t_q1q2+t_q0q3,           t_q1q3-t_q0q2,           0,
        t_q1q2-t_q0q3,           q0_sq-q1_sq+q2_sq-q3_sq, t_q2q3+t_q0q1,           0,
        t_q1q3+t_q0q2,           t_q2q3-t_q0q1,           q0_sq-q1_sq-q2_sq+q3_sq, 0,
        0.,                      0.,                      0.,                      1.
        )


def make_cube(size=(1,1,1), highlight=False, color=None, body=True, edges=True):
    """Make a cube centered on the origin"""
    glEnableClientState(GL_VERTEX_ARRAY)
    if color is None:
        glEnableClientState(GL_COLOR_ARRAY)
        if highlight:
            glColorPointer(3, GL_FLOAT, 0, cube_colors_1.tobytes())
        else:
            glColorPointer(3, GL_FLOAT, 0, cube_colors_0.tobytes())
    else:
        if not highlight:
            s = 0.5   # scale down the brightness if necessary
            color = (color[0]*s, color[1]*s, color[2]*s)
        glColor3f(*color)
    verts = cube_vertices * 1; # copy the array
    for i in range(0,24,3):
        verts[i  ] *= size[0]
        verts[i+1] *= size[1]
        verts[i+2] *= size[2]
    if body:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glVertexPointer(3, GL_FLOAT, 0, verts.tobytes())
        glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tobytes())
    if edges:
        # begin wireframe
        for i in range(0,24): verts[i] *= 1.02
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glVertexPointer(3, GL_FLOAT, 0, verts.tobytes())
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
        glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tobytes())
        # end wireframe
    glDisableClientState(GL_COLOR_ARRAY)
    glDisableClientState(GL_VERTEX_ARRAY)


def make_light_cube(cube_number):
    lcube = robot.world.light_cubes[cube_number]
    if (not lcube.pose) or not lcube.pose.is_valid: return None
    p = lcube.pose.position.x_y_z
    s = light_cube_size_mm
    color = (None, color_red, color_green, color_blue)[cube_number]
    valid_pose = (lcube.pose.origin_id == robot.pose.origin_id)
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    glTranslatef(*p)
    rotmat = array.array('f',quat2rot(*lcube.pose.rotation.q0_q1_q2_q3)).tobytes()
    glMultMatrixf(rotmat)
    #
    if lcube.pose.is_comparable(robot.pose):
        # make solid cube and highlight if visible
        make_cube((s,s,s), highlight=lcube.is_visible, color=color)
    else:
        # make wireframe cube if coords no longer comparable
        make_cube((s,s,s), body=False, highlight=True, color=color)
    glRotatef(-90, 0., 0., 1.)
    glTranslatef(-s/4, -s/4, s/2+0.5)
    glScalef(0.25, 0.2, 0.25)
    glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(ascii(cube_number)))
    glPopMatrix()
    glEndList()
    return c


def make_custom_objects():
    custom_objects = [v for v in robot.world._objects.values()
                     if isinstance(v, (cozmo.objects.CustomObject,
                                       cozmo.objects.FixedCustomObject))]
    if not custom_objects: return None
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    for obj in custom_objects:
        p = obj.pose.position.x_y_z
        obj_size = (obj.x_size_mm, obj.y_size_mm, obj.z_size_mm)
        glPushMatrix()
        glTranslatef(*p)
        rotmat = array.array('f',quat2rot(*obj.pose.rotation.q0_q1_q2_q3)).tobytes()
        glMultMatrixf(rotmat)
        comparable = obj.pose.origin_id == 0 or obj.pose.is_comparable(robot.pose)
        if isinstance(obj, cozmo.objects.FixedCustomObject):
            obj_color = color_yellow
            highlight = True
        else:
            obj_color = color_orange
            highlight = obj.is_visible
        if comparable:
            make_cube(obj_size, highlight=highlight, color=obj_color)
        else:
            make_cube(obj_size, body=False, highlight=False, color=obj_color)
        glPopMatrix()
    glEndList()
    return c


def make_floor():
    floor_size = (800, 800, 1)
    blip = floor_size[2]
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    glTranslatef(0., 0., -blip)
    make_cube(floor_size, highlight=None, color=color_gray)
    glTranslatef(0., 0., 2*blip)
    glColor3f(*color_light_gray)
    for x in range(-floor_size[0]//2,floor_size[0]//2+1,100):
        glBegin(GL_LINES)
        glVertex3f(x,  floor_size[1]//2, 0)
        glVertex3f(x, -floor_size[1]//2, 0)
        glEnd()
    for y in range(-floor_size[1]//2,floor_size[1]//2+1,100):
        glBegin(GL_LINES)
        glVertex3f( floor_size[0]/2, y, 0)
        glVertex3f(-floor_size[0]/2, y, 0)
        glEnd()
    glPopMatrix()
    glEndList()
    return c


def make_charger():
    charger = robot.world.charger
    if (not charger) or (not charger.pose) or not charger.pose.is_valid: return None
    comparable = charger.pose.is_comparable(robot.pose)
    highlight = charger.is_visible or (robot.is_on_charger and comparable)
    c = glGenLists(1)
    p = charger.pose.position.x_y_z
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    glTranslatef(*p)
    glRotatef(charger.pose.rotation.angle_z.degrees, 0, 0, 1)
    glTranslatef(charger_bed_size_mm[0]/2,
                 0,
                 charger_bed_size_mm[2]/2)
    glRotatef(180, 0, 0, 1) # charger "front" is opposite robot "front"
    if comparable:
        make_cube(charger_bed_size_mm, highlight=highlight)
    else:
        make_cube(charger_bed_size_mm, body=False, \
                  highlight=False, color=color_white)
    glTranslatef(
           (charger_back_size_mm[0]-charger_bed_size_mm[0])/2,
           0,
           charger_back_size_mm[2]/2)
    if comparable:
        make_cube(charger_back_size_mm, highlight=highlight)
    else:
        make_cube(charger_back_size_mm, body=False, \
                  highlight=True, color=color_white)
    glPopMatrix()
    glEndList()
    return c


def make_cozmo_robot():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    p = robot.pose.position.x_y_z
    glTranslatef(*p)
    glTranslatef(*robot_body_offset_mm)
    glRotatef(robot.pose.rotation.angle_z.degrees, 0, 0, 1)
    make_cube(robot_body_size_mm, highlight=robot.is_on_charger)
    h = robot_head_offset_mm
    glTranslatef(*h)
    glRotatef(-robot.head_angle.degrees, 0, 1, 0)
    make_cube(robot_head_size_mm, highlight=robot.is_on_charger)
    glPopMatrix()
    glEndList()
    return c

axis_length = 100
axis_width = 1
show_axes = True
print_camera = False


def make_axes():
    if not show_axes: return None
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    len = axis_length
    w = axis_width
    glPushMatrix()
    glTranslate(len/2, 0, 0)
    make_cube((len,w,w), highlight=True, color=color_red, edges=False)
    glPopMatrix()
    glPushMatrix()
    glTranslate(0, len/2, 0)
    make_cube((w,len,w), highlight=True, color=color_green, edges=False)
    glPopMatrix()
    glPushMatrix()
    glTranslate(0, 0, len/2)
    make_cube((w,w,len), highlight=True, color=color_blue, edges=False)
    glPopMatrix()
    glEndList()
    return c


def make_gazepoint():
    c = glGenLists(1)
    glNewList(c, GL_COMPILE)
    glPushMatrix()
    glTranslate(fixation_point[0], fixation_point[1], fixation_point[2])
    s = 3.
    make_cube((s,s,s), highlight=True, color=(1.0, 0.9, 0.1), edges=False)
    glPopMatrix()
    glEndList()
    return c


def make_shapes():
    global axes, gazepoint, cube1, cube2, cube3, charger, cozmo_robot, custom_objects, floor
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
    # custom objects
    custom_objects = make_custom_objects()
    # floor
    floor = make_floor()


def del_shapes():
    if gazepoint: glDeleteLists(gazepoint,1)
    if axes: glDeleteLists(axes,1)
    if cube1: glDeleteLists(cube1,1)
    if cube2: glDeleteLists(cube2,1)
    if cube3: glDeleteLists(cube3,1)
    if charger: glDeleteLists(charger,1)
    if custom_objects: glDeleteLists(custom_objects,1)
    if floor: glDeleteLists(floor,1)
    glDeleteLists(cozmo_robot,1)

initial_fixation_point = [100, -25, 0]
initial_camera_rotation = [0, 40, 270]
initial_camera_distance = 500

fixation_point = initial_fixation_point.copy()
camera_rotation = initial_camera_rotation.copy()
camera_distance = initial_camera_distance
camera_loc = (0., 0., 0.)  # will be recomputed by display()


def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    field_of_view = 50 # degrees
    aspect_ratio = window_width / window_height
    near_clip = 5
    far_clip = 600 # 20.0
    gluPerspective(field_of_view, aspect_ratio, near_clip, far_clip)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    wscale = 0.1
    rotmat = array.array('f',[
        wscale, 0,      0,      0,
        0,      wscale, 0,      0,
        0,      0,      wscale, 0,
        0,      0,      0,      1]).tobytes()
    glMultMatrixf(rotmat)
    # Model transformation switches to robot coordinates: z is up, x forward, y left.
    # View transformation moves the camera, keeping it pointed at the fixation point.
    # Keyboard commands: translations move the fixation point, rotations orbit the camera.
    pitch = camera_rotation[1]
    yaw = camera_rotation[2]
    global camera_loc
    camera_loc = [
        camera_distance * cos(radians(yaw)) + fixation_point[0],
        camera_distance * sin(radians(yaw)) + fixation_point[1],
        camera_distance * sin(radians(pitch)) + fixation_point[2]
        ]
    gluLookAt(*camera_loc, *fixation_point, 0.0, 0.0, 1.0)
    make_shapes()
    if axes: glCallList(axes)
    if cube1: glCallList(cube1)
    if cube2: glCallList(cube2)
    if cube3: glCallList(cube3)
    if charger: glCallList(charger)
    if custom_objects: glCallList(custom_objects)
    if floor: glCallList(floor)
    glCallList(cozmo_robot)
    if gazepoint: glCallList(gazepoint)
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
    global fixation_point, camera_rotation, camera_distance, show_axes, print_camera
    heading = atan2(camera_loc[1]-fixation_point[1], camera_loc[0]-fixation_point[0])*180/pi
    translate_step = 5
    if key == b'a':
        fixation_point[0] -= translate_step * cos(radians(heading+90))
        fixation_point[1] -= translate_step * sin(radians(heading+90))
    elif key == b'd':
        fixation_point[0] += translate_step * cos(radians(heading+90))
        fixation_point[1] += translate_step * sin(radians(heading+90))
    elif key == b'w':
        fixation_point[0] -= translate_step * cos(radians(heading))
        fixation_point[1] -= translate_step * sin(radians(heading))
    elif key == b's':
        fixation_point[0] += translate_step * cos(radians(heading))
        fixation_point[1] += translate_step * sin(radians(heading))
    elif key == b'>':
        camera_distance += translate_step
    elif key == b'<':
        camera_distance -= translate_step
    elif key == b'j':
        camera_rotation[2] -= 2.5
    elif key == b'l':
        camera_rotation[2] += 2.5
    elif key == b'k':
        camera_rotation[1] -= 2.5
    elif key == b'i':
        camera_rotation[1] += 2.5
    elif key == b'x':
        show_axes = not show_axes
    elif key == b'h':
        print(help_text)
    elif key == b'v':
        print_camera = not print_camera
        if not print_camera:
            print("Halted viewing parameters display. Press 'v' again to resume.")
    elif key == b'z':
        fixation_point = initial_fixation_point.copy()
        camera_rotation = initial_camera_rotation.copy()
        camera_distance = initial_camera_distance
    if print_camera:
        pitch = camera_rotation[1]
        yaw = camera_rotation[2]
        print('pitch=%5.1f yaw=%5.1f dist=%f' % (pitch,yaw,camera_distance),
              ' gazepointt[%5.1f %5.1f %5.1f]' %
                  (fixation_point[0], fixation_point[1], fixation_point[2]),
              ' camera[%5.1f %5.1f %5.1f]' % (camera_loc[0], camera_loc[1], camera_loc[2]))
    display()


def special(key, x, y):
    global fixation_point, camera_rotation, camera_distance
    heading = -camera_rotation[1]
    if key == GLUT_KEY_LEFT:
        camera_rotation[2] = (camera_rotation[2] - 2.5) % 360
    elif key == GLUT_KEY_RIGHT:
        camera_rotation[2] = (camera_rotation[2] + 2.5) % 360
    elif key == GLUT_KEY_UP:
        camera_rotation[1] = (camera_rotation[1] + 90 + 2.5) % 180 - 90
    elif key == GLUT_KEY_DOWN:
        camera_rotation[1] = (camera_rotation[1] + 90 - 2.5) % 180 - 90
    elif key == GLUT_KEY_PAGE_UP:
        fixation_point[2] += 1
    elif key == GLUT_KEY_PAGE_DOWN:
        fixation_point[2] -= 1
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
    GLwindow = glutCreateWindow(b"Cozmo's World")
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

RUNNING = False


def viewer(_robot):
    global RUNNING, robot
    if RUNNING:
        return
    else:
        RUNNING = True
    if not isinstance(_robot,cozmo.robot.Robot):
        raise TypeError('Argument must be a cozmo.robot.Robot instance.')
    robot = _robot
    th = threading.Thread(target=init_display)
    th.start()
