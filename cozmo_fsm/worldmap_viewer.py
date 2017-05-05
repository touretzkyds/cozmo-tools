"""
OpenGL world viewer for cozmo_fsm world map.
"""

from math import sin, cos, atan2, pi, radians
import time
import array
import numpy as np

from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

from . import opengl
from . import transform
from . import worldmap

import cozmo

WINDOW = None

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

color_black  = (0., 0., 0.)
color_white  = (1., 1., 1.)
color_red    = (1., 0., 0.)
color_green  = (0., 1., 0.)
color_blue   = (0., 0., 1.0)
color_yellow = (1., .93, 0.)
color_orange = (1., 0.5, .063)
color_gray =   (0.5, 0.5, 0.5)
color_light_gray =   (0.65, 0.65, 0.65)

cube_cIndices = array.array('B', \
    [0, 3, 2, 1, \
     2, 3, 7, 6, \
     0, 4, 7, 3, \
     1, 2, 6, 5, \
     4, 5, 6, 7, \
     0, 1, 5, 4 ])

light_cube_size_mm = 44.3

robot_body_size_mm =   ( 70, 56,   30)
robot_body_offset_mm = (-30,  0,   15)
robot_head_size_mm =   ( 36, 39.4, 36)
robot_head_offset_mm = ( 20,  0,   36)

charger_bed_size_mm =  (104, 98, 10 )
charger_back_size_mm = (  5, 90, 35 )

wscale = 0.02  # millimeters to graphics window coordinates

axis_length = 100
axis_width = 1
print_camera = False

initial_fixation_point = [100, -25, 0]
initial_camera_rotation = [0, 40, 270]
initial_camera_distance = 500

fixation_point = initial_fixation_point.copy()
camera_rotation = initial_camera_rotation.copy()
camera_distance = initial_camera_distance
camera_loc = (0., 0., 0.)  # will be recomputed by display()

class WorldMapViewer():
    def __init__(self, robot, width=512, height=512,
                 windowName = "Cozmo's World",
                 bgcolor = (0,0,0)):
        self.robot = robot
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.windowName = windowName
        self.bgcolor = bgcolor
        self.translation = [0., 0.]  # Translation in mm
        self.scale = 1
        self.show_axes = True

    def make_cube(self,size=(1,1,1), highlight=False, color=None, body=True, edges=True):
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
            glColor4f(*color,1)
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
                    glColor4f(*color_white,1)
                else:
                    glColor4f(*color_black,1)
            else:
                if highlight:
                    glColor4f(*color,1)
                else:
                    s = 0.7   # scale down the brightness if necessary
                    glColor4f(color[0]*s, color[1]*s, color[2]*s, 1)
            glDrawElements(GL_QUADS, 24, GL_UNSIGNED_BYTE, cube_cIndices.tobytes())
            # end wireframe
        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)

    def make_light_cube(self,lcube,cube_obst):
        global gl_lists
        cube_number = cube_obst.id
        pos = (cube_obst.x, cube_obst.y, cube_obst.z)
        color = (None, color_red, color_green, color_blue)[cube_number]
        valid_pose = (lcube.pose.origin_id == self.robot.pose.origin_id)
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        glTranslatef(*pos)
        # Transpose the matrix for sending to OpenCV
        t = transform.quat2rot(*lcube.pose.rotation.q0_q1_q2_q3).transpose()
        rotmat = array.array('f',t.flatten()).tobytes()
        glMultMatrixf(rotmat)
        s = light_cube_size_mm
        if lcube.pose.is_comparable(self.robot.pose):
            # make solid cube and highlight if visible
            self.make_cube((s,s,s), highlight=lcube.is_visible, color=color)
        else:
            # make wireframe cube if coords no longer comparable
            self.make_cube((s,s,s), body=False, highlight=True, color=color)
        glRotatef(-90, 0., 0., 1.)
        glTranslatef(-s/4, -s/4, s/2+0.5)
        glScalef(0.25, 0.2, 0.25)
        glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(ascii(cube_number)))
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_custom_cube(self,custom_obj,obst):
        global gl_lists
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        pos = (obst.x, obst.y, obst.z)
        size = obst.size
        orient = obst.theta
        glPushMatrix()
        glTranslatef(pos[0], pos[1], max(pos[2],5))
        # Transpose the pose rotation matrix for sending to OpenCV
        if isinstance(custom_obj, cozmo.objects.CustomObject):
            t = transform.quat2rot(*custom_obj.pose.rotation.q0_q1_q2_q3).transpose()
        else:
            t = transform.identity()
        rotmat = array.array('f',t.flatten()).tobytes()
        glMultMatrixf(rotmat)
        comparable = True # obj.pose.origin_id == 0 or obj.pose.is_comparable(self.robot.pose)
        obj_color = color_orange
        highlight = True # obj.is_visible
        if comparable:
            self.make_cube(size, highlight=highlight, color=obj_color)
        else:
            self.make_cube(size, body=False, highlight=False, color=obj_color)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_cylinder(self,radius=10,height=25, highlight=True, color=None):
        if color is None:
            color = (1,1,1)
        if len(color) == 3:
            color = (*color, 1)
        if not highlight:
            s = 0.5
            color = (color*s, color*s, color*s, color[3])
        glColor4f(*color)
        quadric = gluNewQuadric()
        gluQuadricOrientation(quadric, GLU_OUTSIDE)
        gluCylinder(quadric, radius, radius, height, 30, 20)
        glTranslatef(0, 0, height)
        gluDisk(quadric, 0, radius, 30, 1)

        # Draw the outline circles
        if highlight:
            color = (1, 1, 1, 1)
        else:
            color = (0, 0, 0, 1)

        r = radius + 0.1
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glColor4f(*color)
        glBegin(GL_LINE_LOOP)
        num_slices = 36
        for i in range(num_slices):
            theta = i * (360/num_slices) * (pi/180)
            glVertex3f(r * cos(theta), r*sin(theta), 0.)
        glEnd()
        glTranslatef(0, 0, -height)
        glBegin(GL_LINE_LOOP)
        for i in range(num_slices):
            theta = i * (360/num_slices) * (pi/180)
            glVertex3f(r * cos(theta), r*sin(theta), 0.)
        glEnd()
            

    def make_chip(self,chip):
        global gl_lists
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        glTranslatef(chip.x, chip.y, chip.z)
        self.make_cylinder(chip.radius, chip.thickness,
                           color=(0,0.8,0), highlight=True)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_wall(self,wall_obst):
        global gl_lists
        wall_spec = worldmap.wall_marker_dict[wall_obst.id]
        half_length = wall_obst.length / 2
        half_height = wall_obst.height / 2
        door_height = wall_obst.door_height
        widths = []
        last_x = -half_length
        edges = [ [0, -half_length, door_height/2, 1.] ]
        for (center,width) in wall_spec.doorways:
            left_edge = center - width/2 - half_length
            edges.append([0., left_edge, door_height/2, 1.])
            widths.append(left_edge - last_x)
            right_edge = center + width/2 - half_length
            edges.append([0., right_edge, door_height/2, 1.])
            last_x = right_edge
        edges.append([0., half_length, door_height/2, 1.])
        widths.append(half_length-last_x)
        edges = np.array(edges).T
        edges = transform.aboutZ(wall_obst.theta).dot(edges)
        edges = transform.translate(wall_obst.x,wall_obst.y).dot(edges)
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        for i in range(0,len(widths)):
            center = edges[:, 2*i : 2*i+2].mean(1).reshape(4,1)
            dimensions=(4.0, widths[i], wall_obst.door_height)
            glPushMatrix()
            glTranslatef(*center.flatten()[0:3])
            glRotatef(wall_obst.theta*180/pi, 0, 0, 1)
            self.make_cube(size=dimensions, color=color_red)
            glPopMatrix()
        # Make the transom
        glPushMatrix()
        transom_height = wall_obst.height - wall_obst.door_height
        z = wall_obst.door_height + transom_height/2
        glTranslatef(wall_obst.x, wall_obst.y, z)
        glRotatef(wall_obst.theta*180/pi, 0, 0, 1)
        self.make_cube(size=(4.0, wall_obst.length, transom_height),
                       edges=False, color=color_red)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_floor(self):
        global gl_lists
        floor_size = (2000, 2000, 1)
        blip = floor_size[2]
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        glTranslatef(0., 0., -blip)
        self.make_cube(floor_size, highlight=None, color=color_gray)
        glTranslatef(0., 0., 2.*blip)
        glColor4f(*color_light_gray,1)
        for x in range(-floor_size[0]//2, floor_size[0]//2+1, 100):
            glBegin(GL_LINES)
            glVertex3f(x,  floor_size[1]//2, 0)
            glVertex3f(x, -floor_size[1]//2, 0)
            glEnd()
        for y in range(-floor_size[1]//2, floor_size[1]//2+1, 100):
            glBegin(GL_LINES)
            glVertex3f( floor_size[0]/2, y, 0)
            glVertex3f(-floor_size[0]/2, y, 0)
            glEnd()
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_charger(self):
        charger = self.robot.world.charger
        if (not charger) or (not charger.pose) or not charger.pose.is_valid: return None
        comparable = charger.pose.is_comparable(self.robot.pose)
        highlight = charger.is_visible or (self.robot.is_on_charger and comparable)
        global gl_lists
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        p = charger.pose.position.x_y_z
        glTranslatef(*p)
        glRotatef(charger.pose.rotation.angle_z.degrees, 0, 0, 1)
        glTranslatef(charger_bed_size_mm[0]/2,
                     0,
                     charger_bed_size_mm[2]/2)
        glRotatef(180, 0, 0, 1) # charger "front" is opposite robot "front"
        if comparable:
            self.make_cube(charger_bed_size_mm, highlight=highlight)
        else:
            self.make_cube(charger_bed_size_mm, body=False, \
                      highlight=False, color=color_white)
        glTranslatef(
            (charger_back_size_mm[0]-charger_bed_size_mm[0])/2,
            0,
            charger_back_size_mm[2]/2)
        if comparable:
            self.make_cube(charger_back_size_mm, highlight=highlight)
        else:
            self.make_cube(charger_back_size_mm, body=False, \
                           highlight=True, color=color_white)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_cozmo_robot(self):
        global gl_lists
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        cur_pose = self.robot.world.particle_filter.pose
        p = (cur_pose[0], cur_pose[1], self.robot.pose.position.z)
        glTranslatef(*p)
        glTranslatef(*robot_body_offset_mm)
        glRotatef(cur_pose[2]*180/pi, 0, 0, 1)
        self.make_cube(robot_body_size_mm, highlight=self.robot.is_on_charger)
        h = robot_head_offset_mm
        glTranslatef(*h)
        glRotatef(-self.robot.head_angle.degrees, 0, 1, 0)
        self.make_cube(robot_head_size_mm, highlight=self.robot.is_on_charger)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_axes(self):
        global gl_lists
        if not self.show_axes: return None
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        len = axis_length
        w = axis_width
        glTranslatef(len/2., 0., 0.)
        self.make_cube((len,w,w), highlight=True, color=color_red, edges=False)
        glPopMatrix()
        glPushMatrix()
        glTranslatef(0., len/2., 0.)
        self.make_cube((w,len,w), highlight=True, color=color_green, edges=False)
        glPopMatrix()
        glPushMatrix()
        glTranslatef(0., 0., len/2.)
        self.make_cube((w,w,len), highlight=True, color=color_blue, edges=False)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_gazepoint(self):
        global gl_lists
        c = glGenLists(1)
        glNewList(c, GL_COMPILE)
        glPushMatrix()
        glTranslate(fixation_point[0], fixation_point[1], fixation_point[2])
        s = 3.
        self.make_cube((s,s,s), highlight=True, color=(1.0, 0.9, 0.1), edges=False)
        glPopMatrix()
        glEndList()
        gl_lists.append(c)

    def make_objects(self):
        items = tuple(self.robot.world.world_map.objects.items())
        for (key,obj) in items:
            if isinstance(obj, worldmap.LightCubeObj):
                self.make_light_cube(key,obj)
            elif isinstance(obj, worldmap.CustomCubeObj):
                self.make_custom_cube(key,obj)
            elif isinstance(obj, worldmap.WallObj):
                self.make_wall(obj)
            elif isinstance(obj, worldmap.ChipObj):
                self.make_chip(obj)

    def make_shapes(self):
        global gl_lists
        gl_lists = []
        self.make_axes()
        self.make_gazepoint()
        self.make_objects()  # walls, light cubes, custom cubes, and chips
        charger = self.make_charger()
        cozmo_robot = self.make_cozmo_robot()
        floor = self.make_floor()

    def del_shapes(self):
        global gl_lists
        for id in gl_lists:
            glDeleteLists(id,1)

    # ================ Window Setup ================

    def window_creator(self):
        global WINDOW
        WINDOW = opengl.create_window(self.windowName, (self.width,self.height))        
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyPressed)
        glutSpecialFunc(self.specialKeyPressed)
        glViewport(0,0,self.width,self.height)
        glClearColor(*self.bgcolor, 0)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        # Enable transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    def start(self): # Displays in background
        if not WINDOW:
            opengl.CREATION_QUEUE.append(self.window_creator)
            while not WINDOW:
                time.sleep(0.1)
        print("Type 'h' in the world map window for help.")

    def display(self):
        global gl_lists
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        field_of_view = 50 # degrees
        near_clip = 5
        far_clip = 600 # 20.0
        gluPerspective(field_of_view, self.aspect, near_clip, far_clip)
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
        self.make_shapes()
        for id in gl_lists:
            glCallList(id)
        glutSwapBuffers()
        self.del_shapes()

    def keyPressed(self, key, x, y):
        if ord(key) == 27:
            print("Use 'exit' to quit.")
            #return
        global fixation_point, camera_rotation, camera_distance, print_camera
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
            self.show_axes = not self.show_axes
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
        self.display()

    def specialKeyPressed(self, key, x, y):
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
        self.display()

    def reshape(self, width, height):
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        glViewport(0, 0, width, height)

