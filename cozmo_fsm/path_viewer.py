"""
Path planner display in OpenGL.
"""

try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    pass

import time
from math import pi, sin, cos
import array
import numpy as np
import platform

WINDOW = None

from . import opengl
from .rrt import RRTNode
from .rrt_shapes import *
from . import transform
from .transform import wrap_angle

the_rrt = None
the_items = []  # each item is a tuple (tree,color)

help_text = """
Path viewer commands:
  arrows   Translate the view up/down/left/right
  Home     Center the view (zero translation)
  <        Zoom in
  >        Zoom out
  o        Show objects
  space    Toggle redisplay (for debugging)
  h        Print this help text
"""

help_text_mac = """
Path viewer commands:
  arrows           Translate the view up/down/left/right
  fn + left-arrow  Center the view (zero translation)
  option + <       Zoom in
  option + >       Zoom out
  o                Show objects
  space            Toggle redisplay (for debugging)
  option + h       Print this help text
"""


class PathViewer():
    def __init__(self, robot, rrt,
                 width=512, height=512,
                 windowName = "path viewer",
                 bgcolor = (0,0,0)):
        global the_rrt, the_items
        the_rrt = rrt
        the_items = []
        self.robot = robot
        self.width = width
        self.height = height
        self.bgcolor = bgcolor
        self.aspect = self.width/self.height
        self.windowName = windowName
        self.translation = [0., 0.]  # Translation in mm
        self.scale = 0.64

    def window_creator(self):
        global WINDOW
        WINDOW = opengl.create_window(bytes(self.windowName,'utf-8'), (self.width,self.height))
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyPressed)
        glutSpecialFunc(self.specialKeyPressed)
        glViewport(0,0,self.width,self.height)
        glClearColor(*self.bgcolor, 0)
        # Enable transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    def start(self): # Displays in background
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)
            while not WINDOW:
                time.sleep(0.1)
        if platform.system() == 'Darwin':
            print("Type 'option' + 'h' in the path viewer window for help.")
        else:
            print("Type 'h' in the path viewer window for help.")

    def clear(self):
        global the_items
        the_items = []

    def set_rrt(self,new_rrt):
        global the_rrt
        the_rrt = new_rrt

    def draw_rectangle(self, center, width=4, height=None,
                       angle=0, color=(1,1,1), fill=True):
        # Default to solid color and square shape
        if len(color)==3:
          color = (color[0],color[1],color[2],1)
        if height is None:
          height = width

        # Calculate vertices as offsets from center
        w = width/2; h = height/2
        v1 = (-w,-h); v2 = (w,-h); v3 = (w,h); v4 = (-w,h)

        # Draw the rectangle
        glPushMatrix()
        if fill:
            glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
        else:
            glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
        glColor4f(color[0],color[1],color[2],color[3])
        glTranslatef(*center,0)
        glRotatef(angle,0,0,1)
        glBegin(GL_QUADS)
        glVertex2f(*v1)
        glVertex2f(*v2)
        glVertex2f(*v3)
        glVertex2f(*v4)
        glEnd()
        glPopMatrix()

    def draw_circle(self,center,radius=1,color=(1,1,1),fill=True):
        if len(color) == 3:
            color = (*color,1)
        glColor4f(*color)
        if fill:
            glBegin(GL_TRIANGLE_FAN)
            glVertex2f(*center)
        else:
            glBegin(GL_LINE_LOOP)
        for angle in range(0,360):
            theta = angle/180*pi
            glVertex2f(center[0]+radius*cos(theta), center[1]+radius*sin(theta))
        glEnd()


    def draw_triangle(self,center,scale=1,angle=0,color=(1,1,1),fill=True):
        # Default to solid color
        if len(color) == 3:
          color = (*color,1)

        glPushMatrix()
        if fill:
          glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
        else:
          glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
        glColor4f(*color)
        glTranslatef(*center,0)
        glRotatef(angle,0,0,1)
        glBegin(GL_TRIANGLES)
        glVertex2f( 5.*scale,  0.)
        glVertex2f(-5.*scale, -3.*scale)
        glVertex2f(-5.*scale,  3.*scale)
        glEnd()
        glPopMatrix()

    def draw_line(self,pt1,pt2,color=(1,1,1,1)):
        if len(color) == 3:
            color = (*color,1)
        glBegin(GL_LINES)
        glColor4f(*color)
        glVertex2f(*pt1)
        glVertex2f(*pt2)
        glEnd()

    def draw_tree(self,tree,color):
        for node in tree:
            self.draw_node(node,color)

    def draw_node(self,node,color):
        self.draw_rectangle((node.x,node.y), color=color)
        if node.parent:
            if node.radius is None or node.radius == 0:
                self.draw_line((node.x,node.y), (node.parent.x,node.parent.y), color=color)
            else:
                color = (1, 1, 0.5)
                init_x = node.parent.x
                init_y = node.parent.y
                init_q = node.parent.q
                targ_q = node.q
                radius = node.radius
                dir = +1 if radius >= 0 else -1
                r = abs(radius)
                center = transform.translate(init_x,init_y).dot(
                    transform.aboutZ(init_q+dir*pi/2).dot(transform.point(r)))
                theta = wrap_angle(init_q - dir*pi/2)
                targ_theta = wrap_angle(targ_q - dir*pi/2)
                ang_step = 0.05 # radians
                while abs(theta - targ_theta) > ang_step:
                    theta = wrap_angle(theta + dir * ang_step)
                    cur_x = center[0,0] + r*cos(theta)
                    cur_y = center[1,0] + r*sin(theta)
                    self.draw_line((init_x,init_y), (cur_x,cur_y), color=color)
                    (init_x,init_y) = (cur_x,cur_y)

    def draw_robot(self,node):
        parts = the_rrt.robot_parts_to_node(node)
        for part in parts:
            if isinstance(part,Circle):
                self.draw_circle(center=(part.center[0,0],part.center[1,0]),
                                 radius=part.radius,
                                 color=(1,1,0,0.7), fill=False)
            elif isinstance(part,Rectangle):
                self.draw_rectangle(center=(part.center[0,0],part.center[1,0]),
                                    width=part.max_Ex-part.min_Ex,
                                    height=part.max_Ey-part.min_Ey,
                                    angle=part.orient*180/pi,
                                    color=(1,1,0,0.7), fill=False)

    def draw_obstacle(self,obst):
        if isinstance(obst,Circle):
            self.draw_circle(center=(obst.center[0,0],obst.center[1,0]),
                             radius=obst.radius,
                             color=(1,0,0,0.5), fill=True)
        elif isinstance(obst,Rectangle):
            width = obst.max_Ex - obst.min_Ex
            height = obst.max_Ey - obst.min_Ey
            if width <= 10*height:
                color = (1, 0, 0, 0.5)
            else:
                color = (1, 1, 0, 0.5)
            self.draw_rectangle(center=(obst.center[0], obst.center[1]),
                                angle=obst.orient*(180/pi),
                                width=width, height=height, color=color, fill=True)

    def add_tree(self, tree, color):
        global the_items
        the_items.append((tree,color))

    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        w = self.width / 2
        glOrtho(-w, w, -w, w, 1, -1)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glRotatef(90,0,0,1)
        glScalef(self.scale, self.scale, self.scale)
        glTranslatef(-self.translation[0], -self.translation[1], 0.)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.draw_rectangle(center=(0,0), angle=45, width=5, height=5, color=(0.9, 0.5, 0), fill=False)

        self.draw_tree(the_rrt.treeA, color=(0,1,0))
        self.draw_tree(the_rrt.treeB, color=(0,0,1))
        for (tree,color) in the_items:
            self.draw_tree(tree,color)

        for obst in the_rrt.obstacles:
            self.draw_obstacle(obst)

        #if the_rrt.start:
        #    self.draw_robot(the_rrt.start)
        pose = self.robot.world.particle_filter.pose
        self.draw_robot(RRTNode(x=pose[0], y=pose[1], q=pose[2]))

        glutSwapBuffers()

    def reshape(self,width,height):
        glViewport(0,0,width,height)
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.display()
        glutPostRedisplay()

    def keyPressed(self,key,mouseX,mouseY):
        # print(str(key), ord(key))
        if key == b'<':       # zoom in
            self.scale *= 1.25
            self.print_display_params()
            return
        elif key == b'>':     # zoom out
            self.scale /= 1.25
            self.print_display_params()
            return
        elif key == b'o':     # show objects
            self.robot.world.world_map.show_objects()
            return
        elif key == b'h':     # print help
            self.print_help()
            return

    def specialKeyPressed(self, key, mouseX, mouseY):
        # arrow keys for translation
        incr = 25.0    # millimeters
        if key == GLUT_KEY_UP:
            self.translation[0] += incr / self.scale
        elif key == GLUT_KEY_DOWN:
            self.translation[0] -= incr / self.scale
        elif key == GLUT_KEY_LEFT:
            self.translation[1] += incr / self.scale
        elif key == GLUT_KEY_RIGHT:
            self.translation[1] -= incr / self.scale
        elif key == GLUT_KEY_HOME:
            self.translation = [0., 0.]
            self.print_display_params()
        glutPostRedisplay()

    def print_display_params(self):
        print('scale=%.2f translation=[%.1f, %.1f]' %
              (self.scale, *self.translation))

    def print_help(self):
        if platform.system() == 'Darwin':
            print(help_text_mac)
        else:
            print(help_text)
