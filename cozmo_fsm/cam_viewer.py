"""
OpenGL based CamViewer
"""

import time
import numpy as np

try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    pass

from . import opengl

WINDOW = None

# width = 1280
# height = 720
nRange = 1.0

class CamViewer():
    def __init__(self, robot, width=512, height=512,
                 windowName="Cozmo's World",
                 bgcolor=(0, 0, 0)):
        self.robot = robot
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.windowName = windowName
        self.bgcolor = bgcolor
        self.translation = [0., 0.]  # Translation in mm
        self.scale = 1
        self.show_axes = True
        self.show_memory_map = False

    # ================ Window Setup ================

    def idle(self):
        image = np.array(self.robot.world.latest_image.raw_image)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 320, 240,0,GL_RGB, GL_UNSIGNED_BYTE, image)
        glutPostRedisplay()

    def window_creator(self):
        global WINDOW
        glutInit(sys.argv)
        WINDOW = opengl.create_window(
            bytes(self.windowName, 'utf-8'), (self.width, self.height))
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.width, self.height)
        glutInitWindowPosition(100, 100)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutIdleFunc(self.idle)
        glutMainLoop()

    def start(self):  # Displays in background
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)
            while not WINDOW:
                time.sleep(0.1)
        print("Type 'h' in the world map window for help.")

    def display(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_TEXTURE_2D)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

        # Set Projection Matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(0, self.width, 0, self.height)

        glMatrixMode(GL_TEXTURE)
        glLoadIdentity()
        glScalef(1.0, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0)
        glVertex2f(0.0, 0.0)
        glTexCoord2f(1.0, 0.0)
        glVertex2f(self.width, 0.0)
        glTexCoord2f(1.0, 1.0)
        glVertex2f(self.width, self.height)
        glTexCoord2f(0.0, 1.0)
        glVertex2f(0.0, self.height)
        glEnd()

        glFlush()
        glutSwapBuffers()

    def reshape(self, w, h):
        if h == 0:
            h = 1

        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)

        glLoadIdentity()

        if w <= h:
            glOrtho(-nRange, nRange, -nRange*h/w, nRange*h/w, -nRange, nRange)
        else:
            glOrtho(-nRange*w/h, nRange*w/h, -nRange, nRange, -nRange, nRange)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
