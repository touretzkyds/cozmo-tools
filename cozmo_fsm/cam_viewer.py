"""
OpenGL based CamViewer
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
# from __future__ import unicode_literals
import numpy as np
from PIL import Image, ImageEnhance
import time
import argparse

# import random as rng

from scipy.ndimage.measurements import label

nRange = 1.0

import cv2
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    pass

import math
import random

from . import opengl
from . import transform
from . import program

#For capturing images
global snapno, path
snapno = 0
path = 'snap/'

WINDOW = None


class CamViewer():
    prog_start = False
    #Incoming image stream as nparray
    incom_image = None

    def __init__(self, robot, image=None, width=640, height=480,
                 windowName="Cozmo's World",
                 bgcolor=(0, 0, 0)):
        self.robot = robot
        self.image = image
        self.is_incom_image = True if CamViewer.incom_image is not None else False
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.windowName = windowName
        self.bgcolor = bgcolor
        self.translation = [0., 0.]  # Translation in mm
        self.scale = 1
        self.show_axes = True
        self.show_memory_map = False


    def process_image(self):
        if not self.is_incom_image and CamViewer.incom_image is None:
            image = cv2.resize(np.array(self.robot.world.latest_image.raw_image), (self.width, self.height))
        else:
            image = CamViewer.incom_image
        self.image = image.copy()

        cozmo_fsm = program.running_fsm
        curim = np.array(self.image) #cozmo-raw image
        gray = cv2.cvtColor(curim,cv2.COLOR_BGR2GRAY)

        # Aruco image processing
        if cozmo_fsm.aruco:
            cozmo_fsm.robot.world.aruco.process_image(gray)
        # Other image processors can run here if the user supplies them.
        cozmo_fsm.user_image(curim,gray)
        # Done with image processing

        # Annotate and display image if requested
        if cozmo_fsm.force_annotation or cozmo_fsm.cam_viewer is not None:
            scale = cozmo_fsm.annotated_scale_factor
            # Apply Cozmo SDK annotations and rescale.
            if cozmo_fsm.annotate_sdk:
                coz_ann = self.robot.world.latest_image.annotate_image(scale=scale)
                annotated_im = np.array(coz_ann)
            elif scale != 1:
                shape = curim.shape
                dsize = (scale*shape[1], scale*shape[0])
                annotated_im = cv2.resize(curim, dsize)
            else:
                annotated_im = curim
            # Yellow viewer crosshairs
            if cozmo_fsm.viewer_crosshairs:
                shape = annotated_im.shape
                cv2.line(annotated_im, (int(shape[1]/2),0), (int(shape[1]/2),shape[0]), (255,255,0), 1)
                cv2.line(annotated_im, (0,int(shape[0]/2)), (shape[1],int(shape[0]/2)), (255,255,0), 1)
            # Aruco annotation
            if cozmo_fsm.aruco and \
                   len(cozmo_fsm.robot.world.aruco.seen_marker_ids) > 0:
                scale = 1
                annotated_im = cozmo_fsm.robot.world.aruco.annotate(annotated_im,scale)
            # Other annotators can run here if the user supplies them.
            annotated_im = cozmo_fsm.user_annotate(annotated_im)
            # Done with annotation
            annotated_im = cv2.cvtColor(annotated_im,cv2.COLOR_RGB2BGR)
            image = annotated_im

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height,0,GL_RGB, GL_UNSIGNED_BYTE, image)
        glutPostRedisplay()

    # ================ Window Setup ================
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
        glutKeyboardFunc(self.keyPressed)
        glutKeyboardUpFunc(self.keyPressedUp)
        glutSpecialFunc(self.specialKeyPressed)
        glutSpecialUpFunc(self.specialKeyUp)

    def start(self):  # Displays in background
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)
            CamViewer.prog_start = True
            self.robot.set_head_angle(cozmo.util.Angle(0)).wait_for_completed()
            while not WINDOW:
                time.sleep(0.1)

    def display(self):
        self.process_image()
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

    def keyPressed(self, key, x, y):
        if ord(key) == 27:
            print("Use 'exit' to quit.")
            #return
        if key == b'c':
            print("Taking a snap")
            self.capture()
        if key == b'q':
            self.robot.move_head(0.5)
        if key == b'a':
            self.robot.move_head(-0.5)
        self.display()

    def keyPressedUp(self, key, x, y):
        if key == b'q':
            self.robot.move_head(0)
        if key == b'a':
            self.robot.move_head(0)
        self.display()


    def specialKeyPressed(self, key, x, y):
        global leftorrightindicate, globthres
        if key == GLUT_KEY_LEFT:
            self.robot.drive_wheels(-100, 100)
            leftorrightindicate = True
            globthres=100
        elif key == GLUT_KEY_RIGHT:
            self.robot.drive_wheels(100, -100)
            leftorrightindicate = True
            globthres = 100
        elif key == GLUT_KEY_UP:
            self.robot.drive_wheels(200, 200)
            leftorrightindicate = False
            globthres = 100
        elif key == GLUT_KEY_DOWN:
            self.robot.drive_wheels(-200, -200)
            leftorrightindicate = True
            globthres = 100
        glutPostRedisplay()

    def specialKeyUp(self, key, x, y):
        global leftorrightindicate, go_forward
        self.robot.drive_wheels(0, 0)
        leftorrightindicate = True
        go_forward = GLUT_KEY_UP
        glutPostRedisplay()

    def capture(self, img=None, name='cozmo_snap'):
        global snapno, path
        if not os.path.exists(path):
                os.makedirs(path)

        if not self.is_incom_image and CamViewer.incom_image is None:
            image = cv2.resize(np.array(self.image), (self.width, self.height))
        else:
            image = CamViewer.incom_image

        if img is not None:
            image = img
        Image.fromarray(image).save(path + '/' + name + str(snapno) + '.jpg')
        snapno +=1
