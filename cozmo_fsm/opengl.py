"""
  Common code for OpenGL window management
"""

from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

from threading import Thread  # for backgrounding window

INIT_DONE = False

def init():
    global INIT_DONE
    if not INIT_DONE:
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)

        # Default to drawing outlines of shapes
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

        # Killing window should not directly kill main program
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)
        INIT_DONE = True

def create_window(name,size=(500,500)):
    init()
    glutInitWindowSize(*size)
    w = glutCreateWindow(name)
    return w

def mainloop():
    glutMainLoop()

def launch_main_loop():
    thread = Thread(target=mainloop)
    thread.daemon = True #ending fg program will kill bg program
    thread.start()
