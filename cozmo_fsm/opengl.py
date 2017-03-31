"""
  Common code for OpenGL window management
"""

from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

from threading import Thread  # for backgrounding window

INIT_DONE = False
MAIN_LOOP_LAUNCHED = False

# Maintain a registry of display functions for our windows
WINDOW_REGISTRY = []

def init():
    global INIT_DONE
    if not INIT_DONE:
        INIT_DONE = True
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)

        # Default to drawing outlines of shapes
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

        # Killing window should not directly kill main program
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)

def create_window(name,size=(500,500)):
    global WINDOW_REGISTRY
    init()
    glutInitWindowSize(*size)
    w = glutCreateWindow(name)
    WINDOW_REGISTRY.append(w)
    return w

def idle():
    for window in WINDOW_REGISTRY:
        glutSetWindow(window)
        glutPostRedisplay()

def launch_main_loop():
    if not INIT_DONE: return   # no windows were created
    global MAIN_LOOP_LAUNCHED
    if MAIN_LOOP_LAUNCHED: return
    MAIN_LOOP_LAUNCHED = True
    glutIdleFunc(idle)
    thread = Thread(target=glutMainLoop)
    thread.daemon = True #ending fg program will kill bg program
    thread.start()
