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

# List of window creation requests that need to be satisfied
CREATION_QUEUE = []

def init():
    global INIT_DONE
    if not INIT_DONE:
        INIT_DONE = True
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)

        # Killing window should not directly kill main program
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)

def create_window(name,size=(500,500)):
    global WINDOW_REGISTRY
    init()
    glutInitWindowSize(*size)
    w = glutCreateWindow(name)
    WINDOW_REGISTRY.append(w)
    return w

def event_loop():
    global CREATION_QUEUE
    while True:
        for window in WINDOW_REGISTRY:
            glutSetWindow(window)
            glutPostRedisplay()
        glutMainLoopEvent()
        # Process any requests for new windows
        queue = CREATION_QUEUE
        CREATION_QUEUE = []
        for req in queue:
            req()  # invoke the window creator

def launch_event_loop():
    global MAIN_LOOP_LAUNCHED
    if MAIN_LOOP_LAUNCHED: return
    MAIN_LOOP_LAUNCHED = True
    thread = Thread(target=event_loop)
    thread.daemon = True #ending fg program will kill bg program
    thread.start()
