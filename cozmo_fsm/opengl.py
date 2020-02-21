"""
  Common code for OpenGL window management
"""

try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    pass

import time

from threading import Thread  # for backgrounding window

INIT_DONE = False
MAIN_LOOP_LAUNCHED = False

# Maintain a registry of display functions for our windows
WINDOW_REGISTRY = []

# List of window creation requests that need to be satisfied
CREATION_QUEUE = []

def init():
    global INIT_DONE, robot
    if not INIT_DONE:
        INIT_DONE = True
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH)

        # Killing window should not directly kill main program
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)
        launch_event_loop()

def create_window(name,size=(500,500)):
    global WINDOW_REGISTRY
    glutInitWindowSize(*size)
    w = glutCreateWindow(name)
    #print('request creation of window',w)
    WINDOW_REGISTRY.append(w)
    return w

def event_loop():
    while True:
        for window in WINDOW_REGISTRY:
            glutSetWindow(window)
            glutPostRedisplay()
        glutMainLoopEvent()
        process_requests()
        time.sleep(0.1)

def process_requests():
    global CREATION_QUEUE
    # Process any requests for new windows
    queue = CREATION_QUEUE
    CREATION_QUEUE = []
    for req in queue:
        req()  # invoke the window creator


def launch_event_loop():
    global MAIN_LOOP_LAUNCHED
    if MAIN_LOOP_LAUNCHED: return
    MAIN_LOOP_LAUNCHED = True
    print('launching opengl event loop')
    thread = Thread(target=event_loop)
    thread.daemon = True #ending fg program will kill bg program
    thread.start()
