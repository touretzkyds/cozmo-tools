"""
Particle filter display in OpenGL.
"""

from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

from threading import Thread  # for backgrounding window
import math
import array
import numpy as np

import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees

RUNNING = False

class ParticleViewer():
    def __init__(self, robot,
                 width=512, height=512, scale=1.0,
                 windowName = "particle viewer",
                 bgcolor = (0,0,0)):
        self.robot = robot
        self.width = width
        self.height = height
        self.bgcolor = bgcolor
        self.aspect = self.width/self.height
        self.translation = [0., 0.]  # Translation in mm
        self.scale = scale
        self.windowName = windowName
        self.thread = None

    def initialize_window(self):
        # OpenGL params
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(self.width,self.height)

        # Default to drawing outlines of shapes
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

        # Killing window should not directly kill main program
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)

        self.window = glutCreateWindow(self.windowName)
        glViewport(0,0,self.width,self.height)
        glClearColor(*self.bgcolor, 0)

        # Enable transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        # Function bindings
        glutReshapeFunc(self.reshape)
        glutIdleFunc(glutPostRedisplay) # Constantly update screen
        glutKeyboardFunc(self.keyPressed)
        glutSpecialFunc(self.specialKeyPressed)
        glutDisplayFunc(self.display)
        glutMainLoop()

    def start_thread(self): # Displays in background
        global RUNNING
        if RUNNING:
            return
        else:
            RUNNING = True
        self.thread = Thread(target=self.initialize_window)
        self.thread.daemon = True #ending fg program will kill bg program
        self.thread.start()
        print("Type 'h' in the particle viewer window for help.")

    def draw_rectangle(self, center, width=10, height=None,
                       angle=0, color=(1,1,1), fill=True):
        # Default to solid color and square window
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

    def draw_triangle(self,center,scale=1,angle=0,color=(1,1,1),fill=True):
        # Default to solid color
        if len(color) == 3:
          color = (color[0],color[1],color[2],1)

        glPushMatrix()
        if(fill):
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
        return

    def draw_landmarks(self):
        landmarks = self.robot.world.particle_filter.sensor_model.landmarks
        if landmarks:
            for (id,specs) in landmarks.items():
                coords = specs.position.x_y_z
                angle = specs.rotation.angle_z.degrees
                glPushMatrix()
                if isinstance(id, cozmo.objects.LightCube):
                    seen = id.is_visible
                    label = next(k for k,v in self.robot.world.light_cubes.items() if v==id)
                else:
                    seen = id in self.robot.world.aruco.seenMarkers
                    label = id
                if seen:
                    color = (0.5, 1, 0.3, 0.75)
                else:
                    color = (0, 0.5, 0, 0.75)
                self.draw_rectangle(coords,20,50,angle=angle,color=color)
                glColor4f(0., 0., 0., 1.)
                glTranslatef(*coords)
                glRotatef(angle-90, 0., 0., 1.)
                label_str = ascii(label)
                glTranslatef(3.-7*len(label_str), -5., 0.) 
                glScalef(0.1,0.1,0.1)
                for char in label_str:
                    glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(char))
                glPopMatrix()

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


        # Draw the particles
        for p in self.robot.world.particle_filter.particles:
            pscale = 1 - p.weight
            color=(1,pscale,pscale)
            self.draw_triangle((p.x,p.y), angle=math.degrees(p.theta),
                               color=color, fill=True)

        # Draw the robot at the best particle location
        est = self.robot.world.particle_filter.pose_estimate()
        rx = est[0]
        ry = est[1]
        rtheta = math.degrees(est[2])
        self.draw_triangle((rx,ry),scale=2,angle=rtheta, color=(0,0,1,0.5))

        # Draw the landmarks last, so they go on top of the particles
        self.draw_landmarks()

        glutSwapBuffers()

    def reshape(self,width,height):
        glViewport(0,0,width,height)
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.display()
        glutPostRedisplay()

    def report_variance(self,pf):
        weights = np.empty(pf.num_particles)
        for i in range(pf.num_particles):
            weights[i] = pf.particles[i].weight
        weights.sort()
        var = np.var(weights)
        print('weights:  min = %3.3e  max = %3.3e med = %3.3e  variance = %3.3e' %
              (weights[0], weights[-1], weights[pf.num_particles//2], var))

    def report_pose(self):
        est = self.robot.world.particle_filter.pose_estimate()
        hdg = math.degrees(est[2])
        print('Pose = (%5.1f, %5.1f) @ %3d deg.' % (est[0], est[1], hdg))

    async def forward(self,distance):
        handle = self.robot.drive_straight(distance_mm(distance), speed_mmps(50),
                                           in_parallel=True,
                                           should_play_anim=False)
        await handle.wait_for_completed()
        self.report_pose()

    async def turn(self,angle):
        handle = self.robot.turn_in_place(degrees(angle), in_parallel=True)
        await handle.wait_for_completed()
        self.report_pose()

    def keyPressed(self,key,mouseX,mouseY):
        pf = self.robot.world.particle_filter
        global particles
        if key == b'e':       # evaluate
            pf.sensor_model.evaluate(pf.particles,force=True)
            pf.update_weights()
        elif key == b'r':     # resample
            pf.sensor_model.evaluate(pf.particles,force=True)
            pf.update_weights()
            pf.resample()
        elif key == b'w':     # forward
            self.robot.loop.create_task(self.forward(10))
        elif key == b's':     # back
            self.robot.loop.create_task(self.forward(-10))
        elif key == b'a':     # left
            self.robot.loop.create_task(self.turn(22.5))
        elif key == b'd':     # right
            self.robot.loop.create_task(self.turn(-22.5))
        elif key == b'D':     # right
            self.robot.loop.create_task(self.turn(-90))
        elif key == b'z':     # randomize
            pf.initializer.initialize(pf.particles)
        elif key == b'v':     # display weight variance
            self.report_variance(pf)
        elif key == b'q': #kill window
        elif key == b'z':     # initialize
            pf.initializer.initialize(pf.particles)
        elif key == b'v':     # display weight variance
            self.report_variance(pf)
            return
        elif key == b' ':     # update display (useful when normal update is disabled)
            glutPostRedisplay()
        elif key == b'+':     # zoom in
            self.scale *= 1.25
            self.print_display_params()
            return
        elif key == b'-':     # zoom out
            self.scale /= 1.25
            self.print_display_params()
            return
        elif key == b'c':     # center map
            self.translation = [0., 0.]
            self.print_display_params()
            return
        elif key == b'h':     # print help
            self.print_help()
            return
        elif key == b'q':     #kill window
            glutDestroyWindow(self.window)
            glutLeaveMainLoop()
        self.report_pose()

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
        self.print_display_params()

    def print_display_params(self):
        print('scale=%.2f translation=[%.1f, %.1f]' %
              (self.scale, *self.translation))

    def print_help(self):
        print("""
Particle viewer commands:
  w/a/s/d    Drive robot +/- 10 mm or turn +/- 22.5 degrees
  W/A/S/D    Drive robot +/- 25 mm or turn +/- 90 degrees
    e        Evaluate particles using current sensor info
    r        Resample particles (evaluates first)
    z        Reset particles (randomize, or all 0 for SLAM)
  arrows     Translate the view up/down/left/right
    c        Center the view (zero translation)
    +        Zoom in
    -        Zoom out
    h        Print this help text
""")
