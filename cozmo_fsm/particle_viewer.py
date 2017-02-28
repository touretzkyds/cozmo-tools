"""
Particle filter display in OpenGL.
"""

from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

from threading import Thread  # for backgrounding window
import math
from math import sin, cos, pi, atan2, sqrt
import array
import numpy as np

import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees

RUNNING = False

class ParticleViewer():
    def __init__(self, robot,
                 width=512, height=512,
                 windowName = "particle viewer",
                 bgcolor = (0,0,0)):
        self.robot=robot
        self.width = width
        self.height = height
        self.bgcolor = bgcolor
        self.aspect = self.width/self.height
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

    def draw_ellipse(self, center, scale, orient=0, color=(1,1,1), fill=False):
        if len(color) == 3:
            color = (*color,1)
        glPushMatrix()
        glTranslatef(*center,0)
        glRotatef(orient,0,0,1)
        glColor4f(*color)
        if fill:
            glBegin(GL_TRIANGLE_FAN)
            glVertex2f(0,0)
        else:
            glBegin(GL_LINE_LOOP)
        for t in range(0,361):
            theta = t/180*pi
            glVertex2f(scale[0]*cos(theta), scale[1]*sin(theta))
        glEnd()
        glPopMatrix()

    def draw_wedge(self, center, radius, orient, span, color=(1,1,1), fill=True):
        if len(color) == 3:
            color = (*color,1)
        glPushMatrix()
        glTranslatef(*center,0)
        glRotatef(orient,0,0,1)
        glColor4f(*color)
        if fill:
            glBegin(GL_TRIANGLE_FAN)
        else:
            glBegin(GL_LINE_LOOP)
        glVertex2f(0,0)
        for t in range(round(-span/2), round(span/2)):
            theta = t/180*pi
            glVertex2f(radius*cos(theta), radius*sin(theta))
        glEnd()
        glPopMatrix()
        

    def draw_landmarks(self):
        landmarks = self.robot.world.particle_filter.sensor_model.landmarks
        if not landmarks: return
        for (id,specs) in landmarks.items():
            coords = (specs.position.x, specs.position.y, 0.)
            angle = specs.rotation.angle_z.degrees
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
            glPushMatrix()
            glColor4f(*color)
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

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


        # Draw the particles
        for p in self.robot.world.particle_filter.particles:
            pscale = 1 - p.weight
            color=(1,pscale,pscale)
            self.draw_triangle((p.x,p.y), angle=math.degrees(p.theta),
                               color=color, fill=True)

        # Draw the robot at the best particle location
        (rx,ry,theta) = self.robot.world.particle_filter.pose_estimate()
        hdg = math.degrees(theta)
        self.draw_triangle((rx,ry),scale=2,angle=hdg, color=(0,0,1,0.5))

        # Draw the error ellipse and heading error wedge
        (xy_var, theta_var) = self.robot.world.particle_filter.variance_estimate()
        (w,v) = np.linalg.eigh(xy_var)
        alpha = atan2(v[1,0],v[0,0])
        self.draw_ellipse((rx,ry), w**0.5, alpha/pi*180, color=(0,1,1))
        self.draw_wedge((rx,ry), 25wwwww, hdg, max(5, sqrt(theta_var)*360),
                        color=(0,1,1,0.4))
        

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
        (xy_var,theta_var) = pf.variance_estimate()
        print ('xy_var=', xy_var, '  theta_var=', theta_var)
        
    def report_pose(self):
        (x,y,theta) = self.robot.world.particle_filter.pose_estimate()
        hdg = math.degrees(theta)
        print('Pose = (%5.1f, %5.1f) @ %3d deg.' % (x, y, hdg))

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
        elif key == b'z':     # randomize
            pf.initializer.initialize(pf.particles)
        elif key == b'v':     # display weight variance
            self.report_variance(pf)
        elif key == b'q': #kill window
            glutDestroyWindow(self.window)
            glutLeaveMainLoop()
        self.report_pose()

