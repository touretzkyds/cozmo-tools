"""
Particle filter display in OpenGL.
"""

from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *

from threading import Thread  # for backgrounding window
import math
import array

class ParticleViewer():
    def __init__(self, robot,
                 width=512,height=512,
                 windowName = "particle viewer",
                 bgcolor = (0,0,0)):
        self.robot = robot
        self.particles = robot.world.particle_filter.particles
        self.landmarks = robot.world.particle_filter.sensor_model.landmarks
        self.width = width
        self.height = height
        self.bgcolor = bgcolor
        self.aspect = self.width/self.height
        self.windowName = windowName
        self.thread = None

        # OpenGL params
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(self.width,self.height)

        # Default to drawing outlines of shapes
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

        # Killing window should not directly kill main program
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)

    def start(self):
        from cozmo_fsm.nodes import Forward, Turn
        self.Forward = Forward
        self.Turn = Turn
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

    def startThread(self): # Displays in background
        self.thread = Thread(target=self.start)
        self.thread.daemon = True #ending fg program will kill bg program
        self.thread.start()

    def drawRectangle(self,center,width=10,height=None,angle=0,color=(1,1,1),fill=True):
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

    def drawTriangle(self,center,scale=1,angle=0,color=(1,1,1),fill=True):
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
        if self.landmarks:
            for (id,specs) in self.landmarks.items():
                coords = specs[0]
                angle = specs[1]
                glPushMatrix()
                if id in self.robot.world.aruco.seenMarkers:
                    color = (0.5, 1, 0.3, 0.75)
                else:
                    color = (0, 0.5, 0, 0.75)
                self.drawRectangle(coords,20,50,angle=angle,color=color)
                glColor4f(0., 0., 0., 1.)
                glTranslatef(coords[0], coords[1], 0.)
                glRotatef(angle-90, 0., 0., 1.)
                glTranslatef(0., -5., 0.) 
                glScalef(0.1,0.1,0.1)
                glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(ascii(id)))
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
        for p in self.particles:
            pscale = 1 - p.weight
            color=(1,pscale,pscale)
            self.drawTriangle((p.x,p.y), angle=math.degrees(p.theta),
                              color=color, fill=True)

        # Draw the robot at the best particle location
        est = self.robot.world.particle_filter.pose_estimate()
        rx = est[0]
        ry = est[1]
        rtheta = math.degrees(est[2])
        self.drawTriangle((rx,ry),scale=2,angle=rtheta, color=(0,0,1,0.5))

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
            self.Forward(10).now()
        elif key == b's':     # back
            self.Forward(-10).now()
        elif key == b'a':     # left
            self.Turn(22.5).now()
        elif key == b'd':     # right
            self.Turn(-22.5).now()
        elif key == b'z':     # randomize
            pf.initializer.initialize(pf.particles)
        elif key == b'v':     # display weight variance
            var = pf.update_weights()
            minw = min(p.weight for p in pf.particles)
            maxw = max(p.weight for p in pf.particles)
            print('weights:','  min =', minw, '  max =', maxw,'  variance =', var)

        if key == b'q': #kill window
            glutDestroyWindow(self.window)
            glutLeaveMainLoop()
        est = self.robot.world.particle_filter.pose_estimate()
        hdg = math.degrees(est[2])
        print('Pose = (%5.1f, %5.1f) @ %3d deg.' % (est[0], est[1], hdg))
