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

    def drawRectangle(self,center,width=0.08,height=None,angle=0,color=(1,1,1),fill=True):
        #default to solid color
        if len(color)==3:
          color = (color[0],color[1],color[2],1)

        #default to square window
        if height is None:
          height = width

        #calculate offsets
        w = width/2
        h = height/2
        v1 = (-w,-h)
        v2 = (w,-h)
        v3 = (w,h)
        v4 = (-w,h)

        #rotate points
        rad = math.radians(angle)
        tcos = math.cos(rad)
        tsin = math.sin(rad)
        v1 = (tcos*v1[0]-tsin*v1[1],tsin*v1[0]+tcos*v1[1])
        v2 = (tcos*v2[0]-tsin*v2[1],tsin*v2[0]+tcos*v2[1])
        v3 = (tcos*v3[0]-tsin*v3[1],tsin*v3[0]+tcos*v3[1])
        v4 = (tcos*v4[0]-tsin*v4[1],tsin*v4[0]+tcos*v4[1])

        #translate points
        (cx,cy) = center
        v1 = (v1[0]+cx,v1[1]+cy)
        v2 = (v2[0]+cx,v2[1]+cy)
        v3 = (v3[0]+cx,v3[1]+cy)
        v4 = (v4[0]+cx,v4[1]+cy)

        #draw rectangle
        if fill:
          glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
        glColor4f(color[0],color[1],color[2],color[3])
        glBegin(GL_QUADS)
        glVertex2f(v1[0],v1[1])
        glVertex2f(v2[0],v2[1])
        glVertex2f(v3[0],v3[1])
        glVertex2f(v4[0],v4[1])
        glEnd()
        if fill:
          glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)

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
            self.drawTriangle((p.x,p.y), angle=math.degrees(p.theta),
                              color=(1,0,0), fill=True)
        # Draw the landmarks
        if self.landmarks:
            for (id,coords) in self.landmarks.items():
                glPushMatrix()
                self.drawRectangle(coords,50,color=(0,1,0))
                glColor4f(0., 0., 0., 1.)
                glTranslatef(coords[0], coords[1], 0.)
                glRotatef(-90, 0., 0., 1.)
                glScalef(0.1,0.1,0.1)
                glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(ascii(id)))
                glPopMatrix()

        # Draw the robot at the best particle location
        est = self.robot.world.particle_filter.pose_estimate()
        rx = est[0]
        ry = est[1]
        rtheta = math.degrees(est[2])
        self.drawTriangle((rx,ry),scale=2,angle=rtheta, color=(0,0,1,0.5))

        #blocks
        #self.drawRectangle((150,0),50,color=(1,0,0,.2))
        #self.drawRectangle((0,75),50,color=(0,1,0,0.2))
        #self.drawRectangle((0,-75),50,color=(0,0,1,0.2))
        #self.drawRectangle((0.75,0.75),0.08,color=(0,1,0))

        glutSwapBuffers()

    def reshape(self,width,height):
        glViewport(0,0,width,height)
        self.width = width
        self.height = height
        self.aspect = self.width/self.height
        self.display()
        glutPostRedisplay()

    def keyPressed(self,key,mouseX,mouseY):
        global particles
        if(key == b'e'): #evaluate
            self.robot.world.particle_filter.sensor_model.force_eval = True
        if(key == b'r'): #resample
            self.robot.world.particle_filter.resample()
        if(key == b'w'): #forward
            self.Forward(10).now()
        if(key == b's'): #back
            self.Forward(-10).now()
        if(key == b'a'): #left
            self.Turn(45).now()
        if(key == b'd'): #left
            self.Turn(-45).now()
        if(key == b'v'): #left
            print('weight_variance =', self.robot.world.particle_filter.weight_variance())

        if(key == b'q'): #kill window
            glutDestroyWindow(self.window)
            glutLeaveMainLoop()
        print('Pose = ', self.robot.world.particle_filter.pose_estimate())



#dummy program to show off viewer
if __name__ == '__main__':
    import random,time #just for demo purposes!
    viewer = OpenGLViewer(bgcolor=(0.4,0.4,0.4)) #create window
    viewer.startThread() #background window and begin displaying
    print("main program begins.")
    while viewer.thread.is_alive():
      particles.append(Particle(random.random(),random.random(),theta=random.random()*360))
      time.sleep(1)
    print("Program terminated")
