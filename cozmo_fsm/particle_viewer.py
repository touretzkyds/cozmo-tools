"""
Particle filter display in OpenGL.
"""

try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    pass

import time
import math
from math import sin, cos, pi, atan2, sqrt
import array
import numpy as np

import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees

from . import opengl

REDISPLAY = True   # toggle this to suspend constant redisplay
WINDOW = None

class ParticleViewer():
    def __init__(self, robot,
                 width=512, height=512, scale=1.0,
                 windowName = "particle viewer",
                 bgcolor = (0,0,0)):
        self.robot=robot
        self.width = width
        self.height = height
        self.bgcolor = bgcolor
        self.aspect = self.width/self.height
        self.translation = [0., 0.]  # Translation in mm
        self.scale = scale
        self.windowName = windowName

    def window_creator(self):
        global WINDOW
        WINDOW = opengl.create_window(bytes(self.windowName, 'utf-8'), (self.width,self.height))
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyPressed)
        glutSpecialFunc(self.specialKeyPressed)
        glViewport(0,0,self.width,self.height)
        glClearColor(*self.bgcolor, 0)
        # Enable transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    def start(self): # Displays in background
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)
            while not WINDOW:
                time.sleep(0.1)
        print("Type 'h' in the particle viewer window for help.")

    def draw_rectangle(self, center, size=(10,10),
                       angle=0, color=(1,1,1), fill=True):
        # Default to solid color and square window
        if len(color)==3:
          color = (*color,1)

        # Calculate vertices as offsets from center
        w = size[0]/2; h = size[1]/2
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

    def draw_triangle(self, center, height=1, angle=0, tip_offset=0,
                      color=(1,1,1), fill=True):
        half = height / 2
        aspect = 3/5
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
        glTranslatef(tip_offset,0,0)
        glBegin(GL_TRIANGLES)
        glVertex2f( half,  0.)
        glVertex2f(-half, -aspect*half)
        glVertex2f(-half,  aspect*half)
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
        # Extract keys and values as quickly as we can because
        # dictionary can change while we're iterating.
        for (id,specs) in list(landmarks.items()):
            color = None
            if isinstance(id, cozmo.objects.LightCube):
                label = id.cube_id
                if id.is_visible:
                    color = (0.5, 0.3, 1, 0.75)
                else:
                    color = (0, 0, 0.5, 0.75)
            elif isinstance(id, str):
                if 'Video' in id:
                    seen = self.robot.aruco_id in self.robot.world.perched.camera_pool and \
                           id in self.robot.world.perched.camera_pool[self.robot.aruco_id]
                    label = id
                elif 'Wall' in id:
                    label = 'W' + id[id.find('-')+1:]
                    try:
                        seen = self.robot.world.world_map.objects[id].is_visible
                    except:
                        seen = False
                    if seen:
                        color = (1, 0.5, 0.3, 0.75)
                    else:
                        color = (0.5, 0, 0, 0.75)
            else:
                seen = id in self.robot.world.aruco.seen_marker_ids
                label = id
            if color is None:
                if seen:
                    color = (0.5, 1, 0.3, 0.75)
                else:
                    color = (0, 0.5, 0, 0.75)
            if isinstance(specs, cozmo.util.Pose):
                self.draw_landmark_from_pose(id, specs, label, color)
            else:
                self.draw_landmark_from_particle(id, specs, label, color)

    def draw_landmark_from_pose(self, id, specs, label, color):
        coords = (specs.position.x, specs.position.y)
        angle = specs.rotation.angle_z.degrees
        if isinstance(id, cozmo.objects.LightCube):
            size = (44,44)
        else:
            size = (20,50)
        glPushMatrix()
        glColor4f(*color)
        self.draw_rectangle(coords, size=size, angle=angle, color=color)
        glColor4f(0., 0., 0., 1.)
        glTranslatef(*coords,0)
        glRotatef(angle-90, 0., 0., 1.)
        label_str = ascii(label)
        glTranslatef(3.-7*len(label_str), -5., 0.)
        glScalef(0.1,0.1,0.1)
        for char in label_str:
            glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(char))
        glPopMatrix()

    def draw_landmark_from_particle(self, id, specs, label, color):
        (lm_mu, lm_orient, lm_sigma) = specs
        coords = (lm_mu[0,0], lm_mu[1,0])
        glPushMatrix()
        glColor4f(*color)
        if isinstance(id, cozmo.objects.LightCube):
            size = (44,44)
        elif isinstance(id,str) and 'Wall' in id:
            try:
                wall = self.robot.world.world_map.objects[id]
            except KeyError:  # race condition: not in worldmap yet
                return
            size = (20, wall.length)
        else: # Aruco
            size = (20,50)
        if isinstance(id,str) and 'Video' in id:
            self.draw_triangle(coords, height=75, angle=lm_orient[1]*(180/pi),
                               color=color, fill=True)
            glColor4f(0., 0., 0., 1.)
            glTranslatef(*coords,0)
            glRotatef(lm_orient[1]*(180/pi)-90, 0., 0., 1.)
        else:
            self.draw_rectangle(coords, size=size, angle=lm_orient*(180/pi), color=color)
            glColor4f(0., 0., 0., 1.)
            glTranslatef(*coords,0)
            glRotatef(lm_orient*(180/pi)-90, 0., 0., 1.)
        label_str = ascii(label)
        glTranslatef(3.-7*len(label_str), -5., 0.)
        glScalef(0.1,0.1,0.1)
        for char in label_str:
            glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ord(char))
        glPopMatrix()
        ellipse_color = (color[1], color[2], color[0], 1)
        self.draw_particle_landmark_ellipse(lm_mu, lm_sigma, ellipse_color)

    def draw_particle_landmark_ellipse(self,coords,sigma,color):
        (w,v) = np.linalg.eigh(sigma[0:2,0:2])
        alpha = atan2(v[1,0],v[0,0])
        self.draw_ellipse(coords, abs(w)**0.5, alpha*(180/pi), color=color)

    def display(self):
        global REDISPLAY
        if not REDISPLAY: return
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
            self.draw_triangle((p.x,p.y), height=10, angle=math.degrees(p.theta),
                               color=color, fill=True)

        # Draw the robot at the best particle location
        (rx,ry,theta) = self.robot.world.particle_filter.pose
        (xy_var, theta_var) = self.robot.world.particle_filter.variance
        hdg = math.degrees(theta)
        self.draw_triangle((rx,ry), height=100, angle=hdg, tip_offset=-10,
                           color=(1,1,0,0.7))

        # Draw the error ellipse and heading error wedge
        (w,v) = np.linalg.eigh(xy_var)
        alpha = atan2(v[1,0],v[0,0])
        self.draw_ellipse((rx,ry), abs(w)**0.5, alpha/pi*180, color=(0,1,1))
        self.draw_wedge((rx,ry), 75, hdg, max(5, sqrt(theta_var)*360),
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
        (xy_var,theta_var) = pf.variance
        print ('xy_var=', xy_var, '  theta_var=', theta_var)
        
    def report_pose(self):
        (x,y,theta) = self.robot.world.particle_filter.pose
        hdg = math.degrees(theta)
        print('Pose = (%5.1f, %5.1f) @ %3d deg.' % (x, y, hdg))

    async def forward(self,distance):
        handle = self.robot.drive_straight(distance_mm(distance), speed_mmps(50),
                                           in_parallel=True,
                                           should_play_anim=False)
        await handle.wait_for_completed()
        pf = self.robot.world.particle_filter
        self.robot.loop.call_later(0.1, pf.look_for_new_landmarks)
        self.report_pose()

    async def turn(self,angle):
        handle = self.robot.turn_in_place(degrees(angle), in_parallel=True)
        await handle.wait_for_completed()
        pf = self.robot.world.particle_filter
        self.robot.loop.call_later(0.1, pf.look_for_new_landmarks)
        self.report_pose()

    async def look(self,angle):
        handle = self.robot.set_head_angle(degrees(angle), in_parallel=True)
        await handle.wait_for_completed()
        pf = self.robot.world.particle_filter
        self.robot.loop.call_later(0.1, pf.look_for_new_landmarks)
        self.report_pose()

    def keyPressed(self,key,mouseX,mouseY):
        pf = self.robot.world.particle_filter
        translate_wasd = 10 # millimeters
        translate_WASD = 40
        rotate_wasd = 22.5  # degrees
        rotate_WASD = 90
        global particles
        if key == b'e':       # evaluate
            pf.sensor_model.evaluate(pf.particles,force=True)
            pf.update_weights()
        elif key == b'r':     # resample
            pf.sensor_model.evaluate(pf.particles,force=True)
            pf.update_weights()
            pf.resample()
        elif key == b'w':     # forward
            self.robot.loop.create_task(self.forward(translate_wasd))
        elif key == b'W':     # forward
            self.robot.loop.create_task(self.forward(translate_WASD))
        elif key == b's':     # back
            self.robot.loop.create_task(self.forward(-translate_wasd))
        elif key == b'S':     # back
            self.robot.loop.create_task(self.forward(-translate_WASD))
        elif key == b'a':     # left
            self.robot.loop.create_task(self.turn(rotate_wasd))
        elif key == b'A':     # left
            self.robot.loop.create_task(self.turn(rotate_WASD))
        elif key == b'd':     # right
            self.robot.loop.create_task(self.turn(-rotate_wasd))
        elif key == b'D':     # right
            self.robot.loop.create_task(self.turn(-rotate_WASD))
        elif key == b'i':     # head up
            ang = self.robot.head_angle.degrees + 5
            self.robot.loop.create_task(self.look(ang))
        elif key == b'k':     # head down
            ang = self.robot.head_angle.degrees - 5
            self.robot.loop.create_task(self.look(ang))
        elif key == b'z':     # randomize
            pf.initializer.initialize(self.robot)
        elif key == b'Z':     # randomize
            pf.increase_variance()
        elif key == b'c':     # clear landmarks
            pf.clear_landmarks()
            print('Landmarks cleared.')
        elif key == b'v':     # display weight variance
            self.report_variance(pf)
        elif key == b'+':     # zoom in
            self.scale *= 1.25
            self.print_display_params()
            return
        elif key == b'-':     # zoom out
            self.scale /= 1.25
            self.print_display_params()
            return
        elif key == b'h':     # print help
            self.print_help()
            return
        elif key == b'$':     # toggle redisplay for debugging
            global REDISPLAY
            REDISPLAY = not REDISPLAY
            print('Redisplay ',('off','on')[REDISPLAY],'.',sep='')
        elif key == b'q':     #kill window
            glutDestroyWindow(self.window)
            glutLeaveMainLoop()
        glutPostRedisplay()
        self.report_pose()

    def specialKeyPressed(self, key, mouseX, mouseY):
        pf = self.robot.world.particle_filter
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
        elif key == GLUT_KEY_HOME:
            self.translation = [0., 0.]
            self.print_display_params()
        glutPostRedisplay()

    def print_display_params(self):
        print('scale=%.2f translation=[%.1f, %.1f]' %
              (self.scale, *self.translation))
        glutPostRedisplay()

    def print_help(self):
        print("""
Particle viewer commands:
  w/a/s/d    Drive robot +/- 10 mm or turn +/- 22.5 degrees
  W/A/S/D    Drive robot +/- 40 mm or turn +/- 90 degrees
   i/k       Head up/down 5 degrees
    e        Evaluate particles using current sensor info
    r        Resample particles (evaluates first)
    z        Reset particle positions (randomize, or all 0 for SLAM)
    c        Clear landmarks (for SLAM)
  arrows     Translate the view up/down/left/right
   Home      Center the view (zero translation)
    +        Zoom in
    -        Zoom out
    $        Toggle redisplay (for debugging)
    h        Print this help text
""")
