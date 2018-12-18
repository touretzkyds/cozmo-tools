"""
OpenGL based CamViewer
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
# from __future__ import unicode_literals
import numpy as np
from PIL import Image
import time
import argparse

# import random as rng

import numpy as np
from scipy.ndimage.measurements import label

nRange = 1.0

import time
import numpy as np
import cv2
import cozmo
from PIL import Image, ImageEnhance
from cozmo.util import degrees, distance_mm, speed_mmps
try:
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except:
    print('OpenGL not found - cam_viewer')

import math
import random

from . import opengl

import time

from . import transform

import math

#For capturing images
global snapno, path, kp, desc, keyframe
snapno = 0
path = 'snap/'
kp = []
desc = []
keyframe = [0, 0, 0]

global pt1, pt2, pt3, pt4, currpt
pt1=pt2=pt3=pt4=currpt = None

WINDOW = None

global A, dist
A = np.mat([[588.8576056918163, 0.0, 305.80427370754563], [0.0, 589.4169696271891, 242.88289746495008], [0.0, 0.0, 1.0]])
dist = np.asarray([[-0.48942154896813833, 9.413177034843194, -0.0018376541059663938, -0.002275458333272118, -50.23842278296178]])

def check_movement(prev_posit, posit):
    return prev_posit.x == posit.x and prev_posit.y == posit.y and prev_posit.z == posit.z

def slide_check(slide, level, image, posit):
    some = False
    if not check_movement(slide[-1][1], posit):
        some = True
        slide.append([image, posit])
    # print(len(slide))
    if level < len(slide):
        slide.pop(0)
    return slide, some

def stereo(image, R, t):
    global A, dist
    return cv2.stereoRectify(A, dist, A, dist, (480, 640), np.asarray(R), np.asarray(t), flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=(480, 640))

def readyStereoBM(roi1, roi2):
    stereobm = cv2.StereoBM_create(numDisparities=112, blockSize=31)
    stereobm.setPreFilterSize(31)  # 41
    stereobm.setPreFilterType(cv2.STEREO_BM_PREFILTER_NORMALIZED_RESPONSE)
    stereobm.setPreFilterCap(31)
    stereobm.setTextureThreshold(10)
    stereobm.setMinDisparity(0)
    stereobm.setSpeckleWindowSize(100)
    stereobm.setSpeckleRange(64)
    stereobm.setUniquenessRatio(0)
    stereobm.setROI1(roi1)
    stereobm.setROI1(roi2)
    return stereobm

def getDisparity(stereo, img1, img2, mapx1, mapy1, mapx2, mapy2):
    dst1 = cv2.remap(img1, mapx1, mapy1, cv2.INTER_LINEAR)
    dst2 = cv2.remap(img2, mapx2, mapy2, cv2.INTER_LINEAR)
    gray1 = dst1
    gray2 = dst2
    disparity = stereo.compute(gray1, gray2)/16
    return disparity

def pointsbet(x1, y1, x2, y2, x):
    m = (y2-y1)/(x2-x1)
    c = y2 - x2*m
    return m*x + c

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

    def groundplane(self, image=None):
        if image is None:
            image = self.image
        canny = cv2.Canny(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 30, 150)
        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 10  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50  # minimum number of pixels making up a line
        max_line_gap = 50  # maximum gap in pixels between connectable line segments
        line_image = np.copy(image) * 0  # creating a blank to draw lines on
        lines = cv2.HoughLinesP(canny, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
        y_max = 300
        x_line = [300]*641
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if y1 > y_max and y2 > y_max:
                        for i in range(x1,x2):
                            poin = pointsbet(x1, y1, x2, y2, i)
                            if poin > x_line[i]:
                                x_line[i] = int(poin)
        x_c, y_c = list(range(641)), x_line
        cannyT = canny.T
        for ii in range(640):
            if y_c[ii] <= 310:
                for kk in range(479, -1, -1):
                    if cannyT[ii][kk] != 0:
                        if kk > y_c[ii]:
                            y_c[ii] = kk
                        break
        points = []
        for i in range(len(x_c)):
            points.append((x_c[i], int(y_c[i])))
        points.insert(0,(0, 480))
        points.append((640, 480))
        points = np.array(points)
        img = image.copy()
        cv2.drawContours(image, [points], 0, (0,255,0), 2)
        cv2.fillPoly(image, pts =[points], color=(0,255,0))
        image = cv2.addWeighted(img, 0.5, image, 0.5, 0)
        dist = ((480-y_c[int(len(y_c)/2)])/180)*(29-8.5)
        cv2.putText(image, str(y_c[int(len(y_c)/2)]),
                    (x_c[int(len(y_c)/2)], y_c[int(len(y_c)/2)]+10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1)
        return image, y_c

    def thinkandmove(self, y, thres, depth=0):
        maxval = max(y[280:360])
        if maxval > thres:
            self.robot.drive_wheels(0, 0)
            leftval = sum((g-279) * y[g] for g in range(280, 321)) / sum(list(range(1, 41)))
            rightval = sum((361-g) * y[g] for g in range(320, 361)) / sum(list(range(1, 41)))
            direction = (-1 * 90 * (leftval - 300)/180) if leftval > rightval else (1 * 90 * (rightval - 300)/180)
            # # print(sum(y[280:320]), sum(y[320:360]))
            self.robot.turn_in_place(degrees(direction)).wait_for_completed()
            print(direction)
            self.robot.set_head_angle(cozmo.util.Angle(-0.3)).wait_for_completed()
            self.image, y = self.groundplane()
            if max(y[280:360]) > thres:
                self.robot.drive_straight(distance_mm(-25), speed_mmps(50)).wait_for_completed()
                leftval = sum((g-279) * y[g] for g in range(280, 321)) / sum(list(range(1, 41)))
                rightval = sum((361-g) * y[g] for g in range(320, 361)) / sum(list(range(1, 41)))
                direction = (-1 * 60 * (leftval - 300)/180) if leftval > rightval else (1 * 60 * (rightval - 300)/180)
                self.robot.turn_in_place(degrees(direction)).wait_for_completed()

            # self.image, y = self.groundplane()
            # print("Hello")
            # print(maxval)
            # self.thinkandmove(y, thres+30, depth=depth+1)
            # print("Finish")
            self.robot.set_head_angle(cozmo.util.Angle(0)).wait_for_completed()
        else:
            self.robot.drive_wheels(50, 50)

    def doordetect(self, vis=False, image=None):
        global path, kp, desc, keyframe

        if image is not None:
            image = image.copy()
        for k in range(len(kp)):
            # img1 = k
            sift = cv2.xfeatures2d.SIFT_create()
            kp1, desc1 = kp[k], desc[k]
            kp2, desc2 = sift.detectAndCompute(image, None)
            index_params = dict(algorithm=0, trees=5)
            sch_params = dict(checks=50)
            flann = cv2.FlannBasedMatcher(index_params, sch_params)
            matches = flann.knnMatch(desc1, desc2, k=2)
            matches_arr = []
            good_matches = []
            for m, n in matches:
                if m.distance < 3 * n.distance:
                    good_matches.append(m)
                matches_arr.append(m)

            src_pts = [kp1[m.queryIdx].pt for m in good_matches]
            src_pts = np.array(src_pts, dtype=np.float32).reshape((-1, 1, 2))
            dst_pts = [kp2[m.trainIdx].pt for m in good_matches]
            dst_pts = np.array(dst_pts, dtype=np.float32).reshape((-1, 1, 2))

            homo, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5)

            matches_mask = mask.ravel().tolist()


            # keyframe[0] = keyframe[0] + 1
            # if keyframe[0] > 10:
            #     keyframe[0] = 0

            # if keyframe[1] < matches_mask.count(1):
            #     keyframe[1] = matches_mask.count(1)
            #     keyframe[2] = (homo, mask)
            # else:
            #     homo, mask = keyframe[2]

            matches_mask = mask.ravel().tolist()
            h, w = (480, 640)
            pts = [[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]
            pts = np.array(pts, dtype=np.float32).reshape((-1, 1, 2))
            dst = cv2.perspectiveTransform(pts, homo)

            c = dst.tolist()
            x = []
            y = []
            for i in dst.tolist():
                x.append(i[0][0])
                y.append(i[0][1])
            x = np.array(x)
            y = np.array(y)
            area = 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

            if area > 10000:
                cv2.polylines(image , [np.int32(dst)], True, [0, 0, 255], 3, 8)

            for i in range(len(matches_mask)):
                if matches_mask[i] == 1:
                    cv2.circle(image, (dst_pts[i][0][0], dst_pts[i][0][1]), 2, (255, 0, 0), thickness=2)


        return image

    def drawMouse(self):
        img = self.image.copy()
        global pt1, pt2, pt3, pt4, currpt
        if pt1 is not None:
            cv2.circle(img, pt1, 2, (255,0,0), thickness=2)
            if pt2 is not None:
                cv2.circle(img, pt2, 2, (255, 0, 0), thickness=2)
                cv2.line(img, pt1, pt2, (0, 0, 255), 3)
                if pt3 is not None:
                    cv2.circle(img, pt3, 2, (255,0,0), thickness=2)
                    cv2.line(img, pt2, pt3, (0, 0, 255), 3)
                    if pt4 is not None:
                        cv2.circle(img, pt4, 2, (255, 0, 0), thickness=2)
                        cv2.line(img, pt3, pt4, (0, 0, 255), 3)
                        cv2.line(img, pt4, pt1, (0, 0, 255), 3)
                    else:
                        cv2.line(img, pt3, currpt, (0, 0, 255), 3)
                else:
                    cv2.line(img, pt2, currpt, (0, 0, 255), 3)
            else:
                cv2.line(img, pt1, currpt, (0, 0, 255), 3)
        return img

    # ================ Window Setup ================
    def idle(self):
        global prev_gray, starttime, filteredImg, posit, slide, prev_rotmat, A, dist
        if not self.is_incom_image and CamViewer.incom_image is None:
            image = cv2.resize(np.array(self.robot.world.latest_image.raw_image), (self.width, self.height))
        else:
            image = CamViewer.incom_image

        self.image = image.copy()

        image = self.drawMouse()
        # gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # sift = cv2.xfeatures2d.SIFT_create()
        # kp = sift.detect(gray,None)
        # image=cv2.drawKeypoints(gray,kp,image)
        image = self.doordetect(image=image)

        # image, y_c = self.groundplane(image=image)
        # self.thinkandmove(y_c, 400)

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height,0,GL_RGB, GL_UNSIGNED_BYTE, image)
        glutPostRedisplay()


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
        # glutMouseFunc(self.mouseClicked)
        glutPassiveMotionFunc(self.motion)
        glutSpecialFunc(self.specialKeyPressed)
        glutSpecialUpFunc(self.specialKeyUp)
        glutIdleFunc(self.idle)
        glutMainLoop()

    def start(self):  # Displays in background
        global starttime
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)
            CamViewer.prog_start = True
            # self.reloadFile()
            starttime = time.time()
            # self.robot.set_head_angle(cozmo.util.Angle(0)).wait_for_completed()
            while not WINDOW:
                time.sleep(0.1)

    def display(self):
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
        if key == b'r':
            global pt1, pt2, pt3, pt4
            print("Clearing coordinates")
            pt1=pt2=pt3=pt4=None
        if key == b's':
            if pt1 is None or pt2 is None or pt3 is None or pt4 is None:
                print("Not all points defined. Can't save perspective")
            else:
                self.savePerspective()
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

    def mouseClicked(self, button, state, x, y):
        if button == GLUT_LEFT_BUTTON and state == GLUT_DOWN:
            global pt1, pt2, pt3, pt4
            if pt1 is None:
                pt1 = (x,y)
            elif pt2 is None:
                pt2 = (x,y)
            elif pt3 is None:
                pt3 = (x,y)
            elif pt4 is None:
                pt4 = (x, y)

    def motion(self, x, y):
        global currpt
        currpt = (x,y)

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

    def reloadFile(self):
        global kp, desc
        ymlhandle = cv2.FileStorage('objects.yml', cv2.FILE_STORAGE_READ)
        num = int(ymlhandle.getNode('noofobjects').real())
        kp = []
        desc = []
        for i in range(1, num+1):
            kploc = []
            for point in ymlhandle.getNode("kp" + str(i)).mat().tolist():
                temp = cv2.KeyPoint(x=point[0],y=point[1],_size=point[2], _angle=point[3], _response=point[4], _octave=int(point[5]), _class_id=int(point[6]))
                kploc.append(temp)
            kp.append(np.asarray(kploc))
            desc.append(ymlhandle.getNode("desc" + str(i)).mat())

        ymlhandle.release()

    def savePerspective(self):
        global pt1, pt2, pt3, pt4
        inipt = np.float32([pt1, pt2, pt3, pt4])
        finpt = np.float32([[0, 0], [0, self.height], [self.width, self.height], [self.width, 0]])
        M = cv2.getPerspectiveTransform(inipt, finpt)
        warped = cv2.warpPerspective(self.image, M, (self.width, self.height))
        # self.capture(img=warped, name='perspective')
        ymlhandle = cv2.FileStorage('objects.yml', cv2.FILE_STORAGE_READ)
        num = ymlhandle.getNode('noofobjects').real()
        noofobjects = 0 if num is None else int(num)
        ymlwrite = cv2.FileStorage('objects.yml', cv2.FILE_STORAGE_WRITE)
        ymlwrite.write("noofobjects", noofobjects+1)
        for i in range(1, noofobjects+1):
            ymlwrite.write("kp" + str(i), ymlhandle.getNode("kp" + str(i)).mat())
            ymlwrite.write("desc" + str(i), ymlhandle.getNode("desc" + str(i)).mat())
        ymlhandle.release()
        sift = cv2.xfeatures2d.SIFT_create()
        kp, desc = sift.detectAndCompute(warped, None)
        kp_store = []
        for point in kp:
            temp = [point.pt[0], point.pt[1], point.size, point.angle, point.response, point.octave, point.class_id]
            kp_store.append(temp)
        ymlwrite.write("kp" + str(noofobjects+1), np.asarray(kp_store))
        ymlwrite.write("desc" + str(noofobjects+1), np.asarray(desc))
        ymlwrite.release()
        self.reloadFile()
        print("Clearing coordinates")
        pt1=pt2=pt3=pt4=None

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
