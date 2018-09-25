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
import pyflow

import random as rng


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
    pass

import math
import random

from . import opengl

import time

WINDOW = None
global snapno, path
global prev_gray, leftorrightindicate, go_forward
path = 'snap'
snapno = 0

prev_gray = None

leftorrightindicate = True
go_forward = GLUT_KEY_UP

global globthres, sizethres, kp0, des0
globthres = 30
sizethres = 0

nRange = 1.0

global rec, storemot
rec = None

global starttime, startvar
startvar = False
starttime = None

import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth

def matdif(p1, p0):
    # if len(p1) != len(p0):
    #     return False
    return sum([ ((p1[i][0][0]-p0[i][0][0])**2 + (p1[i][0][1]-p0[i][0][1])**2) for i in range(len(p1)) ])


def mandif(p1, p0):
    # if len(p1) != len(p0):
    #     return False
    return sum([p1[i][0][0]-p0[i][0][0] for i in range(len(p1)) ])

def angmat(p1, p0):
    return [((p1[i][0][0]-p0[i][0][0])**2 + (p1[i][0][1]-p0[i][0][1])**2, math.degrees(math.atan2(p1[i][0][1]-p0[i][0][1], p1[i][0][0]-p0[i][0][0]))) for i in range(len(p1))]

def drawline(image, p1, p0):
    lis = [((p1[i][0][0] , p1[i][0][1]), (p0[i][0][0] , p0[i][0][1])) for i in range(len(p1))]
    for i in lis:
        # print(i[0], i[1])
        cv2.line(image, i[1], (320, 240), (255, 0, 0), 3)
        # cv2.line(image, i[1], i[0], (255, 0, 0), 5)
    return image, lis

def motiondiff(rec, p1, pcurr):
    # return p1
    storemot = {}
    storeman = {}
    for k,v in rec.items():
        storemot[k] = matdif([p1[i] for i in range(len(p1)) if i in v["lis"]], [pcurr[i] for i in range(len(pcurr)) if i in v["lis"]])**(0.5)
        storeman[k] = mandif([p1[i] for i in range(len(p1)) if i in v["lis"]], [pcurr[i] for i in range(len(pcurr)) if i in v["lis"]])
    return storemot, storeman

def meanshift(p):
    bandwidth = estimate_bandwidth(p, quantile=0.2, n_samples=500)
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(p)
    return ms.labels_, ms.cluster_centers_

def flatten(p):
    return [ i[0] for i in p ]

def give_directions(coord):
    global prev_gray
    if abs(coord) > 10:
        prev_gray = None
        print(coord)
        if coord > 0:
            print("Move left!!!!!!")
        else:
            print("Move right!!!!!!")
        

def create_rec(lis, clus):
    rec = {}
    for i in range(len(lis)):
        if lis[i] not in rec:
            rec[lis[i]] = {"lis":[]}
        rec[lis[i]]["lis"].append(i)
    for i in range(len(clus)):
         rec[i]["clus"] = clus[i].tolist()
    return rec

def imgline(img):
    cv2.line(img, (0, 480), (320, 240), (255, 0, 0), 5)
    cv2.line(img, (640, 480), (320, 240), (255, 0, 0), 5)
    cv2.line(img, (640, 480), (320, 240), (255, 0, 0), 5)
    return img

def centdistance(keypoints):
    ptlist = [kpt.pt for kpt in keypoints]
    return int(sum([math.sqrt(i[0]**2 + i[1]**2) for i in ptlist])/len(keypoints))

def set_surf_threshold(keypoints):
    global globthres, sizethres, starttime, startvar
    if len(keypoints)<20:
        pass
    else:
        # sizethres = centdistance(keypoints)
        # print(globthres, len(keypoints))
        globthres+=100
    if starttime == None:
        starttime = time.time()
    if len(keypoints) <10:
        globthres -= 300
    # print(centdistance(keypoints))
    # try:
    #     if (sizethres - 5) <= centdistance(keypoints) <= (sizethres + 5) or startvar is True:
    #         pass
    #     else:
    #         sizethres = centdistance(keypoints)
    #         print(globthres, sizethres, len(keypoints), time.time() - starttime)
    #         if time.time() - starttime > 0.25:
    #             startvar = True
    #         starttime = time.time()
    #         globthres+=500
    # except:
    #     startvar = False
    #     globthres -= 4000


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

    # ================ Window Setup ================

    def idle(self):
        global prev_gray, globthres, des0, kp0
        if not self.is_incom_image and CamViewer.incom_image is None:
            image = cv2.resize(np.array(self.robot.world.latest_image.raw_image), (self.width, self.height))
        else:
            image = CamViewer.incom_image

        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # canny_output = cv2.Canny(gray, 100, 100 * 2)
        # _, contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # hull_list = []
        # for i in range(len(contours)):
        #     hull = cv2.convexHull(contours[i])
        #     hull_list.append(hull)
        
        # drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
        # rng.seed(12345)
        # for i in range(len(contours)):
        #     color = (rng.randint(0, 256), rng.randint(0, 256), rng.randint(0, 256))
        #     cv2.drawContours(drawing, contours, i, color)
        #     cv2.drawContours(drawing, hull_list, i, color)

        # print(image.shape)
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # ret, thresh = cv2.threshold(gray, 127, 255, 0)
        # im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # cnt = contours[4]
        # cv2.drawContours(im2, [cnt], 0, (0, 255, 0), 3)
        # image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        # gray = cv2.fastNlMeansDenoising(gray,None,15,7,21)

        # self.image = flow
        # sift = cv2.xfeatures2d.SIFT_create()
        # kp = sift.detect(gray,None)
        # image = cv2.drawKeypoints(gray, kp, image, color=(0, 255, 0))
        # image = cv2.fastNlMeansDenoisingColored(image, None, 2, 0, 7, 21)
        # sift = cv2.xfeatures2d.SIFT_create()
        # kp = sift.detect(gray,None)
        
        # orb = cv2.ORB_create(10000)
        # orb.setFastThreshold(30)
        # matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        # gms = GmsMatcher(orb, matcher)
        # kp, des = orb.detectAndCompute(image, np.array([]))

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        surf = cv2.xfeatures2d.SURF_create(globthres)
        kp, des = surf.detectAndCompute(image, None)

        set_surf_threshold(kp)
        image = cv2.drawKeypoints(gray, kp, image, color=(0, 255, 0))

        # orb = cv2.ORB_create(10000)
        # orb.setFastThreshold(globthres)
        # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # if prev_gray is None:
        #     prev_gray = image.copy()
        #     kp0, des0 = orb.detectAndCompute(image, None)
        #     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height,0,GL_RGB, GL_UNSIGNED_BYTE, prev_gray)
        #     glutPostRedisplay()
        # else:
        #     kp1, des1 = orb.detectAndCompute(image, None)
        #     matches = bf.match(des0, des1)
        #     # matches = sorted(matches, key=lambda x: x.distance)
        #     # print()
        #     if max([i.distance for i in matches]) > 80:
        #         print(max([i.distance for i in matches]))
        #         print(globthres)
        #         globthres+=3
        #     if max([i.distance for i in matches]) < 10:
        #         globthres-=5
            # img3 = cv2.drawMatches(prev_gray, kp0, image, kp1, matches[:10], None, flags=2)
            # prev_gray = image.copy()
            # print(image.shape)
            # img3 = cv2.resize(img3, (640, 480))
            # print(img3.shape)
        # gray = cv2.drawKeypoints(gray, kp, image, color=(0, 255, 0))
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height,0,GL_RGB, GL_UNSIGNED_BYTE, image)
        glutPostRedisplay()
        
        # if prev_gray is None: 
        #     prev_gray = gray.copy()
        #     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height,0,GL_RGB, GL_UNSIGNED_BYTE, image)
        #     glutPostRedisplay()
        # else:
        #     flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            
        #     prev_gray = gray.copy()
        #     self.image = self.draw_flow(gray, flow)

        #     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width, self.height,0,GL_RGB, GL_UNSIGNED_BYTE, self.image)
        #     glutPostRedisplay()

        
            # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # feature_params = dict(maxCorners=100,
            #                       qualityLevel=0.2,
            #                       minDistance=7,
            #                       blockSize=7)
            
            # lk_params = dict(winSize=(15, 15),
            #                  maxLevel=2,
            #                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

            # if prev_gray is None:
            #     p0 = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
            #     print(p0.shape)
            #     # p0 = np.array([])
            #     # print(np.array(flatten(p0)))
            #     pcurr = p0.copy()
            #     prev_gray = gray.copy()
            #     mask = np.zeros_like(image)
            #     glutPostRedisplay()

            # p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_params)

            # if len(p1) == len(pcurr):
            #     if (matdif(p1, pcurr)) > 100:
            #         # print(str(matdif(p1, pcurr)) + "Hellooooo")
            #         # if rec is None:
            #         mslab, msclus = meanshift(np.array(flatten(pcurr)))
            #         rec = create_rec(mslab, msclus)
            #             # print(rec)
            #         # try:
            #         someval, newval = motiondiff(rec, p1, pcurr)
            #         # print(someval, newval)
            #         give_directions(newval[max(someval,key=someval.get)])
            #         # except:
            #         #     rec = create_rec(meanshift(np.array(flatten(pcurr))))
            #         #     print(rec)
            #         # print(meanshift(np.array(flatten(p1))))
            #         # print(create_rec(meanshift(np.array(flatten(pcurr)))))
            #         # print(matdif(pcurr, p0), end=" ")

            # pcurr = p1.copy()
            

            # try:
            #     good_new = p1[st == 1]
            #     good_old = p0[st==1]
            # except Exception as e:
            #     p0 = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
            #     prev_gray = gray.copy()
            #     mask = np.zeros_like(image)
            #     glutPostRedisplay()

            # for i,(new,old) in enumerate(zip(good_new,good_old)):
            #     a,b = new.ravel()
            #     c,d = old.ravel()
            #     mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 1)
            #     image = cv2.circle(image,(a,b),2,color[i].tolist(),-1)
        
            # image = cv2.add(image,mask)

            # prev_gray = gray.copy()
            # p0 = good_new.reshape(-1,1,2)

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
        glutSpecialFunc(self.specialKeyPressed)
        glutSpecialUpFunc(self.specialKeyUp)
        glutIdleFunc(self.idle)
        glutMainLoop()

    def start(self):  # Displays in background
        if not WINDOW:
            opengl.init()
            opengl.CREATION_QUEUE.append(self.window_creator)
            CamViewer.prog_start = True
            # print(type(cozmo.robot.MAX_HEAD_ANGLE))
            # Forward(10).now()
            self.robot.set_head_angle(cozmo.util.Angle(0)).wait_for_completed()
            while not WINDOW:
                time.sleep(0.1)
        # print("Type 'h' in the world map window for help.")

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
        # if key == b'p':
        #     print("Producing optical flow")
        #     self.brox()
        self.display()

    def specialKeyPressed(self, key, x, y):
        # self.robot.set_head_angle(cozmo.util.Angle(0)).wait_for_completed()
        global leftorrightindicate, globthres
        if key == GLUT_KEY_LEFT:
            self.robot.drive_wheels(-25, 25)
            leftorrightindicate = True
            globthres=100
            # self.robot.set_head_angle(cozmo.util.Angle(0)).wait_for_completed()
        elif key == GLUT_KEY_RIGHT:
            self.robot.drive_wheels(25, -25)
            leftorrightindicate = True
            globthres = 100
        elif key == go_forward:
            # self.robot.drive_straight(distance_mm(10), speed_mmps(50))
            self.robot.drive_wheels(100, 100)
            leftorrightindicate = False
            globthres = 100
        elif key == GLUT_KEY_DOWN:
            # self.robot.drive_straight(distance_mm(-10), speed_mmps(50))
            self.robot.drive_wheels(-75, -75)
            leftorrightindicate = True
            globthres = 100
        glutPostRedisplay()

    def specialKeyUp(self, key, x, y):
        global leftorrightindicate, go_forward
        self.robot.drive_wheels(0, 0)
        leftorrightindicate = True
        go_forward = GLUT_KEY_UP
        glutPostRedisplay()

    def capture(self):
        global snapno, path
        if not os.path.exists(path):
                os.makedirs(path)

        if not self.is_incom_image and CamViewer.incom_image is None:
            image = cv2.resize(np.array(self.image), (self.width, self.height))
        else:
            image = CamViewer.incom_image
        Image.fromarray(image).save(path + '/cozmo_snap' + str(snapno) + '.jpg')
        snapno +=1

    def limited_flow(self, img, flow, step=15):
        global leftorrightindicate
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step /
                        2:w:step].reshape(2, -1).astype(int)
        fx, fy = flow[y, x].T
        fxfy = np.square(fx) + np.square(fy)
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0))
        pointlist = []
        pointlist2 = []
        for (x1, y1), (x2, y2) in lines:
            cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)

    def draw_flow(self, img, flow, step=15):
        global leftorrightindicate
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2, -1).astype(int)
        fx, fy = flow[y, x].T
        fxfy = np.square(fx) + np.square(fy)
        # if disp > 4000:
        #     print("Motion")
            # print(y[np.argmax(fxfy)], x[np.argmax(fxfy)])
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0))
        pointlist = []
        pointlist2 = []
        # try:
        #     self.capture()
        # except:
        #     pass
        # r = lambda: (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        if True is False:
            for (x1, y1), (x2, y2) in lines:
                cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
                if ((x2-x1)^2 + (y2-y1)^2) > 7:
                    cv2.circle(vis, (x1, y1), 1, (255, 0, 0), -1)
                    cv2.circle(vis, (x2, y2), 1, (255, 0, 0), -1)
                    pointlist.append([x2, y2])
                    pointlist2.append([x1, y1])
            if len(pointlist)>5:
                # grou, centers = meanshift(np.asarray(pointlist))
                # for i in range(len(centers.tolist())):
                #     color = r()
                #     clust_point = np.argwhere(grou == i).flatten().tolist()
                #     net_movement = 0
                #     for j in clust_point:
                #         cv2.circle(vis, (int(pointlist[j][0]), int(pointlist[j][1])), 9, color, -1)
                #         net_movement += fxfy[j]
                #     if net_movement > 0:
                #         displace = 0
                #         for j in clust_point:
                #             displace += pointlist[j][0] - pointlist2[j][0]
                #         if displace < 0 :
                #             print("Move Right!!!!!")
                #         else:
                #             print("Move Left!!!!!")
                disp = np.sum(fxfy)
                if disp> 4000:
                    displace = 0
                    for i in range(len(pointlist)):
                        displace += pointlist[i][0] - pointlist2[i][0]
                    self.avoid_collision(displace)
                
                        # self.robot.drive_wheels(0, 0)
        else:
            for (x1, y1), (x2, y2) in lines:
                cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)


        return vis

    def avoid_collision(self, displace):
        global leftorrightindicate, go_forward
        leftorrightindicate = True
        go_forward = None
        self.robot.drive_wheels(0, 0)
        if displace> 0:
            print("Moving left", displace)
            self.robot.turn_in_place(degrees(10)).wait_for_completed()
        else:
            print("Moving Right", displace)
            self.robot.turn_in_place(degrees(-10)).wait_for_completed()

    # def brox(self):
    #     if snapno >= 2:
    #         im1 = np.array(Image.open(path + '/cozmo_snap' + str(snapno-1) + '.jpg'))
    #         im2 = np.array(Image.open(path + '/cozmo_snap'  + str(snapno-2) +  '.jpg'))
    #         im1 = im1.astype(float) / 255.
    #         im2 = im2.astype(float) / 255.
    #         alpha = 0.012
    #         ratio = 0.75
    #         minWidth = 20
    #         nOuterFPIterations = 7
    #         nInnerFPIterations = 1
    #         nSORIterations = 30
    #         colType = 0  # 0 or default:RGB, 1:GRAY (but pass gray image with shape (h,w,1))
    #         s = time.time()
    #         u, v, im2W = pyflow.coarse2fine_flow(im1, im2, alpha, ratio, minWidth, nOuterFPIterations, nInnerFPIterations, nSORIterations, colType)
    #         e = time.time()
    #         print('Time Taken: %.2f seconds for image of size (%d, %d, %d)' % (e - s, im1.shape[0], im1.shape[1], im1.shape[2]))
    #         flow = np.concatenate((u[..., None], v[..., None]), axis=2)
    #         hsv = np.zeros(im1.shape, dtype=np.uint8)
    #         hsv[:, :, 0] = 0
    #         hsv[:, :, 1] = 0
    #         mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    #         hsv[..., 0] = ang * 180 / np.pi / 2
    #         hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    #         rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    #         image_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    #         Image.fromarray(image_gray).save(path + '/outFlow_op' + str(snapno) + '.png')
    #         # print(image_gray.shape)
    #         # image_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    #         # # print(type(hsv))
    #         image_gray[image_gray > 120] = 255
    #         image_gray[image_gray < 60] = 0
    #         Image.fromarray(image_gray).save(path + '/outFlow_bw' + str(snapno) + '.png')
    #         # np.save('examples/outFlow.npy', flow)

    #     else:
    #         print("Capture more than 2 images")

        
