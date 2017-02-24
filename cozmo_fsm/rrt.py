from math import pi, sin, cos, inf, atan2
import numpy as np
import random
import time

from .transform import wrap_angle
from .shapes import *

class RRTNode():
    def __init__(self, parent=None, x=0, y=0, q=0):
        self.parent = parent
        self.x = x
        self.y = y
        self.q = q

    def __repr__(self):
        return '<RRTNode (%s,%s) %d deg>' % \
            ( ('%5.1f'% self.x).strip(), ('%5.1f' % self.y).strip(), self.q/pi*180)


class RRT():
    def __init__(self, robot, max_iter=1000, step_size=5, tolsq=16,
                 obstacles=[],
                 bounds=(range(-500,500), range(-500,500))):
        # Each tree is an array of RRTNodes starting at element 1.
        # Element 0 contains the current size of the tree.
        # We pre-allocate the storage to speed things up.
        self.robot = robot
        self.max_iter = max_iter
        self.step_size = step_size
        self.tolsq = tolsq
        self.obstacles = obstacles
        self.robot_parts = self.make_robot_parts(robot)
        self.bounds = bounds

    REACHED = 'reached'
    COLLISION = 'collision' 
    INTERPOLATE = 'interpolate'

    def nearest_node(self, tree, target_node):
        best_distance = inf
        closest_node = None
        x = target_node.x
        y = target_node.y
        for this_node in tree:
            distx = this_node.x - x
            disty = this_node.y - y
            distsq = distx*distx + disty*disty
            if distsq < best_distance:
                best_distance = distsq
                closest_node = this_node
        return closest_node

    def random_node(self):
        return RRTNode(x=random.choice(self.bounds[0]),
                       y=random.choice(self.bounds[1]))

    def extend(self, tree, target):
        nearest = self.nearest_node(tree, target)
        status, new_node = self.interpolate(nearest, target)
        if status is not self.COLLISION:
            tree.append(new_node)
        time.sleep(0.01)
        return (status, new_node)

    def interpolate(self, node, target):
        dx = target.x - node.x
        dy = target.y - node.y
        if (dx*dx + dy*dy) < self.tolsq:
            return (self.REACHED, RRTNode(parent=node, x=target.x, y=target.y))
        q = atan2(dy,dx)
        xstep = self.step_size * cos(q)
        ystep = self.step_size * sin(q)
        new_node = RRTNode(parent=node, x=node.x+xstep, y=node.y+ystep, q=q)
        if self.collides(new_node):
            return (self.COLLISION, None)
        else:
            return (self.INTERPOLATE, new_node)

    def parts_to_node(self,node):
        parts = []
        for part in self.robot_parts:
            center = transform.point(part.center[0,0]+node.x, part.center[1,0]+node.y)
            this_part = Circle(center=center, radius=part.radius)
            parts.append(this_part)
        return parts

    def collides(self, node):
        for part in self.parts_to_node(node):
            for obstacle in self.obstacles:
                if part.collides(obstacle):
                    return True
        return False        

    def plan_path(self, start, goal):
        self.start = start
        self.goal = goal
        treeA = [start]
        treeB = [goal]
        self.treeA = treeA
        self.treeB = treeB
        swapped = False
        for i in range(self.max_iter):
            r = self.random_node()
            (status, new_node) = self.extend(treeA, r)
            if status is not self.COLLISION:
                (status, new_node) = self.extend(treeB, treeA[-1])
                if status is self.REACHED:
                    break
                (treeB, treeA) = (treeA, treeB)
                swapped = not swapped
        if i == self.max_iter-1:
            return None
        if swapped:
            (treeB, treeA) = (treeA, treeB)
        print('iterations=',i)
        return self.get_path(treeA, treeB)

    def get_path(self, treeA, treeB):
        nodeA = treeA[-1]
        pathA = [nodeA]
        while nodeA.parent is not None:
            nodeA = nodeA.parent
            pathA.append(nodeA)
        pathA.reverse()
        # treeB was built backwards from the goal, so headings
        # need to be reversed
        nodeB = treeB[-1]
        pathB = []
        while nodeB.parent is not None:
            nodeB = nodeB.parent
            nodeB.q = wrap_angle(nodeB.q + pi)
            pathB.append(nodeB)
        return (treeA, treeB, pathA + pathB)

    def make_robot_parts(self,robot):
        result = []
        for joint in robot.kine.joints.values():
            if joint.collision_model:
                tmat = robot.kine.link_to_base(joint)
                robot_obst = joint.collision_model.instantiate(tmat)
                result.append(robot_obst)
        return result

