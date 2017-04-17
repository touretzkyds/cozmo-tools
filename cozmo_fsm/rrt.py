from math import pi, sin, cos, inf, atan2
import numpy as np
import random
import time

import cozmo_fsm.transform
from .transform import wrap_angle
from .rrt_shapes import *
from .worldmap import Wall, wall_marker_dict, LightCubeObst

#---------------- RRTNode ----------------

class RRTNode():
    def __init__(self, parent=None, x=0, y=0, q=0, radius=0):
        self.parent = parent
        self.x = x
        self.y = y
        self.q = q
        self.radius = radius

    def __repr__(self):
        rad = "" if self.radius == 0 else (" rad. %d" % self.radius)
        return '<RRTNode (%.1f,%.1f)@%d deg%s>' % (self.x, self.y, self.q/pi*180, rad)


#---------------- RRT Path Planner ----------------

class StartCollides(Exception): pass
class GoalCollides(Exception): pass
class MaxIterations(Exception): pass

class RRT():
    def __init__(self, robot, max_iter=1000, step_size=10, arc_radius=40,
                 xy_tolsq=16, q_tol=5/180*pi,
                 obstacles=[], auto_obstacles=True,
                 bounds=(range(-500,500), range(-500,500))):
        self.robot = robot
        self.max_iter = max_iter
        self.step_size = step_size
        self.arc_radius = arc_radius
        self.xy_tolsq = xy_tolsq
        self.q_tol = q_tol
        self.robot_parts = self.make_robot_parts(robot)
        self.bounds = bounds
        self.obstacles = obstacles
        self.auto_obstacles = auto_obstacles
        self.treeA = []
        self.treeB = []
        self.start = None
        self.goal = None

    REACHED = 'reached'
    COLLISION = 'collision' 
    INTERPOLATE = 'interpolate'

    def set_obstacles(self,obstacles):
        self.obstacles = obstacles

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
        time.sleep(0.01)   # *** FOR ANIMATION PURPOSES
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
            tmat = transform.translate(center[0,0],center[1,0]) \
                      .dot(transform.aboutZ(part.orient))
            this_part = part.instantiate(tmat)
            parts.append(this_part)
        return parts

    def collides(self, node):
        for part in self.parts_to_node(node):
            for obstacle in self.obstacles:
                if part.collides(obstacle):
                    return True
        return False        

    def plan_push_chip(self, start, goal, max_turn=20*pi/180, arc_radius=40.):
        return plan_path(self, start, goal, max_turn, arc_radius)

    def plan_path(self, start, goal, max_turn=pi, arc_radius=0):
        if self.auto_obstacles:
            self.obstacles = self.generate_obstacles()
        self.start = start
        self.goal = goal
        if self.collides(start):
            raise StartCollides()
        if self.collides(goal):
            raise GoalCollides()
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
        if swapped:
            (treeB, treeA) = (treeA, treeB)
        if status is self.REACHED:
            return self.get_path(treeA, treeB)
        else:
            raise MaxIterations()

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
        prev_heading = wrap_angle(nodeB.q + pi)
        while nodeB.parent is not None:
            nodeB = nodeB.parent
            (nodeB.q, prev_heading) = (prev_heading, wrap_angle(nodeB.q+pi))
            pathB.append(nodeB)
        self.path = pathA + pathB
        return (treeA, treeB, self.path)

    def make_robot_parts(self,robot):
        result = []
        for joint in robot.kine.joints.values():
            if joint.collision_model:
                tmat = robot.kine.link_to_base(joint)
                robot_obst = joint.collision_model.instantiate(tmat)
                result.append(robot_obst)
        return result

    def generate_obstacles(self):
        self.robot.world.world_map.generate_map()
        result = []
        for obj in self.robot.world.world_map.objects:
            if isinstance(obj, Wall):
                result = result + self.generate_wall_obstacle(obj)
            elif isinstance(obj, LightCubeObst):
                result.append(self.generate_cube_obstacle(obj))
        return result

    def generate_wall_obstacle(self,wall):
        wall_spec = wall_marker_dict[wall.id]
        half_length = wall.length / 2
        widths = []
        last_x = -half_length
        edges = [ [0, -half_length, 0., 1.] ]
        for (center,width) in wall_spec.doorways:
            left_edge = center - width/2 - half_length
            edges.append([0., left_edge, 0., 1.])
            widths.append(left_edge - last_x)
            right_edge = center + width/2 - half_length
            edges.append([0., right_edge, 0., 1.])
            last_x = right_edge
        edges.append([0., half_length, 0., 1.])
        widths.append(half_length-last_x)
        edges = np.array(edges).T
        edges = transform.aboutZ(wall.theta).dot(edges)
        edges = transform.translate(wall.x,wall.y).dot(edges)
        obst = []
        for i in range(0,len(widths)):
            center = edges[:,2*i:2*i+2].mean(1).reshape(4,1)
            dimensions=(4.0, widths[i])
            r = Rectangle(center=center,
                          dimensions=dimensions,
                          orient=wall.theta )
            obst.append(r)
        return obst

    def generate_cube_obstacle(self,obj):
        r = Rectangle(center=transform.point(obj.x, obj.y),
                      dimensions=obj.size[0:2],
                      orient=obj.theta)
        return r
    
