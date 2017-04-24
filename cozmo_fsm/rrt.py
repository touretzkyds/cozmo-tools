from math import pi, sin, cos, inf, asin, atan2, nan, isnan
import numpy as np
import random
import time

import cozmo_fsm.transform
from .transform import wrap_angle
from .rrt_shapes import *
from .worldmap import WallObst, wall_marker_dict, LightCubeObst, CustomCubeObst

# *** TODO: Collision checking needs to use opposite headings
# for treeB nodes because robot is asymmetric.

#---------------- RRTNode ----------------

class RRTNode():
    def __init__(self, parent=None, x=0, y=0, q=0, radius=0):
        self.parent = parent
        self.x = x
        self.y = y
        self.q = q
        self.radius = radius

    def copy(self):
        return RRTNode(self.parent, self.x, self.y, self.q, self.radius)

    def __repr__(self):
        rad = "" if self.radius == 0 else (" rad. %d" % self.radius)
        return '<RRTNode (%.1f,%.1f)@%d deg%s>' % (self.x, self.y, self.q/pi*180, rad)


#---------------- RRT Path Planner ----------------

class StartCollides(Exception): pass
class GoalCollides(Exception): pass
class MaxIterations(Exception): pass

class RRT():
    def __init__(self, robot, max_iter=1000, step_size=10, arc_radius=40,
                 xy_tolsq=90, q_tol=5/180*pi,
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
        distsq = dx*dx + dy*dy
        q = atan2(dy,dx)
        dq = wrap_angle(q - node.q)
        if abs(dq) > self.max_turn:
            dq = self.max_turn if dq > 0 else -self.max_turn
            q = wrap_angle(node.q + dq)
            #return self.arc_interpolate(node,target,dq)
        if abs(dq) >= self.q_tol:
            # Must be able to turn to the new heading without colliding
            turn_dir = +1 if dq >= 0 else -1
            q_inc = turn_dir * self.q_tol
            while abs(q_inc - dq) > self.q_tol:
                if self.collides(RRTNode(x=node.x, y=node.y, q=node.q+q_inc)):
                    return (self.COLLISION, None)
                q_inc += turn_dir * self.q_tol
        if distsq < self.xy_tolsq:
            return (self.REACHED, RRTNode(parent=node, x=target.x, y=target.y,q=q))
        xstep = self.step_size * cos(q)
        ystep = self.step_size * sin(q)
        new_node = RRTNode(parent=node, x=node.x+xstep, y=node.y+ystep, q=q)
        if self.collides(new_node):
            return (self.COLLISION, None)
        else:
            return (self.INTERPOLATE, new_node)

    def arc_interpolate(self,node,target,turn_angle):
        if turn_angle > 0:
            theta = node.q + pi/2
            rad = self.arc_radius
        else:
            theta = node.q - pi/2
            rad = -self.arc_radius
        # center of arc
        cx = node.x + self.arc_radius * cos(theta)
        cy = node.y + self.arc_radius * sin(theta)
        # destination if we turn through turn_angle
        dq = node.q + turn_angle
        dx = cx + self.arc_radius*cos(dq)
        dy = cy + self.arc_radius*sin(dq)
        new_node = RRTNode(parent=node, x=dx, y=dy, q=dq, radius=rad)
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

    def plan_push_chip(self, start, goal, max_turn=20*(pi/180), arc_radius=40.):
        return self.plan_path(start, goal, max_turn, arc_radius)

    def plan_path(self, start, goal, max_turn=pi, arc_radius=0):
        self.max_turn = max_turn
        self.arc_radius = arc_radius
        if self.auto_obstacles:
            self.generate_obstacles()
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
        pathA = [nodeA.copy()]
        while nodeA.parent is not None:
            nodeA = nodeA.parent
            pathA.append(nodeA.copy())
        pathA.reverse()
        # treeB was built backwards from the goal, so headings
        # need to be reversed
        goal_q = treeB[0].q
        nodeB = treeB[-1]
        pathB = []
        prev_heading = wrap_angle(nodeB.q + pi)
        while nodeB.parent is not None:
            nodeB = nodeB.parent
            (nodeB.q, prev_heading) = (prev_heading, wrap_angle(nodeB.q+pi))
            pathB.append(nodeB.copy())
        self.path = pathA + pathB
        self.smooth_path()
        if not isnan(goal_q):
            # Last node turns to desired final heading
            last = self.path[-1]
            goal = RRTNode(parent=None, x=last.x, y=last.y, q=goal_q)
            self.path.append(goal)
        return (treeA, treeB, self.path)

    def smooth_path(self):
        """Smooth a path by picking random subsequences and replacing
        them with a direct link if there is no collision."""
        smoothed_path = self.path
        for _ in range(0,len(smoothed_path)):
            i = random.randrange(0,len(smoothed_path)-1)
            cur_x = smoothed_path[i].x
            cur_y = smoothed_path[i].y
            cur_q = smoothed_path[i].q
            j = random.randrange(i+1, len(smoothed_path))
            dx = smoothed_path[j].x - cur_x
            dy = smoothed_path[j].y - cur_y
            new_q = atan2(dy,dx)
            dist = sqrt(dx**2 + dy**2)
            turn_angle = wrap_angle(new_q - cur_q)
            if abs(turn_angle) <= self.max_angle:
                smoothed_path = self.try_linear_smooth(smoothed_path,i,j,cur_x,cur_y,new_q,dist)
            else:
                smoothed_path = self.try_arc_smooth(smoothed_path,i,j,cur_x,cur_y,cur_q,turn_angle)
        self.path = smoothed_path

    def try_linear_smooth(self,smoothed_path,i,j,cur_x,cur_y,new_q,dist):
        step_x = self.step_size * cos(new_q)
        step_y = self.step_size * sin(new_q)
        traveled = 0
        while traveled < dist:
            traveled += self.step_size
            cur_x += step_x
            cur_y += step_y
            if self.collides(RRTNode(None, cur_x, cur_y, new_q)):
                return smoothed_path
        # no collision, so snip out nodes i+1 ... j-1
        smoothed_path[j].parent = smoothed_path[i]
        smoothed_path[j].q = new_q
        smoothed_path = smoothed_path[:i+1] + smoothed_path[j:]
        return smoothed_path

    def try_arc_smooth(self,smoothed_path,i,j,cur_x,cur_y,cur_q,direct_turn_angle):
        # find center of arc we'll be moving along
        dir = +1 if direct_turn_angle >=0 else -1
        center = transform.translate(cur_x,cur_y).dot(
            transform.aboutZ(cur_q+dir*pi/2).dot(transform.point(self.arc_radius))
            )
        dest_x = smoothed_path[j].x
        dest_y = smoothed_path[j].y
        dx = dest_x - center[0,0]
        dy = dest_y - center[1,0]
        center_dist = sqrt(dx*dx + dy*dy)
        # find tangent points on arc: outer tangent formula from Wikipedia with r=0
        gamma = atan2(dy, dx)
        beta = asin(self.arc_radius / center_dist)
        alpha = gamma - beta
        tang_x1 = center[0,0] + self.arc_radius * cos(pi/2 - alpha)
        tang_y1 = center[1,0] + self.arc_radius * sin(pi/2 - alpha)
        tang_q1 = (atan2(tang_y1-center[1,0], tang_x1-center[0,0]) + dir*pi/2)
        turn1 = tang_q1 - cur_q
        if dir == +1 and turn1 < 0:
            turn1 += 2*pi
        elif dir == -1 and turn1 > 0:
            turn1 += -2*pi
        tang_x2 = center[0,0] + self.arc_radius * cos(pi/2 + alpha)
        tang_y2 = center[1,0] + self.arc_radius * sin(pi/2 + alpha)
        tang_q2 = (atan2(tang_y2-center[1,0], tang_x2-center[0,0]) + dir*pi/2)
        turn2 = tang_q2 - cur_q
        if dir == +1 and turn2 < 0:
            turn2 += 2*pi
        elif dir == -1 and turn2 > 0:
            turn2 += -2*pi
        print('alpha=',alpha*180/pi,'  beta=',beta*180/pi,'  gamma=',gamma*180/pi)
        print('dir=%d tang1=(%.1f,%.1f) tang_q1=%.1f turn1=%.1f' %
              (dir,tang_x1,tang_y1,tang_q1*180/pi,turn1*180/pi))
        print('dir=%d tang2=(%.1f,%.1f) tang_q2=%.1f turn2=%.1f' %
              (dir,tang_x2,tang_y2,tang_q2*180/pi,turn2*180/pi))
        transform.tprint(center)
        # correct tangent point has shortest turn
        if abs(turn1) < abs(turn2):
            (tang_x,tang_y,tang_q,turn) = (tang_x1,tang_y1,tang_q1,turn1)
        else:
            (tang_x,tang_y,tang_q,turn) = (tang_x2,tang_y2,tang_q2,turn2)
        # interpolate along the arc and check for collision
        q_traveled = 0
        while abs(q_traveled) < abs(turn):
            cur_x = center[0,0] + self.arc_radius * cos(cur_q + q_traveled)
            cur_y = center[0,0] + self.arc_radius * sin(cur_q + q_traveled)
            if self.collides(RRTNode(None, cur_x, cur_y, cur_q+q_traveled)):
                return smoothed_path
            q_traveled += dir * self.q_tol
        print('no collision.  q_traveled =',q_traveled*180/pi)
        # Now interpolate from the tangent point to the target
        cur_x = tang_x
        cur_y = tang_y
        dx = dest_x - cur_x
        dy = dest_y - cur_y
        new_q = atan2(dy, dx)
        dist = sqrt(dx*dx + dy*dy)
        step_x = self.step_size * cos(new_q)
        step_y = self.step_size * sin(new_q)
        traveled = 0
        while traveled < dist:
            traveled += self.step_size
            cur_x += step_x
            cur_y += step_y
            if self.collides(RRTNode(None, cur_x, cur_y, new_q)):
                return smoothed_path
        # no collision, so snip out nodes i+1 ... j-1 and insert a turn node
        ni = smoothed_path[i]
        turn_node = RRTNode(ni, tang_x, tang_y, tang_q, radius=self.arc_radius)
        smoothed_path[j].parent = turn_node
        smoothed_path[j].q = new_q
        smoothed_path = smoothed_path[:i+1] + [turn_node] + smoothed_path[j:]
        print(smoothed_path)
        return smoothed_path        

    def make_robot_parts(self,robot):
        result = []
        for joint in robot.kine.joints.values():
            if joint.collision_model:
                tmat = robot.kine.link_to_base(joint)
                robot_obst = joint.collision_model.instantiate(tmat)
                result.append(robot_obst)
        return result

    def generate_obstacles(self):
        self.robot.world.world_map.update_map()
        obstacles = []
        for obj in self.robot.world.world_map.objects.values():
            if isinstance(obj, WallObst):
                obstacles = obstacles + self.generate_wall_obstacles(obj)
            elif isinstance(obj, (LightCubeObst,CustomCubeObst)):
                obstacles.append(self.generate_cube_obstacle(obj))
        self.obstacles = obstacles

    def generate_wall_obstacles(self,wall):
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
    
