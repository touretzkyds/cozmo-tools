from math import pi, sin, cos, inf, asin, atan2, nan, isnan
import numpy as np
import random
import time

import cozmo_fsm.transform
from .transform import wrap_angle

from .rrt_shapes import *
from .cozmo_kin import center_of_rotation_offset
from .worldmap import WallObj, wall_marker_dict, LightCubeObj, CustomCubeObj, ChargerObj, ChipObj, RobotForeignObj

# *** TODO: Collision checking needs to use opposite headings
# for treeB nodes because robot is asymmetric.

#---------------- RRTNode ----------------

class RRTNode():
    def __init__(self, parent=None, x=0, y=0, q=0, radius=None):
        self.parent = parent
        self.x = x
        self.y = y
        self.q = q
        self.radius = radius  # arc radius

    def copy(self):
        return RRTNode(self.parent, self.x, self.y, self.q, self.radius)

    def __repr__(self):
        if isnan(self.q):
            return '<RRTNode (%.1f,%.1f)>' % (self.x, self.y)
        elif not self.parent:
            return '<RRTNode (%.1f,%.1f)@%d deg>' % \
                   (self.x, self.y, round(self.q/pi*180))
        elif self.radius is None:
            return '<RRTNode line to (%.1f,%.1f)@%d deg>' % \
                   (self.x, self.y, round(self.q/pi*180))
        else:
            return '<RRTNode arc to (%.1f,%.1f)@%d deg, rad=%d>' % \
                   (self.x, self.y, round(self.q/pi*180), self.radius)


#---------------- RRT Path Planner ----------------

class RRTException(Exception):
    def __str__(self):
        return self.__repr__()

class StartCollides(RRTException): pass
class GoalCollides(RRTException): pass
class MaxIterations(RRTException): pass

class RRT():
    def __init__(self, robot, max_iter=2000, step_size=10, arc_radius=40,
                 xy_tolsq=90, q_tol=5*pi/180,
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
        #time.sleep(0.01)   # *** FOR ANIMATION PURPOSES
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

    def robot_parts_to_node(self,node):
        parts = []
        for part in self.robot_parts:
            tmat = transform.aboutZ(part.orient)
            tmat = transform.translate(part.center[0,0], part.center[1,0]).dot(tmat)
            tmat = transform.aboutZ(node.q).dot(tmat)
            tmat = transform.translate(node.x, node.y).dot(tmat)
            this_part = part.instantiate(tmat)
            parts.append(this_part)
        return parts

    def collides(self, node):
        for part in self.robot_parts_to_node(node):
            for obstacle in self.obstacles:
                if part.collides(obstacle):
                    return obstacle
        return False        

    def plan_push_chip(self, start, goal, max_turn=20*(pi/180), arc_radius=40.):
        return self.plan_path(start, goal, max_turn, arc_radius)

    def plan_path(self, start, goal, max_turn=pi, arc_radius=40):
        self.max_turn = max_turn
        self.arc_radius = arc_radius
        if self.auto_obstacles:
            self.generate_obstacles()
        self.start = start
        self.goal = goal
        self.target_heading = goal.q

        # Set up start node
        collider = self.collides(start)
        if collider:
            raise StartCollides(start,collider,collider.obstacle)
        else:
            treeA = [start.copy()]
            self.treeA = treeA

        # Set up goal node(s)
        if not isnan(self.target_heading):
            offset_x = goal.x + center_of_rotation_offset * cos(goal.q)
            offset_y = goal.y + center_of_rotation_offset * sin(goal.q)
            offset_goal = RRTNode(x=offset_x, y=offset_y, q=goal.q)
            treeB = [offset_goal]
            self.treeB = treeB
            collider = self.collides(offset_goal)
            if collider:
                raise GoalCollides(goal,collider,collider.obstacle)
        else:  # target_heading is nan
            treeB = [goal.copy()]
            self.treeB = treeB
            temp_goal = goal.copy()
            offset_goal = goal.copy()
            for theta in range(0,360,10):
                q = theta/180*pi
                step = max(self.step_size, abs(center_of_rotation_offset))
                temp_goal.x = goal.x + step*cos(q)
                temp_goal.y = goal.y + step*sin(q)
                temp_goal.q = wrap_angle(q+pi)
                collider = self.collides(temp_goal)
                if collider: continue
                offset_goal.x = temp_goal.x + center_of_rotation_offset * cos(q)
                offset_goal.y = temp_goal.y + center_of_rotation_offset * sin(q)
                offset_goal.q = temp_goal.q
                collider = self.collides(offset_goal)
                if not collider:
                    treeB.append(RRTNode(parent=treeB[0], x=temp_goal.x, y=temp_goal.y, q=temp_goal.q))
            if len(treeB) == 1:
                raise GoalCollides(goal,collider,collider.obstacle)

        # Set bounds for search area
        self.compute_world_bounds(start,goal)

        # Grow the RRT until trees meet or max_iter exceeded
        swapped = False
        for i in range(self.max_iter):
            r = self.random_node()
            (status, new_node) = self.extend(treeA, r)
            if status is not self.COLLISION:
                (status, new_node) = self.extend(treeB, treeA[-1])
                if status is self.REACHED:
                    break
            (treeA, treeB) = (treeB, treeA)
            swapped = not swapped
        if swapped:
            (treeA, treeB) = (treeB, treeA)
        if status is self.REACHED:
            return self.get_path(treeA, treeB)
        else:
            raise MaxIterations(self.max_iter)

    def compute_world_bounds(self,start,goal):
        xmin = min(start.x, goal.x)
        xmax = max(start.x, goal.x)
        ymin = min(start.y, goal.y)
        ymax = max(start.y, goal.y)
        for obst in self.obstacles:
            xmin = min(xmin, np.min(obst.vertices[0]))
            xmax = max(xmax, np.max(obst.vertices[0]))
            ymin = min(ymin, np.min(obst.vertices[1]))
            ymax = max(ymax, np.max(obst.vertices[1]))
        xmin = xmin - 500
        xmax = xmax + 500
        ymin = ymin - 500
        ymax = ymax + 500
        self.bounds = (range(int(xmin), int(xmax)), range(int(ymin), int(ymax)))        

    def get_path(self, treeA, treeB):
        nodeA = treeA[-1]
        pathA = [nodeA.copy()]
        while nodeA.parent is not None:
            nodeA = nodeA.parent
            pathA.append(nodeA.copy())
        pathA.reverse()
        # treeB was built backwards from the goal, so headings
        # need to be reversed
        nodeB = treeB[-1]
        prev_heading = wrap_angle(nodeB.q + pi)
        if nodeB.parent is None:
            pathB = [nodeB.copy()]
        else:
            pathB = []
            while nodeB.parent is not None:
                nodeB = nodeB.parent
                (nodeB.q, prev_heading) = (prev_heading, wrap_angle(nodeB.q+pi))
                pathB.append(nodeB.copy())
        (pathA,pathB) = self.join_paths(pathA,pathB)
        self.path = pathA + pathB
        self.smooth_path()
        target_q = self.target_heading
        if not isnan(target_q):
            # Last nodes turn to desired final heading
            last = self.path[-1]
            goal = RRTNode(parent=last, x=self.goal.x, y=self.goal.y,
                           q=target_q, radius=0)
            self.path.append(goal)
        return (treeA, treeB, self.path)

    def join_paths(self,pathA,pathB):
        turn_angle = wrap_angle(pathB[0].q - pathA[-1].q)
        if abs(turn_angle) <= self.max_turn:
            return (pathA,pathB)
        print('*** JOIN PATHS EXCEEDED MAX TURN ANGLE: ', turn_angle*180/pi)
        return (pathA,pathB)

    def smooth_path(self):
        """Smooth a path by picking random subsequences and replacing
        them with a direct link if there is no collision."""
        smoothed_path = self.path
        for _ in range(0,len(smoothed_path)):
            L = len(smoothed_path)
            if L == 2: break
            i = random.randrange(0,L-2)
            cur_x = smoothed_path[i].x
            cur_y = smoothed_path[i].y
            cur_q = smoothed_path[i].q
            j = random.randrange(i+2, L)
            if j < L-1 and smoothed_path[j+1].radius != None:
                continue  # j is parent node of an arc segment: don't touch
            dx = smoothed_path[j].x - cur_x
            dy = smoothed_path[j].y - cur_y
            new_q = atan2(dy,dx)
            dist = sqrt(dx**2 + dy**2)
            turn_angle = wrap_angle(new_q - cur_q)
            if abs(turn_angle) <= self.max_turn:
                result = self.try_linear_smooth(smoothed_path,i,j,cur_x,cur_y,new_q,dist)
            else:
                result = self.try_arc_smooth(smoothed_path,i,j,cur_x,cur_y,cur_q)
            smoothed_path = result or smoothed_path
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
                return None
        # Since we're arriving at node j via a different heading than
        # before, see if we need to add an arc to get us to node k=j+1
        node_i = smoothed_path[i]
        end_spec = self.calculate_end(smoothed_path, node_i, new_q, j)
        if end_spec is None:
            return None
        # no collision, so snip out nodes i+1 ... j-1
        # print('linear: stitching','%d:'%i,smoothed_path[i],'to %d:'%j,smoothed_path[j])
        if not end_spec:
            smoothed_path[j].parent = smoothed_path[i]
            smoothed_path[j].q = new_q
            smoothed_path[j].radius = None
            smoothed_path = smoothed_path[:i+1] + smoothed_path[j:]
        else:
            (next_node,turn_node) = end_spec
            smoothed_path[j+1].parent = turn_node
            smoothed_path = smoothed_path[:i+1] + \
                            [next_node, turn_node] + \
                            smoothed_path[j+1:]
        return smoothed_path

    def try_arc_smooth(self,smoothed_path,i,j,cur_x,cur_y,cur_q):
        if j == i+2 and smoothed_path[i+1].radius != None:
            return None  # would be replacing an arc node with itself
        arc_spec = self.calculate_arc(smoothed_path[i], smoothed_path[j])
        if arc_spec is None:
            return None
        (tang_x, tang_y, tang_q, radius) = arc_spec
        ni = smoothed_path[i]
        turn_node1 = RRTNode(ni, tang_x, tang_y, tang_q, radius=radius)
        # Since we're arriving at node j via a different heading than
        # before, see if we need to add an arc at the end to allow us
        # to smoothly proceed to node k=j+1
        end_spec = self.calculate_end(smoothed_path, turn_node1, tang_q, j)
        if end_spec is None:
            return None
        # no collision, so snip out nodes i+1 ... j-1 and insert new node(s)
        # print('arc: stitching','%d:'%i,smoothed_path[i],'to %d:'%j,smoothed_path[j])
        if not end_spec:
            smoothed_path[j].parent = turn_node1
            smoothed_path[j].q = tang_q
            smoothed_path[j].radius = None
            smoothed_path = smoothed_path[:i+1] + [turn_node1] + smoothed_path[j:]
        else:
            (next_node, turn_node2) = end_spec
            smoothed_path[j+1].parent = turn_node2
            smoothed_path = smoothed_path[:i+1] + \
                            [turn_node1, next_node, turn_node2] + \
                            smoothed_path[j+1:]
        return smoothed_path        

    def calculate_arc(self, node_i, node_j):
        # Compute arc node parameters to get us on a heading toward node_j.
        cur_x = node_i.x
        cur_y = node_i.y
        cur_q = node_i.q
        dest_x = node_j.x
        dest_y = node_j.y
        direct_turn_angle = wrap_angle(atan2(dest_y-cur_y, dest_x-cur_x) - cur_q)
        # find center of arc we'll be moving along
        dir = +1 if direct_turn_angle >=0 else -1
        cx = cur_x + self.arc_radius * cos(cur_q + dir*pi/2)
        cy = cur_y + self.arc_radius * sin(cur_q + dir*pi/2)
        dx = cx - dest_x 
        dy = cy - dest_y
        center_dist = sqrt(dx*dx + dy*dy)
        if center_dist < self.arc_radius:  # turn would be too wide: punt
            return None
        # tangent points on arc: outer tangent formula from Wikipedia with r=0
        gamma = atan2(dy, dx)
        beta = asin(self.arc_radius / center_dist)
        alpha1 = gamma + beta
        tang_x1 = cx + self.arc_radius * cos(alpha1 + pi/2)
        tang_y1 = cy + self.arc_radius * sin(alpha1 + pi/2)
        tang_q1 = (atan2(tang_y1-cy, tang_x1-cx) + dir*pi/2)
        turn1 = tang_q1 - cur_q
        if dir * turn1 < 0:
            turn1 += dir * 2 * pi
        alpha2 = gamma - beta
        tang_x2 = cx + self.arc_radius * cos(alpha2 - pi/2)
        tang_y2 = cy + self.arc_radius * sin(alpha2 - pi/2)
        tang_q2 = (atan2(tang_y2-cy, tang_x2-cx) + dir*pi/2)
        turn2 = tang_q2 - cur_q
        if dir * turn2 < 0:
            turn2 += dir * 2 * pi
        # Correct tangent point has shortest turn.
        if abs(turn1) < abs(turn2):
            (tang_x,tang_y,tang_q,turn) = (tang_x1,tang_y1,tang_q1,turn1)
        else:
            (tang_x,tang_y,tang_q,turn) = (tang_x2,tang_y2,tang_q2,turn2)
        # Interpolate along the arc and check for collision.
        q_traveled = 0
        while abs(q_traveled) < abs(turn):
            cur_x = cx + self.arc_radius * cos(cur_q + q_traveled)
            cur_y = cy + self.arc_radius * sin(cur_q + q_traveled)
            if self.collides(RRTNode(None, cur_x, cur_y, cur_q+q_traveled)):
                return None
            q_traveled += dir * self.q_tol
        # Now interpolate from the tangent point to the target.
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
                return None
        # No collision, so arc is good.
        return (tang_x, tang_y, tang_q, dir*self.arc_radius)

    def calculate_end(self, smoothed_path, parent, new_q, j):
        # Return False if arc not needed, None if arc not possible,
        # or pair of new nodes if arc is required.
        if  j == len(smoothed_path)-1:
            return False
        node_j = smoothed_path[j]
        node_k = smoothed_path[j+1]
        next_turn = wrap_angle(node_k.q - new_q)
        if abs(next_turn) <= self.max_turn:
            return False
        dist = sqrt((node_k.x-node_j.x)**2 + (node_k.y-node_j.y)**2)
        if False and dist < self.arc_radius:
            return None
        next_x = node_j.x - self.arc_radius * cos(new_q)
        next_y = node_j.y - self.arc_radius * sin(new_q)
        next_node = RRTNode(parent, next_x, next_y, new_q)
        arc_spec = self.calculate_arc(next_node, node_k)
        if arc_spec is None:
            return None
        (tang_x, tang_y, tang_q, radius) = arc_spec
        turn_node = RRTNode(next_node, tang_x, tang_y, tang_q, radius=radius)
        return (next_node, turn_node)

    #---------------- Obstacle Representation ----------------

    def generate_obstacles(self):
        self.robot.world.world_map.update_map()
        obstacles = []
        for obj in self.robot.world.world_map.objects.values():
            if not obj.obstacle: continue
            if self.robot.carrying is obj: continue
            if obj.pose_confidence < 0: continue
            if isinstance(obj, WallObj):
                obstacles = obstacles + self.generate_wall_obstacles(obj)
            elif isinstance(obj, (LightCubeObj,CustomCubeObj,ChargerObj)):
                obstacles.append(self.generate_cube_obstacle(obj))
            elif isinstance(obj, ChipObj):
                obstacles.append(self.generate_chip_obstacle(obj))
            elif isinstance(obj, RobotForeignObj):
               obstacles.append(self.generate_foreign_obstacle(obj))
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
            r.obstacle = wall
            obst.append(r)
        print('wall obst:',obst)
        return obst

    def generate_cube_obstacle(self,obj):
        r = Rectangle(center=transform.point(obj.x, obj.y),
                      dimensions=obj.size[0:2],
                      orient=obj.theta)
        r.obstacle = obj
        return r

    def generate_chip_obstacle(self,obj):
        r = Circle(center=transform.point(obj.x,obj.y),
                   radius=obj.radius)
        r.obstacle = obj
        return r

    def generate_foreign_obstacle(self,obj):
        r = Rectangle(center=transform.point(obj.x, obj.y),
                      dimensions=(obj.size[0:2]),
                      orient=obj.theta)
        r.obstacle = obj
        return r

    def make_robot_parts(self,robot):
        result = []
        for joint in robot.kine.joints.values():
            if joint.collision_model:
                tmat = robot.kine.link_to_base(joint)
                robot_obst = joint.collision_model.instantiate(tmat)
                result.append(robot_obst)
        return result

