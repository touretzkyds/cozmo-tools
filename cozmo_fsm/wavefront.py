"""
Wavefront path planning algorithm.
"""

import numpy as np
import cv2
import heapq
from math import floor, ceil, cos, sin

from .geometry import wrap_angle
from .rrt import StartCollides
from .rrt_shapes import *

class WaveFront():
    def __init__(self, square_size=5, bbox=None, grid_shape=(100,100), inflate_size=50):
        self.square_size = square_size  # in mm
        self.bbox = bbox  # in mm
        self.inflate_size = inflate_size  # in mm
        self.grid_shape = grid_shape  # array shape
        self.goal_marker = 2**31 - 1
        self.initialize_grid(bbox=bbox)

    def initialize_grid(self,bbox=None):
        if bbox:
            self.bbox = bbox
            self.grid_shape = (ceil((bbox[1][0] - bbox[0][0] + 2*self.inflate_size)/self.square_size),
                              ceil((bbox[1][1] - bbox[0][1] + 2*self.inflate_size)/self.square_size))
        self.grid = np.zeros(self.grid_shape, dtype=np.int32)
        self.maxdist = 1

    def convert_coords(self,xcoord,ycoord):
        "Convert world map coordinates to grid subscripts."
        x = int(round((xcoord-self.bbox[0][0]+self.inflate_size)/self.square_size))
        y = int(round((ycoord-self.bbox[0][1]+self.inflate_size)/self.square_size))
        if x >= 0 and x < self.grid_shape[0] and \
           y >= 0 and y < self.grid_shape[1]:
            return (x,y)
        else:
            return (None,None)

    def set_obstacle_cell(self,xcoord,ycoord):
        (x,y) = self.convert_coords(xcoord,ycoord)
        if x:
            self.grid[x,y] = -1

    def add_obstacle(self, obstacle, inflate_size=25):
        if isinstance(obstacle, Rectangle):
            centerX, centerY = obstacle.center[0,0], obstacle.center[1,0]
            width, height = obstacle.dimensions[0]+inflate_size*2, obstacle.dimensions[1]+inflate_size*2
            theta = wrap_angle(obstacle.orient)
            for x in range(floor(centerX-width/2),
                           ceil(centerX+width/2),
                           int(self.square_size/2)):
                for y in range(floor(centerY-height/2),
                               ceil(centerY+height/2),
                               int(self.square_size/2)):
                    new_x = ((x - centerX) * cos(theta) - (y - centerY) * sin(theta)) + centerX
                    new_y = ((x - centerX) * sin(theta) + (y - centerY) * cos(theta)) + centerY
                    self.set_obstacle_cell(new_x, new_y)
        elif isinstance(obstacle, Polygon):
            raise NotImplemented(obstacle)
        elif isinstance(obstacle, Circle):
            raise NotImplemented(obstacle)
        elif isinstance(obstacle, Compound):
            raise NotImplemented(obstacle)
        else:
            raise Exception("%s has no add_obstacle() method defined for %s." % (self, obstacle))

    def set_goal_cell(self,xcoord,ycoord):
        (x,y) = self.convert_coords(xcoord,ycoord)
        if x:
            self.grid[x,y] = self.goal_marker
        else:
            raise ValueError('Coordinates (%s, %s) are outside the wavefront grid' % ((xcoord,ycoord)))

    def set_goal_shape(self,obj):
        """Temporary hack. Should me tracing interior perimeter of the room,
        or setting goal points at each face of the cube."""
        if obj.obstacle_id.startswith('Room'):
            offset = 20   # for rooms
        else:
            offset = 50   # for cubes, charger, markers
        self.set_goal_cell(obj.center[0,0], obj.center[1,0])
        self.set_goal_cell(obj.center[0,0]+offset, obj.center[1,0]+offset)
        self.set_goal_cell(obj.center[0,0]-offset, obj.center[1,0]-offset)

    def check_start_collides(self,xstart,ystart):
        (x,y) = self.convert_coords(xstart,ystart)
        if self.grid[x,y] == 0 or self.grid[x,y] == self.goal_marker:
            return False
        else:
            return True

    def propagate(self,xstart,ystart):
        """
        Propagate the wavefront in eight directions from the starting coordinates
        until a goal cell is reached or we fill up the grid.
        """
        if self.check_start_collides(xstart,ystart):
            raise StartCollides()
            
        grid = self.grid
        (x,y) = self.convert_coords(xstart,ystart)
        goal_marker = self.goal_marker
        fringe = [(1,(x,y))]
        heapq.heapify(fringe)
        xmax = self.grid_shape[0] - 1
        ymax = self.grid_shape[1] - 1
        while fringe:
            dist,(x,y) = heapq.heappop(fringe)
            if grid[x,y] == 0:
                grid[x,y] = dist
            else:
                continue
            dist10 = dist + 10
            dist14 = dist + 14
            self.maxdist = dist14
            if x > 0:
                cell = grid[x-1,y]
                if cell == goal_marker: return (x-1,y)
                elif cell == 0:
                    heapq.heappush(fringe, (dist10,(x-1,y)))
                if y > 0:
                    cell = grid[x-1,y-1]
                    if cell == goal_marker: return (x-1,y-1)
                    elif cell == 0:
                        heapq.heappush(fringe, (dist14,(x-1,y-1)))
                if y < ymax:
                    cell = grid[x-1,y+1]
                    if cell == goal_marker: return (x-1,y+1)
                    elif cell == 0:
                        heapq.heappush(fringe, (dist14,(x-1,y+1)))
            if x < xmax:
                cell = grid[x+1,y]
                if cell == goal_marker: return (x+1,y)
                elif cell == 0:
                    heapq.heappush(fringe, (dist10,(x+1,y)))
                if y > 0:
                    cell = grid[x+1,y-1]
                    if cell == goal_marker: return (x+1,y-1)
                    elif cell == 0:
                        heapq.heappush(fringe, (dist14,(x+1,y-1)))
                if y < ymax:
                    cell = grid[x+1,y+1]
                    if cell == goal_marker: return (x+1,y+1)
                    elif cell == 0:
                        heapq.heappush(fringe, (dist14,(x+1,y+1)))
            if y > 0:
                cell = grid[x,y-1]
                if cell == goal_marker: return (x,y-1)
                elif cell == 0:
                    heapq.heappush(fringe, (dist10,(x,y-1)))
            if y < ymax:
                cell = grid[x,y+1]
                if cell == goal_marker: return (x,y+1)
                elif cell == 0:
                    heapq.heappush(fringe, (dist10,(x,y+1)))
        return None

    def extract(self,search_result):
        "Extract the path once the goal is found, and convert back to worldmap coordinates."
        (x,y) = search_result
        maxdist = self.goal_marker + 1
        grid = self.grid
        xmax = self.grid_shape[0] - 1
        ymax = self.grid_shape[1] - 1
        path = []
        while maxdist > 1:
            path.append((x,y))
            if x > 0:
                if 0 < grid[x-1,y] < maxdist:
                    maxdist = grid[x-1,y]
                    (newx,newy) = (x-1,y)
                if y > 0:
                    if 0 < grid[x-1,y-1] < maxdist:
                        maxdist = grid[x-1,y-1]
                        (newx,newy) = (x-1,y-1)
                if y < ymax:
                    if 0 < grid[x-1,y+1] < maxdist:
                        maxdist = grid[x-1,y+1]
                        (newx,newy) = (x-1,y+1)
            if x < xmax:
                if 0 < grid[x+1,y] < maxdist:
                    maxdist = grid[x+1,y]
                    (newx,newy) = (x+1,y)
                if y > 0:
                    if 0 < grid[x+1,y-1] < maxdist:
                        maxdist = grid[x+1,y-1]
                        (newx,newy) = (x+1,y-1)
                if y < ymax:
                    if 0 < grid[x+1,y+1] < maxdist:
                        maxdist = grid[x+1,y+1]
                        (newx,newy) = (x+1,y+1)
            if y > 0:
                if 0 < grid[x,y-1] < maxdist:
                    maxdist = grid[x,y-1]
                    (newx,newy) = (x,y-1)
            if y < ymax:
                if 0 < grid[x,y+1] < maxdist:
                    maxdist = grid[x,y+1]
                    (newx,newy) = (x,y+1)
            (x,y) = (newx,newy)
        path.append((x,y))
        path.reverse()
        square_size = self.square_size
        xmin = self.bbox[0][0]
        ymin = self.bbox[0][1]
        path_coords = [(x*square_size + xmin - self.inflate_size,
                        y*square_size + ymin - self.inflate_size)
                       for (x,y) in path]
        return path_coords

    def make_grid_display(self):
        scale_factor = 4
        s = self.grid_shape
        x_max = s[0]*scale_factor - 1
        y_max = s[1]*scale_factor - 1
        maxval = float(self.maxdist)
        image = np.zeros((x_max+1, y_max+1, 3), dtype='uint8')
        for i in range(self.grid_shape[0]):
            for j in range(self.grid_shape[1]):
                cell = self.grid[i,j]
                if cell == -1:   # obstacle
                    pixel = (0,0,200)
                elif cell == 0:  # never reached
                    pixel = (255,0,0)
                elif cell == 1:  # starting point
                    pixel = (0,255,255)
                elif cell == self.goal_marker:
                    pixel = (0,200,0)
                else:
                    v = 20 + int(cell/maxval*200)
                    pixel = (v,v,v)
                for i1 in range(scale_factor):
                    for j1 in range(scale_factor):
                        image[x_max-(i*scale_factor+i1), y_max-(j*scale_factor+j1), :] = pixel
        return image

def wf_test():
    start = (261,263)
    goal = (402,454)
    #
    wf = WaveFront()
    wf.grid[:,:] = 0
    wf.set_goal_cell(*goal)
    wf.set_obstacle_cell(280,280)
    wf.set_obstacle_cell(280,290)
    wf.set_obstacle_cell(290,280)
    wf.set_obstacle_cell(290,290)
    result1 = wf.propagate(*start)
    result2 = wf.extract(result1)
    print('path length =', len(result2))
    print(result2)
    print(wf.grid[75:85, 75:85])
    return result2

# wf_test()
