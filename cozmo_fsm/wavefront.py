"""
Wavefront path planning algorithm.
"""

import numpy as np
import heapq
from math import ceil, cos, sin
from .transform import wrap_angle
from .rrt_shapes import *

class WaveFront():
    def __init__(self, square_size=10, bbox=None, grid_size=(100,100), inflate_size=5):
        self.square_size = square_size
        if bbox:
            self.grid_size = (int(bbox[1][0]-bbox[0][0]+2*inflate_size),
                              int(bbox[1][1]-bbox[0][1]+2*inflate_size))
        else:
            self.grid_size = grid_size
        self.grid = np.zeros(self.grid_size, dtype=np.int32)
        self.goal_marker = 2**31 - 1

    def initialize_grid(self,bbox=None,inflate_size=5):
        if bbox:
            self.grid_size = (int(bbox[1][0]-bbox[0][0]+2*inflate_size),
                              int(bbox[1][1]-bbox[0][1]+2*inflate_size))
        self.grid = np.zeros(self.grid_size, dtype=np.int32)

    def convert_coords(self,xcoord,ycoord):
        "Convert world map coordinates to grid subscripts."
        x = int(round((xcoord/self.square_size+self.grid_size[0]/2)))
        y = int(round((ycoord/self.square_size+self.grid_size[1]/2)))
        if x >= 0 and x < self.grid_size[0] and \
           y >= 0 and y < self.grid_size[1]:
            return (x,y)
        else:
            return (None,None)

    def set_obstacle_cell(self,xcoord,ycoord):
        (x,y) = self.convert_coords(xcoord,ycoord)
        if x:
            self.grid[x,y] = -1

    def add_obstacle(self, obstacle, inflate_size=0):
        if isinstance(obstacle, Rectangle):
            centerX, centerY = obstacle.center[0,0], obstacle.center[1,0]
            width, height = obstacle.dimensions[0]+inflate_size*2, obstacle.dimensions[1]+inflate_size*2
            theta = wrap_angle(obstacle.orient)
            for x in range(int(round(centerX-width/2)), int(ceil(centerX+width/2))):
                for y in range(int(round(centerY-height/2)), int(ceil(centerY+height/2))):
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
        """Temporary hack. Should me tracing perimeter of object."""
        self.set_goal_cell(obj.center[0,0], obj.center[1,0])
        self.set_goal_cell(obj.center[0,0]+25, obj.center[1,0]+25)
        self.set_goal_cell(obj.center[0,0]-25, obj.center[1,0]-25)

    def propagate(self,xstart,ystart):
        """
        Propagate the wavefront in eight directions from the starting coordinates
        until a goal cell is reached or we fill up the grid.
        """
        grid = self.grid
        (x,y) = self.convert_coords(xstart,ystart)
        if grid[x,y] != 0:
            raise ValueError("Start collides")
            
        goal_marker = self.goal_marker
        fringe = [(1,(x,y))]
        heapq.heapify(fringe)
        xmax = self.grid_size[0] - 1
        ymax = self.grid_size[1] - 1
        while fringe:
            dist,(x,y) = heapq.heappop(fringe)
            if grid[x,y] == 0:
                grid[x,y] = dist
            else:
                continue
            dist10 = dist + 10
            dist14 = dist + 14
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
        xmax = self.grid_size[0] - 1
        ymax = self.grid_size[1] - 1
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
        xmid = self.grid_size[0]/2
        ymid = self.grid_size[1]/2
        path_coords = [((x-xmid)*square_size, (y-ymid)*square_size) for (x,y) in path]
        return path_coords

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
