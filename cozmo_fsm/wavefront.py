"""
Wavefront path planning algorithm.
"""

import numpy as np
import heapq

class WaveFront():
    def __init__(self, square_size=10, grid_size=(100,100)):
        self.square_size = square_size
        self.grid_size = grid_size
        self.grid = np.ndarray(grid_size, dtype=np.int32)
        self.goal_marker = 2**31 - 1

    def convert_coords(self,xcoord,ycoord):
        "Convert world map coordinates to grid subscripts."
        x = round((xcoord/self.square_size+self.grid_size[0]/2))
        y = round((ycoord/self.square_size+self.grid_size[1]/2))
        if x >= 0 and x < self.grid_size[0] and \
           y >= 0 and y < self.grid_size[1]:
            return (x,y)
        else:
            return (None,None)

    def set_obstacle(self,xcoord,ycoord):
        (x,y) = self.convert_coords(xcoord,ycoord)
        if x:
            self.grid[x,y] = -1

    def set_goal(self,xcoord,ycoord):
        (x,y) = self.convert_coords(xcoord,ycoord)
        if x:
            self.grid[x,y] = self.goal_marker
        else:
            raise ValueError('Coordinates %s are outside the wavefront grid' % ((xcoord,ycoord)))

    def propagate(self,xstart,ystart):
        "Propagate the wavefront from the given starting coordinates until a goal cell is reached."
        (x,y) = self.convert_coords(xstart,ystart)
        grid = self.grid
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
    wf.set_goal(*goal)
    wf.set_obstacle(280,280)
    wf.set_obstacle(280,290)
    wf.set_obstacle(290,280)
    wf.set_obstacle(290,290)
    result1 = wf.propagate(*start)
    result2 = wf.extract(result1)
    print('path length =', len(result2))
    print(result2)
    print(wf.grid[75:85, 75:85])
    return result2

wf_test()
