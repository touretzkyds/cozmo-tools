import numpy as np

class WaveFront():
    def __init__(self, square_size=10, grid_size=(100,100)):
        self.square_size = square_size
        self.grid_size = grid_size
        self.grid = np.ndarray(grid_size, dtype=np.int32)

    def search(self):
        grid = self.grid
        grid[50,50] = 1
        grid[99,99] = -1
        new_fringe = [(50,50)]
        xmax = self.grid_size[0] - 1
        ymax = self.grid_size[1] - 1
        dist = 0
        while new_fringe:
            dist +=1
            fringe = new_fringe
            new_fringe = []
            while fringe:
                (x,y) = fringe.pop()
                if x > 0:
                    cell = grid[x-1, y]
                    if cell == -1: return True
                    elif cell == 0:
                        grid[x-1,y] = dist
                        new_fringe.append((x-1,y))
                if x <xmax:
                    cell = grid[x+1, y]
                    if cell == -1: return True
                    elif cell == 0:
                        grid[x+1,y] = dist
                        new_fringe.append((x+1,y))
                if y > 0:
                    cell = grid[x, y-1]
                    if cell == -1: return True
                    elif cell == 0:
                        grid[x,y-1] = dist
                        new_fringe.append((x,y-1))
                if y < ymax:
                    cell = grid[x, y+1]
                    if cell == -1: return True
                    elif cell == 0:
                        grid[x,y+1] = dist
                        new_fringe.append((x,y+1))


w = WaveFront()
w.search()
print(w.grid[45:55, 45:55])
