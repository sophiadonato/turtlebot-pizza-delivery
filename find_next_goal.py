import numpy as np
from collections import deque
import matplotlib.pyplot as plt

def find_next_goal(grid):
    frontier_cells = []
    neighboors = [(-1, 0), (1,0), (0,-1), (0,1), (-1,-1), (-1, 1), (1, 1), (1, -1)]

    # Finding all frontier cells
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i][j] == 0:
                is_frontier = False
                next_to_wall = False
                for di,dj in neighboors:
                    neighboor_i, neighboor_j = i+di, j+dj
                    if 0<= neighboor_i < grid.shape[0] and 0<= neighboor_j < grid.shape[1]:
                        if grid[neighboor_i][neighboor_j] == -1 :
                            is_frontier = True
                        if grid[neighboor_i][neighboor_j] == 1:
                            next_to_wall = True
                if is_frontier and not next_to_wall:
                    frontier_cells.append((i,j))

    
    # Putting cells into different frontiers
    frontiers = []
    visited = set()

    for cell in frontier_cells:
        if cell not in visited:
            frontier = []
            queue = deque([cell])
            visited.add(cell)

            while queue:
                current_cell = queue.pop()
                frontier.append(current_cell)
                for di,dj in neighboors:
                    neighboor_i, neighboor_j = current_cell[0]+di, current_cell[1]+dj
                    if 0<= neighboor_i < grid.shape[0] and 0<= neighboor_j < grid.shape[1]:
                        if (neighboor_i, neighboor_j) in frontier_cells and (neighboor_i, neighboor_j) not in visited:
                            visited.add((neighboor_i, neighboor_j))
                            queue.append((neighboor_i, neighboor_j))
            frontiers.append(frontier)

    # Removing frontier whose length is too small 
    # Frontier could be too small because of 2 reasons : "bad" prepocessing, or too close of a wall
    frontiers_copy = frontiers.copy()
    for frontier in frontiers_copy:
        if len(frontier) < 10:
            frontiers.remove(frontier)

    # If not frontiers map explo is finished


    # Finding middle of frontiers
    frontier_middles = []
    for frontier in frontiers : 
        frontier = np.array(frontier)
        centroid = np.mean(frontier, axis=0)
        min_dist = np.inf
        for i, j in frontier:
            if np.sqrt((i-centroid[0])**2 + (j-centroid[1])**2) < min_dist:
                min_dist = np.sqrt((i-centroid[0])**2 + (j-centroid[1])**2)
                middle = (i,j)
        frontier_middles.append(middle)

    print(frontier_middles)

    
    # Finding closest point to go to it
    return 

matrix = np.loadtxt('matrix.txt')
find_next_goal(matrix)
    

                        