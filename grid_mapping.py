# -*- coding: utf-8 -*-
"""
Created on Sat Apr 19 13:41:04 2025

@author: Victoria Filée 
"""
import numpy as np 
import math
import matplotlib.pyplot as plt


def angle_to_quaternion(qx, qy, qz, qw):
    return math.atan2(2*(qw * qz + qx * qy), 1-2*(qy**2 + qz**2))

def index_to_real_world(i, j, res, p, q):
    """ Fonction that transform the index of the Occupancy Grid to a real world position
        where the real world origin is determined when the robot is launched
        
        argument:
            - i: index of the line in the Occupancy Grid
            - j: index of the column in the Occupancy Grid
            - res: resolution of the Occupancy Grid
            - p: vector of 3 dimensions = coordinate of the origin of the Occupancy Grid
            - q: vector of 4 dimensions = quaternion of the Occupancy Grid with respect to the real world
            
        return: 
            x, y: the coordinate of the cellule(i,j) in the real world

        /!\: in Occupancy Grid, the cell [0,0] is the low left corner.
    """
    theta = angle_to_quaternion(q[0], q[1], q[2], q[3])
    x_loc = (j + 0.5) * res
    y_loc = (i + 0.5) * res
    
    dx = x_loc * np.cos(theta) - y_loc * np.sin(theta)
    dy = x_loc * np.sin(theta) +  y_loc * np.cos(theta)
    
    x = p[0] + dx
    y = p[1] + dy
    
    return x , y

def world_to_index(x, y, res, p, q):
    """ Fonction that transform the index of the Occupancy Grid to a real world position
        where the real world origin is determined when the robot is launched
        
        argument:
            - x: x-coordinate in the real world
            - y: y-coordinate in the real world
            - res: resolution of the Occupancy Grid
            - p: vector of 3 dimensions = coordinate of the origin of the Occupancy Grid
            - q: vector of 4 dimensions = quaternion of the Occupancy Grid with respect to the real world
            
        return: 
            i, j: index of the point (x,y) in the Occupancy grid

        /!\: in Occupancy Grid, the cell [0,0] is the low left corner.
    """
    theta = angle_to_quaternion(q[0], q[1], q[2], q[3])

    dx = x - p[0]
    dy = y - p[1]

    x_loc = dx * np.cos(theta) + dy * np.sin(theta)
    y_loc = dy * np.cos(theta) - dx * np.sin(theta)

    j = int(x_loc / res - 0.5)
    i = int(y_loc / res - 0.5)

    return i , j

def load_grid(data):
    # width = grid.info.width
    # height = grid.info.height
    # data = grid.data
    # data = np.reshape(data, (height, width))
    # origin = grid.info.origin
    height, width = data.shape
    wall_mask  = (data > 0.6)
    no_mask = (data < 0.3) & (data > 0)
    unknown = (data >= 0.3) & (data <= 0.6)
    data[wall_mask] = 1.0
    data[no_mask] = 0.0
    data[unknown] = -1
    completed_mask = np.zeros_like(data, dtype = bool)
    for i in range(height):
        for j in range(width):
            if i<height-1 and j<width-1:
                if data[i,j] == 1:
                    if data[i-1,j] == 0 and data[i+1,j] == 0 and data[i,j-1] == 0 and data[i,j+1] == 0:
                        data[i,j] = 0
                else: 
                    nb_one = 0
                    if data[i-1,j-1] == 1:
                        nb_one += 1 
                    if data[i-1,j] == 1:
                        nb_one += 1 
                    if data[i-1,j+1] == 1:
                        nb_one += 1  
                    if data[i,j-1] == 1:
                        nb_one += 1
                    if data[i,j+1] == 1:
                        nb_one += 1  
                    if data[i+1,j-1] == 1:
                        nb_one += 1 
                    if data[i+1,j] == 1:
                        nb_one += 1  
                    if data[i+1,j+1] == 1:
                        nb_one += 1 
                    if nb_one >= 1:
                        completed_mask[i,j] = True                  

    data[completed_mask] = 1
    return data
matrix = np.loadtxt('group9_ws/src/group9/matrix.txt')
processed_grid = load_grid(matrix)
plt.imshow(matrix, cmap="viridis")
plt.savefig("processed_map.pdf", dpi=300, bbox_inches='tight')
plt.close()
