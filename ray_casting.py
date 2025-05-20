https://aaaa.sh/creatures/dda-algorithm-interactive/
import numpy as np
import math
from grid_mapping import angle_to_quaternion


def one_ray_cast(map, r_pos_map, angle, p, q, res):

    i_max = map.shape[0]
    j_max = map.shape[1]

    rd = [np.cos(angle), np.sin(angle)]
    
    step = np.sign(rd)

    theta = angle_to_quaternion(q[0], q[1], q[2], q[3])
    dx = r_pos_map[0] - p[0]
    dy = r_pos_map[1] - p[1]

    x_loc = dx * np.cos(theta) + dy * np.sin(theta)
    y_loc = dy * np.cos(theta) - dx * np.sin(theta)

    j_float = x_loc / res - 0.5
    i_float = y_loc / res - 0.5
    j = math.floor(j_float)
    i = math.floor(i_float)

    frac = [j_float - j, i_float - i] 

    unit_step = [np.sqrt(1 + (rd[1]/rd[0])**2),np.sqrt(1 + (rd[0]/rd[1])**2) ]
    #unit_step = [ abs(1 / rd[0]) if rd[0] != 0 else float('inf'),abs(1 / rd[1]) if rd[1] != 0 else float('inf')]


    ray_length = np.zeros(2)
    if (rd[0] < 0):
        ray_length[0] = frac[0] * unit_step[0]
    else:
        ray_length[0] = (1 - frac[0]) * unit_step[0]

    if (rd[1] < 0):
        ray_length[1] = frac[1] * unit_step[1]
    else:
        ray_length[1] = (1 - frac[1]) * unit_step[1]


    while 1:
        if i < 0 or j < 0 or i >= i_max or j >= j_max:
            return False, None, None
        
        if map[i,j] == 1:
            return True, i, j
        
        else: 
            if (ray_length[0] < ray_length[1]):
                j += step[0]
                ray_length[0] += unit_step[0]
            
            else:       
                i += step[1]
                ray_length[1] += unit_step[1]


def raycast_360(occupancy_map, r_pos_map, p, q, res, n_rays=360):


    for i in range(n_rays):
        angle = i * 2 * np.pi / n_rays
        hit, i, j = one_ray_cast(occupancy_map, r_pos_map, angle, p, q, res)
        if hit:
            occupancy_map[i,j] = 2  #à voir ce qu'on veut faire avec 


