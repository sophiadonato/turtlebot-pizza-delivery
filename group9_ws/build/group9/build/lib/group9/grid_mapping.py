import numpy as np 
import math
from scipy.ndimage import binary_dilation


def angle_to_quaternion(qx, qy, qz, qw):
    return math.atan2(2*(qw * qz + qx * qy), 1-2*(qy**2 + qz**2))

def index_to_real_world(i, j, res, p, q):
    """ Fonction that transform the index of the Occupancy Grid to a real world position
        where the real world origin is determined when the robot is launched
        
        argument:
            - i: index of the column in the Occupancy Grid
            - j: index of the line in the Occupancy Grid
            - res: resolution of the Occupancy Grid
            - p: vector of 3 dimensions = coordinate of the origin of the Occupancy Grid
            - q: vector of 4 dimensions = quaternion of the Occupancy Grid with respect to the real world
            
        return: 
            x, y: the coordinate of the cellule(j,i) in the real world
    """
    theta = angle_to_quaternion(q[0], q[1], q[2], q[3])
    
    x_loc = (i + 0.5) * res
    y_loc = (j + 0.5) * res
    
    dx = x_loc * np.cos(theta) - y_loc * np.sin(theta)
    dy = x_loc * np.sin(theta) +  y_loc * np.cos(theta)
    
    x = p[0] + dx
    y = p[1] + dy
    
    return x , y

def load_grid(grid, data):
    world_shape = [1000, 1000]
    world_res = 0.005
    world_center = [500, 500]
    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    data = grid.data
    data = np.reshape(data, (height, width))
    origin = grid.info.origin
    p = np.array([origin.position.x, origin.position.y, origin.position.z])
    q = np.array([origin.quaternion.x, origin.quaternion.y, origin.quaternion.z, origin.quaternion.w])
    for i in width:
        for j in height:
            x, y = index_to_real_world(i, j, resolution, p, q)
            
            j_world = int(world_center[0] - y / world_res)  # raw
            i_world = int(world_center[1] + x / world_res)  # column

            if 0<= i_world < world_shape[1] and 0<=  j_world < world_shape[0]:
                data[i_world, j_world] = data[i,j]

    
    data[data > 0.7] = 1.0
    data[(data < 0.3) & (data > 0)] = 0.0
    unknown = (data >= 0.3) & (data <= 0.7)
    data[unknown] = -1
    unknown_mask = (data == -1)
    data[unknown_mask] = 0.0
    for i in range(height):
        for j in range(width):
            if i<height-1 and j<width-1:
                if data[i,j] == 1:
                        if data[i-1,j] == 0 and data[i+1,j] == 0 and data[i,j-1] == 0 and data[i,j+1] == 0:
                            data[i,j] = 0
                else: 
                    if data[i-1,j] == 1 and data[i+1,j] == 1 and data[i,j-1] == 1 and data[i,j+1] == 1:
                        data[i,j] = 1                  
    unknown_mask = unknown_mask & (data == 0)
    final_matrix = binary_dilation(data, iterations=1)
    final_matrix = final_matrix.astype(int)
    final_matrix[unknown_mask] = -1
    
    return final_matrix
                     
    
