# Planning

The goal is to visit all arucos that have been sent to us trough. The list of customers' Id's, published as a ```std_msgs.msg.Int32MultiArray``` message on the ROS2 topic ```/delivery_locations```. The publication of this list may begin at any time while the robot is exploring and mapping the environment and will continue until the end of the demonstration.  

## Poses to visit
The Poses to visit are accessible with listening to a topic from Aruco_subcriber node.  
This topic sends the locations of all the place to go in other to visit the arucos

## Planning of the visit
The starting position will be the actual turtlebot Pose.  
We have a small number of aruco to visit (less than 10). We therefore have a small TSP (Travel Salesman Problem). 
We need to know the length of the path between all these arucos.

## Path duration between two positions to visit
The path can be measured using between two poses can be measured using ```from nav2_msgs.srv import ComputePathToPos```  
What is important is to consider that there are walls, we therefore need to get all the poses that are travelled to be able to measure the distance.  
The path is not symmetric (due to pose orientation) but it can be considered as symmetric in our problem setting => Only the euclidean distance between two adjecent poses is measured.

## TSP
To use TCP, we can use python_tsp (Docs : https://pypi.org/project/python_tsp/).  
To install the TCP module : ```pip install python-tsp```  
To import : ```from python_tsp.exact import solve_tsp_dynamic_programming```  
To use TCP, we have distance matrix between all the pose to visit and we call ```permutation, distance = solve_tsp_dynamic_programming(distance_matrix)```
TCP works for closed loop => In this project, we don't need to be back to inidistance_matrix[:, 0] = 0tial position  
We use open version of the TCP => set all elements of the first column of the distance matrix to zero ```distance_matrix[:, 0] = 0 ```