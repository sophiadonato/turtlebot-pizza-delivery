# Procedure to follow

- Get IP : https://tplinkeap.net 
Username : admin, mdp : chiefbot
- ssh ubuntu@{IP} mdp: turtlebot

Check that librairies are well installed:  
For TSP, do ```pip show python_tsp```. If pkg not found, ```pip install python_tsp```

- bring up the robot: ```ros2 launch turtlebot3_bringup robot.launch.py```
- launch cartographer: ```ros2 launch turtlebot3_cartographer cartographer.launch.py```
- launch navigation2 : ```ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:='/home/laptop**/group9/projet/group9_ws/my_params.yaml'```
- launch arucoNode : ```ros2 launch ros2_aruco aruco_recognition.py``` 

Our nodes to run on terminal that are sourced ```source install/setup.bash```  
- ```ros2 run group9 aruco_subscriber``` 
- ```ros2 run group9 robot_pose```
- ```ros2 run group9 raycaster```
- ```ros2 run group9 delivery```
- ```ros2 run group9 map_explorer``` (Run at the end)

For camera POV, ```ros2 run rqt_image_view rqt_image_view```  

In case of problems with numpy,
- Desinstall current numpy version ```pip uninstall numpy``` 
- Install numpy ```pip install numpy==1.19.0```

To start launch file:   
In ```projet\group9_ws\src\group9\launch```, do ```ros2 launch group9```  

To send aruco to visit ```ros2 topic pub /arucos_location std_msgs/msg/Int32MultiArray "{data: [1, 2, 3, 4, 5]}"```
