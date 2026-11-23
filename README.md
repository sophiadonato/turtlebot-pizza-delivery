# TurtleBot3 Pizza Delivery: Autonomous Exploration & Delivery

> INFO0948 – Introduction to Intelligent Robotics · Academic year 2024–2025 · Liège Université, Sciences Appliquées

A ROS 2 project in which a **TurtleBot3 Burger** (equipped with a LiDAR and a frontal camera) explores an unknown environment, builds a map with SLAM, locates the "customers" (ArUco markers), and then delivers pizzas to them in an optimized order.

## Authors

Victoria Filée, Sophia Donato, Arnaud Innaurato

## Objectives

1. **Mapping & localization (SLAM)**: map the environment and locate the robot in it while identifying the customers (ArUco markers, IDs 0–10) and memorizing their position on the map.
2. **Delivery**: deliver pizzas by placing the robot facing each ArUco at ~50 cm for 3 seconds, while minimizing the total delivery time.

## Pipeline overview

1. **Explore** the environment and build the map (frontier-based exploration).
2. **Raycast search**: revisit uncovered areas to find missing ArUcos if some remain after exploration.
3. **Localize ArUcos** in the map frame (in parallel with phases 1 and 2), filtered with a Kalman filter.
4. **Deliver** by visiting the requested ArUcos in an optimal order (TSP).

## ROS 2 architecture

| Node | Role |
|------|------|
| `map_exploration` | Frontier-based exploration; sends goals to Nav2 and triggers a 360° turn at each goal |
| `robot_pose` | Continuously publishes the robot pose |
| `aruco_subscriber` | Receives ArUco detections, filters their poses, broadcasts them to tf2 |
| `raycaster` | Finds missing ArUcos by raycasting from visited positions |
| `delivery` | Solves the TSP and drives the robot through the delivery tour |

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `OccupancyGrid` | Map built by Cartographer (SLAM) |
| `/robot_pose` | `PoseStamped` | Current robot pose |
| `/aruco_markers` | `ArrayPose` | Raw ArUco poses from the external detection node (camera frame) |
| `/arucos_number` | `Int32` | Number of ArUcos found so far |
| `/arucos_location` | `ArrayPose` | Delivery poses (50 cm in front of each ArUco); activates the delivery step |
| `/pos_array` | `Float32MultiArray` | Positions where the robot performed a 360° turn; activates the raycaster |
| `/delivery_locations` | `Int32MultiArray` | IDs of the ArUcos to deliver to |
| `/cmd_vel` | `Twist` | Velocity commands (used for the 360° scans) |

Navigation relies on Nav2 (`NavigateToPose` and `ComputePathToPose` actions).

##  How to start the robot?
- Get IP : https://tplinkeap.net 
Username : admin, mdp : chiefbot
- ssh ubuntu@{IP} mdp: turtlebot
- bring up the robot: ```ros2 launch turtlebot3_bringup robot.launch.py```
- launch cartographer: ```ros2 launch turtlebot3_cartographer cartographer.launch.py```
- launch navigation2 : ```ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:='/home/laptop**/group9/projet/group9_ws/my_params.yaml'```
- launch teleop : ```ros2 run turtlebot3_teleop teleop_keyboard```


## Create package
- Create a new package in src directory :  ```ros2 pkg create --build-type ament_python <package_name>``` 
- Customize package.xml and setup.py (license, dependencies, entry points)
- Build package in root of directory : ```colcon build --packages-select my_package```
- Source installation in root of directory : ```source install/setup.bash```
- Run a node in root of directory :  ```ros2 run <package_name> <node_name>``` 


## Useful commands
- reboot instead of shuting down : ``sudo reboot``
- see percentage of battery : ``ros2 topic echo /battery_state``


