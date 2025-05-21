# Project Robotics 

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

## Push on gitlab
Before commit the changes, modify the user data of git on the local device
- ```git config --global user.email "you@example.com"```
- ```git config --global user.name "Your Name"```


