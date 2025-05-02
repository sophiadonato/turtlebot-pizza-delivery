# Project Robotics 

## Getting started
* Créer un node dans lequel on crée une map que l'on rempli avec obstacles et arrucos. Cette map est notre propre mapping (idée faire une map comme IA).
    - Comment lire les obstacles renvoyées par le lidar?
        Se renseigner sur occupacy grid du topic map
    - Comment considérer les positions?
    - Quelle discrétisation de grille?

* Modéliser une trajectoire entre les aruccos efficace (idée Pacman)
    - Comment tenir compte des réalités physiques ? 
    - Qui des obstacles qui bougent?


## Occupancy Grid
nav_msgs/OccupancyGrid
Pour écho, ros2 topic echo map
On a width, height et une liste de points à interpréter comme les positions de la map et la certitude d'occupancy 
L'origine de la map est donnée donc peut update depuis une autre position
Donc il faut créer un node qui lit ce topic et crée notre propre carte

##  How to start the robot?
ssh ubuntu@{IP} mdp: info0948-2
bring up the robot: ``ros2 launch turtlebot3_bringup robot.launch.py```
launch cartographer: ```ros2 launch turtlebot3_cartographer cartographer.launch.py```
launch navigation2 : ```ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml``


## Create package
- Create a new package in src directory :  ```ros2 pkg create --build-type ament_python <package_name>`` 
- Customize package.xml and setup.py (license, dependencies, entry points)
- Build package in root of directory : ```colcon build --packages-select my_package``
- Source installation in root of directory : ```source install/setup.bash``
- Run a node in root of directory :  ```ros2 run <package_name> <node_name>`` 


