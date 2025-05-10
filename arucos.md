# Arucos

Un noeud a été créé pour pouvoir lire les arucos  
Pour le lancer, ```ros2 launch ros2_aruco aruco_recognition.launch.py```  
Ce noeud publie sur les topics `/aruco_markers``` et ```/aruco_poses`

## Published Topics:
* `/aruco_poses` (geometry_msgs.msg.PoseArray)  
       Pose of all detected markers (suitable for rviz visualization)

* `/aruco_markers` (ros2_aruco_interfaces.msg.ArucoMarkers)  
       Provides an array of all poses along with the corresponding
       marker ids 

NB : Il existe des paramètres sur lesquelles il est possible de jouer (_aruco\_parameters.yaml_)
Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `image_topic` - image topic to subscribe to (default `/camera/image_raw`)
* `camera_info_topic` - Camera info topic to subscribe to (default `/camera/camera_info`)
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)


## Messages Types
* geometry_msgs.msg.PoseArray  
Header header  
Pose[] poses  

* ros2_aruco_interfaces.msg.ArucoMarkers  
std_msgs/Header header  
int64[] marker_ids  
geometry_msgs/Pose[] poses  

## Way to consider the information
* Le nombre de arucos est fixe, il est nécessaire de garder en mémoire la position de ces arucos de façon statique, ces positions sont lisibles dans `\aruco_poses` et sont dans la frame `camera_frame` (TBC)  
* Remettre dans les positions dans la frame `map`(?)
* Incorporer cette information dans le grid_mapping

## TODO asap
* Send array of frames through a topic to be listened by the mapping

## Exploration of the whole map for aruco detection
Il est nécessaire de retenir les murs qui ont été vus par la caméra afin de savoir où chercher des arucos markers  
Comment faire ?
* Considérer que la caméra a une certaine portée en distance
* L'orientation de l'avant du robot (=orientation de la caméra) est connue
* Interaction avec la map (Il est necessaire de considérer les murs qui coupent le champ de vision)

Première approximation: considérer que la zone de vision est un rectangle, spécification de la position du robot et de son orientation ainsi que de la profondeur de vision.  
Considération de la map faite avec le lidar pour remarquer les intersections entre le champ de vision et les murs de la map (Est ce vraiment nécessaire?)  

Il est nécessaire d'avoir une fonction de mapping qui renvoie (i,j) pour une paire (x,y) donnée  
Ce raisonnement devrait être fait dans les fonctions de mapping

Comment être sur que les murs vu de cotés ne sont pas comptés?

Manhattan distance => Does not work, consider euclidian?