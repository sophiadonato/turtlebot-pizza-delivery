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