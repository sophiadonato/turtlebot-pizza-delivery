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
* Remettre dans les positions dans la frame `map`
* Incorporer cette information dans le grid_mapping


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

Manhattan distance => Does not work, consider euclidian? See prove of concept  
Utiliser RayCast

## Pizza delivery
Second part of the demonstration, is to have the robot to navigate to a given list of arucos.  
When receiving this list, we must be able to compute the location in which the robot should arrive (50 cm IN FRONT of the aruco)  
How to know to which location the robot must be with the right orientation? Done in aruco_subcription
How to get the right order to visit the arucos?  TBD
How to be sure to be on the right side of the aruco ? Having the orientation should be nice, since we could compute the transform (Done in aruco_subcription).  
What is the actual orientation measured? TBC

## Kalman filter
To integrate all the data, we need a Kalman filter to incorporate new sensor reading  
I think the Kalman filter should be done on the transform, to get the most accurate position to visit the arucos and not on the aruco itself due to noLinearity of quaternion 

TBD soon : 
* Find good transform for given aruco position to visit
* Test Kalmann filter

## Pose to visit

Using gazebo, I saw that the pose where the robot should go is  
* x_aruco 
* z_aruco
* y_aruco -0.5
* rool_aruco
* pitch_aruco - 1.57
* yaw_aruco + 3.14

Does it makes sense ?  
On gazebo, the orientation seems nice but not the position... How come?  
I think, that the only position that should be modified is x_aruco with a + 0.5 (due to the angle we have a transform fixed)