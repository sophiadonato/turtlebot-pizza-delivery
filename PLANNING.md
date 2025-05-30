# Planning

Objectif : visiter tous les arucos dont la position (ie la pose que dois avoir le robot pour être en face et à 50cm), est envoyée sur ```/arucos_location```.
La liste des clients à aller visiter (leurs IDs) est envoyée comme un message de type ```std_msgs.msg.Int32MultiArray``` sur le topic ```/delivery_locations```.
La position du robot est disponible sur le topic ```/robot_pose```


## TSP
Le but étant de visiter un petit nombre de arucos en commencant à la position du robot. On a donc un TSP (Travel Salesman Problem) à résoudre. On utilise python_tsp (Docs : https://pypi.org/project/python_tsp/). 

Installer le module : ```pip install python-tsp```  
Pour l'importer : ```from python_tsp.exact import solve_tsp_dynamic_programming```  
La fonction retourne une matrice de permutation donnant les poses à visiter dans l'ordre (en fonction de l'ordre mise dans la distance mtx) ```permutation, distance = solve_tsp_dynamic_programming(distance_matrix)```
TSP fonctionne pour des boucles fermées => Ici, on veut pas que le robot retourne à sa position de départ, on utilise la version ouverte de TSP => ```distance_matrix[:, 0] = 0 ``` (car colonne 0 est le robot au début).

## Planning
On a 3 noeuds : 

- PathOfDelivery : après avoir reçu la position du robot, les positions des arucos et les arucos à placer, on calcule chacune des distances disponibles sur le graphes en se servant du deuxième noeud. On obtient une distance mtx qu'on passe au TSP solver, qui nous donne les permutations. On permutte et on publie les positions triées sur ```/ordered_aruco_poses```

- ComputePathBetweenLocation : calcule un ensemble de pose à parcourir entre deux points donnés comme field du noeud (donnés par PathOfDelivery). On utilise un Action Client avec des messages de type 'ComputePathToPose' et le même "modèle" que dans map_explorer.
Une fois la liste de pose calculée, le noeud PathOfDelivery calcule une distance estimée entre chaque pose de la liste, pour avoir une distance approximée entre deux poses.

- Delivery : à l'aide d'un action client, parcourt toutes les poses publiées sur ```/ordered_aruco_poses```

Pour les spin : 
PathOfDelivery() : spin dans le main
ComputePathBetweenLocation() : spin_once jusqu'à ce que le path entre les 2 poses voulues soit fini.
Delivery() : spin après avoir envoyé les poses triées


