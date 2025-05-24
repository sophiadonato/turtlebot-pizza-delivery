## MAP EXPLORATION 

Deux noeuds :

- RobotPose qui calcule la position du robot en faisant une transformée entre 'base_footprint' et 'map' et la publie toutes les 0.5 sec sur un topic ```/robot_pose```

- MapExplorer qui fait 2 choses principales :
    - Process l'Occupancy grid recue sur le topic ```/map``` (augmente les bords, bouche les trous, normalise les valeurs sur une échelle à 3 valeurs : 0 libre, -1 inconnu, 1c obstacle) et sauvegarde cette map processée dans un champ

    - Visite la map, cela se fait en appelant une fonction toutes les 10sec, cette fonction fait qqch si la map n'est pas finie d'être explorée et si le précédent objectif a été géré (atteint ou failed mais géré) :
        - Première étape : tourner sur soi même pour voir les aruco, fait grâce à une publication d'un Twist sur un topic ```/cmd_vel``` 
        - Deuxième étape : trouver le goal, fait grâce à la processed grid. On cherche les frontrières (ie les longues lignes de 0 collés à des -1), et on va au milieu de la frontière la plus proche du robot
        - Troisième étape : aller au goal, fait grâce à des callback en chaine voir action.py pour avoir le modèle, inspiré de cette source : 
    
    - L'exploration se finit quand il n'y a plus de frontière détectées

- Prochaines étapes : 
    - Si exploration finie et nombre d'aruco != 10, faire un raycasting et refaire une phase d'exploration jusqu'à ce que tous les arucos soient découvert -> Vic -> DANS UN AUTRE NOEUD EN SE SERVANT DE TOPIC