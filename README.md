# Structure du Projet - Implémentation Real TurtleBot3

Cette branche contient l'**implémentation en conditions réelles** d'un capteur de grille virtuel pour TurtleBot3 avec Intel RealSense D435i.

```
Capteur_De_Grille_Virtuelle_Project/
├── Camera_D435i/                    # Pilote de capteur (s'exécute sur le robot)
│   ├── src/
│   │   └── mqtt_rgb_bridge/         
│   │   └── realsense_camera/         
│   └── README.md
│
└── Turtlebot_project/               # Espace de travail principal (s'exécute sur dev machine)
    ├── src/
    │   ├── combined_decompressor/
    │   ├── ground_segmentation/
    │   └── Turtlebot3_setup/
    └── README.md
```

##  Packages Principaux

Chaque package a son propre fichier README détaillé. Commencez par lire celui dont vous avez besoin :

### **1. Camera_D435i/** 
Pilote de capteur Intel RealSense D435i - capture RGB + Profondeur
- S'exécute sur : **Robot** 
-  [Lire Camera_D435i/README.md](Camera_D435i/README.md)

### **2. Turtlebot_project/** 
Espace de travail ROS 2 principal avec tous les packages de traitement
- S'exécute sur : **Machine de développement**


## Documentation

- **Guide de Configuration et Exécution :** Voir [Turtlebot_project/README.md](Turtlebot_project/README.md)
- **Détails du Capteur :** Voir [Camera_D435i/README.md](Camera_D435i/README.md)

---

**Note :** Ceci est une branche indépendante. La branche `main` contient la version de simulation Gazebo.
