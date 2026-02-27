# Capteur de Grille Virtuel - Implémentation Real TurtleBot3

Implémentation ROS 2 en conditions réelles d'un capteur de grille virtuel pour TurtleBot3 équipé d'une caméra Intel RealSense D435i. Cette branche capture et traite les données de nuage de points et de caméra RGB en temps réel, affichant la sortie de fusion de capteurs dans RViz.

**Note :** Ceci est la version matériel. La branche `main` contient la version de simulation Gazebo.

## Aperçu

Ce système capture en temps réel les données RGB et de profondeur de la caméra Intel RealSense D435i montée sur TurtleBot3, décompresse les flux de capteurs et les visualise dans RViz. Le nuage de points est projeté sur une grille de hauteur (heightmap) pour une cartographie d'élévation en temps réel.

## 🎥 Démonstration

<video width="800" controls>
  <source src="demo.mp4" type="video/mp4">
  Votre navigateur ne supporte pas la balise video. <a href="demo.mp4">Télécharger la vidéo</a>
</video>

### **Package 1 : `combined_decompressor`**
Décompresse les paquets RCMB et génère les images RGB et nuages de points pour RViz.
- Entrée : `/camera/combined` (données de capteur compressées du robot)
- Sortie : Image RGB décompressée et PointCloud2

### **Package 2 : `ground_segmentation`**
Projette le nuage de points 3D sur une grille et le visualise dans RViz avec codage couleur d'élévation.
- Entrée : Nuage de points décompressé
- Sortie : Visualisation de grille de hauteur (MarkerArray)

### **Package 3 : `Turtlebot3_setup`**
Fournit les fichiers de configuration RViz et le modèle du robot pour la visualisation.

---

## 💾 Branches du Dépôt

- **`main`** – Version basée sur simulation (Ignition Gazebo)
- **`real-life-robot-project`** (actuelle) – TurtleBot3 réel avec capteur RealSense D435i


---

## 📦 Dépendances Système

### Paquets Ubuntu
```bash
# Outils de construction ROS 2 principaux
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  build-essential

# SDK RealSense et wrapper ROS
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  librealsense2-dev \
  librealsense2-dkms

# Optimisation de réseau (CycloneDDS)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Visualisation
sudo apt install -y ros-humble-rviz2
```

### Dépendances Python
```bash
pip install numpy scipy scikit-learn opencv-python pyyaml
```
---

## 🚀 Installation

### Étape 1 : Cloner la Branche Matériel
Puisque c'est une branche indépendante de `main`, clochez-la directement :

```bash
cd ~
git clone -b real-life-robot-project https://github.com/SaifBouhamed/Capteur_De_Grille_Virtuelle_Project.git Turtlebot3_Grille_Virtuelle
cd Turtlebot3_Grille_Virtuelle
```

### Étape 2 : Sourcer l'Environnement ROS 2
```bash
source /opt/ros/humble/setup.bash
```

### Étape 3 : Installer les Dépendances ROS
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Étape 4 : Construire l'Espace de Travail
```bash
colcon build --symlink-install
```

Le flag `--symlink-install` crée des liens symboliques au lieu de copier les fichiers, permettant une itération de développement rapide sans reconstruire.

### Étape 5 : Sourcer l'Espace de Travail
```bash
source install/setup.bash
```

---

## ▶️ Exécution du Système

### Démarrage Rapide
```bash
./start_bot.sh
```
Cela lance le pipeline complet avec les paramètres par défaut

**Guide d'Ajustement des Paramètres :**
- `extra_pitch_deg` – Ajustez si la grille de hauteur apparaît inclinée (trop haut = penche vers le bas, trop bas = penche vers le haut)
- `z_offset` – Ajustez la hauteur de la caméra par rapport à la base du robot
- `max_hz` – Augmentez pour un traitement plus rapide (utilisation CPU plus élevée) ; diminuez pour la stabilité

### Ce qui se passe au Démarrage

1. **Configuration RMW** – Configure CycloneDDS comme middleware avec ROS_DOMAIN_ID=1
2. **Lancement RViz** – Ouvre la visualisation avec affichage de la grille et du nuage de points
3. **Récepteur RCMB** – Se connecte à `/camera/combined` et décompresse les paquets
4. **Projection de Grille** – S'abonne au nuage de points décompressé et génère la grille de hauteur
5. **Visualisation** – MarkerArray publié vers RViz en temps réel

---

## 🔍 Vérification et Débogage

### Vérifier la Santé du Système
```bash
# Lister les nœuds ROS actifs
ros2 node list
# Devrait afficher : /rviz2, /rcmb_receiver, /smart_grid_node

# Vérifier les sujets (topics)
ros2 topic list -t
# Devrait inclure :
#   /camera/combined [std_msgs/msg/ByteMultiArray]
#   /camera/color/image_raw_decomp [sensor_msgs/msg/Image]
#   /camera/depth/color/points_decomp [sensor_msgs/msg/PointCloud2]
#   /visualization_marker_array [visualization_msgs/msg/MarkerArray]

# Surveiller les taux de message
ros2 topic hz /camera/color/image_raw_decomp
ros2 topic hz /camera/depth/color/points_decomp

# Inspecter la structure des paquets compressés
ros2 topic echo /camera/combined | head -5

# Vérifier les statistiques du nuage de points
ros2 topic echo /camera/depth/color/points_decomp --limit-1 | head -30
```

### Configuration de la Visualisation RViz

1. **Ajouter → Image** – Afficher `/camera/color/image_raw_decomp` (RGB du D435i)
2. **Ajouter → PointCloud2** – Afficher `/camera/depth/color/points_decomp` (nuage de points, codé par couleur)
3. **Ajouter → Marker** – Afficher `/visualization_marker_array` (grille de hauteur avec couleurs d'élévation)




