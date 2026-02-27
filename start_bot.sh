#!/bin/bash

# 1. Charger ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Aller dans le dossier et charger ton workspace
cd ~/Capteur_De_Grille_Virtuelle_Project/
source install/setup.bash

# 3. Lancer tout le système avec le Launch File qu'on vient de créer
ros2 launch ground_segmentation system_complet.launch.py
