# Projet Turtlebot3 : Contrôle par Gestes

Ce projet implémente un système de contrôle intelligent pour un **Turtlebot3 Burger** équipé d'une caméra de profondeur (**d435i**) dans l'environnement de simulation Gazebo. Il utilise **MediaPipe** pour la reconnaissance de gestes et un **Superviseur** pour orchestrer les comportements du robot.

## 1. Prérequis Système (Ubuntu 22.04 + ROS 2 Humble)

Avant de lancer le projet, assurez-vous que ROS 2 Humble et les outils de compilation sont installés sur votre machine.

### Installation de ROS 2 et des outils de base

```bash
# Installation de ROS2 Humble Desktop
sudo apt update && sudo apt install ros-humble-desktop -y

# Installation de colcon (compilateur) et des outils de dépendances
sudo apt install python3-colcon-common-extensions python3-rosdep ros-humble-rmw-cyclonedds-cpp -y

# Configuration de l'environnement
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

```

### Configuration des paquets Turtlebot3 et Simulation

Il est crucial d'exporter le bon modèle pour que les fichiers URDF et les transformations (TF) se chargent correctement.

```bash
# Installation des dépendances ROS pour la simulation
sudo apt install ros-humble-ros-gz ros-humble-turtlebot3-gazebo \
                 ros-humble-turtlebot3-descriptions ros-humble-turtlebot3-msgs \
                 ros-humble-cv-bridge ros-humble-tf2-ros -y

# Configuration du modèle spécifique avec caméra D435i
echo "export PROJECT_MODEL=turtlebot3_burger_d435i" >> ~/.bashrc
source ~/.bashrc

```

---

## 2. Installation des Dépendances Python (IA)

Le système de vision repose sur MediaPipe et OpenCV pour traiter le flux de la caméra.

```bash
# Installation des bibliothèques de vision
pip install opencv-python mediapipe msgpack

```

---

## 3. Installation du Projet

```bash
# Création de l'espace de travail
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copiez vos fichiers dans ce dossier, puis rendez les scripts exécutables :
chmod +x gesture_detector_node.py person_detector_node.py supervisor.py obstacle_avoider.py changer_images.py

```

---

## 4. Lancement et Utilisation

### Ordre de Lancement (Important!)

Vous devez lancer les composants dans cet ordre exact pour que tout fonctionne correctement.

---

### Étape 1️ : Démarrer le Serveur Coral

**Terminal 0 :**

```bash
cd ~/Turtlebot_gestes-main/coral
python3 coral_server.py
```

**Attendez** que vous voyiez : `[coral_server] OK. input: 513x513. Listening on 0.0.0.0:9900`

---

### Étape 2️ : Lancer le Robot Supervisor + Gazebo

**Terminal 1 :**

```bash
bash ~/Turtlebot_gestes-main/scripts/launch_robot.sh
```

**Ou manuellement :**

```bash
unset ROS_LOCALHOST_ONLY
unset ROS_DISCOVERY_SERVER
unset ROS_DISCOVERY_SERVER_PORT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/Turtlebot_gestes-main/install/setup.bash

ros2 daemon stop
ros2 daemon start

ros2 launch robot_supervisor robot_launch.py server_host:=127.0.0.1 server_port:=9900 image_topic:=/rgb_camera/image
```

**Attendez** que Gazebo se lance avec le robot visible.

---

### Étape 3️ : Lancer le Contrôleur de Gestes

**Terminal 2 :**

```bash
bash ~/Turtlebot_gestes-main/scripts/launch_gesture_controller.sh
```

**Ou manuellement :**

```bash
export ROS_DOMAIN_ID=10
source /opt/ros/humble/setup.bash
source ~/Turtlebot_gestes-main/install/setup.bash

cd ~/Turtlebot_gestes-main/Turtlebot3_gestes/turtlebot3_gazebo/worlds
python3 changer_images.py
```

**Attendez** qu'une fenêtre OpenCV s'ouvre avec votre webcam.

---

### Étape 4️ : Activer le Robot avec le Geste Victory 

1. **Faites le geste Victory** (✌️) 
2. **Appuyez sur V**
3. L'image affichée change en "Victory"
4. **Le robot passe en mode LISTENING et commence à agir**

---

### Étape 5️ : Contrôlez le Robot

Une fois que le robot est activé (image = Victory), vous pouvez lui donner des commandes :

| Geste | Touche | Image Affichée | Action |
|:-----:|:------:|:--------------:|:------:|
| ✌️ Victory | **V** | Victory | **ACTIVATION** |
| 👍 Thumb Up | **Z** | Thumb Up | Avancer |
| 👎 Thumb Down | **S** | Thumb Down | Reculer |
| ✊ Fist | **Q** | Fist | Tourner Gauche |
| ☝️ Pointing | **D** | Pointing | Tourner Droite |
| ✋ Open Hand | **E** | Open Hand | **STOP** |

---

### Exemple de Session Complète

```
Terminal 0> python3 coral_server.py
[coral_server] OK. input: 513x513. Listening on 0.0.0.0:9900

Terminal 1> bash ~/Turtlebot_gestes-main/scripts/launch_robot.sh
[ign gazebo-2] Starting Gazebo simulation...
[supervisor-11] Startup start-burst finished.

Terminal 2> bash ~/Turtlebot_gestes-main/scripts/launch_gesture_controller.sh
OpenCV webcam window opened...

User> Make Victory gesture ✌️ + Press V
Robot> Passe en mode LISTENING (image = Victory)

User> Make Thumb Up gesture 👍 + Press Z
Robot> Avance dans Gazebo

User> Make Open Hand gesture ✋ + Press E
Robot> S'arrête (image = Open Hand, mode = EXPLORING)

---
