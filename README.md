# PER2025-034 - Segmentation et Transformation de Nuages de Points 3D pour la Génération d’un Capteur Grille virtuel

## Guide d'Exécution et Tests
### Prérequis et Installation
```bash

sudo apt update
sudo apt install ros-humble-ros-gz \
                 ros-humble-turtlebot3-msgs \
                 ros-humble-turtlebot3-description \
                 ros-humble-vision-msgs \
                 ros-humble-cv-bridge
pip3 install "numpy<2"
pip3 install opencv-python open3d struct
pip3 install mediapipe
sudo apt install ros-humble-rmw-cyclonedds-cpp
git clone https://github.com/SaifBouhamed/Capteur_De_Grille_Virtuelle_Project.git
cd ~/Capteur_De_Grille_Virtuelle_Project/


```
### Configuration de l'Environnement et des Variables Réseau
```bash

cd ~/Capteur_De_Grille_Virtuelle_Project/
export GZ_SIM_RESOURCE_PATH=~/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models:$GZ_SIM_RESOURCE_PATH
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

```

### Préparation du script
```bash

chmod +x start_bot.sh

```
### Lancement de la Simulation et du Pipeline
#### Terminal 1 : Initialisation de l'environnement et de la perception

```bash

./start_bot.sh

```
#### Terminal 2 : Contrôle manuel (Téléopération)

```bash

cd ~/Capteur_De_Grille_Virtuelle_Project/
export GZ_SIM_RESOURCE_PATH=~/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models:$GZ_SIM_RESOURCE_PATH
export ROS_DOMAIN_ID=10
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard

```

## Identification du Projet
### Titre du Projet : PER2025-034 - Segmentation et Transformation de Nuages de Points 3D pour la Génération d’un Capteur Grille virtuel.

### Étudiants et Parcours :
Saif Bouhamed – Etudiant Ingénieur en Systèmes Embarqués et Mobile (ESPRIT) & Master 2 IoT-CPS (Université Côte d'Azur).

Kamel Chedly Ben Haj Salah – Etudiant Ingénieur en Systèmes Embarqués et Mobile (ESPRIT) et  Master 2 IoT-CPS (Université Côte d'Azur).

Ons Bahri – Etudiante Ingénieur en Systèmes Embarqués et Mobile (ESPRIT) et Master 2 IoT-CPS (Université Côte d'Azur).

Encadrant : Mr.Gérald Rocher.

Date : Année universitaire 2025-2026.

## Présentation du sujet
### Le Problème (Le Reality Gap)
L'entraînement de robots quadrupèdes par Apprentissage par Renforcement (RL) repose sur des données de "proprioception" et de "perception" parfaites en simulation (NVIDIA Isaac Sim). En conditions réelles, le bruit des capteurs et les oscillations du châssis rendent ces données inexploitables. Le défi est de transformer un nuage de points brut (Point Cloud) en une représentation géométrique stable : la Grille d'Élévation (Heightmap).

### Le Contexte et les Utilisateurs
Ce projet s'inscrit dans le domaine de la robotique autonome et de la vision par ordinateur. Les utilisateurs ciblés sont les chercheurs en IA et les ingénieurs en robotique qui ont besoin de données fiables et standardisées pour interfacer leurs modèles d'IA de navigation avec du matériel physique.

### Périmètre et Scope
L'objectif est de créer une couche d'abstraction logicielle (un "capteur virtuel"). Au lieu de fournir un nuage de points 3D brut et chaotique à l'IA, le système doit générer une grille de hauteur (Heightmap) 2D/3D stable, filtrée et physiquement précise, fonctionnant en temps réel sur une architecture embarquée (Raspberry Pi).

## Présentation des Solutions
### L'Espace des solutions possibles
Algorithmes PCL (RANSAC/Filtrage Statistique) : Très précis mais CPU-intensifs. Rejeté pour l'embarqué car incapable de maintenir un flux de 30 FPS sur Raspberry Pi 4.

OctoMaps / Voxel Grids : Excellents pour la cartographie 3D, mais trop lourds pour une boucle de contrôle de locomotion temps réel (latence > 100ms).

### La Solution Retenue :
Pour répondre aux contraintes de temps réel sur architecture embarquée, nous avons conçu une architecture optimisée. Ce système transforme un flux de données brutes et instables en une représentation géométrique structurée et exploitable. Il repose sur trois piliers technologiques :
#### Matériel & Capteurs : 
La solution s'appuie sur la caméra Intel RealSense D435i. Contrairement à une caméra standard, elle combine deux sources de données essentielles :

Vision stéréoscopique active : Elle génère un nuage de points (Point Cloud) permettant de voir le monde en relief, même dans des conditions de faible luminosité.

Centrale inertielle (IMU) intégrée : Elle agit comme une "oreille interne", capturant en permanence l'inclinaison, le roulis et les accélérations du robot.

#### Stabilisation Géométrique via TF2 :
Lorsqu'un robot quadrupède se déplace, son châssis subit des secousses et des inclinaisons permanentes (tangage et roulis). Sans correction, la caméra percevrait le sol comme étant incliné, créant de faux obstacles ou masquant de réels dangers.

Nous utilisons le buffer de transformations de ROS 2 (TF2) pour stabiliser la vision :

Le pipeline interroge l'IMU pour connaître l'inclinaison exacte du robot.

Il applique instantanément une rotation mathématique inverse à chaque point 3D reçu.

Résultat : Le nuage de points est "redressé" par rapport à la gravité. Le sol reste horizontal dans le système de coordonnées du robot, peu importe les mouvements du châssis.

#### Discrétisation par Projection Vectorisée (NumPy) :
Traiter des millions de points individuels est impossible pour un processeur comme celui du Raspberry Pi. Pour atteindre une performance industrielle, nous avons développé une méthode de projection discrète :
- Grille Virtuelle : L'espace devant le robot est découpé en une grille de $20 \times 20$ cellules (comme un plateau de jeu).
- Traitement Vectorisé : Au lieu d'utiliser des boucles for (très lentes en Python), nous utilisons la librairie NumPy. Elle permet de calculer la hauteur maximale de chaque cellule en traitant des blocs entiers de données d'un seul coup.
- Efficacité : Le système réduit la complexité de la scène 3D à une simple Grille d'Élévation (Heightmap). La latence totale est inférieure à 30 ms, ce qui garantit que le robot réagit en temps réel à la topologie du terrain.

## Positionnement de la solution par rapport à l'existant
Le positionnement de notre projet se situe à l'intersection de la vision par ordinateur haute fréquence et de la locomotion robotique agile. Contrairement aux approches conventionnelles, notre pipeline ne se contente pas de détecter des obstacles ; il génère une couche d'abstraction sensorielle optimisée pour les contrôleurs de marche.

### Dépasser les limites de la navigation 2D
La plupart des robots autonomes utilisent des cartes en 2D (Costmaps). Ces cartes fonctionnent de manière binaire : une zone est soit "libre", soit "occupée".

- Le problème : Un robot capable de franchir des obstacles (comme notre quadrupède) verrait un trottoir ou une petite marche comme un mur infranchissable sur une carte 2D.

- Notre approche : Notre système utilise la 2.5D. Il ne se contente pas de détecter un obstacle, il mesure sa hauteur exacte. Cette information permet au robot de faire la différence entre un obstacle à contourner et un relief qu'il peut franchir en levant simplement la patte.

### Optimisation par rapport à la cartographie 3D complète
Il existe des méthodes pour filmer et enregistrer tout l'environnement en 3D (comme les OctoMaps). Bien que très précises, ces méthodes demandent une puissance de calcul trop importante pour les processeurs embarqués.

- Le risque : Sur un Raspberry Pi, traiter toute la scène en 3D créerait un retard (latence). Un robot qui se déplace ne peut pas se permettre de recevoir l'information du terrain avec un décalage, sous peine de tomber.

- Notre approche : Nous avons choisi de limiter la zone de calcul à une "fenêtre de perception" juste devant le robot. En simplifiant les données 3D brutes en une grille légère, nous garantissons un traitement instantané, indispensable pour la réactivité du robot.

### Le rôle de "pont" entre le virtuel et le réel (Sim-to-Real)
C'est le point central de notre travail. Les algorithmes de marche sont d'abord entraînés dans des simulateurs (comme NVIDIA Isaac Sim). Dans ces mondes virtuels, le sol est parfaitement défini sous forme de grille.

- Le Reality Gap : Dans le monde réel, les données d'une caméra sont "bruitées" (imprécises) et instables à cause des vibrations. Ce décalage empêche souvent l'IA de fonctionner correctement une fois installée sur le robot.

- Notre approche : Notre pipeline nettoie et transforme les données réelles pour les faire ressembler exactement à celles du simulateur. En offrant au robot une grille de hauteur stable et familière, nous permettons aux algorithmes entraînés virtuellement de fonctionner sans erreur dans la réalité.

## Travail Réalisé : Produit et Processus 
### Le Processus : Profilage et Optimisation
Le développement a suivi une approche de "Hardware-in-the-loop", consistant à tester chaque brique logicielle directement sur le matériel cible pour en identifier les limites réelles.

- Analyse de performance et choix du matériel : Initialement testé sur Raspberry Pi 4, le pipeline a révélé un goulot d'étranglement critique lors de la "désérialisation" des données. Le passage du message binaire sensor_msgs/PointCloud2 vers un format exploitable (NumPy) consommait trop de ressources CPU, faisant chuter la fréquence à moins de 5 FPS. Le passage au Raspberry Pi 5 a été décisif : sa puissance de calcul accrue a permis de traiter des flux haute résolution (848x480) tout en maintenant une fluidité de 30 FPS.

- Optimisation du trafic réseau (CycloneDDS) : Les nuages de points sont des messages très lourds (plusieurs Mo par seconde). Sur un réseau standard, ces messages arrivent fragmentés, ce qui causait des pertes de données et des saccades sous RViz. Nous avons configuré CycloneDDS pour optimiser la gestion des paquets UDP. En ajustant les paramètres de transmission, nous avons sécurisé le flux de données entre le robot et la station de contrôle, garantissant une visualisation fluide sans latence réseau.

### Le Produit : Livrables Techniques
Le résultat final est une suite logicielle modulaire intégrée à l'écosystème ROS 2.

- Nœud ROS 2 grid_projection_node : C'est le cœur du projet. Ce nœud en Python agit comme un moteur de transformation. Il reçoit le nuage de points 3D brut, applique les corrections d'inclinaison (via les TFs) et projette le résultat sur une grille 2.5D. Il a été optimisé pour n'utiliser que les points situés dans la trajectoire immédiate du robot, économisant ainsi de précieux cycles CPU.

- Système de Visualisation Custom : Pour valider nos résultats, nous avons créé un outil de visualisation sous RViz utilisant des MarkerArray. Au lieu d'un nuage de points abstrait, l'opérateur voit une grille de sphères dynamiques. La hauteur de chaque sphère s'ajuste en temps réel au relief, et sa couleur change selon l'altitude (ex: rouge pour les zones hautes, vert pour le sol). C'est cet outil qui permet de vérifier visuellement la précision de la "Heightmap".

- Bridge de Simulation (Ignition Gazebo) : Nous avons développé un pont de communication complet avec le simulateur Ignition Gazebo. Ce bridge permet de tester l'algorithme dans des mondes virtuels complexes (escaliers, pentes, décombres) avant tout déploiement sur le robot physique. Cela garantit que le code est robuste et que les transformations géométriques sont exactes dans un environnement contrôlé.

## Conclusions et Perspectives

### Bilan Technique et Prise de Recul
Le projet a permis de valider avec succès le pipeline de communication sous ROS 2. Les tests en conditions réelles et en simulation démontrent que l'abstraction géométrique est robuste : la grille de perception générée sous RViz suit fidèlement la topologie du terrain. La détection précise de structures complexes, comme des escaliers, prouve que la stabilisation par IMU compense efficacement les mouvements du robot.

Cependant, l'analyse des performances montre que le traitement purement géométrique (CPU) atteint ses limites. Face à la densité croissante des données issues des capteurs modernes, le processeur sature lorsqu'il doit filtrer des nuages de points trop massifs. Cette observation souligne la nécessité de passer d'un traitement "brut" à un traitement "intelligent" de la donnée.

### Perspectives et Travaux Futurs

Intégration de l'IA de Segmentation Sémantique : L'objectif est d'intégrer un modèle d'IA (actuellement en cours d'entraînement) directement dans le nœud ROS 2. Plutôt que de projeter tous les points géométriques, l'IA permettra de "nettoyer" la scène en amont. Elle pourra identifier et éliminer le bruit non-structurel, comme les jambes des passants ou des objets en mouvement (objets flottants), avant même la génération de la grille. Cela allégera la charge de calcul et garantira que la grille ne contient que des informations fixes et franchissables.

Entraînement sous NVIDIA Isaac Sim : La grille stabilisée que nous avons développée constitue une donnée d'entrée parfaite pour l'Apprentissage par Renforcement (Reinforcement Learning). La prochaine étape consiste à utiliser cette "Heightmap" comme interface standard pour entraîner un modèle de locomotion complexe. En apprenant au robot à réagir à cette grille en simulation, nous faciliterons son déploiement réel, car il retrouvera exactement le même format de données sur son matériel embarqué.

## Références Bibliographiques

### ROS 2 Humble Documentation : Architecture DDS et gestion des Transforms (TF2).

### Intel RealSense SDK 2.0 : API de traitement des trames de profondeur et de l'IMU.

### NumPy Documentation : Optimisation des calculs matriciels pour le traitement de données massives.
