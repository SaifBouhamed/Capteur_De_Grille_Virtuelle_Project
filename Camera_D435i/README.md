# Intel RealSense D435i - Pilote ROS 2

Nœud ROS 2 qui capture les données RGB et de profondeur de l'Intel RealSense D435i, traite les flux de profondeur et publie les paquets RCMB combinés (image JPEG + nuage de points compressé zlib).

## 📋 Aperçu

**Nœud :** `realsense_gpu_combined` (`rgbpc.cpp`)

Ce pilote :
- Capture les flux RGB et de profondeur du RealSense D435i
- Compresse le RGB en JPEG
- Génère un nuage de points avec accélération GPU (fallback CPU)
- Empaquette les deux au format RCMB et publie sur `/camera/combined`
- Publie également le RGB séparément sur `/camera/rgb/image_compressed`

## 🔧 Paramètres

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `width` | 640 | Largeur vidéo (pixels) |
| `height` | 360 | Hauteur vidéo (pixels) |
| `fps` | 30 | Images par seconde |
| `jpeg_quality` | 50 | Qualité de compression JPEG (1-100) |
| `pc_step` | 15 | Pas d'échantillonnage du nuage de points (plus haut = moins de points) |
| `min_dist` | 0.05 | Seuil de distance minimale (mètres) |
| `max_dist` | 2.5 | Seuil de distance maximale (mètres) |
| `axis_to_robot` | false | Convertir le repère caméra en repère robot (x avant, y gauche, z haut) |
| `z_offset` | 0.0 | Décalage axe Z (mètres) |
| `rgb_topic` | `/camera/rgb/image_compressed` | Sujet de sortie RGB |
| `combined_topic` | `/camera/combined` | Sujet de sortie RCMB combiné |

## 🚀 Construire et Exécuter

### Construire
```bash
cd ~/Camera_D435i
colcon build --symlink-install
source install/setup.bash
```

### Exécuter
```bash
ros2 run mqtt_rgb_bridge realsense_gpu_combined
```

### Exécuter avec des Paramètres Personnalisés
```bash
ros2 run mqtt_rgb_bridge realsense_gpu_combined --ros-args \
  -p combined_topic:=/camera/combined \
  -p rgb_topic:=/camera/rgb/image_compressed \
  -p width:=640 -p height:=360 -p fps:=30 \
  -p jpeg_quality:=50 -p pc_step:=15 \
  -p axis_to_robot:=true
```

## 📊 Sujets de Sortie

| Sujet | Type de Message | Description |
|-------|-----------------|-------------|
| `/camera/rgb/image_compressed` | `sensor_msgs/CompressedImage` | Image RGB compressée en JPEG |
| `/camera/combined` | `std_msgs/ByteMultiArray` | Paquet RCMB (JPEG + nuage de points zlib) |

## 🎛️ Fonctionnalités

- **Accélération GPU** – Utilise OpenGL pour la génération rapide de nuage de points ; bascule vers CPU si indisponible
- **Filtrage de Profondeur** – Applique les filtres de seuil, décimation, spatial et temporel
- **Résolution Flexible** – Supporte plusieurs résolutions avec fallback automatique
- **Compression Efficace** – JPEG pour les images, zlib pour les nuages de points
- **Alignement de Cadre** – Aligne la profondeur sur le cadre couleur pour une fusion RGB-D précise

## 📖 Spécifications Matériel

Pour les spécifications détaillées, voir la [Fiche Technique Intel RealSense D400 Series](https://www.intel.com/content/www/us/en/content-details/841984/intel-realsense-d400-series-product-family-datasheet.html)

Spécifications clés :
- **Résolution :** Jusqu'à 1280×720 profondeur, 1920×1080 RGB
- **Champ de Vision :** 87° × 58° (D)
- **Portée :** 0,105m - 5m
- **Fréquence d'Images :** Jusqu'à 90 FPS
- **IMU :** 6 axes (3 axes accélération + 3 axes gyroscope)

## 🔌 Exigences d'Installation

```bash
sudo apt install -y \
  librealsense2-dev \
  librealsense2-dkms \
  ros-humble-librealsense2 \
  libopencv-dev
```

---

**Note :** Le calcul du nuage de points peut être accéléré par GPU ou par CPU selon la disponibilité d'OpenGL.
