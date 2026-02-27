import os
import time
from pathlib import Path

# Base directory = folder where this script is located
BASE_DIR = Path(__file__).resolve().parent

# Images are in the same folder as the script
IMAGE_PATH = str(BASE_DIR)
WORLD_NAME = "default"
MODEL_NAME = "mon_panneau_image"
CMD_PREFIX = "ign"

# x=1 (devant), y=-0.5 (droite), z=0.85 (Hauteur ajustée), rotations inchangées
POSE_STRING = "1 -0.5 0.85 0 0 3.14159"

sdf_template = """
<sdf version='1.6'>
    <model name='{name}'>
        <static>true</static>
        <pose>{pose}</pose>
        <link name='link'>
          <visual name='visual'>
            <geometry>
              <box>
                <size>0.01 0.4 0.4</size>
              </box>
            </geometry>
            <material>
              <ambient>1 1 1 1</ambient>
              <diffuse>1 1 1 1</diffuse>
              <specular>0 0 0 1</specular>
              <pbr>
                <metal>
                  <albedo_map>{texture_path}</albedo_map>
                  <roughness>0.6</roughness>
                  <metalness>0.0</metalness>
                </metal>
              </pbr>
            </material>
          </visual>
          <collision name='collision'>
            <geometry>
              <box>
                <size>0.01 0.4 0.4</size>
              </box>
            </geometry>
          </collision>
        </link>
    </model>
</sdf>
"""

def update_panel(image_filename):
    full_path = os.path.join(IMAGE_PATH, image_filename)

    if not os.path.exists(full_path):
        print(f"ERREUR : Image introuvable : {image_filename}")
        return

    print(f"Update vers : {image_filename}")

    # 1. Suppression de l'ancien panneau
    delete_cmd = (
        f"{CMD_PREFIX} service -s /world/{WORLD_NAME}/remove "
        f"--reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean "
        f"--req 'name: \"{MODEL_NAME}\", type: MODEL' --timeout 2000 > /dev/null 2>&1"
    )
    os.system(delete_cmd)

    time.sleep(0.1)

    # 2. Création du nouveau panneau
    new_sdf = sdf_template.format(name=MODEL_NAME, pose=POSE_STRING, texture_path=full_path)
    new_sdf_flat = new_sdf.replace('\n', ' ')

    spawn_cmd = (
        f'{CMD_PREFIX} service -s /world/{WORLD_NAME}/create '
        f'--reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean '
        f'--req "sdf: \\"{new_sdf_flat}\\"" --timeout 2000 > /dev/null'
    )

    os.system(spawn_cmd)
    print("Image changée !")

def remove_panel():
    print("Panneau supprimé (aucune image)")

    delete_cmd = (
        f"{CMD_PREFIX} service -s /world/{WORLD_NAME}/remove "
        f"--reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean "
        f"--req 'name: \"{MODEL_NAME}\", type: MODEL' --timeout 2000 > /dev/null 2>&1"
    )
    os.system(delete_cmd)

def main():
    print("=== Contrôle Panneau (Taille & Hauteur ajustées) ===")

    while True:
        choice = input("Commande ( stop: e, forward: z, backward: s, left: q, right: d, listen: v, plain: p, exit: r ) > ").strip().lower()
        if choice == 'z': update_panel("thumb_up.png")
        elif choice == 's': update_panel("thumb_down.png")
        elif choice == 'e': update_panel("open_palm.png")
        elif choice == 'q': update_panel("closed_fist.png")
        elif choice == 'd': update_panel("pointing_up.png")
        elif choice == 'v': update_panel("victory.png")
        elif choice == 'p': remove_panel()
        elif choice == 'r': break
        else: print("Touche inconnue")

if __name__ == "__main__":
    main()
