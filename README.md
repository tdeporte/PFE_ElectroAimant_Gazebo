# Docking / Undocking - PFE

## Installation 

### ROS Noetic / Gazebo 11

#### ROS Noetic

Toutes les étapes d'installation de ROS se trouve au lien suivant : http://wiki.ros.org/noetic/Installation/Ubuntu

#### Gazebo 11

Installation:

```
curl -sSL http://get.gazebosim.org | sh
```

#### Test

Pour tester si ROS et Gazebo sont bien installés, lancer les commandes :

```
roscore &
rosrun gazebo_ros gazebo
```

#### Bashrc

Ajouter à la fin du fichier ~/.bashrc les lignes suivantes:

```
source /opt/ros/noetic/setup.bash
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/usr/share/gazebo-11
```

Puis entrer dans un terminal: **source ~/.bashrc**

### PX4

#### Dépendances

Téléchargement et installation de Geographiclib

```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm ./install_geographiclib_datasets.sh  # optional
```
Installation des dépendances

```
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool \
    python3-pip python3-dev python-is-python3 -y
sudo apt install libgstreamer1.0-dev
sudo apt install gstreamer1.0-plugins-bad
```

#### PX4 installation

Clone du git de PX4

```
mkdir ~/PX4 && cd PX4 #Le nom peut être différent
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.11.3
```

Script d'installation

```
cd ~/PX4/PX4-Autopilot/Tools/setup/
bash ubuntu.sh --no-nuttx --no-sim-tools
```

Build PX4

```
cd ~/PX4/PX4-Autopilot
no_sim=1 make px4_sitl_default gazebo
```

#### Bashrc

Ajoute à la suite de l'export précédent

```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/PX4/PX4-Autopilot/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/PX4/PX4-Autopilot/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/PX4/PX4-Autopilot/build/px4_sitl_default/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4/PX4-Autopilot/Tools/sitl_gazebo
```

Puis entrer dans un terminal: **source ~/.bashrc**

<span style="color:red">Le début des chemins : ~/PX4/ doit correspondre au nom de dossier choisis dans la partie de clonage du git</span>

#### Test

Pour tester si PX4 a bien était installé

```
roslaunch px4 mavros_posix_sitl.launch
```

## QGroundControl

Avant d'installer QGroundControl

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
```

Ensuite il faut se déconnecter et se reconnecter pour rendre effectif les permissions

Enfin télécharger via le lien suivant : https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

## Configuration

## Informations

Avant de lancer le script de tests il est préférable de lancer **QGroundControl** car pour certaines raisons il faut simuler un contrôleur pour faire passer tous les tests.