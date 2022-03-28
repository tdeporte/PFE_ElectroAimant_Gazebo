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

Après avoir finit l'installation de ROS , Gazebo , PX4 et QGroundControl , il faut ajouter notre monde , modèles... dans PX4.
Pour se faire il suffit de lancer le script **init_project** qui se trouve dans le dossier Configuration/.

Il faut donner comme argument au script le chemin vers le dossier contenant PX4-Autopilot , ici ce serait ~/PX4/PX4-Autopilot/

```
python3 init_project ~/PX4/PX4-Autopilot/
```

Une fois le script lancé tous les fichiers de notre projet ont été dispersé dans le proejt PX4 et la simulation peut être lancé.

## Utilisation

Une fois l'installation et la configuration terminées, la simulation ainsi que le script de la mission et celui des tests peuvent être lancé.

Lire les parties **informations** et **Problèmes** avant de lancer la simulation ou les scripts.

Pour lancer la simulation de notre monde:

```
roslaunch px4 custom.launch
```

Ensuite pour lancer le script de la mission présent dans Scripts/ :

```
python3 Drone.py #Les variables tolérances, step et le retour camera seront mise à la valeur par défaut. C'est à dire 30 0.2 et 0 (éteint)

python3 Drone.py -t 30 -s 0.2 -c 1 
```

Des touches sont assignés pour controler le drone et le flux vidéo

```
d : Permet d'ordonner au drone d'aller se fixer sur la plaque
u : Permet d'ordonner au drone de se détacher de la plaque et de se poser
w : Le retour vidéo n'est pas lancé par défaut mais si vous le lancer, pour le fermer appuyer sur la touche "w"
```

Enfin pour lancer le script de test présent dans Scripts/ :

```
python3 DroneTests.py #La position cible du drone est celle par défaut

python3 DroneTest.py 2 2 1 #La position x , y , z est initialisée avec ces coordonnées.
```


## Informations

- Les paramètres tolerance et step du script de la mission impactent la fiabilité de placement du drone au centre du QR Code. Les valeurs recommandées sont 30 0.2 ou 40 0.2.
- Avant de lancer le script de tests il est préférable de lancer **QGroundControl** car pour certaines raisons il faut simuler un contrôleur pour faire passer tous les tests.
- Notre première simulation faite avec Unity peut être récupérer au lien suivant: https://github.com/jessynlbe/PFE_M2

## Problèmes possibles

- Si après avoir lancé le monde le drone ne réagit pas au script ou aux commandes, il faut déplacer le contenu du dossier Library qui se trouve dans Configuration/ et le placer dans PX4-Autopilot/Tools/sitl_gazebo/. Ensuite depuis sitl_gazebo/ lancer les commandes **cmake . et make** . Enfin relancer le script init_project pour être sûr d'avoir les bons fichiers ou remplacer le iris.sdf de Tools/sitl_gazebo/models/iris/ par celui de Configuration/models/

- Si le retour caméra n'affiche pas le QR Code sur la plaque ou seulement à une certaine distance cela peut provenir d'un problème d'installation d'installation. Recommencer l'installation résout le problème

- Si le retour caméra est activé il est possible que cela impacte les performances du drone à trouver le centre du qr code. 

## Version "conteneurisée" du projet

Une image contenant le projet dans sa totalité est disponible à l'adresse : 

[https://hub.docker.com/repository/docker/tdeporte/ros_sitl_gazebo](https://hub.docker.com/repository/docker/tdeporte/ros_sitl_gazebo) .

Cette image est publique et son Dockerfile associé est disponible dans le dossier **Docker/** du projet.

**Les étapes suivantes sont nécéssaires avant de pouvoir l'utiliser.**

### Installation de Docker 

Nous vous recommandons de suivre l'installation correspondant à votre envirronement de travail, disponible l'adresse : 

[https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/) .

### Récupération de l'image 

L'image est publique et récupérable grâce à la commande suivante.

```
sudo docker pull tdeporte/ros_sitl_gazebo:latest
```

### Prodiguer les permissions nécéssaires
L'outil **xauth** est nécéssaire pour cette étape.

```
sudo apt-get install -y xauth
```

Il est nécéssaire de prodiguer au conteneur l'autorisation de communiquer avec la machine hôte.

```
xhost +local:*
```

Nous recommandons cependant d'annuler l'autorisation après l'arrêt du conteneur.

```
xhost -local:*
```

### Lancement du conteneur
Nous préférons lancer le conteneur en tâche de fond dans un premier temps.

```
docker run -dit --net=host -e DISPLAY -v /tmp/.X11-unix tdeporte/ros_sitl_gazebo
```

L'identifiant du conteneur nommé plus tard CONTAINER_ID apparait en réponse.

### Lancer une commande dans le conteneur
Afin d'observer le projet nous recommandons de lancer la commande suivante dans deux terminaux.

```
docker exec -it CONTAINER_ID bash
```

Il est alors possible de naviguer comme si le projet était en local.

### Démarrer le logiciel
Dans un terminal, écrire 

```
source ~/.bashrc 
roslaunch px4 custom.launch
```

La fenêtre de Gazebo affichant la simulation s'ouvre en local.

### Démarrer le script de controle du drone
Dans un autre terminal, écrire

```
cd PFE_ElectroAimant_Gazebo/Scripts
python3 Drone.py
```

Le retour caméra du drone s'ouvre en local et vous pouvez alors controler le drone.

Le script de test nécéssite QGroundControl pour opérer efficacement, dans sa version actuelle le conteneur ne permet pas d'ouvrir ce logiciel.
Nous recommandons donc d'utiliser la version classique sur logiciel si l'on veut lancer le script de tests.

