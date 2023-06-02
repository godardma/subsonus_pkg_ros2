# subsonus_pkg 

## Auteur :

:student: Maël GODARD <mael.godard@ensta-bretagne.org> (FISE 2023)

## Git Structure :

* :file_folder: [/src](src) : **dossier contenant les sources**
* :file_folder: [/meshes](meshes) : **dossier contenant les modèles 3D utilisé pour visualiser les pose dans rviz2**
* :file_folder: [/config](config) : **dossier contenant les fichiers de configuration des USBL et de RVIZ2**
* :file_folder: [/launch](launch) : **dossier contenant les launcher, non fonctionnel à l'heure actuelle**
* :spiral_notepad: [/README.md](README.md)

## Technologies :

* Ubuntu 20.04 LTS ou 22.04
* C++


## Dependance :

<ins>Installer les bibliothèques suivantes :</ins>
```bash
sudo apt-get install gpp
```

<ins>Installer ROS2 :</ins>
[ROS2 foxy pour ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
[ROS2 humble pour ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## Setup :
* Setup les USBL

    Pour fonctionner on utilisera deux USBL subsonus, un en config surface (master) à l'adresse 192.168.2.100 l'autre en config subsea (slave) à l'adresse 192.168.2.200. Les fichiers de configuration sont disponibles dans le dossier [config](config)

    On ajoutera un data stream sur le port TCP 19000 du subsonus de surface contenant :
    - System State Packet
    - Remote Track Packet
    - Remote System State Packet

* Cloner le répertoire subsonus_pkg dans un workspace ros2 tel que l'architecture soit : workspace/src/subsonus_pkg

````bash
mkdir -p workspace/src
cd workspace/src/
git clone https://github.com/godardma/subsonus_pkg.git 
cd ../..
````

* Build le projet et execution:  

Dans un premier terminal
````bash
colcon build --packages-select subsonus_pkg
. install/setup.bash
ros2 run subsonus_pkg talker
````
