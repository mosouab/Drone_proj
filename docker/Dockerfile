# On part d'une image maintenue par l'équipe PX4 qui contient déjà ROS Noetic + Gazebo + MAVROS
FROM px4io/px4-dev-ros-noetic:latest

# Mise à jour des paquets
RUN apt-get update && apt-get upgrade -y

# Installation d'outils supplémentaires utiles
RUN apt-get install -y \
    tmux \
    vim \
    net-tools \
    iputils-ping \
    python3-pip

# Installation des dépendances Python pour le scripting
RUN pip3 install \
    mavsdk \
    dronekit \
    opencv-python

# Configuration de l'environnement ROS au démarrage du terminal
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Création du dossier de travail
WORKDIR /root/workspace

# Point d'entrée par défaut : un shell bash
CMD ["/bin/bash"]
