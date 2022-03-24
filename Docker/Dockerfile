### ROS Noetic
FROM osrf/ros:noetic-desktop-full

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

### Gazebo
RUN apt-get install -y curl
RUN curl -sSL http://get.gazebosim.org | sh

### MavROS and JdeRobot-drones
##  Source installation
#   Install catkin tools 
RUN apt-get install -y python3-catkin-tools \
    python3-osrf-pycommon python3-rosdep python3-vcstool git \
    libgeographic-dev ros-noetic-geographic-msgs python3-pip zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool \
    python3-dev python-is-python3 libxml2 libxml2-dev libxslt-dev qt5-default

RUN pip3 install -U lxml future numpy==1.19 opencv-python==4.5.3.56

#   Set up catkin workspace  
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws \
    && catkin init 
RUN echo 'export ROS_WORKSPACE=~/catkin_ws' >> ~/.bashrc 

#   Get jderobot-drones repository that contains ros pkgs
RUN source ~/.bashrc && cd ~/catkin_ws && cd src && git clone https://github.com/JdeRobot/drones.git -b noetic-devel 

#   Link drone source to catkin_ws
RUN source ~/.bashrc && cd ~/catkin_ws && cd src && vcs import < "drones/.rosinstall" 

#   Update ros dependencies
RUN cd ~/catkin_ws/src 

CMD command rosdep update  
 
#   Build
RUN source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && catkin build 

#   Export environment variables
RUN source ~/.bashrc && cd ~/catkin_ws \
    && echo 'source '$PWD'/devel/setup.bash' >> ~/.bashrc \
    && echo 'export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/usr/share/gazebo-11' >> ~/.bashrc \
    && echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/repos/drones/drone_assets/models' >> ~/.bashrc 

### PX4
##  Install PX4 depend
#   Download and install Geographiclibencies
RUN sudo apt-get install wget
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh
RUN rm ./install_geographiclib_datasets.sh

#   Install common dependencies
RUN apt-get update -y
RUN apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool \
    python3-dev python-is-python3 -y
    
##  PX4 source installation
#   Get PX4 source (v1.11.3)
RUN mkdir ~/repos && cd ~/repos && git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.11.3

#   Run PX4 installation script
RUN cd ~/repos/PX4-Autopilot/Tools/setup && bash ubuntu.sh --no-nuttx --no-sim-tools

#   Install gstreamer
RUN apt install -y libgstreamer1.0-dev
RUN apt install -y gstreamer1.0-plugins-bad

#   Build PX4
RUN cd ~/repos/PX4-Autopilot && DONT_RUN=1 make px4_sitl gazebo

#   Export environment variables
RUN echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo' >> ~/.bashrc \
    && echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/repos/PX4-Autopilot/Tools/sitl_gazebo/models' >> ~/.bashrc \
    && echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/repos/PX4-Autopilot/build/px4_sitl_default/build_gazebo' >> ~/.bashrc \
    && echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/repos/PX4-Autopilot' >> ~/.bashrc \
    && echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/repos/PX4-Autopilot/Tools/sitl_gazebo' >> ~/.bashrc 
    
RUN source /opt/ros/noetic/setup.bash

WORKDIR root/repos/

CMD exec bash && make px4_sitl gazebo
