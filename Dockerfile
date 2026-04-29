###
### THIS IS THE INTERACTIVE VERSION OF THE DOCKERFILE, NOT THE FINAL ONE.
###
#add common deps 

FROM ubuntu:24.04
RUN apt update && apt upgrade -y
RUN apt install software-properties-common -y
RUN add-apt-repository universe
#env variables
ENV ROS_DOMAIN_ID=8

#install ROS2 jazzy
RUN apt update &&  apt install curl -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
RUN dpkg -i /tmp/ros2-apt-source.deb
RUN apt update
RUN apt upgrade
RUN apt install ros-jazzy-ros-base -y

# Source ROS on container start
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
#set ROS DOMAIN ID
ENV ROS_DOMAIN_ID=8

# install aria coda + build essentials
RUN apt install git -y
RUN apt install build-essential -y
RUN apt install make g++ -y
WORKDIR /opt
RUN git clone https://github.com/Quarterpie3141/AriaCoda.git
WORKDIR AriaCoda 
RUN make install
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install python3-colcon-common-extensions -y
RUN apt install libboost-all-dev -y

#install joy and joy-teleop
RUN apt install ros-jazzy-joy-teleop -y
RUN apt install ros-jazzy-joy -y

WORKDIR /opt/ros2ws

#lidar stuff 
RUN apt install ros-jazzy-sick-scan-xd -y

# ekf stuff
RUN apt install ros-jazzy-robot-localization -y

#cyclone 
RUN apt install ros-jazzy-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"wlp4s0\"/></Interfaces></General></Domain></CycloneDDS>" 
#wlp0s20f3

#GPS
RUN apt install ros-jazzy-nmea-navsat-driver -y

#CV
RUN apt install ros-jazzy-vision-msgs -y

#SLAM
RUN apt install ros-jazzy-slam-toolbox -y

#NAV2
RUN apt install ros-jazzy-navigation2 -y
RUN apt install ros-jazzy-nav2-bringup -y

#?
RUN apt install ros-jazzy-xacro -y
RUN apt install ros-jazzy-robot-state-publisher -y
RUN apt install ros-jazzy-joint-state-publisher -y

#alex's bullshit
WORKDIR /opt/ros2ws/
RUN apt install ros-jazzy-phidgets-drivers -y
RUN apt install ros-jazzy-phidgets-spatial -y
RUN apt-get install ros-jazzy-imu-tools -y
RUN apt install ros-jazzy-imu-filter-madgwick -y

# Install Phidgets C-library
RUN apt install python3-pip -y
RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash -
RUN apt install -y libphidget22
RUN python3 -m pip install --no-cache-dir -U Phidget22 --break-system-packages

RUN apt install libusb-1.0-0-dev -y

RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash -
RUN apt install -y libphidget22
RUN     pip install -U Phidget22 --break-system-packages

RUN apt install libusb-1.0-0-dev -y

RUN apt install ros-jazzy-nav2-waypoint-follower -y

#copy src dir bring rosaria2 into the src dir
COPY ros2ws /opt/ros2ws
WORKDIR /opt/ros2ws/src
RUN git clone https://github.com/itzmehar/rosaria2.git
WORKDIR /opt/ros2ws

#why are we doing this?
RUN chmod 777 -R /opt/ros2ws

#build all packages in the workspace
WORKDIR /opt/ros2ws/
RUN source /opt/ros/jazzy/setup.bash && colcon build 
RUN echo "source /opt/ros2ws/install/setup.bash" >> ~/.bashrc 

#ENTRYPOINT [""]

#lidar continue
#RUN source /opt/ros/jazzy/setup.bash && source /opt/ros2ws/install/setup.bash && ros2 launch sick_scan_xd sick_tim_7xx.launch.py hostname:=192.168.0.1