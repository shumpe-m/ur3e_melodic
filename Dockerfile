FROM nvidia/opengl:base-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive


########################
### INSTALL PACKAGES ###
########################
RUN apt-get update && apt-get install -y \
    vim \
    wget \
    unzip \
    git \
    python-tk \
    python3-pip


###################
### ROS melodic ###
###################
RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    gnupg2 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
	ros-melodic-desktop-full \
	&& echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


#####################
### INSTALL CMAKE ###
#####################
# Reference: https://apt.kitware.com/
RUN git clone https://gitlab.kitware.com/cmake/cmake.git \
	&& cd cmake \
	&& git checkout tags/v3.16.3 \
	&& ./bootstrap --parallel=8 \
	&& make -j8 \
	&& make install \
	&& cd .. \
	&& rm -rf cmake


######################
### ur_control pkg ###
######################
COPY src /root/catkin_ws/src


###########################################
### INSTALL Universal_Robots_ROS_Driver ###
###########################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-rospkg \
    python-catkin-pkg \
    python-rosinstall \
    python-rosinstall-generator \
    python-rosdep \
    python-catkin-tools \
    ros-melodic-moveit-commander \
    python

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
RUN mkdir -p /root/catkin_ws/src
RUN cd /root/catkin_ws \
    && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver \
    && git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git src/universal_robot \
    && apt-get update -qq \
    && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=melodic -y \
    && /bin/bash -c "source /opt/ros/melodic/setup.bash; cd /root/catkin_ws; catkin_make" \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/root/catkin_ws" >> ~/.bashrc \
    && echo "export ROS_WORKSPACE=/root/catkin_ws" >> ~/.bashrc \ 
    && echo "chmod -R u+x /root/catkin_ws/src/ur_control/scripts" >> ~/.bashrc

RUN echo "cd ~/catkin_ws" >> ~/.bashrc