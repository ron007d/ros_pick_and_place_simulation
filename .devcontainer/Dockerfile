FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Install Extra codes

RUN sudo apt install -y python3-pip 
RUN sudo apt install -y python3-catkin-tools ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-joint-trajectory-controller
RUN sudo apt install -y \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-eigen-conversions \
    ros-$ROS_DISTRO-object-recognition-msgs \
    ros-$ROS_DISTRO-roslint
RUN pip3 install opencv-python pandas
RUN pip3 install -U scikit-learn
# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
