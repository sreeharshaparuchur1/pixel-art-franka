# Start with an official ROS 2 base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3 python3-pip python3-venv python-is-python3 \ 
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Ensure python and pip are available globally
# RUN ln -s /usr/bin/python3 /usr/bin/python && \
#     ln -s /usr/bin/pip3 /usr/bin/pip

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

# Install some ROS 2 dependencies
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Install Python-based libraries using pip
RUN pip install --no-cache-dir \
    numpy \
    opencv-python \
    opencv-contrib-python \
    rospkg \
    catkin-pkg \
    transforms3d

# Install OpenCV and cv_bridge for ROS 2
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-humble-vision-opencv \
        ros-humble-cv-bridge \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Ensure ROS 2 dependencies are installed
WORKDIR /ros2_ws
COPY . /ros2_ws/src
RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
    && vcs import src < src/franka.repos --recursive --skip-existing \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

# Copy and configure entrypoint
COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

# Set the default shell and entrypoint
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws



# # Start with an official ROS 2 base image for the desired distribution
# FROM ros:humble-ros-base

# # Set environment variables
# ENV DEBIAN_FRONTEND=noninteractive \
#     LANG=C.UTF-8 \
#     LC_ALL=C.UTF-8 \
#     ROS_DISTRO=humble

# ARG USER_UID=1001
# ARG USER_GID=1001
# ARG USERNAME=user

# # Install essential packages and ROS development tools
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#         bash-completion \
#         curl \
#         gdb \
#         git \
#         nano \
#         openssh-client \
#         python3-colcon-argcomplete \
#         python3-colcon-common-extensions \
#         sudo \
#         vim \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# # Setup user configuration
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
#     && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
#     && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
    
# USER $USERNAME

# # Install some ROS 2 dependencies to create a cache layer
# RUN sudo apt-get update \
#     && sudo apt-get install -y --no-install-recommends \
#         ros-humble-ros-gz \
#         ros-humble-sdformat-urdf \
#         ros-humble-joint-state-publisher-gui \
#         ros-humble-ros2controlcli \
#         ros-humble-controller-interface \
#         ros-humble-hardware-interface-testing \
#         ros-humble-ament-cmake-clang-format \
#         ros-humble-ament-cmake-clang-tidy \
#         ros-humble-controller-manager \
#         ros-humble-ros2-control-test-assets \
#         libignition-gazebo6-dev \
#         libignition-plugin-dev \
#         ros-humble-hardware-interface \
#         ros-humble-control-msgs \
#         ros-humble-backward-ros \
#         ros-humble-generate-parameter-library \
#         ros-humble-realtime-tools \
#         ros-humble-joint-state-publisher \
#         ros-humble-joint-state-broadcaster \
#         ros-humble-moveit-ros-move-group \
#         ros-humble-moveit-kinematics \
#         ros-humble-moveit-planners-ompl \
#         ros-humble-moveit-ros-visualization \
#         ros-humble-joint-trajectory-controller \
#         ros-humble-moveit-simple-controller-manager \
#         ros-humble-rviz2 \
#         ros-humble-xacro \
#     && sudo apt-get clean \
#     && sudo rm -rf /var/lib/apt/lists/*

# WORKDIR /ros2_ws

# # Install the missing ROS 2 dependencies
# COPY . /ros2_ws/src
# RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
#     && vcs import src < src/franka.repos --recursive --skip-existing \
#     && sudo apt-get update \
#     && rosdep update \
#     && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
#     && sudo apt-get clean \
#     && sudo rm -rf /var/lib/apt/lists/* \
#     && rm -rf /home/$USERNAME/.ros \
#     && rm -rf src \
#     && mkdir -p src

# COPY ./franka_entrypoint.sh /franka_entrypoint.sh
# RUN sudo chmod +x /franka_entrypoint.sh

# # Set the default shell to bash and the workdir to the source directory
# SHELL [ "/bin/bash", "-c" ]
# ENTRYPOINT [ "/franka_entrypoint.sh" ]
# CMD [ "/bin/bash" ]
# WORKDIR /ros2_ws