FROM ros:noetic-robot

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install base packages
RUN apt-get update && apt-get install -y \
    git curl wget build-essential cmake \
    python3-pip python3-catkin-tools \
    lsb-release gnupg2 sudo nano \
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt + CHOMP + other tools
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-moveit-plugins \
    ros-noetic-moveit-chomp-optimizer-adapter \
    ros-noetic-industrial-core \
    ros-noetic-controller-manager \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-xacro \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ws/src
WORKDIR /ws/src

# Clone required packages
RUN git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
RUN git clone https://github.com/ros-industrial-consortium/stomp.git
RUN git clone https://github.com/tesseract-robotics/tesseract.git
RUN git clone https://github.com/tesseract-robotics/tesseract_ros.git

# Fix permissions if needed
RUN chmod -R a+rwX /ws/src

# Install dependencies
WORKDIR /ws
RUN apt-get update && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Setup environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /ws/devel/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]
