FROM ros:iron-ros-base
USER 0

# Get updates
RUN \
apt-get update && \
apt-get upgrade -y

# Setup python and system dependencies
ADD setup /app/setup
RUN cat /app/setup/system-requirements.txt | xargs apt-get install -y
RUN pip install -r /app/setup/python-requirements.txt

RUN sudo apt install -y \
    ros-iron-teleop-twist-keyboard \
    ros-iron-teleop-twist-joy \
    ros-iron-joy-linux \
    ros-iron-rqt*

# Put ros commands on path
RUN echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> ~/.bashrc && \
    echo "source \"/app/src/ros2_ws/install/local_setup.bash\"" >> ~/.bashrc

# Create folder and setup temp directory / work directory
RUN mkdir -p /app/temp && mkdir -p /app/src/ros2_ws
ENV XDG_RUNTIME_DIR=/app/temp
COPY src /app/src
WORKDIR /app/src/ros2_ws

RUN colcon build

ENTRYPOINT ["bash" "/app/src/idle_entry_point.sh"]
