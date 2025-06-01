FROM ros:iron-ros-base
USER 0

# Get updates
RUN apt-get update && \
apt-get upgrade -y

# Setup python and system dependencies
ADD setup /app/setup
RUN bash -c 'apt-get install -y $(cat /app/setup/system-requirements.txt)'
RUN apt full-upgrade -y
RUN pip3 install --no-cache-dir -r /app/setup/python-requirements.txt

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
