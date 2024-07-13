FROM ros:iron-ros-base
USER 0

# Get updates
RUN \
apt-get update && \
apt-get upgrade -y

# Setup python and system dependencies
COPY setup /app/setup
RUN cat /app/setup/system-requirements.txt | xargs apt-get install -y
RUN pip install -r /app/setup/python-requirements.txt

# Put ros commands on path
RUN echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> ~/.bashrc

# Create folder and setup temp directory / work directory
RUN mkdir -p /app/temp && mkdir -p /app/src/ros2_ws
ENV XDG_RUNTIME_DIR=/app/temp
COPY src /app/src
WORKDIR /app/src/ros2_ws

ENTRYPOINT ["bash" "/app/src/idle_entry_point.sh"]
