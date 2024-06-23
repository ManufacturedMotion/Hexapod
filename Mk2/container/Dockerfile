FROM ros:iron-ros-base
USER 0

RUN mkdir -p /app/temp
ENV XDG_RUNTIME_DIR=/app/temp

RUN apt-get update && apt-get upgrade -y && \
apt-get install -y \
python3 \
python3-pip \
git \
cmake \
dkms \
libevdev-dev \
libudev-dev \
libsdl2-dev \
libsdl2-image-dev \
libsdl2-mixer-dev \
libsdl2-ttf-dev

COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

COPY src /app/src
WORKDIR /app/src
ENTRYPOINT python3 /app/src/controller_test.py
