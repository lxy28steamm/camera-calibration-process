FROM osrf/ros:noetic-desktop-full

ARG CATKIN_JOBS=2

ENV http_proxy= \
    https_proxy= \
    HTTP_PROXY= \
    HTTPS_PROXY= \
    no_proxy= \
    NO_PROXY=

RUN set -eux; \
    printf '%s\n' \
      'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse' \
      'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse' \
      'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse' \
      'deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse' \
      > /etc/apt/sources.list; \
    find /etc/apt/sources.list.d -type f -name '*.list' -exec sed -i \
      -e 's#http://archive.ubuntu.com/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ubuntu#g' \
      -e 's#http://security.ubuntu.com/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ubuntu#g' \
      -e 's#http://snapshots.ros.org/noetic/final/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu#g' \
      -e 's#http://packages.ros.org/ros/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu#g' \
      {} +; \
    rm -f /etc/apt/sources.list.d/*ros*.list; \
    printf '%s\n' 'deb [trusted=yes] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu focal main' \
      > /etc/apt/sources.list.d/ros1-tuna.list

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
      git wget autoconf automake nano \
      python3-dev python3-pip python3-scipy python3-matplotlib python3-yaml \
      ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
      libeigen3-dev libboost-all-dev libsuitesparse-dev \
      doxygen libopencv-dev libpoco-dev libtbb-dev libblas-dev liblapack-dev \
      libv4l-dev python3-catkin-tools python3-osrf-pycommon && \
    rm -rf /var/lib/apt/lists/*

ENV WORKSPACE=/catkin_ws \
    KALIBR_MANUAL_FOCAL_LENGTH_INIT=1

RUN mkdir -p "$WORKSPACE/src" && \
    cd "$WORKSPACE" && \
    catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY kalibr_source/ /catkin_ws/src/kalibr/

RUN cd "$WORKSPACE" && catkin build -j"${CATKIN_JOBS}"

WORKDIR /catkin_ws
CMD ["/bin/bash"]
