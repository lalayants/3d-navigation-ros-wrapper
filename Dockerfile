ARG BASE_IMG='ubuntu:20.04'

FROM ${BASE_IMG}
SHELL ["/bin/bash", "-ci"]

# Timezone Configuration
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace/ros_ws

RUN apt update && apt install -y \
    git vim curl nano tmux curl wget lsb-release \
    net-tools build-essential gcc g++ \
    cmake clang make \
    python3 python3-dev python3-pip python3-distutils libpython3-dev \
    gnupg2 ca-certificates software-properties-common \
    libboost-dev libeigen3-dev

# ROS1 Install
ENV ROS1_DISTRO=noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros-focal.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - && \
    apt update && apt install -y ros-$ROS1_DISTRO-ros-base && \
    echo "source /opt/ros/$ROS1_DISTRO/setup.bash">> ~/.bashrc && \
    echo "source /workspace/ros_ws/devel/setup.bash">> ~/.bashrc && \
    apt update && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator \
        python3-wstool python3-catkin-tools python-lxml&& \
    rosdep init && rosdep update && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade numpy matplotlib

RUN apt-get update && apt-get install -y \
    ros-$ROS1_DISTRO-perception ros-$ROS1_DISTRO-xacro \
    ros-$ROS1_DISTRO-robot ros-$ROS1_DISTRO-image-common \
    ros-$ROS1_DISTRO-rviz ros-$ROS_DISTRO-tf2-tools && \
    rm -rf /var/lib/apt/lists/*

# ROS1 Install base packages
# RUN apt update && apt install -y \
#     && \
#     rm -rf /var/lib/apt/lists/*


RUN apt-get update && \
    apt-get install -y \
        castxml \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libpcl-dev \
        libexpat1 \
        libflann-dev \
        libtriangle-dev \
        ninja-build \
        pkg-config \
        python3-pip \
        pypy3 \
        wget \
        git && \
    # Install spot
    wget -O /etc/apt/trusted.gpg.d/lrde.gpg https://www.lrde.epita.fr/repo/debian.gpg && \
        echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && apt-get install -y libspot-dev && \
        apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install pygccxml pyplusplus scipy

RUN mkdir /utils -p
WORKDIR /utils

# Octomap installation
RUN cd /utils && git clone https://github.com/OctoMap/octomap.git --branch v1.8.1 &&\
    cd octomap && mkdir build && cd build && cmake .. && make install && \
    apt update && \
    apt install -y ros-noetic-octomap ros-noetic-octomap-msgs ros-noetic-octomap-ros \
        ros-noetic-octomap-server

# FCL installation
RUN cd /utils && git clone https://github.com/danfis/libccd.git && \
        cd libccd && mkdir build && cd build && cmake -G Ninja .. && ninja && ninja install &&\
    cd /utils && git clone https://github.com/flexible-collision-library/fcl.git --branch 0.7.0 &&\
        cd fcl && mkdir build && cd build && cmake .. && make install

# OMPL installation
RUN cd /utils && git clone https://github.com/ompl/omplapp.git --branch 1.6.0 --recursive &&\
    cd omplapp && mkdir -p build/Release && cd build/Release && cmake ../.. && \
    make && make install 

WORKDIR /workspace/ros_ws

COPY . /workspace/ros_ws/src/3d_navigation
RUN cd /workspace/ros_ws/ && rosdep install -y --from-paths src --ignore-src --rosdistro noetic
# RUN cd src/3d_navigation && mkdir -p build && cd build && cmake .. && make 

