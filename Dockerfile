ARG BASE_IMG
FROM ${BASE_IMG}

ENV DEBIAN_FRONTEND=noninteractive
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
    make update_bindings && make && make install 

WORKDIR /workspace/ros_ws

COPY . /workspace/ros_ws/src/3d_navigation
RUN cd /workspace/ros_ws/ && rosdep install -y --from-paths src --ignore-src --rosdistro noetic
# RUN cd src/3d_navigation && mkdir -p build && cd build && cmake .. && make 

