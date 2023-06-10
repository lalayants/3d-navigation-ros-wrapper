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

# Octomap installation
RUN mkdir /utils -p
WORKDIR /utils
RUN cd /utils && git clone https://github.com/OctoMap/octomap.git --branch master &&\
    cd octomap && mkdir build && cd build && cmake .. && make install

# FCL installation
RUN cd /utils && git clone https://github.com/flexible-collision-library/fcl.git --branch 0.7.0 &&\
    cd fcl && mkdir build && cd build && cmake .. && make install

# OMPL installation
RUN cd / && git clone https://github.com/ompl/ompl.git --branch 1.6.0
WORKDIR /build
RUN cmake \
        -DPYTHON_EXEC=/usr/bin/python3 \
        -DOMPL_REGISTRATION=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -G Ninja \
        /ompl && \
    ninja update_bindings -j `nproc` && \
    ninja -j `nproc` && \
    ninja install
WORKDIR /workspace/ros_ws

