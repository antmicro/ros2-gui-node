# syntax=docker/dockerfile:1
ARG BASE_IMAGE=nvidia/cuda:12.9.1-cudnn-devel-ubuntu24.04
FROM $BASE_IMAGE

ARG BUILD_TVM
ARG ADDITIONAL_PACKAGES

ENV INST='env DEBIAN_FRONTEND=noninteractive apt-get install -y'
ENV PIPINST='python3 -m pip install --no-cache-dir --upgrade'

RUN apt-get update && $INST \
    apt-utils \
    ca-certificates \
    curl \
    gnupg \
    locales \
    locales-all \
    wget \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \"$(source /etc/os-release && echo "$VERSION_CODENAME")\" main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get upgrade -y && $INST \
    clang-14 \
    clang-format-14 \
    clang-tidy-14 \
    clangd-14 \
    cmake \
    cppcheck \
    ffmpeg \
    g++-12 \
    gcc \
    gdb \
    git \
    git-lfs \
    gnupg \
    libboost-python-dev \
    libedit-dev \
    libglew-dev \
    libglfw3-dev \
    libglm-dev \
    libgtest-dev \
    libgtk-3-dev \
    libopencv-contrib-dev \
    libopencv-dev \
    libtinfo-dev \
    libv4l-dev \
    libvulkan-dev \
    libxml2-dev \
    libyaml-dev \
    llvm-14 \
    lsb-release \
    make \
    ninja-build \
    pkg-config \
    protobuf-compiler \
    python3-dev \
    python3-flake8 \
    python3-flake8-docstrings \
    python3-opencv \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-venv \
    python3-empy \
    python3-catkin-pkg \
    python3-lark \
    rapidjson-dev \
    ros-dev-tools \
    spirv-tools \
    sudo \
    unzip \
    valgrind \
    vim \
    vulkan-tools \
    vulkan-utility-libraries-dev \
    wget \
    xterm \
    zlib1g-dev \
    $ADDITIONAL_PACKAGES \
    && apt-get -y clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN curl https://storage.googleapis.com/git-repo-downloads/repo > /usr/local/bin/repo && \
    chmod a+x /usr/local/bin/repo

RUN ln -s /usr/bin/python3 /usr/bin/python

ENV CMAKE_POLICY_VERSION_MINIMUM=3.10

# ROS
WORKDIR /ros2-build
RUN mkdir src && \
    apt-get update && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos --recursive --workers "$(nproc)" src && \
    rosdep init --rosdistro=humble && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-math6 ignition-cmake2" && \
    colcon build --install-base /opt/ros --cmake-args "-DCMAKE_POLICY_VERSION_MINIMUM=3.10" --cmake-force-configure --cmake-clean-first --cmake-clean-cache && \
    rm -rf /ros2-build && \
    rm -rf /var/lib/apt/lists/*

COPY ./third-party/vision_opencv /vision_opencv
WORKDIR /vision_opencv/cv_bridge
RUN source /opt/ros/setup.sh && \
    colcon build --install-base /opt/ros && \
    rm -rf /vision_opencv
WORKDIR /

# TVM
COPY ./third-party/tvm /tvm
COPY ./tvm-config.cmake /tvm-config.cmake
WORKDIR /tvm/build
RUN if [[ -n "$BUILD_TVM" ]]; then \
    cp /tvm-config.cmake config.cmake && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j"$(nproc)" && make install && \
    CONDA_BUILD=yes LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH $PIPINST /tvm/python && \
    rm -rf /tvm/build && ldconfig; fi
WORKDIR /