# syntax=docker/dockerfile:1
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV INST 'env DEBIAN_FRONTEND=noninteractive apt-get install -y'
ENV PIPINST 'python3 -m pip install --no-cache-dir --upgrade'

RUN apt-get update && $INST \
    apt-utils \
    ca-certificates \
    curl \
    gnupg \
    locales \
    locales-all \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \"$(source /etc/os-release && echo "$VERSION_CODENAME")\" main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get upgrade -y && $INST \
    clang-13 \
    clang-format-13 \
    clang-tidy-13 \
    clangd-13 \
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
    llvm-13 \
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
    rapidjson-dev \
    ros-dev-tools \
    spirv-tools \
    sudo \
    unzip \
    valgrind \
    vim \
    vulkan-tools \
    vulkan-validationlayers-dev \
    wget \
    xterm \
    zlib1g-dev \
    && apt-get -y clean && apt-get autoremove && rm -rf /var/lib/apt/lists/*

RUN $PIPINST \
    Cython==0.29.35 \
    Pillow \
    argcomplete \
    astunparse \
    boto3 \
    botocore \
    cmake \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    isort \
    jsonschema \
    matplotlib \
    memory_profiler \
    mock \
    numpy \
    onnx \
    onnxruntime \
    onnxruntime_gpu \
    pandas \
    pip \
    pre-commit \
    psutil \
    pycocotools \
    pytest \
    pytest-repeat \
    pytest-rerunfailures \
    scikit_learn \
    scipy \
    setuptools \
    tqdm \
    wheel && \
    ldconfig

RUN curl https://storage.googleapis.com/git-repo-downloads/repo > /usr/local/bin/repo && \
    chmod a+x /usr/local/bin/repo

RUN ln -s /usr/bin/python3 /usr/bin/python

# ROS
WORKDIR /ros2-build
RUN mkdir src && \
    apt-get update && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos --recursive --workers "$(nproc)" src && \
    rosdep init --rosdistro=humble && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-math6 ignition-cmake2" && \
    apt-get remove -y pybind11-dev && \
    $PIPINST "pybind11[global]" && \
    colcon build --install-base /opt/ros && \
    rm -rf /ros2-build && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /

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
RUN cp /tvm-config.cmake config.cmake && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j"$(nproc)" && make install && \
    CONDA_BUILD=yes LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH $PIPINST /tvm/python && \
    rm -rf /tvm/build && ldconfig
WORKDIR /
