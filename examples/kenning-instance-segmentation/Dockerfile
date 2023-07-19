FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV INST 'env DEBIAN_FRONTEND=noninteractive apt-get install -y'
ENV PIPINST 'python3 -m pip install --no-cache-dir --upgrade'

RUN apt-get update && $INST apt-utils locales locales-all ca-certificates
RUN locale-gen en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN locale

RUN $INST \
    curl \
    gnupg \
    software-properties-common \
    sudo

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN cat /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get upgrade -y

RUN $INST \
    clang-13 \
    clangd-13 \
    clang-format-13 \
    clang-tidy-13 \
    cmake \
    cppcheck \
    ffmpeg \
    g++-12 \
    gcc \
    gdb \
    git \
    git-lfs \
    gnupg \
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
    && apt-get -y clean && apt-get autoremove

RUN $PIPINST pip

RUN $PIPINST \
    argcomplete \
    astunparse \
    boto3 \
    botocore \
    cmake \
    Cython==0.29.35 \
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
    Pillow \
    psutil \
    pycocotools \
    pytest \
    pytest-repeat \
    pytest-rerunfailures \
    scikit_learn \
    scipy \
    setuptools \
    tqdm \
    wheel

RUN curl https://storage.googleapis.com/git-repo-downloads/repo > /usr/local/bin/repo && \
    chmod a+x /usr/local/bin/repo

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN ldconfig

RUN mkdir -p /ros2-build/src && \
    cd /ros2-build && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos --recursive --workers `nproc` src && \
    rosdep init --rosdistro=humble && \
    rosdep update --rosdistro=humble && \
    rosdep install --rosdistro=humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-math6 ignition-cmake2" && \
    apt-get remove -y pybind11-dev && \
    $PIPINST pybind11[global] && \
    colcon build --install-base /opt/ros && \
    cd / && \
    rm -rf /ros2-build

COPY ./third-party/tvm /tvm
COPY ./third-party/tvm-config.cmake /tvm-config.cmake
RUN cd /tvm && \
    mkdir build && cd build && \
    cp /tvm-config.cmake config.cmake && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j`nproc` && make install && cd / && rm -rf /tvm/build && ldconfig && \
    CONDA_BUILD=yes LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH $PIPINST /tvm/python
