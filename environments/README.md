# Docker environments scripts

This document describes scripts for building Docker images for ROS 2 demos.

## Build Docker image for demos

Project comes with helper `build-docker.sh` script building Docker image for demos.
It is a script used for building Docker image for **kenning-multimodel-demo** and **kenning-instance-segmentation** demo.

To build Docker image for the current platform, run:

``` bash
./build-docker.sh
```

To build Docker image for a different platform, e.g. NVIDIA Jetson, pass platform name as an argument, e.g.:

``` bash
./build-docker.sh jetson
```

### Supported platforms

Currently demos can be executed on:

|    Platform   	| Input argument 	|
|:-------------:	|:--------------:	|
|      `x86`      	|                	|
| `NVIDIA Jetson` 	|     `jetson`     	|


### Environments variables:

`build-docker.sh` also supports following environment variables:

- `DOCKER_TAG` - specifies a tag for built image
- `BASE_IMAGE` - specifies a name of base image that will be used to build target image
- `BUILD_TVM` - tells whether to build TVM in the image
- `ADDITIONAL_PACKAGES` - specifies additional APT packages to install in the Docker image

## Running Docker container for demos

Project comes with helper `run-docker.sh` script setting up a container for demos.

If you execute command without any argument like so:

``` bash
./run-docker.sh
```

A standard **bash** terminal will start in which you can type commands.

You can also pass command to execute as an argument.

### Environments variables:

`run-docker.sh` supports following environment variables:

- `CAMERA_PATH` - specifies a path to camera device that should be used in container
- `DOCKER_IMAGE` - a name of the docker image that container should run
- `USE_PLATFORM` - specified a platform that container is running on (for supported platforms look into `Supported platforms` section)
- `USE_NON_INTERACTIVE` - when a variable is set the container will run in non-interactive mode
- `DISPLAY` - X Window display that container should use
