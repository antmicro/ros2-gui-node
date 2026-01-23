# Docker environments scripts

## build-docker.sh 

It is a script used for building Docker image for **kenning-multimodel-demo** and **kenning-instance-segmentation** demo.

As a input argument you can specify platform for which you want to build image,
if you want to build for **Nvidia Jetson** you type:

``` bash
./build-docker.sh jetson
```

When you type only:

``` bash
./build-docker.sh
```

image will be built for x86 based platform in mind.

### Supported platforms

Currently we support platforms:

|    Platform   	| Input argument 	|
|:-------------:	|:--------------:	|
|      x86      	|                	|
| Nvidia Jetson 	|     jetson     	|


### Environments variables:

build-docker.sh also supports various environmental variables:

- DOCKER_TAG - it specified a tag for built image
- BASE_IMAGE - it is a name of base image that will be used to build target image
- BUILD_TVM - set it to blank if you don't want to build TVM for image
- ADDITIONAL_PACKAGES - you can use it to specified additional APT packages that image should come with

## run-docker.sh

It is a script used for running docker container based on built image. 

As input argument you can specified a command that container 
should execute, for example:

``` bash
./run-docker.sh "echo Hello World!"
```
will just print `Hello World!` upon container startup.

If you execute command without any argument like so:

``` bash
./run-docker.sh
```
A standard **bash** terminal will start in which you can type commands.

### Environments variables:

run-docker.sh supports various environmental variables:

- CAMERA_PATH - specified a path to camera device that container should have access to
- DOCKER_IMAGE - a name of the docker image that container should run
- USE_PLATFORM - specified a platform that container is running on ( for supported platforms look into `Supported platforms` section )
- USE_NON_INTERACTIVE - when a variable is not blank a container will run in non interactive mode
- DISPLAY - X Window display that container should use