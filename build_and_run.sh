#!/usr/bin/bash

# Build docker image and drop into a container, mounting the source code
#  for ease of development.
# Warning: this uses docker `--privileged`; it is not "safe".

set -eo pipefail

# get path to here
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# build and tag docker image
docker build -t ghcr.io/danielmohansahu/mppi-impl:latest -f $SCRIPTPATH/Dockerfile .

# drop into a container
docker run \
    -it \
    --rm \
    --gpus=all \
    --ulimit core=-1 \
    --security-opt seccomp=unconfined \
    --cap-add=SYS_PTRACE \
    --net=host \
    --privileged \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v $SCRIPTPATH:/workspace \
    ghcr.io/danielmohansahu/mppi-impl:latest \
    byobu

