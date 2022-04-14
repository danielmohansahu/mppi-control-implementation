#!/usr/bin/bash

# Build docker image and drop into a container, mounting the source code
#  for ease of development.
# Warning: this uses docker `--privileged`; it is not "safe".

set -eo pipefail

# get path to here
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# target / expected image name
DOCKERIMAGE="ghcr.io/danielmohansahu/mppi-impl:latest"

# check if we want to force rebuild the image
REBUILD=0
if [[ "$#" -eq 1 ]]; then
  echo "Forcing a rebuild of docker."
  REBUILD=1
fi

# build and tag docker image, if it's not present
if [[ $REBUILD -eq 1 ]] || [[ ! $(docker inspect $DOCKERIMAGE) ]]; then
  docker build --cache-from $DOCKERIMAGE -t $DOCKERIMAGE -f $SCRIPTPATH/Dockerfile .
fi

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
    $DOCKERIMAGE \
    byobu

