#!/usr/bin/bash

set -eo pipefail

# Use Xacro to update the latest Jackal SDF file.
rosrun xacro xacro $(catkin locate jackal_ignition)/models/jackal.urdf.xacro > $(catkin locate jackal_ignition)/models/jackal.urdf.original

# convert to SDF format
ign sdf -p $(catkin locate jackal_ignition)/models/jackal.urdf.original > $(catkin locate jackal_ignition)/models/jackal.sdf.original

# plugins from jackal.sdf.origin -> jackal.sdf are manually updated
echo "jackal.sdf.original successfully update."
