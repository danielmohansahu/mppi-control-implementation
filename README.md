# mppi-control-implementation
Implementation of Model Predictive Path Integral Control, for learning purposes.

This project provides a rough implementation of the MPPI controller descrived in [Model Predictive Path Integral Control using Covariance Variable Importance Sampling](https://arxiv.org/pdf/1509.01149.pdf). A simulation-based development environment with a [Clearpath Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) is provided for demonstration purposes.

## Installation and Usage

The entirety of this project is encapsulated inside a Docker image for portability, which naturally requires [docker](https://docs.docker.com/get-docker/) to be installed. We also assume an nvidia GPU and the presence of [nvidia-docker](https://github.com/NVIDIA/nvidia-docker). Assuming those pre-requisites are met one can build and drop into a development environment via the following script:

**Security note:** We use the `docker run --privileged` flag to allow Docker to launch GUIs. This is not a "safe" command to execute and should be used with caution.

```bash
# xhost allow Docker; unsafe command!
xhost +si:localuser:root

# build image and drop into a container
./build_and_run.sh
```

Once within the Docker container one can launch our demonstration simulation via:

```bash
# build project
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# launch demo
roslaunch mppi_controller demo.launch
```

The default demo will spawn a Jackal in an Ignition world with the MPPI controller running.

## Theory



## Acknowledgements

Some papers that were instrumental in grokking MPPI are as follows.

 - Williams, Grady & Aldrich, Andrew & Theodorou, Evangelos. (2015). Model Predictive Path Integral Control using Covariance Variable Importance Sampling. [PDF link](https://arxiv.org/pdf/1509.01149.pdf)
 - G. Williams, P. Drews, B. Goldfain, J. M. Rehg and E. A. Theodorou, "Aggressive driving with model predictive path integral control," 2016 IEEE International Conference on Robotics and Automation (ICRA), 2016, pp. 1433-1440, doi: 10.1109/ICRA.2016.7487277. 
 - Williams, Grady & Aldrich, Andrew & Theodorou, Evangelos. (2017). Model Predictive Path Integral Control: From Theory to Parallel Computation. Journal of Guidance, Control, and Dynamics. 40. 1-14. 10.2514/1.G001921. [PDF link](https://arc.aiaa.org/doi/pdf/10.2514/1.G001921)
 - G. Williams et al., "Information theoretic MPC for model-based reinforcement learning," 2017 IEEE International Conference on Robotics and Automation (ICRA), 2017, pp. 1714-1721, doi: 10.1109/ICRA.2017.7989202. [PDF link](https://faculty.cc.gatech.edu/~bboots3/files/InformationTheoreticMPC.pdf)

Some open source projects which were very helpful:

 - [fields-ignition](https://github.com/azazdeaz/fields-ignition)
 - [GaTech AutoRally](https://github.com/AutoRally/autorally)
 - [Getting Started with Ignition Gazebo](https://cyaninfinite.com/getting-started-with-ignition-gazebo/)
 - [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/noetic)
 - [MPPI Implementation](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/controller)
 - [Skid Steer Kimatics](https://mdpi-res.com/d_attachment/sensors/sensors-15-09681/article_deploy/sensors-15-09681-v2.pdf)
 - [Skid Steer Friction Kinematics](https://www.joydeepb.com/Publications/icra2019_skid_steer.pdf)


