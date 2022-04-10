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

#### Baseline System

Once within the Docker container one can launch our demonstration simulation via:

```bash
# build project
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# launch demo
roslaunch mppi_demo demo.launch
```

The default demo will spawn a Jackal in an Ignition world with the MPPI controller running. Companion scripts can be launched to manually send Waypoint goals or execute course following.

```bash
# send a waypoint goal at (3, 2)
rosrun mppi_demo send_goal.py -x 3 -y 2

# command a 'follow ellipse' trajectory for 60 seconds
rosrun mppi_demo follow_course.py -x 5 -y 5 -d 60

# see all commands for either script with 'rosrun mppi_demo SCRIPT --help'
```

Examples of each of the above demos executing for the nominal (read: untuned) system:

Waypoint Goal | Follow Course Goal
--- | ---
![](docs/waypoint_goals.gif) | ![](docs/course_following.gif)


#### Learning a Forward Model

The baseline system performance is sub-optimal, for a number of reasons. We just assumed a simplified geometric slip-steer model with hard-coded parameters like "wheel radius" that may not match the true (simulated) values. We also assume there's no slip. In this section we demonstrate how to learn these parameters in order to better match the system's performance[^1].

This effort can be broadly broken up into three stages:
1. Data Collection: Running simulations of the system to collect performance information. In order not to overfit to a specific scenario we introduce some stochasticity in the data collection procedure by varying the courses and simulating odometry and controller noise. We also randomly perturb the main Forward Model parameters which we expect to drive performance and record these.
2. Data Analysis: Once we have a sufficient (?) number of demonstrations we want to distinguish between "good" and "bad" runs. To this end we define a cost function which penalizes things like "distance from the desired trajectory", "deviation from desired velocity", etc. In this stage we extract all the data and assign costs to each run.
3. Model Fitting: Now we have a set of information mapping Forward Model parameters to system performance. We can try to fit this to an assumed pattern via Linear Regression. Once we have that model we can try to determine the suite of parameters which will provide the best system performance.

Scripts are provided to perform each of these stages:

```bash
# Data Collection requires the simulation to be up in the background
roslaunch mppi_demo demo.launch

# in a separate terminal start the data collection process
rosrun mppi_demo collect_course.py

# this will continually send new goals to the robot and collect data until it is manually shut down
```

```bash
# Data Analysis assumes a decent number of runs have been executed and collected
#  note that the simulator doesn't need to be running anymore
rosrun model_regression extract.py
```

```bash
# Model Fitting can be performed once all the data has been collected
rosrun model_regression fit.py

# This script will print out the optimized parameters once completed
```

[^1]: There are plenty of other issues that affect system performance, like controller timing, sampling stochasticity, vehicle constraints, bugs in my code, etc. We ignore these to focus on the more manageable problem of better forward modeling.

## Theory



## Acknowledgements

Some papers that were instrumental in grokking MPPI are as follows.

 - Williams, Grady & Aldrich, Andrew & Theodorou, Evangelos. (2015). Model Predictive Path Integral Control using Covariance Variable Importance Sampling. [PDF link](https://arxiv.org/pdf/1509.01149.pdf)
 - G. Williams, P. Drews, B. Goldfain, J. M. Rehg and E. A. Theodorou, "Aggressive driving with model predictive path integral control," 2016 IEEE International Conference on Robotics and Automation (ICRA), 2016, pp. 1433-1440, doi: 10.1109/ICRA.2016.7487277. 
 - Williams, Grady & Aldrich, Andrew & Theodorou, Evangelos. (2017). Model Predictive Path Integral Control: From Theory to Parallel Computation. Journal of Guidance, Control, and Dynamics. 40. 1-14. 10.2514/1.G001921. [PDF link](https://arc.aiaa.org/doi/pdf/10.2514/1.G001921)
 - G. Williams et al., "Information theoretic MPC for model-based reinforcement learning," 2017 IEEE International Conference on Robotics and Automation (ICRA), 2017, pp. 1714-1721, doi: 10.1109/ICRA.2017.7989202. [PDF link](https://faculty.cc.gatech.edu/~bboots3/files/InformationTheoreticMPC.pdf)
 - S. Rabiee and J. Biswas, "A Friction-Based Kinematic Model for Skid-Steer Wheeled Mobile Robots," 2019 International Conference on Robotics and Automation (ICRA), 2019, pp. 8563-8569, doi: 10.1109/ICRA.2019.8794216. [PDF link](https://www.joydeepb.com/Publications/icra2019_skid_steer.pdf)

Some open source projects and other links which were very helpful:

 - [GaTech AutoRally](https://github.com/AutoRally/autorally) provides an implementation of MPPI for a scaled racecar.
 - [bostoncleek's MPPI Implementation](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/controller) provides another example implementation of MPPI.


