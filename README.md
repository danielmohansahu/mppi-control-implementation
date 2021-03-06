# mppi-control-implementation
Implementation of Model Predictive Path Integral Control, for learning purposes.

This project provides a rough implementation of the MPPI controller descrived in [Model Predictive Path Integral Control using Covariance Variable Importance Sampling](https://arxiv.org/pdf/1509.01149.pdf), as well as an approach for learning the robot's Forward Model via Linear Regression. A simulation-based development environment with a [Clearpath Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) is provided for demonstration purposes.

Please see the [theory section](#theoretical-overview) for an overview of the methodology of this project.

## Installation and Usage

The entirety of this project is encapsulated inside a Docker image for portability, which naturally requires [docker](https://docs.docker.com/get-docker/) to be installed. We also assume an nvidia GPU and the presence of [nvidia-docker](https://github.com/NVIDIA/nvidia-docker). Assuming those pre-requisites are met one can build and drop into a development environment via the following script:

**Security note:** We use the `docker run --privileged` flag to allow Docker to launch GUIs. This is not a "safe" command to execute and should be used with caution.

```bash
# xhost allow Docker; unsafe command!
xhost +si:localuser:root

# optionally download a pre-compiled Docker Image
docker pull ghcr.io/danielmohansahu/mppi-impl:latest

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

This is all executed in the default simulation environment:

![](docs/simulation_environment.png)



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

## Results

After collecting approximately 600 training examples we received the following "optimal" parameters. This process is inherently stochastic, but these are fairly close to what we expected, e.g. the parameters outlined in our ![model definition](src/jackal_ignition/models/jackal.urdf.xacro). It's worth noting that since we only rely on the platform's reported odometry and have no ground truth estimate we effectively ignore much of the Gazebo simulation's fidelity. More on this in the [Issues section](#issues--future-work) below.

![](docs/optimal_params.png)

After training we went back and tested performance by manually sending some course and waypoint goals. The results aren't a shocking improvement[^2], but they do show a slight improvement in our velocity goal.

Goal Type | Default Params | Optimal Params
--- | --- | ---
Waypoint | ![](docs/default-waypoint-demo.gif) | ![](docs/optimal-waypoint-demo.gif)
Course | ![](docs/default-follow-course-demo.gif) | ![](docs/optimal-follow-course-demo.gif)

[^2]: A slightly philosophical tangent. I considered artificially selecting a "bad" set of forward model parameters at the outset of this project to potentially highlight the improvements, but ultimately decided against it. I find it to be somewhat disengenuous to take that approach because, in practice, one would very rarely be implementing this sort of learning to replace a completely untuned model.

## Issues / Future Work

There is a continuum of potential improvements for and issues with this project, but I want to highlight some of the most pressing.

1. As mentioned above, the Ignition world doesn't really provide anything of value in this project. That is because we're taking the reported open loop odometry from the Jackal model as our true odometry. For example, if the platform gets flipped over it will still report "success" in achieving goals despite the fact that its wheels are spinning uselessly. Technically this doesn't affect the results of this effort, since we're then just learning the odometry of the Ignition Jackal model in isolation, but it would be much more interesting to also encapsulate the wheel-ground interaction and all the other minutae of the full simulation. The correct way to do this would be to use ground truth odometry.

2. Our forward model learning process only converges for a fairly narrow range of parameters. We experimented with [increasing the bounds](https://github.com/danielmohansahu/mppi-control-implementation/commit/d4949d17e8b25ad12e661847fd2997cf47f6b34c), but that resulted in "optimal" slip params that saturated the widened bounds and failed to converge on the expected physical parameters like wheel radii. I have a vague suspicion this is due to a bug rather than some fundamental limitation of this approach, but was unable to confirm or deny that. Results of each are as follows:

Model Fitting (Narrow) | Model Fitting (Wide)
--- | ---
![](docs/fit_results_narrow_param_range.png) | ![](docs/fit_results_wide_param_range.png)

## Theoretical Overview

This project covers a number of topics in Robotics and Machine learning that are the subject of a plethora of papers, books, articles, classes, etc. This section attempts to explain some of the more important topics at a very broad level rather than diving into the technical details and underlying theory. For those sorts of details please see the [Acknowledgements section](#acknowledgements) below.

Speaking generally, we created a simulation (using Ignition Gazebo) of a particular robotic unmanned ground vehicle (UGV) (the Clearpath Jackal). Then, we wrote a Controller which converts goals (desired locations) and states (our position and velocity estimates) into actions (steering and throttle commands) which are used to drive the robot autonomously.

This Controller is an example of [Model Predictive Control (MPC)](https://en.wikipedia.org/wiki/Model_predictive_control), which basically means we use an approximation of our platform (the Forward Model) to figure out the best actions to take over a limited time horizon, like the next 5 seconds. This particular form of MPC is called Model Predictive Path Integral (MPPI) control, which is distinguished from other forms of MPC by how the model is considered. Instead of analytically or numerically converting the Forward Model (actions -> states) into an Inverse Model (states -> actions), we randomly sample the Forward Model to consider a whole host of possible actions and pick the best one. This process of randomly sampling to figure out an underlying process is called the [Monte Carlo method](https://en.wikipedia.org/wiki/Monte_Carlo_method). We use this method twice, because currently we have a problem.

The MPPI controller we developed works well _if_ we have a good Forward Model. If our Forward Model is terrible then we'll have terrible results, because our controller can't predict what the robot will actually do. One way to figure out our Forward Model is to use analytical methods, like looking at the underlying physics of the robot. Another option would be to backtrack and use Machine Learning to solve the problem without a model at all [(e.g. Unsupervised Learning)](https://en.wikipedia.org/wiki/Unsupervised_learning). We chose to use a method that bridges the gap between these approaches, allowing us to use our existing knowledge of the robot (i.e. that it's a Skid-Steer platform) but also not worry about the difficult analytical methods required to calculate things like coefficients of friction. This is called [Supervised Learning](https://en.wikipedia.org/wiki/Supervised_learning), because we are dictating the structure of the problem. Specifically, we know the equation for the Forward Model and want to just learn specific parameters.

In order to learn the "optimal" Forward Model we again employ the Monte Carlo method to collect a host of training examples. These are simulations of the robot driving around and trying to follow an elliptical path with randomized Forward Model parameters. Each training example is then evaluated using a cost function, which lets us compare the performance of different runs. For example, a run which doesn't maintain our desired velocity will score lower than a run which does (all other factors being equal). Finally, we can figure out the underlying equation which returns the cost of given set of parameters, `f(wheel_radius, wheel_seperation, slip, ...) = cost`. We find this equation using [Linear Regression](https://en.wikipedia.org/wiki/Regression_analysis) and use the global minimum (i.e. the set of parameters where the cost is lowest) to determine our "optimal" Forward Model parameters.

That's the theory in _very_ broad strokes. Please see below for much more detailed explanations.

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
 - [OSRF SubT Virtual Competition Software](https://github.com/osrf/subt) provides examples of how to use Ignition Gazebo.


