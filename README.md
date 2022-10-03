# Kinematics-Inertial Fusion for Localization of Suspended Cable-Driven Robots with Hefty Cables

[![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

This repository holds the Ceres implementation of the kinematic solver considering cable sag
and its corresponding ROS nodes for fusing it with an onboard IMU module.
This solver is described in the following IROS 2022 publication.

If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{SCAMPI-Kinematics-Solver,
   author     = {Allak, Eren and KhorramBakht, Rooholla and Brommer, Christian and Weiss, Stephan},
   booktitle  = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
   title      = {Kinematics-Inertial Fusion for Localization of a 4-Cable Underactuated Suspended Robot Considering Cable Sag},
   year       = {2022},
}
```

## License
This software is made available to the public to use (_source-available_),
licensed under the terms of the BSD-2-Clause-License with no commercial use
allowed, the full terms of which are made available in the `LICENSE` file.
No license in patents is granted.


## Prerequisites
- For the C++ solver and the ROS nodes
  - Install ROS as shown [here](http://wiki.ros.org/noetic/Installation/Ubuntu). We tried ROS1 noetic and also melodic on Ubuntu 18.04 and Ubuntu 20.

  - Install the [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) package.

  - Install also the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) library.

  - Install Ceres as described in this link [here](http://ceres-solver.org/installation.html).

  - Install GTest
    ```bash
    sudo apt-get install libgtest-dev
    cd /usr/src/gtest
    sudo cmake CMakeLists.txt
    sudo make
    sudo cp *.a /usr/lib
    ```

  - Install Manif (Lie Theory tools)
    ```bash
    git clone https://github.com/artivis/manif.git
    cd manif && mkdir build && cd build
    cmake ..
    sudo make install
    ```

- For the python code
  - install python3-pip
  - install `pip3 install pyquaternion`
  - install `pip3 install numpy`
  - install `pip3 install pyyaml`
  - install [liegroups](https://github.com/utiasSTARS/liegroups).

- Optional
  - `sudo pip3 install rospkg catkin_pkg` resolves some issues with roslaunch of our ros nodes.



## Kinematic-Inertial fusion for cable robots using ROS
### Compile ROS nodes for the solver and estimator for IMU fusion

- Setup a catkin workspace and get the code

  ```bash
  mkdir -p catkin_ws/src
  catkin init
  cd src
  git clone git@github.com:aau-cns/scampi_ks_mars_fusion.git
  ```

- Compile and install the solver.
  ```bash
  cd scampi_ks_mars_fusion
  cd solver
  mkdir build && cd build
  cmake ..
  sudo make all
  pip3 install .
  ```

- Get the code for the estimator as a git submodule and build the ROS nodes
  ```bash
  cd ../.. # back to .../scampi_ks_mars_fusion
  git submodule update --init --recursive
  catkin build scampi_ks_mars_fusion
  ```
- Source your catkin workspace such that you can use the nodes now
  ```bash
  cd ../.. # back to .../catkin_ws
  source devel/setup.bash
  ```

### Run ROS nodes for the solver and estimator for IMU fusion
Our software setup has **4 main components**:
- The kinematic solver (with an equilibrium detector) `scampi_ks_ros`
- The estimator for IMU fusion `mars_ros`
- Two scripts for configuration of the solver
  - send_init_cable_length.py
  - set_pose_source.py
- ROS tools (roscore, rosbag, rviz)

**Procedure**: The kinematic solver needs to know the total initial cable length from the pulley to the anchors,
such that later cable encoder readings can be used to increment the total cable length.
Furthermore, the solver starts in 'standalone' mode, meaning that it computes the
end-effector pose from previous results, without using the output of the
estimator.
The estimator initializes itself with the output of the solver in standalone mode.
One additional solver output is used for the first update of the estimator.
At this point we consider the estimator to be robust and can use its output in the solver.

**Cable length input to the solver:** We assume that the input to the solver has the form
l_input = l_pulley_to_anchor + l_other.
Only the last part of the cable connecting the anchor on the end-effector and the pulley is relevant for us, this is described as l_pulley_to_anchor. And l_other describes all other cable parts which are not relevant for catenary computations, e.g. the length of the cable going from winch to pulley.
l_other is considered to be constant.
One time at the beginning the true l_pulley_to_anchor needs to be known and send to the solver, such that we can compute the constant l_other.

- Start ROS and replay the bagfile
```bash
# TERMINAL 1, 2 tabs
cd location/of/bagfile
roscore
rosbag play 2D_CD_TC21_1_2020-12-01-16-56-00.bag  --pause -r 1.0 # start in pause mode
```

- On each new terminal that is started the setup.bash in devel needs to be sourced like this `source devel/setup.bash`.
Or the sourcing is written to the `.bashrc` file, such that this is automated.

- Start the estimator, the equilibrium detector and the kinematic solvers
```bash
# TERMINAL 2, 3 tabs
cd catkin_ws
source devel/setup.bash
# Estimator
roslaunch mars_ros mars_pose.launch imu_in_topic:=/mti/sensor/imu pose_with_cov_in_topic:=/equilibrium_detector/eq_pose
# Equilibrium detector (Not doing much yet, future work)
roslaunch equilibrium_detector equilibrium_detector_node.launch
# Kinematics solvers
roslaunch scampi_ks_ros solver_start_prop.launch
```

- Configure the solver and initialize estimator, let the bagfile run
```bash
# TERMINAL 3
cd scampi_ks_mars_fusion/ros
# Set the true l_pulley_to_anchor, see notes on cable length input
python send_init_cable_length.py
# In the bagfile terminal, play back the bagfile a few steps by hitting 's' some seconds
# until in the estimator tab the following messsage comes:
#
# "Update: 1.60684e+09
# Warning: Core is not initialized yet. Measurement is stored but not processed"
#
# This initialized the estimator and updated it once.
# Now the estimator can be used as input for the solver
python set_pose_source.py 1
# In the bagfile terminal, hit 'space' and just let the bagfile run.
```

- Visualize the data in rviz
```bash
# Another terminal
rosrun tf static_transform_publisher 0 0 0 0 0 0 world map 100
# Another terminal
rosrun rviz rviz
# open the provided rviz config
```

- Record data for plotting
```bash
rosbag record -O test.bag /dh_optitrack/CustomDolly/pose /dh_optitrack/D2BackLeft/pose /dh_optitrack/D2BackRight/pose /dh_optitrack/D2FrontLeft/pose /dh_optitrack/D2FrontRight/pose /equilibrium_detector/eq_pose /mars_pose_node/pose_state_out /mars_pose_node/pose_state_out_center /mti/sensor/imu /scampi_ks_ros/fk_pose /spc_ADS_node/spc_cable_length /spc_ADS_node/spc_dolly_pose
```

## Unit tests for solver
Trying out the Kinematic Solver and running some unit tests with google tests.

- Clone the repository

  ```bash
  git clone git@github.com:aau-cns/scampi_ks_mars_fusion.git
  cd scampi_ks_mars_fusion
  git submodule update --init --recursive
  ```
- Compile the library and test modules
  ```bash
  cd solver
  mkdir build && cd build
  cmake ..
  sudo make all
  ```
- Test the library
  ```bash
  ./tests_cost_funcs
  ./tests_solver_funcs
  ./tests_util_funcs
  ```


## Python interface of solver
Here is some data provided from our cable robot and also from simulations.
We are using the python interface of our C++ libraries in a jupyter notebook.
- Clone the repository
  ```bash
  git clone git@github.com:aau-cns/scampi_ks_mars_fusion.git
  cd scampi_ks_mars_fusion
  git submodule update --init --recursive
  ```
- Compile and install the library with pip
  ```bash
  cd solver
  pip3 install .
  ```

- Try out the notebooks
``` bash
cd solver/notebooks
jupyter-notebook
```
