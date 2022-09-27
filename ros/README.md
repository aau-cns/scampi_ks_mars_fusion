# Introduction
 [![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

This is a ROS implementation of IMU and kinematic fusion for the SCAMPI robot based on the MARS EKF fusion backend. The scampi_ks_ros package in this repository utilizes the python wrapped Ceres implementation of the forward kinematics solver to implement a node that listens to the cable length measurements and publishes the forward kinematics solutions in the form of pose measurements for the MARAS EKF platform. On the other hand, the MARS EKF propagates the poses and generates a suitable initial guess for the next iteration of the solver. 

# Installation
## Requirements
The following prerequisites should be installed first:
```sh
$ pip install pyquaternion
$ pip install numpy
$ pip install pyyaml
```
In addition to the above, the liegroup library should also be installed as explained [here](https://github.com/utiasSTARS/liegroups). Furthermore, it is assumed that the 'solvers_ceres/solver_autodiff/python_wrapper' is installed in a system-wide fashion.

## Creating the Catkin Workspace
Create a catkin workspace and copy the nodes in this directory to the src directory of the created workspace:

```sh
$ mkdir scampi_ks_ws && cd scampi_ks_ws
$ git clone https://gitlab.aau.at/aau-cns/scampi_ks.git
$ git submodule update --init --recursive
$ mkdir -p ~/catkin_ws/src && cd catkin_ws/src
$ cp -r ../../scampi_ks/mars_fusion_ros .
$ catkin_make
$ source devel/setup.bash
```
# Usage

## Modules

| Module Name                     | Description                                 |
| --------------------------------| ------------------------------------------- |
| scampi_ks_ros/scampi_ks_node.py | The forward kinematics solver node          |

## Parameter

The forward kinematics node takes robot parameters stored in the `scampi_ks_ros/configs/robot_params.yaml` file. 

| Parameter                   | Description                                                  | Default Value |
| --------------------------- | ------------------------------------------------------------ | ------------- |
| `pulleys`               | The pulley locations in the world coordinate frame.              | set according to the dataset|
| `anchors`  | Cable attachment points on the end-effector expressed in the end-effector body coordinate. | set according to the dataset|
| `r_to_cog`                   | ToDo: Add explanation                                          | set according to the dataset         |
| `gc`      | Cable Specifi parameter set according to the formula in the Matlab implementation.           | set according to the dataset|
| `fg`     | The gravity force applied to the end-effector | set according to the dataset          |

In addition to the above parameters, the following runtime parameters are also available and fed to the node through the launch file:
| Parameter                   | Description                                                  | Default Value |
| --------------------------- | ------------------------------------------------------------ | ------------- |
| `rate`               | The update rate of the solver in Hz              | 10|
| `robot_config`                   | Path to the YAML configuration file.                                         | the default config file stored in `configs` directory         |

## Service
The scampi_ks node will not run until the following service is called at least once. This is to make sure the solvers won't be running with the wrong initial cable length values.

| Topic          | Description                     |
| -------------- |  ------------------------------- |
| `/scampi_ks_ros/init_cable_lengths` | Gets the equilibrium  pose and cable_lenghts and initlizes the cable length. |
| `/scampi_ks_ros/set_pose_source` | which pose source to use for initilizing the solver at each iteration: 0: solver's previous result 1: EKF pose topic. |

## Topics

| Topic                          | Publisher / Subscriber | Type                                     | Content                                    |
| ------------------------------ | ---------------------- | ---------------------------------------- | ------------------------------------------ |
| `cable_len`                 | subscriber             | spc_ads_pub::CableLength   | Cable length measurements from the Spidercam      |
| `ekf_pose_prop`           | subscriber             | geometry_msgs::PoseStamped        | Incoming 6Dof propagated poses from MARS    |
| `fk_pose`                | publisher             | geometry_msgs::PoseStamped        | 6Dof estimated poses of the end-effector |    
