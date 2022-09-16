# Kinematic-Inertial Fusion for Localization of Suspended Cable-Driven Robots with Hefty Cables

[![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

This repository holds the Ceres implementation of the kinematic solver presented
in the following IROS 2022 publication and its corresponding ROS nodes for
fusing it with an onboard IMU module.

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
- Install Ceres as described in this link [here](http://ceres-solver.org/installation.html).
- Install GTest
  ```bash
  sudo apt-get install libgtest-dev
  cd /usr/src/gtest
  sudo cmake CMakeLists.txt
  sudo make
  sudo cp *.a /usr/lib
  ```

- Install Manif (Lie Theorie tools)
  ```bash
  git clone https://github.com/artivis/manif.git
  cd manif && mkdir build && cd build
  cmake ..
  sudo make install
  ```
- To run the notebooks, install [liegroups](https://github.com/utiasSTARS/liegroups).

- Install ROS as shown [here](http://wiki.ros.org/noetic/Installation/Ubuntu). We tried ROS1 noetic and also melodic on Ubuntu 18.04 and Ubuntu 20.

- Install the [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) package.

- Install also the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) library.

## Running the solver

### Trying out the Kinematic Solver and running the google tests

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


### Trying out the Kinematic Solver with Jupyter Notebooks and install python library
- Install python prerequisites
  ```bash
  sudo apt install python3-pip
  pip3 install pyquaternion
  pip3 install numpy
  pip3 install pyyaml
  # sudo pip3 install rospkg catkin_pkg
  # resolves some issues with roslaunch of our ros nodes.
  ```
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

### Try out the solver with ROS
The ROS nodes in this repository require the system-wide installation of the python wrapper as shown above.

- Get the code, setup the catkin workspace, and compile the code

```bash
mkdir -p catkin_ws/src
catkin init
cd src
git clone git@github.com:aau-cns/scampi_ks_mars_fusion.git
cd scampi_ks_mars_fusion
cd solver
mkdir build && cd build
cmake ..
sudo make all
pip3 install .
cd ../..
git submodule update --init --recursive
catkin build scampi_ks_mars_fusion
cd ..
source devel/setup.bash
```

- Now run the code

- TODO: notes of commands to clean readme form.

```bash
```
