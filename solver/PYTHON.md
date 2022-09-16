# Python Wrapper for the SCAMPI Numerical Solver

The [pybind11](https://github.com/pybind/pybind11) bindings for the SCAMPI Numerical solvers are hosted in this folder. All is needed to do to install the package is running the following command while we are in this folder:

## Prerequisites

* A compiler with C++11 support
* Pip 10+ or CMake >= 3.4 (or 3.8+ on Windows, which was the first version to support VS 2015)
* Ninja or Pip 10+
* The requirements of the [SCAMPI C++ Kinematics solver](https://github.com/Rooholla-KhorramBakht/SCAMPI-Kinematics-Solver/tree/main/Ceres/solver_numerical)

For running the demo in the jupyter notebook, the [liegroups](https://github.com/utiasSTARS/liegroups) SO2, SE2, SO3, and SE3 matrix Lie groups implementation from the UTIAS is required.


## Installation

Just clone this repository and pip install.
```bash
cd python_wrapper
pip install .
```

With the `setup.py` file the `pip install` command will invoke the CMake and builds the solvers as a library and binds them using the pybind11 module as specified in `CMakeLists.txt`.


## How to Use it

An example use case has been shown in the Example.ipynb notebook.
