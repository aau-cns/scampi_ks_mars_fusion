// Copyright (C) 2021 Rooholla Khorram Bakht, Eren Allak,
// and others, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <r.khorrambakht@alumni.kntu.ac.ir>,
// <eren.allak@aau.at>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include <ceres/ceres.h>
#include <Eigen/Dense>
//#include "libscampi_ks_costs.h"
#include "libscampi_ks_solvers.h"


using namespace ceres;
using namespace Eigen;

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

class CableRobotParams
{

  public:
    CableRobotParams(const double g_c, const double f_g): f_g_(f_g), g_c_(g_c) {}

    Vector3d p1_, p2_, p3_, p4_;
    Vector3d b1_, b2_, b3_, b4_;
    Vector3d r_to_cog_;
    
    double g_c_, f_g_;
    void setPulleyPoses(Vector3d p1, Vector3d p2, Vector3d p3, Vector3d p4)
    {
      p1_ = p1;
      p2_ = p2;
      p3_ = p3;
      p4_ = p4;
    }
    void setEEAnchors(Vector3d b1, Vector3d b2, Vector3d b3, Vector3d b4)
    {
      b1_ = b1;
      b2_ = b2;
      b3_ = b3;
      b4_ = b4;
    }
    void setCog(Vector3d r_to_cog)
    {
      r_to_cog_ = r_to_cog;
    }
};

std::vector<MatrixXd> inverseKinematicsSolver(CableRobotParams robot_params, Vector3d p_platform, Matrix3d rot_init)
{
  std::vector<MatrixXd> results_list;
  RobotParameters<double> params_;
  //initilize the pulley locations
  params_.pulleys.push_back(robot_params.p1_);
  params_.pulleys.push_back(robot_params.p2_);
  params_.pulleys.push_back(robot_params.p3_);
  params_.pulleys.push_back(robot_params.p4_);

  //initilize cable attachement points
  params_.ef_points.push_back(robot_params.b1_);
  params_.ef_points.push_back(robot_params.b2_);
  params_.ef_points.push_back(robot_params.b3_);
  params_.ef_points.push_back(robot_params.b4_);
  params_.r_to_cog = robot_params.r_to_cog_;
  params_.g_c= robot_params.g_c_;
  params_.f_g = robot_params.f_g_;

  IKDataOut<double> ik_result;
  //Run the first solver to get the end-effector orientation, cable forces, and cable lengths
  ikSolver(p_platform, rot_init, params_, &ik_result);
  //Extract The results and return them as a list of matrices to python
  results_list.push_back(ik_result.rot_platform);
  //Catenary cable lengths
  
  int N = 4;

  VectorXd lc_cat(N);
  lc_cat << ik_result.lc_cat[0], ik_result.lc_cat[1], ik_result.lc_cat[2], ik_result.lc_cat[3];
  results_list.push_back(lc_cat);

  //Cable Forces
  MatrixXd cable_forces(2,N);
  for(int i=0; i<N; i++)
    cable_forces.col(i) =  ik_result.cable_forces[i];
  results_list.push_back(cable_forces);

  //Catenary parameters
  //C1
  VectorXd c1(N);
  for(int i=0; i<N; i++)
    c1 << ik_result.c1[i];
  results_list.push_back(c1);
  //C2
  VectorXd c2(N);
  for(int i=0; i<N; i++)
    c2 << ik_result.c2[i];
  results_list.push_back(c2);
  //body coordinates represented in the world frame
  MatrixXd b_in_w(3,N);
  for(int i=0; i<N; i++)
    b_in_w.col(i) =  ik_result.b_in_w[i];
  results_list.push_back(b_in_w);

  return results_list;
}

std::vector<MatrixXd> forwardKinematicsSolver(CableRobotParams robot_params, VectorXd lc_cat, Vector2d fc_1, Vector3d pos_init, Matrix3d rot_init)
{
  std::vector<MatrixXd> results_list;
  RobotParameters<double> params_;
  //initilize the pulley locations
  params_.pulleys.push_back(robot_params.p1_);
  params_.pulleys.push_back(robot_params.p2_);
  params_.pulleys.push_back(robot_params.p3_);
  params_.pulleys.push_back(robot_params.p4_);
  params_.r_to_cog = robot_params.r_to_cog_;

  //initilize cable attachement points
  params_.ef_points.push_back(robot_params.b1_);
  params_.ef_points.push_back(robot_params.b2_);
  params_.ef_points.push_back(robot_params.b3_);
  params_.ef_points.push_back(robot_params.b4_);
  params_.g_c= robot_params.g_c_;
  params_.f_g = robot_params.f_g_;
  //Run the solver
  FKDataOut<double> fk_results;
  fkSolver(lc_cat.data(), pos_init, rot_init, fc_1, params_, &fk_results);

  results_list.push_back(fk_results.rot_platform);
  results_list.push_back(fk_results.p_platform);

  int N = 4;

  //Cable Forces
  MatrixXd cable_forces(2,N);
  for(int i=0; i<N; i++)
    cable_forces.col(i) =  fk_results.cable_forces[i];
  results_list.push_back(cable_forces);

  //Catenary parameters
  //C1
  VectorXd c1(N);
  for(int i=0; i<N; i++)
    c1 << fk_results.c1[i];
  results_list.push_back(c1);
  //C2
  VectorXd c2(N);
  for(int i=0; i<N; i++)
    c2 << fk_results.c2[i];
  results_list.push_back(c2);
  //body coordinates represented in the world frame
  MatrixXd b_in_w(3,N);
  for(int i=0; i<N; i++)
    b_in_w.col(i) =  fk_results.b_in_w[i];
  results_list.push_back(b_in_w);

  return results_list;
}

namespace py = pybind11;

PYBIND11_MODULE(scampi_ks_solver, m) {
    m.doc() = R"pbdoc(
        Python Bindings for SCAMPI Ceres Numerical Kinematics Solvers
        -----------------------

        .. currentmodule:: scampi_numerical_solvers

        .. autosummary::
           :toctree: _generate

           inverseKinematicsSolver
    )pbdoc";

    m.def("inverseKinematicsSolver", &inverseKinematicsSolver, R"pbdoc(
        Inverse kinematics Solver

        The first solver that take the robot's position and computes cable lenghts
        and force distributions.
    )pbdoc");

    m.def("forwardKinematicsSolver", &forwardKinematicsSolver, R"pbdoc(
        Forward kinematics Solver

        The secon solver that take the calbe lenghts and forces and computes the pose.
    )pbdoc");

    py::class_<CableRobotParams>(m, "CableRobotParams")
        .def(py::init<const double, const double>())
        .def("setPulleyPoses", &CableRobotParams::setPulleyPoses)
        .def("setEEAnchors", &CableRobotParams::setEEAnchors)
        .def("setCog", &CableRobotParams::setCog);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
