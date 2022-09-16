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
#include <random>

#include <ceres/ceres.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "libscampi_ks_costs.h"
// #include "libscampi_numeric_ks_solvers.h"


using namespace ceres;
using namespace Eigen;
using namespace std;


// Define the test fixture
class CostsTest : public ::testing::Test {
 protected:
  void SetUp() override {

    //initilize the pulley locations
    params_.pulleys.push_back(Vector3d(0, 100, 50));
    params_.pulleys.push_back(Vector3d(200, 100, 50));
    params_.pulleys.push_back(Vector3d(200, 0, 50));
    params_.pulleys.push_back(Vector3d(0, 0, 50));

    //initilize cable attachement points
    params_.ef_points.push_back(Vector3d(-0.1, 0.1, 0));
    params_.ef_points.push_back(Vector3d(0.1, 0.1, 0));
    params_.ef_points.push_back(Vector3d(0.1, -0.1, 0));
    params_.ef_points.push_back(Vector3d(-0.1, -0.1, 0));
    params_.g_c= 0.144787774431201;
    params_.f_g = 353.16;

    //initilize robot's state
    state_.p_platform = Vector3d(100,50,0);
    state_.rot_platform = Matrix3d::Identity();

    state_.cable_forces.push_back(Vector2d(19.7186,    8.8290));
    state_.cable_forces.push_back(Vector2d(19.7186,  167.7510));
    state_.cable_forces.push_back(Vector2d(19.7186,    8.8290));
    state_.cable_forces.push_back(Vector2d(19.7186,  167.7510));

    //initilize the geometric variables
     geometric_vars_.p_in_w.push_back(Vector3d(0, 100, 50));
     geometric_vars_.p_in_w.push_back(Vector3d(200, 100, 50));
     geometric_vars_.p_in_w.push_back(Vector3d(200, 0, 50));
     geometric_vars_.p_in_w.push_back(Vector3d(0, 0, 50));

     geometric_vars_.b_in_w.push_back(Vector3d(99.9000,   50.1000,         0));
     geometric_vars_.b_in_w.push_back(Vector3d(100.1000,   50.1000,         0));
     geometric_vars_.b_in_w.push_back(Vector3d(100.1000,   49.9000,         0));
     geometric_vars_.b_in_w.push_back(Vector3d(99.9000,   49.9000,         0));

    geometric_vars_.b_rot.push_back(Vector3d(-0.1, 0.1, 0));
    geometric_vars_.b_rot.push_back(Vector3d(0.1, 0.1, 0));
    geometric_vars_.b_rot.push_back(Vector3d(0.1, -0.1, 0));
    geometric_vars_.b_rot.push_back(Vector3d(-0.1, -0.1, 0));

    geometric_vars_.sx.push_back(Vector3d(0.8946,   -0.4469,        0));
    geometric_vars_.sx.push_back(Vector3d(-0.8946,  -0.4469,        0));
    geometric_vars_.sx.push_back(Vector3d(-0.8946,   0.4469,        0));
    geometric_vars_.sx.push_back(Vector3d(0.8946,    0.4469,        0));

    catenary_vars_.c1.push_back(-1.707751241777864e+02);
    catenary_vars_.c1.push_back(-4.981071636175924e+02);
    catenary_vars_.c1.push_back(-1.707751241777863e+02);
    catenary_vars_.c1.push_back(-4.981071636175924e+02);

    catenary_vars_.c2.push_back(1.527635896655149);
    catenary_vars_.c2.push_back(19.027144502783933);
    catenary_vars_.c2.push_back(1.527635896655148);
    catenary_vars_.c2.push_back(19.027144502783948);

    catenary_vars_.length.push_back(1.116692437513571e+02);
    catenary_vars_.length.push_back(1.116692437513571e+02);
    catenary_vars_.length.push_back(1.116692437513571e+02);
    catenary_vars_.length.push_back(1.116692437513571e+02);

    catenary_vars_.yl_cat.push_back(0);
    catenary_vars_.yl_cat.push_back(0);
    catenary_vars_.yl_cat.push_back(0);
    catenary_vars_.yl_cat.push_back(0);

    catenary_vars_.lc_cat.push_back(1.582039057684239e+02);
    catenary_vars_.lc_cat.push_back(1.479182118208436e+03);
    catenary_vars_.lc_cat.push_back(1.582039057684239e+02);
    catenary_vars_.lc_cat.push_back(1.479182118208437e+03);

  }

  // void TearDown() override {}

  RobotParameters<double> params_;
  RobotState<double> state_;
  GeometricVariables<double> geometric_vars_;
  CatenaryVariables<double> catenary_vars_;
};

TEST_F(CostsTest, numerical_inverse_cost_works)
{
  Matrix3d init_rot = Matrix3d::Identity();
  
  CostFunction* cost_function =
  new AutoDiffCostFunction<Cat3dFhFvCostFunctor, 12, 5>(
    new Cat3dFhFvCostFunctor(state_, params_, init_rot) );

  double x[5] = {19.7186,    8.8290,         0,         0,         0};

  // Build the problem.
  Problem problem;
  problem.AddResidualBlock(cost_function, nullptr, x);

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  cout << x[0] << endl;
  cout << x[1] << endl;
  cout << x[2] << endl;
  cout << x[3] << endl;
  cout << x[4] << endl;

  VectorXd x_ceres = Map<VectorXd,Unaligned>(x,5);
  VectorXd x_matlab(5);
  x_matlab <<   216.8598,   88.2845,   -0.0000,    0.0000,   -0.0000;
  double error = (x_ceres - x_matlab).norm();
  EXPECT_NEAR(error, 0 , 0.1);
  cout << error << endl;

}

TEST_F(CostsTest, numerical_forward_cost_works)
{
  const double lc_meas_[4] = {122.3755,  122.3756,  122.3756,  122.3756};
  Matrix3d init_R_;
  init_R_ << 1.0000,    0.0061,    0.0012,
            -0.0061,    1.0000,   -0.0037,
            -0.0013,    0.0037,    1.0000;

  CostFunction* cost_function =
  new AutoDiffCostFunction<Cat3dLc2PoseCostFunctor, 8, 8>(
    new Cat3dLc2PoseCostFunctor(lc_meas_, init_R_, params_) );
    

  double x[8] = {216.86,    88.2548,         0,         0,         0, 99.9721,   50.0685,    0.0010};

  // Build the problem.
  Problem problem;
  problem.AddResidualBlock(cost_function, nullptr, x);

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  cout << x[0] << endl;
  cout << x[1] << endl;
  cout << x[2] << endl;
  cout << x[3] << endl;
  cout << x[4] << endl;
  cout << x[5] << endl;
  cout << x[6] << endl;
  cout << x[7] << endl;

  VectorXd x_ceres = Map<VectorXd, Unaligned>(x, 8);
  VectorXd x_matlab(8);
  x_matlab << 216.8570,   88.2911,   -0.0037,   -0.0012,    0.0061,   99.9996,   50.0010,   -0.0025;
  double error = (x_ceres - x_ceres).norm();
  EXPECT_NEAR(error, 0., 0.01);
}
