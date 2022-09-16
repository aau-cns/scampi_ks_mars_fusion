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
#include "libscampi_ks_solvers.h"


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

// Put the two solvers back to back and check if you can get the
//input of the first solver from the output of the second
TEST_F(CostsTest, back_to_back_nuerical_solvers_work)
{
  IKDataOut<double> ik_result;

  //The initl guess for the platform orientation
  Matrix3d R_init;

  //The center point (ef position as the input to the IK solver)
  Vector3d r_p(0.0402,   -1.1387,    3.9743);

  R_init << 0.9853,    0.1185,    0.1233,
            -0.1134,    0.9924,   -0.0476,
            -0.1280,    0.0329,    0.9912;

  //Run the first solver to get the end-effector orientation, cable forces, and cable lengths
  
  RobotParameters<double> params;
  params.pulleys.push_back(Vector3d(-1.9875,   -8.3197,    8.4718));
  params.pulleys.push_back(Vector3d(2.5204,   -8.3888,    8.4693));
  params.pulleys.push_back(Vector3d(2.7181,    4.7754,    8.3642));
  params.pulleys.push_back(Vector3d(-1.7967,    4.8335,    8.3701));
  params.g_c = 0.1035;
  params.f_g = 43.1640;

  params.ef_points.push_back(Vector3d(-0.2100,   -0.2100,   -0.0110));
  params.ef_points.push_back(Vector3d(0.2100,    -0.2100,   -0.0110));
  params.ef_points.push_back(Vector3d(0.2100,    0.2100,   -0.0110));
  params.ef_points.push_back(Vector3d(-0.2100,   0.2100,   -0.0110));
  params.r_to_cog = Vector3d(0.0,    0.0,   -0.12);

  ikSolver(r_p, R_init, params, &ik_result);

  //Perturb the center point and orientation and use it as the intil guess for the FK solver
  // Vector3d r_plat_0 = r_p + Vector3d::Random()*0.1; // Perturb the center point

  // // Perturb the orientation
  // Vector3d xi_rand = Vector3d::Random();
  // xi_rand = xi_rand*0.5*3.1415/180;
  // manif::SO3Tangentd xi;
  // xi << xi_rand[0], xi_rand[1], xi_rand[2];
  // manif::SO3 R_rand = manif::exp(xi);
  // Matrix3d R_0 = R_init * R_rand.rotation();

  // //Run the solver
  // FKDataOut<double> fk_results;
  // fkSolver(ik_result.lc_cat.data(),r_plat_0,R_0, ik_result.cable_forces[0],params_, &fk_results);

  // //print the resutls to see what's what
  // cout << endl << "Translation";
  // cout << endl << fk_results.p_platform.transpose();
  // cout << endl << r_p.transpose() << endl;
  // cout << endl << "Orientation";
  // cout << endl << fk_results.rot_platform;
  // cout << endl << ik_result.rot_platform << endl;
}

// TEST_F(CostsTest, forward_solver_works)
// {
//   double lc_meas_[4] = {122.3755,  122.3756,  122.3756,  122.3756};
//   Matrix3d init_rot_;
//   init_rot_ << 1.0000,    0.0061,    0.0012,
//             -0.0061,    1.0000,   -0.0037,
//             -0.0013,    0.0037,    1.0000;

//   Vector3d r_plat_0(99.9721,   50.0685,    0.0010);
//   Vector2d fc0(216.86,    88.2548);
    
//   //Run the solver
//   FKDataOut<double> fk_results;
//   fkSolver(lc_meas_ ,r_plat_0, init_rot_, fc0, params_, &fk_results);

//   cout << endl << "Translation";
//   cout << endl << fk_results.p_platform.transpose() << endl;
//   double error = (fk_results.p_platform - Vector3d(100,50,0)).norm();
//   EXPECT_NEAR(error, 0., 1e-3);
// }
