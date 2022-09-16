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
#include "libscampi_ks_utils.h"


using namespace ceres;
using namespace Eigen;
using namespace std;

// Define the test fixture
class HelpersTest : public ::testing::Test {
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
    params_.g_c= 0.1448;
    params_.f_g = 353.16;
    params_.r_to_cog = Vector3d(0., 0., -0.12);

    //initilize robot's state
    state_.p_platform = Vector3d(100,50,0);
    state_.rot_platform = Matrix3d::Identity();

    state_.cable_forces.push_back(Vector2d(19.718555061614637,    8.829000000000000));
    state_.cable_forces.push_back(Vector2d(19.718555061614644,  1.677510000000000e+02));
    state_.cable_forces.push_back(Vector2d(19.718555061614637,    8.828999999999997));
    state_.cable_forces.push_back(Vector2d(19.718555061614640,  1.677510000000000e+02));

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
    geometric_vars_.r_to_cog = Vector3d(0., 0., -0.12);

    catenary_vars_.c1.push_back(-170.7751);
    catenary_vars_.c1.push_back(-498.1072);
    catenary_vars_.c1.push_back(-170.7751);
    catenary_vars_.c1.push_back(-498.1072);

    catenary_vars_.c2.push_back(1.5276);
    catenary_vars_.c2.push_back(19.0271);
    catenary_vars_.c2.push_back(1.5276);
    catenary_vars_.c2.push_back(19.0271);

    catenary_vars_.length.push_back(111.6692);
    catenary_vars_.length.push_back(111.6692);
    catenary_vars_.length.push_back(111.6692);
    catenary_vars_.length.push_back(111.6692);

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

// Tests for the getGeometricVariables function
TEST_F(HelpersTest, getGeometricVariables_works)
{
  getGeometricVariables<double>(state_, params_, &geometric_vars_);
  // ToDo: Make the tesets for comprehensive
  double error = (geometric_vars_.sx[0] - Vector3d(0.8946, -0.4469, 0)).norm();
  EXPECT_NEAR(error, 0, 1e-2);
}

// Tests for the getCatenaryVariables function
TEST_F(HelpersTest, getCatenaryVariables_works)
{
  RobotState<double> state;
  state.p_platform = Vector3d(0.0402,   -1.1387,    3.9743);
  state.rot_platform << 0.9853,    0.1185,    0.1233,
                       -0.1134,    0.9924,   -0.0476,
                       -0.1280,    0.0329,    0.9912;

  state.cable_forces.push_back(Vector2d(17.5558,    10.7910));
  state.cable_forces.push_back(Vector2d(10.9148,  8.3917));
  state.cable_forces.push_back(Vector2d(21.8973,    15.1257));
  state.cable_forces.push_back(Vector2d(14.8614,  8.8557));
  RobotParameters<double> params;
  params.pulleys.push_back(Vector3d(2.5204,   -8.3888,    8.4693));
  params.pulleys.push_back(Vector3d(2.7181,    4.7754,    8.3642));
  params.pulleys.push_back(Vector3d(-1.7967,    4.8335,    8.3701));
  params.pulleys.push_back(Vector3d(-1.9875,   -8.3197,    8.4718));
  params.g_c = 0.1035;
  params.ef_points.push_back(Vector3d(0.2100,    -0.2100,   -0.0110));
  params.ef_points.push_back(Vector3d(0.2100,    0.2100,   -0.0110));
  params.ef_points.push_back(Vector3d(-0.2100,   0.2100,   -0.0110));
  params.ef_points.push_back(Vector3d(-0.2100,   -0.2100,   -0.0110));
  
  GeometricVariables<double> geom_vars;
  getGeometricVariables(state,params, &geom_vars);
  
  CatenaryVariables<double> catenary_vars;
  getCatenaryVariables<double>(state, params, geom_vars, &catenary_vars);
  
  cout << endl << catenary_vars.c1[3];
  cout << endl << catenary_vars.c2[3];
  cout << endl << catenary_vars.lc_cat[3];
  cout << endl << catenary_vars.length[3];
  cout << endl << catenary_vars.y0_cat[3];
  cout << endl << catenary_vars.yl_cat[3];
  // ToDo: Make the tesets for comprehensive
  // EXPECT_NEAR(catenary_vars_.c1[0], -170.7751, 1e-2);
  // EXPECT_NEAR(catenary_vars_.c2[0],  1.5276, 1e-2);
  // EXPECT_NEAR(catenary_vars_.y0_cat[0],   50, 1e-2);
  // EXPECT_NEAR(catenary_vars_.yl_cat[0],   0, 1e-2);
  // EXPECT_NEAR(catenary_vars_.lc_cat[0],   0.1582e3, 1e-2);
}

// Tests for the getCableForces function
TEST_F(HelpersTest, constructStructuralMat_works)
{
  MatrixXd structure_matrix(6,8);
  MatrixXd structure_matrix_matlab(6,8);
  structure_matrix_matlab << -0.894606219617978,0,0.894606219617978,0,0.894606219617978,0,-0.894606219617978,0,
                              0.446855358948319,	0,	0.446855358948319,	0,	-0.446855358948319,	0,	-0.446855358948319,	0,
                              0,	                1,	0,	                1,	 0,	                1,	 0,	                1,
                              0,	0.100000000000000,	0,	0.100000000000000,	0,	-0.100000000000000,	0,	-0.100000000000000,
                              0,	0.100000000000000,	0,	-0.100000000000000,	0,	-0.100000000000000,	0,	0.100000000000000,
                              0.0447750860669659,	0,	-0.0447750860669659,	0,	0.0447750860669659,	0,	-0.0447750860669659, 0;

  constructStructuralMat<double>(geometric_vars_.sx, geometric_vars_.b_rot, &structure_matrix);
  double error = (structure_matrix - structure_matrix_matlab).norm();
  EXPECT_NEAR(error, 0, 1e-3);
}

// Tests for the getCableForces function
TEST_F(HelpersTest, getCableForces)
{   RobotState<double> state;
    state.p_platform = Vector3d(0.0402,   -1.1387,    3.9743);
    state.rot_platform << 0.9853,    0.1185,    0.1233,
                        -0.1134,    0.9924,   -0.0476,
                        -0.1280,    0.0329,    0.9912;

    state.cable_forces.push_back(Vector2d(17.5558,    10.7910));
    state.cable_forces.push_back(Vector2d(10.9148,  8.3917));
    state.cable_forces.push_back(Vector2d(21.8973,    15.1257));
    state.cable_forces.push_back(Vector2d(14.8614,  8.8557));
    RobotParameters<double> params;
    params.pulleys.push_back(Vector3d(2.5204,   -8.3888,    8.4693));
    params.pulleys.push_back(Vector3d(2.7181,    4.7754,    8.3642));
    params.pulleys.push_back(Vector3d(-1.7967,    4.8335,    8.3701));
    params.pulleys.push_back(Vector3d(-1.9875,   -8.3197,    8.4718));
    params.g_c = 0.1035;
    params.f_g = 43.1640;

    params.ef_points.push_back(Vector3d(0.2100,    -0.2100,   -0.0110));
    params.ef_points.push_back(Vector3d(0.2100,    0.2100,   -0.0110));
    params.ef_points.push_back(Vector3d(-0.2100,   0.2100,   -0.0110));
    params.ef_points.push_back(Vector3d(-0.2100,   -0.2100,   -0.0110));
    params.r_to_cog = Vector3d(0.0,    0.0,   -0.12);
    VectorXd f_vec_matlab(8);
    

    //Run the function
    GeometricVariables<double> geom_vars;
    getGeometricVariables(state,params, &geom_vars);
    getCableForces<double>(17.5558, 10.7910, &state, params, geom_vars);
    cout << endl << state.cable_forces_compact.transpose()<<endl;

    f_vec_matlab << 12.6319,   11.5458,   12.6319,   10.0362,   12.6319,   11.5458,   12.6319,   10.0362;

    // //Cable force error
    // double error = (f_vec_matlab - state_.cable_forces_compact).norm();
    // EXPECT_NEAR(error, 0, 1e-3);
    // //Wrench Error
    // VectorXd wrench_matlab(6);
    // wrench_matlab << 0,0,0, 0,0,0;
    // error = (wrench_matlab - state_.wrench).norm();
    // EXPECT_NEAR(error, 0, 1e-3);
}

// Tests for the reordering function

TEST_F(HelpersTest, changeOrderForSolver_function_works)
{
    VectorXi reordered_idx(4);
    RobotParameters<double> reorderedparams_;
    
    RobotState<double> state;
    state.p_platform = Vector3d(0.0402,   -1.1387,    3.9743);
    state.rot_platform << 0.9853,    0.1185,    0.1233,
                        -0.1134,    0.9924,   -0.0476,
                        -0.1280,    0.0329,    0.9912;

    state.cable_forces.push_back(Vector2d(17.5558,    10.7910));
    state.cable_forces.push_back(Vector2d(10.9148,  8.3917));
    state.cable_forces.push_back(Vector2d(21.8973,    15.1257));
    state.cable_forces.push_back(Vector2d(14.8614,  8.8557));
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

    changeOrderForSolver<double>(state, params, &reorderedparams_, &reordered_idx);

    cout << endl<<"Printing the reorderd params..."<<endl;
    cout << endl<<"Reordered EF Points..."<<endl;
    for(int i=0; i<4; i++)
        cout << endl << reorderedparams_.ef_points[i].transpose();

    cout << endl<<"Reordered Pulley Points..."<<endl;
    for(int i=0; i<4; i++)
        cout << endl << reorderedparams_.pulleys[i].transpose();
    cout << endl;
    
    IKDataOut<double> reordered;
    catenary_vars_.c1.clear();
    for(int i=0; i<4; i++)
      catenary_vars_.c1.push_back(i);
    reverseOrderForSolver<double>(state,geometric_vars_,catenary_vars_, &reordered, reordered_idx);
    cout << endl<<"Reordered Pulley Points..."<<endl;
    for(int i=0; i<4; i++)
        cout << endl << reordered.c1[i];
    cout << endl;
}

TEST_F(HelpersTest, reverseOrderForSolver_function_works)
{
    VectorXi order(4);
    order << 1,2,3,0;
    IKDataOut<double> reordered;
    reverseOrderForSolver<double>(state_,geometric_vars_,catenary_vars_, &reordered, order);
}

TEST_F(HelpersTest, compute_initial_cable_lens_function_works)
{
    Matrix3d R_init;
    Vector3d p_platform(0.0402,   -1.1387,    3.9743);
    R_init << 0.9853,    0.1185,    0.1233,
                       -0.1134,    0.9924,   -0.0476,
                       -0.1280,    0.0329,    0.9912;

    RobotParameters<double> params;
    params.pulleys.push_back(Vector3d(2.5204,   -8.3888,    8.4693));
    params.pulleys.push_back(Vector3d(2.7181,    4.7754,    8.3642));
    params.pulleys.push_back(Vector3d(-1.7967,    4.8335,    8.3701));
    params.pulleys.push_back(Vector3d(-1.9875,   -8.3197,    8.4718));
    params.g_c = 0.1035;
    params.f_g = 43.1640;

    params.ef_points.push_back(Vector3d(0.2100,    -0.2100,   -0.0110));
    params.ef_points.push_back(Vector3d(0.2100,    0.2100,   -0.0110));
    params.ef_points.push_back(Vector3d(-0.2100,   0.2100,   -0.0110));
    params.ef_points.push_back(Vector3d(-0.2100,   -0.2100,   -0.0110));
    params.r_to_cog = Vector3d(0.0,    0.0,   -0.12);
    
    double fh0, fv0;
    computeInitCableForces<double>(&fh0, &fv0, p_platform, R_init, params);
    cout << endl << "Test" << endl;
    cout << fh0 << '\t' << fv0 << endl;
    // EXPECT_NEAR(fv0, -8.829, 0.01);
    // EXPECT_NEAR(fh0, 19.7186, 0.01);
}
