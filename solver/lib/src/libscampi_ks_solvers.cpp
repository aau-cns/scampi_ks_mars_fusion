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

#include "libscampi_ks_solvers.h"
#include "libscampi_ks_utils.h"
#include "libscampi_ks_costs.h"

using namespace ceres;
using namespace Eigen;
using namespace std;

void ikSolver(Vector3d p_platform, 
              Matrix3d rot_init, 
              RobotParameters<double> params,
              IKDataOut<double> *result)
{
    //reorder the cable forces and choose the cable with largest length as the the first cable (for numerical stability)
    VectorXi reorder_idx(params.pulleys.size());
    RobotParameters<double> params_reord;
    RobotState<double> state;
    state.rot_platform = rot_init;
    state.p_platform = p_platform;
    changeOrderForSolver<double>(state, params, &params_reord, &reorder_idx);

    //Compute initil cable forces as starting points for the solver
    double fh0, fv0;
    computeInitCableForces<double>(&fh0, &fv0, p_platform, rot_init, params_reord);

    //Run the Ceres to optimize for the cable forces and robot's orientation

    CostFunction* cost_function =
        new AutoDiffCostFunction<Cat3dFhFvCostFunctor, 12, 5>(
            new Cat3dFhFvCostFunctor(state, params_reord, rot_init) );

    double x[5] = {fh0,    -fv0,   -0.0000,   -0.0000,   -0.0000};

    // Build the problem.
    Problem problem;
    problem.AddResidualBlock(cost_function, nullptr, x);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    // options.num_threads = 4;
    // options.function_tolerance = 0.04;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    //harvest the results
    double fh = x[0]; //optimized horizontal force for the first cable
    double fv = x[1]; //optimized vertical force for the first cable
    // Extract the optimized orientation matrix of the moving platform
    manif::SO3Tangentd xi;
    xi << x[2], x[3], x[4];
    manif::SO3 rot_optimized = manif::exp(xi);
    Matrix3d rot_result = rot_init * rot_optimized.rotation();
    result->rot_platform = rot_result;

    // Use the utils functions once again to compute the force in other cables and the catenary variables
    GeometricVariables<double> geom_vars;
    CatenaryVariables<double> cat_vars;

    state.rot_platform = rot_result; // Now we know the optimzed value of end-effector position so just use it

    getGeometricVariables<double>(state,params_reord,&geom_vars);
    getCableForces<double>(fh, fv, &state, params_reord,geom_vars);
    getCatenaryVariables<double>(state,params_reord, geom_vars,&cat_vars);
    //reverse the order of cables back to the normal configuration
    reverseOrderForSolver<double>(state, geom_vars, cat_vars, result, reorder_idx);
}

void fkSolver(double *lc_cat, 
              Vector3d pos_init,  
              Matrix3d rot_init, Vector2d fc0, 
              RobotParameters<double> params,
              FKDataOut<double> *result)
{
    //Ginve the cable length (lc_cat), initial translation and orientation (pos_init, rot_init) and the robot's parameters
    //Runs the FK solver to get the cable forces, and optimize end-effector pose
    CostFunction* cost_function =
        new AutoDiffCostFunction<Cat3dLc2PoseCostFunctor, 8, 8>(
            new Cat3dLc2PoseCostFunctor(lc_cat, rot_init, params) );

    double x[8] = {fc0[0],    fc0[1],         0,         0,         0, pos_init[0],   pos_init[1],    pos_init[2]};

    // Build the problem.
    Problem problem;
    problem.AddResidualBlock(cost_function, nullptr, x);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    // options.num_threads = 4;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    result->p_platform << x[5], x[6], x[7];

    manif::SO3Tangentd xi;
    xi << x[2], x[3], x[4];
    manif::SO3 rot_optimized = manif::exp(xi);
    Matrix3d rot_result = rot_init * rot_optimized.rotation();
    result->rot_platform = rot_result;

    GeometricVariables<double> geom_vars;
    CatenaryVariables<double> cat_vars;
    RobotState<double> state;
    state.rot_platform = rot_result;
    state.p_platform = result->p_platform;

    double fh = x[0];
    double fv = x[1];

    getGeometricVariables<double>(state,params,&geom_vars);
    getCableForces<double>(fh, fv, &state, params,geom_vars);
    getCatenaryVariables<double>(state,params, geom_vars,&cat_vars);

    result->c1 = cat_vars.c1;
    result->c2 = cat_vars.c2;
    result->lc_cat = cat_vars.lc_cat;
    result->cable_forces = state.cable_forces;
    result->b_in_w = geom_vars.b_in_w;
}
