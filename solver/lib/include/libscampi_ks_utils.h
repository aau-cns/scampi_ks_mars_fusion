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

#ifndef LIB_SCAMPI_KS_UTILS
#define LIB_SCAMPI_KS_UTILS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "libscampi_ks_data_types.h"
#include <cmath>
#include "ceres/jet.h"

using namespace Eigen;
using namespace std;
using std::sinh;
using std::cosh;
using std::asinh;
using std::acosh;
using ceres::Jet;

// asinh(a + h) ~= asinh(a) + 1/cosh(asinh(a)) h
template <typename T, int N>
inline Jet<T, N> asinh(const Jet<T, N>& f) {
  return Jet<T, N>(asinh(f.a), 1/cosh(asinh(f.a)) * f.v);
}

// acosh(a + h) ~= acosh(a) + 1/sinh(acosh(a)) h
template <typename T, int N>
inline Jet<T, N> acosh(const Jet<T, N>& f) {
  return Jet<T, N>(acosh(f.a), 1/sinh(acosh(f.a)) * f.v);
}

// Compute the geometric variables based on the robot's state and parameters
template<typename T>
void getGeometricVariables(RobotState<T> robotstate_, 
                           RobotParameters<T> robotparams_,
                           GeometricVariables<T> *geometric_vars)
{
    Matrix<T, 3, 1> sx, rot_b, b_in_w;
    geometric_vars->b_rot.clear();
    geometric_vars->b_in_w.clear();
    geometric_vars->sx.clear();
    for(int i=0; i < robotparams_.pulleys.size(); i++)
    {
        //Rotate the body attachment points into the world frame
        rot_b = robotstate_.rot_platform*robotparams_.ef_points[i];
        geometric_vars->b_rot.push_back(rot_b);
        //translate the rotated body point to the end-effector location
        b_in_w = rot_b + robotstate_.p_platform;
        geometric_vars->b_in_w.push_back(b_in_w);
        //compute the x-y projection of the direction from end-effector attachment point to the pulley
        sx = (b_in_w-robotparams_.pulleys[i]);
        sx[2] = (T) 0;
        geometric_vars->sx.push_back(sx/sx.norm());

        geometric_vars->p_in_w.push_back(robotparams_.pulleys[i]);
    }
    geometric_vars->r_to_cog = robotstate_.rot_platform * robotparams_.r_to_cog;
}


// Compute the catenary variables from the robot's state and geometric parameters
template<typename T>
void getCatenaryVariables(RobotState<T> robotstate_, 
                             RobotParameters<T> robotparams_,
                             GeometricVariables<T> geometric_vars,
                             CatenaryVariables<T> *cat_vars)
{
    T L, C1, C2, lc_cat, y0_cat, yl_cat;
    T gc = (T) robotparams_.g_c; //for the sake of clarity

    cat_vars->length.clear();
    cat_vars->c1.clear();
    cat_vars->c2.clear();
    cat_vars->lc_cat.clear();
    cat_vars->y0_cat.clear();
    cat_vars->yl_cat.clear();

    for(int i=0; i < robotparams_.pulleys.size(); i++)
    {
        //Compute projection length of the vector from the body to the pulley on the x-y plane
        Map<Matrix<T, -1, 1>> p(geometric_vars.p_in_w[i].data(),2);
        Map<Matrix<T, -1, 1>> b(geometric_vars.b_in_w[i].data(),2);

        L = (p - b).norm();
        cat_vars->length.push_back(L);

        //Compute the C1 paramter
        T fh = robotstate_.cable_forces[i][0];
        T fv = robotstate_.cable_forces[i][1];
        C1 = fh/gc*asinh( -fv/fh) - L;
        cat_vars->c1.push_back(C1);

        //Extract the Z components of the puleys and body attachment points
        y0_cat = geometric_vars.p_in_w[i][2];
        yl_cat = geometric_vars.b_in_w[i][2];
        cat_vars->y0_cat.push_back(y0_cat);
        cat_vars->yl_cat.push_back(yl_cat);

        //compute the C2 paramter
        C2 = cosh(C1*gc/fh) - gc/fh*y0_cat;
        cat_vars->c2.push_back(C2);

        //Compute the true cable length
        lc_cat = fh/gc*( sinh((gc/fh)*(L + C1)) - sinh((gc/fh)*C1));
        cat_vars->lc_cat.push_back(lc_cat);

    }

}

// A function to make the A matrix for the getCableForces function
template<typename T>
void constructStructuralMat(std::vector<Eigen::Matrix<T,3,1>> sx, 
                            std::vector<Eigen::Matrix<T,3,1>> b_rot, 
                            Eigen::Matrix<T,-1,-1> *structure_matrix_res)
{
    Matrix<T,3,1> const_vec((T)0,(T)0,(T)1);
    Matrix<T,-1,-1> A(6,8);

    for(int i=0; i < sx.size(); i++)
    {
        A.block(0, 2*i, 3,1) = -sx[i];
        A.block(3, 2*i, 3,1) = -b_rot[i].cross(sx[i]);
        A.block(0, 2*i + 1, 3,1) = const_vec;
        A.block(3, 2*i + 1, 3,1) = b_rot[i].cross(const_vec);
    }
    *structure_matrix_res = A;
}


// Compute the catenary variables from the robot's state and geometric parameters
template<typename T>
void getCableForces(T fh, T fv,
                      RobotState<T> *robotstate_,
                      RobotParameters<T> robotparams_,
                      GeometricVariables<T> geometric_vars)
{
    Matrix<T, -1, -1> structure_matrix(6,8);
    //The wrench applied to the end-effector
    Matrix<T, -1, 1> platform_wrench(6);
    platform_wrench << (T)0.,   (T)0.,  (T)robotparams_.f_g, (T)0.,   (T)0., (T)0.;
    // The cog modification recently added to the matlab implementation for the real data expriment
    Matrix<T,3,1> const_vec((T)0,(T)0,(T)-1);
    platform_wrench.block(3,0,3,1) = - geometric_vars.r_to_cog.cross(const_vec)*(T)(2.8*9.81);

    // End of cog modification
    Matrix<T, 2, 1> f_v(fh, fv);

    constructStructuralMat(geometric_vars.sx, geometric_vars.b_rot, &structure_matrix);

    //a subset of the structure matrix that defines the force in other cables as a function of forces of the first cable
    Matrix<T, -1, -1> sub_structure_matrix = structure_matrix.block(0,2, 6,6);
    Matrix<T, -1, 1> other_forces = sub_structure_matrix.inverse()*(platform_wrench - structure_matrix.block(0,0, 6,2)*f_v.block(0,0,2,1));
    Matrix<T, -1, 1> cable_forces(8);
    cable_forces.block(0,0,2,1) = f_v;
    cable_forces.block(2,0,6,1) = other_forces;

    Matrix<T, 3, 1> F = structure_matrix.block(0,0,3,8) * cable_forces - Matrix<T,3,1>((T)0, (T)0, robotparams_.f_g);
    Matrix<T, 3, 1> Torque = structure_matrix.block(3,0,3,8) * cable_forces;
    robotstate_->wrench.block(0,0,3,1) = F;
    robotstate_->wrench.block(3,0,3,1) = Torque;

    robotstate_->cable_forces_compact = cable_forces;
    for(int i=0; i < geometric_vars.sx.size(); i++)
        robotstate_->cable_forces.push_back(cable_forces.block(2*i,0,2,1));
}

// A function for the IK solver that puts the cable with the larges length as the first cable for numerical stability

template<typename T>
void changeOrderForSolver(RobotState<T> state, 
                          RobotParameters<T> params,
                          RobotParameters<T> *params_reordered, 
                          Eigen::VectorXi *new_order)
{
    int N = params.pulleys.size();
    VectorXi order(N);

    double distances[N];
    double max_distance, distance = 0;
    int max_index = 0;
    VectorXi indeces(N);
    for(int i=0; i < N; i++)
    {
        indeces[i] = i;
        distance = (params.pulleys[i] - state.p_platform).norm();
        if (distance > max_distance)
        {
            max_distance = distance;
            max_index = i;
        }
    }

    //compute the reorder index map
    order.block(0,0,N - max_index,1) = indeces.block(max_index,0, N-max_index,1);
    order.block(N - max_index,0,max_index,1) = indeces.block(0,0,max_index,1);
    //return a reordered params data structure
    *params_reordered = params;

    params_reordered->ef_points.clear();
    params_reordered->pulleys.clear();

    for(int i=0; i < N; i++)
    {
        params_reordered->pulleys.push_back(params.pulleys[order[i]]);
        params_reordered->ef_points.push_back(params.ef_points[order[i]]);
    }
    *new_order = order;
}

//Return the order of the cables back to the normal configuration
template<typename T>
void reverseOrderForSolver(RobotState<T> robotstate_,
                           GeometricVariables<T> geometric_vars,
                           CatenaryVariables<T> cat_vars,
                           IKDataOut<T> *fixed_cat_out,
                           Eigen::VectorXi order)
{
    int N = geometric_vars.b_in_w.size();
    VectorXi reorder_idx(N);
    for(int i=0; i < N; i++)
        reorder_idx[order[i]] = i;

    for(int i=0; i < N; i++)
    {
        fixed_cat_out->b_in_w.push_back(geometric_vars.b_in_w[reorder_idx[i]]);
        fixed_cat_out->c1.push_back(cat_vars.c1[reorder_idx[i]]);
        fixed_cat_out->c2.push_back(cat_vars.c2[reorder_idx[i]]);
        fixed_cat_out->cable_forces.push_back(robotstate_.cable_forces[reorder_idx[i]]);
        fixed_cat_out->lc_cat.push_back(cat_vars.lc_cat[reorder_idx[i]]);
    }
}

// Compute an initial value for the cable forces for the IK solver
template<typename T>
void computeInitCableForces(T *fh0, 
                            T *fv0, 
                            Eigen::Matrix<T,3,1> p_platform, 
                            Eigen::Matrix<T,3,3> rot_platform, 
                            RobotParameters<T> robotparams_)
{
    Matrix<T, 3, 1> b_w = p_platform + rot_platform*robotparams_.ef_points[0];
    Matrix<T, 3, 1> sx = b_w - robotparams_.pulleys[0];
    sx[2] = 0;
    sx = sx/sx.norm();
    double alpha = acos((sx.transpose()*(b_w-robotparams_.pulleys[0]))[0]/(b_w-robotparams_.pulleys[0]).norm());
    *fv0 = 1*(-robotparams_.f_g)/4.0;
    *fh0 = abs(*fv0)/tan(alpha);
}




#endif
