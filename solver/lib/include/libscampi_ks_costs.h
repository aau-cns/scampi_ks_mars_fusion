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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "libscampi_ks_data_types.h"
#include <cmath>
#include "libscampi_ks_utils.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>


#ifndef __COSTS_FUNCS__
#define __COSTS_FUNCS__

struct Cat3dFhFvCostFunctor {
    Cat3dFhFvCostFunctor(const RobotState<double> robotstate_,
                         const RobotParameters<double> robotparams_,
                         const Eigen::Matrix<double,3,3> init_rot):
                                  state_(robotstate_), params_(robotparams_), init_rot_(init_rot) {}
      template<typename T>                      
      bool operator()(const T *x, T *residuals) const {
          const T fh1 = x[0];
          const T fv1 = x[1];
          const T rx = x[2];
          const T ry = x[3];
          const T rz = x[4];
          const T gc = (T) params_.g_c;

          Eigen::Matrix<T,3,3> init_R;
          init_R = init_rot_.cast<T>();

          GeometricVariables<T> geom_vars;
          CatenaryVariables<T> cat_vars;
          RobotState<T> state;
          RobotParameters<T> params;

          for(int i=0; i<params_.ef_points.size(); i++)
          {
            params.ef_points.push_back(params_.ef_points[i].cast<T>());
            params.pulleys.push_back(params_.pulleys[i].cast<T>());
          }

          params.f_g = (T)(params_.f_g);
          params.g_c = (T)(params_.g_c);
          params.r_to_cog = params_.r_to_cog.cast<T>();

          state.p_platform = state_.p_platform.cast<T>();

          T rot_data[9];
          ceres::AngleAxisToRotationMatrix<T>(&x[2], rot_data);
          Eigen::Matrix<T,3,3> delta_rot;
          delta_rot << rot_data[0], rot_data[1], rot_data[2], 
                       rot_data[3], rot_data[4], rot_data[5], 
                       rot_data[6], rot_data[7], rot_data[8];

          //The output of the AngleAxisToRotationMatrix is retured as comajor array. We apply a transpose to fix the R
          // Hey Rooholla, I think you;re dealing with the rotation section in a worng way. you need to update at the init guess and should not
          // estimate the absolute R mat using the xi format. I think this is why you're having problems.
          state.rot_platform = init_R * delta_rot.transpose();

          getGeometricVariables<T>(state, params, &geom_vars);
          getCableForces<T>(fh1, fv1, &state, params, geom_vars);
          getCatenaryVariables<T>(state, params, geom_vars,&cat_vars);

          for(int i=0; i < params.ef_points.size(); i++)
          {
            T fh = state.cable_forces[i][0];
            residuals[i] = (T)sqrt(500) * (cat_vars.yl_cat[i] - fh/gc * ( cosh(gc/fh * (cat_vars.length[i] + cat_vars.c1[i])) - cat_vars.c2[i] ));
            residuals[i+4] = (T)sqrt(10) * (cat_vars.lc_cat[i] - (geom_vars.p_in_w[i]-geom_vars.b_in_w[i]).norm());
            residuals[i+8] = (T)exp(-fh);
          }
          return true;
      }

    private:
      const RobotState<double> state_;
      const RobotParameters<double> params_;
      const Eigen::Matrix<double,3,3> init_rot_;
  };


struct Cat3dLc2PoseCostFunctor {
    Cat3dLc2PoseCostFunctor(const double *lc_meas, 
                            const Eigen::Matrix3d init_rot_, 
                            const RobotParameters<double> robotparams_): 
                                     lc_meas_(lc_meas), init_rot(init_rot_), params_(robotparams_){}  
      template<typename T>                      
      bool operator()(const T *x, T *residuals) const {
          const T fh1 = x[0];
          const T fv1 = x[1];
          const T rx = x[2];
          const T ry = x[3];
          const T rz = x[4];
          const T tx = x[5];
          const T ty = x[6];
          const T tz = x[7];

          const T g_c = (T) params_.g_c;

          GeometricVariables<T> geom_vars;
          CatenaryVariables<T> cat_vars;
          RobotState<T> state;
          RobotParameters<T> params;

          for(int i=0; i<params_.ef_points.size(); i++)
          {
            params.ef_points.push_back(params_.ef_points[i].cast<T>());
            params.pulleys.push_back(params_.pulleys[i].cast<T>());
          }
          params.f_g = (T)(params_.f_g);
          params.g_c = (T)(params_.g_c);
          params.r_to_cog = params_.r_to_cog.cast<T>();

          T rot_data[9];
          ceres::AngleAxisToRotationMatrix<T>(&x[2], rot_data);
          Matrix<T,3,3> rot_platform;
          rot_platform << rot_data[0], rot_data[1], rot_data[2], 
                          rot_data[3], rot_data[4], rot_data[5], 
                          rot_data[6], rot_data[7], rot_data[8];
          
          state.p_platform = Eigen::Matrix<T,3,1>(tx, ty, tz);

          Eigen::Matrix<T,3,3> init_rot_ = init_rot.cast<T>();
          //The output of the AngleAxisToRotationMatrix is retured as comajor array. We apply a transpose to fix the R
          state.rot_platform = init_rot_ * rot_platform.transpose();

          getGeometricVariables<T>(state, params, &geom_vars);
          getCableForces<T>(fh1, fv1, &state, params, geom_vars);
          getCatenaryVariables<T>(state, params, geom_vars,&cat_vars);

          for(int i=0; i < geom_vars.b_in_w.size(); i++)
          {
            T fh = state.cable_forces[i][0];
            residuals[i] = (T)sqrt(10) * (cat_vars.yl_cat[i] - fh/g_c * ( cosh(g_c/fh * (cat_vars.length[i] + cat_vars.c1[i])) - cat_vars.c2[i] ));
            residuals[i+4] = (T)sqrt(10) * (cat_vars.lc_cat[i] - (T)lc_meas_[i]);
          }
          return true;
      }

    private:
      const double *lc_meas_;
      const RobotParameters<double> params_;
      const Eigen::Matrix<double,3,3> init_rot;
  };

#endif
