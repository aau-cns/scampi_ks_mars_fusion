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

#ifndef LIB_SCAMPI_KS_SOLVERS
#define LIB_SCAMPI_KS_SOLVERS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "libscampi_ks_data_types.h"
#include <cmath>
#include <manif/manif.h>
//#include "libscampi_ks_utils.h"
//#include "libscampi_ks_costs.h"
#include <ceres/ceres.h>


void fkSolver(double *lc_cat, Eigen::Vector3d pos_init,  
              Eigen::Matrix3d rot_init, 
              Eigen::Vector2d fc0, 
              RobotParameters<double> params,
              FKDataOut<double> *result);

void ikSolver(Eigen::Vector3d p_platform, 
              Eigen::Matrix3d rot_init, 
              RobotParameters<double> params,
              IKDataOut<double> *result);
#endif
