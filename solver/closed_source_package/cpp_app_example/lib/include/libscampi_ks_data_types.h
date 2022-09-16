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

#ifndef LIB_SCAMPI_KS_DATA_TYPES
#define LIB_SCAMPI_KS_DATA_TYPES

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

template<typename T>
struct GeometricVariables{

    std::vector< Eigen::Matrix<T, 3, 1> > p_in_w;   // Pulley locations in world frame
    std::vector< Eigen::Matrix<T, 3, 1> > b_in_w;   // Body attachments in world frame
    std::vector< Eigen::Matrix<T, 3, 1> > b_rot;    // Rotated Body attachment points to W
    std::vector< Eigen::Matrix<T, 3, 1> > sx;       // Body to pulley directions projected on xy plane
    Eigen::Matrix<T, 3, 1>                r_to_cog; // I am not sure what this parameter referes to. It will be used in 
                                                    // get_cable_forces and get_geometrical_forces funtions.
};

template<typename T>
struct CatenaryVariables{

    std::vector<T> length;  //The ture distance between the anchor locations and body attachement points
    std::vector<T> c1;      // Catenary parameter 1
    std::vector<T> c2;      // Catenary parameter 2
    std::vector<T> y0_cat;  // Z component of the pulley location in world coordinate
    std::vector<T> yl_cat;  // Z component of the body attachment point in world coordinate
    std::vector<T> lc_cat;  // the ture lable length
};

template<typename T>
struct RobotState{

    Eigen::Matrix<T, 3, 1> p_platform;  // Pulley locations in world frame
    Eigen::Matrix<T, 3, 3> rot_platform; //rot_platform;   Orientation of end-effector with respect to the world
    std::vector<Eigen::Matrix<T, 2, 1>> cable_forces;// Horizontal and vertical cable forces at body attachment point
    Eigen::Matrix<T, 8, 1> cable_forces_compact; //all cable forces lumbed into a 8x1 vector
    Eigen::Matrix<T, 6, 1> wrench; // The wrench applied to the end-effector (F, T)
};

template<typename T>
struct RobotParameters{

    std::vector< Eigen::Matrix<T, 3, 1> > pulleys;   // Pulley locations in world frame
    std::vector< Eigen::Matrix<T, 3, 1> > ef_points; // cable attachment points in ef frame
    Eigen::Matrix<T, 3, 1>     r_to_cog;             // I am not sure what this parameter referes to. It will be used in 
                                                     // get_cable_forces and get_geometrical_forces funtions.
    T g_c;                                           // cable specific force per len
    T f_g;
};

template<typename T>
struct CatDataOut{
    std::vector<Eigen::Matrix<T, 3, 1>> b_in_w; // Body attachments in world frame
    std::vector<Eigen::Matrix<T, 2, 1>> cable_forces;// Horizontal and vertical cable forces at body attachment point
    std::vector<T> c1;      // Catenary parameter 1
    std::vector<T> c2;      // Catenary parameter 2
    std::vector<T> lc_cat;  // the ture lable length
    Eigen::Matrix<T, 3, 3> rot_platform; 
};

template<typename T>
struct IKDataOut{
    std::vector<Eigen::Matrix<T, 3, 1>> b_in_w; // Body attachments in world frame
    std::vector<Eigen::Matrix<T, 2, 1>> cable_forces;// Horizontal and vertical cable forces at body attachment point
    std::vector<T> c1;      // Catenary parameter 1
    std::vector<T> c2;      // Catenary parameter 2
    std::vector<T> lc_cat;  // the ture lable length
    Eigen::Matrix<T, 3, 3> rot_platform; 
};

template<typename T>
struct FKDataOut{
    std::vector<Eigen::Matrix<T, 3, 1>> b_in_w; // Body attachments in world frame
    std::vector<Eigen::Matrix<T, 2, 1>> cable_forces;// Horizontal and vertical cable forces at body attachment point
    std::vector<T> c1;      // Catenary parameter 1
    std::vector<T> c2;      // Catenary parameter 2
    std::vector<T> lc_cat;  // the ture lable length
    Eigen::Matrix<T, 3, 1> p_platform;    // the optimized end-effector position
    Eigen::Matrix<T, 3, 3> rot_platform;    // the optimized end-effector orientation
};

#endif
