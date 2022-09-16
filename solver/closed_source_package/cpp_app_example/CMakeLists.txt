cmake_minimum_required(VERSION 2.8.12)
project(scampi_kinematic_solver_test)

set(CMAKE_CXX_STANDARD 17)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfma")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mfma")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
# Find the manif library: https://github.com/artivis/manif/blob/devel/docs/pages/cpp/Quick-start.md
find_package(manif REQUIRED)

find_library(KS_LIB libscampi_ks)

include_directories(include
    ${KS_LIB_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${manif_INCLUDE_DIRS})


add_executable(solver_test solver_funcs_tests.cpp)
target_link_libraries(solver_test
        ${CERES_LIBRARIES}
        ${KS_LIB}
        gtest
        gtest_main
        ${GTEST_LIBRARIES}
)
