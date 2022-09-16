cmake_minimum_required(VERSION 2.8.12)
project(scampi_kinematic_solver)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_STANDARD 17)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfma")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mfma")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cotire)
include(GNUInstallDirs)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
# Find the manif library: https://github.com/artivis/manif/blob/devel/docs/pages/cpp/Quick-start.md
find_package(manif REQUIRED)


include_directories(include
    lib/include
    ${EIGEN3_INCLUDE_DIRS}
    ${manif_INCLUDE_DIRS}
)


add_library(libscampi_ks
    lib/src/libscampi_ks_solvers.cpp
    )


target_link_libraries(libscampi_ks
${CERES_LIBRARIES}
)

cotire(libscampi_ks)

#install(TARGETS libscampi_ks LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
#install(FILES lib/include/libscampi_ks_data_types.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#install(FILES lib/include/libscampi_ks_solvers.h    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

function(make_gtest test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name}
        libscampi_ks
        gtest
        gtest_main
        ${SCAMPI_SOLVER_LIBS}
        ${GTEST_LIBRARIES}
        ${CERES_LIBRARIES}
        )
    cotire(${test_name})
endfunction()

make_gtest(tests_util_funcs        tests/utils_funcs_tests.cpp)
make_gtest(tests_cost_funcs        tests/cost_funcs_tests.cpp)
make_gtest(tests_solver_funcs      tests/solver_funcs_tests.cpp)


add_subdirectory(pybind11)

pybind11_add_module(scampi_ks_solver src/main.cpp)

target_link_libraries(scampi_ks_solver PRIVATE
                      libscampi_ks
                      ${SCAMPI_SOLVER_LIBS}
                      ${CERES_LIBRARIES}
)


target_compile_definitions(scampi_ks_solver
                           PRIVATE VERSION_INFO=${VERSION_INFO})
