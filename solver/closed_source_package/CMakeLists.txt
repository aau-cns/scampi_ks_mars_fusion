cmake_minimum_required(VERSION 2.8.12)
project(scampi_kinematic_solver)
set(CMAKE_CXX_STANDARD 17)

include(GNUInstallDirs)

find_library(KS_LIB libscampi_ks lib)

install(FILES lib/liblibscampi_ks.a                 DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES lib/include/libscampi_ks_data_types.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES lib/include/libscampi_ks_solvers.h    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})



