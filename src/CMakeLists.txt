find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

message(STATUS "CMAKE SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE CURRENT SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
message(STATUS "GTSAM DIRECTORIES ${GTSAM_INCLUDE_DIR}")
#set (GTSAM_LIBRARY_DIR ${GTSAM_DIR}/../../../lib/libgtsam.so)
#include_directories(${GTSAM_LIBRARY_DIR})
message(STATUS "GTSAM LIBRARY ${GTSAM_LIBRARY_DIR}")

get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

rock_library(envire_sam
    HEADERS Conversions.hpp
            Configuration.hpp
            LandmarkTransformFactor.h
            ESAM.hpp
    SOURCES ESAM.cpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG base-types gtsam
    DEPS_PKGCONFIG pcl_registration-1.7 pcl_filters-1.7 pcl_kdtree-1.7 pcl_features-1.7 pcl_keypoints-1.7 flann
    DEPS_PKGCONFIG envire_core)

