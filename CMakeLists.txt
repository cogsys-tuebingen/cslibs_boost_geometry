cmake_minimum_required(VERSION 2.8.3)
project(utils_boost_geometry)

find_package(catkin REQUIRED)
find_package(Boost  REQUIRED)

set(CMAKE_BUILD_TYPE Release)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_boost_geometry
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS Boost
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)

add_library(utils_boost_geometry
    include/utils_boost_geometry/algorithms.hpp
    include/utils_boost_geometry/types.hpp
    src/algorithms.cpp
)

target_link_libraries(utils_boost_geometry
    ${Boost_LIBRARIES}
)

add_executable(utils_boost_geometry_test
    src/test.cpp
)

target_link_libraries(utils_boost_geometry_test
    utils_boost_geometry
)
