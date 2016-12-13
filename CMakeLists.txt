cmake_minimum_required(VERSION 2.8.3)
project(utils_boost_geometry)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)



## C++11
include(cmake/c++11.cmake)
## OMP
set(OpenMP true)
include(cmake/omp.cmake)
## Extras
include(cmake/extras.cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_boost_geometry
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)

add_library(utils_boost_geometry
    src/algorithms.cpp
    src/linefit_lsq.cpp
)

target_link_libraries(utils_boost_geometry
    ${Boost_LIBRARIES}
)

add_custom_target(show_${PROJECT_NAME}_headers_in_qt SOURCES
    include/utils_boost_geometry/algorithms.h
    include/utils_boost_geometry/algorithms.hpp
    include/utils_boost_geometry/types.hpp
    include/utils_boost_geometry/linefit_lsq.hpp
)

add_executable(utils_boost_geometry_test
    src/test.cpp
)

target_link_libraries(utils_boost_geometry_test
    utils_boost_geometry
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
