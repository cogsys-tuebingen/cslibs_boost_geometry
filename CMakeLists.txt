cmake_minimum_required(VERSION 2.8.3)
project(utils_boost_geometry)

find_package(catkin REQUIRED)
find_package(Boost  REQUIRED)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#-------------------------- OMP -----------------------------------------------------------#
set(OpenMP true)
if(${OpenMP})
    message("utils_vectormaps - OpenMP support enabled!")

    find_package(OpenMP REQUIRED)

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

else()
    message("utils_vectormaps - OpenMP support disabled!")
endif()
#-------------------------- OMP -----------------------------------------------------------#

add_definitions("-Ofast -march=native -ffast-math")
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
    include/utils_boost_geometry/linefit_lsq.hpp
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
