cmake_minimum_required(VERSION 2.8.3)
project(cslibs_boost_geometry)

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
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/algorithms.cpp
    src/linefit_lsq.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
)

add_custom_target(show_${PROJECT_NAME}_headers_in_qt SOURCES
    include/utils_boost_geometry/algorithms.h
    include/utils_boost_geometry/algorithms.hpp
    include/utils_boost_geometry/types.hpp
    include/utils_boost_geometry/linefit_lsq.hpp
)

add_executable(${PROJECT_NAME}_test
    src/test.cpp
)

target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
