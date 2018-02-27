cmake_minimum_required(VERSION 2.8.3)
project(cslibs_boost_geometry)
set(CMAKE_CXX_FLAGS "-fext-numeric-literals ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

set(CSLIBS_BOOST_GEOMETRY_BUILD_TESTS True)

## C++11
include(cmake/cslibs_boost_geometry_c++11.cmake)
## OMP
set(OpenMP true)
include(cmake/cslibs_boost_geometry_omp.cmake)
## Extras
include(cmake/cslibs_boost_geometry_extras.cmake)

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
    src/library/algorithms.cpp
    src/library/linefit_lsq.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
)

add_custom_target(show_${PROJECT_NAME}_headers_in_qt SOURCES
    include/cslibs_boost_geometry/algorithms.h
    include/cslibs_boost_geometry/algorithms.hpp
    include/cslibs_boost_geometry/types.hpp
    include/cslibs_boost_geometry/linefit_lsq.hpp
)

if(${CATKIN_ENABLE_TESTING})
    include(cmake/cslibs_boost_geometry_add_unit_test_gtest.cmake)

    cslibs_boost_geometry_add_unit_test_gtest(test_intersection
                                               SRCS test/test_intersection.cpp
                                               LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_nearest_intersection
                                              SRCS test/test_nearest_intersection.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_nearest_intersection_batch
                                              SRCS test/test_nearest_intersection_batch.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_translation
                                              SRCS test/test_translation.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_polar_line_set
                                              SRCS test/test_polar_line_set.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_polygons
                                              SRCS test/test_polygons.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_nearest_intersection_dist
                                              SRCS test/test_nearest_intersection_dist.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_nearest_intersection_dist_batch
                                              SRCS test/test_nearest_intersection_dist_batch.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_distance
                                              SRCS test/test_distance.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_equality
                                              SRCS test/test_equality.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_rotation
                                              SRCS test/test_rotation.cpp
                                              LIBS ${PROJECT_NAME})

    cslibs_boost_geometry_add_unit_test_gtest(test_line_angles
                                              SRCS test/test_line_angles.cpp
                                              LIBS ${PROJECT_NAME})

endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
