cmake_minimum_required(VERSION 2.8.3)
project(cslibs_boost_geometry)


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

    add_unit_test_gtest(${PROJECT_NAME}_test_intersection
                  test/test_intersection.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_nearest_intersection
                  test/test_nearest_intersection.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_nearest_intersection_batch
                  test/test_nearest_intersection_batch.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_translation
                  test/test_translation.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_polar_line_set
                  test/test_polar_line_set.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_polygons
                  test/test_polygons.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_nearest_intersection_dist
                  test/test_nearest_intersection_dist.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_nearest_intersection_dist_batch
                  test/test_nearest_intersection_dist_batch.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_distance
                  test/test_distance.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_equality
                  test/test_equality.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_rotation
                  test/test_rotation.cpp
                  ${PROJECT_NAME})

    add_unit_test_gtest(${PROJECT_NAME}_test_line_angles
                  test/test_line_angles.cpp
                  ${PROJECT_NAME})

endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
