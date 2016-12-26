cmake_minimum_required(VERSION 2.8.3)
project(cslibs_boost_geometry)


find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

set(CSLIBS_BOOST_GEOMETRY_BUILD_TESTS True)

## C++11
include(cmake/c++11.cmake)
## OMP
set(OpenMP true)
include(cmake/omp.cmake)
## Extras
include(cmake/extras.cmake)
## Tests
include(cmake/add_unit_test.cmake)


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

if(${CSLIBS_BOOST_GEOMETRY_BUILD_TESTS})
    include_directories(${GTEST_INCLUDE_DIRS})

    add_unit_test(${PROJECT_NAME}_test_intersection
                  src/tests/test_intersection.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_nearest_intersection
                  src/tests/test_nearest_intersection.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_nearest_intersection_batch
                  src/tests/test_nearest_intersection_batch.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_translation
                  src/tests/test_translation.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_polar_line_set
                  src/tests/test_polar_line_set.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_polygons
                  src/tests/test_polygons.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_nearest_intersection_dist
                  src/tests/test_nearest_intersection_dist.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_nearest_intersection_dist_batch
                  src/tests/test_nearest_intersection_dist_batch.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_distance
                  src/tests/test_distance.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_equality
                  src/tests/test_equality.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_rotation
                  src/tests/test_rotation.cpp
                  ${PROJECT_NAME})

    add_unit_test(${PROJECT_NAME}_test_line_angles
                  src/tests/test_line_angles.cpp
                  ${PROJECT_NAME})

endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
