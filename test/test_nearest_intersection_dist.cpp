#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(test_nearest_intersection_dist, nearestIntersectionDistance)
{
    EXPECT_NEAR((nearestIntersectionDistance<double, Point2d>(line_n, lines_h)), 1.0, 1e-6);
    EXPECT_NEAR((nearestIntersectionDistance<double, Point2d>(line_o, lines_h)), 0.0, 1e-6);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
