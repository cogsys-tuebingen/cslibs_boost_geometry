#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(test_distance, distance_point_to_line)
{
    EXPECT_NEAR((distance<double, Point2d>(point_a, line_n)), 1.0, 1e-9);
    EXPECT_NEAR((distance<double, Point2d>(point_b, line_n)), 0.0, 1e-9);
    EXPECT_NEAR((distance<double, Point2d>(point_c, line_n)), 2.0, 1e-9);
    EXPECT_NEAR((distance<double, Point2d>(point_d, line_n)), 0.0, 1e-9);
    EXPECT_NEAR((distance<double, Point2d>(point_e, line_n)), 2.0, 1e-9);
    EXPECT_NEAR((distance<double, Point2d>(point_f, line_n)), sqrt(2), 1e-9);
    EXPECT_NEAR((distance<double, Point2d>(point_g, line_n)), sqrt(5), 1e-9);
}

TEST(test_distance, distance_line_to_line)
{
    EXPECT_NEAR((minEndPointDistance<double, Point2d>(line_a, line_b)), 1.0, 1e-9);
    EXPECT_NEAR((minEndPointDistance<double, Point2d>(line_a, line_c)), 0.0, 1e-9);
    EXPECT_NEAR((minEndPointDistance<double, Point2d>(line_a, line_g)), sqrt(0.5), 1e-9);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
