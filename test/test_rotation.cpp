#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testPointRotation)
{
    Rotation2d rot(M_PI);
    Point2d    test_result;

    EXPECT_TRUE(rotate(point_a, rot, test_result));
    EXPECT_TRUE(equal(test_result, Point2d(0, 0), 1e-9));
    EXPECT_TRUE(rotate(point_b, rot, test_result));
    EXPECT_TRUE(equal(test_result, Point2d(-1, -1), 1e-9));
    EXPECT_TRUE(rotate(point_g, rot, test_result));
    EXPECT_TRUE(equal(test_result, Point2d(1, 1), 1e-9));

}

TEST(TestCSLibsBoostGeometry, testLineRotation)
{
    Rotation2d rot(M_PI);
    Line2d     test_result;

    EXPECT_TRUE(rotate<Point2d>(line_a, rot, test_result));
    EXPECT_TRUE(equal(test_result.first, line_a.first, 1e-9));
    EXPECT_TRUE(equal(test_result.second, Point2d(-1,-1), 1e-9));

    EXPECT_TRUE(rotate<Point2d>(line_e, rot, test_result));
    EXPECT_TRUE(equal(test_result.first, Point2d(10.0, -2.0), 1e-9));
    EXPECT_TRUE(equal(test_result.second, Point2d(11,-3.0), 1e-9));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
