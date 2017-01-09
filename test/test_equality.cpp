#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testPointEquality)
{
    EXPECT_TRUE(equal(point_a, point_a, 1e-9));
    EXPECT_FALSE(equal(point_a, point_b, 1e-9));
    EXPECT_FALSE(equal(point_b, point_a, 1e-9));
    EXPECT_TRUE(equal(point_b, point_b, 1e-9));
    EXPECT_FALSE(equal(point_b, point_c, 1e-9));
    EXPECT_TRUE(equal(point_c, point_c, 1e-9));
}

TEST(TestCSLibsBoostGeometry, testLineEquality)
{
    EXPECT_TRUE((equal<Point2d>(line_a, line_a, 1e-9)));
    EXPECT_FALSE((equal<Point2d>(line_a, line_b, 1e-9)));
    EXPECT_FALSE((equal<Point2d>(line_b, line_a, 1e-9)));
    EXPECT_TRUE((equal<Point2d>(line_b, line_b, 1e-9)));
    EXPECT_FALSE((equal<Point2d>(line_b, line_c, 1e-9)));
    EXPECT_TRUE((equal<Point2d>(line_c, line_c, 1e-9)));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
