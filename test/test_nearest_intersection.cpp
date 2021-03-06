#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testNeareIntersection)
{
    PointSet2d test_result;

    /// tests
    EXPECT_TRUE(nearestIntersection<Point2d>(line_a, lines_a, test_result));
    EXPECT_EQ(test_result.size(), 2);
    EXPECT_TRUE(equal(test_result.front(), line_a.first, 1e-6));
    EXPECT_TRUE(equal(test_result.back(), line_a.second, 1e-6));
    test_result.clear();

    EXPECT_FALSE(nearestIntersection<Point2d>(line_a, lines_b, test_result));
    EXPECT_EQ(test_result.size(), 0);
    test_result.clear();


    EXPECT_TRUE(nearestIntersection<Point2d>(line_c, lines_c, test_result));
    EXPECT_EQ(test_result.size(), 1);
    EXPECT_TRUE(equal(test_result.front(), Point2d(0.5,0.5), 1e-6));
    test_result.clear();

    EXPECT_TRUE(nearestIntersection<Point2d>(line_g, lines_c, test_result));
    EXPECT_EQ(test_result.size(), 1);
    EXPECT_TRUE(equal(test_result.front(), Point2d(0.5,0.5), 1e-6));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
