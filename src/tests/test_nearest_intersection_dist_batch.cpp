#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(test_nearest_intersection_dist_batch, nearestIntersectionDistBatch)
{
    std::vector<double> test_result;
    nearestIntersectionDistanceBatch<double, Point2d>(lines_i, lines_h, 0.0, test_result);
    EXPECT_EQ(test_result[0], 1.0);
    EXPECT_EQ(test_result[1], 0.0);
    EXPECT_EQ(test_result[2], 1.0);
    test_result.clear();

    nearestIntersectionDistanceBatch<double, Point2d>(lines_i, lines_i, -1.0, test_result);
    EXPECT_EQ(test_result[0], 0.0);
    EXPECT_EQ(test_result[1], 0.0);
    EXPECT_EQ(test_result[2], 0.0);
    test_result.clear();
    nearestIntersectionDistanceBatch<double, Point2d>(lines_i, lines_h, -1.0, test_result);
    assert(test_result[0] ==  1.0);
    assert(test_result[1] == -1.0);
    assert(test_result[2] ==  1.0);
    test_result.clear();
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
