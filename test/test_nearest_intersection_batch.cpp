#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testLineBatchIntersection)
{

    IntersectionResultSet<Point2d>::type test_result;
    nearestIntersectionBatch<Point2d>(lines_a, lines_a, test_result);

    EXPECT_EQ(test_result.size(), lines_a.size());
    for(unsigned int i = 0 ; i < lines_a.size() ; ++i) {
        EXPECT_TRUE(test_result.at(i).valid);
        EXPECT_EQ(test_result.at(i).result.size(), 2);
        EXPECT_TRUE(equal(test_result.at(i).result.at(0), lines_a.at(i).first, 1e-6));
        EXPECT_TRUE(equal(test_result.at(i).result.at(1), lines_a.at(i).second, 1e-6));
    }
    test_result.clear();

    nearestIntersectionBatch<Point2d>(lines_d, lines_c, test_result);
    EXPECT_EQ(test_result.size(), lines_d.size());
    assert(test_result.size() == lines_d.size());
    for(unsigned int i = 0 ; i < lines_d.size() ; ++i) {
        EXPECT_TRUE(test_result.at(i).valid);
        EXPECT_EQ(test_result.at(i).result.size(), 1);
    }
    EXPECT_TRUE(equal(test_result.front().result.front(), Point2d(0.25, 0.75), 1e-6));
    EXPECT_TRUE(equal(test_result.back().result.front(), Point2d(0.75, 0.25), 1e-6));
    test_result.clear();

    nearestIntersectionBatch<Point2d>(lines_d, lines_e, test_result);
    EXPECT_EQ(test_result.size(), lines_d.size());
    for(unsigned int i = 0 ; i < lines_d.size() ; ++i) {
       EXPECT_FALSE(test_result.at(i).valid);
       EXPECT_EQ(test_result.at(i).result.size(), 0);
    }
    test_result.clear();

    nearestIntersectionBatch<Point2d>(lines_e, lines_f, test_result);
    EXPECT_EQ(test_result.size(), lines_e.size());
    EXPECT_FALSE(test_result.front().valid);
    EXPECT_TRUE(test_result.back().valid);
    EXPECT_EQ(test_result.front().result.size(), 0);
    EXPECT_EQ(test_result.back().result.size(), 1);
    EXPECT_TRUE(equal(test_result.back().result.front(), Point2d(2.0, 2.0), 1e-6));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
