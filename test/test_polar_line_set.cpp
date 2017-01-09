#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testAngleConversion)
{
    EXPECT_EQ(rad(90),      M_PI_2);
    EXPECT_EQ(rad(180),     M_PI);
    EXPECT_EQ(rad(360), 2 * M_PI);
    EXPECT_EQ(rad(0),       0);

    EXPECT_EQ(deg(M_PI_2),   90.0);
    EXPECT_EQ(deg(M_PI),     180.0);
    EXPECT_EQ(deg(2 * M_PI), 360.0);
    EXPECT_EQ(deg(0),        0);
}


TEST(TestCSLibsBoostGeometry, testPolarLineSet)
{
    LineSet2d test_result;
    polarLineSet<Point2d, periodic>(Point2d(0.0,0.0),
                                    rad(0.0),
                                    rad(180.0),
                                    M_PI_2,
                                    1.0,
                                    test_result);
    EXPECT_EQ(test_result.size(), 3);
    for(auto &l : test_result) {
        EXPECT_TRUE(equal(l.first, Point2d(0.0, 0.0), 1e-9));
        EXPECT_NEAR((length<double, Point2d>(l)), 1.0, 1e-9);
    }
    EXPECT_TRUE(equal(test_result[0].second, Point2d(0.0, -1.0), 1e-9));
    EXPECT_TRUE(equal(test_result[1].second, Point2d(1.0,  0.0), 1e-9));
    EXPECT_TRUE(equal(test_result[2].second, Point2d(0.0,  1.0), 1e-9));
    test_result.clear();

    polarLineSet<Point2d, periodic>(Point2d(1.0, 1.0),
                                    rad(0.0),
                                    rad(180.0),
                                    M_PI_2,
                                    3.0,
                                    test_result);
    EXPECT_EQ(test_result.size(), 3);
    for(auto &l : test_result) {
        EXPECT_TRUE(equal(l.first, Point2d(1.0, 1.0), 1e-9));
        EXPECT_NEAR((length<double, Point2d>(l)), 3.0, 1e-9);
    }

    EXPECT_TRUE(equal(test_result[0].second, Point2d(1.0,  -2.0), 1e-9));
    EXPECT_TRUE(equal(test_result[1].second, Point2d(4.0,   1.0), 1e-9));
    EXPECT_TRUE(equal(test_result[2].second, Point2d(1.0,   4.0), 1e-9));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
