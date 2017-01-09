#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testLineToLineAngle)
{
    Point2d p1(0.0, 0.0);
    Point2d p2(0.0, 1.0);
    Point2d p3(1.0, 0.0);
    Point2d p4(1.0, 1.0);
    Point2d p5(2.0, 1.0);
    Point2d p6(0.0,-1.0);
    Point2d p7(1.0,-1.0);

    Line2d  l11(p1, p2);
    Line2d  l12(p2, p1);
    Line2d  l21(p1, p3);
    Line2d  l22(p3, p1);

    Line2d  l31(p1, p4);
    Line2d  l32(p4, p1);
    Line2d  l41(p4, p5);
    Line2d  l42(p5 ,p4);

    Line2d  l51(p1, p6);
    Line2d  l52(p6, p1);
    Line2d  l61(p4, p7);
    Line2d  l62(p7, p4);

    EXPECT_NEAR((angle<double, Point2d>(l11, l21, 1e-9)), M_PI / 2.0, 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l11, l22, 1e-9)), M_PI / 2.0, 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l12, l21, 1e-9)), M_PI / 2.0, 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l12, l22, 1e-9)), M_PI / 2.0, 1e-9);

    EXPECT_NEAR((angle<double, Point2d>(l31, l41, 1e-9)), rad(135), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l31, l42, 1e-9)), rad(135), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l41, 1e-9)), rad(135), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l42, 1e-9)), rad(135), 1e-9);

    EXPECT_NEAR((angle<double, Point2d>(l31, l21, 1e-9)), rad(45), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l31, l22, 1e-9)), rad(45), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l21, 1e-9)), rad(45), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l22, 1e-9)), rad(45), 1e-9);

    EXPECT_NEAR((angle<double, Point2d>(l31, l51, 1e-9)), rad(135), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l31, l52, 1e-9)), rad(135), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l51, 1e-9)), rad(135), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l52, 1e-9)), rad(135), 1e-9);

    EXPECT_NEAR((angle<double, Point2d>(l31, l61, 1e-9)), rad(45), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l31, l62, 1e-9)), rad(45), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l61, 1e-9)), rad(45), 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l62, 1e-9)), rad(45), 1e-9);

    EXPECT_NEAR((angle<double, Point2d>(l31, l32, 1e-9)), 0.0, 1e-9);
    EXPECT_NEAR((angle<double, Point2d>(l32, l31, 1e-9)), 0.0, 1e-9);

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
