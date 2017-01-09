#include <cslibs_boost_geometry/algorithms.h>
#include <gtest/gtest.h>

#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testPointTranslation)
{
    Translation2d t(1.0, 1.0);
    Point2d test_result;

    EXPECT_TRUE(translate(point_a, t, test_result));
    EXPECT_TRUE(equal(test_result, point_b, 1e-6));

    t = Translation2d(-1.0, 2.0);
    EXPECT_TRUE(translate(test_result, t, test_result));
    EXPECT_TRUE(equal(test_result, point_c, 1e-6));


}

TEST(TestCSLibsBoostGeometry, testLineTranslation)
{
    Translation2d t(-1.0, 2.0);
    Line2d test_result;

    EXPECT_TRUE(translate<Point2d>(line_a, t, test_result));
    EXPECT_TRUE(equal<Point2d>(test_result, line_k, 1e-6));
}

TEST(TestCSLibsBoostGeometry, testPointSetTranslation)
{
    Translation2d t(-1.0, 2.0);
    PointSet2d test_result;

    EXPECT_TRUE(translate<Point2d>(points_a, t, test_result));
    EXPECT_EQ(test_result.size(), points_a.size());
    for(unsigned int i = 0 ; i < points_a.size() ; ++i) {
        EXPECT_TRUE(equal(test_result.at(i), points_b.at(i), 1e-6));
    }
}

TEST(TestCSLibsBoostGeometry, testLineSetTranslation)
{
    Translation2d t(-1.0, 2.0);
    LineSet2d test_result;

    EXPECT_TRUE(translate<Point2d>(lines_a, t, test_result));
    for(unsigned int i = 0 ; i < lines_a.size() ; ++i) {
        EXPECT_EQ(test_result.at(i).first.x(), lines_a.at(i).first.x()  - 1.0);
        EXPECT_EQ(test_result.at(i).first.y(), lines_a.at(i).first.y()  + 2.0);
        EXPECT_EQ(test_result.at(i).second.x(),lines_a.at(i).second.x() - 1.0);
        EXPECT_EQ(test_result.at(i).second.y(),lines_a.at(i).second.y() + 2.0);
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
