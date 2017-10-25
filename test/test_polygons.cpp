#include <cslibs_boost_geometry/algorithms.h>

#include <gtest/gtest.h>
#include <boost/geometry/algorithms/area.hpp>


#include "points_and_lines.hpp"

using namespace cslibs_boost_geometry::algorithms;
using namespace cslibs_boost_geometry::types;
using namespace cslibs_boost_geometry::test_samples;

TEST(TestCSLibsBoostGeometry, testPolygons)
{
    Polygon2d test_result;
    circularPolygonApproximation(Point2d(0.0,0.0),
                                 1.0,
                                 rad(1.0),
                                 test_result);

    Polygon2d::ring_type &outer_ring = test_result.outer();

    EXPECT_EQ(outer_ring.size(), 361);
    EXPECT_TRUE(equal(outer_ring.front(), Point2d(1.0, 0.0), 1e-9));
    EXPECT_TRUE(equal(outer_ring.back(),  Point2d(1.0, 0.0), 1e-9));
    EXPECT_TRUE(outer_ring[0].x() > outer_ring[1].x());
    EXPECT_TRUE(outer_ring[0].y() > outer_ring[1].y());
    EXPECT_TRUE(outer_ring[0].x() > outer_ring[2].x());
    EXPECT_TRUE(outer_ring[0].y() > outer_ring[2].y());
    EXPECT_TRUE(outer_ring[0].x() > outer_ring[outer_ring.size() - 3].x());
    EXPECT_TRUE(outer_ring[0].y() < outer_ring[outer_ring.size() - 3].y());

    EXPECT_TRUE(equal(outer_ring[90],  Point2d(0.0, -1.0), 1e-9));
    EXPECT_TRUE(equal(outer_ring[180], Point2d(-1.0, 0.0), 1e-9));
    EXPECT_TRUE(equal(outer_ring[270], Point2d( 0.0, 1.0), 1e-9));
}

TEST(TestCSLibsBoostGeometry, testWithin)
{
    Polygon2d test_result;
    circularPolygonApproximation(Point2d(0.0,0.0),
                                 1.0,
                                 rad(1.0),
                                 test_result);
    EXPECT_TRUE(withinExcl<Point2d>(point_a, test_result));
}

TEST(TestCSLibsBoostGeometry, testIntersection)
{
    Polygon2d test_result;
    circularPolygonApproximation(Point2d(0.0,0.0),
                                 1.0,
                                 rad(1.0),
                                 test_result);
    EXPECT_FALSE(intersects<Point2d>(line_e, test_result));
    EXPECT_TRUE(intersects<Point2d>(line_b, test_result));
}

TEST(TestCSLibsBoostGeometry, testTouching)
{
    Polygon2d test_result;
    circularPolygonApproximation(Point2d(0.0,0.0),
                                 1.0,
                                 rad(1.0),
                                 test_result);
    EXPECT_TRUE(touches<Point2d>(line_b, test_result));
    EXPECT_FALSE(touches<Point2d>(line_e, test_result));
    EXPECT_FALSE(touches<Point2d>(line_f, test_result));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
