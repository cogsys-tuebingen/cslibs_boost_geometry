#include <utils_boost_geometry/algorithms.hpp>
#include <boost/assign.hpp>

using namespace utils_boost_geometry::algorithms;
using namespace utils_boost_geometry::types;

const static Point2d POINT_A(
        1.0, 1.0
        );
const static Point2d POINT_B(
        0.0, 0.0
        );
const static PointSet2d POINTS_A =
        boost::assign::list_of
        (Point2d(0.0,0.0))
        (Point2d(2.0,3.0));
const static Line2d LINE_A(
        Point2d(0.0, 0.0),
        Point2d(1.0, 1.0)
        );
const static Line2d LINE_B(
        Point2d(0.0, 1.0),
        Point2d(1.0, 0.0)
        );
const static Line2d LINE_C(
        Point2d(0.0, 0.0),
        Point2d(2.0, 2.0)
        );
const static Line2d LINE_D(
        Point2d(0.0, 2.0),
        Point2d(3.0, 3.0)
        );
const static Line2d LINE_E(
        Point2d(-10.0, 2.0),
        Point2d(-11.0, 3.0)
        );

const static Line2d LINE_F(
        Point2d(0.0, 2.0),
        Point2d(2.0, 0.0)
        );

const static Line2d LINE_G(
        Point2d(0.5,0.5),
        Point2d(3.0,3.0)
        );

const static Line2d LINE_H(
        Point2d(0.0,0.5),
        Point2d(3.0,3.5)
        );

const static Line2d LINE_I(
        Point2d(0.5,0.0),
        Point2d(3.5,3.0)
        );

const static Line2d LINE_J(
        Point2d(0.0,4.0),
        Point2d(4.0,0.0)
        );

const static LineSet2d LINES_A =
        boost::assign::list_of
        (LINE_B)(LINE_C)(LINE_D);

const static LineSet2d LINES_B =
        boost::assign::list_of
        (LINE_D)(LINE_E);

const static LineSet2d LINES_C =
        boost::assign::list_of
        (LINE_B)(LINE_F);

const static LineSet2d LINES_D =
        boost::assign::list_of
        (LINE_H)(LINE_I);

const static LineSet2d LINES_E =
        boost::assign::list_of
        (LINE_A)(LINE_C);

const static LineSet2d LINES_F =
        boost::assign::list_of
        (LINE_E)(LINE_J);

const static LineSet2d LINES_G =
        boost::assign::list_of
        (LINE_A)(LINE_B);

/**
 * @brief TEST_1_INTERSECT will test out single intersections.
 */
void TEST_1_INTERSECT() {
    PointSet<Point2d>::type inter;
    assert(intersection<Point2d>(LINE_A, LINE_B, inter));
    assert(inter.size() == 1);
    assert(inter.back().x() == 0.5);
    assert(inter.back().y() == 0.5);
    inter.clear();
    assert(intersection<Point2d>(LINE_A, LINE_A, inter));
    assert(inter.size() == 2);
    assert(inter.front().x() == LINE_A.first.x());
    assert(inter.front().y() == LINE_A.first.y());
    assert(inter.back().x()  == LINE_A.second.x());
    assert(inter.back().y()  == LINE_A.second.y());
    inter.clear();
    assert(intersection<Point2d>(LINE_A, LINE_C, inter));
    assert(inter.size() == 2);
    assert(inter.front().x() == LINE_A.first.x());
    assert(inter.front().y() == LINE_A.first.y());
    assert(inter.back().x()  == LINE_A.second.x());
    assert(inter.back().y()  == LINE_A.second.y());
    inter.clear();
    assert(intersection<Point2d>(LINE_C, LINE_A, inter));
    assert(inter.size() == 2);
    assert(inter.front().x() == LINE_A.first.x());
    assert(inter.front().y() == LINE_A.first.y());
    assert(inter.back().x()  == LINE_A.second.x());
    assert(inter.back().y()  == LINE_A.second.y());
    inter.clear();
    assert(!intersection<Point2d>(LINE_A, LINE_D, inter));
    assert(inter.size() == 0);
    std::cout << "TEST 1 PASSED" << std::endl;
}

/**
 * @brief TEST_2_INTERSECTIONS will test out multiple intersecions
 *        given through a set of lines to be intersected.
 */
void TEST_2_INTERSECTIONS() {
    PointSet<Point2d>::type inter;
    assert(nearestIntersection<Point2d>(LINE_A, LINES_A, inter));
    assert(inter.size() == 2);
    assert(inter.front().x() == LINE_A.first.x());
    assert(inter.front().y() == LINE_A.first.y());
    assert(inter.back().x()  == LINE_A.second.x());
    assert(inter.back().y()  == LINE_A.second.y());
    inter.clear();
    assert(!nearestIntersection<Point2d>(LINE_A, LINES_B, inter));
    assert(inter.size() == 0);
    inter.clear();
    assert(nearestIntersection<Point2d>(LINE_C, LINES_C, inter));
    assert(inter.size() == 1);
    assert(inter.back().x() == 0.5);
    assert(inter.back().y() == 0.5);
    inter.clear();
    assert(nearestIntersection<Point2d>(LINE_G, LINES_C, inter));
    assert(inter.size() == 1);
    assert(inter.back().x() == 0.5);
    assert(inter.back().y() == 0.5);
    std::cout << "TEST 2 PASSED" << std::endl;
}

/**
 * @brief TEST_3_INTERSECTIONS will test out multiple intersections
 *        given by two sets of lines which are intersected with each
 *        other.
 */
void TEST_3_INTERSECTIONS() {
    ValidatedResultSet<Point2d>::type results;
    multiNearestIntersection<Point2d>(LINES_A, LINES_A, results);
    assert(results.size() == LINES_A.size());
    for(unsigned int i = 0 ; i < LINES_A.size() ; ++i) {
        assert(results.at(i).valid);
        assert(results.at(i).result.size() == 2);
        assert(results.at(i).result.at(0).x() == LINES_A.at(i).first.x());
        assert(results.at(i).result.at(0).y() == LINES_A.at(i).first.y());
        assert(results.at(i).result.at(1).x() == LINES_A.at(i).second.x());
        assert(results.at(i).result.at(1).y() == LINES_A.at(i).second.y());
    }
    results.clear();
    multiNearestIntersection<Point2d>(LINES_D, LINES_C, results);
    assert(results.size() == LINES_D.size());
    for(unsigned int i = 0 ; i < LINES_D.size() ; ++i) {
        assert(results.at(i).valid);
        assert(results.at(i).result.size() == 1);
    }
    assert(results.front().result.front().x() == 0.25);
    assert(results.front().result.front().y() == 0.75);
    assert(results.back().result.front().x()  == 0.75);
    assert(results.back().result.front().y()  == 0.25);

    results.clear();
    multiNearestIntersection<Point2d>(LINES_D, LINES_E, results);
    assert(results.size() == LINES_D.size());
    for(unsigned int i = 0 ; i < LINES_D.size() ; ++i) {
        assert(!results.at(i).valid);
        assert(results.at(i).result.size() == 0);
    }

    results.clear();
    multiNearestIntersection<Point2d>(LINES_E, LINES_F, results);
    assert(results.size() == LINES_E.size());
    assert(!results.front().valid);
    assert(results.back().valid);
    assert(results.front().result.size() == 0);
    assert(results.back().result.size() == 1);
    assert(results.back().result.front().x() == 2.0);
    assert(results.back().result.front().y() == 2.0);
    std::cout << "TEST 3 PASSED" << std::endl;
}

void TEST_4_TRANSLATION ()
{
    Point2d             p_trans;
    Translation2d trans(1.0, 1.0);
    assert(translate<Point2d>(POINT_A, trans, p_trans));
    assert(p_trans.x() == 2.0);
    assert(p_trans.y() == 2.0);

    trans = Translation2d(-1.0, 2.0);
    assert(translate<Point2d>(POINT_A, trans, p_trans));
    assert(p_trans.x() == 0.0);
    assert(p_trans.y() == 3.0);


    Line2d              l_trans;
    assert(translate<Point2d>(LINE_A, trans, l_trans));
    assert(l_trans.first.x()  == -1.0);
    assert(l_trans.first.y()  ==  2.0);
    assert(l_trans.second.x() ==  0.0);
    assert(l_trans.second.y() ==  3.0);

    PointSet2d          ps_trans;
    assert(translate<Point2d>(POINTS_A, trans, ps_trans));
    assert(ps_trans.size() == POINTS_A.size());
    for(unsigned int i = 0 ; i < POINTS_A.size() ; ++i) {
        assert(ps_trans.at(i).x() == POINTS_A.at(i).x() - 1.0);
        assert(ps_trans.at(i).y() == POINTS_A.at(i).y() + 2.0);
    }

    LineSet2d           ls_trans;
    assert(translate<Point2d>(LINES_A, trans, ls_trans));
    for(unsigned int i = 0 ; i < LINES_A.size() ; ++i) {
        assert(ls_trans.at(i).first.x() == LINES_A.at(i).first.x()   - 1.0);
        assert(ls_trans.at(i).first.y() == LINES_A.at(i).first.y()   + 2.0);
        assert(ls_trans.at(i).second.x() == LINES_A.at(i).second.x() - 1.0);
        assert(ls_trans.at(i).second.y() == LINES_A.at(i).second.y() + 2.0);
    }
    std::cout << "TEST 4 PASSED" << std::endl;
}

void TEST_5_POLAR_LINE_SET ()
{
    assert(rad(90)     == M_PI_2);
    assert(rad(180)    == M_PI);
    assert(deg(M_PI)   == 180);
    assert(deg(M_PI_2) == 90);

    LineSet2d set;
    polarLineSet(Point2d(0.0, 0.0),
                 rad(0.0),
                 rad(180),
                 M_PI_2,
                 1.0,
                 set);
    assert(set.size() == 3);
    for(unsigned int i = 0 ; i < 3 ; ++i) {
        assert(set.at(i).first.x() == 0);
        assert(set.at(i).first.y() == 0);
    }
    assert(equal(set.at(0).second.x(),   0.0, 1.0E-6));
    assert(equal(set.at(0).second.y(),  -1.0, 1.0E-6));
    assert(equal(set.at(1).second.x(),   1.0, 1.0E-6));
    assert(equal(set.at(1).second.y(),   0.0, 1.0E-6));
    assert(equal(set.at(2).second.x(),   0.0, 1.0E-6));
    assert(equal(set.at(2).second.y(),   1.0, 1.0E-6));

    set.clear();
    polarLineSet(Point2d(0.0, 0.0),
                 rad(0.0),
                 rad(270),
                 rad(0.5),
                 1.0,
                 set);
    assert(set.size() == 541);
    for(unsigned int i = 0 ; i < 3 ; ++i) {
        assert(set.at(i).first.x() == 0);
        assert(set.at(i).first.y() == 0);
    }

    assert(equal(-std::acos(set.front().second.x()),        rad(-135.0), 1.0E-6));
    assert(equal(-std::asin(set.front().second.y()) - M_PI, rad(-135.0), 1.0E-6));
    assert(equal( std::acos(set.back().second.x()),         rad( 135.0), 1.0E-6));
    assert(equal(-std::asin(set.back().second.y())  + M_PI, rad( 135.0), 1.0E-6));
    assert(equal(set.at(270).second.x(), 1.0, 1.0E-6));
    assert(equal(set.at(270).second.y(), 0.0, 1.0E-6));

    set.clear();
    double range = 3.0;
    polarLineSet(Point2d(0.0, 0.0),
                 rad(0.0),
                 rad(180),
                 M_PI_2,
                 3.0,
                 set);
    assert(set.size() == 3);
    for(unsigned int i = 0 ; i < 3 ; ++i) {
        double x_o = set.at(i).first.x();
        double y_o = set.at(i).first.y();
        double x_d = set.at(i).second.x();
        double y_d = set.at(i).second.y();

        assert(x_o == 0);
        assert(y_o == 0);
        assert(equal(std::sqrt(x_d * x_d + y_d * y_d), 3.0, 1.0E-6));
    }
    std::cout << "TEST 5 PASSED" << std::endl;
}

void TEST_6_POLYGONS()
{

    Polygon2d polygon;
    circularPolygonApproximation(Point2d(0.0,0.0),
                                 1.0,
                                 rad(1.0),
                                 polygon);

    Polygon2d::ring_type &ring = polygon.outer();
    assert(ring.size() == 361);
    assert(equal(ring.front().x(), 1.0, 1.0E-6));
    assert(equal(ring.back().x(),  1.0, 1.0E-6));
    assert(equal(ring.front().y(), 0.0, 1.0E-6));
    assert(equal(ring.back().y(),  0.0, 1.0E-6));
    assert(ring.at(0).x() > ring.at(1).x());
    assert(ring.at(0).y() > ring.at(1).y());
    assert(equal(ring.at(90).x(),   0.0, 1.0E-6));
    assert(equal(ring.at(90).y(),  -1.0, 1.0E-6));
    assert(equal(ring.at(180).x(), -1.0, 1.0E-6));
    assert(equal(ring.at(180).y(),  0.0, 1.0E-6));
    assert(equal(ring.at(270).x(),  0.0, 1.0E-6));
    assert(equal(ring.at(270).y(),  1.0, 1.0E-6));
    assert(equal(boost::geometry::area(polygon), M_PI, 1.0E-3));
    assert(withinExcl<Point2d>(POINT_B, polygon));
    assert(intersects<Point2d>(LINE_B, polygon));
    assert(touches<Point2d>(LINE_B, polygon));
    assert(!touches<Point2d>(LINE_E, polygon));
    assert(!touches<Point2d>(LINE_F, polygon));

    std::cout << "TEST 6 PASSED" << std::endl;
}

int main(int argc, char *argv[])
{
    TEST_1_INTERSECT();
    TEST_2_INTERSECTIONS();
    TEST_3_INTERSECTIONS();
    TEST_4_TRANSLATION();
    TEST_5_POLAR_LINE_SET();
    TEST_6_POLYGONS();
    return 0;
}

