#include <utils_boost_geometry/algorithms.hpp>

using namespace utils_boost_geometry;
using namespace algorithms;
using namespace types;

template
bool utils_boost_geometry::algorithms::intersection<Point2d>
(const types::Line<Point2d>::type &line_a,
 const types::Line<Point2d>::type &line_b,
       types::PointSet<Point2d>::type   &points);

template
bool utils_boost_geometry::algorithms::intersection<Point2d>
    (const typename types::Line<Point2d>::type       &line_a,
     const typename types::Polygon<Point2d>::type    &polygon,
           typename types::PointSet<Point2d>::type   &points);

template
bool utils_boost_geometry::algorithms::intersects<Point2d>
    (const typename types::Line<Point2d>::type &line_a,
     const typename types::Line<Point2d>::type &line_b);

template
bool utils_boost_geometry::algorithms::intersects<Point2d>
    (const typename types::Line<Point2d>::type    &line_a,
     const typename types::Polygon<Point2d>::type &polygon);


template
bool utils_boost_geometry::algorithms::nearestIntersection<Point2d>
    (const typename types::Line<Point2d>::type          &line_a,
     const typename types::LineSet<Point2d>::type       &lines_b,
           typename types::PointSet<Point2d>::type      &points);

template
bool utils_boost_geometry::algorithms::nearestIntersection<Point2d>
    (const typename types::Line<Point2d>::type           &line_a,
     const typename types::IndexedLineSet<Point2d>::type &lines_b,
           typename types::PointSet<Point2d>::type       &points);

template
void utils_boost_geometry::algorithms::multiNearestIntersection<Point2d>
    (const typename types::LineSet<Point2d>::type            &lines_a,
     const typename types::LineSet<Point2d>::type            &lines_b,
           typename types::ValidatedResultSet<Point2d>::type &results);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const Point2d                                    &src_point,
     const typename types::Translation<Point2d>::type &translation,
           Point2d                                    &dst_point);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const typename types::Line<Point2d>::type        &src_line,
     const typename types::Translation<Point2d>::type &translation,
           typename types::Line<Point2d>::type        &dst_line);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const typename types::PointSet<Point2d>::type    &src_points,
     const typename types::Translation<Point2d>::type &translation,
           typename types::PointSet<Point2d>::type    &dst_points);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const typename types::LineSet<Point2d>::type     &src_lines,
     const typename types::Translation<Point2d>::type &translation,
           typename types::LineSet<Point2d>::type     &dst_lines);

template
bool utils_boost_geometry::algorithms::equal<double>
    (const double value_1,
     const double value_2,
     const double epsilon = 0.0);

template
bool utils_boost_geometry::algorithms::withinExcl<Point2d>
    (const Point2d &p,
     const typename types::Polygon<Point2d>::type &polygon);

template
bool utils_boost_geometry::algorithms::withinExcl<Point2d>
    (const typename types::Line<Point2d>::type &line,
     const typename types::Box<Point2d>::type &box);

template
bool utils_boost_geometry::algorithms::lessEqual<Point2d>
    (const Point2d &p1,
     const Point2d &p2);

template
bool utils_boost_geometry::algorithms::greaterEqual<Point2d>
    (const Point2d &p1,
     const Point2d &p2);

template
bool utils_boost_geometry::algorithms::equal<Point2d>
    (const Point2d &p1,
     const Point2d &p2);


template
bool utils_boost_geometry::algorithms::withinIncl<Point2d>
    (const Point2d &p,
     const typename types::Box<Point2d>::type &box);

template
bool utils_boost_geometry::algorithms::withinIncl<Point2d>
    (const typename types::Line<Point2d>::type &line,
     const typename types::Box<Point2d>::type  &box);

template
bool utils_boost_geometry::algorithms::withinIncl<Point2d>
    (const typename types::Box<Point2d>::type &inner,
     const typename types::Box<Point2d>::type &outer);

template
bool utils_boost_geometry::algorithms::touches<Point2d>
    (const typename types::Line<Point2d>::type     &line,
     const typename types::Polygon<Point2d>::type  &polygon);

template
bool utils_boost_geometry::algorithms::touches<Point2d>
    (const typename types::Line<Point2d>::type &line,
     const typename types::Box<Point2d>::type &box);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
     typename types::LineSet<Point2d>::type &lines);

template
void utils_boost_geometry::algorithms::circularPolygonApproximation<Point2d>
    (const Point2d &center,
     const double  radius,
     const double  ang_res,
     typename types::Polygon<Point2d>::type &polygon);
