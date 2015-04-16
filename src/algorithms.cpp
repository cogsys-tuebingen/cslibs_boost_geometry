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
    (const types::Line<Point2d>::type       &line_a,
     const types::Polygon<Point2d>::type    &polygon,
           types::PointSet<Point2d>::type   &points);

template
double utils_boost_geometry::algorithms::distance<double,Point2d>
    (const Point2d                             &point,
     const types::Line<Point2d>::type &line);

template
bool utils_boost_geometry::algorithms::intersects<Point2d>
    (const  types::Line<Point2d>::type &line_a,
     const  types::Line<Point2d>::type &line_b);

template
bool utils_boost_geometry::algorithms::intersects<Point2d>
    (const  types::Line<Point2d>::type    &line_a,
     const  types::Polygon<Point2d>::type &polygon);


template
bool utils_boost_geometry::algorithms::nearestIntersection<Point2d>
    (const  types::Line<Point2d>::type          &line_a,
     const  types::LineSet<Point2d>::type       &lines_b,
            types::PointSet<Point2d>::type      &points);

template
bool utils_boost_geometry::algorithms::nearestIntersection<Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
            types::PointSet<Point2d>::type       &points);

template
double utils_boost_geometry::algorithms::nearestIntersectionDist<double, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
     const double default_value);

template
float utils_boost_geometry::algorithms::nearestIntersectionDist<float, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::LineSet<Point2d>::type        &lines_b,
     const float default_value);


template
double utils_boost_geometry::algorithms::nearestIntersectionDist<double, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::LineSet<Point2d>::type        &lines_b,
     const double    default_value);


template
float utils_boost_geometry::algorithms::nearestIntersectionDist<float, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
     const float default_value);

template
void utils_boost_geometry::algorithms::multiNearestIntersectionDist<double, Point2d>
    (const LineSet<Point2d>::type &lines_a,
     const LineSet<Point2d>::type &lines_b,
     const double default_value,
     std::vector<double> &results);

template
void utils_boost_geometry::algorithms::multiNearestIntersectionDist<float, Point2d>
    (const LineSet<Point2d>::type &lines_a,
     const LineSet<Point2d>::type &lines_b,
     const float default_value,
     std::vector<float> &results);

template
void utils_boost_geometry::algorithms::multiNearestIntersection<Point2d>
    (const  types::LineSet<Point2d>::type            &lines_a,
     const  types::LineSet<Point2d>::type            &lines_b,
            types::ValidatedResultSet<Point2d>::type &results);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const Point2d                                    &src_point,
     const  types::Translation<Point2d>::type &translation,
           Point2d                                    &dst_point);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const  types::Line<Point2d>::type        &src_line,
     const  types::Translation<Point2d>::type &translation,
            types::Line<Point2d>::type        &dst_line);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const  types::PointSet<Point2d>::type    &src_points,
     const  types::Translation<Point2d>::type &translation,
            types::PointSet<Point2d>::type    &dst_points);

template
bool utils_boost_geometry::algorithms::translate<Point2d>
    (const  types::LineSet<Point2d>::type     &src_lines,
     const  types::Translation<Point2d>::type &translation,
            types::LineSet<Point2d>::type     &dst_lines);

template
bool utils_boost_geometry::algorithms::equal<double>
    (const double value_1,
     const double value_2,
     const double epsilon = 0.0);

template
bool utils_boost_geometry::algorithms::withinExcl<Point2d>
    (const Point2d &p,
     const  types::Polygon<Point2d>::type &polygon);

template
bool utils_boost_geometry::algorithms::withinExcl<Point2d>
    (const  types::Line<Point2d>::type &line,
     const  types::Box<Point2d>::type &box);

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
     const  types::Box<Point2d>::type &box);

template
bool utils_boost_geometry::algorithms::withinIncl<Point2d>
    (const  types::Line<Point2d>::type &line,
     const  types::Box<Point2d>::type  &box);

template
bool utils_boost_geometry::algorithms::withinIncl<Point2d>
    (const  types::Box<Point2d>::type &inner,
     const  types::Box<Point2d>::type &outer);

template
bool utils_boost_geometry::algorithms::within<Point2d>
(const types::Polygon<Point2d>::type &inner,
 const types::Polygon<Point2d>::type &outer);

template
bool utils_boost_geometry::algorithms::covered_by<Point2d>
(const types::Polygon<Point2d>::type &covered,
 const types::Polygon<Point2d>::type &by);

template
bool utils_boost_geometry::algorithms::touches<Point2d>
    (const  types::Line<Point2d>::type     &line,
     const  types::Polygon<Point2d>::type  &polygon);

template
bool utils_boost_geometry::algorithms::touches<Point2d>
    (const  types::Line<Point2d>::type &line,
     const  types::Box<Point2d>::type &box);

template
types::Polygon<Point2d>::type
utils_boost_geometry::algorithms::toPolygon<Point2d>
    (const Point2d &min,
     const Point2d &max);

template
types::Polygon<Point2d>::type
utils_boost_geometry::algorithms::toPolygon<Point2d>
    (const types::Box<Point2d>::type &box);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);


template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);

template
void utils_boost_geometry::algorithms::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);


template
void utils_boost_geometry::algorithms::circularPolygonApproximation<Point2d>
    (const Point2d &center,
     const double  radius,
     const double  ang_res,
      types::Polygon<Point2d>::type &polygon);
