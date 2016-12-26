#include <cslibs_boost_geometry/algorithms.hpp>

using namespace cslibs_boost_geometry;
using namespace algorithms;
using namespace types;

namespace bsa = cslibs_boost_geometry::algorithms;

template
bool bsa::intersection<Point2d>
(const types::Line<Point2d>::type &line_a,
 const types::Line<Point2d>::type &line_b,
       types::PointSet<Point2d>::type   &points);

template
bool bsa::intersection<Point2d>
    (const types::Line<Point2d>::type       &line_a,
     const types::Polygon<Point2d>::type    &polygon,
           types::PointSet<Point2d>::type   &points);

template
double bsa::distance<double,Point2d>
    (const Point2d                             &point,
     const types::Line<Point2d>::type &line);

template
double bsa::minEndPointDistance<double,Point2d>
    (const types::Line<Point2d>::type &line_a,
     const types::Line<Point2d>::type &line_b);

template
double bsa::length<double, Point2d>(const types::Line<Point2d>::type &line);

template
bool bsa::intersects<Point2d>
    (const  types::Line<Point2d>::type &line_a,
     const  types::Line<Point2d>::type &line_b);

template
bool bsa::intersects<Point2d>
    (const  types::Line<Point2d>::type    &line_a,
     const  types::Polygon<Point2d>::type &polygon);


template
bool bsa::nearestIntersection<Point2d>
    (const  types::Line<Point2d>::type          &line_a,
     const  types::LineSet<Point2d>::type       &lines_b,
            types::PointSet<Point2d>::type      &points);

template
bool bsa::nearestIntersection<Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
            types::PointSet<Point2d>::type       &points);

template
bool bsa::nearestIntersection<Point2d>
    (const  types::Line<Point2d>::type          &line_a,
     const  types::LineSet<Point2d>::type       &lines_b,
            types::PointSet<Point2d>::type      &points,
            types::Line<Point2d>::type          &line_b);

template
bool bsa::nearestIntersection<Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
            types::PointSet<Point2d>::type       &points,
            types::Line<Point2d>::type          &line_b);

template
double bsa::nearestIntersectionDistance<double, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
     const double default_value);

template
float bsa::nearestIntersectionDistance<float, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::LineSet<Point2d>::type        &lines_b,
     const float default_value);


template
double bsa::nearestIntersectionDistance<double, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::LineSet<Point2d>::type        &lines_b,
     const double    default_value);


template
float bsa::nearestIntersectionDistance<float, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
     const float default_value);


template
void bsa::nearestIntersectionDistance<double, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
     double &distance,
     double &angle,
     const double default_distance,
     const double default_angle);

template
void bsa::nearestIntersectionDistance<float, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::LineSet<Point2d>::type        &lines_b,
     float &distance,
     float &angle,
     const float default_distance,
     const float default_angle);

template
void bsa::nearestIntersectionDistance<double, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::LineSet<Point2d>::type        &lines_b,
     double &distance,
     double &angle,
     const double default_distance,
     const double default_angle);


template
void bsa::nearestIntersectionDistance<float, Point2d>
    (const  types::Line<Point2d>::type           &line_a,
     const  types::IndexedLineSet<Point2d>::type &lines_b,
     float &distance,
     float &angle,
     const float default_distance,
     const float default_angle);

template
void bsa::nearestIntersectionDistanceBatch<double, Point2d>
    (const LineSet<Point2d>::type &lines_a,
     const LineSet<Point2d>::type &lines_b,
     const double default_value,
     std::vector<double> &results);

template
void bsa::nearestIntersectionDistanceBatch<float, Point2d>
    (const LineSet<Point2d>::type &lines_a,
     const LineSet<Point2d>::type &lines_b,
     const float default_value,
     std::vector<float> &results);

template
void bsa::nearestIntersectionBatch<Point2d>
    (const  types::LineSet<Point2d>::type            &lines_a,
     const  types::LineSet<Point2d>::type            &lines_b,
            types::IntersectionResultSet<Point2d>::type &results);

template
bool bsa::translate<Point2d>
    (const Point2d                                    &src_point,
     const  types::Translation<Point2d>::type &translation,
           Point2d                                    &dst_point);

template
bool bsa::translate<Point2d>
    (const  types::Line<Point2d>::type        &src_line,
     const  types::Translation<Point2d>::type &translation,
            types::Line<Point2d>::type        &dst_line);

template
bool bsa::translate<Point2d>
    (const  types::PointSet<Point2d>::type    &src_points,
     const  types::Translation<Point2d>::type &translation,
            types::PointSet<Point2d>::type    &dst_points);

template
bool bsa::translate<Point2d>
    (const  types::LineSet<Point2d>::type     &src_lines,
     const  types::Translation<Point2d>::type &translation,
            types::LineSet<Point2d>::type     &dst_lines);

template
bool bsa::rotate<Point2d>
    (const Point2d                         &src_point,
     const types::Rotation<Point2d>::type &translation,
           Point2d                         &dst_point);

template
bool bsa::rotate<Point2d>
    (const types::Line<Point2d>::type     &src_linet,
     const types::Rotation<Point2d>::type &translation,
           types::Line<Point2d>::type     &dst_line);

template
double bsa::dot<double, Point2d>
    (const types::Line<Point2d>::type &line_a,
     const types::Line<Point2d>::type &line_b);

template
double bsa::angle<double, Point2d>
    (const types::Line<Point2d>::type &line_a,
     const types::Line<Point2d>::type &line_b);

template
double bsa::angle<double, Point2d>
    (const types::Line<Point2d>::type &line_a,
     const types::Line<Point2d>::type &line_b,
     const double eps);


template
bool bsa::equal<double>
    (const double value_1,
     const double value_2,
     const double epsilon = 0.0);

template
bool bsa::withinExcl<Point2d>
    (const Point2d &p,
     const  types::Polygon<Point2d>::type &polygon);

template
bool bsa::withinExcl<Point2d>
    (const  types::Line<Point2d>::type &line,
     const  types::Box<Point2d>::type &box);

template
bool bsa::lessEqual<Point2d>
    (const Point2d &p1,
     const Point2d &p2);

template
bool bsa::greaterEqual<Point2d>
    (const Point2d &p1,
     const Point2d &p2);

template
bool bsa::equal<Point2d, double>
    (const Point2d &p1,
     const Point2d &p2,
     const double eps);

template
bool bsa::equal<Point2d, double>
    (const Line2d &l1,
     const Line2d &l2,
     const double eps);

template
bool bsa::withinIncl<Point2d>
    (const Point2d &p,
     const  types::Box<Point2d>::type &box);

template
bool bsa::withinIncl<Point2d>
    (const  types::Line<Point2d>::type &line,
     const  types::Box<Point2d>::type  &box);

template
bool bsa::withinIncl<Point2d>
    (const  types::Box<Point2d>::type &inner,
     const  types::Box<Point2d>::type &outer);

template
bool bsa::within<Point2d>
(const types::Polygon<Point2d>::type &inner,
 const types::Polygon<Point2d>::type &outer);

template
bool bsa::covered_by<Point2d>
(const types::Polygon<Point2d>::type &covered,
 const types::Polygon<Point2d>::type &by);

template
bool bsa::touches<Point2d>
    (const  types::Line<Point2d>::type     &line,
     const  types::Polygon<Point2d>::type  &polygon);

template
bool bsa::touches<Point2d>
    (const  types::Line<Point2d>::type &line,
     const  types::Box<Point2d>::type &box);

template
types::Polygon<Point2d>::type
bsa::toPolygon<Point2d>
    (const Point2d &min,
     const Point2d &max);

template
types::Polygon<Point2d>::type
bsa::toPolygon<Point2d>
    (const types::Box<Point2d>::type &box);

template
void bsa::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void bsa::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void bsa::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);

template
void bsa::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const double angular_resolution,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);


template
void bsa::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void bsa::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines);

template
void bsa::polarLineSet<Point2d, periodic>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);

template
void bsa::polarLineSet<Point2d, periodicApprox>
    (const Point2d &center,
     const double center_line_orientation,
     const double opening_angle,
     const unsigned int num_rays,
     const double length,
      types::LineSet<Point2d>::type &lines,
      std::vector<double> &angles);


template
void bsa::circularPolygonApproximation<Point2d>
    (const Point2d &center,
     const double  radius,
     const double  ang_res,
      types::Polygon<Point2d>::type &polygon);
