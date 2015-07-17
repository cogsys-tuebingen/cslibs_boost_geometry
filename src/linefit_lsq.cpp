#include <utils_boost_geometry/linefit_lsq.h>
#include <utils_boost_geometry/linefit_lsq.hpp>

using namespace utils_boost_geometry;
using namespace algorithms;
using namespace types;

template
void linefitLSQ<Point2d, double>
    (const PointSet<Point2d>::type &points,
     const double delta_d,
     const double delta_var,
     const double delta_ang,
     LineSet<Point2d>::type &lines);

template
void linefitLSQ<Point2d, float>
    (const PointSet<Point2d>::type &points,
     const float delta_d,
     const float delta_var,
     const float delta_ang,
     LineSet<Point2d>::type &lines);

template
void linefitPolarLSQ<Point2d, float, double>
    (std::vector<float> &rhos,
     const float angle_min,
     const float angle_incr,
     const float range_min,
     const float range_max,
     const double delta_d,
     const double delta_var,
     const double delta_ang,
     types::LineSet<Point2d>::type &lines);

template
void linefitPolarLSQ<Point2d, float, float>
    (std::vector<float> &rhos,
     const float angle_min,
     const float angle_incr,
     const float range_min,
     const float range_max,
     const float delta_d,
     const float delta_var,
     const float delta_ang,
     types::LineSet<Point2d>::type &lines);
