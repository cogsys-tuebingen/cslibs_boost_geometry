#ifndef POINTS_AND_LINES_HPP
#define POINTS_AND_LINES_HPP

#include <cslibs_boost_geometry/types.hpp>

namespace cslibs_boost_geometry {
namespace test_samples {

const static types::Point2d point_a(0.0, 0.0);
const static types::Point2d point_b(1.0, 1.0);
const static types::Point2d point_c(0.0, 3.0);
const static types::Point2d point_d(0.0, 1.0);
const static types::Point2d point_e(1.0, -1.0);
const static types::Point2d point_f(-1.0, 0.0);
const static types::Point2d point_g(-1.0, -1.0);

const static types::PointSet2d points_a = {
    types::Point2d(0.0,0.0),
    types::Point2d(2.0,3.0)
};
const static types::PointSet2d points_b = {
    types::Point2d(-1.0,2.0),
    types::Point2d( 1.0,5.0)
};

const static types::Line2d line_a(types::Point2d(0.0, 0.0),
                                  types::Point2d(1.0, 1.0));
const static types::Line2d line_b(types::Point2d(0.0, 1.0),
                                  types::Point2d(1.0, 0.0));
const static types::Line2d line_c(types::Point2d(0.0, 0.0),
                                  types::Point2d(2.0, 2.0));
const static types::Line2d line_d(types::Point2d(0.0, 1.0),
                                  types::Point2d(0.0, 2.0));
const static types::Line2d line_e(types::Point2d(-10.0, 2.0),
                                  types::Point2d(-11.0, 3.0));
const static types::Line2d line_f(types::Point2d(0.0, 2.0),
                                  types::Point2d(2.0, 0.0));
const static types::Line2d line_g(types::Point2d(0.5,0.5),
                                  types::Point2d(3.0,3.0));
const static types::Line2d line_h(types::Point2d(0.0,0.5),
                                  types::Point2d(3.0,3.5));
const static types::Line2d line_i(types::Point2d(0.5,0.0),
                                  types::Point2d(3.5,3.0));
const static types::Line2d line_j(types::Point2d(0.0,4.0),
                                  types::Point2d(4.0,0.0));
const static types::Line2d line_k(types::Point2d(-1.0, 2.0),
                                  types::Point2d(0.0, 3.0));
const static types::Line2d line_l(types::Point2d(1.0, 0.0),
                                  types::Point2d(1.0, 2.0));
const static types::Line2d line_m(types::Point2d(2.0, 0.0),
                                  types::Point2d(2.0, 2.0));
const static types::Line2d line_n(types::Point2d(0.0, 1.0),
                                  types::Point2d(3.0, 1.0));
const static types::Line2d line_o(types::Point2d(0.0, 3.0),
                                  types::Point2d(3.0, 3.0));
const static types::Line2d line_p(types::Point2d(0.0, 0.5),
                                  types::Point2d(3.0, 0.5));

const static types::Line2dSet lines_a = {
    line_b,
    line_c,
    line_d
};
const static types::Line2dSet lines_b = {
    line_d,
    line_e
};
const static types::Line2dSet lines_c = {
    line_b,
    line_f
};
const static types::Line2dSet lines_d = {
    line_h,
    line_i
};
const static types::Line2dSet lines_e = {
    line_a,
    line_c
};
const static types::Line2dSet lines_f = {
    line_e,
    line_j
};
const static types::Line2dSet lines_g = {
    line_a,
    line_b
};
const static types::Line2dSet lines_h = {
    line_l,
    line_m
};
const static types::Line2dSet lines_i = {
    line_n,
    line_o,
    line_p
};
}
}


#endif // POINTS_AND_LINES_HPP
