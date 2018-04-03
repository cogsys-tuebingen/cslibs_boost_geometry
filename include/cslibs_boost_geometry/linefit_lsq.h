#ifndef LINEFIT_LSQ_H
#define LINEFIT_LSQ_H

#include "types.hpp"

#include <vector>
#include <utility>

namespace cslibs_boost_geometry {
namespace algorithms {

/**
 * @brief linefitLSQ will fit lines into a point set. The lines fulfill the conditions
 *        formulated through delta_d and delta_var. Points inserted here, have to be
 *        valid.
 * @param points
 * @param lines
 */
template<typename PointT, typename T>
void linefitLSQ(const typename types::PointSet<PointT>::type  &points,
                const T delta_d,
                const T delta_var,
                const T delta_ang,
                typename types::LineSet<PointT>::type         &lines);

template<typename PointT, typename T>
void linefitLSQ(const typename types::PointSet<PointT>::type  &points,
                const T delta_d,
                const T delta_var,
                const T delta_ang,
                typename types::LineSet<PointT>::type         &lines,
                std::vector<std::pair<double,double>>         &angles);

}
}


#endif // LINEFIT_LSQ_H

