#ifndef LINEFIT_LSQ_H
#define LINEFIT_LSQ_H

#include "types.hpp"

namespace utils_boost_geometry {
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

/**
 * @brief linefitPolarLSQ will fit lines like linefitLSQ. The only difference is, that points are
          introduced using angular equidistant polar point coordinates.
 * @param rhos
 * @param angle_min
 * @param angle_incr
 * @param range_min
 * @param range_max
 * @param delta_d
 * @param delta_var
 * @param delta_ang
 * @param lines
 */
template<typename PointT, typename T, typename DeltaT>
void linefitPolarLSQ(std::vector<T> &rhos,
                     const T angle_min,
                     const T angle_incr,
                     const T range_min,
                     const T range_max,
                     const DeltaT delta_d,
                     const DeltaT delta_var,
                     const DeltaT delta_ang,
                     typename types::LineSet<PointT>::type &lines);
}
}


#endif // LINEFIT_LSQ_H

