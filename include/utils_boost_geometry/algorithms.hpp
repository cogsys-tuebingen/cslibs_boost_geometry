#ifndef DXF_ALGORITHMS_HPP
#define DXF_ALGORITHMS_HPP

#include "types.hpp"

/// SYSTEM
#include <boost/version.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>

namespace utils_boost_geometry {
namespace algorithms {
template<typename PointT>
inline bool intersection
(const typename types::Line<PointT>::type       &line_a,
 const typename types::Line<PointT>::type       &line_b,
 typename types::PointSet<PointT>::type   &points)
{
    boost::geometry::intersection(line_a, line_b, points);
    return points.size() > 0;
}


template<typename PointT>
inline bool intersection
(const typename types::Line<PointT>::type       &line_a,
 const typename types::Polygon<PointT>::type    &polygon,
 typename types::PointSet<PointT>::type   &points)
{
    auto &ring = polygon.outer();
    auto it_first = ring.begin();
    auto it_second = it_first + 1;
    while(it_second != ring.end()) {
        typename types::Line<PointT>::type line_b(*it_first, *it_second);
        typename types::PointSet<PointT>::type tmp;
        if(intersection<PointT>(line_a, line_b, tmp)) {
            if(tmp.size() == 2) {
                points.assign(tmp.begin(), tmp.end());
                return true;
            } else {
                points.push_back(tmp.front());
            }
        }
        ++it_first;
        ++it_second;
    }
    return points.size() > 0;
}


template<typename T,
         typename PointT>
inline T distance(const PointT &point,
                  const typename types::Line<PointT>::type &line)

{
    return boost::geometry::distance(point, line);
}

template<typename T,
         typename PointT>
inline T distance(const typename types::Line<PointT>::type &line_a,
                  const typename types::Line<PointT>::type &line_b)
{
    T min = std::numeric_limits<T>::max();
    const T d0 = boost::geometry::distance(line_a.first,  line_b);
    if(d0 < min)
        min = d0;
    const T d1 = boost::geometry::distance(line_a.second, line_b);
    if(d1 < min)
        min = d1;
    const T d2 = boost::geometry::distance(line_a,  line_b.second);
    if(d2 < min)
        min = d2;
    const T d3 = boost::geometry::distance(line_a, line_b.first);
    if(d3 < min)
        min = d3;
    return min;
}

template<typename PointT>
inline bool intersects
(const typename types::Line<PointT>::type &line_a,
 const typename types::Line<PointT>::type &line_b)

{
    return boost::geometry::intersects(line_a, line_b);
}

template<typename PointT>
inline bool intersects
(const typename types::Line<PointT>::type    &line_a,
 const typename types::Polygon<PointT>::type &polygon)
{
#ifdef _OPENMP
    auto &ring = polygon.outer();

    if(ring.size() < 2)
        return false;

    auto ring_ptr    = ring.data();
    typename types::Line<PointT>::type line_b(*(ring_ptr + ring.size() - 1), *ring_ptr);
    bool intersection = intersects<PointT>(line_a, line_b);

#pragma omp parallel for reduction(||:intersection)
    for(unsigned int i = 0 ; i < ring.size() - 1 ; ++i) {
        line_b.first  = *(ring_ptr + i);
        line_b.second = *(ring_ptr + i + 1);
        intersection |= intersects<PointT>(line_a, line_b);
    }
    return intersection;
#else
    auto &ring = polygon.outer();
    auto it_first = ring.begin();
    auto it_second = it_first + 1;

    while(it_second != ring.end()) {
        typename types::Line<PointT>::type line_b(*it_first, *it_second);
        if(intersects<PointT>(line_a, line_b))
            return true;
        ++it_first;
        ++it_second;
    }
    return false;
#endif
}


template<typename PointT, template <typename> class Set>
inline bool nearestIntersection
(const typename types::Line<PointT>::type    &line_a,
 const typename Set<PointT>::type            &lines_b,
 typename types::PointSet<PointT>::type      &points)
{
    if(lines_b.size() == 0)
        return false;

    const PointT &origin = line_a.first;
    double min = std::numeric_limits<double>::max();
    double dx(0.0), dy(0.0), dsq(0.0);
    typename types::PointSet<PointT>::type tmp_points;

    for(auto it =
        lines_b.begin() ;
        it != lines_b.end() ;
        ++it) {
        tmp_points.clear();
        if(intersection<PointT>(line_a, Set<PointT>::getSegment(it), tmp_points)) {
            if(tmp_points.size() == 2) {
                std::swap(tmp_points, points);
                return true;
            }
            if(tmp_points.size() == 1) {
                const PointT &intersection = tmp_points.back();
                dx  = origin.x() - intersection.x();
                dy  = origin.y() - intersection.y();
                dsq = dx * dx + dy * dy;
                if(dsq < min) {
                    min = dsq;
                    std::swap(tmp_points, points);
                }
            }
        }
    }

    return points.size() > 0;
}

template<typename PointT>
inline bool nearestIntersection
(const typename types::Line<PointT>::type    &line_a,
 const typename types::LineSet<PointT>::type &lines_b,
 typename types::PointSet<PointT>::type      &points)
{
    return nearestIntersection<PointT, types::LineSet> (line_a, lines_b, points);
}

template<typename PointT>
inline bool nearestIntersection
(const typename types::Line<PointT>::type           &line_a,
 const typename types::IndexedLineSet<PointT>::type &lines_b,
 typename types::PointSet<PointT>::type       &points)
{
    return nearestIntersection<PointT, types::IndexedLineSet> (line_a, lines_b, points);
}


template<typename PointT, template <typename> class Set>
inline bool nearestIntersection
(const typename types::Line<PointT>::type    &line_a,
 const typename Set<PointT>::type            &lines_b,
 typename types::PointSet<PointT>::type      &points,
 typename types::Line<PointT>::type          &line_b)
{
    if(lines_b.size() == 0)
        return false;

    const PointT &origin = line_a.first;
    double min = std::numeric_limits<double>::max();
    double dx(0.0), dy(0.0), dsq(0.0);
    typename types::PointSet<PointT>::type tmp_points;

    for(auto it =
        lines_b.begin() ;
        it != lines_b.end() ;
        ++it) {
        tmp_points.clear();
        const typename types::Line<PointT>::type &line = Set<PointT>::getSegment(it);
        if(intersection<PointT>(line_a, line, tmp_points)) {
            if(tmp_points.size() == 2) {
                std::swap(tmp_points, points);
                return true;
            }
            if(tmp_points.size() == 1) {
                const PointT &intersection = tmp_points.back();
                dx  = origin.x() - intersection.x();
                dy  = origin.y() - intersection.y();
                dsq = dx * dx + dy * dy;
                if(dsq < min) {
                    min = dsq;
                    line_b = line;
                    std::swap(tmp_points, points);
                }
            }
        }
    }

    return points.size() > 0;
}

template<typename PointT>
inline bool nearestIntersection
(const typename types::Line<PointT>::type    &line_a,
 const typename types::LineSet<PointT>::type &lines_b,
 typename types::PointSet<PointT>::type      &points,
 typename types::Line<PointT>::type          &line_b)
{
    return nearestIntersection<PointT, types::LineSet> (line_a, lines_b, points, line_b);
}

template<typename PointT>
inline bool nearestIntersection
(const typename types::Line<PointT>::type    &line_a,
 const typename types::IndexedLineSet<PointT>::type &lines_b,
 typename types::PointSet<PointT>::type      &points,
 typename types::Line<PointT>::type          &line_b)
{
    return nearestIntersection<PointT, types::IndexedLineSet> (line_a, lines_b, points, line_b);
}

template<typename PointT,
         typename T,
         template <typename> class Set>
inline T nearestIntersectionDist
(const typename types::Line<PointT>::type    &line_a,
 const typename Set<PointT>::type            &lines_b,
 const T default_value)
{
    typename types::PointSet<PointT>::type  points;
    nearestIntersection<PointT, Set> (line_a, lines_b, points);

    if(points.size() == 0)
        return default_value;

    typename types::Line<PointT>::type line;
    line.first  = line_a.first;
    line.second = points.front();
    return boost::geometry::length(line);
}


template<typename T,
         typename PointT>
inline T nearestIntersectionDist
(const typename types::Line<PointT>::type    &line_a,
 const typename types::LineSet<PointT>::type &lines_b,
 const T default_value)
{
    return nearestIntersectionDist<PointT, T, types::LineSet>(line_a, lines_b, default_value);
}

template<typename T,
         typename PointT>
inline T nearestIntersectionDist
(const typename types::Line<PointT>::type           &line_a,
 const typename types::IndexedLineSet<PointT>::type &lines_b,
 const T default_value)
{
    return nearestIntersectionDist<PointT, T, types::IndexedLineSet>(line_a, lines_b, default_value);
}

template<typename PointT,
         typename T,
         template <typename> class Set>
inline void nearestIntersectionDist
(const typename types::Line<PointT>::type    &line_a,
 const typename Set<PointT>::type            &lines_b,
 T &distance,
 T &angle,
 const T default_distance,
 const T default_angle)
{
    typename types::PointSet<PointT>::type  points;
    typename types::Line<PointT>::type      line_b;
    nearestIntersection<PointT, Set> (line_a, lines_b, points, line_b);

    if(points.size() == 0) {
        distance = default_distance;
        angle = default_angle;
        return;
    }

    typename types::Line<PointT>::type line(line_a.first,
                                            points.front());
    distance = boost::geometry::length(line);

    PointT diff_a(line_a.first.x() - line_a.second.x(),
                  line_a.first.y() - line_a.second.y());
    PointT diff_b(line_b.first.x() - line_b.second.x(),
                  line_b.first.y() - line_b.second.y());

    angle = acos((boost::geometry::dot_product(diff_a, diff_b)) /
                 (boost::geometry::length(line_a)*
                  boost::geometry::length(line_b)));
}


template<typename T,
         typename PointT>
inline void nearestIntersectionDist
(const typename types::Line<PointT>::type    &line_a,
 const typename types::LineSet<PointT>::type &lines_b,
 T &distance,
 T &angle,
 const T default_distance,
 const T default_angle)
{
    nearestIntersectionDist<PointT, T, types::LineSet>(line_a, lines_b,
                                                       distance,
                                                       angle,
                                                       default_distance,
                                                       default_angle);
}

template<typename T,
         typename PointT>
inline void nearestIntersectionDist
(const typename types::Line<PointT>::type           &line_a,
 const typename types::IndexedLineSet<PointT>::type &lines_b,
 T &distance,
 T &angle,
 const T default_distance,
 const T default_angle)
{
    nearestIntersectionDist<PointT, T, types::IndexedLineSet>(line_a,
                                                              lines_b,
                                                              distance,
                                                              angle,
                                                              default_distance,
                                                              default_angle);
}

template<typename T,
         typename PointT>
inline void multiNearestIntersectionDist(const typename types::LineSet<PointT>::type  &lines_a,
                                         const typename types::LineSet<PointT>::type  &lines_b,
                                         const T default_value,
                                         std::vector<T> &results)
{
    results.resize(lines_a.size());
    auto lines_a_ptr = lines_a.data();
    auto results_ptr = results.data();

#pragma omp parallel for
    for(unsigned int i = 0 ; i < lines_a.size() ; ++i) {
        auto &line = *(lines_a_ptr + i);
        auto &result = *(results_ptr + i);
        result = nearestIntersectionDist<PointT, T, types::LineSet>(line, lines_b, default_value);
    }
}

template<typename PointT>
inline void multiNearestIntersection
(const typename types::LineSet<PointT>::type            &lines_a,
 const typename types::LineSet<PointT>::type            &lines_b,
 typename types::ValidatedResultSet<PointT>::type &results)
{
    assert(lines_a.size() > 0);
    assert(lines_b.size() > 0);
    unsigned int lines_a_size = lines_a.size();
    results.resize(lines_a_size);

    auto lines_a_ptr = lines_a.data();
    auto results_ptr = results.data();

#pragma omp parallel for
    for(unsigned int i = 0 ; i < lines_a_size ; ++i) {
        auto &result = *(results_ptr + i);
        auto &line = *(lines_a_ptr + i);
        result.valid = nearestIntersection<PointT>(line,
                                                   lines_b,
                                                   result.result);
    }
}

template<typename PointT>
inline bool translate
(const PointT                                    &src_point,
 const typename types::Translation<PointT>::type &translation,
 PointT                                    &dst_point)
{
    return boost::geometry::transform(src_point, dst_point, translation);
}


template<typename PointT>
inline bool translate
(const typename types::Line<PointT>::type        &src_line,
 const typename types::Translation<PointT>::type &translation,
 typename types::Line<PointT>::type        &dst_line)
{
    bool success = true;
    success &= translate<PointT>(src_line.first, translation, dst_line.first);
    success &= translate<PointT>(src_line.second, translation, dst_line.second);
    return success;
}

namespace impl{
/**
 * @brief Do translation for more than one geometry.
 * @param src           the source container
 * @param translation   the translation
 * @param dst           the destination container
 * @return              if all translations were susccessful
 */
template<typename PointT, typename ContainerT, typename TranslationT>
inline bool foreachTranslation(const ContainerT   &src_container,
                               const TranslationT &translation,
                               ContainerT &dst_container)
{
    dst_container.resize(src_container.size());
    auto src_ptr = src_container.data();
    auto dst_ptr = dst_container.data();
    bool success = true;

#pragma omp parallel for reduction(&&:success)
    for(unsigned int i = 0 ; i < src_container.size() ; ++i) {
        auto &src_geom = *(src_ptr + i);
        auto &dst_geom = *(dst_ptr + i);

        success &= translate<PointT>
                (src_geom, translation, dst_geom);
    }
    return success;
}
}

template<typename PointT>
inline bool translate
(const typename types::PointSet<PointT>::type    &src_points,
 const typename types::Translation<PointT>::type &translation,
 typename types::PointSet<PointT>::type    &dst_points)
{
    return impl::foreachTranslation<PointT,
            typename types::PointSet<PointT>::type,
            typename types::Translation<PointT>::type>
            (src_points, translation, dst_points);
}

template<typename PointT>
inline bool translate
(const typename types::LineSet<PointT>::type     &src_lines,
 const typename types::Translation<PointT>::type &translation,
 typename types::LineSet<PointT>::type     &dst_lines)
{
    return impl::foreachTranslation<PointT,
            typename types::LineSet<PointT>::type,
            typename types::Translation<PointT>::type>
            (src_lines, translation, dst_lines);
}

template<typename PointT>
inline bool rotate
(const PointT                                    &src_point,
 const typename types::Rotation<PointT>::type    &rotation,
 PointT                                          &dst_point)
{
    return boost::geometry::transform(src_point, dst_point, rotation);
}

template<typename PointT>
inline bool rotate(const typename types::Line<PointT>::type        &src_line,
                   const typename types::Rotation<PointT>::type    &rotation,
                   typename types::Line<PointT>::type              &dst_line)
{
    return boost::geometry::transform(src_line, dst_line, rotation);
}


template<typename T, typename PointT>
T dot(const typename types::Line<PointT>::type &line_a,
           const typename types::Line<PointT>::type &line_b)
{
    PointT diff_a(line_a.first.x() - line_a.second.x(),
                  line_a.first.y() - line_a.second.y());
    PointT diff_b(line_b.first.x() - line_b.second.x(),
                  line_b.first.y() - line_b.second.y());
    return boost::geometry::dot_product(diff_a, diff_b);
}

template<typename T>
inline bool equal
(const T value_1,
 const T value_2,
 const T epsilon)
{
    return std::abs(value_1 - value_2) < epsilon;
}

template<typename PointT>
inline bool withinExcl
(const PointT &p,
 const typename types::Polygon<PointT>::type &polygon)
{
    return boost::geometry::within(p, polygon);
}

template<typename PointT>
inline bool withinExcl
(const typename types::Line<PointT>::type &line,
 const typename types::Box<PointT>::type &box)
{
    return boost::geometry::within(line.first, box) &&
            boost::geometry::within(line.second, box);
}

template<typename PointT>
inline bool lessEqual
(const PointT &p1,
 const PointT &p2)
{
    return p1.x() <= p2.x() && p1.y() <= p2.y();
}

template<typename PointT>
inline bool greaterEqual
(const PointT &p1,
 const PointT &p2)
{
    return p1.x() >= p2.x() && p1.y() >= p2.y();
}

template<typename PointT, typename T>
inline bool equal
(const PointT &p1,
 const PointT &p2,
 const T eps)
{
    return equal<T>(p1.x(), p2.x(), eps)  && equal(p1.y(), p2.y(), eps);
}

template<typename PointT, typename T>
inline bool equal
(const typename types::Line<PointT>::type &line_a,
 const typename types::Line<PointT>::type &line_b,
 const T eps)
{
    return equal<PointT, T>(line_a.first, line_b.first, eps)  && equal<PointT, T>(line_a.second, line_b.second, eps) ||
           equal<PointT, T>(line_a.first, line_b.second, eps) && equal<PointT, T>(line_a.second, line_b.first, eps);
}

template<typename T, typename PointT>
T angle(const typename types::Line<PointT>::type &line_a,
        const typename types::Line<PointT>::type &line_b)
{
    PointT diff_a(line_a.first.x() - line_a.second.x(),
                  line_a.first.y() - line_a.second.y());
    PointT diff_b(line_b.first.x() - line_b.second.x(),
                  line_b.first.y() - line_b.second.y());

    const double norm_a = hypot(diff_a.x(), diff_a.y());
    const double norm_b = hypot(diff_b.x(), diff_b.y());
    if(norm_a == 0.0)
        return 0.0;
    if(norm_b == 0.0)
        return 0.0;

    diff_a.x(diff_a.x() / norm_a);
    diff_a.y(diff_a.y() / norm_a);

    diff_b.x(diff_b.x() / norm_b);
    diff_b.y(diff_b.y() / norm_b);

    return  acos(boost::geometry::dot_product(diff_a, diff_b));
}

template<typename T, typename PointT>
T angle(const typename types::Line<PointT>::type &line_a,
        const typename types::Line<PointT>::type &line_b,
        const T eps)
{
    PointT diff_a(line_a.first.x() - line_a.second.x(),
                  line_a.first.y() - line_a.second.y());
    PointT diff_b(line_b.first.x() - line_b.second.x(),
                  line_b.first.y() - line_b.second.y());

    double a = hypot(diff_a.x(), diff_a.y());
    double b = hypot(diff_b.x(), diff_b.y());
    double c = 0.0;

    if(equal<PointT, T>(line_a.first, line_b.first, eps)) {
        const double dx = line_a.second.x() - line_b.second.x();
        const double dy = line_a.second.y() - line_b.second.y();
        c = hypot(dx, dy);
    } else
    if(equal<PointT, T>(line_a.first, line_b.second, eps)) {
        const double dx = line_a.second.x() - line_b.first.x();
        const double dy = line_a.second.y() - line_b.first.y();
        c = hypot(dx, dy);
    } else
    if(equal<PointT, T>(line_a.second, line_b.first, eps)) {
        std::swap(a,b);
        const double dx = line_a.first.x() - line_b.second.x();
        const double dy = line_a.first.y() - line_b.second.y();
        c = hypot(dx, dy);
    } else
    if(equal<PointT, T>(line_a.second, line_b.second, eps)) {
        std::swap(a,b);
        const double dx = line_a.first.x() - line_b.first.x();
        const double dy = line_a.first.y() - line_b.first.y();
        c = hypot(dx, dy);
    } else {
        return angle<T, PointT>(line_a, line_b);
    }

    if(a == 0.0)
        return 0.0;
    if(b == 0.0)
        return 0.0;

    return acos((a*a + b*b - c*c) / (2 * a * b));
}

template<typename T>
inline bool withinIncl
(const T p_x,   const T p_y,
 const T min_x, const T min_y,
 const T max_x, const T max_y)
{
    return p_x >= min_x && p_y >= min_y &&
            p_x <= max_x && p_y <= max_y;
}

template<typename PointT>
inline bool withinIncl
(const PointT &p,
 const typename types::Box<PointT>::type &box)
{
    const PointT &min = box.min_corner();
    const PointT &max = box.max_corner();
    return greaterEqual(p, min) && lessEqual(p, max);
}

template<typename PointT>
inline bool withinIncl
(const typename types::Line<PointT>::type &line,
 const typename types::Box<PointT>::type  &box)
{
    return withinIncl(line.first, box) &&
            withinIncl(line.second, box);
}

template<typename PointT>
inline bool withinIncl
(const typename types::Box<PointT>::type &inner,
 const typename types::Box<PointT>::type &outer)
{
    return withinIncl(inner.min_corner(), outer) &&
            withinIncl(inner.max_corner(), outer);
}

template<typename PointT>
inline bool within
(const typename types::Polygon<PointT>::type &inner,
 const typename types::Polygon<PointT>::type &outer)
{
    bool within = true;
    auto inner_pts_ptr = inner.outer().data();
#pragma omp parallel for reduction(&&:within)
    for(unsigned int i = 0 ; i < inner.outer().size() ; ++i) {
        auto &pt  = *(inner_pts_ptr + i);
        within &= boost::geometry::within(pt, outer);
    }

    return within;
}

template<typename PointT>
inline bool covered_by(const typename types::Polygon<PointT>::type &covered,
                       const typename types::Polygon<PointT>::type &by)
{
#if BOOST_VERSION / 100 % 1000 >= 57
    return boost::geometry::covered_by(covered, by);
#else
    if(boost::geometry::intersects(covered, by))
        return true;
    if(within<PointT>(covered, by))
        return true;
    return false;
#endif
}


template<typename PointT>
inline bool touches
(const typename types::Line<PointT>::type     &line,
 const typename types::Polygon<PointT>::type  &polygon)
{
    if(withinExcl<PointT>(line.first, polygon))
        return true;
    if(withinExcl<PointT>(line.second, polygon))
        return true;
    if(intersects<PointT>(line, polygon))
        return true;

    return false;
}

template<typename PointT>
inline bool touches
(const typename types::Line<PointT>::type &line,
 const typename types::Box<PointT>::type  &box)
{
    if(boost::geometry::within(line.first, box))
        return true;
    if(boost::geometry::within(line.second, box))
        return true;

    const PointT& min = box.min_corner();
    const PointT& max = box.max_corner();
    PointT lup (min.x(), max.y());
    PointT rlo (max.x(), min.y());
    typename types::Line<PointT>::type t_edge(max, lup);
    if(intersects<PointT>(line, t_edge))
        return true;
    typename types::Line<PointT>::type l_edge(min, lup);
    if(intersects<PointT>(line, l_edge))
        return true;
    typename types::Line<PointT>::type r_edge(max, rlo);
    if(intersects<PointT>(line, r_edge))
        return true;
    typename types::Line<PointT>::type b_edge(min, rlo);
    if(intersects<PointT>(line, b_edge))
        return true;

    return false;
}

template<typename PointT>
typename types::Polygon<PointT>::type toPolygon
(const PointT &min,
 const PointT &max)
{
    typename types::Polygon<PointT>::type poly;
    boost::geometry::append(poly.outer(), PointT(min.x(), min.y()));
    boost::geometry::append(poly.outer(), PointT(max.x(), min.y()));
    boost::geometry::append(poly.outer(), PointT(max.x(), max.y()));
    boost::geometry::append(poly.outer(), PointT(min.x(), max.y()));
    return poly;
}

template<typename PointT>
typename types::Polygon<PointT>::type toPolygon
(const typename types::Box<PointT>::type &box)
{
    typename types::Polygon<PointT>::type poly;
    const PointT &min = box.min_corner();
    const PointT &max = box.max_corner();
    boost::geometry::append(poly.outer(), PointT(min.x(), min.y()));
    boost::geometry::append(poly.outer(), PointT(max.x(), min.y()));
    boost::geometry::append(poly.outer(), PointT(max.x(), max.y()));
    boost::geometry::append(poly.outer(), PointT(min.x(), max.y()));
    return poly;
}

template<typename PointT,
         typename Periodic>
inline void polarLineSet
(const PointT &center,
 const double center_line_orientation,
 const double opening_angle,
 const double angle_increment,
 const double length,
 typename types::LineSet<PointT>::type &lines)
{
    unsigned int num_rays = floor(opening_angle / angle_increment) + 1;
    lines.resize(num_rays);

    auto lines_ptr = lines.data();
    double start_angle = center_line_orientation - opening_angle * 0.5;
    double angle = 0.0;

#pragma omp parallel for private(angle)
    for(unsigned int i = 0 ; i < num_rays ; ++i) {
        auto &line = *(lines_ptr + i);
        angle = start_angle + i * angle_increment;
        types::Point2d &origin      = line.first;
        types::Point2d &destination = line.second;
        origin.x(center.x());
        origin.y(center.y());
        destination.x(center.x() + cos(angle) * length);
        destination.y(center.y() + sin(angle) * length);
    }
}

template<typename PointT,
         typename Periodic>
inline void polarLineSet
(const PointT &center,
 const double center_line_orientation,
 const double opening_angle,
 const double angle_increment,
 const double length,
 typename types::LineSet<PointT>::type &lines,
 std::vector<double> &angles)
{
    unsigned int num_rays = floor(opening_angle / angle_increment) + 1;
    lines.resize(num_rays);
    angles.resize(num_rays);

    auto angles_ptr = angles.data();
    auto lines_ptr  = lines.data();

    double start_angle = center_line_orientation - opening_angle * 0.5;

#pragma omp parallel for
    for(unsigned int i = 0 ; i < num_rays ; ++i) {
        auto &line  = *(lines_ptr + i);
        auto &angle = *(angles_ptr + i);

        angle = start_angle + i * angle_increment;

        types::Point2d &origin      = line.first;
        types::Point2d &destination = line.second;
        origin.x(center.x());
        origin.y(center.y());
        destination.x(center.x() + cos(angle) * length);
        destination.y(center.y() + sin(angle) * length);
    }
}

template<typename PointT,
         typename Periodic>
inline void polarLineSet
(const PointT         &center,
 const double          center_line_orientation,
 const double          opening_angle,
 const unsigned int    num_rays,
 const double          length,
 typename types::LineSet<PointT>::type &lines)
{
    lines.resize(num_rays);

    auto lines_ptr = lines.data();

    double angle_increment(opening_angle / (double) num_rays);
    double start_angle = center_line_orientation - opening_angle * 0.5;
    double angle = 0.0;
    double cos = 1.0;
    double sin = 0.0;

#pragma omp parallel for private(cos, sin, angle)
    for(unsigned int i = 0 ; i < num_rays ; ++i) {
        auto &line = *(lines_ptr + i);
        angle = start_angle + i * angle_increment;
        types::Point2d &origin      = line.first;
        types::Point2d &destination = line.second;
        origin.x(center.x());
        origin.y(center.y());
        Periodic::sin_cos(angle, sin, cos);
        destination.x(center.x() + cos * length);
        destination.y(center.y() + sin * length);
    }
}

template<typename PointT,
         typename Periodic>
inline void polarLineSet
(const PointT         &center,
 const double          center_line_orientation,
 const double          opening_angle,
 const unsigned int    num_rays,
 const double          length,
 typename types::LineSet<PointT>::type &lines,
 std::vector<double> &angles)
{
    lines.resize(num_rays);
    angles.resize(num_rays);
    double angle_increment(opening_angle / (double) num_rays);
    double start_angle = center_line_orientation - opening_angle * 0.5;
    double cos = 1.0;
    double sin = 0.0;

    auto lines_ptr = lines.data();
    auto angles_ptr = angles.data();
#pragma omp parallel for private(cos, sin)
    for(unsigned int i = 0 ; i < num_rays ; ++i) {
        auto &line  = *(lines_ptr + i);
        auto &angle = *(angles_ptr + i);
        angle = start_angle + i * angle_increment;
        types::Point2d &origin      = line.first;
        types::Point2d &destination = line.second;
        origin.x(center.x());
        origin.y(center.y());
        Periodic::sin_cos(angle, sin, cos);
        destination.x(center.x() + cos * length);
        destination.y(center.y() + sin * length);
    }
}

template<typename PointT>
inline void circularPolygonApproximation
(const PointT &center,
 const double  radius,
 const double  ang_res,
 typename types::Polygon<PointT>::type &polygon)
{
    unsigned int iterations = floor(2 * M_PI / ang_res + 0.5);
    double       angle = 0.0;

    for(unsigned int i = 0 ; i < iterations ; ++i, angle -= ang_res) {
        PointT p;
        p.x(center.x() + cos(angle) * radius);
        p.y(center.y() + sin(angle) * radius);
        boost::geometry::append(polygon.outer(), p);
    }
    boost::geometry::append(polygon.outer(), polygon.outer().front());
}
}
}
#endif // DXF_ALGORITHMS_HPP
