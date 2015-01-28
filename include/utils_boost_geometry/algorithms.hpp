#ifndef DXF_ALGORITHMS_HPP
#define DXF_ALGORITHMS_HPP

/// COMPONENT
#include "algorithms.h"

/// SYSTEM
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/algorithms/length.hpp>

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
    const typename types::Polygon<PointT>::type::ring_type           &ring = polygon.outer();
    typename types::Polygon<PointT>::type::ring_type::const_iterator it_first = ring.begin();
    typename types::Polygon<PointT>::type::ring_type::const_iterator it_second = it_first + 1;
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
    const typename types::Polygon<PointT>::type::ring_type           &ring = polygon.outer();
    typename types::Polygon<PointT>::type::ring_type::const_iterator it_first = ring.begin();
    typename types::Polygon<PointT>::type::ring_type::const_iterator it_second = it_first + 1;
    while(it_second != ring.end()) {
        typename types::Line<PointT>::type line_b(*it_first, *it_second);
        if(intersects<PointT>(line_a, line_b))
            return true;
        ++it_first;
        ++it_second;
    }
    return false;
}


template<typename PointT, template <typename> class Set>
inline bool nearestIntersection
    (const typename types::Line<PointT>::type    &line_a,
     const typename Set<PointT>::type            &lines_b,
     typename types::PointSet<PointT>::type      &points)
{
    assert(lines_b.size() > 0);
    const PointT &origin = line_a.first;
    double min = std::numeric_limits<double>::max();
    for(typename Set<PointT>::type::const_iterator it =
        lines_b.begin() ;
        it != lines_b.end() ;
        ++it) {
        typename types::PointSet<PointT>::type tmp_points;
        if(intersection<PointT>(line_a, Set<PointT>::getSegment(it), tmp_points)) {
            if(tmp_points.size() == 2) {
                points = tmp_points;
                return true;
            }
            if(tmp_points.size() == 1) {
                const PointT &intersection = tmp_points.back();
                double diff_x  = origin.x() - intersection.x();
                double diff_y  = origin.y() - intersection.y();
                double diff_sq = diff_x * diff_x + diff_y * diff_y;
                if(diff_sq < min) {
                    min = diff_sq;
                    points = tmp_points;
                }
            }
        }
    }

    return points.size() > 0;
}

template<typename PointT>
inline bool nearestIntersection
    (const typename types::Line<PointT>::type          &line_a,
     const typename types::LineSet<PointT>::type       &lines_b,
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

template<typename T,
         typename PointT>
inline void multiNearestIntersectionDist(const typename types::LineSet<PointT>::type  &lines_a,
                                  const typename types::LineSet<PointT>::type  &lines_b,
                                  const T default_value,
                                  std::vector<T> &results)
{
    results.resize(lines_a.size());
    for(unsigned int i = 0 ; i < lines_a.size() ; ++i) {
        results.at(i) = nearestIntersectionDist<T, PointT>(lines_a.at(i),
                                                           lines_b,
                                                           default_value);
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
    for(unsigned int i = 0 ; i < lines_a_size ; ++i) {
        typename types::ValidatedPointSet<PointT>::type &result = results.at(i);
        result.valid = nearestIntersection<PointT>(lines_a.at(i),
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
    ContainerT tmp(src_container.size());
    typename ContainerT::const_iterator src_it = src_container.begin();
    typename ContainerT::iterator       tmp_it = tmp.begin();
    bool success = true;
    while(src_it != src_container.end()) {
        success &= translate<PointT>
                        (*src_it, translation, *tmp_it);
        ++src_it;
        ++tmp_it;
    }
    dst_container.assign(tmp.begin(), tmp.end());
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

template<typename PointT>
inline bool equal
    (const PointT &p1,
     const PointT &p2)
{
    return p1.x() == p2.x() && p1.y() == p2.y();
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
    double angle = center_line_orientation - opening_angle * 0.5;
    for(unsigned int i = 0 ; i < num_rays ; ++i, angle += angle_increment) {
        types::Point2d &origin      = lines.at(i).first;
        types::Point2d &destination = lines.at(i).second;
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
    double angle_increment(opening_angle / (double) num_rays);
    double angle = center_line_orientation - opening_angle * 0.5;
    for(unsigned int i = 0 ; i < num_rays ; ++i, angle += angle_increment) {
        types::Point2d &origin      = lines.at(i).first;
        types::Point2d &destination = lines.at(i).second;
        origin.x(center.x());
        origin.y(center.y());
        destination.x(center.x() + Periodic::cos(angle) * length);
        destination.y(center.y() + Periodic::sin(angle) * length);
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
    double            angle = 0.0;

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
