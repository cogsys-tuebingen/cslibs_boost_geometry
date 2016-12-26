#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "types.hpp"

namespace cslibs_boost_geometry {
namespace algorithms {
/**
 * @brief This method can be used to intersect two lines.
 * @param line_a    the first line
 * @param line_b    the second line
 * @param points    the resulting points (must be empty)
 *                  - no point   : no intersection was found
 *                  - one point  : the intersection
 *                  - two points : one line is laying the other
 * @return          if intersections were found
 */
template<typename PointT>
bool intersection(const typename types::Line<PointT>::type       &line_a,
                  const typename types::Line<PointT>::type       &line_b,
                  typename types::PointSet<PointT>::type   &points);


/**
 * @brief Intersect a polygon with a line and retrieve all intersection points.
 * @param line_a    the line to intersect
 * @param polygon   the polygon to intersect
 * @param points    the resulting points (required to be empty)
 *                  - no point   : no intersection was found
 *                  - one point  : only one intersection was found
 *                  - two points : if endpoints of line, line within border of the polygon
 *                               : if not endpoints, to intersections
 *                  - more points: several intersections with the polygon
 * @return
 */
template<typename PointT>
bool intersection(const typename types::Line<PointT>::type      &line_a,
                  const typename types::Polygon<PointT>::type   &polygon,
                  typename types::PointSet<PointT>::type  &points);


/**
 * @brief Get distance from point to line.
 * @param point     the first line
 * @param line_b    the second line
 * @return          if intersections were found
 */
template<typename T,
         typename PointT>
T distance(const PointT                             &point,
           const typename types::Line<PointT>::type &line);

/**
 * @warning temporary implementation with end point to end point metric
 * @brief Calculate the end point to end point distance of line segments.
 * @param line_a    the first line
 * @param line_b    the second line
 * @return          the distance
 */
template<typename T,
         typename PointT>
T minEndPointDistance(const typename types::Line<PointT>::type &line_a,
                      const typename types::Line<PointT>::type &line_b);


template<typename T, typename PointT>
T length(const typename types::Line<PointT>::type &line);

/**
 * @brief Check if one line intersect another.
 * @param line_a    the first line
 * @param line_b    the second line
 * @return          if intersections were found
 */
template<typename PointT>
bool intersects(const typename types::Line<PointT>::type &line_a,
                const typename types::Line<PointT>::type &line_b);


/**
 * @brief Check if a line intersects a polygon.
 * @param line_a    the line to check for intersection
 * @param polygon   the polygon to check for intersection
 * @return          if the line intersects at least one point
 */
template<typename PointT>
bool intersects(const typename types::Line<PointT>::type    &line_a,
                const typename types::Polygon<PointT>::type &polygon);


template<typename PointT, template <typename> class Set>
bool nearestIntersection(const typename types::Line<PointT>::type    &line_a,
                         const typename Set<PointT>::type            &lines_b,
                         typename types::PointSet<PointT>::type      &points);

template<typename PointT>
bool nearestIntersection(const typename types::Line<PointT>::type    &line_a,
                         const typename types::LineSet<PointT>::type &lines_b,
                         typename types::PointSet<PointT>::type      &points);

template<typename PointT>
bool nearestIntersection(const typename types::Line<PointT>::type           &line_a,
                         const typename types::IndexedLineSet<PointT>::type &lines_b,
                         typename types::PointSet<PointT>::type       &points);

template<typename PointT>
bool nearestIntersection(const typename types::Line<PointT>::type    &line_a,
                         const typename types::LineSet<PointT>::type &lines_b,
                         typename types::PointSet<PointT>::type      &points,
                         typename types::Line<PointT>::type          &line_b);

template<typename PointT>
bool nearestIntersection(const typename types::Line<PointT>::type           &line_a,
                         const typename types::IndexedLineSet<PointT>::type &lines_b,
                         typename types::PointSet<PointT>::type             &points,
                         typename types::Line<PointT>::type                 &line_b);


template<typename T,
         typename PointT>
T nearestIntersectionDistance(const typename types::Line<PointT>::type           &line_a,
                              const typename types::LineSet<PointT>::type        &lines_b,
                              const T default_value = 0);

template<typename T,
         typename PointT>
T nearestIntersectionDistance(const typename types::Line<PointT>::type           &line_a,
                              const typename types::IndexedLineSet<PointT>::type &lines_b,
                              const T default_value = 0);


template<typename T,
         typename PointT>
void nearestIntersectionDistance(const typename types::Line<PointT>::type           &line_a,
                                 const typename types::LineSet<PointT>::type        &lines_b,
                                 T &distance,
                                 T &angle,
                                 const T default_distance = 0,
                                 const T default_angle = 0);

template<typename T,
         typename PointT>
void nearestIntersectionDistance(const typename types::Line<PointT>::type           &line_a,
                                 const typename types::IndexedLineSet<PointT>::type &lines_b,
                                 T &distance,
                                 T &angle,
                                 const T default_distance = 0,
                                 const T default_angle = 0);

template<typename T,
         typename PointT>
void nearestIntersectionDistanceBatch(const typename types::LineSet<PointT>::type    &lines_a,
                                      const typename types::LineSet<PointT>::type    &lines_b,
                                      const T default_value,
                                      std::vector<T> &results);

/**
 * @brief Intersect one line set with another one.
 * @param lines_a       the first line set
 * @param lines_b       the second line set
 * @param results       the results
 *                      - will contain size(lines_a) results
 *                      - each result can be valid or not
 */
template<typename PointT>
void nearestIntersectionBatch(const typename types::LineSet<PointT>::type      &lines_a,
                              const typename types::LineSet<PointT>::type      &lines_b,
                              typename types::IntersectionResultSet<PointT>::type &results);
/**
 * @brief Apply a translation to a point.
 * @param src_point     the original point
 * @param translation   the translation
 * @param dst_point     the translated point
 * @return              if the translation was successful
 */
template<typename PointT>
bool translate(const PointT                                    &src_point,
               const typename types::Translation<PointT>::type &translation,
               PointT                                    &dst_point);

/**
 * @brief Apply a tarnslation to a line.
 * @param src_line      the original line
 * @param translation   the translation
 * @param dst_line      the translated line
 * @return              if the translation was successful
 */
template<typename PointT>
bool translate(const typename types::Line<PointT>::type        &src_line,
               const typename types::Translation<PointT>::type &translation,
               typename types::Line<PointT>::type        &dst_line);

/**
 * @brief Do translation for a point set.
 * @param src           the source container
 * @param translation   the translation
 * @param dst           the destination container
 * @return              if all translations were susccessful
 */
template<typename PointT>
bool translate(const typename types::PointSet<PointT>::type    &src_points,
               const typename types::Translation<PointT>::type &translation,
               typename types::PointSet<PointT>::type    &dst_points);

/**
 * @brief Do translation for a line set.
 * @param src           the source container
 * @param translation   the translation
 * @param dst           the destination container
 * @return              if all translations were susccessful
 */
template<typename PointT>
bool translate(const typename types::LineSet<PointT>::type     &src_lines,
               const typename types::Translation<PointT>::type &translation,
               typename types::LineSet<PointT>::type     &dst_lines);

/**
 * @brief Rotate a point around the origin.
 * @param src_point     the original point
 * @param rotation      the rotation
 * @param dst_point     the resulting point
 * @return              sucess of the operation
 */
template<typename PointT>
bool rotate(const PointT                                    &src_point,
            const typename types::Rotation<PointT>::type    &rotation,
            PointT                                          &dst_point);

/**
 * @brief Rotate a line segment around the origin.
 * @param src_line  the original line
 * @param rotation  the rotation to apply
 * @param dst_line  the resulting line
 * @return          success of the operation
 */
template<typename PointT>
bool rotate(const typename types::Line<PointT>::type        &src_line,
            const typename types::Rotation<PointT>::type    &rotation,
            typename types::Line<PointT>::type              &dst_line);

/**
 * @brief Calculate the dot product of two line segments by using their
 *        non-normalized direction vectors.
 * @param line_a
 * @param line_b
 * @return
 */
template<typename T, typename PointT>
T dot(const typename types::Line<PointT>::type &line_a,
      const typename types::Line<PointT>::type &line_b);


#define TO_RAD M_PI / 180.0
#define TO_DEG 180.0 / M_PI
/**
 * @brief Convert radian to degree.
 * @param rad       value in radian
 * @return          value in degree
 */
inline double deg(const double rad)
{
    return TO_DEG * rad;
}

/**
 * @brief Convert degree to radian.
 * @param deg       value in degree
 * @return          value in radian
 */
inline double rad(const double deg)
{
    return TO_RAD * deg;
}

/**
 * @brief Test on equality with a certain accuracy.
 * @param value_1   the first value
 * @param value_2   the second value
 * @param epsilon   the accuracy
 * @return          if equal under accuracy constraint
 */
template<typename T>
bool equal(const T value_1,
           const T value_2,
           const T epsilon);

template<typename PointT>
bool withinExcl(const PointT &p,
                const typename types::Polygon<PointT>::type &polygon);

template<typename PointT>
bool withinExcl(const typename types::Line<PointT>::type &line,
                const typename types::Box<PointT>::type &box);

template<typename PointT>
bool lessEqual (const PointT &p1,
                const PointT &p2);

template<typename PointT>
bool greaterEqual(const PointT &p1,
                  const PointT &p2);

template<typename PointT, typename T>
bool equal(const PointT &p1,
           const PointT &p2,
           const T eps);

template<typename PointT, typename T>
bool equal(const typename types::Line<PointT>::type &line_a,
           const typename types::Line<PointT>::type &line_b,
           const T eps);

/**
 * @brief Calculate the angle between two line segments by using their
 *        non-normalized direction vectors.
 * @param line_a    the first line
 * @param line_b    the second line
 * @return          the angle
 */
template<typename T, typename PointT>
T angle(const typename types::Line<PointT>::type &line_a,
        const typename types::Line<PointT>::type &line_b);

/**
 * @brief Calculate the angle between two line segments.
 *        If end points equal, trigonometry is applied to gain
 *        angles greateer 90 degrees. The standard dot product in combination
 *        with the acos is only capable of calculating the minimum angle.
 * @param line_a    the first line
 * @param line_b    the second line
 * @param eps       the maximum deviation between end points
 * @return          the angle
 */
template<typename T, typename PointT>
T angle(const typename types::Line<PointT>::type & line_a,
        const typename types::Line<PointT>::type & line_b,
        const T eps);


template<typename T>
bool withinIncl(const T p_x,   const T p_y,
                const T min_x, const T min_y,
                const T max_x, const T max_y);

template<typename PointT>
bool withinIncl(const PointT &p,
                const typename types::Box<PointT>::type &box);

template<typename PointT>
bool withinIncl(const typename types::Line<PointT>::type &line,
                const typename types::Box<PointT>::type  &box);

template<typename PointT>
bool withinIncl(const typename types::Box<PointT>::type &inner,
                const typename types::Box<PointT>::type &outer);

template<typename PointT>
bool within(const typename types::Polygon<PointT>::type &inner,
            const typename types::Polygon<PointT>::type &outer);

template<typename PointT>
bool covered_by(const typename types::Polygon<PointT>::type &covered,
                const typename types::Polygon<PointT>::type &by);

/**
 * @brief Check if a line is whithin a polygon.
 * @param line      the line to check
 * @param polygon   the polygon to check the line with
 * @return          true if
 *                  - one or both end points within the polygon
 *                  - the line intersects the polygon
 */
template<typename PointT>
bool touches(const typename types::Line<PointT>::type     &line,
             const typename types::Polygon<PointT>::type  &polygon);


/**
 * @brief Check if line is at least with one endpoint within a box or is
 *        intersecting it.
 * @param line      the line to test
 * @param box       the box to test
 * @return          if the line intersects the box or has one endpoint in it
 */
template<typename PointT>
bool touches(const typename types::Line<PointT>::type &line,
             const typename types::Box<PointT>::type &box);

template<typename PointT>
typename types::Polygon<PointT>::type
    toPolygon(const PointT& min,
              const PointT& max);

template<typename PointT>
typename types::Polygon<PointT>::type
    toPolygon(const typename types::Box<PointT>::type& box);

/**
 * @brief Generate a polar line set.
 * @param line_origin               the origin point of the lines
 * @param center_line_orientation   the origin orienation (starting angle)
 * @param opening_angle             the opening angle
 * @param angular_resolution        the angular resolution
 * @param length                    the length of all lines
 * @param rays                      the lines generated.
 */
template<typename PointT,
         typename Periodic>
void polarLineSet(const PointT &center,
                  const double center_line_orientation,
                  const double opening_angle,
                  const double angular_resolution,
                  const double length,
                  typename types::LineSet<PointT>::type &lines);

template<typename PointT,
         typename Periodic>
void polarLineSet(const PointT &center,
                  const double center_line_orientation,
                  const double opening_angle,
                  const double angular_resolution,
                  const double length,
                  typename types::LineSet<PointT>::type &lines,
                  std::vector<double> &angles);


template<typename PointT,
         typename Periodic>
void polarLineSet(const PointT                          &center,
                  const double                           center_line_orientation,
                  const double                           opening_angle,
                  const unsigned int                     num_rays,
                  const double                           length,
                  typename types::LineSet<PointT>::type &lines);


template<typename PointT,
         typename Periodic>
void polarLineSet(const PointT                          &center,
                  const double                           center_line_orientation,
                  const double                           opening_angle,
                  const unsigned int                     num_rays,
                  const double                           length,
                  typename types::LineSet<PointT>::type &lines,
                  std::vector<double> &angles);
/**
 * @brief Generate a polygon approximating a circle.
 * @param center                    the center of the approximated circle
 * @param radius                    the radius of the approximated circle
 * @param ang_res                   the angular resolution to span polygon points
 * @param polygon                   the polygon (clockwise point order !)
 */
template<typename PointT>
void circularPolygonApproximation(const PointT &center,
                                  const double  radius,
                                  const double  ang_res,
                                  typename types::Polygon<PointT>::type &polygon);

}
}



#endif // ALGORITHMS_H
