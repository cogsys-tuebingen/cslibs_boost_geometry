#ifndef DXF_TYPES_HPP
#define DXF_TYPES_HPP

/// SYSTEM
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/strategies/transform/inverse_transformer.hpp>
#include <boost/function.hpp>

#include <set>

namespace utils_boost_geometry {
namespace types {
/**
 * @brief Type definition for point sets.
 */
template<typename PointT>
struct PointSet {
    typedef std::vector<PointT>
    type;
};
/**
 * @brief Type definition of a polygon.
 */
template<typename PointT>
struct Polygon {
    typedef boost::geometry::model::polygon<PointT>
    type;
};

/**
 * @brief Type definition for a line.
 */
template<typename PointT>
struct Line {
    typedef boost::geometry::model::segment<PointT>
    type;
};
/**
 * @brief Type definition for a line set.
 */
template<typename PointT>
struct LineSet {
    typedef std::vector<boost::geometry::model::segment<PointT> >
    type;

    static const boost::geometry::model::segment<PointT>& getSegment(const typename type::const_iterator& it) {
        return *it;
    }

    static boost::geometry::model::segment<PointT>& getSegment(const typename type::iterator& it) {
        return *it;
    }
};
/**
 * @brief Type definition for an indexed line set.
 */
template<typename PointT>
struct IndexedLineSet {
    typedef std::vector<boost::geometry::model::segment<PointT>* >
    type;

    static const boost::geometry::model::segment<PointT>& getSegment(const typename type::const_iterator& it) {
        return **it;
    }

    static boost::geometry::model::segment<PointT>& getSegment(const typename type::iterator& it) {
        return **it;
    }
};

/**
 * @brief Type definition for a box.
 */
template<typename PointT>
struct Box {
    typedef boost::geometry::model::box<PointT>
    type;
};

/**
 * @brief Type definition for point sets that
 *        can be declared invalid.
 */
template<typename PointT>
struct ValidatedPointSet {
    typedef ValidatedPointSet<PointT>
    type;

    ValidatedPointSet() :
        valid(false)
    {
    }

    typename PointSet<PointT>::type result;
    bool                            valid;
};
/**
 * @brief Type definition for a set of
 *        invalidatable point sets.
 */
template<typename PointT>
struct ValidatedResultSet {
    typedef std::vector<ValidatedPointSet<PointT> >
    type;
};


/**
 * @brief Type defintion for a translation.
 */
template<typename PointT>
struct Translation {
    typedef boost::geometry::strategy::transform::translate_transformer<PointT, PointT>
    type;
};

/**
 * @brief Invert a translation.
 * @param translation   the translation to be inverted
 * @return              the inverted translation
 */
template<typename PointT, typename TransT>
TransT invert(const TransT &translation)
{
    return boost::geometry::strategy::transform::inverse_transformer<PointT, PointT>::inverse_transformer(translation);
}


struct periodic {
    inline static double sin(const double rad)
    {
        return std::sin(rad);
    }

    inline static double cos(const double rad)
    {
        return std::cos(rad);
    }
};


struct periodicApprox {
    inline static double angleClamp(const double rad)
    {
        double rad_(rad);
        while (rad_ < -M_PI)
            rad_ += 2*M_PI;

        while (rad_ >= M_PI)
            rad_ -= 2*M_PI;

        return rad_;
    }


    inline static double sin(const double rad)
    {
        double angle = angleClamp(rad);

        if(angle == 0.0) {
            return 1.0;
        }

        double sin(0.0);

        if (angle < 0) {
            sin = 1.27323954 * angle + .405284735 * angle * angle;

            if (sin < 0.0)
                sin = .225 * (sin *-sin - sin) + sin;
            else
                sin = .225 * (sin * sin - sin) + sin;

        } else {
            sin = 1.27323954 * angle - 0.405284735 * angle * angle;

            if (sin < 0)
                sin = .225 * (sin *-sin - sin) + sin;
            else
                sin = .225 * (sin * sin - sin) + sin;
        }

        return sin;
    }

    inline static double cos(const double rad)
    {
        double angle = angleClamp(rad + M_PI_2);

        if(angle == 0.0) {
            return 0.0;
        }

        double cos(0.0);
        if (angle < 0) {
            cos = 1.27323954 * angle + 0.405284735 * angle * angle;

            if (cos < 0)
                cos = .225 * (cos *-cos - cos) + cos;
            else
                cos = .225 * (cos * cos - cos) + cos;
        } else {
            cos = 1.27323954 * angle - 0.405284735 * angle * angle;

            if (cos < 0)
                cos = .225 * (cos *-cos - cos) + cos;
            else
                cos = .225 * (cos * cos - cos) + cos;
        }

        return cos;
    }

};

/// PREDIFINED TYPES
typedef boost::geometry::model::d2::point_xy<double> Point2d;
typedef boost::geometry::model::d2::point_xy<float>  Point2f;
typedef boost::geometry::model::d2::point_xy<int>    Point2i;
typedef Point2i                                      Dim2i;
typedef Point2f                                      Dim2f;
typedef Point2d                                      Dim2d;
typedef Line<Point2d>::type                          Line2d;
typedef Line<Point2f>::type                          Line2f;
typedef Line<Point2i>::type                          Line2i;
typedef Box<Point2d>::type                           Box2d;
typedef Box<Point2d>::type                           Box2f;
typedef Box<Point2d>::type                           Box2i;
typedef PointSet<Point2i>::type                      PointSet2i;
typedef PointSet<Point2f>::type                      PointSet2f;
typedef PointSet<Point2d>::type                      PointSet2d;
typedef Polygon<Point2i>::type                       Polygon2i;
typedef Polygon<Point2f>::type                       Polygon2f;
typedef Polygon<Point2d>::type                       Polygon2d;
typedef ValidatedResultSet<Point2i>::type            ValidResults2i;
typedef ValidatedResultSet<Point2f>::type            ValidResults2f;
typedef ValidatedResultSet<Point2d>::type            ValidResults2d;
typedef ValidatedPointSet<Point2i>::type             ValidPointSet2i;
typedef ValidatedPointSet<Point2f>::type             ValidPointSet2f;
typedef ValidatedPointSet<Point2d>::type             ValidPointSet2d;
typedef LineSet<Point2i>::type                       LineSet2i;
typedef LineSet<Point2f>::type                       LineSet2f;
typedef LineSet<Point2d>::type                       LineSet2d;
typedef Translation<Point2i>::type                   Translation2i;
typedef Translation<Point2f>::type                   Translation2f;
typedef Translation<Point2d>::type                   Translation2d;
typedef std::vector<Line2i*>                         Line2iPtrSet;
typedef std::vector<Line2f*>                         Line2fPtrSet;
typedef std::vector<Line2d*>                         Line2dPtrSet;
}
}
#endif // DXF_TYPES_HPP
