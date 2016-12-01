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

class Mask {
public:
    Mask(const unsigned int _size) :
        mask(new bool[_size]),
        size(_size)
    {
        std::fill(mask, mask + _size, true);
    }

    bool& operator[](const unsigned int i)
    {
        return mask[i];
    }

    void resize(const unsigned int _size)
    {
        if(mask != nullptr) {
            delete [] mask;
            mask = new bool[_size];
        }

        size = _size;
        std::fill(mask, mask + _size, true);
    }

    virtual ~Mask()
    {
        if(mask != nullptr)
            delete [] mask;
    }

private:
    bool        *mask;
    unsigned int size;
};

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
#if BOOST_VERSION >= 105500
    typedef boost::geometry::strategy::transform::translate_transformer<double, 2, 2>
#else
    typedef boost::geometry::strategy::transform::translate_transformer<PointT, PointT>
#endif
    type;
};

template<typename PointT>
struct Rotation {
#if BOOST_VERSION >= 105500
    typedef boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
#else
    typedef boost::geometry::strategy::transform::rotate_transformer<PointT, PointT,
                                                                     boost::geometry::radian>
#endif
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
#if BOOST_VERSION >= 105500
    return boost::geometry::strategy::transform::inverse_transformer<double, 2, 2>::inverse_transformer(translation);
#else
    return boost::geometry::strategy::transform::inverse_transformer<PointT, PointT>::inverse_transformer(translation);
#endif
}


struct periodic {
    inline static double sin(const double rad)
    {
        return sin(rad);
    }

    inline static double cos(const double rad)
    {
        return cos(rad);
    }

    inline static void sin_cos(const double x,
                               double &sinx,
                               double &cosx)
    {
        sincos(x, &sinx, &cosx);
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
        return fastersinfull(angle);

    }

    inline static double cos(const double rad)
    {
        double angle = angleClamp(rad);
        return fastercosfull(angle);
    }

    inline static void sin_cos(const double rad,
                               double &sinx,
                               double &cosx)
    {
        double angle = angleClamp(rad);
        sinx = fastersinfull(angle);
        cosx = fastercosfull(angle);
    }

    ///__________________ FAST MATH _________________///

    static inline float
    fastsin (float x)
    {
      static const float fouroverpi = 1.2732395447351627f;
      static const float fouroverpisq = 0.40528473456935109f;
      static const float q = 0.78444488374548933f;
      union { float f; uint32_t i; } p = { 0.20363937680730309f };
      union { float f; uint32_t i; } r = { 0.015124940802184233f };
      union { float f; uint32_t i; } s = { -0.0032225901625579573f };

      union { float f; uint32_t i; } vx = { x };
      uint32_t sign = vx.i & 0x80000000;
      vx.i = vx.i & 0x7FFFFFFF;

      float qpprox = fouroverpi * x - fouroverpisq * x * vx.f;
      float qpproxsq = qpprox * qpprox;

      p.i |= sign;
      r.i |= sign;
      s.i ^= sign;

      return q * qpprox + qpproxsq * (p.f + qpproxsq * (r.f + qpproxsq * s.f));
    }

    static inline float
    fastersin (float x)
    {
      static const float fouroverpi = 1.2732395447351627f;
      static const float fouroverpisq = 0.40528473456935109f;
      static const float q = 0.77633023248007499f;
      union { float f; uint32_t i; } p = { 0.22308510060189463f };

      union { float f; uint32_t i; } vx = { x };
      uint32_t sign = vx.i & 0x80000000;
      vx.i &= 0x7FFFFFFF;

      float qpprox = fouroverpi * x - fouroverpisq * x * vx.f;

      p.i |= sign;

      return qpprox * (q + p.f * qpprox);
    }

    static inline float
    fastsinfull (float x)
    {
      static const float twopi = 6.2831853071795865f;
      static const float invtwopi = 0.15915494309189534f;

      int k = x * invtwopi;
      float half = (x < 0) ? -0.5f : 0.5f;
      return fastsin ((half + k) * twopi - x);
    }

    static inline float
    fastersinfull (float x)
    {
      static const float twopi = 6.2831853071795865f;
      static const float invtwopi = 0.15915494309189534f;

      int k = x * invtwopi;
      float half = (x < 0) ? -0.5f : 0.5f;
      return fastersin ((half + k) * twopi - x);
    }

    static inline float
    fastcosfull (float x)
    {
      static const float halfpi = 1.5707963267948966f;
      return fastsinfull (x + halfpi);
    }

    static inline float
    fastercosfull (float x)
    {
      static const float halfpi = 1.5707963267948966f;
      return fastersinfull (x + halfpi);
    }
};

/// PREDIFINED TYPES
typedef boost::geometry::model::d2::point_xy<double> Point2d;
typedef boost::geometry::model::d2::point_xy<float>  Point2f;
typedef boost::geometry::model::d2::point_xy<int>    Point2i;
typedef Point2i                                      Dim2i;
typedef Point2f                                      Dim2f;
typedef Point2d                                      Dim2d;
typedef Point2i                                      Vec2i;
typedef Point2f                                      Vec2f;
typedef Point2d                                      Vec2d;
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
typedef Rotation<Point2i>::type                      Rotation2i;
typedef Rotation<Point2f>::type                      Rotation2f;
typedef Rotation<Point2d>::type                      Rotation2d;
typedef std::vector<Line2i*>                         Line2iPtrSet;
typedef std::vector<Line2f*>                         Line2fPtrSet;
typedef std::vector<Line2d*>                         Line2dPtrSet;
}
}
#endif // DXF_TYPES_HPP
