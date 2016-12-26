/// COMPONENT
#include "linefit_lsq.h"
#ifndef LINEFIT_LSQ_HPP
namespace cslibs_boost_geometry {
namespace algorithms {
namespace fitting {
/**
 * @brief The LSQ struct is used to incrementally update least squares
 *        focusing line fitting. Onlie 2D is regarded.
 * @param T         the lsq resoluation considering data types
 * @üaram PointT    the boost point type to be processed
 */
template<typename PointT, typename T>
struct LSQ {
    T sxx, syy, sxy;
    T sum_x, sum_y;
    unsigned n;

    T theta, rho, var;

    LSQ() :
        sxx(0.0), syy(0.0), sxy(0.0),
        sum_x(0.0), sum_y(0.0),
        n(0),
        theta(0.0), rho(0.0), var(0.0)
    {
    }

    inline void reset()
    {
        sxx = syy = sxy     = 0.0;
        sum_x = sum_y       = 0.0;
        n                   = 0;
        theta = rho = var   = 0.0;
    }

    inline void add(const PointT &p)
    {
        add(p.x(), p.y());
    }

    inline void sub(const PointT &p)
    {
        sub(p.x(), p.y());
    }

    inline void add(const T x, const T y)
    {
        sxx   += x * x;
        syy   += y * y;
        sxy   += x * y;
        sum_x += x;
        sum_y += y;
        ++n;
    }

    inline void sub(const T x, const T y)
    {
        sxx   -= x * x;
        syy   -= y * y;
        sxy   -= x * y;
        sum_x -= x;
        sum_y -= y;
        --n;
    }

    inline void update()
    {
        const T n_inv = 1.0 / (double) n;
        const T mean_x = sum_x * n_inv;
        const T mean_y = sum_y * n_inv;
        const T sxx = this->sxx - n * mean_x * mean_x;
        const T syy = this->syy - n * mean_y * mean_y;
        const T sxy = this->sxy - n * mean_x * mean_y;
        const T theta_x = syy - sxx;
        const T theta_y = -2.0 * sxy;

        theta = 0.5 * atan2(theta_y, theta_x);
        rho   = mean_x * cos(theta) + mean_y * sin(theta);
        var   = 0.5 * n_inv * (sxx + syy - sqrt(4.0 * sxy * sxy + (syy - sxx) * (syy - sxx)));
    }
};

template<typename PointT, typename T>
inline T euclidean(const PointT &p1,
                   const PointT &p2)
{
    return hypot(p1.x() - p2.x(), p1.y() - p2.y());
}
template<typename PointT, typename T>
inline T line(const PointT &p,
              const LSQ<PointT, T> &lsq)
{
    return fabs(p.x() * cos(lsq.theta) +
                p.y() * sin(lsq.theta) -
                lsq.rho);
}

template<typename T>
inline T normalize(const T z)
{
    double r = z;
    while(r < -M_PI)
        r += 2 * M_PI;
    while(r >= M_PI)
        r -= 2 * M_PI;
    return r;
}

template<typename PointT, typename T>
inline T angular(const PointT &p1,
                 const PointT &p2,
                 const PointT &p3)
{
    PointT v1(p2.x() - p1.x(), p2.y() - p1.y());
    PointT v2(p3.x() - p2.x(), p3.y() - p2.y());

    T angle = atan2(v2.y(), v2.x()) - atan2(v1.y(), v1.x());
    return normalize<T>(angle);
}

template<typename PointT, typename T>
inline void addFittedLineLSQ(const fitting::LSQ<PointT, T>         &lsq,
                             const PointT                          &d_k,
                             const PointT                          &d_kn,
                             typename types::LineSet<PointT>::type &lines)
{
    T t1 =  -d_k.x() * sin(lsq.theta) +  d_k.y() * cos(lsq.theta);
    T t2 = -d_kn.x() * sin(lsq.theta) + d_kn.y() * cos(lsq.theta);
    typename types::Line<PointT>::type line;
    line.first.x(lsq.rho * cos(lsq.theta) - t1 * sin(lsq.theta));
    line.first.y(lsq.rho * sin(lsq.theta) + t1 * cos(lsq.theta));
    line.second.x(lsq.rho * cos(lsq.theta) - t2 * sin(lsq.theta));
    line.second.y(lsq.rho * sin(lsq.theta) + t2 * cos(lsq.theta));
    lines.push_back(line);
}

template<typename PointT, typename T>
inline void getFittedLineLSQ(const fitting::LSQ<PointT, T>         &lsq,
                             const PointT                          &d_k,
                             const PointT                          &d_kn,
                             typename types::Line<PointT>::type    &line)
{
    T t1 =  -d_k.x() * sin(lsq.theta) +  d_k.y() * cos(lsq.theta);
    T t2 = -d_kn.x() * sin(lsq.theta) + d_kn.y() * cos(lsq.theta);
    line.first.x(lsq.rho * cos(lsq.theta) - t1 * sin(lsq.theta));
    line.first.y(lsq.rho * sin(lsq.theta) + t1 * cos(lsq.theta));
    line.second.x(lsq.rho * cos(lsq.theta) - t2 * sin(lsq.theta));
    line.second.y(lsq.rho * sin(lsq.theta) + t2 * cos(lsq.theta));
}
}

template<typename PointT, typename T>
void linefitLSQ(const typename types::PointSet<PointT>::type  &points,
                const T delta_d,
                const T delta_var,
                const T delta_ang,
                typename types::LineSet<PointT>::type         &lines)
{
    const PointT      *pts = points.data();
    const unsigned int n   = points.size();

    fitting::LSQ<PointT, T> lsq;
    unsigned int k_0 = 0;
    unsigned int k_1 = k_0+1;
    unsigned int k_2 = k_0+2;

    PointT d_k;
    PointT d_kn;
    while(k_2 < n) {
        /// FIND INITIAL POINTS
        while(k_2 < n) {
            if(fitting::euclidean<PointT, T>(pts[k_1], pts[k_0]) < delta_d &&
                    fitting::euclidean<PointT, T>(pts[k_2], pts[k_1]) < delta_d) {
                lsq.add(pts[k_0]);
                lsq.add(pts[k_1]);
                lsq.add(pts[k_2]);
                lsq.update();
                if(lsq.var < delta_var) {
                    d_k = pts[k_0];
                    d_kn= pts[k_2];
                    /// FOUND FIRST MATCHING POINT SUBSET
                    break;
                }
                lsq.reset();
            }
            ++k_0;++k_1;++k_2;
        }
        ++k_2;++k_0;++k_1;
        while(k_2 < n) {
            if(fitting::euclidean<PointT, T>(pts[k_2], pts[k_1]) < delta_d &&
                    fitting::line<PointT, T>(pts[k_2], lsq) < delta_d &&
                    fabs(fitting::angular<PointT, T>(pts[k_0], pts[k_1], pts[k_2])) < delta_ang) {
                d_kn = pts[k_2];
                lsq.add(pts[k_2]);
                lsq.update();
            } else {
                addFittedLineLSQ(lsq, d_k, d_kn, lines);
                lsq.reset();
                k_0 = k_2;
                k_1 = k_0 + 1;
                k_2 = k_0 + 2;
                break;
            }
            ++k_2;++k_0;++k_1;
        }
    }
    if(lsq.n > 0)
        addFittedLineLSQ(lsq, d_k, d_kn, lines);
}

template<typename PointT, typename T>
void linefitLSQ(const typename types::PointSet<PointT>::type  &points,
                const T delta_d,
                const T delta_var,
                const T delta_ang,
                typename types::LineSet<PointT>::type    &lines,
                std::vector<std::pair<double, double> > &angles)
{
    const PointT      *pts = points.data();
    const unsigned int n   = points.size();

    fitting::LSQ<PointT, T> lsq;
    unsigned int k_0 = 0;
    unsigned int k_1 = k_0+1;
    unsigned int k_2 = k_0+2;

    PointT d_k;
    PointT d_kn;
    while(k_2 < n) {
        /// FIND INITIAL POINTS
        while(k_2 < n) {
            if(fitting::euclidean<PointT, T>(pts[k_1], pts[k_0]) < delta_d &&
                    fitting::euclidean<PointT, T>(pts[k_2], pts[k_1]) < delta_d) {
                lsq.add(pts[k_0]);
                lsq.add(pts[k_1]);
                lsq.add(pts[k_2]);
                lsq.update();
                if(lsq.var < delta_var) {
                    d_k = pts[k_0];
                    d_kn= pts[k_2];
                    /// FOUND FIRST MATCHING POINT SUBSET
                    break;
                }
                lsq.reset();
            }
            ++k_0;++k_1;++k_2;
        }
        ++k_2;++k_0;++k_1;
        while(k_2 < n) {
            if(fitting::euclidean<PointT, T>(pts[k_2], pts[k_1]) < delta_d &&
                    fitting::line<PointT, T>(pts[k_2], lsq) < delta_d &&
                    fabs(fitting::angular<PointT, T>(pts[k_0], pts[k_1], pts[k_2])) < delta_ang) {
                d_kn = pts[k_2];
                lsq.add(pts[k_2]);
                lsq.update();
            } else {
                typename types::Line<PointT>::type l;
                getFittedLineLSQ(lsq, d_k, d_kn, l);
                lines.push_back(l);
                angles.push_back(std::make_pair(atan2(l.first.y(), l.first.x()),
                                                atan2(l.second.y(), l.second.x())));

                lsq.reset();
                k_0 = k_2;
                k_1 = k_0 + 1;
                k_2 = k_0 + 2;
                break;
            }
            ++k_2;++k_0;++k_1;
        }
    }
    if(lsq.n > 0) {
        typename types::Line<PointT>::type l;
        getFittedLineLSQ(lsq, d_k, d_kn, l);
        lines.push_back(l);
        angles.push_back(std::make_pair(atan2(l.first.y(), l.first.x()),
                                        atan2(l.second.y(), l.second.x())));
    }
}
}
}

#define LINEFIT_LSQ_HPP
#endif // LINEFIT_LSQ_HPP

