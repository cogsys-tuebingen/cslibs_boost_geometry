#ifndef LINEFIT_LSQ_H
#define LINEFIT_LSQ_H

#include "types.hpp"

namespace utils_boost_geometry {
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

    inline void add(const T x, const T y)
    {
        sxx   += x * x;
        syy   += y * y;
        sxy   += x * y;
        sum_x += x;
        sum_y += y;
        ++n;
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
inline T euclidean(const PointT &b1,
                   const PointT &b2)
{
    return hypot(b1.x() - b2.x(), b1.y() - b2.y());
}
template<typename PointT, typename T>
inline T line(const PointT &b,
              const LSQ<PointT, T> &lsq)
{
    return fabs(b.x() * cos(lsq.theta) +
                b.y() * sin(lsq.theta) -
                lsq.rho);
}
}

template<typename PointT, typename T>
void addFittedLineLSQ(const fitting::LSQ<PointT, T>         &lsq,
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
        while(k_2 < n) {
            ++k_2;++k_0;++k_1;

            if(fitting::euclidean<PointT, T>(pts[k_2], pts[k_1]) < delta_d &&
               fitting::line<PointT, T>(pts[k_2], lsq) < delta_d) {
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
        }
    }
    if(lsq.n > 0)
        addFittedLineLSQ(lsq, d_k, d_kn, lines);
}

template<typename PointT, typename T, typename DeltaT>
void linefitPolarLSQ(std::vector<T> &rhos,
                     const T angle_min,
                     const T angle_incr,
                     const T range_min,
                     const T range_max,
                     const DeltaT delta_d,
                     const DeltaT delta_var,
                     typename types::LineSet<PointT>::type &lines)
{
    const T           *r = rhos.data();
    const unsigned int n = rhos.size();

    assert(rhos.size() > 2);

    fitting::LSQ<PointT, T> lsq;
    unsigned int k_0 = 0;
    unsigned int k_1 = k_0+1;
    unsigned int k_2 = k_0+2;
    T angle = angle_min;
    PointT pt_k_0(r[k_0] * cos(angle), r[k_0] * sin(angle));
    angle += angle_incr;
    PointT pt_k_1(r[k_1] * cos(angle), r[k_1] * sin(angle));
    angle += angle_incr;
    PointT pt_k_2(r[k_2] * cos(angle), r[k_2] * sin(angle));


    PointT d_k;
    PointT d_kn;
    while(k_2 < n) {
        /// FIND INITIAL POINTS
        while(k_2 < n) {
            if(fitting::euclidean<PointT, T>(pt_k_1, pt_k_0) < delta_d &&
               fitting::euclidean<PointT, T>(pt_k_2, pt_k_1) < delta_d) {
                lsq.add(pt_k_0);
                lsq.add(pt_k_1);
                lsq.add(pt_k_2);
                lsq.update();
                if(lsq.var < delta_var) {
                    d_k = pt_k_0;
                    d_kn= pt_k_2;
                    /// FOUND FIRST MATCHING POINT SUBSET
                    break;
                }
                lsq.reset();
            }
            ++k_0;++k_1;++k_2;
            angle += angle_incr;
            pt_k_0 = pt_k_1;
            pt_k_1 = pt_k_2;
            pt_k_2 = PointT(r[k_2] * cos(angle), r[k_2] * sin(angle));

        }
        while(k_2 < n) {
            ++k_2;
            angle += angle_incr;
            if(r[k_2] > range_max ||
                    r[k_2] < range_min)
                continue;
            ++k_0;++k_1;
            pt_k_0 = pt_k_1;
            pt_k_1 = pt_k_2;
            pt_k_2 = PointT(r[k_2] * cos(angle), r[k_2] * sin(angle));

            if(fitting::euclidean(pt_k_2, pt_k_1) < delta_d &&
               fitting::line(pt_k_2, lsq) < delta_d) {
                d_kn = pt_k_2;
                lsq.add(pt_k_2);
                lsq.update();
            } else {
                addFittedLineLSQ(lsq, d_k, d_kn, lines);
                lsq.reset();
                k_0 = k_2;
                k_1 = k_2 + 1;
                k_2 = k_2 + 2;
                break;
            }
        }
    }
    if(lsq.n > 0)
        addFittedLineLSQ(lsq, d_k, d_kn, lines);
}
}
}


#endif // LINEFIT_LSQ_H
