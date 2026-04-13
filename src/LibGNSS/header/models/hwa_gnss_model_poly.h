
/**
*
* @brief    Purpose: implements polynomial approximation
*           directly interpolate
*           get/fit/evaluate polynomials
*/

#ifndef hwa_gnss_model_poly_H
#define hwa_gnss_model_poly_H

#include "hwa_base_eigendef.h"
#include <vector>
#include <iostream>

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model_poly. */
    class gnss_model_poly
    {
    public:
        /** @brief default constructor. */
        explicit gnss_model_poly();
        /** @brief default destructor. */
        virtual ~gnss_model_poly();

        /** @brief reset polynom. */
        void reset();

        /** @brief get polynomial degree. */
        int degree() const { return _ncoeff - 1; }

        /** @brief get # of coefficients. */
        int ncoeff() const { return _ncoeff; }

        /** @brief get accuracy. */
        double rms() const { return _rms; }

        /** @brief get validity. */
        bool valid() const { return _valid; }

        /** @brief get x-span. */
        double span() const { return _span; }

        /** @brief get x-reference value. */
        double xref() const { return _xref; }

        /** @brief get polynomials. */
        std::vector<double> polynomials() const { return _coef; }

        int interpolate(const std::vector<double> &X, const std::vector<double> &Y, const double &x, double &y, double &dy);
        int polynomials(const std::vector<double> &X, const std::vector<double> &Y);
        void evaluate(double x, int I, double &y);

        /**
        * @brief fit polynom
        * @param[in]  X            xdata time-difference.
        * @param[in]  Y            ydata.
        * @param[in]  N            degree of polynom.
        * @param[in]  tunit        X-time units [sec].
        * @param[in]  t            reference time.
        * @return      int
        */
        int fitpolynom(const std::vector<double> &X,
                       const std::vector<double> &Y,
                       int N, double tunit,
                       const base_time &t);

    private:
        bool _valid;          ///< are coefficients valid ?
        int _ncoeff;          ///< polynomial order (n) for n+1 points
        double _rms;          ///< RMS of fitted coefficients  [meters]
        double _xref;         ///< x reference value is always 0.0 !!!!!!
        double _span;         ///< x span
        std::vector<double> _coef; ///< coefficients
    };

    double lagrange_interpolate(const std::vector<double> &X, const std::vector<double> &Y, double x, const bool &lvel = false);
    //double lagrange_interpolate_v(const vector<double>& X, const vector<double>& Y, double x, bool ispos);

} // namespace

#endif
