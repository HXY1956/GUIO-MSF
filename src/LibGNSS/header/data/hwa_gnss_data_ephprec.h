
/**
* @file        gephprec.h
* @brief    derive eph to implement precise ephemerides class
*/

#ifndef hwa_gnss_data_ephprec_H
#define hwa_gnss_data_ephprec_H

#include "hwa_gnss_data_Eph.h"
#include "hwa_gnss_model_poly.h"

#define UNDEFVAL_CLK 999999999.999
#define UNDEFVAL_POS 0.000

#define MAXDIFF_CLK 300.0
#define MAXDIFF_EPH 900.0

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for precise ephemerides data. */
    class gnss_data_ephprec : public gnss_data_eph
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_ephprec();

        /**
         * @brief Construct a new t gephprec object
         * 
         * @param spdlog 
         */
        explicit gnss_data_ephprec(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ephprec();

        // pointers to support NULL if not requested!

        /**
        * @brief get the position value
        * @param[in]  t            GPST.
        * @param[in]  xyz        position value.
        * @param[in]  var        position variation, default value is NULL.
        * @param[in]  vel        vel, default value is NULL.
        * @param[in]  chk_health    the heath of satellite, default value is true.
        * @return      int
        */
        int pos(const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true); // [m]

        /**
        * @brief get the GPS time of transmission,
        * @param[in]  t            GPST.
        * @param[in]  clk        clock offset value.
        * @param[in]  var        clock variation, default value is NULL.
        * @param[in]  dclk        clock difference, default value is NULL.
        * @param[in]  chk_health    the heath of satellite, default value is true.
        * @return      int
        */
        int clk(const base_time &t, double *clk, double *var = NULL, double *dclk = NULL, bool chk_health = true); // [s]

        /**
        * @brief get the position value
        * @param[in]  t            GPST.
        * @param[in]  xyz        position value.
        * @param[in]  var        position variation, default value is NULL.
        * @param[in]  vel        vel, default value is NULL.
        * @return      int
        */
        int pos_int(const base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL); // [m])

        /**
        * @brief get the GPS time of transmission,
        * @param[in]  t            GPST.
        * @param[in]  clk        clock offset value.
        * @param[in]  var        clock variation, default value is NULL.
        * @param[in]  dclk        clock difference, default value is NULL.
        * @return      int
        */
        int clk_int(const base_time &t, double *clk, double *var = NULL, double *dclk = NULL); // [s]

        /**
        * @brief set the data of xyzc,
        * @param[in]  t            GPST.
        * @param[in]  x         _xcrd value.
        * @param[in]  y         _ycrd value.
        * @param[in]  z         _zcrd value.
        * @param[in]  c         _clkc value.
        * @return      int
        */
        int add(std::string sat, std::vector<base_time> t,
                const std::vector<double> &x,
                const std::vector<double> &y,
                const std::vector<double> &z,
                const std::vector<double> &c);

        /** @brief get chk. */
        int chk() const { return 1; }

        /** @brief get str. */
        std::string str() const { return ""; }

        /** @brief print str. */
        int print()
        {
            std::cout << str();
            return 0;
        }

        /** @brief set degree of polynomials. */
        void degree(int d)
        {
            _clear();
            _degree = d;
        }

        /** @brief get degree of polynomials. */
        unsigned int degree() const { return _degree; }

        /** @brief get number of needed data. */
        unsigned int ndata() const { return _degree + 1; }

        /** @brief get validity span. */
        unsigned int interval() const { return (size_t)_poly_x.span(); }

        /** @brief get xreference value. */
        unsigned int xref() const { return (size_t)_poly_x.xref(); }

        /** @brief check validity (incl.data span). */
        bool valid(const base_time &t) const;

    protected:
        /** @brief clear. */
        void _clear();

        /** @brief clk valid? */
        bool _valid_clk() const;

        /** @brief crd valid? */
        bool _valid_crd() const;

    private:
        unsigned int _degree; ///< polynomial degree
        gnss_model_poly _poly_x;      ///< X polynomials
        gnss_model_poly _poly_y;      ///< Y polynomials
        gnss_model_poly _poly_z;      ///< Z polynomials
        gnss_model_poly _poly_c;      ///< C polynomials

        // _epoch is reference epoch
        std::vector<double> _dt;   ///< time difference to _epoch
        std::vector<double> _xcrd; ///< x-coordinate
        std::vector<double> _ycrd; ///< y-coordinate
        std::vector<double> _zcrd; ///< z-coordinate
        std::vector<double> _clkc; ///< clock correction
    };

}

#endif
