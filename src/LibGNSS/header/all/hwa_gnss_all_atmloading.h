
/**
*
*
* @file     gallatmloading.h
* @brief    Purpose: container for atmospheric pressure loading data
*
*/

#ifndef GALLATMLOADING_H
#define GALLATMLOADING_H

#include <string>
#include <map>

#include "hwa_gnss_data_atmloading.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
     *@brief Class for gnss_all_otl derive from base_data
     */
    class gnss_all_atmloading : public base_data
    {

        /** ================================================ Function =========================================================== */
    public:
        /** @brief default constructor. */
        gnss_all_atmloading();

        /** @brief default destructor. */
        ~gnss_all_atmloading();

        /** @brief get data. */
        bool data(const double &lon, const std::string &lonmode, const double &lat, const std::string &latmode, Matrix &coeffs);

        /** @Spline interpolation for atmospheric pressure loading coefficients. */
        bool spine(const double &lon, const double &lat, Matrix &coeff);

        /**
         * @brief 
         * 
         * @param atmld 
         */
        void add(gnss_data_atmloading &atmld);

        /**
         * @brief 
         * 
         */
        void print();

        /**
         * @brief 
         * 
         */
        void splie2_coeffs();

        /**
         * @brief 
         * 
         * @param ya 
         * @param y2a 
         */
        void splie2(Matrix &ya, Matrix &y2a);

        /**
         * @brief 
         * 
         * @param x2a 
         * @param ytmp 
         * @param n 
         * @param eps1 
         * @param eps2 
         * @param y2tmp 
         */
        void spline(Matrix &x2a, Matrix &ytmp, const int &n, const double &eps1, const double &eps2, Matrix &y2tmp);

        /**
         * @brief Get the coef object
         * 
         * @param lon 
         * @param lat 
         * @param coeff 
         * @return true 
         * @return false 
         */
        bool get_coef(const double &lon, const double &lat, Matrix &coeff);

        /**
         * @brief 
         * 
         * @param grid_coef 
         * @param grid_coef_spined 
         * @param lon 
         * @param lat 
         * @return double 
         */
        double splin2(Matrix &grid_coef, Matrix &grid_coef_spined, const double &lon, const double &lat);

        /**
         * @brief 
         * 
         * @param xa 
         * @param ya 
         * @param y2a 
         * @param n 
         * @param x 
         * @return double 
         */
        double splint(Matrix &xa, Matrix &ya, Matrix &y2a, const int &n, const double &x);

        /**
         * @brief 
         * 
         * @param x1y1 
         * @param x2y1 
         * @param x1y2 
         * @param x2y2 
         * @param dx 
         * @param dy 
         * @return double 
         */
        double linear_intep(const double &x1y1, const double &x2y1, const double &x1y2, const double &x2y2, const double &dx, const double &dy);

    protected:
    private:
        /** ================================================ ******  =========================================================== */
    public:
    protected:
        std::vector<gnss_data_atmloading> _gridatmld;
        Matrix _grid_cs1du, _grid_ss1du, _grid_cs2du, _grid_ss2du;
        Matrix _grid_cs1dn, _grid_ss1dn, _grid_cs2dn, _grid_ss2dn;
        Matrix _grid_cs1de, _grid_ss1de, _grid_cs2de, _grid_ss2de;
        Matrix _grid_cs1dui, _grid_ss1dui, _grid_cs2dui, _grid_ss2dui;
        Matrix _grid_cs1dni, _grid_ss1dni, _grid_cs2dni, _grid_ss2dni;
        Matrix _grid_cs1dei, _grid_ss1dei, _grid_cs2dei, _grid_ss2dei;
        int _nlon = 360;
        int _nlat = 181;
        Matrix _lon, _lat;

    private:
    };

} // namespace

#endif
