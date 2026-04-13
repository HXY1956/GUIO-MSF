
/**
* @file        gblindmops.h
* @brief    Purpose: subroutine focused on RTCA MOPS blind model
*/

#ifndef hwa_gnss_model_blindmops_H
#define hwa_gnss_model_GBLINDMOPS_H

#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_interp.h"
#include "hwa_base_Time.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model_blindmops. */
    class gnss_model_blindmops
    {

    public:
        /** @brief default constructor. */
        gnss_model_blindmops();

        gnss_model_blindmops(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_model_blindmops(){};

        /** @brief procedures/functions. */
        void mops(double &lat, double &H, double &elev, double &DOY, double &ZHD, double &ZWD);

        /** @brief Delay calculation. */
        int delay(double &H, double &elev, std::vector<double> &EVAL, double &ZHD0, double &ZHD, double &ZWD0, double &ZWD, double &mf);

    protected:
        /** @brief tabular reading culation. */
        int _tabval(double &lat, Matrix AVG, Matrix VAR, std::vector<double> &I_AVG, std::vector<double> &I_VAR);

        /** @brief extrap. */
        int _extrap(double &DOY, double &DOYmin, std::vector<double> &AVG, std::vector<double> &VAR, std::vector<double> &EVAL);

    private:
        static double a[30];
        static double b[30];
        base_log _spdlog; ///< spdlog pointer
    };

} // namespace

#endif