#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <iomanip>

#include "hwa_base_globaltrans.h"
#include "hwa_gnss_model_tide.h"
#include "hwa_gnss_model_tide96.h"

using namespace std;

namespace hwa_gnss
{
    gnss_model_tide96::gnss_model_tide96(base_log spdlog) : gnss_model_tide(spdlog)
    {
    }

    gnss_model_tide96::~gnss_model_tide96()
    {
    }

    // solid base_earth tides
    // ----------
    Triple gnss_model_tide96::tide_earth(const base_time &epoch, Triple &crd)
    {
        _mutex.lock();

        Triple dxyz(0.0, 0.0, 0.0);

        Vector xSun;
        Vector xMoon;
        double rSun;
        double rMoon;

        double Mjd = epoch.dmjd();

        xSun = _planEph.sunPos(Mjd);

        rSun = sqrt(xSun.dot(xSun));
        xSun /= rSun;
        xMoon = _planEph.moonPos(Mjd);
        rMoon = sqrt(xMoon.dot(xMoon));
        xMoon /= rMoon;

        Vector xyz = crd;
        double rRec = sqrt(xyz.dot(xyz));
        Vector xyzUnit = xyz / rRec;

        // Love's Numbers
        // --------------
        const double H2 = 0.6078;
        const double L2 = 0.0847;

        // Tidal Displacement
        // ------------------
        double scSun = xyzUnit.dot(xSun);
        double scMoon = xyzUnit.dot(xMoon);

        double p2Sun = 3.0 * (H2 / 2.0 - L2) * scSun * scSun - H2 / 2.0;
        double p2Moon = 3.0 * (H2 / 2.0 - L2) * scMoon * scMoon - H2 / 2.0;

        double x2Sun = 3.0 * L2 * scSun;
        double x2Moon = 3.0 * L2 * scMoon;

        const double gmWGS = 398.6005e12;
        const double gms = 1.3271250e20;
        const double gmm = 4.9027890e12;

        double facSun = gms / gmWGS *
                        (rRec * rRec * rRec * rRec) / (rSun * rSun * rSun);

        double facMoon = gmm / gmWGS *
                         (rRec * rRec * rRec * rRec) / (rMoon * rMoon * rMoon);

        Vector dX = facSun * (x2Sun * xSun + p2Sun * xyzUnit) +
                          facMoon * (x2Moon * xMoon + p2Moon * xyzUnit);

        for (int i = 0; i < 3; i++)
            dxyz[i] = dX(i);
        //std::cout << "return tide: " << dxyz[0] << " " << dxyz[1] << " " << dxyz[2] << std::endl;
        _mutex.unlock();
        return dxyz;
    }

    // pole tides
    // ----------
    Triple gnss_model_tide96::tide_pole()
    {
        _mutex.lock();

        Triple dxyz(0.0, 0.0, 0.0);

        _mutex.unlock();
        return dxyz;
    }

    // ocean tide loading
    // ----------
    Triple gnss_model_tide96::load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec)
    {
        _mutex.lock();

        Matrix otl;

        Triple dxyz(0.0, 0.0, 0.0);
        if (!_gotl || _gotl->data(otl, site) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "WARNING: Site " + site + " not found in ocean tide BLQ file!");
            _mutex.unlock();
            return dxyz;
        }

#ifdef DEBUG
        std::cout << "OTL matrix: " << otl << std::endl;
#endif

        const double args[11][5] = {
            {1.40519E-4, 2.0, -2.0, 0.0, 0.00},  /* M2 */
            {1.45444E-4, 0.0, 0.0, 0.0, 0.00},   /* S2 */
            {1.37880E-4, 2.0, -3.0, 1.0, 0.00},  /* N2 */
            {1.45842E-4, 2.0, 0.0, 0.0, 0.00},   /* K2 */
            {0.72921E-4, 1.0, 0.0, 0.0, 0.25},   /* K1 */
            {0.67598E-4, 1.0, -2.0, 0.0, -0.25}, /* O1 */
            {0.72523E-4, -1.0, 0.0, 0.0, -0.25}, /* P1 */
            {0.64959E-4, 1.0, -3.0, 1.0, -0.25}, /* Q1 */
            {0.53234E-5, 0.0, 2.0, 0.0, 0.00},   /* Mf */
            {0.26392E-5, 0.0, 1.0, -1.0, 0.00},  /* Mm */
            {0.03982E-5, 2.0, 0.0, 0.0, 0.00}    /* Ssa */
        };

        base_time time75(1975, 1, 1, 0, 0, 0);

        /* angular argument: see subroutine arg.f for reference [1] */
        double fday = epoch.sod();                // sec number of the day
        double days = (epoch - time75) / 86400.0; // decimal days from 1975
        double t = (27392.500528 + 1.000000035 * days) / 36525.0;

        double a[5];
        a[0] = fday;
        a[1] = (279.69668 + 36000.768930485 * t + 3.03E-4 * t * t) * D2R;                        /* H0 */
        a[2] = (270.434358 + 481267.88314137 * t - 0.001133 * t * t + 1.9E-6 * t * t * t) * D2R; /* S0 */
        a[3] = (334.329653 + 4069.0340329577 * t + 0.010325 * t * t - 1.2E-5 * t * t * t) * D2R; /* P0 */
        a[4] = 2.0 * hwa_pi;

        double ang;
        Triple dp(0, 0, 0);

        /* displacements by 11 constituents */
        for (unsigned int i = 0; i < 11; i++)
        {
            ang = 0.0;
            for (unsigned int j = 0; j < 5; j++)
                ang += a[j] * args[i][j];
            for (unsigned int j = 0; j < 3; j++)
                dp[j] += otl(j + 1, i + 1) * cos(ang - otl(j + 4, i + 1) * D2R);
        }

#ifdef DEBUG
        std::cout << std::fixed << setprecision(6) << setw(15) << dp[0] << setw(15) << dp[1] << setw(15) << dp[2] << std::endl;
#endif

        Triple dneu;
        dneu[0] = -dp[2];
        dneu[1] = -dp[1];
        dneu[2] = dp[0];

        Triple ell;
        xyz2ell(xRec, ell, false);
        neu2xyz(ell, dneu, dxyz);

        _mutex.unlock();
        return dxyz;
    }

    // atmospheric tide loading
    // ----------
    Triple gnss_model_tide96::load_atmosph()
    {
        _mutex.lock();

        Triple dxyz(0.0, 0.0, 0.0);

        _mutex.unlock();
        return dxyz;
    }

} // namespace
