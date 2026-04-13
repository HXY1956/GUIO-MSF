#include "hwa_base_eigendef.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_model_eop80.h"

using namespace hwa_base;
using namespace std;

namespace hwa_gnss
{
    gnss_model_eop80::gnss_model_eop80()
    {
    }

    gnss_model_eop80::~gnss_model_eop80()
    {
    }

    // Nutation Matrix (Nutation theory 1980)
    // --------------------------------------
    Matrix gnss_model_eop80::nutMatrix(double mjd)
    {
        //  boost::mutex::scoped_lock lock(_mutex);

        const double T = (mjd - MJD_J2000) / 36525.0;

        double ls = 2.0 * hwa_pi * frac(0.993133 + 99.997306 * T);
        double D = 2.0 * hwa_pi * frac(0.827362 + 1236.853087 * T);
        double F = 2.0 * hwa_pi * frac(0.259089 + 1342.227826 * T);
        double N = 2.0 * hwa_pi * frac(0.347346 - 5.372447 * T);

        double dpsi = (-17.200 * sin(N) - 1.319 * sin(2 * (F - D + N)) - 0.227 * sin(2 * (F + N)) + 0.206 * sin(2 * N) + 0.143 * sin(ls)) / RHO_SEC;
        double deps = (+9.203 * cos(N) + 0.574 * cos(2 * (F - D + N)) + 0.098 * cos(2 * (F + N)) - 0.090 * cos(2 * N)) / RHO_SEC;

        double eps = 0.4090928 - 2.2696E-4 * T;

        return rotX(-eps - deps) * rotZ(-dpsi) * rotX(+eps);
    }

    // Precession Matrix (Precession theory 1976)
    // ------------------------------------------
    Matrix gnss_model_eop80::precMatrix(double mjd)
    {
        //  boost::mutex::scoped_lock lock(_mutex);

        // Time interval betwen epoch J2000 and start epoch (in Julian centuries)
        const double T = (mjd - MJD_J2000) / 36525.0;

        // Euler angles
        double zeta = T * (2306.2181 + T * (0.30188 + T * 0.017998)) * SEC2RAD;
        double theta = T * (2004.3109 + T * (-0.42665 - T * 0.041833)) * SEC2RAD;
        double z = T * (2306.2181 + T * (1.09468 + T * 0.018203)) * SEC2RAD;

        return rotZ(-z) * rotY(theta) * rotZ(-zeta);
    }

    // Frac part of double
    // ------------------------
    double gnss_model_eop80::frac(double x)
    {
        return x - floor(x);
    }

} // namespace
