#include <cmath>
#include <iomanip>
#include "hwa_gnss_model_tropo.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_gnss
{

    gnss_model_tropo::gnss_model_tropo() // : gnss_model_tropo::gnss_model_tropo("")
    {
    }

    gnss_model_tropo::~gnss_model_tropo()
    {
    }

    // get ZHD
    // ----------
    double gnss_model_tropo::getZHD(const Triple &ell, const base_time &epo) // ell v RADIANECH !! TREBA SJEDNOTIT
    {

        double pp = 1013.25 * pow(1.0 - 2.26e-5 * ell[2], 5.225);
        double res = (0.002277 * pp) / (1.0 - 0.00266 * cos(2.0 * ell[0]) - 0.00000028 * ell[2]);

        std::cerr << "gnss_model_tropo:getZHD not available data " << epo.str_ymdhms() << ", using default: 2.3m \n";

        return res;
    }

    // get ZWD
    // ----------
    double gnss_model_tropo::getZWD(const Triple &ell, const base_time &epo) // ell v RADIANECH !! TREBA SJEDNOTIT
    {

        return 0.0; // NWM_UNKNOWN;
    }

    // ----------------------------- SITE MODELS ------------------------------------

    // ---------
    // Saastamoinen 1972
    // ----------
    double gnss_model_tropo_SAAST::getSTD(const double &ele, const double &height)
    {

        double pp = 1013.25 * pow(1.0 - 2.26e-5 * height, 5.225);
        double TT = 18.0 - height * 0.0065 + 273.15;
        double hh = 50.0 * exp(-6.396e-4 * height);
        double ee = hh / 100.0 * exp(-37.2465 + 0.213166 * TT - 0.000256908 * TT * TT);

#ifdef DEBUG
        std::cout << "meteo data: " << std::fixed << setprecision(3)
             << " pp " << setw(12) << pp
             << " TT " << setw(12) << TT
             << " hh " << setw(12) << hh
             << " ee " << setw(12) << ee << std::endl;
#endif

        double h_km = height / 1000.0;

        if (h_km < 0.0)
            h_km = 0.0;
        if (h_km > 5.0)
            h_km = 5.0;
        int ii = int(h_km + 1);
        double href = ii - 1;

        double bCor[6];
        bCor[0] = 1.156;
        bCor[1] = 1.006;
        bCor[2] = 0.874;
        bCor[3] = 0.757;
        bCor[4] = 0.654;
        bCor[5] = 0.563;

        double BB = bCor[ii - 1] + (bCor[ii] - bCor[ii - 1]) * (h_km - href);

        double zen = hwa_pi / 2.0 - ele;
        double delay = (0.002277 / cos(zen)) * (pp + ((1255.0 / TT) + 0.05) * ee - BB * (tan(zen) * tan(zen)));
        return delay;
    }

    // ----------
    double gnss_model_tropo_SAAST::getZHD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;

        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double delay = (0.002277 * P) /
                       (1.0 - 0.00266 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);

#ifdef DEBUG
        std::cout << "GPT data: " << std::fixed << setprecision(3)
             << " P " << setw(12) << P
             << " T " << setw(12) << T
             << " N " << setw(12) << N
             << " ZHD " << setw(12) << delay << std::endl;
#endif

        return delay;
    }

    // ----------
    double gnss_model_tropo_SAAST::getZWD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        //add need test
        double hh = 0.6;
        double e = hh * 6.11 * pow(10.0, (7.5 * T / (T + 237.3)));

        T += 273.15;

        //double hh =  50.0 * exp(-6.396e-4 * Ell[2]);
        //double e =  hh / 100.0 * exp(-37.2465 + 0.213166*T - 0.000256908*T*T);

        double delay = (0.0022768 * e * (1255.0 / T + 0.05)) / (1.0 - 0.00266 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);
        return delay;
    }

    // ---------
    // Davis 1985 model (Saast + Thayer 1974)
    // ----------
    double gnss_model_tropo_davis::getZHD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double delay = (0.0022768 * P) / (1.0 - 0.0026 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);

#ifdef DEBUG
        std::cout << "GPT data: " << std::fixed << setprecision(3)
             << " P " << setw(12) << P
             << " T " << setw(12) << T
             << " N " << setw(12) << N
             << " ZHD " << setw(12) << delay << std::endl;
#endif

        return delay;
    }

    // ---------
    double gnss_model_tropo_davis::getZWD(const Triple &Ell, const base_time &epoch)
    {

        // not implemented yet
        return 0.0;
    }

    // ----------
    // Hopfield model
    // ----------
    double gnss_model_tropo_hopf::getZHD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;

        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double delay = (1e-06 / 5.0) * (77.64 * (P / T)) * (40136.0 + 148.72 * T);
        return delay;
    }

    // ----------
    double gnss_model_tropo_hopf::getZWD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;
        double hh = 50.0 * exp(-6.396e-4 * Ell[2]);
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double e = hh / 100.0 * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

        double delay = (1.e-06 / 5.0) * ((-12.96) * (e / T) + (3.718 * 1.e05) * (e / (T * T))) * 11000.0;
        return delay;
    }

    // ---------
    // Baby model
    // ---------
    double gnss_model_tropo_baby::getZHD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        T += 273.15; // [K]

        double gs = 9.81; // [ms^2] surface gravity !!!
        double rs = A_WGS + Ell[2];
        double sigma = Eps / T;
        double mu = gs / (Rd * Eps) * (1.0 - (2.0 / (rs * sigma)));

        double delay = (0.022277 * P / gs) * (1.0 + (2.0 / (rs * sigma * (mu + 1.0))));
        return delay;
    }

    // ---------
    double gnss_model_tropo_baby::getZWD(const Triple &Ell, const base_time &epoch)
    {

        // not implemeted yet
        return 0.0;
    }

    // Chao model
    // --------
    double gnss_model_tropo_chao::getZHD(const Triple &Ell, const base_time &epoch)
    {

        // not implemeted yet
        return 0.0;
    }

    // Chao model (ZWD), Mendes pp. 85
    // --------
    double gnss_model_tropo_chao::getZWD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N, alpha;
        double hh = 50.0 * exp(-6.396e-4 * Ell[2]);
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        T += 273.15;
        alpha = 0.0065; // model is not very sensitive to the temperature lapse rate

        double e = hh / 100.0 * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

        double delay = 4.70 * 100.0 * (pow(e, 1.23) / (T * T)) + 1.71 * 1.e6 * (pow(e, 1.46) / (T * T * T)) * alpha;
        return delay;
    }

    // ---------
    // Ifadis model
    // ---------
    double gnss_model_tropo_ifad::getZHD(const Triple &Ell, const base_time &epoch)
    {

        // not implemeted yet
        return 0.0;
    }

    // ---------
    double gnss_model_tropo_ifad::getZWD(const Triple &Ell, const base_time &epoch)
    {

        double P, T, N;
        double hh = 50.0 * exp(-6.396e-4 * Ell[2]);
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        T += 273.15;

        double e = hh / 100.0 * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

        double delay = 0.00554 - 0.0880 * 1.e-4 * (P - 1000.0) + 0.272 * 1.e-4 * e + 2.771 * (e / T);
        return delay;
    }

} // namespace
