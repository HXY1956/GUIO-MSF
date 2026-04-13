#include <cmath>
#include <iomanip>
#include "hwa_gnss_model_tropoblind.h"

using namespace std;

namespace hwa_gnss
{
    double gnss_model_blindtropos::getZHD(const Triple &ell, const base_time &epo) // RAD
    {

        double zhd, zwd;
        double ele = 90.0;
        double doy = double(epo.doy());
        double lat = ell[0] * R2D;
        double hel = ell[2];

        _model->mops(lat, hel, ele, doy, zhd, zwd); // DEG

#ifdef DEBUG
        std::cout << " MOPS: " << std::fixed << setprecision(3)
             << "  DOY: " << setw(12) << doy
             << "  ELE: " << setw(12) << ele
             << "  ZHD: " << setw(12) << zhd
             << std::endl;
#endif

        return zhd;
    }

    // ---------
    // MOPS ZWD interface
    // ---------
    double gnss_model_blindtropos::getZWD(const Triple &ell, const base_time &epo) // RAD
    {

        double zhd, zwd;
        double ele = 90.0;
        double doy = double(epo.doy());
        double lat = ell[0] * R2D;
        double hel = ell[2];

        _model->mops(lat, hel, ele, doy, zhd, zwd); // DEG

#ifdef DEBUG
        std::cout << " MOPS: " << std::fixed << setprecision(3)
             << "  DOY: " << setw(12) << doy
             << "  ELE: " << setw(12) << ele
             << "  ZWD: " << setw(12) << zwd
             << std::endl;
#endif

        return zwd;
    }

} // namespace
