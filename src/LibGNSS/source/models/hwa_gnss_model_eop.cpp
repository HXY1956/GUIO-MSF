#include "hwa_base_eigendef.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_model_eop.h"

using namespace hwa_base;
using namespace std;

namespace hwa_gnss
{
    gnss_model_eop::gnss_model_eop()
    {
    }

    //Destructor
    gnss_model_eop::~gnss_model_eop()
    {
    }

    // Nutation Matrix
    // --------------------------------------
    Matrix gnss_model_eop::nutMatrix(double mjd)
    {
        //  boost::mutex::scoped_lock lock(_mutex);

        Matrix I(3, 3);
        I << 1, 0, 0
          ,0 , 1 , 0
          ,0 , 0 , 1;

        return I;
    }

    // Precession Matrix
    // ------------------------------------------
    Matrix gnss_model_eop::precMatrix(double mjd_1)
    {
        //  boost::mutex::scoped_lock lock(_mutex);

        Matrix I(3, 3);
        I << 1 , 0 ,0
          ,0 , 1 , 0
          ,0 , 0 , 1;

        return I;
    }

    // Normalize angle into interval 0 - 2pi
    // -----------------------------------
    double gnss_model_eop::_normangle(double x)
    {
        double norm;
        norm = fmod(x, 2 * hwa_pi);
        if (norm < 0)
            norm += 2 * hwa_pi;

        return norm;
    }

} // namespace
