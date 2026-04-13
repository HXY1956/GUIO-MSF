#ifndef hwa_base_globaltrans_h
#define hwa_base_globaltrans_h
#include <string>
#include "hwa_base_eigendef.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"

namespace hwa_gnss {
    class gnss_data_sats;
}

namespace hwa_base
{
    int ell2xyz(const double* Ell, double* XYZ, bool degrees); // True/False:  LatLon in Degrees/Radians!
    int xyz2ell(const double* XYZ, double* Ell, bool degrees); // True/False:  LatLon in Degrees/Radians!
    int xyz2neu(const double* XYZ, const double* XYZ_Ref, double* neu);
    int neu2xyz(const double* Ell, const double* neu, double* xyz); // ! Radians only: Ell[0], Ell[1]
    int rao2xyz_rot(const Triple& pos, const Triple& vel, SO3& R);

    int rao2xyz(const Triple& pos, const Triple& vel,
        const Triple& rao, Triple& xyz);
    int xyz2rao(const Triple& pos, const Triple& vel,
        Triple& xyz, Triple& rao);

    int ell2xyz(const Triple& ell, Triple& xyz, bool degrees);
    int xyz2ell(const Triple& crd, Triple& ell, bool degrees); // True/False:  LatLon in Degrees/Radians!
    int xyz2ell2(const Triple& crd, Triple& ell, bool degrees);      // True/False:  LatLon in Degrees/Radians!
    int xyz2ell_vlbi(const Triple& crd, Triple& ell);
    void xyz2neu(Triple& ell, Triple& xyz, Triple& neu); // Radians only: Ell[0], Ell[1]
    void neu2xyz(Triple& ell, Triple& neu, Triple& xyz); // Radians only: Ell[0], Ell[1]

    int xyz2neu(Triple& xyz, Symmetric& Q_xyz, Symmetric& Q_neu);

    int ell2ipp(hwa_gnss::gnss_data_sats* satdata, Triple& ell_site, Triple& ell_ipp, bool GPStkflag = false); 
    int ell2ipp(hwa_gnss::gnss_data_sats* satdata, Triple& ell_site, double radius, double ion_hgt, Triple& ell_ipp);

    SO3 rotm(double angle, int type);
    SO3 drotm(double angle, int type);

    Triple Cart2Geod(const Triple& CartPos, bool b);
    Triple Geod2Cart(const Triple& GeodPos, bool b);
    Triple XYZ2ENU(const Triple& xyz, const Triple& xyz_ref);
}

#endif

