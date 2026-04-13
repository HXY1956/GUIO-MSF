/**
* @file                gtrs2crs.h
* @brief            calculate the rotation matrix from TRS to CRS and the corresponding partial derivation matrix
*/

#ifndef hwa_gnss_TRS2CRS_H
#define hwa_gnss_TRS2CRS_H

#include "hwa_base_time.h"
#include "hwa_gnss_data_poleut.h"
#include "hwa_gnss_data_poleutVLBI.h"
#include "hwa_base_const.h"
#include "hwa_gnss_data_interp.h"
#include "hwa_gnss_data_leapsecond.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging pole and ut1 data for one day
    */
    class gnss_base_pudaily
    {
    public:
        /** @brief default constructor. */
        gnss_base_pudaily();

        /** @brief default destructor. */
        virtual ~gnss_base_pudaily(){};

        base_time time; ///< epoch time
        double xpole; ///< x pole
        double ypole; ///< y pole
        double ut1;   ///< dut1
        double dx;    ///< dx pole
        double dy;    ///< dy pole
    };

    /**
     *@brief       Class for nutation for one epoch
     */
    class gnss_base_epoch_nutation
    {
    public:
        /** @brief default constructor. */
        gnss_base_epoch_nutation();

        /** @brief default destructor. */
        virtual ~gnss_base_epoch_nutation(){};

        double T;  ///< epoch
        double Pi; ///< nutation angle of the longitude(radian)
        double Ep; ///< nutation angle of the obliquity(radian)
    };
    /**
     *@brief       Class for zonaltide data
     */
    class gnss_base_zonaltide
    {
    public:
        /** @brief default constructor. */
        gnss_base_zonaltide();

        /** @brief default destructor. */
        virtual ~gnss_base_zonaltide(){};

        base_time time; ///< epoch time
        double ut1;   ///< dut1
        double lod;   ///< dlod
        double omega; ///< omega
    };

    /**
    *@brief       Class for calculating the rotation matrix from TRS to CRS and the corresponding partial derivation matrix
    */
    class gnss_base_trs2crs
    {
    public:
        /** @brief default constructor. */
        gnss_base_trs2crs();

        /** @brief constructor. */
        gnss_base_trs2crs(std::string cver);

        /** @brief constructor. */
        gnss_base_trs2crs(bool erptab, gnss_data_poleut *poleut);
        /**
         * @brief Construct a new t gtrs2crs object
         * @param[in]  erptab    if ERP table is used or not
         * @param[in]  poleut   Earth orientation parameters
         * @param[in]  cver      IAU version: 00(default) or 06
         */
        gnss_base_trs2crs(bool erptab, gnss_data_poleut *poleut, std::string cver);
        /**
         * @brief Construct a new t gtrs2crs object
         * @param[in]  erptab    if ERP table is used or not
         * @param[in]  poleut   Earth orientation parameters
         */
        //gnss_base_trs2crs(bool erptab, gnss_data_poleutvlbi *poleut);
        //gnss_base_trs2crs(bool erptab, gnss_data_poleut *poleut);
        /**
          * @brief define = for gnss_base_trs2crs
          * @param[in]  Other     other gnss_base_trs2crs
          * @return gnss_base_trs2crs& =
          */
        gnss_base_trs2crs &operator=(const gnss_base_trs2crs &Other);

        Matrix fund_arg(double t, int opt);

        double rg_zont2(double mjd, int opt35);

        Triple eop_eanes(double rjd);

        base_pair pm_libration(double rjd);

        double ut_libration(double rjd);

        Triple xys2006a(double mjd);
        void ut12ut1r(base_time &t, double *x, gnss_data_poleut *poleut, std::string type = "");

        std::vector<double> eophf(double tt);

        /** @brief default destructor. */
        virtual ~gnss_base_trs2crs(){};

        /** @brief calculte process matrix. */
        void calcProcMat(const bool &partial, const int &axis, const double &angle, std::vector<Matrix> &rot);

        /** @brief calculate the rotation matrix. */
        //** yqyuan: renamed as calcRotMat_equinox; adding ldpsi, ldeps for nutation in 2021/08/25 */
        void calcRotMat_equinox(const base_time &epoch, const bool &ldxdpole, const bool &ldydpole, const bool &ldudpole, const bool &ldpsi, const bool &ldeps);

        /** @brief calculate the rotation matrix using the new CIO based method. */
        /** yqyuan added in 2021/08/20 */
        void calcRotMat(const base_time &epoch, const bool &ldxdpole, const bool &ldydpole, const bool &ldudpole, const bool &ldX, const bool &ldY);

        void calcRotMat_vlbi(const base_time &epoch, const bool &ldxdpole, const bool &ldydpole, const bool &ldudpole, const bool &ldX, const bool &lY);

        //todo const??
        /** @brief return rotation matrix. */
        Matrix &getRotMat();

        /** @brief return du matrix. */
        Matrix &getMatDu();

        /** @brief return dx matrix. */
        Matrix &getMatDx();

        /** @brief return dy matrix. */
        Matrix &getMatDy();

        /** @brief return dy matrix. */
        Matrix &getMatDp();

        /** @brief return dy matrix. */
        Matrix &getMatDe();

        /** @brief return xpole. */
        double getXpole();
        /** @brief return ypole. */
        double getYpole();
        /** @brief return gmst. */
        double getGmst();
        /** @brief return now epoch. */
        base_time getCurtEpoch();

    protected:
        /** @brief interpolate short term tidal corrections(fortran: polut1_ray_interpolation). */
        void _tide_corrections(base_time &t, Triple &xyu);

        /** @brief calculate short term tidal corrections(fortran: polut1_ray_calc). */
        gnss_base_pudaily &_tideCor1Cal(gnss_base_pudaily &b);

        /** @brief calculate the diurnal and semi-diurnal variations in Earth Orientation Parameters (x,y, UT1) from ocean tides(fortran: ORTHO_EOP). */
        void _ORTHO_EOP(base_time &t, Triple &eop);

        /** @brief calculate the diurnal and semi-diurnal variations in Earth Orientation Parameters (x,y, UT1, LOD) from ocean tides (fortran: interp.f). */
        /** agree with _ORTHO_EOP at the level of a few microarcseconds in polar motion and a few tenths of a microsecond in UT1*/
        /** create by yqyuan from interp.f 20210829; calculate both UT1 and LOD; LOD not outputed antway */
        void _PMUT1_OCEANS(base_time &t, Triple &eop);

        /** @brief calculate the partials of the tidal variations to the orthoweights(fortran: CNMTX). */
        void _CNMTX(base_time &t, double *h);

        /** @brief evaluate the model of polar motion for a nonrigid Earth due to tidal gravitation. */
        void _PMSDNUT2(base_time &t, double *pm);

        /** @brief evaluate the model of subdiurnal libration in the axial component of rotation, expressed by UT1 and LOD.. */
        void _UTLIBR(base_time &t, double *temp);

        /** @brief calculate zonaltide correction on ut1(fortran: ut1_zonaltide_interpolation). */
        double _tideCor2(const double &dRmjd);

        /** @brief evaluate the effects of zonal Earth tides on the rotation of the Earth. */
        void _RG_ZONT2(const double &dT, double *DUT, double *DLOD, double *DOMEGA);
        void _cal_dPole_dut1(base_time &t, double *x, gnss_data_poleut *poleut, std::string type);
        /** @brief calculate fund arg. */
        void _FUNDARG(const double &T, double *L, double *LP, double *F, double *D, double *OM);
        /** @brief Mean anomaly of the Moon. */
        double _iauFal03(const double &t);
        /** @brief Mean anomaly of the Sun. */
        double _iauFalp03(const double &t);
        /** @brief Mean argument of the latitude of the Moon. */
        double _iauFaf03(const double &t);
        /** @brief Mean elongation of the Moon from the Sun. */
        double _iauFad03(const double &t);
        /** @brief Mean longitude of the ascending node of the Moon. */
        double _iauFaom03(const double &t);
        /** @brief Planetary longitudes, Mercury. */
        double _iauFame03(const double &t);
        /** @brief Planetary longitudes, Venus. */
        double _iauFave03(const double &t);
        /** @brief Planetary longitudes, Earth. */
        double _iauFae03(const double &t);
        /** @brief Planetary longitudes, Mars. */
        double _iauFama03(const double &t);
        /** @brief Planetary longitudes, Jupiter. */
        double _iauFaju03(const double &t);
        /** @brief Planetary longitudes, Saturn. */
        double _iauFasa03(const double &t);
        /** @brief Planetary longitudes, Uranus. */
        double _iauFaur03(const double &t);
        /** @brief Planetary longitudes, Neptune. */
        double _iauFane03(const double &t);
        /** @brief General accumulated precession in longitude. */
        double _iauFapa03(const double &t);

        /** @brief calculate _pudata(fortran: polut1_interpolation). */
        void _calPoleut1(base_time &t, double *x, gnss_data_poleut *poleut, std::string type = "");

        //void _calPoleut1_lagint(base_time &t, double *x, gnss_data_poleutvlbi *poleut, std::string type = "");
        void _calPoleut1_lagint(const base_time &t, double *x, gnss_data_poleut *poleut, std::string type = "");

        double _lagrange_interplate_eop(const std::vector<double> &X, const std::vector<double> &Y, double x);

        /** @brief interpolate the nutation(fortran: nutation_interpolation). */
        void _nutInt(const double &dRmjd, double *dpsi, double *deps, const double &step);

        /** @brief calculate nutation according to IAU 2000A model(fortran: nu2000a). */
        void _nutCal(const double &dATE1, const double &dATE2, double *dPSI, double *dEPS);

        /** @brief calculate nutation according to IAU 2000A model adjusted for IAU2006 precession. */
        /** Created: yqyuan 2021.05.10                                                              */
        /**  Tested: yqyuan 2021.05.10                                                              */
        void _nutCal_06(double dATE1, double dATE2, double *dPSI, double *dEPS);

        /** @brief calculate free-core-nutation (FCN) using a fcn model */
        /** Created: yqyuan 2021.08.05                                                              */
        /**  Tested: yqyuan 2021.08.05                                                              */
        void _nutFCN(const double &dRmjd, double *dX, double *dY, double *sigdX, double *sigdY);

        /** @brief interpolation. */
        double _interpolation(const int &iOrder, const int &iPoint, double *pdX, double *pdY, const double &dXin);

        /** @brief calculate rotation matrix of Precession and nutation. */
        // void      _process2000(const double& dRmjd, const double& dpsi, const double& deps);
        //** yqyuan: adding ldpsi, ldeps for nutation in 2021/08/25 */
        void _process2000(const double &dRmjd, const double &dpsi, const double &deps, Matrix &qmat, const bool &ldpsi, const bool &ldeps);

        /** @brief calculate rotation matrix of IAU2006/2000A Precession and nutation. */
        /** Created: yqyuan 2021.05.10                                                 */
        /**  Tested: yqyuan 2021.05.10                                                 */
        //** yqyuan: adding ldpsi, ldeps for nutation in 2021/08/25 */
        void _process2006(const double &dRmjd, const double &dpsi, const double &deps, Matrix &qmat, const bool &ldpsi, const bool &ldeps);

        /** @brief calculate X/Y coordinates of celestial intermediate pole            */
        /** from series based on IAU 2000 precession and IAU 2000A nutation.           */
        /** Created: yqyuan 2021.08.22 from IERS Conventions 2003 and SOFA .F          */
        /** Tested: yqyuan 2021.XX.XX                                                  */
        /** Not suggested ??                                                           */
        void _iau_XY00(double date1, double date2, double *x, double *y);

        /** @brief calculate X/Y coordinates of celestial intermediate pole            */
        /** from series based on IAU 2006 precession and IAU 2000A nutation.           */
        /** Created: yqyuan 2021.08.20 from SOFA package                               */
        /**  Tested: yqyuan 2021.XX.XX                                                 */
        /** Not suggested ??                                                           */
        void _iau_XY06(double date1, double date2, double *x, double *y);

        /** @brief calculate the CIO locator s, IAU 2000/2006. */
        /** Created: yqyuan 2021.08.20 from SOFA package       */
        /**  Tested: yqyuan 2021.08.20                         */
        double _iau_CIO_locator(double date1, double date2, double x, double y);

        /** @brief calculate the CIO locator s, IAU 2006. */
        /** Created: yqyuan 2021.05.10 from SOFA package  */
        /**  Tested: yqyuan 2021.05.10                    */
        //double _iau_S06(double date1, double date2, double x, double y);

        /** @brief calculate the angle (unit: radian) according to the current epoch and J2000(fortran: sp2000). */
        double _sp2000(const double &dDATE1, const double &dDATE2);

        /** @brief calculate the base_earth rotation angle (unit: radian)(fortran: era2000). */
        double _era2000(const double &dJ1, const double &dJ2);

        /** @brief calculate Greenwich sidereal time, IAU 2000. */
        double _gst2000(const double &dRmjd, const double &era, const double &dpsi);

        /** @brief calculate Greenwich sidereal time, IAU 2006. */
        /** called: _iau_S06; _iau_Eors                         */
        /** Created: yqyuan 2021.05.10                          */
        /**  Tested: yqyuan 2021.05.10                          */
        double _gst2006(const double &dRmjd, const double &era, const double &dpsi);

        /** @brief extract X/Y from BPN(Q) matrix            */
        /** Created: yqyuan 2021.08.20 from SOFA package  */
        /**  Tested: yqyuan 2021.08.20                    */
        void _iau_bpn2xy(Matrix qmat, double *X, double *Y);

        /** @brief Ebase_quation of the origins, given the classical NPB matrix and the quantity s. */
        /** Created: yqyuan 2021.05.10 from SOFA package                                       */
        /**  Tested: yqyuan 2021.05.10                                                         */
        double _iau_Eors(Matrix rnpb, double s);

        /** @brief fmod to (0,2pi). */
        double _iau_anp(const double &dA);

        /** @brief Ebase_quation of the equinoxes complementary terms **/
        /*    consistent with IAU 2000 resolutions(return complementary terms (radians)). */
        /* used for both GAST2000 and GAST 2006 */
        double _eect2000(const double &dRmjd);

        /** @brief fmod. */
        double _iau_anpm(const double &dA);

        //void rotation_matrix(bool ldot, int iaxis, double angle, double *protmat, double *pdrotmat);  //generate rotation matrix and its derivation given the rotating angle and axis(bool ldot : true - with derivation false - without derivation)

        // inner variable
        bool _erptab = false; ///< if ERP table is used or not
        base_time _epo;         ///< cureent epoch
        Triple _pudata;    ///< pole and ut1 data after interpolation(xpole, ypole and ut1-tai)
        base_time _tdt;         ///< dynamic time
        double _arg[5];       ///< fundamental arguements of nutation in radius
        Matrix _qmat;         ///< TODO
        double _epsa = 0.0;   ///< TODO

        // out variable
        double _xpole = 0.0; ///< x pole of pole shift
        double _ypole = 0.0; ///< y pole of pole shift
        double _gmst = 0.0;  ///< Greenwich Mean Sidereal Time
        Matrix _rotmat;      ///< rotation matrix from TRS to CRS
        Matrix _rotdu;       ///< partial of rotmat wrt to ut1.
        Matrix _rotdx;       ///< partial of rotmat wrt to xpole
        Matrix _rotdy;       ///< partial of rotmat wrt to ypole

        Matrix _rotdpsi; ///< partial of rotmat wrt to psi of nutation
        Matrix _rotdeps; ///< partial of rotmat wrt to eps of nutation

        gnss_data_poleut *_poleut; ///< poleut data
        //gnss_data_poleutvlbi *_poleutvlbi; ///< TODO
        gnss_data_poleut *_poleutvlbi; ///< TODO

        // orig static values
        gnss_base_epoch_nutation sTB1[3]; ///< TODO
        gnss_base_zonaltide sTZB[3];     ///< TODO
        gnss_base_pudaily tb0, tb1;      ///< TODO

        // constant values
        const double _rr = 1.00273781191135448e0; ///< r = gmst/ut1 = 1.00273781191135448D0 (ERA2000.f90, IERS20xx)
        const double _djc = 36525.0;              ///< days per julian century

        // === void gnss_base_trs2crs::_process2000(const double& dRmjd, const double& dpsi, const double& deps) ===
        const double dEps0 = 84381.448 / (RAD2SEC);      ///< J2000 obliquity (Lieske et al. 1977)
        const double dEps0_2006 = 84381.406 / (RAD2SEC); ///< J2000 obliquity in IAU2006
        const double dRa0 = -0.0146 / (RAD2SEC);         ///< The ICRS RA of the J2000 equinox (Chapront et al., 2002)
        const double dPrecor = -0.29965 / (RAD2SEC);     ///< The precession and obliquity corrections (radians per century) (page 43, equ(27))
        const double dOblcor = -0.02524 / (RAD2SEC);     ///< The obliquity corrections (radians per century) (page 43, equ(27))
        const double dPsibi = -0.041775 / (RAD2SEC);     ///< The frame bias corrections in longitude and obliquity (page 43, equ(28) ?)
        const double dEpsbi = -0.0068192 / (RAD2SEC);    ///< The eps bias corrections in longitude and obliquity (page 43, equ(28) ?)

        // === double gnss_base_trs2crs::_era2000(const double& dJ1, const double& dJ2) ===
        const double dPI = 3.141592653589793238462643;  ///< PI
        const double d2PI = 6.283185307179586476925287; ///< DOUBLE PRECISION DJ0
        const double dJ0 = 51544.5;                     ///< Reference epoch (J2000), JD

        // === double gnss_base_trs2crs::_eect2000(const double& dRmjd) ===
        const double dAS2R = 4.848136811095359935899141 * 0.000001; ///<         *  Arcseconds to radians
        const double dJC = 36525;                                   ///<         *  Days per Julian century

        // == void  gnss_base_trs2crs::_nutCal(const double& dATE1, const double& dATE2, double *dPSI, double *dEPS) ===
        const double dDAS2R = 4.848136811095359935899141e-6;
        const double dDMAS2R = dDAS2R / 1e3;
        const double dTURNAS = 1296000e0;
        const double dU2R = dDAS2R / 1e7;
        const double dDJ0 = 51544.5e0;
        const double dDJC = 36525e0;

        // === void gnss_base_trs2crs::_PMSDNUT2(base_time& t,double* pm) ===
        const double _sec2rad = 4.848136811095359935899141e-6;
        const double _turnas = 1296000.0;
        const double rmjd0 = 51544.5;
        const double rad2sec = 86400.0 / d2PI;

        // IAU2000A or IAU2006/2000A; added by yqyuan 2021.05.10
        std::string _cver; ///< IAU version: 00(default) or 06
    };
} // namespace

#endif