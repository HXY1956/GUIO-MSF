#ifndef hwa_gnss_data_ionex_H
#define hwa_gnss_data_ionex_H

#include "hwa_set_gproc.h"
#include "hwa_base_time.h"
#include "hwa_base_globaltrans.h"
#include "hwa_gnss_data_interp.h"
#include "hwa_gnss_data_SATDATA.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief     Class for storaging ionex file data
    */
    class gnss_data_ionex_head
    {
    public:
        /** @brief default constructor. */
        gnss_data_ionex_head();
        /** @brief default constructor. */
        virtual ~gnss_data_ionex_head();

        /** @brief assignment operator . */
        void operator=(const gnss_data_ionex_head &Other);

        base_time beg;                                ///< EPOCH OF FIRST MAP
        base_time end;                                ///< EPOCH OF LAST MAP
        int interval;                               ///< interval between the TEC mapS
        int map_dimension;                          ///< Dimension of TEC/RMS mapS -- 2/3
        std::tuple<double, double, double> hgts;         ///< definition of an equidistant grid in height [HGT1 / HGT2 / DHGT]
                                                    /// HGT1 -> HGT2 with increment DHGT (in km), e.g. 50.0 450.0  50.0
                                                    ///< for 2-dimension, HGT1 = HGT2. here, stored in meters
        std::tuple<double, double, double> lons;         ///< definition of an equidistant grid in longitude [LON1 / LON2 / DLON]
                                                    /// LON1 -> LON2 with increment DLON (in degree), e.g. -180.0 180.0   5.0
        std::tuple<double, double, double> lats;         ///< definition of an equidistant grid in latitude [LAT1 / LAT2 / DLAT]
                                                    /// LAT1 -> LAT2 with increment DLAT (in degree), e.g. 87.5 -87.5  -2.5
        int nhgt;                                   ///< height in n frame
        int nlon;                                   ///< longitude in n frame
        int nlat;                                   ///< latitude in n frame
        int exponent;                               ///< default -1
        double base_radius;                         ///< mean base_earth radius or bottom of height, here, stored in meters
        IONMPFUNC ion_mapfunc;                      ///< COSZ: 1/cos(z) / QFAC: Q-factor
        std::map<std::string, std::pair<double, double>> p1p2_dcb; ///< sat  bias[ns]  rms
    };
    /**
    *@brief     Class for storaging ionex file data
    */
    class gnss_data_ionex_DATA
    {
    public:
        /** @brief default constructor. */
        gnss_data_ionex_DATA(){};
        /** @brief default constructor. */
        virtual ~gnss_data_ionex_DATA(){};

        std::map<std::tuple<int, int, int>, double> tec_val; ///< lat_index/lon_index/height_index , unit TECU
        std::map<std::tuple<int, int, int>, double> tec_rms; ///< lat_index/lon_index/height_index , unit TECU
    };

    /**
    *@brief     Class for storaging ionex file data
    */
    class gnss_data_ionex : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ionex();

        /** @brief default constructor. */
        gnss_data_ionex(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ionex();

        /**
        * @brief add head of GIM grid files.
        * @param[in] ionex_hd   head structure.
        */
        void add_head(gnss_data_ionex_head ionex_hd);

        /**
        * @brief add TEC grid data.
        * @param[in] time             curruent epoch.
        * @param[in] type             TEC_MAP/RMS_MAP
        * @param[in] ilat/ilon/ihgt   position of gnss_data_ionex_DATA structure.
        * @param[in] v                corresponding TEC.
        * @return                      void
        */
        void add_data(const base_time &time, std::string type, int ilat, int ilon, int ihgt, double v);

        /**
        * @brief get STEC by GIM grid data (position and time interpolation)
        * @param[in] time        curruent epoch.
        * @param[in] satdata     satellite data, provide elevation and azimuth.
        * @param[in] site_ell    receiver position {lat,lon,h}.
        * @param[out] value      STEC value for the satellite, unit : TECU.
        * @param[out] rms        rms value, unit : TECU.
        * @return                 true success, false fail
        */
        bool getSTEC(const base_time &time, gnss_data_sats &satdata, Triple &site_ell, double &value, double &rms);

    protected:
        gnss_data_ionex_head _ionex_hd;                     ///< ionex head info
        std::map<base_time, gnss_data_ionex_DATA> _map_ionex_data; ///< std::map of  ionex data

        /**
        * @brief interpolate tec grid data according ipp coordinate (position interpolation)
        * @param[in] time        curruent epoch.
        * @param[in] ipp_ell     ionospheric pierce point position(B/L).
        * @param[in] ihgt        which ionosphere layer
        * @param[out] value      VTEC at ipp, unit : TECU.
        * @param[out] rms        rms of VTEC at ipp, unit : TECU.
        * @return                 
            @retval bool         true success, false fail
        */
        bool _interpolate_ion(const base_time &time, const Triple &ipp_ell, const int &ihgt, double &value, double &rms);
    };
}

#endif // !GION_H
