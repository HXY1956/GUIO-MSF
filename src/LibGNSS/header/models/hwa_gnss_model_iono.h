
/**
* @file        giono.h
* @brief    implements ionosphere model class
*/

#ifndef hwa_iono_H
#define hwa_iono_H

#include "hwa_gnss_all_Nav.h"
#include "hwa_gnss_data_ionex.h"
#include "hwa_gnss_data_ion.h"
#include "hwa_gnss_data_SATDATA.h"

namespace hwa_gnss
{
    /** @brief model of IONO    */
    class hwa_gnss_model_iono
    {
    public:
        /** @brief Constructor    */
        hwa_gnss_model_iono();

        /** @brief default destructor. */
        virtual ~hwa_gnss_model_iono();

        /** @brief ionosphere mapPing function
        *
        *param[in] mapFunc            mapPing function parameter
        *param[in] ele                elevation
        @return double                result
        */
        double ionMapFunc(IONMPFUNC mapfunc, double ele);

    protected:
    };
    /** @brief model of brdc    */
    class hwa_gnss_model_iono_brdc : public hwa_gnss_model_iono
    {
    public:
        /** @brief Constructor    */
        hwa_gnss_model_iono_brdc() {}

        /** @brief default destructor. */
        virtual ~hwa_gnss_model_iono_brdc() {}

        /**
        * @brief compute ionospheric delay by broadcast ionosphere model (klobuchar model)
        * @param[in] nav         store iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
        * @param[in] epo         curruent epoch.
        * @param[in] satdata     provide azimuth/elevation angle {az,el} (rad)
        * @param[in] site_ell    receiver position {lat,lon,h} (rad,m)
        * @return double         ionospheric delay (L1) (m)
        **/
        double getIonoDelay(gnss_all_nav *nav, gnss_data_sats &satdata, const base_time &epo, const Triple &site_ell);
    };

    /** @brief model of ionf    */
    class hwa_gnss_model_iono_ionf : public hwa_gnss_model_iono
    {
    public:
        /** @brief Constructor    */
        hwa_gnss_model_iono_ionf(){};

        /** @brief default destructor. */
        virtual ~hwa_gnss_model_iono_ionf(){};

        /**
        * @brief compute ionospheric delay 
        * @param[in] epo         curruent epoch.
        * @param[in] sat         satlite name
        * @param[in] rec         recievers
        * @return double         ionospheric delay
        **/
        double getIonoDelay(const base_time &epo, const std::string &sat, const std::string &rec);

    protected:
        gnss_data_ion *_gion; ///< ion file data
    };

    /** @brief model of sbas    */
    class hwa_gnss_model_iono_sbas : public hwa_gnss_model_iono
    {
    public:
        /** @brief Constructor    */
        hwa_gnss_model_iono_sbas() {}

        /** @brief default destructor. */
        virtual ~hwa_gnss_model_iono_sbas() {}
    };

    /** @brief get ionospheric delay by tec grid data    */
    class hwa_gnss_model_iono_tecgrid : public hwa_gnss_model_iono
    {
    public:
        /** @brief Constructor    */
        hwa_gnss_model_iono_tecgrid() {}

        /** @brief default destructor. */
        virtual ~hwa_gnss_model_iono_tecgrid() {}

        /**
        * @brief compute ionospheric delay
        * @param[in] ionexdata            ionex file data
        * @param[in] satdata            satlite data
        * @param[in] epo                epoch
        * @param[in] site_pos            position
        * @param[in] value                value
        * @param[in] rms                RMS
        * @return bool                    status of calculation
        **/
        bool getIonoDelay(gnss_data_ionex *ionexdata, gnss_data_sats &satdata, const base_time &epo, Triple &site_pos, double &value, double &rms);
    };

} // namespace

#endif
