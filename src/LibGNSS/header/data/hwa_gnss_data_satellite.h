#ifndef hwa_gnss_data_satellite_H
#define hwa_gnss_data_satellite_H

#include <string>
#include <vector>

#include "hwa_base_eigendef.h"
#include "hwa_base_data.h"
#include "hwa_gnss_data_obsmanager.h"
#include "hwa_base_time.h"
#include "hwa_gnss_all_Nav.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_data_satellite : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_satellite();

        // gnss_data_satellite(string site, string sat, const base_time& t);  // TOTO ODSTRANIT ?!
        gnss_data_satellite(base_log spdlog);

        /** @brief constructor 1. */
        gnss_data_satellite(base_log spdlog, gnss_data_obs_manager* obs);

        /** @brief default destructor. */
        virtual ~gnss_data_satellite();

        /** @brief add satellite position. */
        void addcrd(const Triple& crd);

        /** @brief add satellite clocks at the transmision time. */
        void addclk(double d);

        /** @brief determine wheather eclipsed or not. */
        void addecl(Triple& sat, base_time& epoch);

        /** @brief add satellite pos, clk and ecl ( // true to support INP:chk_health settings only. */
        int addprd(gnss_all_nav* gnav, bool msk_health = true);

        /**
         * @brief
         *
         * @param d
         */
        void addele(double d); ///< add satellite elevation

        /**
         * @brief
         *
         * @param d
         */
        void addazi(double d); ///< add satellite azimuth

        /**
         * @brief
         *
         * @param d
         */
        void addrho(double d); ///< add satellite rho-vector

        /**
         * @brief
         *
         * @return Triple
         */
        Triple satcrd(); ///< get satellite position

        /**
         * @brief
         *
         * @return double
         */
        double clk(); ///< get satellite clocks at the transmision time

        /**
         * @brief
         *
         * @return double
         */
        double dclk();

        /**
         * @brief
         *
         * @return double
         */
        double ele(); ///< get satellite elevation

        /**
         * @brief
         *
         * @return double
         */
        double ele_deg(); ///< get satellite elevation [deg]

        /**
         * @brief
         *
         * @return double
         */
        double azi(); ///< get satellite azimuth

        /**
         * @brief
         *
         * @return double
         */
        double rho(); ///< get satellite rho-vector

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool ecl(); ///< get eclipsing

        /**
         * @brief
         *
         */
        void clear();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        bool valid();

        gnss_data_obs_manager* gobs; ///< pointer na gobsgnss

    private:
        /**
         * @brief
         *
         */
        virtual void _clear();

        /**
         * @brief
         *
         * @return true
         * @return false
         */
        virtual bool _valid() const;

        Triple _satcrd; ///< satellite position (X,Y,Z)
        double _clk;       ///< satellite clocks (precise, time of transmision) (meters)
        double _dclk;      ///< satellite clocks drift(precise, time of transmision) (meters/s)
        double _ele;       ///< satellite elevation
        double _azi;       ///< satellite azimuth
        double _rho;       ///< satellite-station geometrical distance
        bool _eclipse;     ///< eclipsed satellite
        base_time _lastEcl;  ///< last eclipse time
    };

} // namespace

#endif
