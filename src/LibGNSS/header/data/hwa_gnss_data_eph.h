#ifndef hwa_gnss_data_eph_H
#define hwa_gnss_data_eph_H

#include <memory>

#include "hwa_base_time.h"
#include "hwa_base_io.h"
#include "hwa_base_data.h"
#include "hwa_gnss_sys.h"

namespace hwa_gnss
{
    /** @brief navigation data. */
    enum NAVDATA
    {
        NAV_UNDEF,  ///< undefined
        NAV_A,      ///< nav a
        NAV_E,      ///< nav e
        NAV_M,      ///< nav m
        NAV_I,      ///< nav i
        NAV_IDOT,   ///< nav idot
        NAV_OMEGA,  ///< nav omega
        NAV_OMG,    ///< nav omg
        NAV_OMGDOT, ///< nav omgdot
        NAV_DN,     ///< nav dn
        NAV_CRC,    ///< nav crc
        NAV_CIC,    ///< nav cic
        NAV_CUC,    ///< nav cuc
        NAV_CRS,    ///< nav crs
        NAV_CIS,    ///< nav cis
        NAV_CUS,    ///< nav cus
        NAV_F0,     ///< nav f0
        NAV_F1,     ///< nav f1
        NAV_F2,     ///< nav f2
        NAV_X,      ///< nav x
        NAV_XD,     ///< nav xd
        NAV_XDD,    ///< nav xdd
        NAV_Y,      ///< nav y
        NAV_YD,     ///< nav yd
        NAV_YDD,    ///< nav ydd
        NAV_Z,      ///< nav z
        NAV_ZD,     ///< nav zd
        NAV_ZDD,    ///< nav zdd
        NAV_IOD,    ///< nav iod
        NAV_HEALTH, ///< nav health
        NAV_TGD0,   ///< nav tgd0
        NAV_TGD1,   ///< nav tgd1
        NAV_TGD2,   ///< nav tgd2
        NAV_TGD3    ///< nav tgd3
    };

    /** @brief time + double. */
    typedef std::pair<hwa_base::base_time, double> hwa_map_time_value;

    /** @brief class for navigation data. */
    class gnss_data_eph : public hwa_base::base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_eph();
        explicit gnss_data_eph(hwa_base::base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_eph();

        /**
        * @brief get the clock offset value
        * @param[in]  t            GPST.
        * @param[in]  clk        clock offset value.
        * @param[in]  var        clock variation, default value is NULL.
        * @param[in]  dclk        clock difference, default value is NULL.
        * @param[in]  chk_health    the heath of satellite, default value is true.
        * @return      int 
        */
        virtual int clk(const hwa_base::base_time &t, double *clk, double *var = NULL, double *dclk = NULL, bool chk_health = true) { return -1; } // [s]

        /**
        * @brief get the position value
        * @param[in]  t            GPST.
        * @param[in]  xyz        position value.
        * @param[in]  var        position variation, default value is NULL.
        * @param[in]  vel        vel, default value is NULL.
        * @param[in]  chk_health    the heath of satellite, default value is true.
        * @return      int
        */
        virtual int pos(const hwa_base::base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true) { return -1; } // [m]

        /**
        * @brief get the nav position value
        * @param[in]  t            GPST.
        * @param[in]  xyz        position value.
        * @param[in]  var        position variation, default value is NULL.
        * @param[in]  vel        vel, default value is NULL.
        * @param[in]  chk_health    the heath of satellite, default value is true.
        * @return      int
        */
        virtual int nav(const hwa_base::base_time &t, double xyz[3], double var[3] = NULL, double vel[3] = NULL, bool chk_health = true)
        {
            return this->pos(t, xyz, var, vel, chk_health);
        } // [m]

        /** @brief get the heath of satellite. */
        virtual bool healthy() const { return true; }

        /** @brief get the heath of satellite. (string) */
        virtual std::string health_str() const { return ""; }

        /** @brief get the chk. */
        virtual int chk() const { return -1; }

        /** @brief get the nav type. */
        virtual GNAVTYPE gnavtype(bool full = true) const { return NAV; }

        /** @brief get the src. */
        virtual int src(bool full = true) const { return 0; }

        /** @brief get the line format. */
        virtual std::string linefmt() const { return ""; }

        /** @brief get the line. */
        virtual std::string line() const { return ""; }

        /** @brief print line format. */
        virtual void print() const { std::cout << linefmt(); }

        /** @brief get the parameter of nav. */
        virtual hwa_map_time_value param(const NAVDATA &n);

        /** @brief override param. */
        virtual int param(const NAVDATA &n, double val);

        /** @brief cyclic. */
        virtual bool param_cyclic(const NAVDATA &n);

        /** @brief set the value of gio. */
        virtual void gio(std::shared_ptr<hwa_base::base_io> p) { _gio_ptr = p; }

        /** @brief get the value of gio. */
        std::shared_ptr<hwa_base::base_io> gio() { return _gio_ptr; }

        /** @brief clear data. */
        void clear();

        /** @brief valid. */
        bool valid();

        /** @brief override valid. */
        void valid(bool validity) { _validity = validity; }

        /** @brief get the name of GNSS system. */
        GSYS gsys() const;

        /** @brief get the name of satellite. */
        std::string gsat() const;

        // POZDEJI JEN GSAT a vse pres MUTEX !!!
        /** @brief get the value of _sat. */
        std::string sat() const { return _sat; }

        /** @brief get the value of validity interval. */
        double interval() const { return _interval; }

        /** @brief get the value of reference epoch. */
        hwa_base::base_time epoch() const { return _epoch; }

        /** @brief get the begin time of validity. */
        hwa_base::base_time begin() const { return _epoch - _interval / 2; }

        /** @brief get the end time of validity. */
        hwa_base::base_time end() const { return _epoch + _interval / 2; }

        /** @brief chktot. */
        virtual bool chktot(const hwa_base::base_time &t) { return true; }

    protected:
        /** @brief clear. */
        virtual void _clear();

        /** @brief valid. */
        virtual bool _valid() const;

        std::string _sat;      ///< satellite number
        hwa_base::base_time _epoch;   ///< reference epoch
        double _interval; ///< validity interval
        bool _validity;   ///< validity

        std::shared_ptr<hwa_base::base_io> _gio_ptr; ///< gio pointer

    private:
    };
}

#endif
