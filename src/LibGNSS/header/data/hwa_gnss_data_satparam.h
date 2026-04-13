/**
 * @file        gsatparam.h
 * @brief        The class for storaging one sat parameters data.
 */

#ifndef GSATPARAM_H
#define GSATPARAM_H
#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for storaging one satellite's parameters data
    *
    * The class contains one satellite's parameters for file
    *        and provide the interface to get parameters' value.
    */
    class gnss_data_satparam : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_satparam();

        gnss_data_satparam(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_satparam();

        bool valid = false; ///< effectiveness of the sat data

        /// set satelllite information(include prn,svn,start time and so on)
        /**
        * @brief save satellite prn from file.
        * @param[in]  prn    satellite prn
        */
        void set_prn(const std::string &prn);

        /**
        * @brief save satellite svn from file.
        * @param[in]  svn    satellite svn
        */
        void set_svn(const std::string &svn);

        /**
        * @brief save satellite launched time from file.
        * @param[in]  start        satellite launched time
        */
        void set_start(const base_time &start);

        /**
        * @brief save satellite decommissioned time from file.
        *
        * When the end time is equen to 0 or LAST_TIME, that means 
        *    the satellite is still in service.
        *
        * @param[in]  end        decommissioned time
        */
        void set_end(const base_time &end);

        /**
        * @brief save satellite's cosparid from file.
        * @param[in]  cosparid        satellite's cosparid
        */
        void set_cosparid(const std::string &cosparid);

        /**
        * @brief save satellite's weight from file.
        * @param[in]  mass        satellite's weight(kg)
        */
        void set_mass(double mass);

        /**
        * @brief save satellite's max_yaw from file.
        * @param[in]  maxyaw        satellite's max_yaw
        */
        void set_maxyaw(double maxyaw);

        /**
        * @brief save LRA COM correction 
        * @param[in]  x,y,z        LRA COM correction in satellite std::fixed reference system
        */
        void set_lra(double x, double y, double z);

        /**
        * @brief save LRA type
        * @param[in]  itype        type number of LRA
        */
        void set_lra_type(int itype);

        /**
        * @brief save satellite antenna transmit power(W).
        * @param[in] lpower
        */
        void set_lpower(double lpower);

        /**
        * @brief save satellite's frequency id from file.
        *
        * Only the GLONASS satellites need to be noticed
        *
        * @param[in]  fid        satellite's frequency id
        */
        void set_fid(int fid);

        /**
        * @brief save satellite's block type from file.
        * @param[in]  blocktype        satellite's block type
        */
        void set_blocktype(const std::string &blocktype);

        /// return satelllite information
        /**
        * @brief get satellite prn.
        * @return     satellite prn.
        */
        std::string prn();

        /**
        * @brief get satellite svn.
        * @return     satellite svn.
        */
        std::string svn();

        /**
        * @brief get satellite launched time.
        * @return     satellite launched time.
        */
        base_time start();

        /**
        * @brief get satellite decommissioned time.
        * @return     satellite decommissioned time.
        */
        base_time end();

        /**
        * @brief get satellite's cosparid.
        * @return     satellite's cosparid.
        */
        std::string cosparid();

        /**
        * @brief get satellite's power.
        * @return     satellite's power.
        */
        double power();

        /**
        * @brief get satellite's weight.
        * @return     satellite's weight.
        */
        double mass();

        /**
        * @brief get satellite's max yaw.
        * @return     satellite's max yaw.
        */
        double maxyaw();

        /**
        * @brief get satellite's LRA COM correction
        * @return     LRA COM correction
        */
        double *lra();

        /**
        * @brief get satellite's LRA COM correction
        * @return     LRA COM correction
        */
        int lratype();

        /**
        * @brief get satellite's frequency id.
        * @return     satellite's frequency id.
        */
        int fid();

        /**
        * @brief get satellite's block type.
        * @return     satellite's block type.
        */
        std::string blocktype();

    protected:
        std::string _prn;            ///< satellite prn
        std::string _svn;            ///< satelllite svn
        base_time _start;         ///< launch time of satellite
        base_time _end;           ///< retired time of satellite
        std::string _cosparid;       ///< cospar_id of satellite
        double _mass = 0.0;     ///< weight of satellite(unit: kg)
        double _maxyaw = 0.0;   ///< max_yaw of satellite(unit: degree)
        double _lra[3] = {0.0}; ///< LRA COM correction(unit:m)
        int _lratype = 0;       ///< LRA type: 0 for unknown, 1 for 4-prism LRA, 2 for 7-prism LRA, 3 for 9-prism LRA
        double _lpower = 0.0;   ///< Transmitting antenna power(unit: W)
        int _fid = 0;           ///< frequence id of satellite
        std::string _blocktype;      ///< block type
    };
}
#endif // !GSATPARAM_H
