
/**
* @file     rinexo2.h
* @brief    Purpose: Obs RINEX encoder/decoder
*    Todo:    header information completed
            encoders implementation
*/

#ifndef hwa_gnss_coder_rinexo2_H
#define hwa_gnss_coder_rinexo2_H

#include "hwa_set_base.h"
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_gnss_data_rnxhdr.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_coder_rinexo2 derive from base_coder
    */
    class gnss_coder_rinexo2 : public gnss_base_coder
    {
    public:
        /** @brief constructor set + version + sz. */
        gnss_coder_rinexo2(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief constructor beg + end + set + version + sz. */
        gnss_coder_rinexo2(base_time beg, base_time end, set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_rinexo2();

        /** @brief clear. */
        virtual void clear();

        /** @brief decode head. */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) = 0;

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) = 0;

        /** @brief encode head. */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) = 0;

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) = 0;

        /** @brief get epoch. */
        base_time get_epoch() { return _epoch; };

    protected:
        /** @brief decode head. */
        virtual int _decode_head();

        /** @brief decode data. */
        virtual int _decode_data();

        /** @brief read epoch & number of satellites, return flag. */
        virtual int _read_epoch();

        /** @brief read sat/sys-specific key to _mapobs (G,E,R,.. M, satellite). */
        virtual int _read_syskey(std::string &sat, std::string &key);

        /** @brief read satellite list (vector). */
        virtual int _read_satvec(std::vector<std::string> &satellites);

        /** @brief read single satellite observation types. */
        virtual int _read_obstypes(const std::string &sat, const std::string &sys);

        /** @brief fill single observation element. */
        virtual int _read_obs(const unsigned int &idx,
                              const gnss_data_rnxhdr::hwa_vector_obs::const_iterator &it,
                              hwa_spt_obsmanager obs);

        /** @brief fill header information. */
        virtual int _fill_head();

        /** @brief fill observation data structure. */
        virtual int _fill_data();

        /** @brief fill header information. */
        virtual int _check_head();

        /** @brief check band (extended RINEX 2.11). */
        virtual int _fix_band(std::string sys, std::string &go);

        /** @brief write log for null observations (sat, obstype). */
        virtual int _null_log(const std::string &sat, const std::string &obstype);

        /** @brief common stop reading used in local subroutines. */
        virtual int _stop_read();

        std::string _csys;             ///< A1 (G=GPS, R=GLO, E=GAL, S=SBAS, M=Mix), but may be also G11, ..
        base_time _epoch;           ///< working epoch
        std::string _line;             ///< working line read from
        std::string _site;             ///< working site
        int _tmpsize;             ///< working amount of bytes processed
        int _consume;             ///< working total amount of bytes (return)
        bool _complete;           ///< working flag for completed epoch decoding
        char _flag;               ///< working special event flag
        std::vector<hwa_spt_obsmanager> _vobs; ///< working GNSS observation vector
        int _nsat;                ///< working # of satellites
        int _count;               ///< working # of epochs

        base_time _epo_beg; ///< begin time
        base_time _epo_end; ///< end time

        int _xbeg; ///< # epochs filtered before BEG
        int _xend; ///< # epochs filtered after  END
        int _xsmp; ///< # epochs filtered by sampling
        int _xsys; ///< # obs data filtered by systems

        gnss_data_rnxhdr _rnxhdr;             ///< Rinex header instance
        std::vector<std::string> _comment;      ///< Rinex comments
        std::vector<char> _pcosat;         ///< GNSS (G/R/E/S/...) for antenna phase center vs. ARP
        std::vector<std::string> _pcosys;       ///< XYZ or NEU system for pco
        std::vector<Triple> _pcoecc;    ///< PCO-ARP values in system above (XYZ, NEU) [m]
        gnss_data_rnxhdr::hwa_map_obs _mapobs; ///< GNSS/sat (G/R/E/S/...) observation group
                                      // vector due to saving order in header !
                                      // including scale factor (1000,100,10,1)
    };

} // namespace

#endif
