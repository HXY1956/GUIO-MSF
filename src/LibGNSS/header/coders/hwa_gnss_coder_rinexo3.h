
/**
*
* @file     rinexo3.h
* @brief    Purpose: Obs RINEX encoder/decoder
*    Todo:    header information completed
            encoders implementation
*/

#ifndef hwa_gnss_coder_rinexo3_H
#define hwa_gnss_coder_rinexo3_H

#include "hwa_set_base.h"
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_gnss_coder_rinexo2.h"
#include "hwa_gnss_data_rnxhdr.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_coder_rinexo3 derive from gnss_coder_rinexo2
    */
    class gnss_coder_rinexo3 : public gnss_coder_rinexo2
    {
    public:
        /** @brief constructor set + version + sz. */
        gnss_coder_rinexo3(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief constructor beg + end + set + version + sz. */
        gnss_coder_rinexo3(base_time beg, base_time end, set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_rinexo3(){};

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

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_data();

        /** @brief fill header information. */
        virtual int _check_head();

        /** @brief read epoch & number of satellites, return flag. */
        virtual int _read_epoch();

        /** @brief read single satellite observation types. */
        virtual int _read_obstypes(const std::string &sat, const std::string &sys);

        /** @brief fix band (BDS). */
        virtual int _fix_band(std::string sys, std::string &go);

        gnss_data_rnxhdr::hwa_map_obs _mapcyc;  ///< map of GOBS phase base_quater-cycle shifts
        gnss_data_rnxhdr::hwa_map_obs _glofrq;  ///< map of GLONASS slot/frequency
        gnss_data_rnxhdr::hwa_vector_obs _globia; ///< vec of GLONASS obs code-phase biases

    private:
    };

} // namespace

#endif
