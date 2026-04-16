
/**
*
* @file     rinexo.h
* @brief    Purpose: Obs RINEX encoder/decoder
*    Todo:    header information completed
            gobj getting information from rinexo header
            encoders implementation
            to finish obs-filtering via settings
*/

#ifndef hwa_gnss_coder_rinexo_h
#define hwa_gnss_coder_rinexo_h

#include "hwa_set_base.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_base_time.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_data_obs.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_coder_base.h"
#include "hwa_gnss_coder_rinexo2.h"
#include "hwa_gnss_coder_rinexo3.h"
#include "hwa_gnss_data_rec.h"
#include "hwa_gnss_data_rnxhdr.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_coder_rinexo derive from gnss_coder_rinexo3
    */
    class gnss_coder_rinexo : public gnss_coder_rinexo3
    {

    public:
        /** @brief constructor set + version + sz. */
        explicit gnss_coder_rinexo(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief constructor beg + end + set + version + sz. */
        gnss_coder_rinexo(base_time beg, base_time end, set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_rinexo(){};

        /** @brief decode head. */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

        /** @brief encode head. */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

        /** @brief set site. */
        void setSite(const std::string &site) { _encode_site = site; };

    protected:
        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_head();

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_data();

    private:
        std::string _encode_site; ///< encode site
    };

} // namespace

#endif
