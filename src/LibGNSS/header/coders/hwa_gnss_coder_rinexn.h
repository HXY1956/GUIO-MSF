
/**
*
* @file     rinexn.h
* @brief    Purpose: brdm RINEX encoder/decoder
*
*/

#ifndef hwa_gnss_coder_rinexn_H
#define hwa_gnss_coder_rinexn_H

#define ID_allSAT "ALL_SATELLITES"

#include <vector>
#include <sstream>

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_rxnhdr.h"
#include "hwa_set_inp.h"
#include "hwa_gnss_data_nav.h"
#include "hwa_gnss_data_navglo.h"
#include "hwa_gnss_data_navgal.h"
#include "hwa_gnss_data_navgps.h"
#include "hwa_gnss_data_navbds.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_coder_rinexn derive from base_coder
    */
    class gnss_coder_rinexn : public gnss_base_coder
    {

    public:
        /** @brief constructor set + version + sz. */
        gnss_coder_rinexn(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_rinexn(){};

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

        /** @brief set/get gnss system. */
        void gnsssys(char s) { _gnsssys = s; }

        /**
         * @brief 
         * 
         * @return char 
         */
        char gnsssys() { return _gnsssys; }

        /** @brief get rinex header all. */
        void rxnhdr_all(bool b) { _rxnhdr_all = b; }

    protected:
        /** @brief fill header information. */
        virtual int _fill_head();

        /** @brief filter out navigation mess. types. */
        bool _filter_gnav(std::shared_ptr<gnss_data_nav> geph, const std::string &prn);

        /** @brief consolidate header records. */
        gnss_data_rxnhdr _consolidate_header();

    private:
        char _gnsssys;           ///< gnss system
        bool _rxnhdr_all;        ///< use all RINEX headers in encoder
        base_time _check_dt;       ///< to validate the message
        gnss_data_rxnhdr _rxnhdr;        ///< RINEX header
        std::vector<std::string> _comment; ///< RINEX comments
    };

} // namespace

#endif
