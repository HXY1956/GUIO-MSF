
/**
* @file     rinexc.h
* @brief    Purpose: Clock RINEX encoder/decoder
*/

#ifndef hwa_gnss_coder_rinexc_H
#define hwa_gnss_coder_rinexc_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_base_time.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_all_prec.h"

#define RINEXC_BUFFER_LEN 81

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_coder_rinexc derive from base_coder
    */
    class gnss_coder_rinexc : public gnss_base_coder
    {

    public:
        /** @brief constructor set + version + sz. */
        explicit gnss_coder_rinexc(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_rinexc(){};

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

        /**
        * @brief encode header of  clkfile
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of header encoding
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode data of  clkfile
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of data body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /** @brief set/get gnss system. */
        void gnsssys(char s) { _gnsssys = s; }
        char gnsssys() { return _gnsssys; }

    protected:
        /** @brief meta struct. */
        struct gnss_STC_META
        {
            Triple xyz;  ///< position
            std::string name;    ///< name
            std::string domes;   ///< domes
            std::string desc;    ///< desc
            std::string ant;     ///< ant
            std::string rec;     ///< reciver
            base_time begCLK; ///< begin time
            base_time endCLK; ///< end time
        };

        base_time _file_beg; ///< file first epoch
        base_time _file_end; ///< file last epoch
        base_time _file_run; ///< file created

        gnss_all_obj *_allobj;                                    ///< all object
        std::map<std::string, std::shared_ptr<gnss_data_obj>> _mapsat;               ///< map satellite
        std::map<std::string, std::shared_ptr<gnss_data_obj>> _maprec;               ///< map rec
        std::map<std::string, std::shared_ptr<gnss_data_obj>>::const_iterator itOBJ; ///< itOBJ

        std::set<std::string> _sites;      ///< site list
        std::set<std::string> _satellites; ///< satellite list

    private:
        char _gnsssys; ///< gnss system
        /** @brief get galllprec data from data. */
        gnss_all_prec *_get_prec_data();
    };

} // namespace

#endif
