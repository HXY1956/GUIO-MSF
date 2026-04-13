
/**
*
* @file     sinex.h
* @brief    Purpose: SINEX
*    Todo:    header information completed
            encoders implementation
*/

#ifndef SINEX_H
#define SINEX_H

#include "hwa_set_base.h"
#include "hwa_base_time.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_base_pair.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_all_Prod.h"
#include "hwa_gnss_Model_Gpt.h"
#include "hwa_set_rec.h"
#include "hwa_gnss_data_rec.h"

using namespace hwa_set;

namespace hwa_gnss
{

    enum SINEX_TYPE
    {
        SINEX_GNS,
        TROSNX_GNS,
        TROSNX_NWM,
        TROSNX_OTH
    };

    /**
    *@brief Class for gnss_coder_sinex derive from base_coder
    */
    class gnss_coder_sinex : public gnss_base_coder
    {

    public:
        /** @brief constructor set + version + sz + id. */
        gnss_coder_sinex(set_base *s, std::string version, int sz = DEFAULT_BUFFER_SIZE, std::string id = "sinex");

        /** @brief default destructor. */
        virtual ~gnss_coder_sinex(){};

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

        /** @brief technique. */
        virtual void technique(char c);

        /**
         * @brief 
         * 
         * @return char 
         */
        virtual char technique();

    protected:
        /** @brief struct gnss_STC_META. */
        struct gnss_STC_META
        {
            Triple ell, xyz, ecc, apr, std, rms, var, cov, idx;
            Triple gps_neu_L1, gps_neu_L2;
            std::vector<std::string> par;
            std::string id, name, domes, desc, ant, rec, snx_code;
            base_time begPOS, endPOS;
        };

        /**
         * @brief 
         * 
         * @param site 
         * @return base_time 
         */
        virtual base_time _site_beg(std::string site); ///< begin site

        /**
         * @brief 
         * 
         * @param site 
         * @return base_time 
         */
        virtual base_time _site_end(std::string site); ///< end site

        /**
         * @brief 
         * 
         * @param site 
         * @return shared_ptr<set_rec> 
         */
        virtual std::shared_ptr<gnss_data_rec> _get_rec(std::string site); ///< get reciver

        /**
         * @brief 
         * 
         * @param id 
         * @param pt_data 
         */
        virtual void _add_data(std::string id, base_data *pt_data); ///< add data

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _initialize_data(); ///< init data

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_vers();

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_data();

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_comm();

        /**
         * @brief 
         * 
         * @return int 
         */
        virtual int _decode_block(); ///< decode

        /** @brief prefill metadata. */
        virtual int _fill_site_INI(std::string site, gnss_STC_META &meta);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_IDE(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_SOL(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_XYZ(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_ECC(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_ANT(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_REC(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_PCO(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_EST(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_APR(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param site 
         * @param meta 
         * @param os 
         */
        virtual void _fill_site_COV(std::string site, gnss_STC_META &meta, std::ostringstream &os);

        /**
         * @brief 
         * 
         */
        virtual void _fill_head_INI();

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_IDE(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_SOL(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_XYZ(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_ECC(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_ANT(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_REC(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_head_PCO(std::ostringstream &os);

        /**
         * @brief 
         * 
         */
        virtual void _fill_data_STT();

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_data_EST(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_data_APR(std::ostringstream &os);

        /**
         * @brief 
         * 
         * @param os 
         */
        virtual void _fill_data_COV(std::ostringstream &os);

        /** @brief complete metadata to object from _allobj. */
        virtual void _complete_obj(std::shared_ptr<gnss_data_rec> obj, const base_time &epo);

        SINEX_TYPE _snx_type;    ///< TRO-SINEX TYPE
        char _technique;         ///< TECHNIQUE TYPE
        int _parindex;           ///< PARAMETER INDEX
        int _tmpsize;            ///< working amount of bytes processed
        int _consume;            ///< working total amount of bytes (return)
        bool _complete;          ///< working flag for completed epoch decoding
        bool _estimation;        ///< estimated parameters
        std::string _code_label;      ///< site code label (CODE:4 vs STATION__:9)
        std::string _list_gnss;       ///< list of GNSSs (GREC)
        std::string _line;            ///< working line read from
        std::string _site;            ///< cache
        std::string _block;           ///< working block name
        std::string _pco_mod;         ///< PCO model
        std::string _ac;              ///< analyses center abbr
        std::vector<std::string> _comment; ///< std::vector of comments

        gnss_model_gpt _ggpt;          ///< gnss_model_gpt
        gnss_all_prod *_pt_prod; ///< all prod

        base_time _file_beg; ///< file first epoch
        base_time _file_end; ///< file last epoch
        base_time _file_run; ///< file created

        gnss_all_obj *_allobj; ///< all obj
        std::map<std::string, std::shared_ptr<gnss_data_obj>> _mapobj;
        std::map<std::string, std::shared_ptr<gnss_data_obj>>::const_iterator itOBJ;

        std::map<std::string, std::set<base_time>> _epo_pos;  ///< site/product epochs (POS)
        std::map<std::string, std::pair<int, int>> _mapidx; ///< map index
        std::set<std::string> _set_types;              ///< set type
        std::set<std::string> _sites;                  ///< sites
        std::set<std::string>::const_iterator itSET;   ///< itset

        // Decoding estimated coordinates
        std::map<std::string, std::set<base_time>> _sol_epoch;                       ///< For decoding SOLUTION/EPOCH
        bool _end_epoch;                                            ///< The block completed
        std::map<std::string, std::map<base_time, std::map<std::string, base_pair>>> _sol_estim; ///< For decoding SOLUTION/ESTIMATE
        bool _end_estim;                                            ///< The block completed
        void _set_rec_crd();                                        ///< set rec crd
    };

} // namespace

#endif
