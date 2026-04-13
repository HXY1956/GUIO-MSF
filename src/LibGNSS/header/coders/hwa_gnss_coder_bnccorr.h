#ifndef hwa_gnss_coder_bnccorr_H
#define hwa_gnss_coder_bnccorr_H

#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_all_prec.h"
#include "hwa_gnss_all_bias.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for BNC correction data storaging, derive from t_gcoder
    */
    class gnss_coder_bnccorr : public gnss_base_coder
    {

    public:
        /**
        *@brief       correction type
        */
        enum CORR_TYPE
        {
            ORBIT,      ///< orbit product
            CLOCK,      ///< clock product
            VTEC,       ///< VTEC
            CODE_bias,  ///< code bias
            PHASE_bias, ///< phase bias
            UPD_COR,    ///< UPD correction
            AUG_COR,    ///< AUG correction
            DEFAULT     ///< default
        };

        /**
        * @brief       interval
        */
        std::map<int, int> intv{{0, 1}, {1, 2}, {2, 5}, {3, 10}, {4, 15}, {5, 30}, {6, 60}, {7, 120}, {8, 240}, {9, 300}, {10, 600}, {11, 900}, {12, 1800}, {13, 3600}, {14, 7200}, {15, 10800}};

        /**
        * @brief constructor.
        *
        * @param[in]  s              gsetbase
        * @param[in]  version      version
        * @param[in]  sz          size
        *
        */
        gnss_coder_bnccorr(hwa_set::set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        ~gnss_coder_bnccorr(){};

        /**
        * @brief decode the head of the aug data file.
        *
        * @param[in]  buff        data flow
        * @param[in]  sz          size
        * @param[in]  errmsg      error message
        * @return      int          decode mode
        *
        */
        int decode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
        * @brief decode the data body of the aug data file.
        * @param[in]  buff        data flow
        * @param[in]  sz          size
        * @param[in]  cnt          todo
        * @param[in]  errmsg      error message
        * @return      int          decode mode
        *
        */
        int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

        /**
        * @brief judge the current time whether available.
        * @param[in]  now         the current time
        * @return      bool          whether the data available
        *
        */
        bool available(const base_time &now);

        /**
        * @brief set the available = false.
        */
        void available_false();

        /**
        * @brief set the available = ture.
        */
        void available_true();

        /**
        * @brief convert the correction type to string.
        */
        std::string corr2str(const CORR_TYPE &type);

        /**
        * @brief convert the string to correction type.
        */
        CORR_TYPE str2corr(const std::string &str);

    protected:
        /**
        * @brief decode the orbit product.
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_orbit(const base_time &epo, std::istringstream &ss);

        /**
        * @brief decode the clock product.
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_clock(const base_time &epo, std::istringstream &ss);

        /**
        * @brief decode code bias.
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_cb(const base_time &epo, std::istringstream &ss);

        /**
        * @brief decode phase bias.
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_pb(const base_time &epo, std::istringstream &ss);

        /**
        * @brief decode VTEC.
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_vtec(const base_time &epo, std::istringstream &ss);

        /**
        * @brief decode upd.
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_upd(const base_time &epo, std::istringstream &ss);

        /**
        * @brief decode aug.
        * @param[in]  site          site name
        * @param[in]  epo         the current time
        * @param[in]  ss          data flow
        * @return      int          decode mode
        *
        */
        int _decode_aug(const std::string &site, const base_time &epo, std::istringstream &ss);

    private:
        /**
        * @brief check reciver object.
        * @param[in]  site          site name
        * @return      void
        *
        */
        void _check_recobj(std::string &site);

        /**
        * @brief judge the current time whether valid.
        * @param[in]  t              the current time
        * @return      bool        true:valid, false:no-valid
        *
        */
        bool _validepo(const base_time &t);

        CORR_TYPE _type;         ///< the correction type
        bool _begepoch = false;  ///< the begin epoch
        bool _available = false; ///< whether available
        base_time _tt;
        unsigned int _udclkInt = 0; ///< SSR clk interval (added by zhShen)
        unsigned int _udorbInt = 0; ///< SSR orb interval (added by zhShen)
        unsigned int _udbiaInt = 0; ///< SSr bias interval

        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
    };

} // namespace

#endif
