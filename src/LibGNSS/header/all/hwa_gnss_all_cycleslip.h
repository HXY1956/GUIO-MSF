/**
* @file        gallcycleslip.h
* @brief    Storage the cycle slip data.(Maybe it was written by Wu Jiaqi)
*/

#ifndef hwa_all_CYCLESLIP_H
#define hwa_all_CYCLESLIP_H

#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_all_ambflag.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for all cycle slip data storaging
    */
    class gnss_all_cycleslip
    {
    public:
        /** @brief default constructor. */
        explicit gnss_all_cycleslip();

        /**
        * @brief add ambflag head of one station.
        * @param[in]  set        xml setting.
        * @param[in]  log        log file.
        * @return      void
        */
        explicit gnss_all_cycleslip(set_base *set, base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_cycleslip();

    public:
    protected:
        /**
        * @brief add ambflag head of one station.
        * @param[in]  freq_1            The first freq ID.
        * @param[in]  freq_2            The second freq ID.
        * @param[in]  _amb_info_files    Log file path.
        * @param[in]  _all_ambflag        Ambflog data class.
        * @return      void
        */
        void read_amb_info_files(const FREQ_SEQ &freq_1, const FREQ_SEQ &freq_2, const std::vector<std::string> &_amb_info_files, gnss_all_ambflag *_all_ambflag);

    protected:
        base_log _spdlog = nullptr;                                        ///< lof file
        set_base *_set = nullptr;                                        ///< set file
        std::map<std::pair<FREQ_SEQ, FREQ_SEQ>, SLIPMODEL> _slip_model;              ///< freq pair, such as FREQ_1 and FREQ_2 for slip12
        std::map<std::pair<FREQ_SEQ, FREQ_SEQ>, std::shared_ptr<gnss_data_cycleslip>> _amb_info; ///< amb  pair
        std::set<std::string> _rec_list;                                             ///< rec name list
        std::map<std::pair<FREQ_SEQ, FREQ_SEQ>, std::set<std::string>> _amb_info_files;        ///< amb file list
    };

}
#endif // !GALLCYCLESLIP_H
