/**
* @file            gallsum.h
* @brief            Storage the sum of residuals.
*/

#ifndef hwa_gnss_all_sum_H
#define hwa_gnss_all_sum_H

#include "hwa_gnss_proc_lsqmatrix.h"
#include "hwa_base_data.h"
#include "hwa_set_gproc.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for all LEO ICS data storaging
    */
    class gnss_all_sum : public base_data
    {
    public:
        /**
         * @brief Construct a new t gallsum object
         */
        explicit gnss_all_sum();
        explicit gnss_all_sum(base_log spdlog);
        /**
         * @brief Destroy the t gallsum object
         */
        virtual ~gnss_all_sum();
        /**
         * @brief Set the Sat list
         * @param[in]  satlist   satellite's names
         */
        void setSat(const std::vector<std::string> &satlist);
        /**
         * @brief Set the Site
         * @param[in]  sitelist  station's names
         */
        void setSite(const std::vector<std::string> &sitelist);
        /**
         * @brief set RMS
         * @param[in]  rms       rms map data
         */
        void setRMS(const std::map<gnss_data_obscombtype, std::map<int, std::map<int, double>>> &rms);
        /**
         * @brief set number of epoch
         * @param[in]  nepo      number of epoch
         */
        void setNEPO(const std::map<int, std::map<int, int>> &nepo);
        /**
         * @brief set SLR number of epoch
         * @param[in]  nepo      SLR number of epoch
         */
        void setNEPO_SLR(const std::map<int, std::map<int, int>> &nepo);
        /**
         * @brief set RMS of SLR
         * @param[in]  rms       rms of slr
         */
        void setRMS_SLR(const std::map<std::string, std::map<int, std::map<int, double>>> &rms);
        /**
         * @brief Get the Sat
         * @return vector<string> satellite's name
         */
        const std::vector<std::string> &getSat() const;
        /**
         * @brief Get the Site
         * @return vector<string> station's name 
         */
        const std::vector<std::string> &getSite() const;
        /**
         * @brief get all RMS
         * @return map<gnss_data_obscombtype, map<int, map<int, double>>>& all of the rms
         */
        const std::map<gnss_data_obscombtype, std::map<int, std::map<int, double>>> &getRMS() const;
        /**
         * @brief get number of epoch
         * @return map<int, map<int, int>>& all of the number of per epoch
         */
        const std::map<int, std::map<int, int>> &getNEPO() const;
        /**
         * @brief get SLR RMS
         * @return map<string, map<int, map<int, double>>>& rms of the slr
         */
        const std::map<std::string, std::map<int, std::map<int, double>>> &getRMS_SLR() const;
        /**
         * @brief get slr number of per epoch
         * @return map<int, map<int, int>>& slr of the number of per epoch
         */
        const std::map<int, std::map<int, int>> &getNEPO_SLR() const;

    protected:
        std::vector<std::string> _sat_list;                                  ///< satellite names
        std::vector<std::string> _site_list;                                 ///< station names;
        std::map<gnss_data_obscombtype, std::map<int, std::map<int, double>>> _rmstable; ///< all of the rms
        std::map<std::string, std::map<int, std::map<int, double>>> _rmstable_slr;     ///< rms of the slr
        std::map<int, std::map<int, int>> _nepotbl;                          ///< all number of per epoch
        std::map<int, std::map<int, int>> _nepotbl_slr;                      /// slr number of per epoch
    };
}

#endif