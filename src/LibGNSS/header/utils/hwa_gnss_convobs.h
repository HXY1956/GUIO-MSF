/**
* @file            gconvobs.h
* @brief        Convert raw observations to carrier-range
*/

#ifndef hwa_gnss_base_convobs_h
#define hwa_gnss_base_convobs_h

#include "hwa_set_out.h"
#include "hwa_set_gen.h"
#include "hwa_base_file.h"
#include "hwa_base_fileconv.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_all_ambflag.h"
#include "hwa_gnss_coder_rinexo.h"
#include "hwa_set_gbase.h"

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
     * @brief class for Convert raw observations to carrier-range
     */
    class gnss_base_convobs
    {
    public:
        /**
         * @brief Construct a new t gconvobs object
         * @param[in]  settings  std::setbase control
         * @param[in]  spdlog      logbase control
         * @param[in]  data      all proc data
         */
        gnss_base_convobs(set_base *settings, base_log spdlog,  base_all_proc *data);
        /**
         * @brief Destroy the t gconvobs object
         */
        virtual ~gnss_base_convobs(){};
        /**
         * @brief batch process for Convert raw observations to carrier-range
         * @param[in]  site      station name
         * @param[in]  begT      begin time
         * @param[in]  endT      end time
         * @param[in]  sampling  sampling
         */
        bool ProcessBatch(std::string site, const base_time &begT, const base_time &endT, double sampling);

        /**
        * @brief  generate product for LEO DYN POD
        * @param[in] site station name
        * @param[in] begT begin time
        * @return
            @retval true    generate product correctly
            @retval false   generate product failed
        */
        bool GenerateProduct(std::string site, base_time begT);
        /**
         * @brief Set the Obj
         * @param[in]  obj       all object
         */
        void setbj(gnss_all_obj *obj) { _allobj = obj; };

    private:
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< index of band

         base_all_proc *_data = nullptr;            ///< all proc data
        gnss_all_obs *_allobs = nullptr;           ///< obs data
        gnss_all_ambflag *_allambflag = nullptr;   ///< ambflag data
        gnss_all_ambflag *_allambflag13 = nullptr; ///< ambflag13 data
        gnss_all_obj *_allobj = nullptr;           ///< all object
        set_base *_set = nullptr;             ///< std::setbase control
        base_log _spdlog;                       ///<logbase control
        std::set<std::string> _site_list;                 ///< station name list
    };

} // namespace

#endif
