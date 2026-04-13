#ifndef hwa_gnss_all_pcv_H
#define hwa_gnss_all_pcv_H

#include "hwa_base_time.h"
#include "hwa_base_const.h"
#include "hwa_base_common.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_data_pcv.h"

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_all_pcv derive from base_data
    */
    class gnss_all_pcv : public base_data
    {

    public:
        /** @brief default constructor. */
        gnss_all_pcv();

        /**
         * @brief Construct a new t gallpcv object
         * 
         * @param spdlog 
         */
        gnss_all_pcv(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_pcv();

        typedef std::map<base_time, std::shared_ptr<gnss_data_pcv>> hwa_map_tiv; ///< std::map of 1-ant/1-sn/N-epochs
        typedef std::map<std::string, hwa_map_tiv> hwa_map_num;           ///< std::map of 1-ant/N-sn/N-epochs
        typedef std::map<std::string, hwa_map_num> hwa_map_pcv;           ///< std::map of N-ant/N-sn/N-epochs

        /**
         *@brief find appropriate gnss_data_pcv element
         */
        std::shared_ptr<gnss_data_pcv> find(const std::string &ant, const std::string &ser,
                                const base_time &t);

        /**
        *@brief add single antenn pattern (PCV)
        */
        int addpcv(std::shared_ptr<gnss_data_pcv> pcv);

        /**
        *@brief get single antenn pattern (PCV)
        */
        std::shared_ptr<gnss_data_pcv> gpcv(const std::string &ant, const std::string &num,
                                const base_time &t);

        /**
        *@brief get std::vector of all antennas
        */
        std::vector<std::string> antennas();

        /**
        *@brief std::set/get overwrite mode
        */
        void overwrite(bool b) { _overwrite = b; }

        /**
         * @brief 
         * 
         * @return true 
         * @return false 
         */
        bool overwrite() { return _overwrite; }

        /**
        *@brief get mappcv
        */
        hwa_map_pcv mappcv() { return _mappcv; }

    protected:
        /**
        *@brief find appropriate gnss_data_pcv element
        */
        virtual std::shared_ptr<gnss_data_pcv>
        _find(const std::string &ant, const std::string &ser, const base_time &t);

    private:
        hwa_map_pcv _mappcv; ///< complete PCV-std::map
        bool _overwrite;   ///< rewrite/add only mode

        std::shared_ptr<gnss_data_pcv> _pcvnull; ///< pcvnull
    };

} // namespace

#endif
