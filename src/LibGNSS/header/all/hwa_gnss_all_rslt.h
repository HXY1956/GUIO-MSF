/**
*
* @file     gallrslt.h
* @brief    container for all observation
*/

#ifndef hwa_gnss_all_rslt_H
#define hwa_gnss_all_rslt_H

#include <vector>
#include <set>
#include <iostream>
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_all_rslt
    */
    class gnss_all_rslt
    {
    public:
        /** @brief default constructor. */
        gnss_all_rslt();

        /** @brief struct result. */
        struct result
        {
            std::string type; ///< type
            std::string prn;  ///< prn

            base_time beg; ///< begin time
            base_time end; ///< end time

            int index; ///< index

            double adj; ///< adj
            double rms; ///< rms
            double val; ///< val

            /** @brief override operator <. */
            bool operator<(const gnss_all_rslt::result &) const;
        };

        std::vector<gnss_all_rslt::result> v_rslt; ///< result vector
        std::set<gnss_all_rslt::result> set_crd;   ///< crd set
        std::set<gnss_all_rslt::result> set_trp;   ///< trop set
        std::set<gnss_all_rslt::result> set_clk;   ///< clk set
        std::set<gnss_all_rslt::result> set_amb;   ///< amb set

        /** @brief append result structure. */
        void append(const gnss_all_rslt::result &rslt);

        /**
         * @brief 
         * 
         * @param type 
         * @param beg 
         * @param end 
         * @param idx 
         * @param prn 
         * @param adj 
         * @param rms 
         * @param val 
         */
        void append(const std::string &type, const base_time &beg, const base_time &end, int idx,
                    const std::string &prn, double adj, double rms, double val);
    };

} // namespace

#endif // GALLRSLT_H
