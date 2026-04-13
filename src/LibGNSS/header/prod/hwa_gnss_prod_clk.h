
/**

* @file        gprodclk.h
* @brief    clk class
*/

#ifndef hwa_prodCLK_H
#define hwa_prodCLK_H

#include <iostream>
#include "hwa_gnss_Prod.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for gnss_prod_clk derive from gnss_prod. */
    class gnss_prod_clk : public gnss_prod
    {

    public:
        /** @brief constructor 1. */
        gnss_prod_clk(const base_time &t, std::shared_ptr<gnss_data_obj> pt = nullobj);

        gnss_prod_clk(base_log spdlog, const base_time &t, std::shared_ptr<gnss_data_obj> pt = nullobj);
        /** @brief default destructor. */
        virtual ~gnss_prod_clk();

        /** @brief add clk. */
        void clk(const double &val, const double &rms = 0.0);

        /** @brief get clk. */
        double clk();

        /** @brief get clk rms. */
        double clk_rms();

        /** @brief add ICB. */
        void icb(const double &val, const double &rms = 0.0);

        /** @brief get ICB. */
        double icb();

        /** @brief get ICB rms. */
        double icb_rms();

    protected:
        double _clk;     ///< clk
        double _clk_rms; ///< clk rms
        double _icb;     ///< ICB
        double _icb_rms; ///< ICB rms

    private:
    };

} // namespace

#endif
