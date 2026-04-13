#include <cmath>
#include "hwa_gnss_prod_Clk.h"

namespace hwa_gnss
{
    gnss_prod_clk::gnss_prod_clk(const base_time &t, std::shared_ptr<gnss_data_obj> pt)
        : gnss_prod(t, pt),
          _clk(0.0),
          _clk_rms(0.0)
    {
        id_type(CLK);
        id_group(GRP_prodUCT);
    }
    gnss_prod_clk::gnss_prod_clk(base_log spdlog, const base_time &t, std::shared_ptr<gnss_data_obj> pt)
        : gnss_prod(spdlog, t, pt),
          _clk(0.0),
          _clk_rms(0.0)
    {

        id_type(CLK);
        id_group(GRP_prodUCT);
    }

    // destructor
    // --------------
    gnss_prod_clk::~gnss_prod_clk()
    {
    }

    // add clk
    // --------------------------
    void gnss_prod_clk::clk(const double &val, const double &rms)
    {
        _clk = val;
        _clk_rms = rms;
        return;
    }

    // get clk
    // --------------------------
    double gnss_prod_clk::clk()
    {
        return _clk;
    }

    // get clk rms
    // -------------------------
    double gnss_prod_clk::clk_rms()
    {
        return _clk_rms;
    }

    // add ICB
    // --------------------------
    void gnss_prod_clk::icb(const double &val, const double &rms)
    {
        _icb = val;
        _icb_rms = rms;
        return;
    }

    // get ICB
    // --------------------------
    double gnss_prod_clk::icb()
    {
        return _icb;
    }

    // get ICB rms
    // -------------------------
    double gnss_prod_clk::icb_rms()
    {
        return _icb_rms;
    }

} // namespace