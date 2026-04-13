#include "hwa_set_ppp.h"

using namespace std;
using namespace pugi;

namespace hwa_set
{
    set_ppp::set_ppp() : set_amb(),
        set_gnss(),
        set_gen(),
        set_flt(),
        set_gproc(),
        set_inp(),
        set_npp(),
        set_rec(),
        set_turboedit(),
        set_out(),
        set_par()
    {
        _IFMT_supported.insert(IFMT::RINEXO_INP);
        _IFMT_supported.insert(IFMT::RINEXC_INP);
        _IFMT_supported.insert(IFMT::RINEXN_INP);
        _IFMT_supported.insert(IFMT::BNCOBS_INP);
        _IFMT_supported.insert(IFMT::BNCCORR_INP);
        _IFMT_supported.insert(IFMT::BNCBRDC_INP);
        _IFMT_supported.insert(IFMT::ATX_INP);
        _IFMT_supported.insert(IFMT::BLQ_INP);
        _IFMT_supported.insert(IFMT::SP3_INP);
        _IFMT_supported.insert(IFMT::BIAS_INP);
        _IFMT_supported.insert(IFMT::SINEX_INP);
        _IFMT_supported.insert(IFMT::BIASINEX_INP);
        _IFMT_supported.insert(IFMT::UPD_INP);

        _IFMT_supported.insert(IFMT::EOP_INP);
        _IFMT_supported.insert(IFMT::LEAPSECOND_INP);
        _IFMT_supported.insert(IFMT::DE_INP);
        _IFMT_supported.insert(IFMT::BIASINEX_INP);
        _IFMT_supported.insert(IFMT::IFCB_INP);
        _IFMT_supported.insert(IFMT::AUG_INP);
        _IFMT_supported.insert(IFMT::IONEX_INP);

        _OFMT_supported.insert(LOG_OUT);
        _OFMT_supported.insert(PPP_OUT);
        _OFMT_supported.insert(FLT_OUT);
        _OFMT_supported.insert(KML_OUT);
        _OFMT_supported.insert(ENU_OUT);
        _OFMT_supported.insert(AUG_OUT);
        _OFMT_supported.insert(RATIO_OUT);
        _OFMT_supported.insert(RECOVER_OUT);

        //LVHB ADDED IN 20210321
        _OFMT_supported.insert(GPGGA_OUT);
    }

    set_ppp::~set_ppp()
    {
    }

    // settings check
    // ----------
    void set_ppp::check()
    {
        set_amb::check();
        set_gnss::check();
        set_gen::check();
        set_flt::check();
        set_gproc::check();
        set_inp::check();
        set_npp::check();
        set_rec::check();
        set_turboedit::check();
        set_out::check();
        set_par::check();
    }

    // settings help
    // ----------
    void set_ppp::help()
    {
        set_base::help_header();
        set_amb::help();
        set_gnss::help();
        set_gen::help();
        set_flt::help();
        set_gproc::help();
        set_inp::help();
        set_npp::help();
        set_rec::help();
        set_turboedit::help();
        set_out::help();
        set_par::help();
        set_base::help_footer();
    }

} // namespace
