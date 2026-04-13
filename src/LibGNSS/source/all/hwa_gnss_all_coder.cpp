#include "hwa_gnss_all_coder.h"
#include "hwa_base_coder.h"
#include "hwa_gnss_coder_rinexo.h"
#include "hwa_gnss_coder_rinexc.h"
#include "hwa_gnss_coder_rinexn.h"
#include "hwa_gnss_coder_biasinex.h"
#include "hwa_gnss_coder_biabernese.h"
#include "hwa_gnss_coder_sp3.h"
#include "hwa_gnss_coder_atx.h"
#include "hwa_gnss_coder_blq.h"
#include "hwa_base_io.h"
#include "hwa_base_file.h"

namespace hwa_gnss
{
    gnss_all_coder::gnss_all_coder()
    {
    }

    gnss_all_coder::gnss_all_coder(base_log spdlog)
    {
        // set spdlog
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }

    hwa_gnss::gnss_all_coder::~gnss_all_coder()
    {
    }

    void gnss_all_coder::spdlog(base_log spdlog)
    {
        // set spdlog
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }

    void gnss_all_coder::set(set_base *set)
    {
        // set the setting pointer
        if (nullptr == set)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = set;
        }
    }

    IFMT gnss_all_coder::data2ifmt(base_data::ID_TYPE type)
    {
        switch (type)
        {
        case base_data::ID_TYPE::LAST:
        default:
            break;
        }
        return IFMT::UNDEF;
    };

    base_data::ID_TYPE gnss_all_coder::ifmt2data(IFMT ifmt)
    {
        switch (ifmt)
        {
        case IFMT::SP3_INP:
        case IFMT::SP3LEO_INP:
        case IFMT::RINEXC_INP:
        case IFMT::RINEXN_INP:
            return base_data::ID_TYPE::ALLNAV;
        case IFMT::ATX_INP:
            return base_data::ID_TYPE::ALLPCV;
        case IFMT::BLQ_INP:
            return base_data::ID_TYPE::ALLOTL;
        case IFMT::VLBIBLQ_INP:
            return base_data::ID_TYPE::ALLVLBIOTL;
        case IFMT::VLBIOPL_INP:
            return base_data::ID_TYPE::ALLOPL;
        case IFMT::OPL_INP:
            return base_data::ID_TYPE::ALLOPL;
        case IFMT::VLBIEOP_INP:
            return base_data::ID_TYPE::ALLPOLEUT1_VLBI;
        case IFMT::VLBIATL_INP:
            return base_data::ID_TYPE::ALLVLIBIATL;
        case IFMT::VLBIANTL_INP:
            return base_data::ID_TYPE::ALLANTL;
        case IFMT::OCEANTIDE_INP:
            return base_data::ID_TYPE::OCEANTIDE;
        case IFMT::RINEXO_INP:
            return base_data::ID_TYPE::ALLOBS;
        case IFMT::BIASINEX_INP:
        case IFMT::BIAS_INP:
            return base_data::ID_TYPE::ALLBIAS;
        case IFMT::IFCB_INP:
            return base_data::ID_TYPE::IFCB;
        case IFMT::ORBIT_INP:
            return base_data::ID_TYPE::ALLORB;
        case IFMT::GRAVITY_INP:
            return base_data::ID_TYPE::EGM;
        case IFMT::IOPNAV_INP:
            return base_data::ID_TYPE::ALLICS;
        case IFMT::EOP_INP:
            return base_data::ID_TYPE::ALLPOLEUT1;
        case IFMT::LEAPSECOND_INP:
            return base_data::ID_TYPE::LEAPSECOND;
        case IFMT::SATINFO_INP:
            return base_data::ID_TYPE::SATPARS;
        case IFMT::DE_INP:
            return base_data::ID_TYPE::ALLDE;
        case IFMT::AMBIGUITY_INP:
            return base_data::ID_TYPE::AMBCON;
        case IFMT::ION_INP:
            return base_data::ID_TYPE::ION;
        case IFMT::DESAISCOPOLECOEF_INP:
            return base_data::ID_TYPE::DESAISCOPOLECOEF;
        case IFMT::LEOATT_INP:
            return base_data::ID_TYPE::AllATTITUDE;
        case IFMT::IOPLEO_INP:
            return base_data::ID_TYPE::ALLICSLEO;
        case IFMT::PANNEL_INP:
            return base_data::ID_TYPE::ALLPANNEL;
        case IFMT::SOLAR_INP:
            return base_data::ID_TYPE::ALLSOLAR;
        case IFMT::ALBEDO_INP:
            return base_data::ID_TYPE::ALLALBEDO;
        case IFMT::RB_INP:
            return base_data::ID_TYPE::ALLRB;
        case IFMT::PREOBS_INP:
            return base_data::ID_TYPE::ALLPREOBS;
        case IFMT::LRA_INP:
            return base_data::ID_TYPE::ALLLRA;
        case IFMT::SLRF_INP:
            return base_data::ID_TYPE::SLRF;
        case IFMT::PCVNEQ_INP:
            return base_data::ID_TYPE::ALLPCVNEQ;
        case IFMT::NGS_INP:
            return base_data::ID_TYPE::ALLNGS;
        case IFMT::VLBISNX_INP:
            return base_data::ID_TYPE::VLBISTA;
        case IFMT::VLBIANT_INP:
            return base_data::ID_TYPE::VLBISTA;
        case IFMT::VLBIPSD_INP:
            return base_data::ID_TYPE::ALLPSD;
        case IFMT::VLBIECC_INP:
            return base_data::ID_TYPE::VLBISTA;
        case IFMT::VLBIVMF_INP:
            return base_data::ID_TYPE::VLBISTA;
        case IFMT::VLBISOU_INP:
            return base_data::ID_TYPE::ALLVLBISRC;
        case IFMT::VLBICLK_INP:
            return base_data::ID_TYPE::ALLVLBICLK;
        case IFMT::KBR_INP:
            return base_data::ID_TYPE::ALLKBR;
        case IFMT::LRI_INP:
            return base_data::ID_TYPE::ALLLRI;
        case IFMT::ACCELEROMETER_INP:
            return base_data::ID_TYPE::ALLACC;
        case IFMT::NPT_INP:
        case IFMT::NPT2_INP:
            return base_data::ID_TYPE::ALLNPT;
        case IFMT::SLRPSD_INP:
            return base_data::ID_TYPE::ALLPSD;
        case IFMT::GNSSPSD_INP:
            return base_data::ID_TYPE::ALLPSD;
        case IFMT::MANEUVER_INP:
            return base_data::ID_TYPE::ALLMANEUVER;
        case IFMT::ATMLOADING_INP:
            return base_data::ID_TYPE::ALLATL;
        case IFMT::SINEX_INP:
        default:
            if (_spdlog)
            {
                SPDLOG_LOGGER_CRITICAL(_spdlog, "do not support the fmt");
            }
            throw std::logic_error("do not support the fmt");
        }
    }

    void gnss_all_coder::destroy(std::map<base_data::ID_TYPE, base_data *> &map_gdata)
    {
        for (auto &iter : map_gdata)
        {
            if (iter.second)
            {
                delete iter.second;
                iter.second = nullptr;
            }
        }
    }
}
