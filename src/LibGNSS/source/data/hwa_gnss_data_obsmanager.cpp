#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "hwa_gnss_data_obsmanager.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_all_bias.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_obs_manager::gnss_data_obs_manager()
        : base_data(),
          _apr_ele(-1),
          _channel(DEF_CHANNEL),
          _rtcm_end(2),
          _health(true),
          _dcb_correct_mark(false),
          _bds_code_bias_mark(false),
          _range_smooth_mark(false)
    {
        id_type(base_data::OBSGNSS);
        id_group(base_data::GRP_obsERV);
    }

    gnss_data_obs_manager::gnss_data_obs_manager(base_log spdlog)
        : base_data(spdlog),
          _apr_ele(-1),
          _channel(DEF_CHANNEL),
          _rtcm_end(2),
          _health(true),
          _dcb_correct_mark(false),
          _bds_code_bias_mark(false),
          _range_smooth_mark(false)
    {
        id_type(base_data::OBSGNSS);
        id_group(base_data::GRP_obsERV);
    }
    gnss_data_obs_manager::gnss_data_obs_manager(base_log spdlog, const std::string &sat)
        : base_data(spdlog),
          _apr_ele(-1),
          _channel(DEF_CHANNEL),
          _rtcm_end(2),
          _health(true),
          _dcb_correct_mark(false),
          _bds_code_bias_mark(false),
          _range_smooth_mark(false),
          _satid(sat),
          _gsys(gnss_sys::char2gsys(sat[0]))
    {
        id_type(base_data::OBSGNSS);
        id_group(base_data::GRP_obsERV);
    }

    gnss_data_obs_manager::gnss_data_obs_manager(base_log spdlog,
                           const std::string &site,
                           const std::string &sat,
                           const base_time &t)
        : base_data(spdlog),
          _staid(site),
          _satid(sat),
          _gsys(gnss_sys::char2gsys(_satid[0])),
          _epoch(t),
          _apr_ele(-1),
          _channel(DEF_CHANNEL),
          _rtcm_end(2),
          _health(true),
          _dcb_correct_mark(false),
          _bds_code_bias_mark(false),
          _range_smooth_mark(false)
    {
        id_type(base_data::OBSGNSS);
        id_group(base_data::GRP_obsERV);
    }

    // is_leo must be true
    // add leo
    gnss_data_obs_manager::gnss_data_obs_manager(base_log spdlog,
                           const std::string &site,
                           const std::string &sat,
                           const base_time &t,
                           const bool &is_leo)
        : base_data(spdlog),
          _staid(site),
          _satid(sat),
          _gsys(gnss_sys::char2gsys_addleo(_satid[0])),
          _epoch(t),
          _apr_ele(-1),
          _channel(DEF_CHANNEL),
          _rtcm_end(2),
          _health(true),
          _dcb_correct_mark(false),
          _bds_code_bias_mark(false),
          _range_smooth_mark(false)
    {
        id_type(base_data::OBSGNSS);
        id_group(base_data::GRP_obsERV);
    }

    // destructor
    // -----------
    gnss_data_obs_manager::~gnss_data_obs_manager()
    {
#ifdef DEBUG
        std::cout << "OBS-destruct: " << site() << " " << sat() << " "
             << epoch().str("  %Y-%m-%d %H:%M:%S[%T] ") << std::fixed << std::setprecision(3);

        std::vector<GOBS> v_obs = this->getobs();
        std::vector<GOBS>::iterator itOBS = v_obs.begin();
        for (; itOBS != v_obs.end(); ++itOBS)
            std::cout << " " << gobs2str(*itOBS) << ":" << this->getobs(*itOBS);
        std::cout << std::endl;
        std::cout.flush();
#endif
    }

    // add observation - special std::map of doubles
    // -----------
    void gnss_data_obs_manager::addobs(const GOBS &obs, const double &d)
    {
        //   std::cout << "addobs: " << sat() << " " << gobs2str(obs) << " " << d << std::endl;
        _gobs[obs] = d;
        return;
    }

    // add lost-of-lock indicators (LLI) - special std::map of integers
    // -----------
    void gnss_data_obs_manager::addlli(const GOBS &obs, const int &i)
    {
        if (i != 0)
            _glli[obs] = i;
        return;
    }

    // add an estimated cycle slip
    // -----------
    void gnss_data_obs_manager::addslip(const GOBS &obs, const int &i)
    {
        _gslip[obs] = i;
        return;
    }

    // add range outliers
    // -----------
    void gnss_data_obs_manager::addoutliers(const GOBS &obs, const int &i)
    {
        if (i != 0)
            _goutlier[obs] = i;
        return;
    }

    void gnss_data_obs_manager::setrangestate(const std::string &name, const bool &b)
    {
        if (name == "bds_code_bias")
            _bds_code_bias_mark = b;
        else if (name == "smooth_range")
            _range_smooth_mark = b;
        return;
    }

    bool gnss_data_obs_manager::getrangestate(const std::string &name)
    {
        bool tmp = false;
        if (name == "bds_code_bias")
            tmp = _bds_code_bias_mark;
        else if (name == "smooth_range")
            tmp = _range_smooth_mark;
        return tmp;
    }

    void gnss_data_obs_manager::setobsLevelFlag(const GOBS &gobs, const int &flag)
    {
        _gLevel[gobs] = flag;
        return;
    }

    // get std::vector of observations types (GOBS)
    // -----------
    std::vector<GOBS> gnss_data_obs_manager::obs() const
    {
        std::vector<GOBS> tmp;
        std::map<GOBS, double>::const_iterator it = _gobs.begin();
        while (it != _gobs.end())
        {
            tmp.push_back(it->first);
            it++;
        }
        return tmp;
    }

    // get std::vector of observations types (GOBS) for a badn
    // -----------
    std::set<GOBS> gnss_data_obs_manager::obs_phase(const int &band) const
    {
        std::set<GOBS> tmp;
        std::map<GOBS, double>::const_iterator it = _gobs.begin();
        while (it != _gobs.end())
        {
            GOBS gobs = it->first;
            int b = gobs2band(gobs);
            if (band == b && gobs_phase(gobs))
                tmp.insert(gobs);
            it++;
        }
        return tmp;
    }

    // add OBS - code [m], phase [whole cycles], dopler[cycles/sec], snr [DBHZ], ...  observations (double)
    // -----------
    double gnss_data_obs_manager::getobs(const GOBS &obs) const
    {
        double tmp = NULL_GOBS;
        if (_gobs.find(obs) != _gobs.end())
            tmp = _gobs.at(obs);
        return tmp;
    }

    // reset OBS - code [m], phase [whole cycles], dopler[cycles/sec], snr [DBHZ], ...  observations (double)
    // -----------
    void gnss_data_obs_manager::resetobs(const GOBS &obs, const double &v)
    {
        if (_gobs.find(obs) != _gobs.end())
            _gobs[obs] = v;
    }

    // erase OBS - code [m], phase [whole cycles], dopler[cycles/sec], snr [DBHZ], ...  observations (double)
    // -----------
    void gnss_data_obs_manager::eraseobs(const GOBS &obs)
    {
        if (_gobs.find(obs) != _gobs.end())
            _gobs.erase(obs);
        if (_glli.find(obs) != _glli.end())
            _glli.erase(obs);
        if (_goutlier.find(obs) != _goutlier.end())
            _goutlier.erase(obs);
        if (_gslip.find(obs) != _gslip.end())
            _gslip.erase(obs);
        if (_gLevel.find(obs) != _gLevel.end())
            _gLevel.erase(obs);
    }

    void gnss_data_obs_manager::eraseband(const GOBSBAND &b)
    {
        //

        std::vector<GOBS> obs_vec = this->obs();
        for (auto it : obs_vec) // delete
        {
            if (str2gobsband(gobs2str(it)) == b)
                this->eraseobs(it);
        }

        //
    }

    bool gnss_data_obs_manager::apply_bias(gnss_all_bias *allbias)
    {
         if (nullptr == allbias)
         {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "{} : NO OSB FILE", this->site());
            return true;
         }

        if (allbias->get_used_ac() == "SGG_A")
            return apply_code_phase_bias(allbias); // Temporarily reserved
        else if (allbias->is_osb())
            return apply_osb(allbias);
        else
            return apply_dcb(allbias);
    }

    bool gnss_data_obs_manager::apply_osb(gnss_all_bias* allbias)
    {
         if (allbias == nullptr)
         {
             if (_spdlog)
                 SPDLOG_LOGGER_WARN(_spdlog, "{} : NO OSB FILE", this->site());
             return true;
         }

        // skip already correct dcb obsdata
        if (this->_dcb_correct_mark)
            return true;

        std::string gsat = this->sat();
        base_time gepo = this->epoch();
        GSYS gsys = this->gsys();

        for (GOBS obs_type : this->obs())
        {
            gnss_data_obs gobs_type(obs_type);
            gnss_data_obs gobs_type2(obs_type);
            gobs_type.gobs2to3(gsys);

            double bias = 0.0;
            if (gnss_data_obs(obs_type).is_code())
            {
                bias = allbias->get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs());
            }
            else if (gnss_data_obs(obs_type).is_phase())
            {
                bias = allbias->get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs()) / wavelength(gnss_data_obs(obs_type).band()); // units from meter to cycle
            }
            else
            {
                continue;
            }
            // apply osb to obsdata
            double obs_value = this->getobs(gobs_type2.gobs());
            obs_value -= bias;
            this->resetobs(gobs_type2.gobs(), obs_value);
        }
        this->_dcb_correct_mark = true;
        return true;
    }

    bool gnss_data_obs_manager::apply_code_phase_bias(gnss_all_bias *allbias)
    {
        double bias1 = 0.0;
        double bias2 = 0.0;
        double c1 = 0.0;
        double c2 = 0.0;
        double biasif = 0.0;
        GOBSBAND b1, b2;
        // skip already correct dcb obsdata
        if (this->_dcb_correct_mark)
        {
            return true;
        }

        std::string gsat = this->sat();
        base_time gepo = this->epoch();
        GSYS gsys = this->gsys();

        for (GOBS obs_type : this->obs())
        {

            gnss_data_obs gobs_type(obs_type);
            gobs_type.gobs2to3(gsys);

            // double bias = 0.0;
            if (gnss_data_obs(obs_type).is_code())
            {
                // bias = allbias.get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU");
            }
            else if (gnss_data_obs(obs_type).is_phase())
            {
                // bias = -1 * allbias.get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU_PHASE") / wavelength(gnss_data_obs(obs_type).band());
                if (obs_type == L1W)
                {
                    // bias += allbias.get(gepo, gsat, C1W, C1W, "WHU") / wavelength(gnss_data_obs(obs_type).band());
                    bias1 = allbias->get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU_PHASE");
                    b1 = gnss_data_obs(obs_type).band();
                }
                else if (obs_type == L2W)
                {
                    //      bias += allbias.get(gepo, gsat, C2W, C2W, "WHU") / wavelength(gnss_data_obs(obs_type).band());
                    bias2 = allbias->get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU_PHASE");
                    b2 = gnss_data_obs(obs_type).band();
                }
            }
            else
            {
                continue;
            }

            // apply dcb to obsdata
            double obs_value = this->getobs(gobs_type.gobs());
            if (obs_type == L2W)
            {
                //
                coef_ionofree(b1, c1, b2, c2);
                biasif = c1 * bias1 + c2 * bias2;
                std::cout << gsat << " " << gnss_data_obs(obs_type).gobs() << " " << biasif << std::endl;
                // printf("");
            }
            //        obs_value -= bias;
            this->resetobs(gobs_type.gobs(), obs_value);
        }
        this->_dcb_correct_mark = true;
        return true;
    }

    bool gnss_data_obs_manager::apply_bias_tmp(gnss_all_bias &allbias)
    {
        std::string gsat = this->sat();
        base_time gepo = this->epoch();
        GSYS gsys = this->gsys();

        for (GOBS obs_type : this->obs())
        {

            gnss_data_obs gobs_type(obs_type);
            gobs_type.gobs2to3(gsys);

            double bias = 0.0;
            if (gnss_data_obs(obs_type).is_code())
            {
                bias = allbias.get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU");
            }
            else if (gnss_data_obs(obs_type).is_phase())
            {
                bias = 2 * allbias.get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU_PHASE") / wavelength(gnss_data_obs(obs_type).band());
            }

            else
            {
                continue;
            }

            // apply dcb to obsdata
            double obs_value = this->getobs(gobs_type.gobs());
            if (obs_type == C1W)
            {
                //            std::cout << gsat << " " << gnss_data_obs(obs_type).gobs() << " " << bias << "  " << std::setw(20) << std::setprecision(6) << obs_value << std::endl;
                // printf("");
            }
            obs_value -= bias;
            this->resetobs(gobs_type.gobs(), obs_value);
        }
        this->_dcb_correct_mark = true;
        return true;
    }
    bool gnss_data_obs_manager::apply_bias_tmp2(gnss_all_bias &allbias)
    {
        std::string gsat = this->sat();
        base_time gepo = this->epoch();
        GSYS gsys = this->gsys();

        for (GOBS obs_type : this->obs())
        {

            gnss_data_obs gobs_type(obs_type);
            gobs_type.gobs2to3(gsys);

            double bias = 0.0;
            if (gnss_data_obs(obs_type).is_code())
            {
                bias = allbias.get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU");
            }
            else if (gnss_data_obs(obs_type).is_phase())
            {
                bias = 2 * allbias.get(gepo, gsat, gobs_type.gobs(), gobs_type.gobs(), "WHU_PHASE") / wavelength(gnss_data_obs(obs_type).band());
            }
            else
            {
                continue;
            }

            // apply dcb to obsdata
            double obs_value = this->getobs(gobs_type.gobs());
            if (obs_type == C1W)
            {
                //        std::cout << gsat << " " << gnss_data_obs(obs_type).gobs() << " " << bias << "  " << std::setw(20) << std::setprecision(6) << obs_value << std::endl;
                // printf("");
            }
            obs_value += bias;
            this->resetobs(gobs_type.gobs(), obs_value);
        }
        this->_dcb_correct_mark = true;
        return true;
    }

    bool gnss_data_obs_manager::apply_dcb(gnss_all_bias *allbias)
    {
        // lvhb modified in 20200730
          if (allbias == NULL)
          {
              if (_spdlog)
                  SPDLOG_LOGGER_DEBUG(_spdlog, "{} : NO DCB FILE", this->site());
              return true;
          }

        //  skip already correct dcb obsdata
        if (this->_dcb_correct_mark)
        {
            return true;
        }

        std::string gsat = this->sat();
        base_time gepo = this->epoch();
        GSYS gsys = this->gsys();

        // two band is bound to precise clock
        auto band1 = GNSS_BAND_PRIORITY.at(gsys)[1];
        auto band2 = GNSS_BAND_PRIORITY.at(gsys)[2];

        // caculate interdcb band1-band2
        double alfa12 = 0.0;
        // double alfa13 = 0.0;
        double beta12 = 0.0;
        // double beta13 = 0.0;
        this->coef_ionofree(band1, alfa12, band2, beta12);
        double inter_dcb_12 = this->interFreqDcb(*allbias, band1, band2);
        double inter_dcb_1x = 0.0;
        double inter_dcb_2x = 0.0;
        for (GOBS obs_type : this->obs())
        {

            // skip not code obs
            if (!gnss_data_obs(obs_type).is_code())
            {
                continue;
            }
            double obs_value = this->getobs(obs_type);

            if (double_eq(obs_value, 0.0))
                continue;

            // apply intra dcb correct [GPS]
            gnss_data_obs gobs_type(obs_type); 
            gobs_type.gobs2to3(gsys);   

            double dcb_value = 0.0;
            if (gsys == GSYS::GPS)
            {
                switch (gobs_type.gobs())
                {
                case C1C:
                    dcb_value = allbias->get(gepo, gsat, C1W, C1C);
                    break;
                case C1P:
                    dcb_value = 0.0;
                case C1Y:
                    dcb_value = 0.0;
                case C1W:
                    dcb_value = 0.0;
                    break;
                case C2C:
                    dcb_value = allbias->get(gepo, gsat, C2W, C2C);
                    break;
                case C2L:
                    dcb_value = allbias->get(gepo, gsat, C2W, C2L);
                    break;
                case C2X:
                    dcb_value = allbias->get(gepo, gsat, C2W, C2X);
                    break;
                case C2P:
                    dcb_value = 0.0;
                case C2Y:
                    dcb_value = 0.0;
                case C2W:
                    dcb_value = 0.0;
                    break;
                case C5Q:
                    dcb_value = allbias->get(gepo, gsat, C5X, C5Q);
                    break;
                case C5X:
                    dcb_value = 0.0;
                    break;
                default:
                    dcb_value = 0.0;
                    break;
                }
            }
            else if (gsys == GSYS::GLO)
            {
                switch (gobs_type.gobs())
                {
                case C1C:
                    dcb_value = allbias->get(gepo, gsat, C1P, C1C);
                    break;
                case C1P:
                    dcb_value = 0.0;
                    break;
                case C2C:
                    dcb_value = allbias->get(gepo, gsat, C2P, C2C);
                    break;
                case C2P:
                    dcb_value = 0.0;
                    break;
                default:
                    dcb_value = 0.0;
                    break;
                }
            }

            // apply inter dcb correct
            if (gobs_type.band() == band1)
            {
                dcb_value += -beta12 * inter_dcb_12;
                // std::cout << gepo.str_mjdsod("DCB + " + gsat) << "  dcb12 for band 1 : " + gobs2str(gobs_type.gobs()) << " " << std::setw(20) << inter_dcb_12 << std::endl;
            }
            else if (gobs_type.band() == band2)
            {
                dcb_value += +alfa12 * inter_dcb_12;
                // std::cout << gepo.str_mjdsod("DCB + " + gsat) << "  dcb12 for band 2 : " << gobs2str(gobs_type.gobs()) << " " << std::setw(20) << inter_dcb_12 << std::endl;
            }
            else
            {
                GOBSBAND band_x = gobs_type.band();
                inter_dcb_1x = this->interFreqDcb(*allbias, band1, band_x);
                inter_dcb_2x = this->interFreqDcb(*allbias, band2, band_x);
                dcb_value += alfa12 * inter_dcb_1x + beta12 * inter_dcb_2x;
                // std::cout << gepo.str_mjdsod("DCB + " + gsat) << "  dcb13 for band 3 : " << gobs2str(gobs_type.gobs()) << " " << std::setw(20) << inter_dcb_1x << std::endl;
                // std::cout << gepo.str_mjdsod("DCB + " + gsat) << "  dcb23 for band 3 : " << gobs2str(gobs_type.gobs()) << " " << std::setw(20) << inter_dcb_2x << std::endl;
                // std::cout << gepo.str_mjdsod("DCB + " + gsat) << "  dcb   for band 3 : " << gobs2str(gobs_type.gobs()) << " " << std::setw(20) << dcb_value    << std::endl;
            }

            // apply dcb to obsdata

            obs_value += dcb_value;
            this->resetobs(obs_type, obs_value);
        }
        this->_dcb_correct_mark = true;
        return true;
    }

    double gnss_data_obs_manager::interFreqDcb(gnss_all_bias &allbias, const GOBSBAND &band1, const GOBSBAND &band2) const
    {

        auto gsys = this->gsys();
        auto gsat = this->sat();
        auto grec = this->site();
        auto gepo = this->epoch();

        gnss_data_obs g1 = gnss_data_obs(this->select_range(band1, true));
        gnss_data_obs g2 = gnss_data_obs(this->select_range(band2, true));

        // std::cout << "2 " << gobs2str(g1.gobs()) << std::endl;
        // std::cout << "2 " << gobs2str(g2.gobs()) << std::endl;

        g1.gobs2to3(this->gsys());
        g2.gobs2to3(this->gsys());

        // std::cout << "3 " << gobs2str(g1.gobs()) << std::endl;
        // std::cout << "3 " << gobs2str(g2.gobs()) << std::endl;

        double dcb_value = 0.0;

        GOBS obs1 = gnss_sys::gobs_priority(gsys, g1.gobs());
        GOBS obs2 = gnss_sys::gobs_priority(gsys, g2.gobs());

        // auto default_band1 = GNSS_BAND_PRIORITY.at(gsys)[1];
        // auto default_band2 = GNSS_BAND_PRIORITY.at(gsys)[2];
        // auto default_band3 = GNSS_BAND_PRIORITY.at(gsys)[3];

        // if (band1 == default_band1 && obs1 == GOBS::X)
        //{
        //     return 0.0;
        // }

        if (obs1 == GOBS::X && obs2 == GOBS::X)
        {
            return 0.0;
        }

        // if (isIono13 && band1 == default_band2 && band2 == default_band3)
        //{
        //     obs1 = gnss_sys::gobs_defaults(gsys, obs2, band1);
        // }

        // if (isIono13 && band1 == default_band1 && band2 == default_band2)
        //{
        //     obs2 = gnss_sys::gobs_defaults(gsys, obs1, band1);
        // }

        if (obs1 == GOBS::X)
        {
            obs1 = gnss_sys::gobs_defaults(gsys, obs2, band1);
            // std::cout << gobs2str(obs1) + " " + gobs2str(obs2) + " " << " " + gsat + " " << " " + grec + " " << gepo.str_mjdsod("wrong type of GNSS sys, only support GPS and GAL.") << std::endl;
            // std::cerr << gobs2str(obs1) + " " + gobs2str(obs2) + " " << " " + gsat + " " << " " + grec + " " << gepo.str_mjdsod("wrong type of GNSS sys, only support GPS and GAL.") << std::endl;
            // return 0.0;
        }

        if (obs2 == GOBS::X)
        {
            obs2 = gnss_sys::gobs_defaults(gsys, obs1, band2);
        }

        dcb_value = allbias.get(gepo, gsat, obs1, obs2);
        if (double_eq(dcb_value, 0.0))
        {
            // lvhb
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "interFreqDcb {}", gobs2str(obs1) + " " + gobs2str(obs2) + " " + " " + gsat + " " + " " + grec + " " + gepo.str_mjdsod("wrong type of GNSS sys, only support GPS and GAL."));
            // std::cout << gobs2str(obs1) + " " + gobs2str(obs2) + " " << " " + gsat + " " << " " + grec + " " << gepo.str_mjdsod("wrong type of GNSS sys, only support GPS and GAL.")  << std::endl;
            return 0.0;
        }

        return dcb_value;
    }

    // add OBS - code [m], phase [whole cycles], dopler[cycles/sec], snr [DBHZ], ...  observations (double)
    // -----------
    double gnss_data_obs_manager::getobs(const std::string &obs) const
    {
        double tmp = NULL_GOBS;
        if (_gobs.find(str2gobs(obs)) != _gobs.end())
        {
            tmp = _gobs.at(str2gobs(obs));
        }
        return tmp;
    }

    // get LLI - lost of lock indicator (int)
    // -----------
    int gnss_data_obs_manager::getlli(const GOBS &obs) const
    {
        int tmp = 0;
        if (_glli.find(obs) != _glli.end())
            tmp = _glli.at(obs);
        return tmp;
    }

    // get an estimated cycle slip
    // -----------
    int gnss_data_obs_manager::getslip(const GOBS &obs) const
    {
        int tmp = 0;
        if (_gslip.find(obs) != _gslip.end())
            tmp = _gslip.at(obs);
        return tmp;
    }

    // get range outliers
    // -----------
    int gnss_data_obs_manager::getoutliers(const GOBS &obs) const
    {
        int tmp = 0;
        if (_goutlier.find(obs) != _goutlier.end())
            tmp = _goutlier.at(obs);
        return tmp;
    }

    // get gObsLevel indicator, default -1
    // -----------
    int gnss_data_obs_manager::getobsLevelFlag(const GOBS &obs)
    {
        int tmp = -1;
        if (_gLevel.find(obs) != _gLevel.end())
            tmp = _gLevel[obs];
        return tmp;
    }

    bool gnss_data_obs_manager::obsWithCorr()
    {
        bool tmp = true;
        for (auto it : _gLevel)
        {
            if (it.second == 0)
            {
                tmp = false;
                break;
            }
        }
        return tmp;
    }

    // get LLI - lost of lock indicator (int)
    // -----------
    int gnss_data_obs_manager::getlli(const std::string &obs) const
    {
        int tmp = 0;
        if (_glli.find(str2gobs(obs)) != _glli.end())
            tmp = _glli.at(str2gobs(obs));
        return tmp;
    }

    // add observation - special std::map of doubles
    // -----------
    void gnss_data_obs_manager::addele(const double &d)
    {
        _apr_ele = d;
        return;
    }

    // add observation - special std::map of doubles
    // -----------
    const double &gnss_data_obs_manager::getele() const
    {
        return _apr_ele;
    }

    // std::set chanel number for Glonass satellites
    // -----------------
    void gnss_data_obs_manager::channel(const int &ch)
    {
        _channel = ch;
        return;
    }

    // get chanel number for Glonass satellites
    // -----------------
    const int &gnss_data_obs_manager::channel() const
    {
        return _channel;
    }

    // return # of observations
    size_t gnss_data_obs_manager::size() const
    {
        size_t tmp = _gobs.size();
        return tmp;
    }

    // // get system-specific frequency for band i
    // ----------
    // TEMPORARY !!!!
    double gnss_data_obs_manager::frequency(const int &b) const
    {
        GOBSBAND gb = GOBSBAND(b);
        return this->frequency(gb);
    }

    // // get system-specific frequency for band i
    // ----------
    double gnss_data_obs_manager::frequency(const GOBSBAND &b) const
    {
        //  return gnss_sys::frequency( _gsys, b ); // CHYBI CHANNEL FOR GLONASS !!

        switch (_gsys)
        {

        case GPS:
            switch (b)
            {
            case BAND_1:
                return G01_F;
            case BAND_2:
                return G02_F;
            case BAND_5:
                return G05_F;
            default:
                return 0.0;
            }

        case GLO:
            switch (b)
            {
            case BAND_1:
                return R01_F(_channel);
            case BAND_2:
                return R02_F(_channel);
            case BAND_3:
                return R03_F_CDMA;
            case BAND_5:
                return R05_F_CDMA;
            default:
                return 0.0;
            }

        case GAL:
            switch (b)
            {
            case BAND_1:
                return E01_F;
            case BAND_5:
                return E05_F;
            case BAND_6:
                return E06_F;
            case BAND_7:
                return E07_F;
            case BAND_8:
                return E08_F;
            default:
                return 0.0;
            }

        case BDS:
            switch (b)
            {
            case BAND_2:
                return C02_F;
            case BAND_6:
                return C06_F;
            case BAND_7:
                return C07_F;
            case BAND_5:
                return C05_F; // glfeng add following
            case BAND_8:
                return C08_F;
            case BAND_9:
                return C09_F;
            case BAND_1:
                return C01_F;

            default:
                return 0.0;
            }

        case QZS:
            switch (b)
            {
            case BAND_1:
                return J01_F;
            case BAND_2:
                return J02_F;
            case BAND_5:
                return J05_F;
            case BAND_6:
                return J06_F;
            default:
                return 0.0;
            }

        case IRN:
            switch (b)
            {
            case BAND_5:
                return I05_F;
            default:
                return 0.0;
            }

        case SBS:
            switch (b)
            {
            case BAND_1:
                return S01_F;
            case BAND_5:
                return S05_F;
            default:
                return 0.0;
            }

        case GNS:
            return 0.0;
        default:
            return 0.0;
        }

        return 0.0;
    }

    // get system-specific wavelength for band i
    // ----------
    double gnss_data_obs_manager::wavelength(const GOBSBAND &b) const
    {
        double frq = this->frequency(b);
        if (frq != 0.0)
            return CLIGHT / frq;

        return 0.0;
    }

    // get wavelength for iono-fee LC band i,j
    // ----------
    double gnss_data_obs_manager::wavelength_L3(const GOBSBAND &b1, const GOBSBAND &b2) const
    {
        double c1, c2;
        _coef_ionofree(b1, c1, b2, c2);

        double f1 = this->frequency(b1);
        double f2 = this->frequency(b2);

        if (!double_eq(f1, 0.0) && !double_eq(f2, 0.0))
        {
            double lamb_L3 = c1 * CLIGHT / f1 + c2 * CLIGHT / f2;
            return lamb_L3;
        }
        return 0.0;
    }

    // get wavelength for wild lane WL
    // ----------
    double gnss_data_obs_manager::wavelength_WL(const GOBSBAND &b1, const GOBSBAND &b2) const
    {
        /*double c1, c2;
        _coef_ionofree(b1, c1, b2, c2);*/

        double f1 = this->frequency(b1);
        double f2 = this->frequency(b2);

        if (!double_eq(f1, 0.0) && !double_eq(f2, 0.0))
        {
            return CLIGHT / (f1 - f2);
        }
        return 0.0;
    }

    // get wavelength for narrow lane NL
    // ----------
    double gnss_data_obs_manager::wavelength_NL(const GOBSBAND &b1, const GOBSBAND &b2) const
    {
        /*double c1, c2;
        _coef_ionofree(b1, c1, b2, c2);*/

        double f1 = this->frequency(b1);
        double f2 = this->frequency(b2);

        if (!double_eq(f1, 0.0) && !double_eq(f2, 0.0))
        {
            double lamb_WL = CLIGHT / (f1 + f2);
            return lamb_WL;
        }
        return 0.0;
    }

    // get GNSS system from satellite IDOC
    // ----------
    GSYS gnss_data_obs_manager::gsys() const
    {
        return _gsys;
    }

    // code observations [m]
    // ----------
    double gnss_data_obs_manager::obs_C(const gnss_data_obs &go) const
    {
        double tmp = this->_obs_range(go);
        return tmp;
    }

    double gnss_data_obs_manager::obs_C(const GOBS &gobs) const
    {
        gnss_data_obs obs(gobs);
        double tmp = this->_obs_range(obs);
        return tmp;
    }

    // code observations [m]
    // ----------
    double gnss_data_obs_manager::obs_C(const gnss_data_band &gb) const
    {
        double tmp = this->_obs_range(gb);
        return tmp;
    }

    // code observations [m]
    // ----------
    double gnss_data_obs_manager::_obs_range(const gnss_data_obs &go) const
    {
        // Glonass has 255 channel number
        if (!_valid_obs())
            return NULL_GOBS;

        // AUTO SELECTION
        if (go.attr() == ATTR)
        {
            GOBS gobs = _id_range(go.band());
            if (gobs == X)
                return NULL_GOBS;
            else
                return _gobs.at(gobs);
            return NULL_GOBS;
        }

        // FIXED SELECTION
        // here avoid a PROBLEM WITH the legacy TYPE_P (only when NULL_attr)
        // -----------------------------
        std::map<GOBS, double>::const_iterator it = _gobs.end();

        if (go.attr() == ATTR_NULL &&
            (go.type() == TYPE ||
             go.type() == TYPE_P))
        {
            gnss_data_obs gobs(TYPE_P, go.band(), go.attr());
            it = _gobs.find(gobs.gobs());
        }

        if (it == _gobs.end() &&
            go.type() != TYPE_P)
        {
            gnss_data_obs gobs(TYPE_C, go.band(), go.attr());
            it = _gobs.find(gobs.gobs());
        }

        if (it == _gobs.end())
            return NULL_GOBS;

        return it->second;
    }

    // code observations [m]
    // ----------
    double gnss_data_obs_manager::_obs_range(const gnss_data_band &gb) const
    {
        gnss_data_obs gobs(TYPE, gb.band(), gb.attr());

#ifdef DEBUG
        std::cout << _epoch.str_hms() << " sat: " << sat() << " band: " << gobsband2str(gb.band()) << " attr: " << gobsattr2str(gb.attr()) << std::endl;
#endif

        return _obs_range(gobs);
    }

    // phase observations [m]
    // ----------
    double gnss_data_obs_manager::obs_L(const gnss_data_obs &go) const
    {
        double tmp = this->_obs_phase(go.gband());
        return tmp;
    }

    double gnss_data_obs_manager::obs_L(const GOBS &gobs) const
    {
        gnss_data_obs obs(gobs);
        double tmp = this->_obs_phase(obs.gband());
        return tmp;
    }

    double gnss_data_obs_manager::obs_D(const gnss_data_obs &gobs) const
    {
        double tmp = this->_obs_doppler(gobs.gband());
        return tmp;
    }

    double gnss_data_obs_manager::obs_D(const GOBS &gobs) const
    {
        gnss_data_obs obs(gobs);
        double tmp = this->_obs_doppler(obs.gband());
        return tmp;
    }

    double gnss_data_obs_manager::obs_S(const gnss_data_obs &gobs) const
    {
        double tmp = this->_obs_snr(gobs.gband());
        return tmp;
    }

    // phase observations [m]
    // ----------
    double gnss_data_obs_manager::obs_L(const gnss_data_band &gb) const
    {
        double tmp = this->_obs_phase(gb);
        return tmp;
    }

    double gnss_data_obs_manager::obs_D(const gnss_data_band &gb) const
    {
        double tmp = this->_obs_doppler(gb);
        return tmp;
    }

    double gnss_data_obs_manager::obs_S(const gnss_data_band &gb) const
    {
        double tmp = this->_obs_snr(gb);
        return tmp;
    }

    // phase observations [m]
    // ----------
    double gnss_data_obs_manager::_obs_phase(const gnss_data_band &gb) const
    {
        // Glonass has 255 channel number
        if (!_valid_obs())
            return NULL_GOBS;

        // AUTO SELECTION
        if (gb.attr() == ATTR)
        {
            GOBS gobs = _id_phase(gb.band());
            if (gobs == X)
                return NULL_GOBS;
            else
                return _gobs.at(gobs) * this->wavelength(gb.band()); // transfer from whole cycles to meters!;
        }

        // FIXED SELECTION
        gnss_data_obs go(TYPE_L, gb.band(), gb.attr());

        std::map<GOBS, double>::const_iterator it;
        it = _gobs.find(go.gobs());

        if (it == _gobs.end())
            return NULL_GOBS;
        return it->second * this->wavelength(gb.band()); // transfer from whole cycles to meters!
    }

    double gnss_data_obs_manager::_obs_doppler(const gnss_data_band &gb) const
    {
        // Glonass has 255 channel number
        if (!_valid_obs())
            return NULL_GOBS;

        // AUTO SELECTION
        if (gb.attr() == ATTR)
        {
            GOBS gobs = _id_doppler(gb.band());
            if (gobs == X)
                return NULL_GOBS;
            else
                return _gobs.at(gobs) * this->wavelength(gb.band());
            return NULL_GOBS;
        }

        gnss_data_obs go(TYPE_D, gb.band(), gb.attr());

        std::map<GOBS, double>::const_iterator it = _gobs.end();

        it = _gobs.find(go.gobs());
        if (it == _gobs.end())
            return NULL_GOBS;

        return it->second * this->wavelength(gb.band());

        return 0.0;
    }

    double gnss_data_obs_manager::_obs_snr(const gnss_data_band &gb) const
    {
        // Glonass has 255 channel number
        if (!_valid_obs())
            return NULL_GOBS;

        // AUTO SELECTION
        if (gb.attr() == ATTR)
        {
            GOBS gobs = _id_snr(gb.band());
            if (gobs == X)
                return NULL_GOBS;
            else
                return _gobs.at(gobs);
            return NULL_GOBS;
        }

        gnss_data_obs go(TYPE_S, gb.band(), gb.attr());

        std::map<GOBS, double>::const_iterator it = _gobs.end();

        it = _gobs.find(go.gobs());
        if (it == _gobs.end())
            return NULL_GOBS;

        return it->second;

        return 0.0;
    }

    // modify phase observations [m]
    // ----------
    int gnss_data_obs_manager::mod_L(const double &dL, const GOBS &gobs, const int i)
    {
        std::map<GOBS, double>::iterator it;

        if (gobs == X)
        {
            for (it = _gobs.begin(); it != _gobs.end(); ++it)
            {
                // phase only !!!!
                std::string gobs_str = gobs2str(it->first);
                if (gobs_str.compare(0, 1, "L") != 0)
                    continue;

                // TEMPORARY CONVERSION !
                GOBSBAND gb1 = GOBSBAND(gobs2band(it->first));

                if (i == 1)
                    it->second += dL / this->wavelength(gb1); // transfer back to cycles
                else if (i == 0)
                    it->second += dL;
                else
                    std::cout << "Observations not modified!" << std::endl;
                //      std::cout << "! POZOR ! modifikuji: " << gobs2str(it->first) << " o " << dL << " band " << band << " " << epoch().str_hms() << std::endl;
            }

            // modify requested only
        }
        else
        {
            it = _gobs.find(gobs);
            if (it == _gobs.end())
                return -1;

            // TEMPORARY CONVERSION !
            GOBSBAND gb1 = GOBSBAND(gobs2band(gobs));

            if (i == 1)
                it->second += dL / this->wavelength(gb1); // transfer back to cycles
            else if (i == 0)
                it->second += dL;
            else
                std::cout << "Observations not modified!" << std::endl;
            //    std::cout << "! POZOR ! modifikuji: " << gobs2str(gobs) << " o " << dL << " band " << band << std::endl;
        }
        return 0; // not found
    }

    // modify phase observations of one band [m]
    // ----------
    int gnss_data_obs_manager::mod_L(const double &dL, const GOBSBAND &band, const int i)
    {
        for (std::map<GOBS, double>::iterator it = _gobs.begin(); it != _gobs.end(); ++it)
        {
            // phase only !!!!
            std::string gobs_str = gobs2str(it->first);
            if (gobs_str.compare(0, 1, "L") != 0)
                continue;

            // TEMPORARY CONVERSION !
            GOBSBAND gb1 = GOBSBAND(gobs2band(it->first));
            if (gb1 != band)
                continue;

            if (i == 1)
                it->second += dL / this->wavelength(gb1); // transfer back to cycles
            else if (i == 0)
                it->second += dL;
            else if (i == 2)
                it->second = 0; // std::set to zero
            else
                std::cout << "Observations not modified!" << std::endl;
            //      std::cout << "! POZOR ! modifikuji: " << gobs2str(it->first) << " o " << dL << " band " << band << " " << epoch().str_hms() << std::endl;
        }
        return 0;
    }

    // return value of carrier-phase linear commbination frequency
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::frequency_lc(const int &band1, const double &coef1,
                                    const int &band2, const double &coef2,
                                    const int &band3, const double &coef3) const
    {
        double f1 = 0.0;
        double f2 = 0.0;
        double f3 = 0.0;

        // TEMPORARY CONVERSION !
        GOBSBAND gb1 = GOBSBAND(band1);
        GOBSBAND gb2 = GOBSBAND(band2);
        GOBSBAND gb3 = GOBSBAND(band3);

        if (coef1 != 0.0)
        {
            f1 = this->frequency(gb1);
            if (double_eq(f1, NULL_GOBS))
                return NULL_GOBS;
        }
        if (coef2 != 0.0)
        {
            f2 = this->frequency(gb2);
            if (double_eq(f2, NULL_GOBS))
                return NULL_GOBS;
        }
        if (coef3 != 0.0)
        {
            f3 = this->frequency(gb3);
            if (double_eq(f3, NULL_GOBS))
                return NULL_GOBS;
        }

        double freq = (coef1 * f1 + coef2 * f2 + coef3 * f3);

#ifdef DEBUG
        std::cout << _satid << " " << _staid
             << " frequency_lc BAND(" << band1 << "," << band2 << "," << band3 << ") "
             << "  COEF(" << coef1 << "," << coef2 << "," << coef3 << ") "
             << std::fixed << std::setprecision(3) << std::setw(16) << f1 << std::setw(16) << f2 << std::setw(16) << f3 << "  freq = " << std::setw(16) << freq << std::endl;
#endif
        return freq;
    }

    // return value of carrier-phase linear combination ionosphere scale factor
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::isf_lc(const int &band1, const double &coef1,
                              const int &band2, const double &coef2) const
    {

        double f1 = 1.0;
        double f2 = 1.0;

        // TEMPORARY CONVERSION !
        GOBSBAND gb1 = GOBSBAND(band1);
        GOBSBAND gb2 = GOBSBAND(band2);

        f1 = this->frequency(gb1);
        if (double_eq(f1, NULL_GOBS))
            return NULL_GOBS;
        f2 = this->frequency(gb2);
        if (double_eq(f2, NULL_GOBS))
            return NULL_GOBS;

        double denom = coef1 * f1 + coef2 * f2;
        double num = f1 * f1 * (coef1 / f1 + coef2 / f2);
        double isf = num / denom;

#ifdef DEBUG
        std::cout << _satid << " " << _staid
             << " isf_lc BAND(" << band1 << "," << band2 << ") "
             << "  COEF(" << coef1 << "," << coef2 << ") "
             << std::fixed << std::setprecision(3) << std::setw(16) << f1 << std::setw(16) << f2 << std::setw(16) << "  isf = " << std::setw(16) << isf << std::endl;
#endif
        return isf;
    }

    // return value of carrier-phase linear combination noise factor
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::pnf_lc(const int &band1, const double &coef1,
                              const int &band2, const double &coef2,
                              const int &band3, const double &coef3) const
    {
        double f1 = 1.0;
        double f2 = 1.0;
        double f3 = 1.0;

        // TEMPORARY CONVERSION !
        GOBSBAND gb1 = GOBSBAND(band1);
        GOBSBAND gb2 = GOBSBAND(band2);
        GOBSBAND gb3 = GOBSBAND(band3);

        if (coef1 != 0.0)
        {
            f1 = this->frequency(gb1);
            if (double_eq(f1, NULL_GOBS))
                return NULL_GOBS;
        }
        if (coef2 != 0.0)
        {
            f2 = this->frequency(gb2);
            if (double_eq(f2, NULL_GOBS))
                return NULL_GOBS;
        }
        if (coef3 != 0.0)
        {
            f3 = this->frequency(gb3);
            if (double_eq(f3, NULL_GOBS))
                return NULL_GOBS;
        }

        double denom = pow(coef1 * f1 + coef2 * f2 + coef3 * f3, 2);
        double num = f1 * f1 * coef1 * coef1 + f2 * f2 * coef2 * coef2 + f3 * f3 * coef3 * coef3;
        double pnf = num / denom;

#ifdef DEBUG
        std::cout << _satid << " " << _staid
             << " pnf_lc BAND(" << band1 << "," << band2 << "," << band3 << ") "
             << "  COEF(" << coef1 << "," << coef2 << "," << coef3 << ") "
             << std::fixed << std::setprecision(3) << std::setw(16) << f1 << std::setw(16) << f2 << std::setw(16) << f3 << "  pnf = " << std::setw(16) << sqrt(pnf) << std::endl;
#endif
        return sqrt(pnf);
    }

    // return value of general code linear combination
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::_lcf_range(const gnss_data_obs *gobs1, const int &coef1,
                                  const gnss_data_obs *gobs2, const int &coef2,
                                  const gnss_data_obs *gobs3, const int &coef3) const
    {
        double C1 = NULL_GOBS, f1 = 0.0;
        double C2 = NULL_GOBS, f2 = 0.0;
        double C3 = NULL_GOBS, f3 = 0.0;

        if (gobs1 && coef1 != 0.0)
        {
            f1 = frequency(gobs1->band());
            C1 = obs_C(*gobs1);
            if (double_eq(C1, NULL_GOBS))
                return NULL_GOBS;
        }
        if (gobs2 && coef2 != 0.0)
        {
            f2 = frequency(gobs2->band());
            C2 = obs_C(*gobs2);
            if (double_eq(C2, NULL_GOBS))
                return NULL_GOBS;
        }
        if (gobs3 && coef3 != 0.0)
        {
            f3 = frequency(gobs3->band());
            C3 = obs_C(*gobs3);
            if (double_eq(C3, NULL_GOBS))
                return NULL_GOBS;
        }

        double lc = NULL_GOBS;

        if (fabs(C1 - C2) > 100)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Inconsistencies of the code observations " + _epoch.str_ymdhms());
            return lc;
        }

        std::string sat = this->sat();

        if (gobs3)
            lc = (coef1 * f1 * C1 + coef2 * f2 * C2 + coef3 * f3 * C3) / (coef1 * f1 + coef2 * f2 + coef3 * f3);
        else
            lc = (coef1 * f1 * C1 + coef2 * f2 * C2) / (coef1 * f1 + coef2 * f2);

#ifdef DEBUG
        std::cout << std::fixed << std::setprecision(3)
             << " _lcf_range C1(" << gobs1->band() << ":" << coef1 << "):" << std::setw(16) << C1
             << "  C2(" << gobs2->band() << ":" << coef2 << "):" << std::setw(16) << C2;
        if (gobs3)
            std::cout << "  C3(" << gobs3->band() << ":" << coef3 << "):" << std::setw(16) << C3;
        std::cout << " lc = " << lc << std::endl;
#endif

        return lc;
    }

    // return value of general phase linear combination
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::_lcf_phase(const gnss_data_obs *gobs1, const int &coef1,
                                  const gnss_data_obs *gobs2, const int &coef2,
                                  const gnss_data_obs *gobs3, const int &coef3) const
    {
        double L1 = NULL_GOBS, f1 = 0;
        double L2 = NULL_GOBS, f2 = 0;
        double L3 = NULL_GOBS, f3 = 0;

        if (gobs1 && coef1 != 0.0)
        {
            f1 = frequency(gobs1->band());
            L1 = obs_L(*gobs1);
            if (double_eq(L1, NULL_GOBS))
                return NULL_GOBS;
        }
        if (gobs2 && coef2 != 0.0)
        {
            f2 = frequency(gobs2->band());
            L2 = obs_L(*gobs2);
            if (double_eq(L2, NULL_GOBS))
                return NULL_GOBS;
        }
        if (gobs3 && coef3 != 0.0)
        {
            f3 = frequency(gobs3->band());
            L3 = obs_L(*gobs3);
            if (double_eq(L3, NULL_GOBS))
                return NULL_GOBS;
        }

        double lc = NULL_GOBS;

        if (gobs3)
            lc = (coef1 * f1 * L1 + coef2 * f2 * L2 + coef3 * f3 * L3) / (coef1 * f1 + coef2 * f2 + coef3 * f3);
        else
            lc = (coef1 * f1 * L1 + coef2 * f2 * L2) / (coef1 * f1 + coef2 * f2);

#ifdef DEBUG
        std::cout << std::fixed << std::setprecision(3)
             << " _lcf_phase L1(" << gobs1->band() << ":" << coef1 << "):" << std::setw(16) << L1
             << "  L2(" << gobs2->band() << ":" << coef2 << "):" << std::setw(16) << L2;
        if (gobs3)
            std::cout << "  L3(" << gobs3->band() << ":" << coef3 << "):" << std::setw(16) << L3;
        std::cout << " lc = " << lc << std::endl;
#endif

        return lc;
    }

    // return coefficients (c1,c2) of the code multipath linear combination
    // ----------
    int gnss_data_obs_manager::_coef_multpath(const GOBSBAND &bC, double &cC,
                                   const GOBSBAND &b1, double &c1,
                                   const GOBSBAND &b2, double &c2) const
    {
        if (b1 == b2)
        {
            c1 = c2 = 0.0;
            return -1;
        } // phase should be different!

        double pfacC = pow(this->frequency(bC), 2);
        double pfac1 = pow(this->frequency(b1), 2);
        double pfac2 = pow(this->frequency(b2), 2);

        double denominator = (pfac1 - pfac2) * pfacC;

        cC = 1.0;
        c1 = -(pfac1 * (pfacC + pfac2)) / denominator;
        c2 = +(pfac2 * (pfacC + pfac1)) / denominator;

        return 0;
    }

    // Ionosphere-linear code combination [m]
    // ----------
    double gnss_data_obs_manager::P3(const GOBSBAND &b1, const GOBSBAND &b2) const // const
    {
        double coef1, coef2;

        gnss_data_obs g1(TYPE, b1, ATTR);
        gnss_data_obs g2(TYPE, b2, ATTR);

        _coef_ionofree(g1.band(), coef1, g2.band(), coef2);

        double lc = _lc_range(&g1, coef1, &g2, coef2); // still dual-frequency only

        //  std::cout << "gnss_data_obs_manager::P3() " << sat() << " " << std::fixed << std::setprecision(3) << lc << std::endl;
        return lc;
    }

    // Ionosphere-linear code combination [m]
    // ----------
    double gnss_data_obs_manager::P3(const gnss_data_obs &g1, const gnss_data_obs &g2) const // const
    {
        double coef1, coef2;

        _coef_ionofree(g1.band(), coef1, g2.band(), coef2);

        double lc = _lc_range(&g1, coef1, &g2, coef2); // still dual-frequency only

        //  std::cout << "gnss_data_obs_manager::P3() " << sat() << " " << std::fixed << std::setprecision(3) << lc << std::endl;
        return lc;
    }

    double gnss_data_obs_manager::P3(const GOBS &g1, const GOBS &g2) const
    {
        gnss_data_obs tmp1(g1);
        gnss_data_obs tmp2(g2);

        double coef1, coef2;

        _coef_ionofree(tmp1.band(), coef1, tmp2.band(), coef2);

        double lc = _lc_range(&tmp1, coef1, &tmp2, coef2); // still dual-frequency only

        //  std::cout << "gnss_data_obs_manager::P3() " << sat() << " " << std::fixed << std::setprecision(3) << lc << std::endl;
        return lc;
    }

    // get Geometry-free phase combination [m]
    // ----------
    double gnss_data_obs_manager::P4(const GOBSBAND &b1, const GOBSBAND &b2) const // const
    {
        double coef1, coef2;

        gnss_data_obs g1(TYPE, b1, ATTR);
        gnss_data_obs g2(TYPE, b2, ATTR);

        _coef_geomfree(g1.band(), coef1, g2.band(), coef2);

        double lc = _lc_range(&g1, coef1, &g2, coef2); // still dual-frequency only
        return lc;
    }

    // get Geometry-free phase combination [m]
    // ----------
    double gnss_data_obs_manager::P4(const gnss_data_obs &g1, const gnss_data_obs &g2) const // const
    {
        double coef1, coef2;

        _coef_geomfree(g1.band(), coef1, g2.band(), coef2);

        double lc = _lc_range(&g1, coef1, &g2, coef2); // still dual-frequency only
        return lc;
    }

    // Ionosphere-free linear phase combination [m] !
    // ----------
    double gnss_data_obs_manager::L3(const GOBSBAND &b1, const GOBSBAND &b2) const // const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            //      std::cout << epoch().str_hms() << " " << sat() << " " << _channel << std::endl;
            return 0.0;
        }

        double coef1, coef2;

        gnss_data_obs g1(TYPE_L, b1, ATTR);
        gnss_data_obs g2(TYPE_L, b2, ATTR);

        _coef_ionofree(b1, coef1, b2, coef2);

        double lc = _lc_phase(&g1, coef1, &g2, coef2); // still dual-frequency only

        //  std::cout << "gnss_data_obs_manager::L3() " << sat() << " " << std::fixed << std::setprecision(3) << lc << std::endl;
        return lc;
    }

    // Ionosphere-free linear phase combination [m] !
    // ----------
    double gnss_data_obs_manager::L3(const gnss_data_obs &g1, const gnss_data_obs &g2) const // const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            //      std::cout << epoch().str_hms() << " " << sat() << " " << _channel << std::endl;
            return 0.0;
        }

        double coef1, coef2;

        _coef_ionofree(g1.band(), coef1, g2.band(), coef2);

        double lc = _lc_phase(&g1, coef1, &g2, coef2); // still dual-frequency only

        //  std::cout << "gnss_data_obs_manager::L3() " << sat() << " " << std::fixed << std::setprecision(3) << lc << std::endl;
        return lc;
    }

    double gnss_data_obs_manager::L3(const GOBS &g1, const GOBS &g2) const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            //      std::cout << epoch().str_hms() << " " << sat() << " " << _channel << std::endl;
            return 0.0;
        }

        gnss_data_obs tmp1(g1);
        gnss_data_obs tmp2(g2);

        double coef1, coef2;

        _coef_ionofree(tmp1.band(), coef1, tmp2.band(), coef2);

        double lc = _lc_phase(&tmp1, coef1, &tmp2, coef2); // still dual-frequency only

        //  std::cout << "gnss_data_obs_manager::L3() " << sat() << " " << std::fixed << std::setprecision(3) << lc << std::endl;
        return lc;
    }

    // Ionosphere-free linear phase combination [cycle] ! glfeng
    // ----------
    double gnss_data_obs_manager::L3_cycle(const gnss_data_obs &g1, const gnss_data_obs &g2) const // const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            //      std::cout << epoch().str_hms() << " " << sat() << " " << _channel << std::endl;
            return 0.0;
        }

        double wlength1 = wavelength(g1.band());
        double wlength2 = wavelength(g2.band());
        double fact = wlength1 / wlength2;
        double fact2 = fact * fact;

        double L1 = NULL_GOBS;
        double L2 = NULL_GOBS;

        L1 = obs_L(g1) / wlength1;
        if (double_eq(L1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L2 = obs_L(g2) / wlength2;
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double lc = L1 / (1.0 - fact2) - L2 / (1.0 / fact - fact);
        return lc;
    }

    // get Geometry-free combination [m] !
    // ----------
    double gnss_data_obs_manager::L4(const GOBSBAND &b1, const GOBSBAND &b2) const // const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        gnss_data_obs g1(TYPE_L, b1, ATTR);
        gnss_data_obs g2(TYPE_L, b2, ATTR);

        double coef1, coef2;

        _coef_geomfree(b1, coef1, b2, coef2);

        double lc = _lc_phase(&g1, coef1, &g2, coef2); // still dual-frequency only

#ifdef DEBUG
        std::cout << " L4 = " << std::fixed << std::setprecision(3)
             << std::setw(16) << lc
             << std::setw(16) << _lc_phase(&g1, 1, &g2, 0) // first  band
             << std::setw(16) << _lc_phase(&g1, 0, &g2, 1) // second band
             << std::endl;
#endif
        return lc;
    }

    // get Geometry-free combination [m] !
    // ----------
    double gnss_data_obs_manager::L4(const gnss_data_obs &g1, const gnss_data_obs &g2) const // const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double coef1, coef2;

        _coef_geomfree(g1.band(), coef1, g2.band(), coef2);

        // double lc =  phase_lc( &g1, coef1, &g2, coef2 ); // still dual-frequency only
        double lc = _lc_phase(&g1, coef1, &g2, coef2); // still dual-frequency only

#ifdef DEBUG
        std::cout << " L4 = " << std::fixed << std::setprecision(3)
             << std::setw(16) << lc
             << std::setw(16) << _lc_phase(&g1, 1, &g2, 0) // first  band
             << std::setw(16) << _lc_phase(&g1, 0, &g2, 1) // second band
             << std::endl;
#endif
        return lc;
    }

    // get Geometry-free combination [cycle] ! glfeng
    // ----------
    double gnss_data_obs_manager::L4_cycle(const gnss_data_obs &g1, const gnss_data_obs &g2) const // const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wlength1 = wavelength(g1.band());
        double wlength2 = wavelength(g2.band());
        double fact = wlength1 / wlength2;

        double L1 = NULL_GOBS;
        double L2 = NULL_GOBS;

        L1 = obs_L(g1) / wlength1;
        if (double_eq(L1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L2 = obs_L(g2) / wlength2;
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double lg = (fact * L1 - L2) / (1 - fact);
        return lg;
    }

    // Melbourne-Wuebenna combination phase & code [m] !
    // ----------
    double gnss_data_obs_manager::MW(const gnss_data_obs &g1, const gnss_data_obs &g2, bool phacod_consistent) const
    {
        if (gsys() == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wideL = NULL_GOBS;
        double narrP = NULL_GOBS;

        gnss_data_obs L1(TYPE_L, g1.band(), g1.attr()); // should always be as input 1st argument
        gnss_data_obs L2(TYPE_L, g2.band(), g2.attr()); // should always be as input 2nd argument

        gnss_data_obs C1(TYPE, g1.band(), ATTR); // TYPE universal to handle P+C code
        gnss_data_obs C2(TYPE, g2.band(), ATTR); // TYPE universal to handle P+C code

        if (phacod_consistent)
        { // overwrite attribute
            C1.gattr(g1.gattr());
            C2.gattr(g2.gattr());
        }

        wideL = _lcf_phase(&L1, 1, &L2, -1);
        narrP = _lcf_range(&C1, 1, &C2, 1);

        if (double_eq(narrP, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        if (double_eq(wideL, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double lc = narrP - wideL;

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << " " << _satid << " " << gobs2str(L1.gobs()) << " " << gobs2str(C1.gobs()) << " " << gobs2str(L2.gobs()) << " " << gobs2str(C2.gobs()) << std::endl;
        std::cout << "wideL = " << wideL << std::endl;
        std::cout << "narrP = " << narrP << std::endl;
        std::cout << "MW = " << lc << std::endl;
#endif
        return lc;
    }

    // Melbourne-Wuebenna combination phase & code [cycle] !
    // ----------
    double gnss_data_obs_manager::MW_cycle(const gnss_data_obs &gL1, const gnss_data_obs &gL2,
                                const gnss_data_obs &gC1, const gnss_data_obs &gC2) const
    {
        if (gsys() == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wlength1 = wavelength(gL1.band());
        double wlength2 = wavelength(gL2.band());
        double fact = wlength1 / wlength2;

        double C1 = NULL_GOBS;
        double C2 = NULL_GOBS;
        double L1 = NULL_GOBS;
        double L2 = NULL_GOBS;

        C1 = obs_C(gC1);
        if (double_eq(C1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        C2 = obs_C(gC2);
        if (double_eq(C2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L1 = obs_L(gL1) / wlength1;
        if (double_eq(L1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L2 = obs_L(gL2) / wlength2;
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double mw = L1 - L2 - (C1 / wlength1 + C2 / wlength2) * (1.0 - fact) / (1.0 + fact);

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << " " << _satid << " " << gobs2str(L1.gobs()) << " " << gobs2str(C1.gobs()) << " " << gobs2str(L2.gobs()) << " " << gobs2str(C2.gobs()) << std::endl;
        std::cout << "wideL = " << wideL << std::endl;
        std::cout << "narrP = " << narrP << std::endl;
        std::cout << "MW = " << lc << std::endl;
#endif
        return mw;
    }
    double gnss_data_obs_manager::EWL_cycle(const gnss_data_obs &gL1, const gnss_data_obs &gL2, const gnss_data_obs &gL3,
                                 const gnss_data_obs &gC1, const gnss_data_obs &gC2) const
    {
        if (gsys() == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wlength1 = wavelength(gL1.band());
        double wlength2 = wavelength(gL2.band());
        double wlength3 = wavelength(gL3.band());
        if (double_eq(wlength1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        if (double_eq(wlength2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        if (double_eq(wlength3, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        double alpha = (wlength2 * wlength3 - wlength1 * wlength1) / (wlength2 * wlength2 - wlength1 * wlength1);

        double C1 = NULL_GOBS;
        double C2 = NULL_GOBS;
        double L2 = NULL_GOBS;
        double L3 = NULL_GOBS;

        C1 = obs_C(gC1);
        if (double_eq(C1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        C2 = obs_C(gC2);
        if (double_eq(C2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L2 = obs_L(gL2) / wlength2;
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L3 = obs_L(gL3) / wlength3;
        if (double_eq(L3, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double ewl = L2 - L3 - (C1 * (1 - alpha) + C2 * alpha) * (wlength3 - wlength2) / (wlength2 * wlength3);

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << " " << _satid << " " << gobs2str(L2.gobs()) << " " << gobs2str(C1.gobs()) << " " << gobs2str(L3.gobs()) << " " << gobs2str(C2.gobs()) << std::endl;
        std::cout << "extrawideL = " << extrawideL << std::endl;
        std::cout << "narrP = " << narrP << std::endl;
        std::cout << "EWL = " << lc << std::endl;
#endif
        return ewl;
    }
    double gnss_data_obs_manager::LW_meter(const gnss_data_obs &gL1, const gnss_data_obs &gL2) const
    {
        if (gsys() == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wlength1 = wavelength(gL1.band());
        double wlength2 = wavelength(gL2.band());

        double L1 = NULL_GOBS;
        double L2 = NULL_GOBS;

        L1 = obs_L(gL1) / wlength1;
        if (double_eq(L1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L2 = obs_L(gL2) / wlength2;
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double lw = (L1 - L2) / (1.0 / wlength1 - 1.0 / wlength2);

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << " " << _satid << " " << gobs2str(L2.gobs()) << " " << gobs2str(C1.gobs()) << " " << gobs2str(L3.gobs()) << " " << gobs2str(C2.gobs()) << std::endl;
        std::cout << "extrawideL = " << extrawideL << std::endl;
        std::cout << "narrP = " << narrP << std::endl;
        std::cout << "LW = " << lc << std::endl;
#endif
        return lw;
    }
    double gnss_data_obs_manager::LE_meter(const gnss_data_obs &gL2, const gnss_data_obs &gL3) const
    {
        if (gsys() == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wlength2 = wavelength(gL2.band());
        double wlength3 = wavelength(gL3.band());
        if (double_eq(wlength2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        if (double_eq(wlength3, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double L2 = NULL_GOBS;
        double L3 = NULL_GOBS;

        L2 = obs_L(gL2) / wlength2;
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L3 = obs_L(gL3) / wlength3;
        if (double_eq(L3, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double le = (L2 - L3) / (1.0 / wlength2 - 1.0 / wlength3);

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << " " << _satid << " " << gobs2str(L2.gobs()) << " " << gobs2str(C1.gobs()) << " " << gobs2str(L3.gobs()) << " " << gobs2str(C2.gobs()) << std::endl;
        std::cout << "extrawideL = " << extrawideL << std::endl;
        std::cout << "narrP = " << narrP << std::endl;
        std::cout << "LE = " << lc << std::endl;
#endif
        return le;
    }
    double gnss_data_obs_manager::LWL_factor13(const gnss_data_obs &gL1, const gnss_data_obs &gL3) const
    {
        if (gsys() == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double wlength1 = wavelength(gL1.band());
        double wlength3 = wavelength(gL3.band());
        if (double_eq(wlength1, NULL_GOBS) || double_eq(wlength3, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double lwl_factor13 = wlength1 / wlength3;

#ifdef DEBUG
        std::cout << _epoch.str_ymdhms() << " " << _satid << " " << gobs2str(L2.gobs()) << " " << gobs2str(C1.gobs()) << " " << gobs2str(L3.gobs()) << " " << gobs2str(C2.gobs()) << std::endl;
        std::cout << "extrawideL = " << extrawideL << std::endl;
        std::cout << "narrP = " << narrP << std::endl;
        std::cout << "LE = " << lc << std::endl;
#endif
        return lwl_factor13;
    }
    // get wide-lane combination for phase [m]!
    // ---------------------------------------
    double gnss_data_obs_manager::LNL(const gnss_data_obs &g1, const gnss_data_obs &g2) const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double lc = _lcf_phase(&g1, 2, &g2, -1); // still dual-frequency only

#ifdef DEBUG
        std::cout << " LNL = " << std::fixed << std::setprecision(3)
             << std::setw(16) << lc
             << std::endl;
#endif
        return lc;
    }

    // get wide-lane combination for code [m]! add by BoWong 11/11/2019
    // ---------------------------------------
    double gnss_data_obs_manager::PWL(const gnss_data_obs &g1, const gnss_data_obs &g2) const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double lc = _lcf_range(&g1, 1, &g2, 1); // still dual-frequency only

#ifdef DEBUG
        std::cout << " LWL = " << std::fixed << std::setprecision(3)
             << std::setw(16) << lc
             << std::endl;
#endif
        return lc;
    }

    // get wide-lane combination for phase [m]!
    // ---------------------------------------
    double gnss_data_obs_manager::LWL(const gnss_data_obs &g1, const gnss_data_obs &g2) const
    {
        if (_gsys == GLO && double_eq(_channel, DEF_CHANNEL))
        {
            return 0.0;
        }

        double lc = _lcf_phase(&g1, 1, &g2, -1); // still dual-frequency only

#ifdef DEBUG
        std::cout << " LWL = " << std::fixed << std::setprecision(3)
             << std::setw(16) << lc
             << std::endl;
#endif
        return lc;
    }

    // get mulitpath LC
    //---------------------
    double gnss_data_obs_manager::MP(const gnss_data_obs &code, const gnss_data_obs &L1, const gnss_data_obs &L2) const
    {
        if (!code.is_code())
            return NULL_GOBS;
        if (!L1.is_phase())
            return NULL_GOBS;
        if (!L2.is_phase())
            return NULL_GOBS;

        double coef1, coef2, coefC;
        double C = _obs_range(code);

        if (double_eq(C, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        _coef_multpath(code.band(), coefC, L1.band(), coef1, L2.band(), coef2);

        double L = _lc_phase(&L1, coef1, &L2, coef2);
        if (double_eq(L, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        std::string sat = this->sat();

        double lc = coefC * C + L;

#ifdef DEBUG
        std::cout << "MP code " << gobs2str(code.gobs())
             << std::fixed << std::setprecision(3)
             << ": coef1 = " << std::setw(7) << coef1
             << "     coef2 = " << std::setw(7) << coef2
             << "         C = " << std::setw(7) << C
             << "         L = " << std::setw(7) << L
             << "        lc = " << std::setw(7) << lc
             << std::endl;
#endif
        return lc;
    }

    double gnss_data_obs_manager::GFIF_meter(const gnss_data_obs &gL1, const gnss_data_obs &gL2, const gnss_data_obs &gL3) const
    {
        double frequency1 = frequency(gL1.band());
        double frequency2 = frequency(gL2.band());
        double frequency3 = frequency(gL3.band());
        if (double_eq(frequency1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        if (double_eq(frequency2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        if (double_eq(frequency3, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double coef_1 = frequency1 * frequency1 / (frequency1 * frequency1 - frequency2 * frequency2);
        double coef_2 = -frequency2 * frequency2 / (frequency1 * frequency1 - frequency2 * frequency2);
        double coef_3 = frequency1 * frequency1 / (frequency1 * frequency1 - frequency3 * frequency3);
        double coef_4 = -frequency3 * frequency3 / (frequency1 * frequency1 - frequency3 * frequency3);

        double L1 = NULL_GOBS;
        double L2 = NULL_GOBS;
        double L3 = NULL_GOBS;

        L1 = obs_L(gL1);
        if (double_eq(L1, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L2 = obs_L(gL2);
        if (double_eq(L2, NULL_GOBS))
        {
            return NULL_GOBS;
        }
        L3 = obs_L(gL3);
        if (double_eq(L3, NULL_GOBS))
        {
            return NULL_GOBS;
        }

        double GFIF_obs = (coef_1 * L1 + coef_2 * L2) - (coef_3 * L1 + coef_4 * L3);
        return GFIF_obs;
    }

    // valid
    // ----------
    bool gnss_data_obs_manager::valid() const
    {
        bool tmp = this->_valid();
        return tmp;
    }

    bool gnss_data_obs_manager::obs_empty() const
    {
        return _gobs.empty();
    }

    // clean data
    // ----------
    void gnss_data_obs_manager::clear()
    {
        this->_clear();
        return;
    }

    // clean internal function
    // ----------
    void gnss_data_obs_manager::_clear()
    {
        _staid.clear();
        _epoch = FIRST_TIME;
    }

    // clean internal function
    // ----------
    bool gnss_data_obs_manager::_valid() const
    {
        if (_staid.empty() ||
            _staid == "" ||
            _epoch == FIRST_TIME)
            return false;

        return true;
    }

    // Band of code/phase
    // ------------------------------
    void gnss_data_obs_manager::nbands(std::pair<int, int> &nb)
    {
        for (int b = 1; b <= 8; b++)
        {
            if (_cod_id(b) != X)
                nb.first++;
            if (_pha_id(b) != X)
                nb.second++;
        }
    }

    // Public version of _freq_avail()
    // --------------------------------
    std::set<GOBSBAND> gnss_data_obs_manager::band_avail(bool phase) const
    {
        std::set<GOBSBAND> s_f;

        if (phase)
            s_f = _band_avail();
        else
            s_f = _band_avail_code();
        return s_f;
    }

    // Public version of _freq_avail()
    // --------------------------------
    std::set<GFRQ> gnss_data_obs_manager::freq_avail() const
    {
        std::set<GFRQ> s_f = _freq_avail();
        return s_f;
    }

    // find out wether GFRQ is included
    bool gnss_data_obs_manager::contain_freq(const FREQ_SEQ &freq) const
    {
        bool res = false;

        std::set<GFRQ> frqs = _freq_avail();
        for (auto it = frqs.begin(); it != frqs.end(); it++)
        {
            FREQ_SEQ fs = gnss_sys::gfrq2freq(_gsys, *it);
            if (fs == freq)
                res = true;
        }
        return res;
    }

    // Get available frequences
    // --------------------------------
    std::set<GFRQ> gnss_data_obs_manager::_freq_avail() const
    {
        std::set<GFRQ> s_f;
        for (std::map<GOBS, double>::const_iterator it = _gobs.begin(); it != _gobs.end(); it++)
        {
            gnss_data_obs gobs;
            gobs.gobs(it->first);
            if (double_eq(it->second, 0.0) || !gobs.is_phase())
                continue;
            else
            {
                s_f.insert(gnss_sys::band2gfrq(_gsys, gobs.band()));
            }
        }
        return s_f;
    }

    // Get available bands for phase
    // --------------------------------
    std::set<GOBSBAND> gnss_data_obs_manager::_band_avail() const
    {
        std::set<GOBSBAND> s_f;
        for (std::map<GOBS, double>::const_iterator it = _gobs.begin(); it != _gobs.end(); it++)
        {
            gnss_data_obs gobs;
            gobs.gobs(it->first);

            if (double_eq(it->second, 0.0) || !gobs.is_phase())
                continue;
            else
                s_f.insert(gobs.band());
        }
        return s_f;
    }

    // Get available bands for phase
    // --------------------------------
    std::set<GOBSBAND> gnss_data_obs_manager::_band_avail_code() const
    {
        std::set<GOBSBAND> s_f;
        for (std::map<GOBS, double>::const_iterator it = _gobs.begin(); it != _gobs.end(); it++)
        {
            gnss_data_obs gobs;
            gobs.gobs(it->first);

            if (double_eq(it->second, 0.0) || gobs.is_phase())
                continue;
            else
                s_f.insert(gobs.band());
        }
        return s_f;
    }

    // Validity test for GLONASS data
    // ---------------
    bool gnss_data_obs_manager::_valid_obs() const
    {
        if (_gsys == GLO && _channel == 255)
            return false;
        else
            return true;
    }

    // return value of general carrier-phase linear combination
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::_lc_range(const gnss_data_obs *g1, const double &coef1,
                                 const gnss_data_obs *g2, const double &coef2,
                                 const gnss_data_obs *g3, const double &coef3) const
    {
        double C1 = NULL_GOBS;
        double C2 = NULL_GOBS;
        double C3 = NULL_GOBS;

        if (g1 && coef1 != 0.0)
        {
            C1 = obs_C(*g1);
            if (double_eq(C1, NULL_GOBS))
                return NULL_GOBS;
        }
        if (g2 && coef2 != 0.0)
        {
            C2 = obs_C(*g2);
            if (double_eq(C2, NULL_GOBS))
                return NULL_GOBS;
        }
        if (g3 && coef3 != 0.0)
        {
            C3 = obs_C(*g3);
            if (double_eq(C3, NULL_GOBS))
                return NULL_GOBS;
        }

        if (fabs(C1 - C2) > 100)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Inconsistencies of the code observations " + _epoch.str_ymdhms());
            return 0.0;
        }

        std::string sat = this->sat();

        double lc = (coef1 * C1 + coef2 * C2 + coef3 * C3);

#ifdef DEBUG
        int b1, b2, b3;
        b1 = b2 = b3 = 0;
        if (g1)
            b1 = g1->band();
        if (g2)
            b2 = g2->band();
        if (g2)
            b2 = g2->band();
        std::cout << _satid << " " << _staid
             << " _lc_range BAND(" << b1 << "," << b2 << " ," << b3 << ") "
             << "  COEF(" << coef1 << "," << coef2 << "," << coef3 << ") "
             << std::fixed << std::setprecision(3)
             << std::setw(16) << C1 << std::setw(16) << C2 << std::setw(16) << C3
             << "  lc = " << std::setw(16) << lc << std::endl;
#endif
        return lc;
    }

    // return value of general carrier-phase linear combination
    // for 2 or 3 bands with given coefficients
    // ----------
    double gnss_data_obs_manager::_lc_phase(const gnss_data_obs *g1, const double &coef1,
                                 const gnss_data_obs *g2, const double &coef2,
                                 const gnss_data_obs *g3, const double &coef3) const
    {
        double L1 = NULL_GOBS;
        double L2 = NULL_GOBS;
        double L3 = NULL_GOBS;

        if (g1 && coef1 != 0.0)
        {
            L1 = obs_L(*g1);
            if (double_eq(L1, NULL_GOBS))
                return NULL_GOBS;
        }
        if (g2 && coef2 != 0.0)
        {
            L2 = obs_L(*g2);
            if (double_eq(L2, NULL_GOBS))
                return NULL_GOBS;
        }
        if (g3 && coef3 != 0.0)
        {
            L3 = obs_L(*g3);
            if (double_eq(L3, NULL_GOBS))
                return NULL_GOBS;
        }

        double lc = (coef1 * L1 + coef2 * L2 + coef3 * L3);

        if (double_eq(L1, 0.0) || double_eq(L2, 0.0))
            return 0.0;

#ifdef DEBUG
        int b1, b2, b3;
        b1 = b2 = b3 = 0;
        if (g1)
            b1 = g1->band();
        if (g2)
            b2 = g2->band();
        if (g2)
            b2 = g2->band();
        std::cout << _satid << " " << _staid
             << " _lc_phase BAND(" << b1 << "," << b2 << " ," << b3 << ") "
             << "  COEF(" << coef1 << "," << coef2 << "," << coef3 << ") "
             << std::fixed << std::setprecision(3)
             << std::setw(16) << L1 / wavelength(g1->band()) << std::setw(16) << L2 / wavelength(g2->band()) << std::setw(16) << L3
             << "  lc = " << std::setw(16) << lc << std::endl;
#endif
        return lc;
    }

    // get selected code (GOBS)
    // ----------
    GOBS gnss_data_obs_manager::_id_range(const GOBSBAND &b) const
    {
        std::map<GOBS, double>::const_iterator it = _gobs.begin();

        while (it != _gobs.end())
        {
            GOBS gobs = it->first;
            if (gobs_code(gobs))
            {
                GOBSBAND tmp = str2gobsband(gobs2str(gobs));
                if (tmp == b)
                    return gobs;
            }
            it++;
        }
        return X;
    }

    // get selected phase (GOBS)
    // ----------
    GOBS gnss_data_obs_manager::_id_phase(const GOBSBAND &b) const
    {
        std::map<GOBS, double>::const_iterator it = _gobs.begin();

        while (it != _gobs.end())
        {
            GOBS gobs = it->first;
            if (gobs_phase(gobs))
            {
                GOBSBAND tmp = str2gobsband(gobs2str(gobs));
                if (tmp == b)
                    return gobs;
            }
            it++;
        }
        return X;
    }

    GOBS gnss_data_obs_manager::_id_doppler(const GOBSBAND &b) const
    {
        std::map<GOBS, double>::const_iterator it = _gobs.begin();

        while (it != _gobs.end())
        {
            GOBS gobs = it->first;
            if (gobs_doppler(gobs))
            {
                GOBSBAND tmp = str2gobsband(gobs2str(gobs));
                if (tmp == b)
                    return gobs;
            }
            it++;
        }
        return X;
    }

    GOBS gnss_data_obs_manager::_id_snr(const GOBSBAND &b) const
    {
        std::map<GOBS, double>::const_iterator it = _gobs.begin();

        while (it != _gobs.end())
        {
            GOBS gobs = it->first;
            if (gobs_snr(gobs))
            {
                GOBSBAND tmp = str2gobsband(gobs2str(gobs));
                if (tmp == b)
                    return gobs;
            }
            it++;
        }
        return X;
    }

    // get selected code (GOBS)
    // ----------
    GOBS gnss_data_obs_manager::select_range(const GOBSBAND &band, const bool &isRawAll) const
    {
        // jdhuang : RAW all do not consider some type of obs
        // jdhuang : Optimize code efficiency

        int max = -1;
        std::map<GOBS, double>::const_iterator it = _gobs.begin();
        std::map<GOBS, double>::const_iterator it_find = _gobs.begin();
        while (it != _gobs.end())
        {
            GOBS gobs = it->first;

            if (gobs_code(gobs))
            {
                GOBSBAND tmp = str2gobsband(gobs2str(gobs));
                if (tmp == band)
                {
                    gnss_data_obs gobs_temp(gobs);
                    gobs_temp.gobs2to3(_gsys);
                    int loc = -1;

                    char attr_tmp = (gobs2str(gobs_temp.gobs()))[2];
                    try
                    {
                        if (!isRawAll)
                        {
                            loc = range_order_attr_cmb.at(_gsys).at(band).find(attr_tmp);
                        }
                        else
                        {
                            loc = range_order_attr_raw.at(_gsys).at(band).find(attr_tmp);
                        }
                    }
                    catch (...)
                    {
                        loc = -1;
                    }

                    if (loc > max)
                    {
                        it_find = it;
                        max = loc;
                    }
                }
            }
            it++;
        }

        if (max == -1)
        {
            return X;
        }
        else
        {
            return (it_find->first);
        }
    }

    // get selected phase (GOBS)
    // ----------
    GOBS gnss_data_obs_manager::select_phase(const GOBSBAND &band, const bool &isRawAll) const
    {
        // jdhuang : RAW all do not consider some type of obs
        // jdhuang : Optimize code efficiency

        int max = -1;
        std::map<GOBS, double>::const_iterator it = _gobs.begin();
        std::map<GOBS, double>::const_iterator it_find = _gobs.begin();
        while (it != _gobs.end())
        {
            GOBS gobs = it->first;
            if (gobs_phase(gobs))
            {
                GOBSBAND tmp = str2gobsband(gobs2str(gobs));
                if (tmp == band)
                {
                    gnss_data_obs gobs_temp(gobs);
                    gobs_temp.gobs2to3(_gsys);
                    int loc = -1;

                    char attr_tmp = (gobs2str(gobs_temp.gobs()))[2];
                    try
                    {
                        if (!isRawAll)
                        {
                            loc = phase_order_attr_cmb.at(_gsys).at(band).find(attr_tmp);
                        }
                        else
                        {
                            loc = phase_order_attr_raw.at(_gsys).at(band).find(attr_tmp);
                        }
                    }
                    catch (...)
                    {
                        loc = -1;
                    }

                    if (loc > max)
                    {
                        it_find = it;
                        max = loc;
                    }
                }
            }
            it++;
        }

        if (max == -1)
        {
            return X;
        }
        else
        {
            return (it_find->first);
        }
    }

    /*
    // get selected code (GOBS)
// ----------
GOBS gnss_data_obs_manager::select_range(GOBSBAND b, const bool& isRawAll) const
{
    // jdhuang : change, add an option for isRawAll
    std::map<GSYS, std::map<GOBSBAND, std::string> > _order_attr;
    if (!isRawAll)
    {
        _order_attr =
        {
            //{GPS,{{BAND_1,"CSLXPWYM"},{BAND_2,"CDLXPWYM"},{BAND_5,"IQX"}}},
            //{GAL,{{BAND_1,"ABCXZ"},{BAND_5,"IQX"},{BAND_7,"IQX"},{BAND_8,"IQX"},{BAND_6,"ABCXZ"}}},
            //{BDS,{{BAND_2,"QXI"},{BAND_7,"IQX"},{BAND_6,"IQX"},{BAND_5,"DPX"},{BAND_9,"DPZ"},{BAND_8,"DPX"},{BAND_1,"DPX"}}},
            //{GLO,{{BAND_1,"CP"},{BAND_2,"CP"}}}
            {GPS,{{BAND_1,"CPW"},{BAND_2,"CLXPW"},{BAND_5,"QX"}}},
            {GAL,{{BAND_1,"CX"},{BAND_5,"IQX"},  {BAND_7,"IQX"},{BAND_8,"IQX"},{BAND_6,"ABCXZ"}}},
            {BDS,{{BAND_2,"IQX"},{BAND_7,"IQX"},{BAND_6,"IQX"},{BAND_5,"DPX"},{BAND_9,"DPZ"},{BAND_8,"DPX"},{BAND_1,"DPX"}}},
            {GLO,{{BAND_1,"CP"},{BAND_2,"CP"}}},
            {QZS,{{BAND_1,"CSLX"},{BAND_2,"LX"},{BAND_5,"IQX"}}} //glfeng
        };
    }
    else
    {
        // jdhuang : add
        _order_attr =
        {
            {GPS,{{BAND_1,"CPW"},{BAND_2,"CLXPW"},{BAND_5,"QX"}}},
            {GAL,{{BAND_1,"CX"},{BAND_5,"QX"},  {BAND_7,"QX"},{BAND_8,"IQX"},{BAND_6,"ABCXZ"}}},
            {BDS,{{BAND_2,"IQX"},{BAND_7,"IQX"},{BAND_6,"IQX"},{BAND_5,"DPX"},{BAND_9,"DPZ"},{BAND_8,"DPX"},{BAND_1,"DPX"}}},
            {GLO,{{BAND_1,"CP"},{BAND_2,"CP"}}}
            //{GPS,{{BAND_1,"CSLXPWYM"},{BAND_2,"CDLXPWYM"},{BAND_5,"IQX"}}},
            //{GAL,{{BAND_1,"ABCXZ"},{BAND_5,"IQX"},{BAND_7,"IQX"},{BAND_8,"IQX"},{BAND_6,"ABCXZ"}}},
            //{BDS,{{BAND_2,"QXI"},{BAND_7,"IQX"},{BAND_6,"IQX"},{BAND_5,"DPX"},{BAND_9,"DPZ"},{BAND_8,"DPX"},{BAND_1,"DPX"}}},
            //{GLO,{{BAND_1,"CP"},{BAND_2,"CP"}}}
        };
    }

    int max = -1;
    GOBS gobs_ans;
    std::map<GOBS, double>::const_iterator it = _gobs.begin();
    while (it != _gobs.end())
    {
        GOBS gobs = it->first;
        if (gobs_code(gobs))
        {
            GOBSBAND tmp = str2gobsband(gobs2str(gobs));
            if (tmp == b)
            {
                gnss_data_obs gobs_temp(gobs); gobs_temp.gobs2to3(_gsys);
                int loc = _order_attr[_gsys][b].find(gobs2str(gobs_temp.gobs())[2]);
                if (loc > max)
                {
                    gobs_ans = gobs;
                    max = loc;
                }
            }
        }
        it++;
    }
    if (max == -1)
    {
        return X;
    }
    else
    {
        return gobs_ans;
    }
}

// get selected phase (GOBS)
// ----------
GOBS gnss_data_obs_manager::select_phase(GOBSBAND b, const bool& isRawAll) const
{
    std::map<GSYS, std::map<GOBSBAND, std::string> > _order_attr;

    if (!isRawAll)
    {
        _order_attr =
        {
            {GPS,{{BAND_1,"CSLXPWYM"},{BAND_2,"CDLXPWYM"},{BAND_5,"IQX"}}},
            {GAL,{{BAND_1,"ABCXZ"},{BAND_5,"IQX"},{BAND_7,"IQX"},{BAND_8,"IQX"},{BAND_6,"ABCXZ"}}},
            //LX changed for BDS3
            {BDS,{{BAND_2,"XIQ"},{BAND_7,"IQX"},{BAND_6,"IQX"},{BAND_5,"DPX"},{BAND_9,"DPZ"},{BAND_8,"DPX"},{BAND_1,"DPX"}}},
            {GLO,{{BAND_1,"PC"},{BAND_2,"CP"}}},  // fix bugs change Band_1 CP->PC  glfeng
            {QZS,{{BAND_1,"CSLX"},{BAND_2,"LX"},{BAND_5,"IQX"}}} //glfeng
        };
    }
    else
    {
        // jdhuang : RAW all do not consider some type of obs
        _order_attr =
        {
            {GPS,{{BAND_1,"CSLXPWYM"},{BAND_2,"CDLXPWYM"},{BAND_5,"IQX"}}},
            {GAL,{{BAND_1,"ABCXZ"},{BAND_5,"IQX"},{BAND_7,"IQX"},{BAND_8,"IQX"},{BAND_6,"ABCXZ"}}},
            {BDS,{{BAND_2,"QXI"},{BAND_7,"IQX"},{BAND_6,"IQX"},{BAND_5,"DPX"},{BAND_9,"DPZ"},{BAND_8,"DPX"},{BAND_1,"DPX"}}},
            {GLO,{{BAND_1,"PC"},{BAND_2,"CP"}}}  // fix bugs change Band_1 CP->PC  glfeng
        };
    }

    int max = -1;
    GOBS gobs_ans;
    std::map<GOBS, double>::const_iterator it = _gobs.begin();
    while (it != _gobs.end()) {
        GOBS gobs = it->first;
        if (gobs_phase(gobs)) {
            GOBSBAND tmp = str2gobsband(gobs2str(gobs));
            if (tmp == b) {
                gnss_data_obs gobs_temp(gobs); gobs_temp.gobs2to3(_gsys);
                int loc = _order_attr[_gsys][b].find(gobs2str(gobs_temp.gobs())[2]);
                if (loc > max) {
                    gobs_ans = gobs;
                    max = loc;
                }
            }
        }
        it++;
    }
    if (max == -1) {
        return X;
    }
    else {
        return gobs_ans;
    }
}
    */

    // get selected code (GOBS)
    // ----------
    GOBS gnss_data_obs_manager::_cod_id(const int &band) const // , const GOBS& obs)
    {
        size_t sz = sizeof(code_choise[band]) / sizeof(GOBS);
        std::map<GOBS, double>::const_iterator it; // = _gobs.find( obs );

        GOBS tmp = X; // obs;

        // choices
        for (size_t i = 0; i < sz; ++i)
        {
            tmp = code_choise[band][i];

            it = _gobs.find(tmp);
            if (tmp == X || it == _gobs.end())
                continue;

            if (!double_eq(it->second, NULL_GOBS))
                return tmp;
            //    { std::cout << " found  code ID: [" << band << "] " << gobs2str( tmp ) << std::endl; return tmp; }
        }

        return X;
    }

    // get selected phase (GOBS)
    // ----------
    GOBS gnss_data_obs_manager::_pha_id(const int &band) const // , const GOBS& obs)
    {
        size_t sz = sizeof(phase_choise[band]) / sizeof(GOBS);
        std::map<GOBS, double>::const_iterator it; // = _gobs.find( obs );

        GOBS tmp = X; // obs;

        // choices
        for (size_t i = 0; i < sz; ++i)
        {
            tmp = phase_choise[band][i];

            it = _gobs.find(tmp);
            if (tmp == X || it == _gobs.end())
                continue;

            if (!double_eq(it->second, NULL_GOBS))
                return tmp;
            //    { std::cout << " found phase ID: [" << band << "] " << gobs2str( tmp ) << std::endl; return tmp; }
        }

        return X;
    }

    // return coefficients (c1,c2) of the ionosphere-free linear combination
    // ----------
    int gnss_data_obs_manager::_coef_ionofree(const GOBSBAND &b1, double &c1,
                                   const GOBSBAND &b2, double &c2) const
    {
        if (b1 == b2)
        {
            c1 = c2 = 0.0;
            return -1;
        }

        double fac1 = this->frequency(b1);
        double fac2 = this->frequency(b2);

        c1 = fac1 * fac1 / (fac1 * fac1 - fac2 * fac2);
        c2 = -fac2 * fac2 / (fac1 * fac1 - fac2 * fac2);

#ifdef DEBUG
        std::cout << " coefs (D) = " << std::fixed << std::setprecision(3)
             << std::setw(15) << fac1
             << std::setw(15) << fac2
             << std::setw(15) << c1
             << std::setw(15) << c2
             << std::endl;
#endif

        return 0;
    }

    // return coefficients (c1,c2) of the ionosphere-free linear combination
    // ----------
    int gnss_data_obs_manager::coef_ionofree(const GOBSBAND &b1, double &c1,
                                  const GOBSBAND &b2, double &c2) const
    {
        return _coef_ionofree(b1, c1, b2, c2);
    }

    int gnss_data_obs_manager::coef_ionofree_phi(const GOBSBAND &b1, double &c1, const GOBSBAND &b2, double &c2) const
    {
        if (b1 == b2)
        {
            c1 = c2 = 0.0;
            return -1;
        }

        double fac1 = this->frequency(b1);
        double fac2 = this->frequency(b2);

        c1 = fac1 * fac1 / (fac1 * fac1 - fac2 * fac2);
        c2 = -fac1 * fac2 / (fac1 * fac1 - fac2 * fac2);

        return 0;
    }

    // return coefficients (c1,c2) of the geometry-free linear combination
    // ----------
    int gnss_data_obs_manager::_coef_geomfree(const GOBSBAND &b1, double &c1,
                                   const GOBSBAND &b2, double &c2) const
    {
        if (b1 == b2)
        {
            c1 = c2 = 0.0;
            return -1;
        }
        c1 = 1.0;
        c2 = -1.0;

        return 0;
    }

    // return coefficients (c1,c2) of the geometry-free linear combination
    // ----------
    int gnss_data_obs_manager::coef_geomfree(const GOBSBAND &b1, double &c1,
                                  const GOBSBAND &b2, double &c2) const
    {
        return _coef_geomfree(b1, c1, b2, c2);
    }

    // return coefficients (c1,c2) of the narrow-lane linear combination
    // ----------
    int gnss_data_obs_manager::_coef_narrlane(const GOBSBAND &b1, double &c1,
                                   const GOBSBAND &b2, double &c2) const
    {
        if (b1 == b2)
        {
            c1 = c2 = 0.0;
            return -1;
        }

        double fac1 = this->frequency(b1);
        double fac2 = this->frequency(b2);

        c1 = fac1 / (fac1 + fac2);
        c2 = fac2 / (fac1 + fac2);

        return 0;
    }

    // return coefficients (c1,c2) of the narrow-lane linear combination
    // ----------
    int gnss_data_obs_manager::coef_narrlane(const GOBSBAND &b1, double &c1,
                                  const GOBSBAND &b2, double &c2) const
    {
        return _coef_narrlane(b1, c1, b2, c2);
    }

    // return coefficients (c1,c2) of the wide-lane linear combination
    // ----------
    int gnss_data_obs_manager::_coef_widelane(const GOBSBAND &b1, double &c1,
                                   const GOBSBAND &b2, double &c2) const
    {
        if (b1 == b2)
        {
            c1 = c2 = 0.0;
            return -1;
        }

        double fac1 = this->frequency(b1);
        double fac2 = this->frequency(b2);

        c1 = fac1 / (fac1 - fac2);
        c2 = -fac2 / (fac1 - fac2);

        return 0;
    }

    // return coefficients (c1,c2) of the wide-lane linear combination
    // ----------
    int gnss_data_obs_manager::coef_widelane(const GOBSBAND &b1, double &c1,
                                  const GOBSBAND &b2, double &c2) const
    {
        return _coef_widelane(b1, c1, b2, c2);
    }

    // -------------------------------------------------------------------------------------------
    // gnss_data_obscmb class
    // -------------------------------------------------------------------------------------------

    // operator< for gnss_data_obscmb
    // --------------------------
    bool gnss_data_obscmb::operator<(const gnss_data_obscmb &t) const
    {
        return (this->first.type() < t.first.type() &&
                this->first.band() < t.first.band() &&
                this->first.attr() < t.first.attr());
    }

} // namespace
