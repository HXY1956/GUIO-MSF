#include <algorithm>
#include <iomanip>
#include <memory>
#include "hwa_gnss_convobs.h"

namespace hwa_gnss
{
    gnss_base_convobs::gnss_base_convobs(set_base *settings, base_log spdlog,  base_all_proc *data) : _set(settings), _spdlog(spdlog), _data(data)
    {
        std::set<std::string> sys_list = dynamic_cast<set_gen *>(_set)->sys();
        for (const auto &sys : sys_list)
        {
            GSYS gsys = gnss_sys::str2gsys(sys);
            _band_index[gsys] = dynamic_cast<set_gnss *>(_set)->band_index(gsys);
        }
        _site_list = dynamic_cast<set_gen *>(_set)->recs();
        // init data
        _allobs = dynamic_cast<gnss_all_obs *>((*_data)[base_data::ALLOBS]);
        _allambflag = dynamic_cast<gnss_all_ambflag *>((*_data)[base_data::AMBFLAG]);
        _allambflag13 = dynamic_cast<gnss_all_ambflag *>((*_data)[base_data::AMBFLAG13]);
    }

    bool gnss_base_convobs::ProcessBatch(std::string site, const base_time &begT, const base_time &endT, double sampling)
    {
        std::string lower_site(site);
        transform(site.begin(), site.end(), lower_site.begin(), ::tolower);
        bool apply13 = true;
        if (_allambflag->getAllAmbFlag().find(lower_site) == _allambflag->getAllAmbFlag().end())
            return false;
        if (_allambflag13)
        {
            if (_allambflag13->getAllAmbFlag().find(lower_site) == _allambflag13->getAllAmbFlag().end())
                apply13 = false;
        }
        else
        {
            apply13 = false;
        }
        gnss_data_ambflag ambflag_site = _allambflag->getOneAmbFlag(lower_site);
        gnss_data_ambflag ambflag13_site;
        if (apply13)
            ambflag13_site = _allambflag13->getOneAmbFlag(lower_site);
        base_time crt_time = begT;
        while (crt_time <= endT)
        {
            std::vector<std::shared_ptr<gnss_data_obs_manager>> epoData = _allobs->obs_pt(site, crt_time);
            for (auto it = epoData.begin(); it != epoData.end(); it++)
            {
                const std::string &obs_sat = (*it)->sat();
                auto sys = gnss_sys::sat2gsys(obs_sat);
                GOBSBAND b1 = _band_index[sys][FREQ_1];
                GOBSBAND b2 = _band_index[sys][FREQ_2];
                GOBSBAND b3 = _band_index[sys][FREQ_3];
                double c1 = 0, c2 = 0, c3 = 0, c4 = 0;
                bool valid_12 = ambflag_site.is_carrier_range(obs_sat, crt_time, c1, c2);
                bool valid_13 = false;
                if (apply13)
                    valid_13 = ambflag13_site.is_carrier_range(obs_sat, crt_time, c4, c3);
                if (!(valid_12 || valid_13))
                {
                    _allobs->erase(site, crt_time, obs_sat);
                    continue;
                }
                if (valid_12 && valid_13 && c1 != c4)
                {
                    _allobs->erase(site, crt_time, obs_sat);
                    continue;
                }
                if (valid_12)
                {
                    (*it)->mod_L(-1.0 * c1, b1, 0); // modify L1 by dL [cyc]
                    (*it)->mod_L(-1.0 * c2, b2, 0); // modify L2 by dL [cyc]
                    if (valid_13)
                        (*it)->mod_L(-1.0 * c3, b3, 0); // modify L3 by dL [cyc]
                    else
                        (*it)->mod_L(0, b3, 2); // std::set L3 to zero
                }
                else
                {
                    (*it)->mod_L(-1.0 * c4, b1, 0); // modify L1 by dL [cyc]
                    (*it)->mod_L(-1.0 * c3, b3, 0); // modify L3 by dL [cyc]
                    (*it)->mod_L(0, b2, 2);         // std::set L2 to zero
                }
            }

            // Next
            crt_time = crt_time + sampling;
        }
        return true;
    }

    bool gnss_base_convobs::GenerateProduct(std::string site, base_time begT)
    {
        std::string obspath = dynamic_cast<set_out *>(_set)->outputs("obs_dir");
        if (obspath.empty())
            obspath = "file://obs_fix";
        if (ACCESS(obspath.substr(7).c_str(), 0) != 0)
        {
            MKDIR(obspath.substr(7).c_str());
        }

        std::string site_tmp = site;
        transform(site_tmp.begin(), site_tmp.end(), site_tmp.begin(), ::tolower);

        std::stringstream obs_file;
        obs_file << obspath << PATH_SEPARATOR << site_tmp << std::setw(3) << std::setfill('0') << begT.doy() << "0."
                 << std::setw(2) << std::setfill('0') << begT.yr() << "o";

        if (ACCESS(obs_file.str().c_str(), 0) == 0)
        {
            std::string obs_file_orig = obs_file.str() + ".orig";
            int isok = rename(obs_file.str().c_str(), obs_file_orig.c_str());
            std::cout << site << "  " << isok << std::endl;
        }

        base_io *gout = new base_file(_spdlog);
        //std::set I/O
        gout->path(obs_file.str());
        gout->spdlog(_spdlog);
        //std::set coder
        base_coder *gcoder = new gnss_coder_rinexo(nullptr, "", 4096);
        dynamic_cast<gnss_coder_rinexo *>(gcoder)->setSite(site);
        gcoder->clear();
        gcoder->spdlog(_spdlog);
        gcoder->path(obs_file.str());
        gcoder->add_data("IDX", _allobs);
        gcoder->add_data("OBJ", _allobj);
        gout->coder(gcoder);
        // write
        gout->run_write();
        delete gout;
        gout = nullptr;
        delete gcoder;
        gcoder = nullptr;
        return true;
    }

} // namespace
