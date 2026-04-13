#include "hwa_gnss_amb_fixchk.h"
#include "hwa_gnss_all_amb.h"
#include "hwa_set_gbase.h"
#include "hwa_gnss_coder_ambcon.h"
#include "hwa_set_out.h"
#include "hwa_base_file.h"

namespace hwa_gnss
{
    gnss_amb_fixchk::gnss_amb_fixchk()
    {
    }

    gnss_amb_fixchk::gnss_amb_fixchk(base_log spdlog, set_base *set)
    {
        // std::set spdlog
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }

        // std::set the setting pointer
        if (nullptr == set)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = set;
        }

        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "================> constructor of gnss_amb_fixchk");
    }

    gnss_amb_fixchk::~gnss_amb_fixchk()
    {
    }

    bool gnss_amb_fixchk::ProcessBatch()
    {
        std::string func_id = "ProcessBatch";
        if (!get_setting())
        {
            return false;
        }

        if (!this->_gset || !this->_grecover1 || !this->_grecover2 || !this->_gambcon)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, this->_class_id, func_id, "check the data");
            return false;
        }

        // int iepo = 0, mepo = 0, beg_epo = 0, nepo = 0;
        // double res1 = 0.0, res2 = 0.0, wgt = 0.0;
        base_time epo = _beg;
        int epo_beg = 0;
        // int epo_end = 0;
        std::set<std::string>::iterator site_beg = _sites.begin();
        std::set<std::string>::iterator site_end = _sites.end();
        std::set<std::string>::iterator sat_beg = _sats.begin();
        std::set<std::string>::iterator sat_end = _sats.end();
        hwa_map_SITE_EQU allrecover1 = _grecover1->get_map_site_equ();
        hwa_map_SITE_EQU allrecover2 = _grecover2->get_map_site_equ();
        int isnew = 0;
        for (std::set<std::string>::iterator itsite = site_beg; itsite != site_end; itsite++)
        {
            for (std::set<std::string>::iterator itsat = sat_beg; itsat != sat_end; itsat++)
            {
                epo = _beg;
                int mepo = 0;
                int res1 = 0;
                int res2 = 0;
                int wgt = 0;
                // int beg_epo = 0;
                while (epo <= _end)
                {
                    auto tmp1 = allrecover1[*itsite][*itsat][epo];
                    if (tmp1.size() != 0)
                        isnew = tmp1[0]->is_newamb;
                    else
                        isnew = 0;

                    if (isnew == 1 || epo == _end)
                    {
                        if (mepo > 0)
                        {
                            res1 = sqrt(res1 / wgt);
                            res2 = sqrt(res2 / wgt);
                            if (res2 > 10 && res2 / res1 > 1.7)
                            {
                                // epo_end = epo_beg + mepo - 1;
                                //std::cout << *itsite << "   " << *itsat << "   " << epo_beg << "   " << epo_end << "   " << res1 << "   " << res2 << std::endl;
                                for (int i = 0; i < mepo; i++)
                                    _flag[*itsite][*itsat][epo_beg + i] = 1;
                            }
                            mepo = 0;
                            res1 = 0;
                            res2 = 0;
                            wgt = 0;
                        }
                        epo_beg = (epo - _beg) / _intv + 1;
                    }

                    tmp1 = allrecover1[*itsite][*itsat][epo];
                    auto tmp2 = allrecover2[*itsite][*itsat][epo];
                    if (tmp1.size() == 0 || tmp2.size() == 0)
                    {
                        epo = epo + _intv;
                        continue;
                    }
                    /*if (mepo == 0)
                    {
                        epo_beg = epo;
                    }*/
                    mepo++;
                    auto iter1 = tmp1[0];
                    res1 += pow(iter1->resuidal * 1000, 2) * iter1->weight;
                    wgt += iter1->weight;
                    auto iter2 = tmp2[0];
                    res2 += pow(iter2->resuidal * 1000, 2) * iter2->weight;
                    epo = epo + _intv;
                }
            }
        }

        // todo: test this function
        write_ambcon();
        return true;
    }

    bool gnss_amb_fixchk::get_setting()
    {
        _beg = dynamic_cast<set_gen *>(this->_gset)->beg();
        _end = dynamic_cast<set_gen *>(this->_gset)->end();
        _bias = 100;
        _sites = dynamic_cast<set_gen *>(this->_gset)->recs();
        _sats = dynamic_cast<set_gnss *>(this->_gset)->sat();
        base_time beg_ambcon;
        std::set<std::string> sagnss_coder_ambcon, site_ambcon, sats, sites;
        _num_all = _gambcon->num_all();
        _num_fixed = _gambcon->num_fixed();
        sagnss_coder_ambcon = _gambcon->sat_list();
        site_ambcon = _gambcon->rec_list();

        std::set<std::string>::const_iterator site_beg = _sites.begin();
        std::set<std::string>::const_iterator site_end = _sites.end();
        std::set<std::string>::const_iterator site2_beg = site_ambcon.begin();
        std::set<std::string>::const_iterator site2_end = site_ambcon.end();
        set_intersection(site_beg, site_end, site2_beg, site2_end,
                         inserter(sites, sites.begin()));
        _sites = sites;

        std::set<std::string>::const_iterator sat_beg = _sats.begin();
        std::set<std::string>::const_iterator sat_end = _sats.end();
        std::set<std::string>::const_iterator sat2_beg = sagnss_coder_ambcon.begin();
        std::set<std::string>::const_iterator sat2_end = sagnss_coder_ambcon.end();
        set_intersection(sat_beg, sat_end, sat2_beg, sat2_end,
                         inserter(sats, sats.begin()));
        _sats = sats;

        if (_sites.size() == 0 || _sats.size() == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "need site or _sat1!");
            return false;
        }
        return true;
    }

    bool gnss_amb_fixchk::add_data(gnss_all_recover *res1, gnss_all_recover *res2, gnss_data_ambcon *ambcon)
    {
        if (!res1 && !res2 && !ambcon)
            return false;
        try
        {
            _grecover1 = res1;
            _grecover2 = res2;
            _gambcon = ambcon;
        }
        catch (...)
        {
            return false;
        }

        return true;
    }

    void gnss_amb_fixchk::write_ambcon()
    {

        gnss_data_ambcon _gambcon_save(_spdlog);
        base_time epo = _beg;

        while (epo <= _end)
        {
            auto vambs = _gambcon->get_amb(epo);
            if (vambs.empty())
            {
                epo = epo + _intv;
                continue;
            }
            for (const auto &iter : vambs)
            {
                gnss_amb Sd_amb = *iter;
                const base_time &beg_time = Sd_amb.t_beg();
                const base_time &end_time = Sd_amb.t_end();
                std::string rec = Sd_amb.recs().first;
                std::string sat_1st = Sd_amb.sats().first;
                std::string sat_2nd = Sd_amb.sats().second;
                int beg = (beg_time - _beg) / _intv + 1;
                // int end = (end_time - _beg) / _intv + 1;
                if (_flag[rec][sat_1st][beg] == 1 || _flag[rec][sat_2nd][beg] == 1)
                {
                    //std::cout << sat_1st << "  " << sat_2nd <<"  "<<_beg<< std::endl;
                    //_ambnum--;
                    continue;
                }
                _gambcon_save.add_amb(Sd_amb);
            }
            epo = epo + _intv;
        }
        _gambcon_save.set_sat_rec(_sats, _sites);
        _gambcon_save.set_time(_beg, _end);
        //_gambcon_save.set_amb_num(_ambnum);
        base_file gout(_spdlog);
        gnss_coder_ambcon gamb_coder(_gset, "", 4096);
        std::string path = (dynamic_cast<set_out *>(this->_gset)->outputs("ambiguity_leo"));
        if (path.empty())
        {
            path = "grt$(date).con";
        }
        base_type_conv::substitute(path, "$(date)", _beg.str_yyyydoy(), false);
        gout.path(path);
        gout.spdlog(this->_spdlog);
        gamb_coder.clear();
        gamb_coder.spdlog(this->_spdlog);
        gamb_coder.path(path);
        gamb_coder.add_data("ID1", &_gambcon_save);
        gout.coder(&gamb_coder);
        gout.run_write();
    }
}
