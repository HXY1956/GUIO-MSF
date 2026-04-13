#include "hwa_gnss_proc_updateparif.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    gnss_proc_update_par_if::gnss_proc_update_par_if(std::shared_ptr<gnss_data_cycleslip> cycleslip, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list)
    {
        _cycleslip = cycleslip;
        _band_list = band_list;
    }

    gnss_proc_update_par_if::~gnss_proc_update_par_if()
    {
    }

    void gnss_proc_update_par_if::_update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info)
    {
        if (_lite_update_amb)
        {
            _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB_IF, update_info);
        }
        else
        {
            _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB_IF, _cycleslip.get(), update_info);
        }
    }

    void gnss_proc_update_par_if::_update_one_type_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, par_type ambtype, gnss_data_cycleslip *_slip, gnss_proc_update_par_info &update_info)
    {

        std::map<std::string, std::set<std::string>> mapPrn;
        int num = 0;
        for (auto &obs_data : obsdata)
        {
            std::string rec = obs_data.site();
            std::string sat = obs_data.sat();
            mapPrn[rec].insert(sat);

            int idx_amb = allpars.getParam(rec, ambtype, sat);

            // judge cycle  and else
            base_time crt_epoch = epoch;
            // first add new amb
            // second update old amb if cycleslip
            if ((idx_amb < 0 && _slip->num_of_amb_arc(rec, sat, crt_epoch)) ||
                (idx_amb >= 0 && _slip->cycle_slip(obs_data, crt_epoch)))
            {
                int idx_max = allpars[allpars.parNumber() - 1].index;
                base_par par_newamb = base_par(rec, ambtype, idx_max + 1 + num, sat);

                int amb_flag = _slip->num_of_amb_arc(rec, sat, epoch);
                _slip->set_amb_flag(rec, sat, amb_flag);
                auto tb_slip = dynamic_cast<gnss_data_turboedit *>(_slip);

                base_time end_time = LAST_TIME;
                if (tb_slip)
                {
                    end_time = tb_slip->get_crt_amb_end(rec, sat);
                }
                num++;
                par_newamb.value(0.0);
                par_newamb.setTime(epoch, end_time);
                par_newamb.apriori(10000.0); //1122 xjhan

                // add new amb
                if (idx_amb < 0)
                {
                    update_info.add(par_newamb);
                }
                // reset the new amb
                else
                {
                    allpars[idx_amb].end = epoch - _intv;
                    update_info.add(idx_amb + 1, par_newamb);
                }
            }
        }

        // delete old amb
        for (unsigned int i = 0; i < allpars.parNumber(); i++)
        {
            if (allpars[i].parType == ambtype)
            {
                if (allpars[i].end < epoch)
                {
                    if (!update_info.exist(i + 1))
                    {
                        allpars[i].end = epoch - _intv; // glfeng 2018.5.1
                        update_info.add(i + 1);
                    }
                }
                else if (mapPrn[allpars[i].site].count(allpars[i].prn) == 0)
                {
                    if (!update_info.exist(i + 1))
                    {
                        allpars[i].end = epoch - _intv; // glfeng 2018.5.1
                        update_info.add(i + 1);
                    }
                }
                else
                {
                    continue;
                }
            }
        }
    }

    void gnss_proc_update_par_if::_update_one_type_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, par_type ambtype, gnss_proc_update_par_info &update_info)
    {
        hwa_map_band vec_band = gnss_band_sorted();

        for (auto obs_data : obsdata)
        {
            std::string rec = obs_data.site();
            std::string sat = obs_data.sat();
            GSYS sys = obs_data.gsys();

            int idx_amb = allpars.getParam(rec, ambtype, sat);
            GOBS obsL1, obsL2;
            if (ambtype == par_type::AMB_IF)
            {
                obsL1 = obs_data.select_phase(vec_band[sys][1]);
                obsL2 = obs_data.select_phase(vec_band[sys][2]);
            }
            else if (ambtype == par_type::AMB13_IF)
            {
                obsL1 = obs_data.select_phase(vec_band[sys][1]);
                obsL2 = obs_data.select_phase(vec_band[sys][3]);
            }
            else if (ambtype == par_type::AMB14_IF)
            {
                obsL1 = obs_data.select_phase(vec_band[sys][1]);
                obsL2 = obs_data.select_phase(vec_band[sys][4]);
            }
            else if (ambtype == par_type::AMB15_IF)
            {
                obsL1 = obs_data.select_phase(vec_band[sys][1]);
                obsL2 = obs_data.select_phase(vec_band[sys][5]);
            }
            else
            {
                return;
            }

            if (idx_amb < 0)
            {
                if (obsL1 != GOBS::X && obsL2 != GOBS::X)
                {
                    int idx_max = allpars[allpars.parNumber() - 1].index;
                    base_par par_newamb = base_par(rec, ambtype, idx_max + 1, sat);

                    par_newamb.value(0.0);
                    par_newamb.setTime(epoch, epoch);
                    par_newamb.apriori(9000.0);
                    update_info.add(par_newamb);
                }
            }
            else
            {
                if (obsL1 != GOBS::X && obsL2 != GOBS::X)
                {
                    if (obs_data.getlli(obsL1) < 1 && obs_data.getlli(obsL2) < 1)
                    {
                        allpars[idx_amb].end = epoch;
                    }
                    else
                    {
                        allpars[idx_amb].end = epoch - _intv;

                        int idx_max = allpars[allpars.parNumber() - 1].index;
                        base_par par_newamb = base_par(rec, ambtype, idx_max + 1, sat);

                        par_newamb.value(0.0);
                        par_newamb.setTime(epoch, epoch);
                        par_newamb.apriori(9000.0);

                        update_info.add(idx_amb + 1, par_newamb);
                    }
                }
                else // obsL1==X || obsL2==X
                {
                    update_info.add(idx_amb + 1);
                }
            }
        }

        // delete old amb
        for (unsigned int i = 0; i < allpars.parNumber(); i++)
        {
            if (allpars[i].parType == ambtype)
            {
                if (allpars[i].end < epoch)
                {
                    if (!update_info.exist(i + 1))
                    {
                        update_info.add(i + 1);
                    }
                }
                else if (allpars[i].amb_ini == true && allpars[i].beg + -1 * _intv < epoch)
                {
                    base_par newamb = allpars[i];
                    newamb.beg = epoch;
                    if (!update_info.exist(i + 1))
                    {
                        update_info.add(i + 1, newamb);
                    }
                }
            }
        }
    }

    gnss_proc_update_par_if_1X::gnss_proc_update_par_if_1X(std::shared_ptr<gnss_data_cycleslip> cycleslip12, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list) : gnss_proc_update_par_if(cycleslip12, band_list)
    {
    }

    gnss_proc_update_par_if_1X::gnss_proc_update_par_if_1X(std::shared_ptr<gnss_data_cycleslip> cycleslip12, std::shared_ptr<gnss_data_cycleslip> cycleslip13, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list) : gnss_proc_update_par_if(cycleslip12, band_list)
    {
        _cycleslip13 = cycleslip13;
    }

    gnss_proc_update_par_if_1X::~gnss_proc_update_par_if_1X()
    {
    }

    void gnss_proc_update_par_if_1X::set_cycleslip13(std::shared_ptr<gnss_data_cycleslip> cycleslip13)
    {
        _cycleslip13 = cycleslip13;
    }

    void gnss_proc_update_par_if_1X::set_cycleslip14(std::shared_ptr<gnss_data_cycleslip> cycleslip14)
    {
        _cycleslip14 = cycleslip14;
    }

    void gnss_proc_update_par_if_1X::set_cycleslip15(std::shared_ptr<gnss_data_cycleslip> cycleslip15)
    {
        _cycleslip15 = cycleslip15;
    }

    gnss_proc_update_par_info gnss_proc_update_par_if_1X::get_all_update_parameters(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata)
    {
        gnss_proc_update_par_info update_info;
        // update AMB
        gnss_proc_update_par_if_1X::_update_amb_pars(epoch, allpars, obsdata, update_info);

        // update gps rec ifb pars
        if (allpars.orbParNumber() > 0)
        {
            _udpate_gps_rec_ifb_pars(epoch, allpars, obsdata, update_info);
        }

        // update Other pars
        _update_process_pars(epoch, allpars, update_info);

        return update_info;
    }

    void gnss_proc_update_par_if_1X::_update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info)
    {
        gnss_proc_update_par_if::_update_amb_pars(epoch, allpars, obsdata, update_info);
        if (_lite_update_amb)
        {
            _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB13_IF, update_info);
        }
        else
        {
            if (_cycleslip13 != nullptr)
                _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB13_IF, _cycleslip13.get(), update_info);
            if (_cycleslip14 != nullptr)
                _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB14_IF, _cycleslip14.get(), update_info);
            if (_cycleslip15 != nullptr)
                _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB15_IF, _cycleslip15.get(), update_info);
        }
    }

    t_updateparWL::t_updateparWL(std::shared_ptr<gnss_data_cycleslip> cycleslip, const std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> &band_list) : gnss_proc_update_par_if(cycleslip, band_list)
    {
    }

    t_updateparWL::~t_updateparWL()
    {
    }
    void t_updateparWL::_update_amb_pars(const base_time &epoch, base_allpar &allpars, const std::vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info)
    {
        _update_one_type_amb_pars(epoch, allpars, obsdata, par_type::AMB_WL, _cycleslip.get(), update_info);
    }
}