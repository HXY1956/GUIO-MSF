#include "hwa_gnss_all_recover.h"
#include "hwa_base_globaltrans.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    gnss_all_recover::gnss_all_recover() : base_data()
    {
        _type = base_data::ALLRECOVER;
    }

    gnss_all_recover::gnss_all_recover(base_log spdlog) : base_data(spdlog)
    {
        _type = base_data::ALLRECOVER;
    }
    gnss_all_recover::~gnss_all_recover()
    {
        for (gnss_data_recover *data : _recover_data)
        {
            delete data;
            data = nullptr;
        }
        _time_equmap.clear();
        _time_parmap.clear();
        _site_sat_time_equmap.clear();
        _type_parmap.clear();
    }

    void gnss_all_recover::add_recover_equation(const gnss_data_recover_equation &recover_equ)
    {
        gnss_data_recover_equation *equ_data = new gnss_data_recover_equation(recover_equ);

        _add_common_data(equ_data);

        _recover_head.site_list.insert(recover_equ.site_name);
        _recover_head.sat_list.insert(recover_equ.sat_name);

        // default set && add
        _time_equmap.insert(std::make_pair(recover_equ.time, vector<gnss_data_recover_equation *>()));

        // add equ map
        _time_equmap[recover_equ.time].push_back(equ_data);
        _site_sat_time_equmap.insert(std::make_pair(recover_equ.site_name, hwa_map_SAT_EQU()));

        hwa_map_SAT_EQU &temp_sat_equ = _site_sat_time_equmap[recover_equ.site_name];
        temp_sat_equ.insert(std::make_pair(recover_equ.sat_name, hwa_map_TIME_EQU()));

        hwa_map_TIME_EQU &temp_time_equ = temp_sat_equ[recover_equ.sat_name];
        temp_time_equ.insert(std::make_pair(recover_equ.time, vector<gnss_data_recover_equation *>()));

        temp_time_equ[recover_equ.time].push_back(equ_data);
    }

    void gnss_all_recover::add_recover_par(const gnss_data_recover_par &recover_par)
    {
        gnss_data_recover_par *par_data = new gnss_data_recover_par(recover_par);

        _add_common_data(par_data);
        _time_parmap.insert(std::make_pair(recover_par.get_recover_time(), vector<gnss_data_recover_par *>()));
        _time_parmap_pce.insert(std::make_pair(recover_par.get_recover_time(), vector<gnss_data_recover_par>()));
        _time_parmap[recover_par.get_recover_time()].push_back(par_data);
        _time_parmap_pce[recover_par.get_recover_time()].push_back(*par_data);

        _type_parmap.insert(std::make_pair(recover_par.par.parType, vector<gnss_data_recover_par *>()));
        _type_parmap[recover_par.par.parType].push_back(par_data);
    }

    void gnss_all_recover::get_clkdata(gnss_all_prec &clkdata, gnss_all_prec::clk_type type)
    {
        set<gnss_all_prec::clk_type> type_list;
        if (type != gnss_all_prec::UNDEF)
        {
            type_list.insert(type);
        }
        else
        {
            type_list.insert(gnss_all_prec::AR);
            type_list.insert(gnss_all_prec::AS);
        }

        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto par : iter->second)
            {
                string obj = "";
                if (par->par.parType == par_type::CLK && type_list.find(gnss_all_prec::AR) != type_list.end())
                {
                    obj = par->par.site;
                }
                else if (par->par.parType == par_type::CLK_SAT && type_list.find(gnss_all_prec::AS) != type_list.end())
                {
                    obj = par->par.prn;
                }
                else
                {
                    continue;
                }

                // jdhuang
                if (par->correct_value == 0.0)
                {
                    continue;
                }

                double clk[3] = {0.0, 0.0, 0.0};
                double var[3] = {0.0, 0.0, 0.0};

                clk[0] = (par->par.value() + par->correct_value) / CLIGHT;
                clkdata.addclk(obj, epoch, clk, var);
            }
        }
    }

    /*void gnss_all_recover::get_rbdata(t_gallrb& rbdata)
     {

          for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
          {
               base_time epoch = iter->first;
               for (auto par : iter->second) {
                    string obj = "";
                    string obj_sat = "";
                    if (par->par.parType == par_type::RB) {
                         obj = par->par.site;
                         obj_sat = par->par.prn;
                    }
                    else {
                         continue;
                    }

                    double rb = 0.0;

                    rb = par->par.value() + par->correct_value;

                    rbdata.addrb(obj, obj_sat, rb);
               }
          }
     }*/

    //void gnss_all_recover::get_eopdata(t_galleop& eopdata)
    //{
    //     map<base_time, eop> eop_all;
    //     eop eop_one;
    //     int tab = 0;

    //     for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
    //     {
    //          base_time epoch = iter->first;
    //          for (auto par : iter->second) {
    //               if (par->par.parType == par_type::XPOLE) {
    //                    //eop_one.xpole = par->par.value() + par->correct_value;
    //                    eop_one.xpole = par->correct_value;
    //                    tab = 1;
    //               }
    //               else if (par->par.parType == par_type::YPOLE) {
    //                    //eop_one.ypole = par->par.value() + par->correct_value;
    //                    eop_one.ypole = par->correct_value;
    //               }
    //               else if (par->par.parType == par_type::DXPOLE) {
    //                    //eop_one.dxpole = par->par.value() + par->correct_value;
    //                    eop_one.dxpole = par->correct_value;
    //               }
    //               else if (par->par.parType == par_type::DYPOLE) {
    //                    //eop_one.dypole = par->par.value() + par->correct_value;
    //                    eop_one.dypole = par->correct_value;
    //               }
    //               else if (par->par.parType == par_type::UT1) {
    //                    //eop_one.ut1 = par->par.value() + par->correct_value;
    //                    eop_one.ut1 = par->correct_value;
    //               }
    //               else if (par->par.parType == par_type::DUT1) {
    //                    //eop_one.dut1 = par->par.value() + par->correct_value;
    //                    eop_one.dut1 = par->correct_value;
    //               }
    //               else {
    //                    continue;
    //               }

    //          }
    //          if (tab == 1)
    //          {
    //               eop_all[iter->first] = eop_one;
    //               eop_one.clear_value();
    //               tab = 0;
    //          }

    //     }

    //     for (auto iter = eop_all.begin(); iter != eop_all.end(); iter++)
    //     {
    //          eopdata.addeop(iter->first, iter->second);
    //     }

    //}

    void gnss_all_recover::get_poledata(gnss_data_poleut &poledata)
    {
        map<string, double> pole_one;
        int tab = 0;

        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto par : iter->second)
            {
                if (par->par.parType == par_type::XPOLE)
                {
                    //eop_one.xpole = par->par.value() + par->correct_value;
                    pole_one["XPOLE"] = par->correct_value + par->par.value();
                    tab = 1;
                }
                else if (par->par.parType == par_type::YPOLE)
                {
                    //eop_one.ypole = par->par.value() + par->correct_value;
                    pole_one["YPOLE"] = par->correct_value + par->par.value();
                }
                else if (par->par.parType == par_type::DXPOLE)
                {
                    //eop_one.dxpole = par->par.value() + par->correct_value;
                    pole_one["DXPOLE"] = par->correct_value + par->par.value();
                }
                else if (par->par.parType == par_type::DYPOLE)
                {
                    //eop_one.dypole = par->par.value() + par->correct_value;
                    pole_one["DYPOLE"] = par->correct_value + par->par.value();
                }
                else if (par->par.parType == par_type::UT1)
                {
                    //eop_one.ut1 = par->par.value() + par->correct_value;
                    pole_one["UT1"] = par->correct_value + par->par.value();
                }
                else if (par->par.parType == par_type::DUT1)
                {
                    //eop_one.dut1 = par->par.value() + par->correct_value;
                    pole_one["DUT1"] = par->correct_value + par->par.value();
                }
                else
                {
                    continue;
                }
            }
            if (tab == 1)
            {
                poledata.setEopData(iter->first, pole_one, "UT1R", 1.0);
                pole_one.clear();
                tab = 0;
            }
        }
    }

    void gnss_all_recover::get_pcvdata(gnss_all_pcv &pcvdata, gnss_all_obj &allobj)
    {
        shared_ptr<gnss_data_pcv> pcv = std::make_shared<gnss_data_pcv>(_spdlog);
        map<double, double> zen_map;
        map<double, map<double, double>> azi_map;
        Triple pco(0.0, 0.0, 0.0);
        base_par par;
        string sat, site;
        vector<GFRQ> fqs;
        int npar = 0;
        int ipar = 0;
        bool lzero = false;
        shared_ptr<gnss_data_obj> obj;
        shared_ptr<gnss_data_pcv> obj_pcv;
        map<GFRQ, Triple> pco_map;
        base_time t_beg = _recover_head.get_beg_time();
        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto pars : iter->second)
            {
                par = pars->par;
                if (ipar < 0)
                {
                    ipar++;
                    if (sat != par.prn || site != par.site || par.parType < par_type::PCV_SAT || par.parType > par_type::PCO_REC_Z)
                    {
                        pcvdata.addpcv(pcv);
                        pcv = std::make_shared<gnss_data_pcv>(_spdlog);
                    }
                    else
                    {
                        fqs = par.fq;
                    }
                }
                if (par.parType < par_type::PCV_SAT || par.parType > par_type::PCO_REC_Z)
                {
                    continue;
                }
                if (ipar == 0)
                {
                    site = par.site;
                    sat = par.prn;
                    fqs = par.fq;
                    npar = par.nazi * par.nzen;
                    if (par.parType >= par_type::PCV_SAT && par.parType <= par_type::PCO_SAT_Z)
                    {
                        obj = allobj.obj(sat);
                        obj_pcv = (obj != nullptr) ? obj->pcv(t_beg) : nullptr;
                    }
                    else
                    {
                        obj = allobj.obj(site);
                        obj_pcv = (obj != nullptr) ? obj->pcv(t_beg) : nullptr;
                    }
                    pcv->beg(obj_pcv->beg());
                    pcv->end(obj_pcv->end());
                    pcv->anten(obj_pcv->anten());
                    pcv->ident(obj_pcv->ident());
                    pcv->svcod(obj_pcv->svcod());
                    pcv->source(obj_pcv->source());
                    pcv->method(obj_pcv->method());
                    pcv->snxcod(obj_pcv->snxcod());
                    pcv->zen1(obj_pcv->zen1());
                    pcv->dazi(obj_pcv->dazi());
                    pcv->dzen(obj_pcv->dzen());
                    pcv->zen2(obj_pcv->zen2());
                    pco_map = obj_pcv->pco();
                }
                if (par.parType == par_type::PCV_SAT || par.parType == par_type::PCV_REC)
                {
                    ipar++;
                    pcv->dazi(par.dazi);
                    pcv->dzen(par.dzen);
                    pcv->zen2(par.zen2);
                    if (double_eq(par.dazi, 0.0))
                    {
                        zen_map[par.zen1] = (pars->correct_value + pars->par.value()) * 1000;
                        if (abs(zen_map[par.zen1]) > 100)
                        {
                            lzero = true;
                        }
                        if (ipar >= npar)
                        {
                            if (lzero)
                            {
                                for (auto zen_tmp = zen_map.begin(); zen_tmp != zen_map.end(); zen_tmp++)
                                {
                                    zen_map[zen_tmp->first] = 0.0;
                                }
                            }
                            for (auto fq = fqs.begin(); fq != fqs.end(); fq++)
                            {
                                pcv->pcvzen(*fq, zen_map);
                                Triple t = pco_map[*fq];
                                pcv->pco(*fq, t);
                            }
                            ipar = -1;
                            zen_map.clear();
                            lzero = false;
                        }
                    }
                    else
                    {
                        azi_map[par.azi1][par.zen1] = (pars->correct_value + pars->par.value()) * 1000;
                        if (ipar >= npar)
                        {
                            for (auto fq = fqs.begin(); fq != fqs.end(); fq++)
                            {
                                //std::cout << t_gfreq::gfreq2str(*fq) << endl;
                                pcv->pcvazi(*fq, azi_map);
                                pcv->pco(*fq, obj_pcv->pco(*fq));
                            }
                            ipar = -1;
                            azi_map.clear();
                        }
                    }
                }
                else if (par.parType == par_type::PCO_SAT_X || par.parType == par_type::PCO_REC_X)
                {
                    ipar++;
                    pco[0] = (pars->correct_value + pars->par.value()) * 1000;
                }
                else if (par.parType == par_type::PCO_SAT_Y || par.parType == par_type::PCO_REC_Y)
                {
                    ipar++;
                    pco[1] = (pars->correct_value + pars->par.value()) * 1000;
                }
                else if (par.parType == par_type::PCO_SAT_Z || par.parType == par_type::PCO_REC_Z)
                {
                    pco[2] = (pars->correct_value + pars->par.value()) * 1000;
                    for (auto fq = fqs.begin(); fq != fqs.end(); fq++)
                    {
                        Triple t = pco_map[*fq];
                        if (double_eq(pco[0], 0.0))
                        {
                            pco[0] = t[0];
                        }
                        if (double_eq(pco[1], 0.0))
                        {
                            pco[1] = t[1];
                        }
                        pcv->pco(*fq, pco);
                    }
                    map<GFRQ, map<double, double>> zen_map = obj_pcv->pcvzen();
                    map<GFRQ, map<double, map<double, double>>> azi_map = obj_pcv->pcvazi();
                    if (zen_map.size() > 0)
                    {
                        for (auto iter = zen_map.begin(); iter != zen_map.end(); iter++)
                        {
                            pcv->pcvzen(iter->first, iter->second);
                        }
                    }
                    if (azi_map.size() > 0)
                    {
                        for (auto iter = azi_map.begin(); iter != azi_map.end(); iter++)
                        {
                            pcv->pcvazi(iter->first, iter->second);
                        }
                    }
                    pco = {0.0, 0.0, 0.0};
                    ipar = -1;
                }
                if (pars == iter->second[iter->second.size() - 1])
                {
                    pcvdata.addpcv(pcv);
                }
            }
        }
    }

    /*void gnss_all_recover::get_stadata(t_gallsta& stadata)
     {
          map<string, double> dx;
          map<string, double> dy;
          map<string, double> dz;

          for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
          {
               base_time epoch = iter->first;
               for (auto par : iter->second) {
                    string obj = "";
                    if (par->par.parType == par_type::CRD_X) {
                         obj = par->par.site;
                         dx[obj] = par->correct_value;
                    }
                    else if (par->par.parType == par_type::CRD_Y) {
                         obj = par->par.site;
                         dy[obj] = par->correct_value;
                    }
                    else if (par->par.parType == par_type::CRD_Z) {
                         obj = par->par.site;
                         dz[obj] = par->correct_value;
                    }
                    else {
                         continue;
                    }

               }
          }

          for (auto iter = dx.begin(); iter != dx.end(); iter++)
          {
               stadata.addsta(iter->first, iter->second, dy[iter->first], dz[iter->first]);
          }

     }*/

    void gnss_all_recover::get_clk13data(gnss_all_prec &clkdata, gnss_all_prec::clk_type type)
    {
        set<gnss_all_prec::clk_type> type_list;
        if (type != gnss_all_prec::UNDEF)
        {
            type_list.insert(type);
        }
        else
        {
            type_list.insert(gnss_all_prec::AR);
            type_list.insert(gnss_all_prec::AS);
        }

        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            map<string, double> ifcb_data;
            base_time epoch = iter->first;
            // int i = 0;
            sort(_time_parmap_pce[epoch].begin(), _time_parmap_pce[epoch].end());
            for (auto par : _time_parmap_pce[epoch])
            {
                string obj = "";
                if (par.par.parType == par_type::CLK && type_list.find(gnss_all_prec::AR) != type_list.end())
                {
                    obj = par.par.site;
                }
                //else if ((par->par.parType == base_par::CLK15_G_SAT || par->par.parType == base_par::E15CLK_SAT ||
                //     par->par.parType == base_par::C15CLK_SAT || par->par.parType == base_par::J15CLK_SAT)
                //     && type_list.find(gnss_all_prec::AS) != type_list.end()) {
                //     obj = par->par.prn;
                //}
                else if (par.par.parType == par_type::CLK_SAT && type_list.find(gnss_all_prec::AS) != type_list.end())
                {
                    obj = par.par.prn;
                }
                else if (par.par.parType == par_type::IFCB_F3 && type_list.find(gnss_all_prec::AS) != type_list.end())
                {
                    obj = par.par.prn;
                }
                else
                {
                    continue;
                }

                if (par.correct_value == 0.0)
                {
                    continue;
                }

                double clk[3] = {0.0, 0.0, 0.0};
                double clk_tri[4] = {0.0, 0.0, 0.0, 0.0}; //add by xiongyun
                double var[3] = {0.0, 0.0, 0.0};

                // xiongyun change for IFCB

                if (par.par.parType == par_type::IFCB_F3)
                {
                    ifcb_data[par.par.prn] = (par.par.value() + par.correct_value) / CLIGHT;
                    continue;
                }

                clk[0] = (par.par.value() + par.correct_value) / CLIGHT;
                clk_tri[0] = clk[0];
                if (ifcb_data.find(par.par.prn) != ifcb_data.end())
                {
                    clk_tri[3] = ifcb_data[par.par.prn];
                }
                if (ifcb_data.size() == 0)
                {
                    clkdata.addclk(obj, epoch, clk, var);
                }
                else
                {
                    clkdata.addclk_tri(obj, epoch, clk_tri, var);
                }
            }
        }
    }

    void gnss_all_recover::get_iondata(vector<hwa_TUPLE_gnss_ION> &ion_data)
    {
        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto par : iter->second)
            {
                string type = "";
                if (par->par.parType == par_type::SION)
                {
                    type = "SION";
                }
                else if (par->par.parType == par_type::VION)
                {
                    type = "VION";
                }
                else
                {
                    continue;
                }

                if (double_eq(par->correct_value, 0.0))
                    continue;

                double value = 0.0;
                double sigma = 0.5;

                value = (par->par.value() + par->correct_value);
                auto tmp = make_tuple(type, par->par.site, par->par.prn,
                                      value, sigma, par->par.beg, par->par.end);
                ion_data.push_back(tmp);
            }
        }
    }

    vector<gnss_data_recover_equation> gnss_all_recover::get_first_phase_recover_equation(const string &site, const string &sat, const string &freq)
    {
        string trim_freq = (base_type_conv::trim(freq));

        map<gnss_data_obscombtype, int> phase_order =
            {
                {gnss_data_obscombtype("LC12"), 13},
                {gnss_data_obscombtype("LC13"), 12},
                {gnss_data_obscombtype("LC14"), 11},
                {gnss_data_obscombtype("LC15"), 10},
                {gnss_data_obscombtype("L1"), 9},
                {gnss_data_obscombtype("L2"), 8},
                {gnss_data_obscombtype("L3"), 7},
                {gnss_data_obscombtype("L4"), 6},
                {gnss_data_obscombtype("L5"), 5},
            };

        vector<gnss_data_recover_equation> first_phase_recover_equation;

        //TODO COMMENT
        if (phase_order.find(gnss_data_obscombtype(trim_freq)) != phase_order.end())
        {
            phase_order[gnss_data_obscombtype(trim_freq)] = 14;
        }
        else
        {
            throw runtime_error("check your xml file, the edit res freq should be LC1X or LCX (X means 1-5)");
        }

        for (auto time_equ : _site_sat_time_equmap[site][sat])
        {
            gnss_data_recover_equation first_equ(_spdlog, time_equ.first, site, sat);

            bool find = false;
            int max_order = 0;
            for (auto equ : time_equ.second)
            {
                //add for slr
                if (equ->obstype.is_SLR())
                {
                    first_equ = *equ;
                    find = true;
                    continue; // jdhuang : slr process is over
                }

                if (!equ->obstype.is_phase())
                {
                    continue;
                }
                if (equ->obstype.is_KBR())
                {
                    first_equ = *equ;
                }
                if (equ->obstype.is_LRI())
                {
                    first_equ = *equ;
                }

                string trim_equ_freq = base_type_conv::trim(("L" + gfreqseq2str(equ->obstype.getFreq_1()) + gfreqseq2str(equ->obstype.getFreq_2())));
                if (trim_freq != trim_equ_freq)
                {
                    continue;
                }

                if (phase_order[equ->obstype] > max_order)
                {
                    max_order = phase_order[equ->obstype];
                    first_equ = *equ;
                    find = true;
                    continue;
                }
            }

            if (find)
                first_phase_recover_equation.push_back(first_equ); // jdhuang : return only when the freq found
        }
        return first_phase_recover_equation;
    }

    bool gnss_all_recover::get_first_pseudorange_recover_equation(string site, string sat, vector<gnss_data_recover_equation> &equ, string freq)
    {
        string trim_freq = (base_type_conv::trim(freq));
        map<gnss_data_obscombtype, int> pseudorange_order =
            {
                {gnss_data_obscombtype("PC12"), 13},
                {gnss_data_obscombtype("PC13"), 12},
                {gnss_data_obscombtype("PC14"), 11},
                {gnss_data_obscombtype("PC15"), 10},
                {gnss_data_obscombtype("P1"), 9},
                {gnss_data_obscombtype("P2"), 8},
                {gnss_data_obscombtype("P3"), 7},
                {gnss_data_obscombtype("P4"), 6},
                {gnss_data_obscombtype("P5"), 5},
            };

        vector<gnss_data_recover_equation> first_pseudorange_recover_equation;

        //TODO COMMENT
        // bool freq_find = false;
        int max_order = 0;
        if (pseudorange_order.find(gnss_data_obscombtype(trim_freq)) != pseudorange_order.end())
        {
            pseudorange_order[gnss_data_obscombtype(trim_freq)] = 14;
            max_order = 14;
            // freq_find = true;
        }
        else
        {
            throw runtime_error("check your xml file, the edit res freq should be PC1X or PCX (X means 1-5)");
        }

        for (const auto &time_equ : _site_sat_time_equmap[site][sat])
        {
            gnss_data_recover_equation first_equ(time_equ.first, site, sat);

            bool find = false;
            for (auto equ : time_equ.second)
            {
                if (!equ->obstype.is_pseudorange())
                {
                    continue;
                }

                if (pseudorange_order[equ->obstype] == max_order)
                {
                    if (find == false)
                        first_equ = *equ;
                    find = true;
                    continue;
                }

                if (pseudorange_order[equ->obstype] > max_order)
                {
                    max_order = pseudorange_order[equ->obstype];
                    first_equ = *equ;
                    find = true;
                    continue;
                }
            }

            if (find)
                first_pseudorange_recover_equation.push_back(first_equ); // jdhuang : return only when the freq found
        }

        if (first_pseudorange_recover_equation.empty())
            return false;
        equ = first_pseudorange_recover_equation;

        return true;
    }

    bool gnss_all_recover::get_first_phase_recover_equation(const string &site, const string &sat, vector<gnss_data_recover_equation> &equ, const string &freq)
    {
        string trim_freq = (base_type_conv::trim(freq));

        map<gnss_data_obscombtype, int> phase_order =
            {
                {gnss_data_obscombtype("LC12"), 13},
                {gnss_data_obscombtype("LC13"), 12},
                {gnss_data_obscombtype("LC14"), 11},
                {gnss_data_obscombtype("LC15"), 10},
                {gnss_data_obscombtype("L1"), 9},
                {gnss_data_obscombtype("L2"), 8},
                {gnss_data_obscombtype("L3"), 7},
                {gnss_data_obscombtype("L4"), 6},
                {gnss_data_obscombtype("L5"), 5},
            };

        vector<gnss_data_recover_equation> first_phase_recover_equation;

        //TODO COMMENT
        // bool freq_find = false;
        int max_order = 0;
        if (phase_order.find(gnss_data_obscombtype(trim_freq)) != phase_order.end())
        {
            phase_order[gnss_data_obscombtype(trim_freq)] = 14;
            max_order = 14;
            // freq_find = true;
        }
        else
        {
            throw runtime_error("check your xml file, the edit res freq should be LC1X or LCX (X means 1-5)");
        }

        for (auto time_equ : _site_sat_time_equmap[site][sat])
        {
            gnss_data_recover_equation first_equ(_spdlog, time_equ.first, site, sat);

            bool find = false;
            if (time_equ.second.empty())
                continue;
            for (auto equ : time_equ.second)
            {
                //add for slr
                if (equ->obstype.is_SLR())
                {
                    first_equ = *equ;
                    find = true;
                    continue; // jdhuang : slr process is over
                }

                if (!equ->obstype.is_phase())
                {
                    continue;
                }

                //string trim_equ_freq = base_type_conv::trim(("L" + gfreqseq2str(equ->obstype.getFreq_1()) + gfreqseq2str(equ->obstype.getFreq_2())));
                //if (trim_freq != trim_equ_freq)
                //{
                //     continue;
                //}

                if (phase_order[equ->obstype] == max_order)
                {
                    if (find == false)
                        first_equ = *equ;
                    find = true;
                    continue;
                }

                if (phase_order[equ->obstype] > max_order)
                {
                    max_order = phase_order[equ->obstype];
                    first_equ = *equ;
                    find = true;
                    continue;
                }
            }

            if (find)
                first_phase_recover_equation.push_back(first_equ); // jdhuang : return only when the freq found
        }

        if (first_phase_recover_equation.empty())
            return false;
        equ = first_phase_recover_equation;

        return true;
    }

    vector<gnss_data_recover_par> gnss_all_recover::get_recover_par(const par_type &parType)
    {
        vector<gnss_data_recover_par> tmp;
        for (auto type_par : _type_parmap[parType])
        {
            gnss_data_recover_par par(_spdlog, type_par->par, type_par->correct_value);
            tmp.push_back(par);
        }
        return tmp;
    }

    base_time gnss_all_recover::get_beg_time() const
    {
        return _recover_head.get_beg_time();
    }

    base_time gnss_all_recover::get_end_time() const
    {
        return _recover_head.get_end_time();
    }

    base_time gnss_all_recover::get_equ_beg_time() const
    {
        return _time_equmap.begin()->first;
    }

    double gnss_all_recover::get_interval() const
    {
        return _recover_head.interval;
    }

    double gnss_all_recover::get_sigma0() const
    {
        return _recover_head.sigma0;
    }

    void gnss_all_recover::set_interval(double intv)
    {
        _recover_head.interval = intv;
    }

    void gnss_all_recover::set_sigma0(double sigma0)
    {
        _recover_head.sigma0 = sigma0;
    }

    const vector<gnss_data_recover *> &gnss_all_recover::get_all_recover_data() const
    {
        return _recover_data;
    }

    double gnss_all_recover::get_recover_data_value(par_type parType)
    {
        vector<gnss_data_recover *>::iterator it;
        for (it = _recover_data.begin(); it != _recover_data.end(); ++it)
        {
            if (parType == dynamic_cast<gnss_data_recover_par *>(*it)->par.parType)
            {
                double par_value = dynamic_cast<gnss_data_recover_par *>(*it)->par.value();
                double correct_value = dynamic_cast<gnss_data_recover_par *>(*it)->correct_value;
                if (double_eq(par_value, 0.0) || double_eq(correct_value, 0.0))
                    return 0;
                return par_value + correct_value;
            }
        }
        return 0;
    }

    const hwa_map_gnss_TIME_PAR &gnss_all_recover::get_map_time_par() const
    {
        return _time_parmap;
    }

    const hwa_map_TIME_EQU &gnss_all_recover::get_map_time_equ() const
    {
        return _time_equmap;
    }
    const hwa_map_SITE_EQU &gnss_all_recover::get_map_site_equ() const
    {
        return _site_sat_time_equmap;
    }

    set<string> gnss_all_recover::get_sat_list() const
    {
        return _recover_head.sat_list;
    }

    set<string> gnss_all_recover::get_site_list() const
    {
        return _recover_head.site_list;
    }

    set<base_time> gnss_all_recover::get_time_list() const
    {
        return _recover_head.time_list;
    }

    vector<base_time> gnss_all_recover::get_all_obs_time_list() const
    {
        base_time beg = _time_equmap.begin()->first;
        base_time end = _time_equmap.rbegin()->first;

        int nepoch = floor((end - beg) / _recover_head.interval) + 1;
        vector<base_time> ans;
        for (int i = 0; i < nepoch; i++)
        {
            ans.push_back(beg + _recover_head.interval * i);
        }
        return ans;
    }

    base_allpar gnss_all_recover::get_all_pars()
    {
        base_allpar tmp;

        base_time beg = get_beg_time();
        base_time end = get_end_time();
        while (beg <= end)
        {
            auto par_end = _time_parmap[beg].end();
            for (auto par_crt = _time_parmap[beg].begin(); par_crt != par_end; par_crt++)
            {
                auto recover_par = (*par_crt)->par;
                recover_par.value(recover_par.value() + (*par_crt)->correct_value);
                tmp.addParam(recover_par);
            }
        }

        return tmp;
    }

    map<base_time, int> gnss_all_recover::get_sat_number_per_epo() const
    {
        map<base_time, int> nsat;
        for (auto it : _time_equmap)
        {
            nsat[it.first] = it.second.size() / 2;
        }
        return nsat;
    }

    void gnss_all_recover::get_stadata(map<string, Triple> &xyz, map<string, Triple> &neu)
    {
        map<string, double> dx, dy, dz;
        map<string, double> x, y, z;
        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto par : iter->second)
            {
                string obj = "";
                if (par->par.parType == par_type::CRD_X)
                {
                    obj = par->par.site;
                    dx[obj] = par->correct_value;
                    x[obj] = par->par.value();
                }
                else if (par->par.parType == par_type::CRD_Y)
                {
                    obj = par->par.site;
                    dy[obj] = par->correct_value;
                    y[obj] = par->par.value();
                }
                else if (par->par.parType == par_type::CRD_Z)
                {
                    obj = par->par.site;
                    dz[obj] = par->correct_value;
                    z[obj] = par->par.value();
                }
                else
                {
                    continue;
                }
            }
        }

        for (auto iter = x.begin(); iter != x.end(); iter++)
        {
            Triple ell, xyz_apriori, neutmp, dxyz;
            xyz_apriori[0] = x[iter->first];
            xyz_apriori[1] = y[iter->first];
            xyz_apriori[2] = z[iter->first];
            dxyz[0] = dx[iter->first];
            dxyz[1] = dy[iter->first];
            dxyz[2] = dz[iter->first];

            xyz2ell(xyz_apriori, ell, false);
            xyz2neu(ell, dxyz, neutmp);

            xyz[iter->first] = dxyz;
            neu[iter->first] = neutmp;
        }
    }

    void gnss_all_recover::get_rbdata(vector<string> &objs, vector<string> &obj_sats, vector<double> &rbs)
    {
        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto par : iter->second)
            {
                string obj = "";
                string obj_sat = "";
                if (par->par.parType == par_type::RB)
                {
                    obj = par->par.site;
                    obj_sat = par->par.prn;
                }
                else
                {
                    continue;
                }

                double rb = 0.0;

                rb = par->par.value() + par->correct_value;

                objs.push_back(obj);
                obj_sats.push_back(obj_sat);
                rbs.push_back(rb);
            }
        }
    }

    void gnss_all_recover::get_eopdata(vector<base_time> &epoches,
                                    vector<double> &xpoles,
                                    vector<double> &ypoles,
                                    vector<double> &dxpoles,
                                    vector<double> &dypoles,
                                    vector<double> &ut1s,
                                    vector<double> &dut1s)
    {
        //map<base_time, eop> eop_all;
        //eop eop_one;
        // int tab = 0;

        for (auto iter = _time_parmap.begin(); iter != _time_parmap.end(); iter++)
        {
            base_time epoch = iter->first;
            for (auto par : iter->second)
            {
                if (par->par.parType == par_type::XPOLE)
                {
                    //eop_one.xpole = par->par.value() + par->correct_value;
                    //eop_one.xpole = par->correct_value;
                    xpoles.push_back(par->correct_value);
                    //tab = 1;
                }
                else if (par->par.parType == par_type::YPOLE)
                {
                    //eop_one.ypole = par->par.value() + par->correct_value;
                    //eop_one.ypole = par->correct_value;
                    ypoles.push_back(par->correct_value);
                }
                else if (par->par.parType == par_type::DXPOLE)
                {
                    //eop_one.dxpole = par->par.value() + par->correct_value;
                    //eop_one.dxpole = par->correct_value;
                    dxpoles.push_back(par->correct_value);
                }
                else if (par->par.parType == par_type::DYPOLE)
                {
                    //eop_one.dypole = par->par.value() + par->correct_value;
                    //eop_one.dypole = par->correct_value;
                    dypoles.push_back(par->correct_value);
                }
                else if (par->par.parType == par_type::UT1)
                {
                    //eop_one.ut1 = par->par.value() + par->correct_value;
                    //eop_one.ut1 = par->correct_value;
                    ut1s.push_back(par->correct_value);
                }
                else if (par->par.parType == par_type::DUT1)
                {
                    //eop_one.dut1 = par->par.value() + par->correct_value;
                    //eop_one.dut1 = par->correct_value;
                    dut1s.push_back(par->correct_value);
                }
                else
                {
                    continue;
                }
            }
            //if (tab == 1)
            //{
            //     eop_all[iter->first] = eop_one;
            //     eop_one.clear_value();
            //     tab = 0;
            //}
        }

        //for (auto iter = eop_all.begin(); iter != eop_all.end(); iter++)
        //{
        //     eopdata.addeop(iter->first, iter->second);
        //}
    }

    void gnss_all_recover::_add_common_data(gnss_data_recover *data)
    {
        _recover_data.push_back(data);
        _recover_head.time_list.insert(data->get_recover_time());
    }

}