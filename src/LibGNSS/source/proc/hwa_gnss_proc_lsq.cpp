#include "hwa_set_param.h"
#include "hwa_set_amb.h"
#include "hwa_set_out.h"
#include "hwa_set_turboedit.h"
#include "hwa_set_npp.h"
#include "hwa_set_leo.h"
#include "hwa_base_file.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_time.h"
#include "hwa_base_string.h"
#include "hwa_gnss_proc_lsq.h"
#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_gnss_coder_recover.h"
#include "hwa_gnss_coder_rinexc.h"
#include "hwa_gnss_coder_poleut.h"
#include "hwa_gnss_coder_atx.h"
#include "hwa_gnss_coder_ambupd.h"
#include "hwa_gnss_coder_sp3.h"
#include "hwa_gnss_proc_updateparif.h"
#include "hwa_gnss_proc_updateparall.h"
#include "hwa_gnss_coder_ion.h"
#include "hwa_gnss_data_pcv.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_set_rec.h"
#include <io.h>
#include <direct.h>
#define ACCESS(fileName, accessMode) _access(fileName, accessMode)
#define MKDIR(path) _mkdir(path)

namespace hwa_gnss
{
    gnss_proc_lsq::gnss_proc_lsq()
    {
    }

    gnss_proc_lsq::gnss_proc_lsq(set_base *set,  base_all_proc *data, base_log spdlog) : _spdlog(spdlog),
                                                                                 _gset(set),
                                                                                 _gall_proc(data),
                                                                                 _recclk_threshold(1e-8)
    {
    }

    gnss_proc_lsq::~gnss_proc_lsq()
    {
        // Init Paramter in Constructor???
        if (_glsq)
        {
            delete _glsq;
            _glsq = nullptr;
        }
    }

    bool gnss_proc_lsq::ProcessBatch( base_all_proc *data, const base_time &beg, const base_time &end)
    {
        return true;
    }

    bool gnss_proc_lsq::GenerateProduct()
    {
        return false;
    }

    bool gnss_proc_lsq::_init_crs_erp_pars(gnss_proc_lsqbase *lsq)
    {
        int ipar_num = _glsq->_x_solve.parNumber();

        // init EOP parmeter
        // init the Eop dx dy vdx vdy
        base_par par_xpole("", par_type::XPOLE, ++ipar_num, "");
        base_par par_ypole("", par_type::YPOLE, ++ipar_num, "");
        base_par par_dxpole("", par_type::DXPOLE, ++ipar_num, "");
        base_par par_dypole("", par_type::DYPOLE, ++ipar_num, "");
        base_par par_ut1("", par_type::UT1, ++ipar_num, "");
        base_par par_dut1("", par_type::DUT1, ++ipar_num, "");

        // TODO:init value time sigma
        std::map<std::string, double> data1, data2;

        try
        {
            if (!_gdata_erp)
                _gdata_erp = dynamic_cast<gnss_data_poleut *>((*_gall_proc)[base_data::ALLPOLEUT1]);
            if (!_gdata_erp)
                return false;
            _gdata_erp->getEopData(_beg_time, data1);
            _gdata_erp->getEopData(_end_time, data2);
        }
        catch (std::exception e)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, e.what());
            return false;
        }

        double sig_xpole = dynamic_cast<set_par *>(_gset)->sigXpole();
        double sig_ypole = dynamic_cast<set_par *>(_gset)->sigYpole();
        double sig_dxpole = dynamic_cast<set_par *>(_gset)->sigDxpole();
        double sig_dypole = dynamic_cast<set_par *>(_gset)->sigDypole();
        double sig_ut1 = dynamic_cast<set_par *>(_gset)->sigUt1();
        double sig_dut1 = dynamic_cast<set_par *>(_gset)->sigDut1();

        par_xpole.setTime(_beg_time, _end_time);
        par_xpole.value(data1["XPOLE"]);
        par_xpole.apriori(sig_xpole);
        par_ypole.setTime(_beg_time, _end_time);
        par_ypole.value(data1["YPOLE"]);
        par_ypole.apriori(sig_ypole);
        par_dxpole.setTime(_beg_time, _end_time);
        par_dxpole.value(data2["XPOLE"] - data1["XPOLE"]);
        par_dxpole.apriori(sig_dxpole);
        par_dypole.setTime(_beg_time, _end_time);
        par_dypole.value(data2["YPOLE"] - data1["YPOLE"]);
        par_dypole.apriori(sig_dypole);
        par_ut1.setTime(_beg_time, _end_time);
        par_ut1.value(data1["UT1-TAI"]);
        par_ut1.apriori(sig_ut1);
        par_dut1.setTime(_beg_time, _end_time);
        par_dut1.value(data2["UT1-TAI"] - data1["UT1-TAI"]);
        par_dut1.apriori(sig_dut1);

        // add the Estimator
        lsq->add_parameter(par_xpole);
        lsq->add_parameter(par_ypole);
        lsq->add_parameter(par_dxpole);
        lsq->add_parameter(par_dypole);
        lsq->add_parameter(par_ut1);
        lsq->add_parameter(par_dut1);

        return true;
    }

    //HwangShih added
    bool gnss_proc_lsq::_init_crs_geoc_pars(gnss_proc_lsqbase *lsq)
    {
        int ipar_num = _glsq->_x_solve.parNumber();

        // init GEOC parmeters
        // init the geocx geocy geocz
        base_par par_geocx("", par_type::GEOCX, ++ipar_num, "");
        base_par par_geocy("", par_type::GEOCY, ++ipar_num, "");
        base_par par_geocz("", par_type::GEOCZ, ++ipar_num, "");

        // init value time sigma
        par_geocx.setTime(_beg_time, _end_time);
        par_geocx.value(0.0);
        par_geocx.apriori(dynamic_cast<set_par *>(_gset)->sigCX());
        par_geocy.setTime(_beg_time, _end_time);
        par_geocy.value(0.0);
        par_geocy.apriori(dynamic_cast<set_par *>(_gset)->sigCY());
        par_geocz.setTime(_beg_time, _end_time);
        par_geocz.value(0.0);
        par_geocz.apriori(dynamic_cast<set_par *>(_gset)->sigCZ());

        // add the Estimator
        lsq->add_parameter(par_geocx);
        lsq->add_parameter(par_geocy);
        lsq->add_parameter(par_geocz);

        return true;
    }

    bool gnss_proc_lsq::_init_rec_trop_pars(gnss_proc_lsqbase *lsq)
    {
        if (_rec_list.empty() || !_glsq || !_gset)
            return false;

        int ipar_num = _glsq->_x_solve.parNumber();
        auto it_beg = _rec_list.begin();
        auto it_end = _rec_list.end();
        for (auto it = it_beg; it != it_end; it++)
        {
            // init the ztd
            std::string rec = *it;
            base_par par_ztd(rec, par_type::TRP, ++ipar_num, "");
            std::shared_ptr<gnss_data_obj> _grec = _gall_obj->obj(rec);
            if (_grec->id_type() == base_data::REC_LEO)
            {
                //it++;  //by zhangwei
                continue;
            }
            double sig_ztd = dynamic_cast<set_par *>(_gset)->sigZtd(rec);
            double sig_TropPd = dynamic_cast<set_par *>(_gset)->sigTropPd(rec);
            par_ztd.apriori(sig_ztd);
            par_ztd.value(0.0);
            par_ztd.setTime(_beg_time, _beg_time + _pwc_intv * 60.0);
            lsq->add_parameter(par_ztd);
            if (_ztd_model == ZTDMODEL::PWC)
            {
                lsq->add_par_state_equ(par_ztd.parType, 1, _pwc_intv / 60.0, sig_TropPd);
            }
            if (_ztd_model == ZTDMODEL::STO)
            {
                lsq->add_par_state_equ(par_ztd.parType, 1, _obs_intv / 3600.0, sig_TropPd);
            }

            // init tropospheric gradients
            if (_tropo_grad)
            {
                double sig_grd = dynamic_cast<set_par *>(_gset)->sigGRD(rec);
                double sig_GrdPd = dynamic_cast<set_par *>(_gset)->sigGrdPd(rec);
                base_par par_grdN(rec, par_type::GRD_N, ++ipar_num, "");
                par_grdN.setTime(_beg_time, _beg_time + _grd_pwc_intv * 60.0);
                par_grdN.apriori(sig_grd);
                par_grdN.value(0.0);
                lsq->add_parameter(par_grdN);

                base_par par_grdE(rec, par_type::GRD_E, ++ipar_num, "");
                par_grdE.setTime(_beg_time, _beg_time + _grd_pwc_intv * 60.0);
                par_grdE.value(0.0);
                par_grdE.apriori(sig_grd);
                lsq->add_parameter(par_grdE);

                if (_grd_model == GRDMODEL::GRD_PWC)
                {
                    lsq->add_par_state_equ(par_grdN.parType, 1, _grd_pwc_intv / 60.0, sig_GrdPd);
                    lsq->add_par_state_equ(par_grdE.parType, 1, _grd_pwc_intv / 60.0, sig_GrdPd);
                }
                if (_grd_model == GRDMODEL::GRD_STO)
                {
                    lsq->add_par_state_equ(par_grdN.parType, 1, _obs_intv / 3600.0, sig_GrdPd);
                    lsq->add_par_state_equ(par_grdE.parType, 1, _obs_intv / 3600.0, sig_GrdPd);
                }
            }
        }
        return true;
    }

    bool gnss_proc_lsq::_init_gns_vion_pars(gnss_proc_lsqbase *lsq)
    {
        if (_rec_list.empty() || _sat_list.empty() || !_glsq || !_gset)
            return false;

        int ipar_num = _glsq->_x_solve.parNumber();

        auto it_beg = _rec_list.begin();
        auto it_end = _rec_list.end();
        for (auto it = it_beg; it != it_end; it++)
        {
            std::string rec = *it;
            // init the ztd
            base_par par_ion(rec, par_type::VION, ++ipar_num, "");
            double sig_ion = dynamic_cast<set_par *>(_gset)->sigVion(rec);
            par_ion.apriori(sig_ion);

            // init in precisemodel
            double dt = dynamic_cast<set_gen *>(_gset)->sampling(); // minute
            if (dt == 0.0)
                return false;
            par_ion.value(0.0);
            par_ion.setTime(_beg_time, _beg_time + dt);
            lsq->add_parameter(par_ion);
            double sigIonoPd = dynamic_cast<set_par *>(_gset)->sigIonoPd(rec);
            lsq->add_par_state_equ(par_ion.parType, 1, dt / 60.0, sigIonoPd);
        }
        return true;
    }

    bool gnss_proc_lsq::_init_gns_sion_pars(gnss_proc_lsqbase *lsq)
    {
        if (_rec_list.empty() || _sat_list.empty() || !_glsq || !_gset)
            return false;

        for (const auto &it_rec : _rec_list)
        {
            for (const auto &it_sat : _sat_list)
            {
                std::string rec = it_rec;
                std::string sat = it_sat;
                //double sig_ion = dynamic_cast<set_par*>(_gset)->sigVion(rec);  glfeng
                double sig_ion = dynamic_cast<set_par *>(_gset)->sigSion(rec);

                int ipar_num = _glsq->_x_solve.parNumber();
                base_par par_ion(rec, par_type::SION, ++ipar_num, sat);
                par_ion.apriori(sig_ion);
                par_ion.value(0.0);
                par_ion.setTime(_beg_time, _beg_time);
                lsq->add_parameter(par_ion);
            }
        }
        return true;
    }

    bool gnss_proc_lsq::_init_rec_clk_pars(gnss_proc_lsqbase *lsq)
    {

        if (_rec_list.empty() || !lsq || !_gset)
        {
            return false;
        }

        std::string site_ref = dynamic_cast<set_gproc *>(_gset)->ref_clk();
        auto it_beg = _rec_list.begin();
        auto it_end = _rec_list.end();
        for (auto it = it_beg; it != it_end; it++)
        {

            int ipar_num = lsq->_x_solve.parNumber();

            std::string site = *it;
            double sigclk = dynamic_cast<set_par *>(_gset)->sigRecCLK(site);
            double sig_refclk = dynamic_cast<set_gproc *>(_gset)->sig_ref_clk();

            if (site == site_ref)
            {
                if (sig_refclk == 0.0)
                {
                    //it++;
                    continue;
                }
                else
                {
                    sigclk = sig_refclk;
                }
            }

            //intit the Clock
            base_par par_clk(site, par_type::CLK, ++ipar_num, "");
            par_clk.value(0.0);
            par_clk.setTime(_beg_time, _beg_time);
            par_clk.apriori(sigclk);
            lsq->add_parameter(par_clk);
        }
        return true;
    }

    bool gnss_proc_lsq::_init_rec_clk_1X_pars(const int &freq_num, gnss_proc_lsqbase *lsq)
    {

        if (_rec_list.empty() || !lsq || !_gset)
        {
            return false;
        }

        std::string site_ref = dynamic_cast<set_gproc *>(_gset)->ref_clk();
        auto it_beg = _rec_list.begin();
        auto it_end = _rec_list.end();
        for (auto it = it_beg; it != it_end; it++)
        {

            int ipar_num = lsq->_x_solve.parNumber();

            std::string site = *it;
            double sigclk = dynamic_cast<set_par *>(_gset)->sigRecCLK(site);
            double sig_refclk = dynamic_cast<set_gproc *>(_gset)->sig_ref_clk();

            if (site == site_ref)
            {
                if (sig_refclk == 0.0)
                {
                    //it++;
                    continue;
                }
                else
                {
                    sigclk = sig_refclk;
                }
            }

            //intit the Clock
            std::set<std::string>::const_iterator iter_sys = _sys_list.begin();
            for (; iter_sys != _sys_list.end(); iter_sys++)
            {
                GSYS crt_sys = gnss_sys::str2gsys(*iter_sys);
                switch (crt_sys)
                {
                case GPS:
                    if (freq_num > 2 && _freq_number[GPS] > 2)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK13_G, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 3 && _freq_number[GPS] > 3)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK14_G, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 4 && _freq_number[GPS] > 4)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK15_G, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    break;
                case GAL:
                    if (freq_num > 2 && _freq_number[GAL] > 2)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK13_E, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 3 && _freq_number[GAL] > 3)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK14_E, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 4 && _freq_number[GAL] > 4)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK15_E, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    break;
                case BDS:
                    if (freq_num > 2 && _freq_number[BDS] > 2)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK13_C, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 3 && _freq_number[BDS] > 3)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK14_C, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 4 && _freq_number[BDS] > 4)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK15_C, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    break;
                case QZS:
                    if (freq_num > 2 && _freq_number[BDS] > 2)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK13_J, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 3 && _freq_number[BDS] > 3)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK14_J, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    if (freq_num > 4 && _freq_number[BDS] > 4)
                    {
                        base_par par_clk;
                        par_clk = base_par(site, par_type::CLK15_J, ++ipar_num, "");
                        par_clk.value(0.0);
                        par_clk.setTime(_beg_time, _beg_time);
                        par_clk.apriori(sigclk);
                        lsq->add_parameter(par_clk);
                    }
                    break;
                default:
                    break;
                }
            }
        }
        return true;
    }

    bool gnss_proc_lsq::_init_sat_clk_pars(gnss_proc_lsqbase *lsq)
    {
        if (!lsq || !_gset || _sat_list.empty())
            return false;

        int ipar_num = _glsq->_x_solve.parNumber();
        auto it_beg = _sat_list.begin();
        auto it_end = _sat_list.end();
        std::string sat_ref = dynamic_cast<set_gproc *>(_gset)->ref_clk();
        double sig_refclk = dynamic_cast<set_gproc *>(_gset)->sig_ref_clk();

        for (auto iter = it_beg; iter != it_end; iter++)
        {
            std::string satID = *iter;
            double sigclk = dynamic_cast<set_par *>(_gset)->sigSatCLK(satID);
            if (satID == sat_ref && sig_refclk == 0.0)
            {
                continue;
            }
            else if (satID == sat_ref && sig_refclk != 0.0)
            {
                sigclk = sig_refclk;
            }
            else
            {
                sigclk = dynamic_cast<set_par *>(_gset)->sigSatCLK(satID);
            }

            base_par par_satclk("", par_type::CLK_SAT, ++ipar_num, satID);
            par_satclk.setTime(_beg_time, _beg_time);
            par_satclk.apriori(sigclk);
            par_satclk.value(0.0);
            lsq->add_parameter(par_satclk);
        }
        return true;
    }

    bool gnss_proc_lsq::_init_sagnss_coder_ifcb_pars(gnss_proc_lsqbase *lsq)
    {
        if (!lsq || !_gset || _sat_list.empty())
            return false;

        int ipar_num = _glsq->_x_solve.parNumber();
        auto it_beg = _sat_list.begin();
        auto it_end = _sat_list.end();
        for (auto iter = it_beg; iter != it_end; iter++)
        {
            std::string satID = *iter;
            // double sigclk = dynamic_cast<set_par *>(_gset)->sigSatCLK(satID);

            base_par par_satclk("", par_type::IFCB_F3, ++ipar_num, satID);
            par_satclk.setTime(_beg_time, _beg_time);
            par_satclk.apriori(3000);
            par_satclk.value(0.0);
            lsq->add_parameter(par_satclk);
        }
        return true;
    }

    // std::set sys bia pars
    bool gnss_proc_lsq::_init_sys_bia_pars(gnss_proc_lsqbase *lsq)
    {
        if (_sys_list.size() < 2 && _obs_mode == OBSCOMBIN::IONO_FREE && _frequency == 2)
            return true;
        if (_sys_list.empty())
            return false;

        // get the ref sys
        //std::string ref_sys = _sys_list.count("GPS") > 0 ? "GPS" : *_sys_list.begin();
        std::string ref_sys;
        if (_sys_list.count("GPS") > 0)
            ref_sys = "GPS";
        else if (_sys_list.count("LEO") > 0)
            ref_sys = "LEO";
        else
            ref_sys = *_sys_list.begin();
        for (const auto &sys : _sys_list)
        {
            if (sys == ref_sys)
                continue;
            for (const auto &crt_rec : _rec_list)
            {
                base_par par_ISB;
                par_ISB.value(0.0);
                par_ISB.apriori(3000.0);

                GSYS crt_sys = gnss_sys::str2gsys(sys);
                switch (crt_sys)
                {
                case GPS:
                    break;
                case LEO:
                    par_ISB = base_par(crt_rec, par_type::LEO_ISB, lsq->_x_solve.parNumber() + 1, ""); //xjhan
                    break;
                case GAL:
                    par_ISB = base_par(crt_rec, par_type::GAL_ISB, lsq->_x_solve.parNumber() + 1, "");
                    break;
                case BDS:
                    par_ISB = base_par(crt_rec, par_type::BDS_ISB, lsq->_x_solve.parNumber() + 1, "");
                    break;
                case QZS:
                    par_ISB = base_par(crt_rec, par_type::QZS_ISB, lsq->_x_solve.parNumber() + 1, "");
                    break;
                case GLO:
                    par_ISB = base_par(crt_rec, par_type::GLO_ISB, lsq->_x_solve.parNumber() + 1, "");
                    break;
                default:
                    break;
                }

                if ((crt_sys != GLO))
                {
                    //if (_sys_list.count("GPS") > 0 && _sys_list.count("LEO") > 0 && crt_sys == LEO) continue; //xjhan test
                    _set_isb_par_setting(lsq, par_ISB, _sysbias_model, 0.0, 3000.0);
                }

                // IFB, special for GLONASS
                if (crt_sys != GLO)
                    continue;
                if (_sysbias_model != SYSBIASMODEL::AUTO_CON && _sysbias_model != SYSBIASMODEL::AUTO_RWK && _sysbias_model != SYSBIASMODEL::AUTO_WHIT)
                {
                    _set_isb_par_setting(lsq, par_ISB, _sysbias_model, 0.0, 3000.0);
                    continue;
                }
                for (const auto &iter_sat : _sat_list)
                {
                    if ((iter_sat).substr(0, 1) != "R")
                        continue;
                    base_par par_IFB;
                    par_IFB = base_par(crt_rec, par_type::GLO_IFB, lsq->_x_solve.parNumber() + 1, iter_sat);
                    _set_isb_par_setting(lsq, par_IFB, _sysbias_model, 0.0, 3000.0);
                } // next glonass sat

            } // next rec
        }     // next system

        return true;
    }

    bool gnss_proc_lsq::_init_sys_rec_ifb_pars(gnss_proc_lsqbase *lsq)
    {
        if (_ifb_model != IFB_model::EST_REC_IFB)
            return false;

        // if raw all mode, init IFB par for each satellite
        // one sat one ifb
        std::map<GSYS, std::set<par_type>> par_list;

        //TODO COMMENT
        switch (_frequency)
        {
        case 5:
            par_list[GPS].insert(par_type::GPS_REC_IFB_C5);
            par_list[GAL].insert(par_type::GAL_REC_IFB_C5);
            par_list[BDS].insert(par_type::BDS_REC_IFB_C5);
        case 4:
            par_list[GPS].insert(par_type::GPS_REC_IFB_C4);
            par_list[GAL].insert(par_type::GAL_REC_IFB_C4);
            par_list[BDS].insert(par_type::BDS_REC_IFB_C4);
        case 3:
            par_list[GPS].insert(par_type::GPS_REC_IFB_C3);
            par_list[GAL].insert(par_type::GAL_REC_IFB_C3);
            par_list[BDS].insert(par_type::BDS_REC_IFB_C3);
        case 2:
            break;
        default:
            return false;
        }

        // one rec one ifb
        for (const auto &rec : _rec_list)
        {
            base_par par_IFB;
            par_IFB.value(0.0);
            par_IFB.apriori(3000.0);

            for (const auto &sys : _sys_list)
            {
                auto gsys = gnss_sys::str2gsys(sys);
                switch (gsys)
                {
                case GPS:
                {
                    for (const auto &par_type : par_list[GPS])
                    {
                        par_IFB = base_par(rec, par_type, lsq->_x_solve.parNumber() + 1, "");
                        _set_isb_par_setting(lsq, par_IFB, _sysbias_model, 0.0, 9000.0);
                    }
                    break;
                }
                case GAL:
                {
                    for (const auto &par_type : par_list[GAL])
                    {
                        par_IFB = base_par(rec, par_type, lsq->_x_solve.parNumber() + 1, "");
                        _set_isb_par_setting(lsq, par_IFB, _sysbias_model, 0.0, 9000.0);
                    }
                    break;
                }
                case BDS:
                    for (const auto &par_type : par_list[BDS])
                    {
                        par_IFB = base_par(rec, par_type, lsq->_x_solve.parNumber() + 1, "");
                        _set_isb_par_setting(lsq, par_IFB, _sysbias_model, 0.0, 9000.0);
                    }
                    break;
                default:
                    throw std::logic_error("can not support sys : " + sys);
                }
            }
        }

        return true;
    }

    bool gnss_proc_lsq::_init_sys_sat_ifb_pars(gnss_proc_lsqbase *lsq)
    {
        if (_ifb_model != IFB_model::EST_SAT_IFB)
            return false;

        // if raw all mode, init IFB par for each satellite
        // one sat one ifb
        std::set<par_type> par_list;

        //TODO COMMENT
        switch (_frequency)
        {
        case 5:
            par_list.insert(par_type::SAT_IFB_C5);
        case 4:
            par_list.insert(par_type::SAT_IFB_C4);
        case 3:
            par_list.insert(par_type::SAT_IFB_C3);
        case 2:
            break;
        default:
            return false;
        }

        for (const auto &par_type : par_list)
        {
            for (const auto &sat : _sat_list)
            {
                if (_ifb_model != IFB_model::EST_SAT_IFB)
                    continue;
                if (sat.find("R") != std::string::npos)
                    continue;

                base_par par_IFB;
                par_IFB = base_par("", par_type, lsq->_x_solve.parNumber() + 1, sat);
                _set_isb_par_setting(lsq, par_IFB, _sysbias_model, 0.0, 3000.0);
            }
        }

        return true;
    }

    bool gnss_proc_lsq::_init_sys_ifb_pars(gnss_proc_lsqbase *lsq)
    {
        if (_ifb_model != IFB_model::EST_IFB)
            return false;

        // if raw all mode, init IFB par for each satellite
        // one sat one ifb
        std::set<par_type> par_list;

        //TODO COMMENT
        switch (_frequency)
        {
        case 5:
            par_list.insert(par_type::IFB_C5);
        case 4:
            par_list.insert(par_type::IFB_C4);
        case 3:
            par_list.insert(par_type::IFB_C3);
        case 2:
            break;
        default:
            return false;
        }

        // one sat-rec one ifb
        for (const auto &sat : _sat_list)
        {
            // pass GLONASS
            if (sat.find("R") != std::string::npos)
                continue;

            for (const auto &rec : _rec_list)
            {
                base_par par_IFB;
                par_IFB.value(0.0);
                par_IFB.apriori(3000.0);

                for (const auto &par_type : par_list)
                {
                    par_IFB = base_par(rec, par_type, lsq->_x_solve.parNumber() + 1, sat);
                    _set_isb_par_setting(lsq, par_IFB, _sysbias_model, 0.0, 3000.0);
                }
            }
        }
        return true;
    }

    // std::set sys bia pars
    bool gnss_proc_lsq::_init_sys_bia_pars_pce(gnss_proc_lsqbase *lsq)
    {
        if (_sys_list.size() < 2)
            return true;
        if (_sys_list.empty())
            return false;

        // get the ref sys
        std::string ref_sys;
        std::set<std::string>::const_iterator iter_sys = _sys_list.begin();
        if (_sys_list.count("GPS") > 0)
            ref_sys = "GPS";
        else
            ref_sys = *iter_sys;
        for (; iter_sys != _sys_list.end(); iter_sys++)
        {
            if (*iter_sys == ref_sys)
                continue;

            std::set<std::string>::const_iterator rec_beg = _rec_list.begin();
            std::set<std::string>::const_iterator rec_end = _rec_list.end();
            std::set<std::string>::const_iterator rec_iter = rec_beg;
            for (; rec_iter != rec_end; ++rec_iter)
            {
                base_par par_ISB, par_IFB;
                int ipar_num = _glsq->_x_solve.parNumber();
                GSYS crt_sys = gnss_sys::str2gsys(*iter_sys);
                std::string crt_rec = *rec_iter;

                switch (crt_sys)
                {
                case GPS:
                    break;
                case GAL:
                    par_ISB = base_par(crt_rec, par_type::GAL_ISB, ++ipar_num, "");
                    par_ISB.value(0.0);
                    par_ISB.setTime(_beg_time, _end_time);
                    par_ISB.apriori(3000.0);
                    _glsq->add_parameter(par_ISB);
                    break;
                case BDS:
                    par_ISB = base_par(crt_rec, par_type::BDS_ISB, ++ipar_num, "");
                    par_ISB.value(0.0);
                    par_ISB.setTime(_beg_time, _end_time);
                    par_ISB.apriori(3000.0);
                    _glsq->add_parameter(par_ISB);
                    break;
                case LEO: //xjhan
                    par_ISB = base_par(crt_rec, par_type::LEO_ISB, ++ipar_num, "");
                    par_ISB.value(0.0);
                    par_ISB.setTime(_beg_time, _end_time);
                    par_ISB.apriori(3000.0);
                    _glsq->add_parameter(par_ISB);
                    break;

                case QZS:
                    par_ISB = base_par(crt_rec, par_type::QZS_ISB, ++ipar_num, "");
                    par_ISB.value(0.0);
                    par_ISB.setTime(_beg_time, _end_time);
                    par_ISB.apriori(3000.0);
                    _glsq->add_parameter(par_ISB);
                    break;
                case GLO:
                    par_ISB = base_par(crt_rec, par_type::GLO_ISB, ++ipar_num, "");
                    par_ISB.value(0.0);
                    par_ISB.setTime(_beg_time, _end_time);
                    par_ISB.apriori(3000.0);
                    _glsq->add_parameter(par_ISB);
                    break;
                default:
                    break;
                }
            }
        }
        return true;
    }

    bool gnss_proc_lsq::_init_IF_amb_1X_pars(const int &freq_num, gnss_proc_lsqbase *lsq)
    {
        if (_rec_list.empty() ||
            _sat_list.empty())
        {
            std::string rec_empty("false");
            std::string sat_empty("false");
            if (_rec_list.empty())
                rec_empty = "true";
            if (_sat_list.empty())
                sat_empty = "true";
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "can not init lsq-IF amb13 pars for  rec list empty is " + rec_empty + " and sat list empty is " + sat_empty);
            return false;
        }

        double sigAmb = dynamic_cast<set_par *>(_gset)->sigAmb();
        std::set<std::string>::const_iterator rec_beg = _rec_list.begin();
        std::set<std::string>::const_iterator rec_end = _rec_list.end();
        std::set<std::string>::const_iterator sat_beg = _sat_list.begin();
        std::set<std::string>::const_iterator sat_end = _sat_list.end();
        for (std::set<std::string>::const_iterator it_rec = rec_beg; it_rec != rec_end; ++it_rec)
        {
            for (std::set<std::string>::const_iterator it_sat = sat_beg; it_sat != sat_end; ++it_sat)
            {
                int ipar_num = _glsq->_x_solve.parNumber();
                if (freq_num > 1)
                {
                    base_par par_amb = base_par(*it_rec, par_type::AMB_IF, ++ipar_num, *it_sat);
                    par_amb.value(0.0);
                    par_amb.setTime(_beg_time, _end_time);
                    par_amb.apriori(sigAmb);
                    lsq->add_parameter(par_amb);
                }
                if (freq_num > 2)
                {
                    base_par par_amb = base_par(*it_rec, par_type::AMB13_IF, ++ipar_num, *it_sat);
                    par_amb.value(0.0);
                    par_amb.setTime(_beg_time, _end_time);
                    par_amb.apriori(sigAmb);
                    lsq->add_parameter(par_amb);
                }
                if (freq_num > 3)
                {
                    base_par par_amb = base_par(*it_rec, par_type::AMB14_IF, ++ipar_num, *it_sat);
                    par_amb.value(0.0);
                    par_amb.setTime(_beg_time, _end_time);
                    par_amb.apriori(sigAmb);
                    lsq->add_parameter(par_amb);
                }
                if (freq_num > 4)
                {
                    base_par par_amb = base_par(*it_rec, par_type::AMB15_IF, ++ipar_num, *it_sat);
                    par_amb.value(0.0);
                    par_amb.setTime(_beg_time, _end_time);
                    par_amb.apriori(sigAmb);
                    lsq->add_parameter(par_amb);
                }
            }
        }
        return true;
    }

    bool gnss_proc_lsq::_initLsqProdData(gnss_all_prod *data)
    {
        return false;
    }

    bool gnss_proc_lsq::_initLsqProcPars(gnss_proc_lsqbase *lsq)
    {
        return false;
    }

    bool gnss_proc_lsq::_initLsqProcData( base_all_proc *data)
    {
        return false;
    }

    bool gnss_proc_lsq::_select_obs(const base_time &epoch, std::vector<gnss_data_sats> &all_obs)
    {
        // ===================================================================================================================
        if (!_slip12)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "_slip 12 is nullptr");
            return false;
        }

        if ((_frequency == 3) && !_slip13)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "_slip 13 is nullptr");
            return false;
        }

        if ((_frequency == 4) && (!_slip13 || !_slip14))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "_slip 13/14 is nullptr");
            return false;
        }

        if ((_frequency == 5) && (!_slip13 || !_slip14 || !_slip15))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "_slip 13/14/15 is nullptr");
            return false;
        }

        // ===================================================================================================================
        // std::set tb spdlog in satdata
        for (auto &iter : all_obs)
        {
            const std::string &obs_rec = iter.site();
            const std::string &obs_sat = iter.sat();
            const GSYS &obs_sys = iter.gsys();

            if (_frequency < 2)
                return false;
            bool obs_12 = _slip12->use_of_obs(obs_rec, obs_sat, epoch) && (_freq_number[obs_sys] >= 2);
            iter.tb12(obs_12);

            if (_frequency < 3)
                continue;
            bool obs_13 = _slip13->use_of_obs(obs_rec, obs_sat, epoch) && (_freq_number[obs_sys] >= 3);
            iter.tb13(obs_13);

            if (_frequency < 4)
                continue;
            bool obs_14 = _slip14->use_of_obs(obs_rec, obs_sat, epoch) && (_freq_number[obs_sys] >= 4);
            iter.tb14(obs_14);

            if (_frequency < 5)
                continue;
            bool obs_15 = _slip15->use_of_obs(obs_rec, obs_sat, epoch) && (_freq_number[obs_sys] >= 5);
            iter.tb15(obs_15);
        }
        return true;
    }

    bool gnss_proc_lsq::_init_sat_clk_IF(std::vector<gnss_data_sats> &crt_obs, Vector &l, gnss_proc_lsqbase *lsq)
    {
        if (!_glsq || crt_obs.empty())
            return false;
        for (int i = 0; i < crt_obs.size(); i++)
        {
            std::string sat = crt_obs[i].sat();
            int idx_satclk = _glsq->_x_solve.getParam("", par_type::CLK_SAT, sat);
            if (idx_satclk >= 0 && _glsq->_x_solve[idx_satclk].value() == 0.0)
            {
                try
                {
                    // satclk delay = - code_omc
                    double satclk = -(l(2 * i + 1) + 1.0);
                    // code omc
                    l(2 * i + 1) = l(2 * i + 1) + satclk;
                    // phase omc
                    l(2 * i + 2) = l(2 * i + 2) + satclk;
                    // clk par init
                    _glsq->_x_solve[idx_satclk].value(satclk);
                }
                catch (std::exception e)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, e.what());
                    return false;
                }
            }
        }

        return true;
    }

    bool gnss_proc_lsq::_init_sat_clk()
    {
        std::set<std::string> sat_list = _crt_equ.get_satlist(_crt_rec);
        for (auto sat_iter = sat_list.begin(); sat_iter != sat_list.end(); sat_iter++)
        {
            // get satclk par idx
            int idx_satclk = _glsq->_x_solve.getParam("", par_type::CLK_SAT, *sat_iter);
            if (idx_satclk >= 0 && double_eq(_glsq->_x_solve[idx_satclk].value(), 0.0))
            {
                try
                {
                    // get all phase code observ equations;
                    std::vector<int> equ_list = _crt_equ.find_equ(_crt_rec, *sat_iter);
                    double satclk = 0.0;

                    // get satclk delay initial value
                    // TODO: use average function(now use the first code omc as the initial value)
                    for (int idx : equ_list)
                    {
                        if (_crt_equ.get_obscombtype(idx).is_code())
                        {
                            satclk = -(_crt_equ.l[idx] + 1.0);
                            break;
                        }
                    }
                    // clk par init
                    _glsq->_x_solve[idx_satclk].value(satclk);
                    _bias_model->update_obj_clk(*sat_iter, _crt_time, satclk / CLIGHT);

                    // omc = omc + satclk;
                    for (int idx : equ_list)
                    {
                        _crt_equ.l[idx] += satclk;
                    }
                }
                catch (std::exception e)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, e.what());
                    return false;
                }
            }
        }
        return true;
    }

    bool gnss_proc_lsq::_extract_resfile(gnss_all_recover &recover_data, std::string resfile)
    {

        // encode recover file
        base_file gout(_spdlog);
        gnss_coder_resfile grecover_coder(_gset);
        std::string recover_path = dynamic_cast<set_out *>(_gset)->outputs("recover");
        if (recover_path.empty())
            recover_path = dynamic_cast<set_out *>(_gset)->outputs("rcv");
        if (recover_path.empty())
            recover_path = resfile;
        if (recover_path.empty())
            recover_path = "resfile_temp_$(date)";

        base_type_conv::substitute(recover_path, "$(date)", _beg_time.str_yyyydoy(), false);
        base_type_conv::substitute(recover_path, "$(rec)", _crt_rec, false);
        //std::set I/O
        gout.path(recover_path);
        gout.spdlog(_spdlog);
        //std::set coder
        grecover_coder.spdlog(_spdlog);
        grecover_coder.path(recover_path);
        grecover_coder.add_data("ID1", &recover_data);

        gout.coder(&grecover_coder);
        // write
        gout.run_write();

        return true;
    }

    bool gnss_proc_lsq::_extract_clkfile(gnss_all_recover &recover_data, gnss_all_prec::clk_type type)
    {
        // get allprec data
        gnss_all_prec clk_data(_spdlog);

        recover_data.get_clkdata(clk_data, type);

        // encoder data
        base_file gout(_spdlog);
        std::string clk_path = "";
        if (type == gnss_all_prec::AS)
        {
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("satclk");
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("cas");
        }
        else if (type == gnss_all_prec::AR)
        {
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("recclk");
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("car");
        }

        if (clk_path.empty())
        {
            clk_path = "clkfiletemp_$(date)";
        }
        base_type_conv::substitute(clk_path, "$(date)", _beg_time.str_yyyydoy(), false);
        base_type_conv::substitute(clk_path, "$(rec)", _crt_rec, false);

        gout.path(clk_path);
        gout.spdlog(_spdlog);

        gnss_coder_rinexc clk_coder(_gset);
        clk_coder.add_data("ID1", &clk_data);
        gout.coder(&clk_coder);

        gout.run_write();
        return true;
    }

    bool gnss_proc_lsq::_extract_clkfile(gnss_all_recover &recover_data, gnss_all_prec::clk_type type, GSYS gsys, par_type clk_type)
    {
        // get allprec data
        gnss_all_prec clk_data(_spdlog);

        recover_data.get_clkdata(clk_data, type);

        // encoder data
        base_file gout(_spdlog);
        std::string clk_path = "";
        if (type == gnss_all_prec::AS)
        {
            if (clk_path.empty() && clk_type == par_type::CLK13_C)
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("satclk13");
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("cas13");
        }
        else if (type == gnss_all_prec::AR)
        {
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("recclk13");
            if (clk_path.empty())
                clk_path = dynamic_cast<set_out *>(_gset)->outputs("car13");
        }

        if (clk_path.empty())
        {
            clk_path = "clkfiletemp_$(date)";
        }
        base_type_conv::substitute(clk_path, "$(date)", _beg_time.str_yyyydoy(), false);
        base_type_conv::substitute(clk_path, "$(rec)", _crt_rec, false);

        gout.path(clk_path);
        gout.spdlog(_spdlog);

        gnss_coder_rinexc clk_coder(_gset);
        clk_coder.add_data("ID1", &clk_data);
        gout.coder(&clk_coder);

        gout.run_write();
        return true;
    }

    bool gnss_proc_lsq::_extract_clkfile(gnss_all_recover &recover_data, gnss_all_prec::clk_type type, bool isTriple)
    {
        if (!isTriple)
        {
            return _extract_clkfile(recover_data, type);
        }
        // get allprec data
        gnss_all_prec clk_data(_spdlog);

        recover_data.get_clk13data(clk_data, type);

        // encoder data
        base_file gout(_spdlog);
        std::string clk_path = "";
        if (type == gnss_all_prec::AS)
        {
            clk_path = dynamic_cast<set_out *>(_gset)->outputs("satclk");
        }
        else if (type == gnss_all_prec::AR)
        {
            clk_path = dynamic_cast<set_out *>(_gset)->outputs("recclk13");
        }
        if (clk_path.empty())
        {
            clk_path = "clkfiletemp_$(date)";
        }
        base_type_conv::substitute(clk_path, "$(date)", _beg_time.str_yyyydoy(), false);

        gout.path(clk_path);
        gout.spdlog(_spdlog);

        gnss_coder_rinexc clk_coder(_gset);
        clk_coder.add_data("ID1", &clk_data);
        gout.coder(&clk_coder);

        gout.run_write();
        return true;
    }

    bool gnss_proc_lsq::_extract_ambupd(gnss_all_ambupd *allambupd)
    {
        // Encoder ambupd files
        std::string path = dynamic_cast<set_out *>(_gset)->outputs("ambupd");
        std::set<std::string> ambupd_sites = allambupd->get_sites();
        for (const auto site : ambupd_sites)
        {
            if (path.empty())
                path = "file://" + site + "_ambupd_" + _beg_time.str_yyyydoy();
            else
                base_type_conv::substitute(path, "$(rec)", site, false);

            gnss_all_ambupd *outambupd = new gnss_all_ambupd();
            outambupd->addAmbUpd(site, allambupd->getOneSiteAmbUpd(site));
            base_file *gout = new base_file(_spdlog);
            gout->spdlog(_spdlog);
            gout->path(path);

            base_coder *gcoder = new gnss_coder_ambupd(_gset, "", 4096);
            gcoder->clear();
            gcoder->spdlog(_spdlog);
            gcoder->path(path);
            gcoder->add_data("IDX", outambupd);

            gout->coder(gcoder);

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " Writing AMBUPD file : " + path);
            gout->run_write();
            delete gout;
            gout = NULL;
            delete gcoder;
            gcoder = NULL;
            delete outambupd;
            outambupd = NULL;
        }
        delete allambupd;
        allambupd = NULL;

        return true;
    }

    bool gnss_proc_lsq::_extract_ion_file(gnss_all_recover &recover_data)
    {
        gnss_data_ion ion_data;
        std::vector<hwa_TUPLE_gnss_ION> ions;
        recover_data.get_iondata(ions);

        for (const auto &iter : ions)
        {
            gnss_data_ion_record tmp(std::get<0>(iter),
                              std::get<1>(iter), std::get<2>(iter),
                              std::get<3>(iter), std::get<4>(iter),
                              std::get<5>(iter), std::get<6>(iter));

            ion_data.add_record(tmp);
        }

        // encoder data
        base_file gout(_spdlog);
        std::string ion_path = dynamic_cast<set_out *>(_gset)->outputs("ion");

        if (ion_path.empty())
        {
            ion_path = "grt$(date).ion";
        }
        base_type_conv::substitute(ion_path, "$(date)", _beg_time.str_yyyydoy(), false);

        gout.path(ion_path);
        gout.spdlog(_spdlog);

        gnss_coder_ion coder(_gset);
        coder.add_data("ID1", &ion_data);
        gout.coder(&coder);

        gout.run_write();
        return true;
    }

    void gnss_proc_lsq::_set_isb_par_setting(gnss_proc_lsqbase *lsq, base_par &par, SYSBIASMODEL model, double value, double apriori)
    {
        par.value(value);
        par.apriori(apriori);

        if (model == SYSBIASMODEL::AUTO_CON || model == SYSBIASMODEL::ISB_CON)
        {
            par.setTime(_beg_time, _end_time);
            lsq->add_parameter(par);
        }
        else if (model == SYSBIASMODEL::AUTO_RWK || model == SYSBIASMODEL::ISB_RWK)
        {
            par.setTime(_beg_time, _beg_time);
            lsq->add_parameter(par);
            lsq->add_par_state_equ(par.parType, 1, 24, 3000.0);
        }
        else if (model == SYSBIASMODEL::AUTO_WHIT || model == SYSBIASMODEL::ISB_WHIT)
        {
            par.setTime(_beg_time, _beg_time);
            lsq->add_parameter(par);
        }
    }

    int Check_Range(std::vector<double> &omc, double &recclk, double &sigma)
    {
        double mean = 0.0;
        double sig = 0.0;
        std::vector<double> res;

        for (auto it = omc.begin(); it != omc.end();)
        {
            if (fabs(*it) < 10e-8)
            {
                it = omc.erase(it);
                continue;
            }
            mean += *it;
            it++;
        }

        if (omc.size() == 1 || (omc.size() == 2 && fabs(omc[0] - omc[1]) > 1000))
        {
            recclk = 0.0;
            return 1;
        }

        mean /= omc.size();

        for (auto it = omc.begin(); it != omc.end(); it++)
        {
            sig += (*it - mean) * (*it - mean);
            res.push_back(fabs(*it - mean));
        }
        sig = sqrt(sig / (omc.size() - 1));

        double sig1 = 2000;
        double mean1 = 0.0;
        bool isFound = true;

        while (isFound && sig1 > 1000 && omc.size() >= 3)
        {
            auto maxRes = max_element(res.begin(), res.end());
            auto maxPos = omc.begin() + distance(res.begin(), maxRes);
            double MaxValue = *maxPos;

            mean1 = 0.0;
            sig1 = 0.0;
            res.clear();
            for (auto it = omc.begin(); it != omc.end(); it++)
            {
                if (it == maxPos)
                    continue;
                mean1 += *it / (omc.size() - 1);
            }

            for (auto it = omc.begin(); it != omc.end(); it++)
            {
                if (it == maxPos)
                    continue;
                sig1 += (*it - mean1) * (*it - mean1);
                res.push_back(fabs(*it - mean1));
            }
            sig1 = sqrt(sig1 / (omc.size() - 2));

            isFound = fabs(MaxValue - mean1) > 3 * (sig1 > 1000 / 3.0 ? sig1 : 1000 / 3.0);
            if (!isFound)
                continue;
            mean = mean1;
            sig = sig1;
            omc.erase(maxPos);
        }

        recclk = mean;
        sigma = sig;
        return omc.size();
    }
}