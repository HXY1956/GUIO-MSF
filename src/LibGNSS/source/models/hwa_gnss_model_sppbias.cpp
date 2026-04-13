#include "hwa_gnss_model_Sppbias.h"
#include "hwa_gnss_model_gmf.h"

using namespace std;

namespace hwa_gnss
{
    hwa_gnss::gnss_proc_sppbias::gnss_proc_sppbias(base_log spdlog, set_base *settings)
    {
    }

    hwa_gnss::gnss_proc_sppbias::~gnss_proc_sppbias()
    {
    }

    bool gnss_proc_sppbias::cmb_equ(base_time &epoch, base_allpar &params, gnss_data_sats &obsdata, gnss_data_obs &gobs, gnss_model_base_equation &result)
    {
        return false;
    }

    void gnss_proc_sppbias::update_obj_clk(const std::string &obj, const base_time &epo, double clk)
    {
    }

    double gnss_proc_sppbias::get_rec_clk(const std::string &obj)
    {
        return 0.0;
    }

    double gnss_proc_sppbias::tropoDelay(base_time &epoch, std::string &rec, base_allpar &param, Triple ell, gnss_data_sats &satdata)
    {
        if (_tropoModel == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "Tropo Model setting is not correct. Default used! Check config.");
            throw std::runtime_error("Can not find _tropoModel !");
        }

        double ele = satdata.ele();
        double delay = 0.0;
        double zwd = 0.0;
        double zhd = 0.0;

        if (abs(ell[2]) > 1E4)
        {
            return 0.0;
        }

        int i = param.getParam(rec, par_type::TRP, "");
        if (i >= 0)
        {
            zwd = param[i].value();
            zhd = param[i].apriori();
        }
        else
        {
            if (_tropoModel != 0)
            {
                zwd = _tropoModel->getZWD(ell, epoch);
                zhd = _tropoModel->getZHD(ell, epoch);
            }
        }

        if (_mf_ztd == ZTDMPFUNC::GMF)
        {
            double gmfh, gmfw, dgmfh, dgmfw;
            gnss_model_gmf mf;
            mf.gmf(epoch.mjd(), ell[0], ell[1], ell[2], hwa_pi / 2.0 - ele,
                   gmfh, gmfw, dgmfh, dgmfw);
            //      delay = gmfh * _tropoModel->getZHD(ell, epoch) + gmfw * zwd;
            delay = gmfh * zhd + gmfw * zwd;

#ifdef DEBUG
            std::cout << epoch.str("EPOCH: %H:%M:%S") << std::endl
                 << std::fixed << std::setprecision(3);
            std::cout << "Ell:" << ell[0] << " " << ell[1] << " " << ell[2]
                 << " Hydrostatic part: " << zhd // _tropoModel->getZHD(ell, epoch)
                 << " Wet part: " << zwd
                 << " gmfh: " << gmfh
                 << " gmfw: " << gmfw
                 << " Delay: " << delay << std::endl
                 << std::endl;
            int ooo;
            cin >> ooo;
#endif
        }
        else if (_mf_ztd == ZTDMPFUNC::COSZ)
        {
            double mf = 1 / sin(ele);
            //      delay = 1/sin(ele) * _tropoModel->getZHD(ell, epoch) +1/sin(ele) * zwd;
            delay = mf * zhd + mf * zwd;
        }

        return delay;
    }

    double gnss_proc_sppbias::ionoDelay(base_time &epoch, base_allpar &param, Triple site_ell, gnss_data_sats &satdata, gnss_data_obs &gobs)
    {
        return 0.0;
    }

    double gnss_proc_sppbias::isbDelay(base_allpar &param, std::string &sat, std::string &rec, gnss_data_obs &gobs)
    {
        return 0.0;
    }

    double gnss_proc_sppbias::ifbDelay(base_allpar &param, std::string &sat, std::string &rec, gnss_data_obs &gobs)
    {
        return 0.0;
    }

    double gnss_proc_sppbias::cmpObs(base_time &epo, std::string &sat, std::string &rec, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs)
    {

        // Cartesian coordinates to ellipsodial coordinates
        Triple xyz;
        Triple ell;

        // give crd initial value modified by glfeng
        if (param.getCrdParam(rec, xyz) < 0)
        {
            throw std::runtime_error("Can not find CRD par in base_allpar !");
        }
        xyz2ell(xyz, ell, false);

        Triple satcrd = gsatdata.satcrd();
        Triple cSat = satcrd;

        // Tropospheric wet delay correction
        double trpDelay = 0;
        trpDelay = tropoDelay(epo, rec, param, ell, gsatdata);
        if (fabs(trpDelay) > 50)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "trpDelay > 50");
            return -1;
        }

        // Receiver clock correction
        double clkRec = 0.0;
        int i = param.getParam(rec, par_type::CLK, "");
        if (i >= 0)
        {
            clkRec = param[i].value();
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, rec + " ! warning:  Receiver Clock is not included in parameters!");
        }

        // system time offset
        double isb_offset = isbDelay(param, sat, rec, gobs);

        // jdhuang : DCB is correct by apply DCB all
        double ifb_offset = ifbDelay(param, sat, rec, gobs);

#ifdef DEBUG_LEO
        std::cout << std::setw(20) << " EPOCH is :     " << epoch.str_mjdsod() << " " << std::setw(6) << gsatdata.sat() << std::setw(6) << gsatdata.site()
             << std::setw(20) << " Band is :      " << std::setw(6) << gobs.band()
             << std::setw(20) << " gsatdata.rho() " << std::fixed << left << std::setw(20) << std::setprecision(6) << gsatdata.rho()
             << std::setw(20) << " clkRec         " << std::fixed << left << std::setw(20) << std::setprecision(6) << clkRec
             << std::setw(20) << " gsatdata.clk() " << std::fixed << left << std::setw(20) << std::setprecision(6) << gsatdata.clk()
             << std::setw(20) << " trpDelay       " << std::fixed << left << std::setw(20) << std::setprecision(6) << trpDelay
             << std::endl;
#endif
        // Return value
        return gsatdata.rho() +
               clkRec -
               gsatdata.clk() +
               trpDelay +
               isb_offset +
               ifb_offset;
    }
}
