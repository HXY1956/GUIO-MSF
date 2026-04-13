#include <algorithm>
#include <set>
#include "hwa_gnss_proc_ClKDIFF.h"
#include "hwa_set_out.h"
#include "hwa_set_gbase.h"

namespace hwa_gnss
{

    gnss_proc_clkdif::gnss_proc_clkdif()
    {
    }

    gnss_proc_clkdif::gnss_proc_clkdif(set_base *set, base_log spdlog)
    {
        _set = set;
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
    }

    gnss_proc_clkdif::~gnss_proc_clkdif()
    {
    }

    bool gnss_proc_clkdif::processBatch()
    {
        std::string function_name = "gnss_proc_clkdif::processBatch";

        if (!_gclkprd || !_gclkref)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : clkdif ==> gnss_proc_clkdif::no comeplete file !");
            return false;
        }
        // get begin/end time,sampling, sat list from xml
        if (!_get_setting())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : clkdif ==> gnss_proc_clkdif::there is something wrong witn xml !");
            return false;
        }
        _set_out();

        //check time
        if (!_check_time(""))
        {
            return false;
        }

        if (!_get_sat())
        {
            return false;
        }

        _init_par();

        base_time epo_now = _beg;
        int nepoch = 0;
        double clk_rms = 0.0;
        double dclk = 0.0;

        // loop by epoch
        while (epo_now < _end)
        {
            int ncount = 0;
            // loop by sat
            for (auto iter = _sat.begin(); iter != _sat.end(); iter++)
            {
                //get clock in rinexc reference file and rinexc product file
                double clk_tmp[2], clkdif;
                if (dynamic_cast<gnss_all_prec *>(_gclkprd)->clk(*iter, epo_now, &clk_tmp[0], &clk_rms, &dclk) == -1 || dynamic_cast<gnss_all_prec *>(_gclkref)->clk(*iter, epo_now, &clk_tmp[1], &clk_rms, &dclk) == -1)
                {
                    clk_tmp[0] = 0.0;
                    clk_tmp[1] = 0.0;
                    _flag[nepoch][ncount] = 0;
                }
                clkdif = clk_tmp[0] - clk_tmp[1];
                clkdif = clkdif * 1e9;
                _res[nepoch][ncount] = clkdif;
                ncount++;
            }
            nepoch++;
            epo_now = epo_now + _sampling;
        }

        //clock difference between satellites
        for (int ncount = 0; ncount < _numsat; ncount++)
        {
            for (int nepoch = 0; nepoch < _nepoch; nepoch++)
            {
                if (ncount == _ncount)
                {
                    break;
                }
                _res[nepoch][ncount] = _res[nepoch][ncount] - _res[nepoch][_ncount];
            }
        }
        //rms statistics
        _rms_statistics();

        //generate result
        _generate_result();
        return true;
    }

    void gnss_proc_clkdif::setCLKPRD(gnss_all_nav *gclkprd)
    {
        _gclkprd = gclkprd;
    }

    void gnss_proc_clkdif::setCLKREF(gnss_all_nav *gclkref)
    {
        _gclkref = gclkref;
    }

    bool gnss_proc_clkdif::_get_setting()
    {
        _beg = dynamic_cast<set_gen *>(_set)->beg();
        _end = dynamic_cast<set_gen *>(_set)->end();
        _sampling = dynamic_cast<set_gen *>(_set)->sampling();

        _end = _beg + ((int)((_end - _beg) / _sampling)) * _sampling;

        _refsat = dynamic_cast<set_gen *>(_set)->refsat();

        // get satlist from xml
        std::set<std::string> sys = dynamic_cast<set_gen *>(_set)->sys();
        if (sys.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : clkdif ==> No satellite systerm, check your xml file(sys) !");
            return false;
        }
        // sats list from gset
        _sat = dynamic_cast<set_gnss *>(_set)->sat();
        if (_sat.empty())
        {
            hwa_map_sats _gnss_sats = gnss_sats();
            for (auto itGNS = sys.begin(); itGNS != sys.end(); ++itGNS)
            {
                GSYS gsys = gnss_sys::str2gsys(*itGNS);
                _sat.insert(_gnss_sats[gsys].begin(), _gnss_sats[gsys].end());
            }
        }
        _numsat = _sat.size();
        return true;
    }

    bool gnss_proc_clkdif::_set_out()
    {
        std::string tmp = dynamic_cast<set_out *>(_set)->outputs("clkdif");

        if (tmp.empty())
        {
            std::string tmp = dynamic_cast<set_out *>(_set)->outputs("dif");
        }

        if (tmp.empty())
        {
            tmp = "clkdif" + base_type_conv::int2str(_beg.year()) + base_type_conv::int2str(_beg.doy(), 3);
        }
        else
        {
            base_type_conv::substitute(tmp, "$(date)", base_type_conv::int2str(_beg.year()) + base_type_conv::int2str(_beg.doy(), 3), false);
        }
        _clkdif = new base_iof;
        _clkdif->mask(tmp);
        _clkdif->append(false);

        return true;
    }

    bool gnss_proc_clkdif::_check_time(std::string sat)
    {
        base_time beg_prd = dynamic_cast<gnss_all_prec *>(_gclkprd)->beg_clk(sat);
        base_time end_prd = dynamic_cast<gnss_all_prec *>(_gclkprd)->end_clk(sat);

        base_time beg_ref = dynamic_cast<gnss_all_prec *>(_gclkref)->beg_clk(sat);
        base_time end_ref = dynamic_cast<gnss_all_prec *>(_gclkref)->end_clk(sat);

        if (_beg < beg_prd)
            _beg = beg_prd;
        if (_beg < beg_ref)
            _beg = beg_ref;
        if (_end > end_prd)
            _end = end_prd;
        if (_end > end_ref)
            _end = end_ref;

        if (_beg >= _end)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : clkdif ==> time is otrange, please chelk time in xml file or rinexc reference file or rinexc product file  !");
            return false;
        }
        return true;
    }

    bool gnss_proc_clkdif::_get_sat()
    {
        std::string function_name = "gnss_proc_clkdif::_get_sat";
        std::set<std::string> sat_prd = dynamic_cast<gnss_all_prec *>(_gclkprd)->clk_objs();
        std::set<std::string> sat_ref = dynamic_cast<gnss_all_prec *>(_gclkref)->clk_objs();
        for (auto iter = _sat.begin(); iter != _sat.end();)
        {
            //skip sat
            if (sat_prd.find(*iter) == sat_prd.end())
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "WARNING : " + *iter + "can not be found in rinexc product file!");
                _sat.erase(iter++);
                continue;
            }
            if (sat_ref.find(*iter) == sat_ref.end())
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "WARNING : " + *iter + "can not be found in rinexc reference file!");
                _sat.erase(iter++);
                continue;
            }
            iter++;
        }
        return true;
    }

    bool gnss_proc_clkdif::_init_par()
    {
        std::string function_name = "gnss_proc_clkdif::_init_par";
        _nepoch = (int)((_end - _beg) / _sampling);
        _res.resize(_nepoch);
        _numsat = _sat.size();
        for (int i = 0; i < _nepoch; i++)
        {
            _res[i].resize(_numsat);
        }

        _flag.resize(_nepoch);
        for (int tmp = 0; tmp < _nepoch; tmp++)
        {
            _flag[tmp].resize(_numsat);
            for (int ncount = 0; ncount < _numsat; ncount++)
            {
                _flag[tmp][ncount] = 1;
            }
        }

        std::set<std::string>::iterator refsat;
        if ((refsat = _sat.find(_refsat)) == _sat.end())
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : clkdif ==> gnss_proc_clkdif::there is no reference satellite in satellite list!");
            return false;
        }
        int ncount = 0;
        for (auto iter = _sat.begin(); iter != _sat.end(); iter++)
        {
            if (*iter == _refsat)
            {
                _ncount = ncount;
                break;
            }
            ncount++;
        }
        return true;
    }

    bool gnss_proc_clkdif::_rms_statistics()
    {
        //3 for MEAN,RMS,STD
        _res_rms.resize(3);
        for (int tmp = 0; tmp < 3; tmp++)
        {
            _res_rms[tmp].resize(_numsat);
        }
        double mean_rms = 0.0;
        double mean_std = 0.0;
        //1 for DEL
        _res_del.resize(_numsat);

        for (int ncount = 0; ncount < _numsat;)
        {
            double mean = 0.0;
            double rms = 0.0;
            double std = 0.0;
            int m = 0;
            int ndel = 0;

            for (int nepoch = 0; nepoch < _nepoch; nepoch++)
            {
                if (_flag[nepoch][ncount])
                {
                    if (nepoch >= 1)
                    {
                        if (_res[nepoch][ncount] - _res[nepoch - 1][ncount] > 100.0)
                        {
                            _flag[nepoch][ncount] = 0;
                            continue;
                        }
                    }
                    mean = mean + _res[nepoch][ncount];
                    rms = rms + _res[nepoch][ncount] * _res[nepoch][ncount];
                    m++;
                }
            }
            mean = mean / m;

            for (int nepoch = 0; nepoch < _nepoch; nepoch++)
            {
                if (_flag[nepoch][ncount])
                {
                    std = std + (_res[nepoch][ncount] - mean) * (_res[nepoch][ncount] - mean);
                }
            }

            if (m == 1)
            {
                rms = 999;
                std = 999;
            }
            else
            {
                rms = sqrt(rms / m);
                std = sqrt(std / (m - 1));
            }

            for (int nepoch = 0; nepoch < _nepoch; nepoch++)
            {
                if (!_flag[nepoch][ncount] || fabs(_res[nepoch][ncount] - mean) <= 3 * std)
                    continue;
                _flag[nepoch][ncount] = 0;
                ndel++;
            }
            _res_del[ncount] = _res_del[ncount] + ndel;

            if (!ndel)
            {
                _res_rms[0][ncount] = mean;
                _res_rms[1][ncount] = rms;
                _res_rms[2][ncount] = std;
                if (ncount != _ncount)
                {
                    mean_rms = mean_rms + _res_rms[1][ncount];
                    mean_std = mean_std + _res_rms[2][ncount];
                }
                ncount++;
            }
        }
        mean_rms = mean_rms / (_numsat - 1);
        mean_std = mean_std / (_numsat - 1);
        _mean_rms = mean_rms;
        _mean_std = mean_std;

        return true;
    }

    bool gnss_proc_clkdif::_generate_result()
    {
        std::ostringstream os;
        os << "  MJD       SOD";
        for (const auto &sat : _sat)
            os << std::setw(9) << std::right << sat;
        os << std::endl;
        for (int nepoch = 0; nepoch < _nepoch; nepoch++)
        {
            base_time Te = _beg + (nepoch * _sampling);
            os << std::setw(5) << std::right << Te.mjd() << std::setw(10) << Te.sod();
            // os << std::setw(15) << right << nepoch + 1;
            for (int i = 0; i < _sat.size(); i++)
            {
                os << " ";
                _double2sstream(os, _res[nepoch][i], 8, 2);
                // os << std::setw(9) << std::fixed << std::setprecision(2) << right << _res[nepoch][i];
            }
            os << std::endl;
        }
        os << "-------------------------------------------" << std::endl;
        os << std::setw(15) << std::left << "MEAN";
        for (int ncount = 0; ncount < _numsat; ncount++)
        {
            os << " ";
            _double2sstream(os, _res_rms[0][ncount], 8, 2);
            // os << std::setw(9) << std::fixed << std::setprecision(2) << right << _res_rms[0][ncount];
        }
        os << std::endl;
        os << std::setw(15) << std::left << "RMS:";
        for (int ncount = 0; ncount < _numsat; ncount++)
        {
            os << " ";
            _double2sstream(os, _res_rms[1][ncount], 8, 2);
            // os << std::setw(9) << std::fixed << std::setprecision(2) << right << _res_rms[1][ncount];
        }
        os << std::endl;
        os << std::setw(15) << std::left << "STD:";
        for (int ncount = 0; ncount < _numsat; ncount++)
        {
            os << " ";
            _double2sstream(os, _res_rms[2][ncount], 8, 2);
            // os << std::setw(9) << std::fixed << std::setprecision(2) << right << _res_rms[2][ncount];
        }
        os << std::endl;
        os << std::setw(15) << std::left << std::setfill(' ') << "DEL:";
        for (int ncount = 0; ncount < _numsat; ncount++)
        {
            os << std::setw(9) << std::right << _res_del[ncount];
        }
        os << std::endl;
        os << std::setw(15) << std::left << "NAME";
        for (const auto &sat : _sat)
            os << std::setw(9) << std::right << sat;
        os << std::endl;
        os << std::setw(3) << std::right << "EOF" << std::endl;
        os << " MEAN RMS =";
        os << std::setw(7) << std::fixed << std::setprecision(2) << std::right << _mean_rms;
        os << "  MEAN STD";
        os << std::setw(7) << std::fixed << std::setprecision(2) << std::right << _mean_std;
        os << std::endl;

        _clkdif->write(os.str().c_str(), os.str().size());
        _clkdif->flush();
        if (_clkdif)
        {
            delete _clkdif;
            _clkdif = NULL;
        }

        return true;
    }
}