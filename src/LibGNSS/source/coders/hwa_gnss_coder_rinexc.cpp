#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include "hwa_gnss_coder_rinexc.h"
#include "hwa_gnss_all_prec.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_rinexc::gnss_coder_rinexc(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz),
          _allobj(0)
    {
        _gnsssys = 'G';
    }

    /* ----------
 * CLK-RINEX header
 */
    int gnss_coder_rinexc::decode_head(char *buff, int sz, vector<string> &errmsg)
    {

        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        //  double a0, a1;
        //  int t,w;

        string tmp;
        int consume = 0;
        int tmpsize = 0;
        while ((tmpsize = gnss_base_coder::_getline(tmp)) >= 0)
        {

            consume += tmpsize;
            if (tmp.find("RINEX VERSION", 60) != string::npos)
            { // first line
                _version = tmp.substr(0, 9);
                if (base_type_conv::substitute(_version, " ", "") > 0)
                {
                    std::ostringstream ltmp;
                    ltmp << "version = " << _version;
                    if (_spdlog)
                        SPDLOG_LOGGER_INFO(_spdlog, ltmp.str());
                }
                double ver;
                ver = base_type_conv::str2dbl(_version);
                if ((tmp[20] != 'C' && ver < 3.04) || (tmp[21] != 'C' && ver == 3.04) || ver > 3.04)
                {
                    string ltmp("warning - not rinex clock data file");
                    if (_spdlog)
                        SPDLOG_LOGGER_WARN(_spdlog, tmp);
                    _mutex.unlock();
                    return -1;
                }

                _gnsssys = tmp[41];
            }
            else if (tmp.find("PGM / RUN BY / DATE", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "PGM / RUN BY / DATE");
            }
            else if (tmp.find("COMMENT", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "COMMENT");
            }
            else if (tmp.find("SYS / # / OBS TYPES", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " SYS / # / OBS TYPES");
            }
            else if (tmp.find("TIME SYSTEM ID", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " TIME SYSTEM ID");
            }
            else if (tmp.find("LEAP SECONDS", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " LEAP SECONDS");
            }
            else if (tmp.find("SYS / DCBS APPLIED", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " SYS DCBS APPLIED");
            }
            else if (tmp.find("SYS / PCVS APPLIED", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " SYS PCVS APPLIED");
            }
            else if (tmp.find("# / TYPES OF DATA", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " TYPES OF DATA");
            }
            else if (tmp.find("STATION NAME / NUM", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " STATION NAME");
            }
            else if (tmp.find("STATION CLK REF", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " STATION CLK REF");
            }
            else if (tmp.find("ANALYSIS CENTER", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " ANALYSIS CENTER");
            }
            else if (tmp.find("# OF CLK REF", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " # OF CLK REF");
            }
            else if (tmp.find("ANALYSIS CLK REF", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " ANALYSIS CLK REF");
            }
            else if (tmp.find("# OF SOLN STA / TRF", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " # OF SOLN STA / TRF");
            }
            else if (tmp.find("SOLN STA NAME / NUM", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " SOLN STA NAME / NUM");
            }
            else if (tmp.find("# OF SOLN SATS", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " # OF SOLN SATS");
            }
            else if (tmp.find("PRN LIST", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " PRN LIST");

                /*
      a0 = base_type_conv::str2dbl(tmp.substr( 3,  19 );
      a1 = base_type_conv::str2dbl(tmp.substr( 22, 19 );
      w  = base_type_conv::str2int(tmp.substr( 42,  8 ));
      t  = base_type_conv::str2int(tmp.substr( 53,  6 ));
      if( _log ) _log->comment(2,"rinexc"," DELTA-UTC");
#ifdef DEBUG
      std::cout << "--- set DELTA-UTC: " << a0 << " : " << a1 << " : " << t << " : " << w << endl;
#endif
*/
            }
            else if (tmp.find("END OF HEADER", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " END OF HEADER ");
                gnss_base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }

            gnss_base_coder::_consume(tmpsize);
        }

        _mutex.unlock();
        return consume;
    }

    /* ----------
 * CLK-RINEX body
 */
    int gnss_coder_rinexc::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {

        base_time epoch(base_time::GPS); // epoch
        double data[6];              // clk/rms values (off,vel,acc)
        string flg, id, line;
        int tmpsize = 0; // individual reading counter
        int consume = 0; // total read counter

        for (int i = 0; i < 6; i++)
            data[i] = UNDEFVAL_CLK; // initialize

        _mutex.lock();

        // complete main buffer
        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        while ((tmpsize = gnss_base_coder::_getline(line)) >= 0)
        {

            std::istringstream istr(line);
            istr.clear();

            int yr, mn, dd, hr, mi;
            double sc;
            int ncol = 1;

            // new record
            istr >> flg;

            if (flg.find("AS") != string::npos)
            {
                char sat[3 + 1];
                sat[3] = '\0';
                char dummy;
                istr >> noskipws >> dummy >> sat[0] >> sat[1] >> sat[2] >> skipws >> yr >> mn >> dd >> hr >> mi >> sc >> ncol;
                id = gnss_sys::eval_sat(string(sat));
            }
            else
            {
                istr >> id >> yr >> mn >> dd >> hr >> mi >> sc >> ncol;
            }

            if (istr.fail())
                break;

            epoch.from_ymd(yr, mn, dd, (hr * 3600 + mi * 60), sc);

            // loop over n-column
            for (int i = 0; i < ncol; i++)
            {

                // first line
                istr >> data[i];
                if (istr.fail())
                    break;

                // additional (second) line
                // xiongyun:change 2 -> 3
                if (i > 3)
                {
                    if ((tmpsize += gnss_base_coder::_getline(line)) < 0)
                        break;

                    istr >> data[i];
                    if (istr.fail())
                        break;
                }
            }

            // ZHJ:add for recclk
            if ((!_data.empty()) && (flg.find("AS") != string::npos || flg.find("AR") != string::npos))
            {

                double clk[3] = {data[0], data[2], data[4]};
                double var[3] = {data[0], data[2], data[4]};
                // xiongyun:add clk_tri for ifcb
                double clk_tri[4] = {data[0], data[1], data[2], data[3]};

                // fill clk
                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {

                    if (it->second->id_type() == base_data::ALLPREC)
                    {
                        if (data[3] == UNDEFVAL_CLK)
                        {
                            ((gnss_all_prec *)it->second)->addclk(id, epoch, clk, var);
                        }
                        else
                        {
                            ((gnss_all_prec *)it->second)->addclk_tri(id, epoch, clk_tri, var);
                        }
                    }

                    it++;
                }
                cnt++;
            }
            consume += gnss_base_coder::_consume(tmpsize);
        }

        _mutex.unlock();
        return consume;
    }

    int gnss_coder_rinexc::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        gnss_all_prec *clkdata = nullptr;

        if (!(clkdata = _get_prec_data()))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "[gnss_coder_rinexc::encode_head][ERROR]:Can't get the clk data!");
            _mutex.unlock();
            return -1;
        }

        set<gnss_all_prec::clk_type> types = clkdata->get_clk_type();
        if (types.empty())
        {
            _mutex.unlock();
            return -1;
        }

        string strAR = (types.find(gnss_all_prec::clk_type::AR) != std::end(types)) ? "AR" : "";
        string strAS = (types.find(gnss_all_prec::clk_type::AS) != std::end(types)) ? "AS" : "";

        clkdata->add_interval(_int);
        double intv = clkdata->intv();

        if (_ss_position == 0)
        {
            _ss << setiosflags(ios::fixed) << setprecision(2)
                << setiosflags(ios::right) << setw(9) << 2.0 << setw(11) << ""
                << setw(1) << "C" << setw(19) << ""
                << setw(1) << "G" << setw(19) << ""
                << "RINEX VERSION / TYPE" << endl;

            _ss << setiosflags(ios::right) << setw(9)
                << std::fixed << setprecision(2) << intv
                << setw(51) << " "
                << "INTERVAL" << endl;

            _ss << setiosflags(ios::left)
                << setw(20) << "GREAT"
                << setw(20) << "SGG-WHU"
                << setw(16) << base_time(base_time::UTC).str("%Y%m%d %H%M%S")
                << setw(4) << "UTC"
                << "PGM / RUN BY / DATE " << endl;

            _ss << setw(6) << clkdata->get_clk_type().size();
            if (strAR.empty())
            {
                _ss << setw(4) << "" << setw(2) << strAS
                    << setw(4) << "" << setw(2) << strAR;
            }
            else
            {
                _ss << setw(4) << "" << setw(2) << strAR
                    << setw(4) << "" << setw(2) << strAS;
            }
            _ss << setw(42) << ""
                << "# / TYPES OF DATA   " << endl;

            base_time beg_time = clkdata->beg_clk();
            base_time end_time = clkdata->end_clk();
            _ss << setw(6) << 1 << setw(1) << ""
                << beg_time.str("%Y %C %L %K %O %P")
                << setw(1) << ""
                << end_time.str("%Y %C %L %K %O %P")
                << "# OF CLK REF" << endl;

            _ss << setw(60) << ""
                << "END OF HEADER       " << endl;
        }

        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

    int gnss_coder_rinexc::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        gnss_all_prec *clkdata = nullptr;

        if (!(clkdata = _get_prec_data()))
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "[gnss_coder_rinexc::encode_head][ERROR]:Can't get the clk data!");
            _mutex.unlock();
            return -1;
        }

        if (_ss_position == 0)
        {
            auto epochs = clkdata->clk_epochs();
            auto objs = clkdata->clk_objs();

            for (auto epoch : epochs)
            {
                for (auto obj : objs)
                {

                    double data[4] = {0.0, 0.0, 0.0, 0.0}; //data[3] -> data[4] by xiongyun
                    clkdata->clk_cdr(obj, epoch, data, data + 1, data + 2, data + 3);

                    if (double_eq(0.0, data[0]))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_INFO(_spdlog, "The clk is Losing for " + epoch.str_mjdsod(obj));
                    }

                    int col = 1;
                    if (data[3] != 0.0)
                    {
                        col = 4;
                    }
                    else if (data[2] != 0.0)
                    {
                        col = 3;
                    }
                    else if (data[1] != 0.0)
                    {
                        col = 2;
                    }
                    else if (data[0] == 0.0)
                    {
                        continue;
                    }

                    if (obj.length() == 3)
                    {
                        _ss << "AS ";
                    }
                    else
                    {
                        _ss << "AR ";
                    }
                    _ss << setw(4) << left << obj << setw(1) << "";

                    _ss << setw(26) << epoch.str("%Y %C %L %K %O %P")
                        << setw(3) << right << col << setw(2) << "";

                    for (int i = 0; i < col; i++)
                    {
                        _ss << setw(20) << right << setprecision(12) << scientific << uppercase << data[i];
                    }

                    _ss << endl;
                }
            }
        }

        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

    gnss_all_prec *gnss_coder_rinexc::_get_prec_data()
    {
        for (auto iter = _data.begin(); iter != _data.end(); iter++)
        {
            if (iter->second->id_type() == base_data::ALLPREC)
            {
                return dynamic_cast<gnss_all_prec *>(iter->second);
            }
        }
        return nullptr;
    }

} // namespace
