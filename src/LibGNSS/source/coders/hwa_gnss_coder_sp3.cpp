#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include "hwa_gnss_all_prec.h"
#include "hwa_gnss_coder_sp3.h"
#include "hwa_base_typeconv.h"
#include "spdlog.h"
using namespace std;

namespace hwa_gnss
{
    gnss_coder_sp3::gnss_coder_sp3(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {

        _start.tsys(base_time::GPS);
        _lastepo.tsys(base_time::GPS);
        _nrecord = -1;
        _nrecmax = -1;
    }

    /* ----------
 * SP3 header
 */
    int gnss_coder_sp3::decode_head(char *buff, int sz, vector<string> &errmsg)
    {

        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        string tmp;
        int consume = 0;
        int tmpsize = 0;

        while ((tmpsize = gnss_base_coder::_getline(tmp)) >= 0)
        {

            consume += tmpsize;

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Current Line is : {}", tmp);
            // first information
            if (tmp.substr(0, 1) == "#")
            {
                // first line
                if (tmp.substr(1, 1) == "a" || // 60-columns
                    tmp.substr(1, 1) == "c"    // 80-columns
                )
                {

                    _version = tmp.substr(1, 1);
                    //_start.from_str( "%Y %m %d  %H %M %S", tmp.substr(3,28) ); //yjqin
                    _start.from_str("%Y %m %d  %H %M %S", tmp.substr(3, 30));
                    _nepochs = base_type_conv::str2int(tmp.substr(32, 7));
                    _orbrefs = tmp.substr(46, 5);
                    _orbtype = tmp.substr(52, 3);
                    if (tmp.size() > 60)
                    {
                        _agency = tmp.substr(56, 4);
                    }
                    else
                    {
                        _agency = "";
                    }
                    // second line
                }
                else if (tmp.substr(1, 1) == "#")
                {
                    _orbintv = (long)base_type_conv::str2dbl(tmp.substr(24, 14)); // [sec]
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "start time = {}", _start.str(" %Y-%m-%d %H:%M:%S"));
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "refs       = {}", _orbrefs);
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "type       = {}", _orbtype);
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "intv       = {}", _orbintv);
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "nepo       = {}", _nepochs);
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_WARN(_spdlog, "unknown record");
                }
            }
            else if (tmp.substr(0, 2) == "+ ")
            {
                std::ostringstream ltmp("reading satellites:");
                for (unsigned int i = 9; i < 60 && i + 3 < tmp.size(); i = i + 3)
                {
                    string ssat = tmp.substr(i, 3);

                    if (base_type_conv::trim(ssat) == "00")
                    {
                        continue;
                    }
                    if (ssat.substr(0, 2) == "  ")
                    {
                        ssat = "G0" + ssat.substr(2, 1);
                    }
                    else if (ssat.substr(0, 1) == " ")
                    {
                        ssat = "G" + ssat.substr(1, 2);
                    }
                    _prn.push_back(ssat);
                    ltmp << " " << tmp.substr(i, 3);
                }
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "decoder for {} ", ltmp.str());
            }
            else if (tmp.substr(0, 2) == "++")
            {
                std::ostringstream ltmp("reading accuracies:");
                for (unsigned int i = 9; i < 60 && i + 3 < tmp.size(); i = i + 3)
                {
                    _acc.push_back(base_type_conv::str2int(tmp.substr(i, 3)));
                    ltmp << " " << tmp.substr(i, 3);
                }
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "decoder for {} ", ltmp.str());
            }
            else if (tmp.substr(0, 2) == "%c")
            {
                if (tmp.substr(3, 1) == "L")
                {
                    std::istringstream istr(tmp.substr(4));
                    string ileo;
                    while (istr >> ileo)
                    {
                        _leo[_prn[_num_leo]] = ileo;
                        _num_leo++;
                    }
                }
                else
                {
                    _timesys.push_back(tmp.substr(3, 2));
                    _timesys.push_back(tmp.substr(6, 2));
                    _timesys.push_back(tmp.substr(9, 3));
                    _timesys.push_back(tmp.substr(13, 3));
                    _timesys.push_back(tmp.substr(17, 4));
                    _timesys.push_back(tmp.substr(22, 4));
                    _timesys.push_back(tmp.substr(27, 4));
                    _timesys.push_back(tmp.substr(32, 4));
                    _timesys.push_back(tmp.substr(37, 5));
                    _timesys.push_back(tmp.substr(43, 5));
                    _timesys.push_back(tmp.substr(49, 5));
                    _timesys.push_back(tmp.substr(55, 5));
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "reading satellite systems");
                }
            }
            else if (tmp.substr(0, 2) == "%f")
            {
                _accbase.push_back(base_type_conv::str2int(tmp.substr(3, 9)));
                _accbase.push_back(base_type_conv::str2int(tmp.substr(14, 9)));
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "reading PV base");
            }
            else if (tmp.substr(0, 2) == "%i")
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "additional info");
            }
            else if (tmp.substr(0, 2) == "/*")
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "comments");
            }
            else if (tmp.substr(0, 2) == "* ")
            { // END OF HEADER !!!
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "END OF HEADER");
                _mutex.unlock();
                return -1;
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "warning: unknown header message");
            }

            gnss_base_coder::_consume(tmpsize);
        }

        _mutex.unlock();
        return consume;
    }

    /* ----------
 * SP3 body
 */
    int gnss_coder_sp3::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {

        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        string tmp;
        //  int consume = 0;
        //  int recsize = 0;
        int tmpsize = 0;
        //  bool epoch_defined = false;

        //  while( ( tmpsize = gnss_base_coder::_getline( tmp, recsize ) ) >= 0 ){
        while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
        {

#ifdef DEBUG
            std::cout << " 0: " << tmp;
#endif

            // EPOCH record
            if (tmp.substr(0, 1) == "*" ||
                tmp.substr(0, 3) == "EOF")
            {

                if (tmp.substr(0, 3) == "EOF")
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "EOF found");
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return tmpsize;
                }

                if (_nrecord > 0 && _nrecord != _nrecmax)
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_WARN(_spdlog, "not equal number of satellites _nrecord is {}, _nrecmax is {}!", _nrecord, _nrecmax);
                }

                char dummy;
                int yr, mn, dd, hr, mi;
                double sc;
                stringstream ss(tmp);
                ss >> dummy >> yr >> mn >> dd >> hr >> mi >> sc;

                if (ss.fail())
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_CRITICAL(_spdlog, "incorrect SP3 epoch record : {}!", ss.str().c_str());
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }

                int sod = hr * 3600 + mi * 60 + (int)sc;
                _lastepo.from_ymd(yr, mn, dd, sod);

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "reading EPOCH [{}] - {} ", _nrecord, _lastepo.str(" %Y-%m-%d %H:%M:%S"));
                _nrecord = -1;
            }

            // POSITION reccord
            if (tmp.substr(0, 1) == "P")
            { // and epoch_defined ){

                Triple xyz(0.0, 0.0, 0.0);
                Triple dxyz(0.0, 0.0, 0.0);
                double t = 0.0, dt = 0.9;

                char sat[3 + 1];
                sat[3] = '\0';
                char flg;
                double pos[4] = {0.0, 0.0, 0.0, 0.0};
                //      double var[4] = {0.0, 0.0, 0.0, 0.0};
                stringstream ss(tmp);

                ss >> noskipws >> flg >> sat[0] >> sat[1] >> sat[2] >> skipws >> pos[0] >> pos[1] >> pos[2] >> pos[3];
                string prn;
                if (_leo.size() == 0 && _num_leo == 0)
                {

                    prn = gnss_sys::eval_sat(string(sat));
                }
                else
                {
                    prn = _leo[sat];
                }

                for (int i = 0; i < 3; i++)
                    if (pos[i] == 0.0)
                        xyz[i] = UNDEFVAL_POS; // gephprec
                    else
                        xyz[i] = pos[i] * 1000; // km -> m

                if (pos[3] > 999999)
                    t = UNDEFVAL_CLK; // gephprec
                else
                    t = pos[3] / 1000000; // us -> s

#ifdef DEBUG
                std::cout << " reading:" << _lastepo.str(" %Y-%m-%d %H:%M:%S ") << flg << " " << prn << " " << sat
                     << " " << pos[0] << " " << pos[1] << " " << pos[2] << " " << pos[3] * 1000000 << endl;
#endif

                // fill single data record
                if (_leo.size() == 0 && _num_leo == 0)
                {
                    if (!_filter_gnss(prn))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "skip satellite : {} for you do not set at the xml file", prn);
                    }
                    else
                    {
                        map<string, base_data *>::iterator it = _data.begin();
                        while (it != _data.end())
                        {
                            if (it->second->id_type() == base_data::ALLPREC)
                            {
                                ((gnss_all_prec *)it->second)->addpos(prn, _lastepo, xyz, t, dxyz, dt);
                                ((gnss_all_prec *)it->second)->add_interval(prn, _orbintv);
                                ((gnss_all_prec *)it->second)->add_agency(_agency);
                            }

                            it++;
                        }
                    }
                }
                else
                {
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLPREC)
                        {
                            ((gnss_all_prec *)it->second)->addpos(prn, _lastepo, xyz, t, dxyz, dt);
                            ((gnss_all_prec *)it->second)->add_interval(prn, _orbintv);
                            ((gnss_all_prec *)it->second)->add_agency(_agency);
                        }

                        it++;
                    }
                }

                if (ss.fail())
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "incorrect SP3 data record: {}", ss.str());
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                cnt++;
            }

            // VELOCITY reccord
            if (tmp.substr(0, 1) == "V")
            { // and epoch_defined ){

                char sat[3 + 1];
                sat[3] = '\0';
                char flg;
                double vel[4] = {0.0, 0.0, 0.0, 0.0};
                double var[4] = {0.0, 0.0, 0.0, 0.0};

                stringstream ss(tmp);
                ss >> noskipws >> flg >> sat[0] >> sat[1] >> sat[2] >> skipws >> vel[0] >> vel[1] >> vel[2] >> vel[3];

                string prn;
                if (_leo.size() == 0 && _num_leo == 0)
                {

                    prn = gnss_sys::eval_sat(string(sat));
                }
                else
                {
                    prn = _leo[sat];
                }

                // fill single data record
                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (_leo.size() == 0 && _num_leo == 0)
                    {
                        if (!_filter_gnss(prn))
                        {

                            it++;
                        }
                        else
                        {
                            //      map<string,base_data*>::iterator it = _data.begin();
                            //        while( it != _data.end() ){
                            if (it->second->id_type() == base_data::ALLPREC)
                            {
                                ((gnss_all_prec *)it->second)->addvel(prn, _lastepo, vel, var);
                            }
                            it++;
                            //     }
                        }
                    }
                    else
                    {
                        if (it->second->id_type() == base_data::ALLPREC)
                        {
                            ((gnss_all_prec *)it->second)->addvel(prn, _lastepo, vel, var);
                        }
                        it++;
                    }

                    // fill single data record
                }

                if (ss.fail())
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "incorrect SP3 data record: {}", ss.str());
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
            }

            _nrecord++;

            if (_nrecord > _nrecmax)
                _nrecmax = _nrecord;

            gnss_base_coder::_consume(tmpsize);
        }
        _mutex.unlock();
        return tmpsize;
    }

    //yjqin:add

    int gnss_coder_sp3::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        // init ss position
        _mutex.lock();
        int gpsweek = 0;
        int sow = 0;
        int year=0;
        int month = 0;
        int day = 0;
        int hour=0;
        int minute = 0;
        int sec = 0;

        double dsec = 0.0;
        string data_used = "ORBIT"; //track
        string frame = "IGb14";
        string ctype = "NAV";
        string agency = "GREAT";
        string time_type = "GPST";
        vector<string> acc;
        int nsats = 0;
        try
        {
            if (_ss_position == 0)
            {

                // initial val from data
                for (auto data_iter = _data.begin(); data_iter != _data.end(); data_iter++)
                {
                    if (data_iter->second->id_type() != base_data::ALLPREC)
                        continue;

                    _start = ((gnss_all_prec *)data_iter->second)->get_beg();
                    _lastepo = ((gnss_all_prec *)data_iter->second)->get_end();

                    gpsweek = _start.gwk();
                    sow = _start.sow();
                    _start.ymd(year, month, day);
                    _start.hms(hour, minute, sec);
                    dsec = sec * 1.0;

                    _sagnss_coder_sp3= ((gnss_all_prec *)data_iter->second)->get_sats();

                    _prn = ((gnss_all_prec *)data_iter->second)->get_sat3();
                    set<string> sp3_sat = ((gnss_all_prec *)data_iter->second)->satellites();
                    _sagnss_coder_sp3.assign(sp3_sat.begin(), sp3_sat.end());
                    _prn.assign(sp3_sat.begin(), sp3_sat.end());
                    _data_type = ((gnss_all_prec *)data_iter->second)->get_data_type();
                    _sattype = ((gnss_all_prec *)data_iter->second)->get_sat_type();
                    _orbintv = ((gnss_all_prec *)data_iter->second)->intv(_prn[0]);
                    _nepochs = int((_lastepo - _start) / _orbintv) + 1;
                    if (double_eq(_orbintv, 0.0))
                    {
                        _orbintv = ((gnss_all_prec *)data_iter->second)->intv();
                    }
                    agency = ((gnss_all_prec *)data_iter->second)->get_agency();
                    _maxsats = _prn.size();
                    nsats = _sagnss_coder_sp3.size();

                    for (int ii = 0; ii < _maxsats; ii++)
                    {
                        acc.push_back("  0");
                    }
                }
                _ss << "#a" << _data_type
                    << setw(4) << right << year
                    << setw(3) << right << month
                    << setw(3) << right << day
                    << setw(3) << right << hour
                    << setw(3) << right << minute
                    << setw(12) << std::fixed << setprecision(8) << showpoint << right << dsec
                    << " " << setw(7) << right << _nepochs
                    << " " << setw(5) << right << data_used
                    << " " << setw(5) << right << frame
                    << " " << setw(3) << right << ctype;
                if (_sattype == "LEO")
                    _ss << " " << setw(3) << right << agency;

                _ss << " " << setw(4) << right << time_type << endl;
                _ss << "## " << setw(4) << right << gpsweek
                    << setw(16) << std::fixed << setprecision(8) << showpoint << right << double(sow)
                    << setw(15) << std::fixed << setprecision(8) << showpoint << right << double(_orbintv)
                    << " " << setw(5) << right << _start.mjd()
                    << " " << setw(15) << std::fixed << setprecision(13) << showpoint << _start.dsec() << endl;

                _ss << "+"
                    << "  " << setw(3) << right << nsats << "   ";

                int prn_size = (_prn.size() % 17 + 1) * 17;
                for (int i = 0; i < prn_size; i++)
                {

                    if (i % 17 == 0 && i != 0)
                    {
                        _ss << endl;
                        _ss << "+"
                            << "        ";
                    }
                    if (i < _prn.size())
                    {
                        _ss << setw(3) << right << _prn[i];
                    }
                    else
                    {
                        _ss << setw(3) << right << " 00";
                    }
                }

                auto acc_size = (acc.size() % 17 + 1) * 17;
                for (unsigned int i = 0; i < acc_size; i++)
                {
                    if (i % 17 == 0)
                    {
                        _ss << endl;
                        _ss << "++"
                            << "       ";
                    }
                    if (i < acc.size())
                    {
                        _ss << setw(3) << right << acc[i];
                    }
                    else
                    {
                        _ss << setw(3) << right << "0";
                    }
                }

                if (_sattype == "LEO")
                {
                    _ss << endl;
                    _ss << "%c"
                        << " "
                        << "L"
                        << " ";
                    for (unsigned int i = 0; i < _sagnss_coder_sp3.size(); i++)
                    {

                        if (i % 3 == 0 && i != 0)
                        {
                            _ss << endl;
                            _ss << "     ";
                        }
                        _ss << setw(6) << right << _sagnss_coder_sp3[i];
                    }
                    _ss << endl;
                }
                else
                {
                    _ss << endl;
                    _ss << "%c"
                        << " "
                        << "M"
                        << " "
                        << " cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc" << endl;
                }

                _ss << "/* generated by GREAT" << endl;
            }

            int size = _fill_buffer(buff, sz);

            _mutex.unlock();
            return size;
        }

        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_sp3::encode_head throw exception");
            _mutex.unlock();
            return -1;
        }
    }

    int gnss_coder_sp3::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                for (auto data_iter = _data.begin(); data_iter != _data.end(); ++data_iter)
                {
                    if (data_iter->second->id_type() != base_data::ALLPREC)
                        continue;

                    base_time ref;

                    int nsize = _sagnss_coder_sp3.size();
                    base_time epoch = _start;
                    int year, month, day, hour, minute, sec;

                    while (epoch <= _lastepo)
                    {
                        epoch.ymd(year, month, day);
                        epoch.hms(hour, minute, sec);

                        double xyz[3] = {0.0, 0.0, 0.0}, vel[3] = {0.0, 0.0, 0.0};
                        double clk = 0.0;
                        int obs_num = 0;

                        for (int i = 0; i < nsize; i++)
                        {

                            if (!((gnss_all_prec *)data_iter->second)->get_pos_vel(_prn[i], epoch, xyz, vel, obs_num))
                                continue;
                            if (double_eq(xyz[0] * xyz[1] * xyz[2], 0.0))
                            {
                                continue;
                            }
                            if (i == 0)
                            {
                                _ss << "*" << setw(6) << right << year << setw(3) << right << month
                                    << setw(3) << right << day << setw(3) << right << hour
                                    << setw(3) << right << minute
                                    << setw(12) << std::fixed << setprecision(8) << showpoint << right << double(sec) << endl;
                            }
                            _ss << "P" << _prn[i]
                                << setw(14) << std::fixed << setprecision(6) << showpoint << right << xyz[0] / 1000.0
                                << setw(14) << std::fixed << setprecision(6) << showpoint << right << xyz[1] / 1000.0
                                << setw(14) << std::fixed << setprecision(6) << showpoint << right << xyz[2] / 1000.0
                                << setw(14) << std::fixed << setprecision(6) << showpoint << right << clk / 1000000.0
                                << setw(4) << right << obs_num << setw(4) << right << 0 << endl;
                        }
                        epoch = epoch + _orbintv;
                    }
                    _ss << "EOF" << endl;
                }

                //_get_delta_pos_vel
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_sp3::encode_data throw exception");
            return -1;
        }
    }

} // namespace
