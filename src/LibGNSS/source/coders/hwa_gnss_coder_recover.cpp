#include "hwa_gnss_coder_recover.h"
#include "hwa_gnss_data_obs.h"

using namespace std;

namespace hwa_gnss
{
    const string gnss_coder_resfile::END_OF_HEADER = "##END OF HEADER";
    const string gnss_coder_resfile::TIME_HEADER = "##Time&Interval :";
    const string gnss_coder_resfile::SIGMA_HEADER = "##Sigma:";
    const string gnss_coder_resfile::SAT_HEADER = "##SAT:";
    const string gnss_coder_resfile::SITE_HEADER = "##STA:";
    const string gnss_coder_resfile::OBS_DATA = "RES:=";
    const string gnss_coder_resfile::PAR_DATA = "PAR:=";

    gnss_coder_resfile::gnss_coder_resfile(hwa_set::set_base *s, string version, int sz) : 
        base_coder(s, version, sz), gnss_base_coder(s, version, sz), _recover_data(nullptr)
    {
    }

    gnss_coder_resfile::~gnss_coder_resfile()
    {
    }

    int gnss_coder_resfile::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        if (!_recover_data)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " ERROR: Have no storeage allrecover for decoding resfile");
            _mutex.unlock();
            return -1;
        }

        int tmpsize = 0;
        int linesize = 0;
        string line;
        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        while ((linesize = gnss_base_coder::_getline(line, tmpsize)) >= 0)
        {
            tmpsize += linesize;
            // int idx = 0;
            if (line.find(gnss_coder_resfile::END_OF_HEADER) != string::npos)
            {
                gnss_base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }
            else if (line.find(gnss_coder_resfile::TIME_HEADER) != string::npos)
            {
                line = line.substr(gnss_coder_resfile::TIME_HEADER.length());
                string temp_ymd, temp_hms;
                double interval;
                stringstream templine(line);
                templine >> temp_ymd >> temp_hms >> interval;
                _recover_data->set_interval(interval);
            }
            else if (line.find(gnss_coder_resfile::SIGMA_HEADER) != string::npos)
            {
                line = line.substr(gnss_coder_resfile::SIGMA_HEADER.length());
                double sigma;
                stringstream templine(line);
                templine >> sigma;
                _recover_data->set_sigma0(sigma);
            }
            else
            {
                continue;
            }
        }

        gnss_base_coder::_consume(tmpsize);
        _mutex.unlock();
        return tmpsize;
    }

    int gnss_coder_resfile::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();
        if (!_recover_data)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " ERROR: Have no storeage allrecover for decoding resfile");
            _mutex.unlock();
            return -1;
        }

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        int linesize = 0;
        int tmpsize = 0;
        string line;

        while ((linesize = gnss_base_coder::_getline(line, tmpsize)) >= 0)
        {
            tmpsize += linesize;
            // int idx = 0;
            if (line.find(gnss_coder_resfile::OBS_DATA) != string::npos)
            {
                int idx = line.find(gnss_coder_resfile::OBS_DATA);
                if (line.substr(58, 3) == "SLR")
                    continue;
                _recover_data->add_recover_equation(strline2recover_equation(_spdlog, line.substr(idx + gnss_coder_resfile::OBS_DATA.length())));
            }
            else if (line.find(gnss_coder_resfile::PAR_DATA) != string::npos)
            {
                int idx = line.find(gnss_coder_resfile::PAR_DATA);
                _recover_data->add_recover_par(strline2recover_par(_spdlog, line.substr(idx + gnss_coder_resfile::OBS_DATA.length())));
            }
            else
            {
                continue;
            }
        }

        tmpsize = gnss_base_coder::_consume(tmpsize);
        _mutex.unlock();
        return tmpsize;
    }

    int gnss_coder_resfile::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        if (!_recover_data)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " ERROR: Have no data for encode in resfile encoder");
            _mutex.unlock();
            return -1;
        }

        if (_ss_position == 0)
        {
            _ss << gnss_coder_resfile::TIME_HEADER
                << setiosflags(ios::right)
                << setw(30) << _recover_data->get_beg_time().str()
                << setw(15) << _recover_data->get_interval()
                << endl;

            _ss << gnss_coder_resfile::SIGMA_HEADER
                << setw(15) << fixed << setprecision(3) << _recover_data->get_sigma0() << endl;

            _ss << gnss_coder_resfile::SITE_HEADER;
            int count = 0;
            for (string site : _recover_data->get_site_list())
            {
                _ss << setw(5) << setiosflags(ios::right) << site;
                if (++count == 10)
                {
                    _ss << endl
                        << gnss_coder_resfile::SITE_HEADER;
                    count = 0;
                }
            }
            _ss << endl;

            _ss << gnss_coder_resfile::SAT_HEADER;
            count = 0;
            for (string sat : _recover_data->get_sat_list())
            {
                _ss << setw(4) << setiosflags(ios::right) << sat;
                if (++count == 10)
                {
                    _ss << endl
                        << gnss_coder_resfile::SAT_HEADER;
                    count = 0;
                }
            }
            _ss << endl;

            _ss << gnss_coder_resfile::END_OF_HEADER << endl;
        }
        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

    int gnss_coder_resfile::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        // jdhuang : if the par not be estimated, no output
        if (_ss_position == 0)
        {
            for (gnss_data_recover *recover_record : _recover_data->get_all_recover_data())
            {
                _ss << recover_record->convert2strline();
            }
        }
        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

    void gnss_coder_resfile::_add_data(const string &id, base_data *data)
    {
        if (data->id_type() == base_data::ALLRECOVER)
        {
            _recover_data = dynamic_cast<gnss_all_recover *>(data);
        }
    }

    gnss_data_recover_par strline2recover_par(base_log spdlog, const string &line)
    {
        stringstream templine(line);
        string str_partype;
        string time_beg1, time_beg2, time_end1, time_end2;
        double inital_value, correct_value;
        templine >> str_partype >> time_beg1 >> time_beg2 >> time_end1 >> time_end2 >> inital_value >> correct_value;

        base_par par = str2gpar(str_partype);
        base_time beg_time(GPS), end_time(GPS);
        beg_time.from_str("%Y-%m-%d %H:%M:%S", time_beg1 + " " + time_beg2);
        end_time.from_str("%Y-%m-%d %H:%M:%S", time_end1 + " " + time_end2);
        par.beg = beg_time;
        par.end = end_time;
        par.value(inital_value);
        gnss_data_recover_par recover_par(spdlog, par, correct_value);
        return recover_par;
    }

    gnss_data_recover_equation strline2recover_equation(base_log spdlog, const string &line)
    {
        stringstream templine(line);

        string time1, time2;
        string site, sat, str_obstype;
        double weight, residual;
        int is_newamb;
        templine >> time1 >> time2 >> is_newamb >> site >> sat >> str_obstype >> weight >> residual;
        base_time time;
        time.from_str("%Y-%m-%d %H:%M:%S", time1 + " " + time2);

        gnss_data_recover_equation recover_equ(spdlog, time, site, sat);
        recover_equ.set_recover_equation(gnss_data_obscombtype(str_obstype), make_pair(weight, residual), is_newamb);

        return recover_equ;
    }

}