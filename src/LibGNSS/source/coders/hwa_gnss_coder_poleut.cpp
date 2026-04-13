#include <string>
#include <algorithm>
#include "hwa_gnss_coder_poleut.h"
#include "hwa_gnss_data_poleut.h"
#include "hwa_gnss_all_obj.h"

using namespace std;

namespace hwa_gnss
{

    /**
    * @brief constructor.
    * @param[in]  s        setbase control
    * @param[in]  version  version of the gcoder
    * @param[in]  sz       size of the buffer
    */
    gnss_coder_poleut::gnss_coder_poleut(hwa_set::set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz), _begtime(0), _endtime(0), _interval(0), _parnum(0)
    {
    }

    /**
    * @brief decode header of poleut1 file
    * @param[in]  buff        buffer of the data
    * @param[in]  sz          buffer size of the data
    * @param[in]  errmsg      error message of the data decoding
    * @return consume size of header decoding
    */
    int gnss_coder_poleut::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();
        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

#ifdef DEBUG
        cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        cout.flush();
#endif

        string tmp;
        int consume = 0;
        int tmpsize = 0;
        string timetype;
        string time;
        string param;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp)) >= 0)
            {

                consume += tmpsize;

                // first information
                if (tmp.substr(0, 2) == "% ")
                {
                    istringstream istr(tmp.substr(2));
                    if (tmp.substr(2, 1) == "U")
                    {
                        istr >> timetype >> timetype >> timetype >> _timetype;
                    }
                    else if (tmp.substr(2, 1) == "S")
                    {
                        istr >> time >> time >> _begtime >> _endtime >> _interval;
                        map<string, base_data *>::iterator it = _data.begin();
                        while (it != _data.end())
                        {
                            if (it->second->id_type() == base_data::ALLPOLEUT1)
                            {
                                istr >> time >> time >> _begtime >> _endtime >> _interval;
                                dynamic_cast<gnss_data_poleut *>(it->second)->setBegEndTime(_begtime, _endtime);
                            }
                            it++;
                        }
                    }
                    else if (tmp.substr(2, 1) == "N")
                    {
                        istr >> param >> param >> param >> param >> _parnum;
                        string unitstr;
                        while (istr >> unitstr)
                        {
                            replace(unitstr.begin(), unitstr.end(), 'D', 'E');
                            _parunit.push_back(atof(unitstr.c_str()));
                        }
                    }
                }
                else if (tmp.substr(0, 2) == "+p")
                {
                    istringstream istr2(tmp.substr(2));
                }
                else if (tmp.substr(0, 2) == "%%") //change by glfeng
                {
                    if (tmp.substr(3, 1) == "M")
                    {
                        istringstream istr3(tmp.substr(6));
                        string name;
                        int i = 0;
                        while (i < _parnum)
                        {
                            istr3 >> name;
                            _parname.push_back(name);
                            i++;
                        }
                    }
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "END OF HEADER");
                    _mutex.unlock();
                    return -1;
                }

                gnss_base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : gnss_coder_poleut::decode_head throw exception");
            return -1;
        }
    }

    /**
    * @brief decode data body of poleut1 file
    * @param[in]  buff        buffer of the data
    * @param[in]  sz          buffer size of the data
    * @param[in]  errmsg      error message of the data decoding
    * @return consume size for data body decoding
    */
    int gnss_coder_poleut::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        cout.flush();
#endif
        string tmp;
        int tmpsize = 0;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
            {
                if (tmp.substr(0, 1) == " ")
                {
                    double t;
                    istringstream istr(tmp.substr(0, 65));
                    vector<double> par;
                    istr >> t;
                    double pa;
                    while (istr >> pa)
                    {
                        par.push_back(pa);
                    }
                    int mjd = floor(t);
                    int sec = (int)((t - mjd) * 86400 / 3600.0) * 3600;
                    base_time T;
                    T.from_mjd(mjd, sec, 0.0);
                    map<string, double> data;
                    for (int i = 0; i < _parname.size(); i++)
                    {
                        data[_parname[i]] = par[i] * _parunit[i];
                    }
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (T.mjd() > _endtime)
                            break;
                        if (it->second->id_type() == base_data::ALLPOLEUT1)
                            dynamic_cast<gnss_data_poleut *>(it->second)->setEopData(T, data, _timetype, _interval);
                        it++;
                    }
                }
                gnss_base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return tmpsize;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : gnss_coder_poleut::decode_data throw exception");
            return -1;
        }
    }

    int gnss_coder_poleut::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        try
        {
            /*_ss.seekg(0, _ss.end);
            long len = (long)_ss.tellg();
            len = len - _ss_position;
            _ss.seekg(_ss_position, ios_base::beg);*/

            if (_ss_position == 0)
            {
                for (auto itt = _data.begin(); itt != _data.end(); itt++)
                {

                    if (itt->second->id_type() == base_data::ALLPOLEUT1)
                    {
                        map<base_time, map<string, double>> *mapEOP = dynamic_cast<gnss_data_poleut *>(itt->second)->getPoleUt1DataMap();
                        auto iter_beg = mapEOP->begin();
                        auto iter_end = (--mapEOP->end());
                        _ss << "%% eopupd          : GREAT " << endl;
                        _ss << "%% Bulletin A file : EST" << endl;
                        _ss << "%% Input EOP  file : poleut1" << endl;
                        _ss << "%%" << endl;
                        _ss << "+pole&ut1" << endl;
                        _ss << "% UT1 type = UT1R" << endl;
                        _ss << "% Start&End%Interval = " << setw(9) << right << iter_beg->first.dmjd() << setw(8) << right << iter_end->first.dmjd() << setw(7) << right << fixed << setprecision(2) << 1.0 << endl;
                        _ss << "% Num. of Vars&Units =     5  0.1D+01  0.1D+01  0.1D+01  0.1D-02  0.1D-02" << endl;
                        _ss << "% Format = (f9.2,1x,2f10.6,f15.7,2f10.3,3(1x,a1))" << endl;
                        _ss << "%% MJD        XPOLE     YPOLE      UT1-TAI        DPSI     DEPSI    PRED_ID" << endl;

                        for (auto iterAll = mapEOP->begin(); iterAll != mapEOP->end(); iterAll++)
                        {

                            _ss << setw(12) << right << fixed << setprecision(5) << iterAll->first.dmjd();
                            _ss << setw(11) << right << fixed << setprecision(6) << iterAll->second["XPOLE"];
                            _ss << setw(10) << right << fixed << setprecision(6) << iterAll->second["YPOLE"];
                            _ss << setw(15) << right << fixed << setprecision(7) << iterAll->second["UT1-TAI"];
                            _ss << setw(10) << right << fixed << setprecision(3) << 0.0;
                            _ss << setw(10) << right << fixed << setprecision(3) << 0.0;
                            _ss << " I I I" << endl;
                        }
                    }
                }
            }
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : t_result::encode_data throw exception");
            return -1;
        }

        int size = _fill_buffer(buff, sz);

        _mutex.unlock();
        return size;
    }

}
