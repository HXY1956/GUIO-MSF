#include "hwa_gnss_coder_upd.h"
#include "hwa_set_amb.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_upd::gnss_coder_upd(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz), _updtype(UPDTYPE::NONE)
    {

        _epoch = FIRST_TIME;
        _updtype = UPDTYPE::NONE;
    }

    /** @brief destructor. */
    gnss_coder_upd::~gnss_coder_upd()
    {
    }

    int gnss_coder_upd::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
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
        int consume = 0;
        string mode_tmp;
        try
        {
            while ((tmpsize = base_coder::_getline(tmp, 0)) >= 0)
            {
                consume += tmpsize;
                if (tmp.substr(0, 1) == "%")
                {
                    if (tmp.substr(2, 1) != "U")  //for upd in new format
                    {
                        base_coder::_consume(tmpsize);
                        continue;
                    }
                    int pos = tmp.find("_");
                    tmp.erase(0, pos + 1);
                    tmp = cut_crlf(tmp);  // remove "\n" or "\r" at the end of line (WindowsMac/Linux)
                    mode_tmp = base_type_conv::trim(tmp); // remove "\t" at the end of line
                                          //tmp.erase(tmp.end() - 1,tmp.end());
                    //mode_tmp = tmp;
                    if (mode_tmp == "EWL" || mode_tmp == "ewl" || mode_tmp == "WL" || mode_tmp == "wl" ||
                        mode_tmp == "NL" || mode_tmp == "nl" || mode_tmp == "EWL_EPOCH" || mode_tmp == "ewl_epoch" || mode_tmp == "RTPPP_SAVE" ||
                        mode_tmp == "EWL24" || mode_tmp == "ewl24" || mode_tmp == "EWL25" || mode_tmp == "ewl25")
                    {
                        if (mode_tmp == "EWL" || mode_tmp == "ewl")
                        {
                            _epoch = base_time(EWL_IDENTIFY);
                            _updtype = UPDTYPE::EWL;
                        }
                        else if (mode_tmp == "EWL24" || mode_tmp == "ewl24")
                        {
                            _epoch = base_time(EWL24_IDENTIFY);
                            _updtype = UPDTYPE::EWL24;
                        }
                        else if (mode_tmp == "EWL25" || mode_tmp == "ewl25")
                        {
                            _epoch = base_time(EWL25_IDENTIFY);
                            _updtype = UPDTYPE::EWL25;
                        }
                        else if (mode_tmp == "WL" || mode_tmp == "wl")
                        {
                            _epoch = base_time(WL_IDENTIFY);
                            _updtype = UPDTYPE::WL;
                        }
                        else if (mode_tmp == "NL" || mode_tmp == "nl")
                        {
                            _updtype = UPDTYPE::NL;
                        }
                        else if (mode_tmp == "EWL_EPOCH" || mode_tmp == "ewl_epoch")
                        {
                            _updtype = UPDTYPE::EWL_EPOCH;
                        }
                        else if (mode_tmp == "RTPPP_SAVE")
                        {
                            _updtype = UPDTYPE::RTPPP_SAVE;
                            map<string, base_data *>::iterator it = _data.begin();
                            while (it != _data.end())
                            {
                                if (it->second->id_type() == base_data::UPD)
                                {
                                    dynamic_cast<gnss_data_upd *>(it->second)->wl_epo_mode(true);
                                }
                                ++it;
                            }
                        }

                        base_coder::_consume(tmpsize);
                        break;
                    }
                    else
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown upd mode");
                        base_coder::_consume(tmpsize);
                        _mutex.unlock();
                        return -1;
                    }
                }
                else
                {
                    _mutex.unlock();
                    return -1;
                }
            }
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown mistake");
            return -1;
        }
        _mutex.unlock();
        return consume;
    }

    int gnss_coder_upd::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
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
        int consume = 0;
        string str; // no use

        int mjd;
        double sod;

        string prn;
        gnss_data_updrec upd;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
            {
                istringstream istr(tmp);
                consume += tmpsize;
                if (tmp.substr(1, 10) == "EPOCH-TIME" && _updtype != UPDTYPE::RTPPP_SAVE)
                {
                    istr >> str >> mjd >> sod;
                    _epoch.from_mjd(mjd, sod, sod - floor(sod));
                }
                else if ((tmp.substr(0, 1) == "x" || tmp.substr(0, 1) == "X") && _updtype != UPDTYPE::RTPPP_SAVE)
                {
                    // glfeng
                    istr >> prn >> upd.value >> upd.sigma >> upd.npoint;
                    upd.sigma = 10000.0;
                    //fill data loop
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::UPD)
                            dynamic_cast<gnss_data_upd *>(it->second)->add_sat_upd(_updtype, _epoch, prn.substr(1), upd);
                        ++it;
                    }
                }
                else if (tmp.substr(0, 1) == " " && _updtype != UPDTYPE::RTPPP_SAVE)
                {
                    istr >> prn >> upd.value >> upd.sigma >> upd.npoint;
                    //fill data loop
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::UPD)
                            dynamic_cast<gnss_data_upd *>(it->second)->add_sat_upd(_updtype, _epoch, prn, upd);
                        ++it;
                    }
                }
                else if (tmp.substr(0, 1) == "*" && _updtype == UPDTYPE::RTPPP_SAVE)
                {
                    int yyyy, mm, dd, hh, min, nsat;
                    double ss;
                    stringstream ssstr(tmp);
                    string ssss;
                    ssstr >> ssss >> yyyy >> mm >> dd >> hh >> min >> ss >> nsat;
                    _epoch.from_ymdhms(yyyy, mm, dd, hh, min, ss);
                }
                else if (tmp.substr(0, 3) != "EOF" && _updtype == UPDTYPE::RTPPP_SAVE)
                {
                    stringstream ssstr(tmp);
                    gnss_data_updrec wl, nl;
                    int wl_all, nl_all;
                    ssstr >> prn >> wl.value >> nl.value >> wl.npoint >> wl_all >> nl.npoint >> nl_all >> wl.sigma >> nl.sigma;
                    wl.obj = prn;
                    nl.obj = prn;
                    nl.value = -nl.value;
                    if (wl.npoint * 1.0 / wl_all >= 0.7 && nl.npoint * 1.0 / nl_all >= 0.7)
                    {
                        map<string, base_data *>::iterator it = _data.begin();
                        while (it != _data.end())
                        {
                            if (it->second->id_type() == base_data::UPD)
                            {

                                dynamic_cast<gnss_data_upd *>(it->second)->add_sat_upd(UPDTYPE::WL, _epoch, prn, wl);

                                dynamic_cast<gnss_data_upd *>(it->second)->add_sat_upd(UPDTYPE::NL, _epoch, prn, nl);
                            }
                            ++it;
                        }
                    }
                }
                else if (tmp.substr(0, 3) == "EOF")
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "WARNING: End of file" + tmp);
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "WARNING: unknown upd-data message" + tmp);
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                gnss_base_coder::_consume(tmpsize);
            }
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown mistake");
            return -1;
        }
        _mutex.unlock();
        return consume;
    }

    int gnss_coder_upd::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                _ss << "% UPD generated using upd_";
                //get upd head info
                map<string, base_data *>::iterator it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::UPD)
                    {
                        _updtype = dynamic_cast<gnss_data_upd *>(it->second)->get_est_updtype();
                    }
                }


                if (_updtype != UPDTYPE::NONE)
                {
                    _ss << updmode2str(_updtype) << endl;
                }
                else
                {
                    _ss << "Wrong mode of upd: " << updmode2str(_updtype) << endl;
                    int size = _fill_buffer(buff, sz);
                    _mutex.unlock();
                    return size;
                }
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown mistake");
            return -1;
        }
    }

    int gnss_coder_upd::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                base_time valid_beg;
                //get data from _data
                map<base_time, one_epoch_upd> upd;
                auto it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::UPD)
                    {
                        upd = dynamic_cast<gnss_data_upd *>(it->second)->get_upd(_updtype);
                        valid_beg = dynamic_cast<gnss_data_upd *>(it->second)->get_valid_beg(_updtype);
                    }
                }

                // encode
                // upd
                if (_updtype == UPDTYPE::NL || _updtype == UPDTYPE::EWL_EPOCH || _updtype == UPDTYPE::IFCB)
                {
                    
                    for (auto itUpd = upd.begin(); itUpd != upd.end(); ++itUpd)
                    {
                        if (itUpd->first < valid_beg || itUpd->second.size() == 0)
                        {
                            continue;
                        }
                        _ss << " EPOCH-TIME" << setw(8) << itUpd->first.mjd()
                            << setw(10) << fixed << setprecision(1) << itUpd->first.sod() + itUpd->first.dsec() << endl;

                        
                        for (auto itepo = itUpd->second.begin(); itepo != itUpd->second.end(); itepo++)
                        {
                            if (itepo->second->npoint <= 2 || itepo->second->sigma > 0.10)
                            {
                                _ss << "x";
                            }
                            else
                            {
                                _ss << " ";
                            }
                            _ss << setw(11) << left << itepo->first << fixed << right << setw(10) << setprecision(3)
                                << itepo->second->value << fixed << setw(10) << setprecision(3) << itepo->second->sigma
                                << setw(5) << itepo->second->npoint << endl;
                        }
                    }
                }
                else if (_updtype == UPDTYPE::WL || _updtype == UPDTYPE::EWL || _updtype == UPDTYPE::EWL24 || _updtype == UPDTYPE::EWL25)
                {
                    base_time fixed_time = base_time(WL_IDENTIFY);
                    if (_updtype == UPDTYPE::EWL)
                        fixed_time = base_time(EWL_IDENTIFY);
                    else if (_updtype == UPDTYPE::EWL24)
                        fixed_time = base_time(EWL24_IDENTIFY);
                    else if (_updtype == UPDTYPE::EWL25)
                        fixed_time = base_time(EWL25_IDENTIFY);
                    
                    for (auto itUpd = upd.begin(); itUpd != upd.end(); ++itUpd)
                    {
                        if (itUpd->first != fixed_time)
                            continue;

                        
                        for (auto itepo = itUpd->second.begin(); itepo != itUpd->second.end(); itepo++)
                        {
                            if (itepo->second->npoint <= 2 || itepo->second->sigma > 0.10)
                            {
                                _ss << "x";
                            }
                            else
                            {
                                _ss << " ";
                            }
                            _ss << setw(9) << left << itepo->first << fixed << right << setw(10) << setprecision(3)
                                << itepo->second->value << fixed << setw(10) << setprecision(3) << itepo->second->sigma
                                << setw(5) << itepo->second->npoint << endl;
                        }
                    }
                }
                else
                {
                    _ss << "Wrong data" << endl;
                }

                _ss << "EOF" << endl;
            }
            //else
            //{
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown mistake");
            return -1;
        }
    }

} //namespace