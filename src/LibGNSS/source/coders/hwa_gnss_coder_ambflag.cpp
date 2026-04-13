#include "hwa_gnss_coder_ambflag.h"

using namespace std;

namespace hwa_gnss
{

    gnss_coder_ambflag::gnss_coder_ambflag(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz),
          _max_epo(0)
    {
    }

    /** @brief destructor. */
    gnss_coder_ambflag::~gnss_coder_ambflag()
    {
    }

    int gnss_coder_ambflag::decode_head(char *buff, int sz, vector<string> &errmsg)
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
        int consume = 0;
        string str;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
            {
                consume += tmpsize;
                istringstream istr(tmp);
                if (tmp.find("End of header") == string::npos)
                {
                    if (tmp.find("Start time and interval :") != string::npos)
                    {
                        istr >> str >> str >> str >> str >> str >> _ambflag_head.beg_mjd >> _ambflag_head.beg_sod >> _ambflag_head.duration >> _ambflag_head.intv;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("Max ambc in one epoch   :") != string::npos)
                    {
                        istr >> str >> str >> str >> str >> str >> str >> _ambflag_head.max_amb_1epo;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("Old remvoed observations:") != string::npos)
                    {
                        istr >> str >> str >> str >> _ambflag_head.old_rm_obs;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("New removed observations:") != string::npos)
                    {
                        istr >> str >> str >> str >> _ambflag_head.new_rm_obs;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("Existed    ambiguities  :") != string::npos)
                    {
                        istr >> str >> str >> str >> _ambflag_head.exist_amb;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("New    ambiguities      :") != string::npos)
                    {
                        istr >> str >> str >> str >> _ambflag_head.new_amb;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("Available observations  :") != string::npos)
                    {
                        istr >> str >> str >> str >> _ambflag_head.avaiable_obs;
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else if (tmp.find("%%") != string::npos)
                    {
                        // %% means comment, skip it.
                        gnss_base_coder::_consume(tmpsize);
                        continue;
                    }
                    else
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "gnss_coder_ambflag::decode_head, unknown ambflag-head message {}", tmp);
                        gnss_base_coder::_consume(tmpsize);
                        _mutex.unlock();
                        return -1;
                    }
                }
                else
                {
                    //add head data
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::AMBFLAG ||
                            it->second->id_type() == base_data::AMBFLAG13 ||
                            it->second->id_type() == base_data::AMBFLAG14 ||
                            it->second->id_type() == base_data::AMBFLAG15)
                        {
                            string site_name;
                            //site_name = this->_fname.substr(_fname.rfind("/") + 1).substr(0, 4);
                            site_name = this->_fname.substr(_fname.rfind(".log") - 12).substr(0, 4);
                            dynamic_cast<gnss_all_ambflag *>(it->second)->addAmbFlagHead(site_name, _ambflag_head);
                        }
                        it++;
                    }
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
            }
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR: unknown mistake");
            _mutex.unlock();
            return -1;
        }
        _mutex.unlock();
        return consume;
    }

    int gnss_coder_ambflag::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
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
        gnss_data_ambflag_data data_tmp;
        string sat_name;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
            {
                istringstream istr(tmp);
                if (tmp.substr(0, 3) == "AMB" ||
                    tmp.substr(0, 3) == "IAM" ||
                    tmp.substr(0, 3) == "DEL" ||
                    tmp.substr(0, 3) == "BAD")
                {
                    tmp.erase(std::remove(tmp.begin(), tmp.end(), '\n'), tmp.end());
                    istr >> data_tmp.identify >> sat_name >> data_tmp.beg_epo >> data_tmp.end_epo >> data_tmp.iflag >> data_tmp.C1 >> data_tmp.C2 >> data_tmp.reason;

                    //fill data loop
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::AMBFLAG || it->second->id_type() == base_data::AMBFLAG13 || it->second->id_type() == base_data::AMBFLAG14 || it->second->id_type() == base_data::AMBFLAG15)
                        {
                            string rec_name = this->_fname.substr(_fname.rfind(".log") - 12).substr(0, 4);
                            dynamic_cast<gnss_all_ambflag *>(it->second)->addAmbFlagData(rec_name, sat_name, data_tmp);
                        }
                        it++;
                    }
                }
                else
                {
                    tmp.erase(std::remove(tmp.begin(), tmp.end(), '\n'), tmp.end());
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "WARNING: unknown ambflag-data message :" + tmp);
                    //                     _mutex.unlock();
                    //                     return -1;
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
        return tmpsize;
    }

    int gnss_coder_ambflag::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                //get upd head info
                shared_ptr<gnss_data_ambflag_head> head;
                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::AMBFLAG || it->second->id_type() == base_data::AMBFLAG13 || it->second->id_type() == base_data::AMBFLAG14 || it->second->id_type() == base_data::AMBFLAG15)
                    {
                        string site_name;
                        // site_name = this->_fname.substr(_fname.rfind("/") + 1).substr(0, 4);
                        site_name = this->_fname.substr(_fname.rfind(".log") - 12).substr(0, 4);
                        head = dynamic_cast<gnss_all_ambflag *>(it->second)->getOneAmbFlag(site_name).getAmbFlagHead();
                    }
                    it++;
                }

                _max_epo = floor(head->duration / head->intv);

                // fill head data
                _ss << fixed << "%Start time and interval :" << setw(6) << head->beg_mjd << setw(10) << setprecision(3) << head->beg_sod
                    << setw(9) << head->duration << setw(7) << setprecision(2) << head->intv << endl;
                _ss << "%Max ambc in one epoch   :" << setw(6) << head->max_amb_1epo << endl;
                _ss << "%Old remvoed observations:" << setw(12) << head->old_rm_obs << endl;
                _ss << "%New removed observations:" << setw(12) << head->new_rm_obs << endl;
                _ss << "%Existed    ambiguities  :" << setw(12) << head->exist_amb << endl;
                _ss << "%New    ambiguities      :" << setw(12) << head->new_amb << endl;
                _ss << "%Available observations  :" << setw(12) << head->avaiable_obs << endl;
                _ss << "%End of header" << endl;
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

    int gnss_coder_ambflag::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                //get data from _data
                hwa_map_sat_ambflag data_tmp;
                auto it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::AMBFLAG || it->second->id_type() == base_data::AMBFLAG13 || it->second->id_type() == base_data::AMBFLAG14 || it->second->id_type() == base_data::AMBFLAG15)
                    {
                        // site_name = this->_fname.substr(_fname.rfind("/") + 1).substr(0, 4);
                        string rec_name = this->_fname.substr(_fname.rfind(".log") - 12).substr(0, 4);
                        data_tmp = dynamic_cast<gnss_all_ambflag *>(it->second)->getOneAmbFlag(rec_name).getAmbFlagData();
                    }
                }

                // encode
                //auto itsat = data_tmp.begin();
                // jdhuang
                for (auto itsat = data_tmp.begin(); itsat != data_tmp.end(); ++itsat)
                {
                    for (auto itvet = itsat->second.begin(); itvet != itsat->second.end(); itvet++)
                    {
                        for (int i = 0; i < _max_epo; i++)
                        {
                            if ((*itvet)->beg_epo == i + 1)
                            {
                                _ss << fixed << setw(3) << (*itvet)->identify << setw(4) << itsat->first << setw(7) << (*itvet)->beg_epo
                                    << setw(7) << (*itvet)->end_epo << setw(4) << (*itvet)->iflag << setw(20)
                                    << setprecision(3) << (*itvet)->C1 << setw(20) << setprecision(3) << (*itvet)->C2
                                    << "  " << (*itvet)->reason << endl;
                            }
                        }
                    }
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
} //namespace