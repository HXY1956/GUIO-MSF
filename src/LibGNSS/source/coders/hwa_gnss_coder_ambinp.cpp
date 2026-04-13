#include "hwa_gnss_coder_ambinp.h"

namespace hwa_gnss
{
    gnss_coder_ambinp::gnss_coder_ambinp(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
        _flag = 0;
    }

    /** @brief destructor. */
    gnss_coder_ambinp::~gnss_coder_ambinp()
    {
    }

    int gnss_coder_ambinp::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {

        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        std::cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        std::cout.flush();
#endif

        int consume = 0;
        int tmpsize = 0;
        std::string line;
        try
        {
            while ((tmpsize = base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;
                int pos = line.find("=");
                if (line.find("%Time and Interval") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);

                    int beg_mjd, end_mjd;
                    double beg_sod, end_sod, interval;
                    ss >> beg_mjd >> beg_sod >> end_mjd >> end_sod >> interval;
                    _ambinp_hd->beg.from_mjd(beg_mjd, (int)beg_sod, beg_sod - (int)beg_sod);
                    _ambinp_hd->end.from_mjd(end_mjd, (int)end_sod, end_sod - (int)end_sod);
                    _ambinp_hd->interval = interval;
                }
                else if (line.find("%Fix mode") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);

                    ss >> _ambinp_hd->fix_mode;
                }
                else if (line.find("%Total ambiguities") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);

                    ss >> _ambinp_hd->num_amb;
                }
                else if (line.find("%STA") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);
                    std::string site;
                    while (ss >> site)
                    {
                        _ambinp_hd->site.insert(site);
                    }
                }
                else if (line.find("%SAT") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);
                    std::string sat;
                    while (ss >> sat)
                    {
                        _ambinp_hd->sat.insert(sat);
                    }
                }
                else if (line.find("End Of Header") != std::string::npos)
                {
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::AMBINP)
                        {
                            ((gnss_data_ambinp *)it->second)->add_hdinfo(_ambinp_hd);
                        }
                        it++;
                    }

                    base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambinp::decode_head throw exception");
            return -1;
        }
    }

    int gnss_coder_ambinp::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {

        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        std::cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        std::cout.flush();
#endif

        int consume = 0;
        int tmpsize = 0;
        std::string line;
        try
        {
            while ((tmpsize = base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;
                if (line.find("+Ambiguity") != std::string::npos)
                {
                    _flag = 1;
                }
                else if (line.find("+NEQ") != std::string::npos)
                {
                    _flag = 2;
                }

                if (_flag == 1)
                {
                    if (line.find("-Ambiguity") != std::string::npos)
                    {
                        base_coder::_consume(tmpsize);
                        _flag = 0;
                        continue;
                    }
                    std::stringstream ss(line);
                    int idx, beg_mjd, end_mjd;
                    std::string sat, site;
                    double beg_sod, end_sod, value, sigma;
                    base_time beg, end;
                    ss >> idx >> sat >> site >> beg_mjd >> beg_sod >> end_mjd >> end_sod >> value >> sigma;
                    if (!ss.fail())
                    {
                        beg.from_mjd(beg_mjd, (int)beg_sod, beg_sod - (int)beg_sod);
                        end.from_mjd(end_mjd, (int)end_sod, end_sod - (int)end_sod);
                    }
                    base_par par_amb(site, par_type::AMB_IF, idx, sat);
                    par_amb.setTime(beg, end);
                    par_amb.value(value);
                    par_amb.apriori(sigma);
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::AMBINP)
                        {
                            ((gnss_data_ambinp *)it->second)->add_ambpar(par_amb);
                        }
                        it++;
                    }
                }
                else if (_flag == 2)
                {
                    if (line.find("-NEQ") != std::string::npos)
                    {
                        _flag = 0;
                        base_coder::_consume(tmpsize);
                        continue;
                    }
                    std::stringstream ss(line);
                    std::string type;
                    double value;
                    std::vector<double> q;
                    ss >> type;
                    while (ss >> value)
                    {
                        q.push_back(value);
                    }
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::AMBINP)
                        {
                            ((gnss_data_ambinp *)it->second)->add_neq(type, q);
                        }
                        it++;
                    }
                }
                base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambinp::decode_data throw exception");
            return (-1);
        }
    }

    int gnss_coder_ambinp::encode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {

                std::map<std::string, base_data *>::iterator it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::AMBINP)
                    {
                        //get ambinp head info
                        dynamic_cast<gnss_data_ambinp *>(it->second)->get_hdinfo(_ambinp_hd);
                    }
                }
                //fill head data
                _ss << "## Header Of Ambinp file" << std::endl;
                // time and interval
                _ss << "%Time and Interval = ";
                int beg_mjd = _ambinp_hd->beg.mjd();
                double beg_sod = _ambinp_hd->beg.sod() + _ambinp_hd->beg.dsec();
                int end_mjd = _ambinp_hd->end.mjd();
                double end_sod = _ambinp_hd->end.sod() + _ambinp_hd->end.dsec();
                _ss << beg_mjd << " " << beg_sod << "   " << end_mjd << " " << end_sod << "   " << _ambinp_hd->interval << std::endl;
                //fix mode
                _ss << "%Fix mode           = " << _ambinp_hd->fix_mode << std::endl;
                //total ambigulity
                _ss << "%Total ambiguities = " << _ambinp_hd->num_amb << std::endl;
                //station number and list
                _ss << "%Station            = " << _ambinp_hd->site.size() << std::endl;
                _ss << "   %STA  = ";
                for (auto itsta = _ambinp_hd->site.begin(); itsta != _ambinp_hd->site.end(); itsta++)
                {
                    _ss << *itsta;
                }
                _ss << std::endl;
                //satellite number and list
                _ss << "%Satellite = " << _ambinp_hd->sat.size() << std::endl;
                _ss << "   %PRN  = ";
                for (auto itsat = _ambinp_hd->sat.begin(); itsat != _ambinp_hd->sat.end(); itsat++)
                {
                    _ss << *itsat;
                }
                _ss << std::endl;
                //end
                _ss << "## End Of Header" << std::endl;
                _ss << std::endl;
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambinp::encode_head throw exception");
            return -1;
        }
    }

    int gnss_coder_ambinp::encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                std::vector<base_par> ambpar;
                std::vector<std::vector<double>> neq;
                std::vector<std::string> parlist;
                std::map<std::string, base_data *>::iterator it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::AMBINP)
                    {
                        //get ambinp amb and neq
                        dynamic_cast<gnss_data_ambinp *>(it->second)->get_allambpar(ambpar);
                        dynamic_cast<gnss_data_ambinp *>(it->second)->get_neq(parlist, neq);
                    }
                }
                //fill ambgulity data
                _ss << "+Ambiguity " << std::endl;
                _ss << std::setw(20) << std::right << "SAT"
                    << "SITE"
                    << "BEG_TIME"
                    << "END_TIME"
                    << "VALUE" << std::endl;
                for (int i = 0; i < ambpar.size(); i++)
                {
                    _ss << std::setw(10) << std::right << i;
                    _ss << std::setw(10) << std::right << ambpar[i].prn;
                    _ss << std::setw(20) << std::right << ambpar[i].beg.mjd() << " " << ambpar[i].beg.sod() + ambpar[i].beg.dsec();
                    _ss << std::setw(20) << std::right << ambpar[i].end.mjd() << " " << ambpar[i].end.sod() + ambpar[i].end.dsec();
                    _ss << std::setw(20) << std::right << ambpar[i].value() << std::endl;
                }
                _ss << "-Ambiguity " << std::endl;
                //fill neq
                _ss << std::endl;
                _ss << "+NEQ" << std::endl;
                if (neq.size() != parlist.size())
                {
                    throw std::exception();
                }
                for (int i = 0; i < neq.size(); i++)
                {
                    _ss << std::setw(20) << std::left << parlist[i];
                    for (int j = 0; j < neq[i].size(); j++)
                    {
                        _ss << std::setw(20) << std::right << neq[i][j];
                    }
                    _ss << std::endl;
                }
                _ss << "-NEQ" << std::endl;
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambinp::encode_data throw exception");
            return -1;
        }
    }
}