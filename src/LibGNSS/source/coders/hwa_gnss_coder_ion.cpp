#include "hwa_gnss_coder_ion.h"
#include "hwa_gnss_data_ion.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_coder_ion::gnss_coder_ion(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    /** @brief destructor. */
    gnss_coder_ion::~gnss_coder_ion()
    {
    }

    int gnss_coder_ion::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
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
                if (line.find("%Begin Time") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);

                    int beg_mjd;
                    double beg_sod;
                    ss >> beg_mjd >> beg_sod;
                    _beg.from_mjd(beg_mjd, (int)beg_sod, beg_sod - (int)beg_sod);
                }
                else if (line.find("%End   Time") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);

                    int end_mjd;
                    double end_sod;
                    ss >> end_mjd >> end_sod;
                    _end.from_mjd(end_mjd, (int)end_sod, end_sod - (int)end_sod);
                }
                else if (line.find("%Interval") != std::string::npos)
                {
                    line.erase(0, pos + 1);
                    std::stringstream ss(line);
                    ss >> _intv;
                }
                else if (line.find("%PRN") != std::string::npos)
                {
                    std::stringstream ss(line);
                    line.erase(0, pos + 1);
                    std::string sat;
                    while (ss >> sat)
                    {
                        _sats.insert(sat);
                    }
                }
                else if (line.find("%STA") != std::string::npos)
                {
                    std::stringstream ss(line);
                    line.erase(0, pos + 1);
                    std::string site;
                    while (ss >> site)
                    {
                        _recs.insert(site);
                    }
                }
                else if (line.find("%End Of Header") != std::string::npos)
                {
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ION)
                        {
                            ((gnss_data_ion *)it->second)->set_header(_beg, _end, _intv, _numb, _sats, _recs);
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
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ion::decode_head throw exception");
            return -1;
        }
    }

    int gnss_coder_ion::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
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

                if (line.find("END OF FILE") != std::string::npos)
                {
                    break;
                }

                std::stringstream ss(line);
                std::string mark;
                std::string rec;
                std::string sat;
                int beg_mjd = 0;
                int end_mjd = 0;
                double beg_sod = 0.0;
                double end_sod = 0.0;
                double value = 0.0;
                double sigma = 0.0;
                ss >> mark >> rec >> sat >> value >> sigma >> beg_mjd >> beg_sod >> end_mjd >> end_sod;
                if (!ss.fail())
                {
                    base_time beg, end;
                    beg.from_mjd(beg_mjd, (int)(beg_sod), beg_sod - (int)(beg_sod));
                    end.from_mjd(beg_mjd, (int)(end_sod), end_sod - (int)(end_sod));

                    gnss_data_ion_record record(mark, rec, sat, value, sigma, beg, end);

                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ION)
                        {
                            ((gnss_data_ion *)it->second)->add_record(record);
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
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ion::decode_data throw exception");

            return -1;
        }
    }

    int gnss_coder_ion::encode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                std::map<std::string, base_data *>::iterator it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::ION)
                    {
                        //get ambinp head info
                        dynamic_cast<gnss_data_ion *>(it->second)->get_header(_beg, _end, _intv, _sats, _recs);
                    }
                }

                //fill head data
                _ss << "## Header Of Ion File" << std::endl;
                // beg time
                _ss << "%Begin Time         = ";
                _ss << _beg.str_mjdsod() << std::endl;
                _ss << "%End   Time         = ";
                _ss << _end.str_mjdsod() << std::endl;
                //interval(seconds)
                _ss << "%Interval           = " << _intv << std::endl;
                //satellite number and list
                _ss << "%Satellites         = " << _sats.size() << std::endl;
                _ss << "   %PRN  = ";
                for (auto itsat = _sats.begin(); itsat != _sats.end(); itsat++)
                {
                    _ss << *itsat << " ";
                }
                _ss << std::endl;
                //station number and list
                _ss << "%Stations             = " << _recs.size() << std::endl;
                _ss << "   %STA  = ";
                int i = 0;
                for (auto itsta = _recs.begin(); itsta != _recs.end(); itsta++)
                {
                    i++;
                    if (i > 20)
                    {
                        i = 0;
                        _ss << std::endl;
                        _ss << "   %STA  = ";
                    }
                    _ss << *itsta << " ";
                }
                _ss << std::endl;
                //end
                _ss << "%End Of Header" << std::endl;
                _ss << std::endl;
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ion::encode_head throw exception");

            return -1;
        }
    }

    int gnss_coder_ion::encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                std::map<std::pair<std::string, std::string>, std::map<base_time, gnss_data_ion_record>> tmp;
                for (auto it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::ION)
                    {
                        for (const auto &it_rec : _recs)
                        {
                            for (const auto &it_sat : _sats)
                            {
                                auto key = std::make_pair(it_rec, it_sat);
                                //get ambCON DDambs
                                tmp[key] = dynamic_cast<gnss_data_ion *>(it->second)->get_records(it_rec, it_sat);
                            }
                        }
                    }
                }

                _ss << "TYPE--SITE--SAT--------VALUE------------SIGMA------BEGIN TIME-------END TIME--------------" << std::endl;

                std::string mark;
                for (const auto &it_pair : tmp)
                {
                    for (const auto &it_record : it_pair.second)
                    {
                        _ss << std::setw(6) << std::left << it_record.second.ion_type
                            << std::setw(6) << it_record.second.rec
                            << std::setw(6) << it_record.second.sat
                            << std::setw(16) << std::right << std::fixed << std::setprecision(8) << it_record.second.value
                            << std::setw(16) << std::right << std::fixed << std::setprecision(8) << it_record.second.sigma
                            << std::setw(16) << std::left << it_record.second.beg.str_mjdsod()
                            << std::setw(16) << std::left << it_record.second.end.str_mjdsod()
                            << std::endl;
                    }
                }

                _ss << "END OF FILE" << std::endl;
            }

            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ion::encode_data throw exception");
            return -1;
        }
    }

}
