#include "hwa_gnss_coder_ifcb.h"

namespace hwa_gnss
{

    gnss_coder_ifcb::gnss_coder_ifcb(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    /** @brief destructor. */
    gnss_coder_ifcb::~gnss_coder_ifcb()
    {
    }

    int gnss_coder_ifcb::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {

        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        std::cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        std::cout.flush();
#endif

        _mutex.unlock();
        return -1;
    }

    int gnss_coder_ifcb::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {
        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        std::cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        std::cout.flush();
#endif
        std::string tmp;
        int tmpsize = 0;
        int consume = 0;
        std::string str; // no use

        int mjd;
        double sod;

        std::string prn;
        gnss_data_ifcb_rec ifcb;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
            {
                std::istringstream istr(tmp);
                consume += tmpsize;
                if (tmp.substr(1, 10) == "EPOCH-TIME")
                {
                    istr >> str >> mjd >> sod;
                    _epoch.from_mjd(mjd, sod, sod - floor(sod));
                }
                else if (tmp.substr(0, 1) == "x" || tmp.substr(0, 1) == "X")
                {
                    //std::cerr << "warning: data can not use :" << tmp << std::endl;
                }
                else if (tmp.substr(0, 1) == " ")
                {
                    istr >> prn >> ifcb.value >> ifcb.sigma >> ifcb.npoint;
                    //fill data loop
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::IFCB)
                            ((gnss_data_ifcb *)it->second)->add_sat_ifcb(_epoch, prn, ifcb);
                        it++;
                    }
                }
                else if (tmp.substr(0, 3) == "EOF")
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_WARN(_spdlog, "gnss_coder_ifcb::decode_data End of file {}", tmp);
                    gnss_base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_WARN(_spdlog, "gnss_coder_ifcb::decode_data unknown ifcb-data message {}", tmp);
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
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ifcb::decode_data throw exception");
            return -1;
        }
        _mutex.unlock();
        return consume;
    }

    int gnss_coder_ifcb::encode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                _ss << "% ifcb with Triple-frequency GPS observations";
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ifcb::encode_head throw exception");
            return -1;
        }
    }

    int gnss_coder_ifcb::encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                base_time valid_beg;
                //get data from _data
                std::map<base_time, hwa_map_id_ifcb> ifcb;
                auto it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::IFCB)
                    {
                        ifcb = dynamic_cast<gnss_data_ifcb *>(it->second)->get_icfb();
                        valid_beg = dynamic_cast<gnss_data_ifcb *>(it->second)->get_valid_beg();
                    }
                }

                // encode
                // ifcb
                
                for (auto itifcb = ifcb.begin(); itifcb != ifcb.end(); ++itifcb)
                {

                    if (itifcb->first < valid_beg || itifcb->second.size() == 0)
                    {
                        continue;
                    }
                    _ss << " EPOCH-TIME" << std::setw(8) << itifcb->first.mjd()
                        << std::setw(10) << std::fixed << std::setprecision(1) << itifcb->first.sod() + itifcb->first.dsec() << std::endl;

                    
                    for (auto itepo = itifcb->second.begin(); itepo != itifcb->second.end(); itepo++)
                    {
                        if (itepo->second->npoint <= 2 || itepo->second->sigma > 0.10)
                        {
                            _ss << "x";
                        }
                        else
                        {
                            _ss << " ";
                        }
                        _ss << std::setw(11) << std::left << itepo->first << std::fixed << std::right << std::setw(10) << std::setprecision(3)
                            << itepo->second->value << std::fixed << std::setw(10) << std::setprecision(3) << itepo->second->sigma
                            << std::setw(5) << itepo->second->npoint << std::endl;
                    }
                }

                _ss << "EOF" << std::endl;
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ifcb::encode_data throw exception");
            return -1;
        }
    }

} //namespace