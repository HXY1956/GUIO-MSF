#include "hwa_gnss_coder_ambupd.h"

namespace hwa_gnss
{
    gnss_coder_ambupd::gnss_coder_ambupd(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    /** @brief destructor. */
    gnss_coder_ambupd::~gnss_coder_ambupd()
    {
    }

    int gnss_coder_ambupd::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
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

        _mutex.unlock();
        return -1;
    }

    int gnss_coder_ambupd::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
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

        std::string tmp;
        int tmpsize = 0;
        int consume = 0;
        std::string sta;
        std::string prn;
        gnss_data_ambupd_batch data_tmp;
        base_time epoch;
        int mjd;
        double sod;
        try
        {
            while ((tmpsize = base_coder::_getline(tmp, 0)) >= 0)
            {
                consume += tmpsize;
                std::istringstream istr(tmp);
                istr >> mjd >> sod >> sta >> prn >> data_tmp.amb_if >> data_tmp.amb_wl >> data_tmp.amb_l3 >> data_tmp.amb_l4 >> data_tmp.amb_l5 >> data_tmp.flag >> data_tmp.amb_wl_sigma;
                epoch.from_mjd(mjd, sod, sod - floor(sod));

                std::map<std::string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::AMBUPD)
                    {
                        ((gnss_all_ambupd *)it->second)->addAmbUpd(sta, epoch, prn, data_tmp);
                    }
                    it++;
                }
                base_coder::_consume(tmpsize);
            }
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambupd::decode_data throw exception");
            return -1;
        }
        _mutex.unlock();
        return tmpsize;
    }

    int gnss_coder_ambupd::encode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {
        _mutex.lock();

        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

    int gnss_coder_ambupd::encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                //get data from _data
                hwa_map_ti_ambupd data_tmp;
                std::string site("    ");
                auto it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::AMBUPD)
                    {
                        for (const auto site_ambupd : dynamic_cast<gnss_all_ambupd *>(it->second)->get_sites())
                        {
                            data_tmp = dynamic_cast<gnss_all_ambupd *>(it->second)->getOneSiteAmbUpd(site_ambupd).getAllAmbUpd();
                            site = site_ambupd;
                            break;
                        }
                    }
                }

                // encode
                for (auto itepo = data_tmp.begin(); itepo != data_tmp.end(); ++itepo)
                {
                    base_time epo = itepo->first;
                    for (auto itsat = itepo->second.begin(); itsat != itepo->second.end(); itsat++)
                    {
                        // i8,f10.1,1x,a4,1x,a3,2f19.3,i8,f10.3
                        _ss << std::fixed << std::setw(8) << epo.mjd() << std::setw(10) << std::setprecision(1) << epo.sod() + 0.0
                            << " " << std::setw(4) << site << " " << std::setw(3) << itsat->first
                            << std::setw(19) << std::setprecision(3) << itsat->second->amb_if
                            << std::setw(19) << std::setprecision(3) << itsat->second->amb_wl
                            << std::setw(19) << std::setprecision(3) << itsat->second->amb_l3
                            << std::setw(19) << std::setprecision(3) << itsat->second->amb_l4
                            << std::setw(19) << std::setprecision(3) << itsat->second->amb_l5
                            << std::setw(8) << itsat->second->flag << std::setw(10) << std::setprecision(3)
                            << itsat->second->amb_wl_sigma << std::endl;
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
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambupd::encode_data throw exception");
            return -1;
        }
    }
}