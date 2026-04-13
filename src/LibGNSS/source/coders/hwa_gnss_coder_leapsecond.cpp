#include "hwa_gnss_coder_leapsecond.h"

namespace hwa_gnss
{
    /**
    * @brief constructor.
    * @param[in]  s        std::setbase control
    * @param[in]  version  version of the gcoder
    * @param[in]  sz       size of the buffer
    */
    gnss_coder_leapsecond::gnss_coder_leapsecond(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    /** @brief destructor. */
    gnss_coder_leapsecond::~gnss_coder_leapsecond()
    {
    }

    /**
    * @brief decode header of leap_second file
    * @param[in]  buff        buffer of the data
    * @param[in]  sz          buffer size of the data
    * @param[in]  errmsg      error message of the data decoding
    * @return consume size of header decoding
    */
    int gnss_coder_leapsecond::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
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

    /**
    * @brief decode data body of leap_second file
    * @param[in]  buff        buffer of the data
    * @param[in]  sz          buffer size of the data
    * @param[in]  errmsg      error message of the data decoding
    * @return consume size for data body decoding
    */
    int gnss_coder_leapsecond::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
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

        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp, 0)) >= 0)
            {
                if (tmp.substr(0, 1) == "+")
                {
                    gnss_base_coder::_consume(tmpsize);
                    continue;
                }
                else if (tmp.substr(0, 1) == " ")
                {
                    std::istringstream istr(tmp);
                    istr >> _mjd >> _leap;
                    //fill data loop
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::LEAPSECOND)
                            ((gnss_data_leapsecond *)it->second)->add_data(_mjd, _leap);
                        it++;
                    }
                }
                else if (tmp.substr(0, 1) == "-")
                {
                    _mutex.unlock();
                    return tmpsize;
                }
                gnss_base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return tmpsize;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_leapsecond::decode_data throw exception");
            _mutex.unlock();
            return -1;
        }
    }

} //namespace