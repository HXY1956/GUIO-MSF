#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include "hwa_gnss_coder_atmospheric.h"
#include "hwa_base_typeconv.h"

using namespace hwa_gnss;

namespace hwa_gnss
{
    gnss_coder_atmloading::gnss_coder_atmloading(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    int gnss_coder_atmloading::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {
        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        std::string line;
        int consume = 0;
        int tmpsize = 0;
        while ((tmpsize = base_coder::_getline(line)) >= 0)
        {

            consume += tmpsize;

            if (line.find("END OF HEADER") != std::string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_INFO(_spdlog, "reading END OF HEADER ");

                base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }

            base_coder::_consume(tmpsize);
        } // end while

        _mutex.unlock();
        return consume;
    }

    int gnss_coder_atmloading::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
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

        std::string line;
        int consume = 0;
        int tmpsize = 0;
        int sitsize = 0;

        while ((tmpsize = base_coder::_getline(line, sitsize)) >= 0)
        {
            sitsize += tmpsize;
            consume += tmpsize;

            std::istringstream istr(line);
            double _cs1du, _ss1du, _cs2du, _ss2du; ///< sine and cosine terms of S1 and S2 loading in Up
            double _cs1dn, _ss1dn, _cs2dn, _ss2dn; ///< sine and cosine terms of S1 and S2 loading in North
            double _cs1de, _ss1de, _cs2de, _ss2de; ///< sine and cosine terms of S1 and S2 loading in East
            istr >> _lon >> _lat >> _cs1du >> _ss1du >> _cs2du >> _ss2du >> _cs1dn >> _ss1dn >> _cs2dn >> _ss2dn >> _cs1de >> _ss1de >> _cs2de >> _ss2de;
            _gridatmld.resize(3, 4);
            _gridatmld.row(0) << _cs1du, _ss1du, _cs2du, _ss2du;
            _gridatmld.row(1) << _cs1dn, _ss1dn, _cs2dn, _ss2dn;
            _gridatmld.row(2) << _cs1de, _ss1de, _cs2de, _ss2de;

            gnss_data_atmloading atmld;
            std::map<std::string, base_data *>::iterator it = _data.begin();
            if (it != _data.end())
                atmld.setdata(_lon, _lat, _gridatmld);
            while (it != _data.end())
            {
                if (it->second->id_type() == base_data::ALLATL)
                {
                    ((gnss_all_atmloading *)it->second)->add(atmld);
                }
                it++;
            }
            base_coder::_consume(sitsize);
            sitsize = 0;
        }

        _mutex.unlock();
        return consume;
    }

} // namespace
