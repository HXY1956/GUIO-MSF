#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include "hwa_gnss_coder_blq.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_blq::gnss_coder_blq(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    /* ----------
 * BLQ header
 */
    int gnss_coder_blq::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        string line;
        int consume = 0;
        int tmpsize = 0;
        while ((tmpsize = base_coder::_getline(line)) >= 0)
        {

            consume += tmpsize;

            if (line.find("END HEADER") != string::npos)
            {

                base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }

            base_coder::_consume(tmpsize);
        } // end while

        _mutex.unlock();
        return consume;
    }

    /* ----------
 * BLQ body
 */
    int gnss_coder_blq::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
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

        string line;
        int consume = 0;
        int tmpsize = 0;
        int sitsize = 0;
        bool complete = false;

        while ((tmpsize = base_coder::_getline(line, sitsize)) >= 0)
        {
            sitsize += tmpsize;
            consume += tmpsize;
            complete = false;

            if (line.substr(0, 2).compare("$$") == 0)
            {
                unsigned int pos = 0;
                if ((line.find("lon/lat:")) != string::npos)
                { // read lat, lon
                    pos = line.find("lon/lat:");
                    line.erase(0, pos + 8);
                    std::istringstream istr(line);
                    istr >> _lon >> _lat;
                    if (_lon > 180)
                    {
                        _lon -= 360.0;
                    }
                    continue;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                std::istringstream istr(line);
                double a, b, c, d, e, f, g, h, i, j, k;
                istr >> a >> b >> c >> d >> e >> f >> g >> h >> i >> j >> k;
                if (!istr.fail())
                {
                    int ii = 0;
                    _blqdata.resize(6, 11);
                    _blqdata.row(ii) << a, b, c, d, e, f, g, h, i, j, k;
                    ii++;
                    while (ii <= 5)
                    {
                        if ((tmpsize = base_coder::_getline(line, sitsize)) < 0)
                        {
                            break;
                        }
                        std::istringstream istr(line);
                        istr >> a >> b >> c >> d >> e >> f >> g >> h >> i >> j >> k;
                        _blqdata.row(ii) << a, b, c, d, e, f, g, h, i, j, k;
                        ii++;
                        sitsize += tmpsize;
                        consume += tmpsize;
                        if (ii == 5)
                            complete = true;
                    }
                }
                else
                {
                    std::istringstream istr(line);
                    istr >> _site;
                    // jdhuang : for POD ocean load correction
                    _site = _site.substr(0, 4); //base_type_conv::trim(_site);
                }
            }

            if (complete)
            {
                gnss_data_otl otl(_spdlog);
                map<string, base_data *>::iterator it = _data.begin();
                if (it != _data.end())
                    otl.setdata(_site, _lon, _lat, _blqdata);

                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::ALLOTL)
                    {
                        ((gnss_all_otl *)it->second)->add(otl);
                    }
                    it++;
                }
                base_coder::_consume(sitsize);
                sitsize = 0;
            }

#ifdef DEBUG
            if (complete)
            {
                std::cout << "Site: " << _site << " lon: " << _lon << " lat: " << _lat << endl;
                std::cout << _blqdata << endl;
            }
#endif
        }

        _mutex.unlock();
        return consume;
    }

} // namespace
