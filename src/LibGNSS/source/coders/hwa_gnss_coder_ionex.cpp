#include "hwa_gnss_coder_ionex.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_coder_ionex::gnss_coder_ionex(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {

        _data_type = "";
        _nlon = 0;
        this->clear();
    }

    /** @brief destructor. */
    gnss_coder_ionex::~gnss_coder_ionex()
    {
    }

    int gnss_coder_ionex::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
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

        int consume = 0;
        int tmpsize = 0;
        std::string line;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;
                if (line.substr(60, 18).find("EPOCH OF FIRST MAP") != std::string::npos)
                {
                    std::stringstream ss(line.substr(0, 60));

                    int yyyy, mm, dd, hh, min, sec;
                    ss >> yyyy >> mm >> dd >> hh >> min >> sec;
                    _ionex_hd.beg = base_time(yyyy, mm, dd, hh, min, sec);
                }
                else if (line.substr(60, 17).find("EPOCH OF LAST MAP") != std::string::npos)
                {
                    std::stringstream ss(line);

                    int yyyy, mm, dd, hh, min, sec;
                    ss >> yyyy >> mm >> dd >> hh >> min >> sec;
                    _ionex_hd.end = base_time(yyyy, mm, dd, hh, min, sec);
                }
                else if (line.substr(60, 8).find("INTERVAL") != std::string::npos)
                {
                    std::stringstream ss(line);
                    ss >> _ionex_hd.interval; // s
                }
                else if (line.substr(60, 16).find("MAPPING FUNCTION") != std::string::npos)
                {
                    std::stringstream ss(line);
                    std::string mapFunc;
                    ss >> mapFunc;
                    _ionex_hd.ion_mapfunc = dynamic_cast<set_gproc *>(_set)->str2ionmpfunc(mapFunc);
                }
                else if (line.substr(60, 11).find("BASE RADIUS") != std::string::npos)
                {
                    std::stringstream ss(line);
                    ss >> _ionex_hd.base_radius;   // kilometers
                    _ionex_hd.base_radius *= 1000; // meters
                }
                else if (line.substr(60, 13).find("MAP DIMENSION") != std::string::npos)
                {
                    std::stringstream ss(line);
                    ss >> _ionex_hd.map_dimension;
                }
                else if (line.substr(60, 18).find("HGT1 / HGT2 / DHGT") != std::string::npos)
                {
                    std::stringstream ss(line);
                    double l1, l2, dl;
                    ss >> l1 >> l2 >> dl;
                    l1 *= 1000;
                    l2 *= 1000;
                    dl *= 1000;
                    _ionex_hd.hgts = std::make_tuple(l1, l2, dl); // meters
                    if (double_eq(l1, l2))
                        _ionex_hd.nhgt = 1;
                    else
                        _ionex_hd.nhgt = round((l2 - l1) / dl) + 1;
                }
                else if (line.substr(60, 18).find("LAT1 / LAT2 / DLAT") != std::string::npos)
                {
                    std::stringstream ss(line);
                    double l1, l2, dl;
                    ss >> l1 >> l2 >> dl;
                    _ionex_hd.lats = std::make_tuple(l1, l2, dl); // degree
                    _ionex_hd.nlat = round((l2 - l1) / dl) + 1;
                }
                else if (line.substr(60, 18).find("LON1 / LON2 / DLON") != std::string::npos)
                {
                    std::stringstream ss(line);
                    double l1, l2, dl;
                    ss >> l1 >> l2 >> dl;
                    _ionex_hd.lons = std::make_tuple(l1, l2, dl); // degree
                    _ionex_hd.nlon = round((l2 - l1) / dl) + 1;
                }
                else if (line.substr(60, 8).find("EXPONENT") != std::string::npos)
                {
                    std::stringstream ss(line);
                    ss >> _ionex_hd.exponent;
                }
                else if (line.substr(60, 16).find("PRN / BIAS / RMS") != std::string::npos)
                {
                    std::stringstream ss(line);
                    std::string sat;
                    double bias, rms;
                    ss >> sat >> bias >> rms;
                    _ionex_hd.p1p2_dcb[sat] = std::make_pair(bias, rms); // ns
                }
                else if (line.substr(60, 13).find("END OF HEADER") != std::string::npos)
                {
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::IONEX)
                        {
                            ((gnss_data_ionex *)it->second)->add_head(_ionex_hd);
                        }
                        it++;
                    }
                    gnss_base_coder::_consume(tmpsize);
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
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ion::decode_head throw exception");

            return -1;
        }
    }

    int gnss_coder_ionex::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
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

        int consume = 0;
        int tmpsize = 0;
        std::string line;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;

                if (line.find("START OF TEC MAP") != std::string::npos)
                {
                    _data_type = "TEC_MAP";
                }
                else if (line.find("START OF RMS MAP") != std::string::npos)
                {
                    _data_type = "RMS_MAP";
                }
                else if (line.find("END OF TEC MAP") != std::string::npos)
                {
                    _data_type = "";
                }
                else if (line.find("END OF RMS MAP") != std::string::npos)
                {
                    _data_type = "";
                }
                else if (line.find("EPOCH OF CURRENT MAP") != std::string::npos)
                {
                    std::stringstream ss(line.substr(0, 60));

                    int yyyy, mm, dd, hh, min, sec;
                    ss >> yyyy >> mm >> dd >> hh >> min >> sec;
                    _crt_time = base_time(yyyy, mm, dd, hh, min, sec);
                }
                else if (line.find("LAT/LON1/LON2/DLON/H") != std::string::npos)
                {
                    double lat = base_type_conv::str2dbl(line.substr(2, 6));
                    double lon1 = base_type_conv::str2dbl(line.substr(8, 6));
                    double lon2 = base_type_conv::str2dbl(line.substr(14, 6));
                    double dlon = base_type_conv::str2dbl(line.substr(20, 6));
                    double height = base_type_conv::str2dbl(line.substr(26, 6)) * 1000; // meters

                    _ilat = round((lat - std::get<0>(_ionex_hd.lats)) / std::get<2>(_ionex_hd.lats)) + 1;
                    if (_ionex_hd.map_dimension == 2)
                        _ihgt = 1;
                    else
                        _ihgt = round((height - std::get<0>(_ionex_hd.hgts)) / std::get<2>(_ionex_hd.hgts)) + 1;

                    if (_ilat <= 0 || _ihgt <= 0 || !double_eq(lon1, std::get<0>(_ionex_hd.lons)) || !double_eq(lon2, std::get<1>(_ionex_hd.lons)) || !double_eq(dlon, std::get<2>(_ionex_hd.lons)))
                    {
                        _nlon = -1;
                    }
                    else
                    {
                        _nlon = _ionex_hd.nlon;
                        _ilon = 1;
                    }
                }
                else if (_data_type == "TEC_MAP" && _nlon > 0)
                {
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::IONEX)
                        {
                            int line_size = line.size() - 3;
                            for (int i_size = 0; i_size < line_size; i_size = i_size + 5)
                            {
                                double val = base_type_conv::str2int(line.substr(i_size, 5)) * pow(10, _ionex_hd.exponent);
                                ((gnss_data_ionex *)it->second)->add_data(_crt_time, "TEC_MAP", _ilat, _ilon, _ihgt, val);
                                _ilon++;
                                _nlon--;
                            }
                        }
                        it++;
                    }
                }

                else if (_data_type == "RMS_MAP" && _nlon > 0)
                {
                    std::map<std::string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::IONEX)
                        {
                            int line_size = line.size() - 3;
                            for (int i_size = 0; i_size < line_size; i_size = i_size + 5)
                            {
                                double rms = base_type_conv::str2int(line.substr(i_size, 5)) * pow(10, _ionex_hd.exponent);
                                ((gnss_data_ionex *)it->second)->add_data(_crt_time, "RMS_MAP", _ilat, _ilon, _ihgt, rms);
                                _ilon++;
                                _nlon--;
                            }
                        }
                        it++;
                    }
                }
                else if (line.substr(60, 11).find("END OF FILE") != std::string::npos)
                {
                    gnss_base_coder::_consume(tmpsize);
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
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ionex::decode_data throw exception");
            return -1;
        }
    }

}
