#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <memory>
#include "hwa_gnss_coder_biabernese.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_data_bias.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_biabernese::gnss_coder_biabernese(set_base *s, string version, int sz, string id)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz, id)
    {

        _allbias = 0;
        _ac = "";
        _beg = FIRST_TIME; // glfeng add
        _end = LAST_TIME;  // glfeng add
    }

    /* -------- *
 * DECODER
 * -------- */
    // rewrite by glfeng
    int gnss_coder_biabernese::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        // Origin modified by glfeng
        // --------------------
        //  _mutex.lock();
        //
        //  // no header expected, but fill the buffer
        //  base_coder::_add2buffer(buff, sz);
        //  _mutex.unlock(); return -1;
        // --------------------------

        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        string line;
        int consume = 0;
        int tmpsize = 0;
        size_t idx = 0;
        string year = "";
        string mon = "";

        while ((tmpsize = base_coder::_getline(line)) >= 0)
        {
            consume += tmpsize;
            if (line.find("%=BIA") == 0)
            { // first line
                _is_bias = true;
                _version = base_type_conv::str2dbl(line.substr(6, 4));
                _ac = line.substr(11, 3);
                if (_ac == "IGG")
                    _ac = "CAS";
                if (_ac == "CNT" && line[64] == 'P')
                    _is_absolute = true;
                else
                    _is_absolute = line.substr(64, 1) == "A";

                if (_is_absolute)
                    _ac += "_A";
                else
                    _ac += "_R";
            }
            else if (line.find("+BIAS/SOLUTION") != string::npos)
            {
                base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }
            else if (line.find("CODE'S MONTHLY") != string::npos)
            {
                _ac = "COD_R";
                // change for another header of DCB files glfeng
                if ((idx = line.find("YEAR ")) != string::npos)
                {
                    year = line.substr(idx + 5, 4);
                    mon = line.substr(idx + 17, 2);
                }
                // add for another header of DCB files glfeng
                if ((idx = line.find("YEAR-MONTH")) != string::npos)
                {
                    if (line.substr(idx + 11, 1) > "8")
                        year = "19" + line.substr(idx + 11, 2);
                    else
                        year = "20" + line.substr(idx + 11, 2);
                    mon = line.substr(idx + 14, 2);
                }
                base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }
            else if (line.find("CODE'S 30-DAY") != string::npos)
            {
                _ac = "COD_R";
                string end_doy = "";
                if ((idx = line.find("ENDING DAY")) != string::npos)
                {
                    year = line.substr(idx + 16, 4);
                    end_doy = line.substr(idx + 11, 3);
                }
                string time_str = year + ":" + end_doy;
                //_end.from_str("%Y:%j", time_str);
                //_beg = _end - 86400 * 30;
                base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }
            base_coder::_consume(tmpsize);
        }
        _mutex.unlock();
        return consume;
    }

    // rewrite by glfeng
    int gnss_coder_biabernese::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        //LX added the DLR and WHU dcb
        if (_is_bias)
        {
            if (_version < 1.0)
                return _decode_data_sinex_0(buff, sz, cnt, errmsg);
            else
                return _decode_data_sinex(buff, sz, cnt, errmsg);
        }
        else if (_ac == "COD_R")
        {
            return _decode_data_CODE(buff, sz, cnt, errmsg);
        }
        else
        {
            return -1;
        }
    }

    // add by glfeng
    int gnss_coder_biabernese::_decode_data_CODE(char *buff, int sz, int &cnt, vector<string> &errmsg)
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
        bool complete = false;

        GOBS gobs1, gobs2;
        gobs1 = gobs2 = X;

        while ((tmpsize = base_coder::_getline(line)) >= 0)
        {
            complete = false;

            size_t idx = 0;
            string signals = "";
            if ((idx = line.find("DIFFERENTIAL (")) != string::npos)
            {
                signals = line.substr(idx + 14, 5);
                gobs1 = str2gobs(signals.substr(0, 2));
                gobs2 = str2gobs(signals.substr(3, 2));
                consume += base_coder::_consume(tmpsize);
            }

            string prn = "";
            double dcb, std;
            dcb = std = 0.0;

            std::istringstream istr(line);
            istr >> prn >> dcb >> std;
            if (!istr.fail())
            {
                complete = true;
            }
            else
            {
                consume += base_coder::_consume(tmpsize);
                continue;
            }

            if (complete)
            {

#ifdef DEBUG
                std::cout << "DCB decoding: " << prn << " " << beg.str_ymdhms() << " " << end.str_ymdhms() << " " << gobs2str(gobs1) << " " << gobs2str(gobs2) << " " << dcb << endl;
                int ooo;
                cin >> ooo;
#endif

                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::ALLBIAS)
                    {
                        shared_ptr<gnss_data_bias> pt_bias = make_shared<gnss_data_bias>(_spdlog);
                        pt_bias->set(_beg, _end, dcb * 1e-9 * CLIGHT, gobs1, gobs2);
                        ((gnss_all_bias *)it->second)->add(_ac, _beg, prn, pt_bias);
                    }
                    it++;
                }
                consume += base_coder::_consume(tmpsize);
            }
        }

        _mutex.unlock();
        return consume;
    }

    // add by glfeng
    // jqwu rewrite according to sinex_bias_100.pdf
    int gnss_coder_biabernese::_decode_data_sinex(char *buff, int sz, int &cnt, vector<string> &errmsg)
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
        bool complete = false;

        GOBS gobs1, gobs2;
        gobs1 = gobs2 = X;

        while ((tmpsize = base_coder::_getline(line)) >= 0)
        {
            complete = false;
            //LX changed for OSB
            if (line.find("-BIAS/SOLUTION") != line.npos || line.find("%=ENDBIA") == 0)
            {
                _mutex.unlock();
                return 0;
            }
            else if (line.substr(1, 4) == "DSB " || line.substr(1, 4) == "ISB " || line.substr(1, 4) == "OSB ")
            {
                string svn, prn, station, obs1, obs2, units;
                double value,factor;
                if (line.size() >= 104)
                {
                    svn = line.substr(6, 4);
                    prn = line.substr(11, 3);
                    station = base_type_conv::trim(line.substr(15, 9));
                    if (station.size() != 0)
                    { // currently station-specific bias is not used
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    obs1 = base_type_conv::trim(line.substr(25, 4));
                    obs2 = base_type_conv::trim(line.substr(30, 4));
                    gobs1 = str2gobs(obs1);
                    gobs2 = str2gobs(obs2);
                    _beg.from_str("%Y:%j:%s", line.substr(35, 14));
                    _end.from_str("%Y:%j:%s", line.substr(50, 14));
                    units = base_type_conv::trim(line.substr(65, 4));
                    if (units == "ns")
                    {
                        factor = 1e-9 * CLIGHT;
                    }
                    else if (units == "cyc")
                    {
                        // may for phase bias, but wavelength is needed, so not recommended
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    else
                    {
                        // other units type currently not considered
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    value = base_type_conv::str2dbl(line.substr(70, 21));
                    complete = true;
                }
                else if (line.size() >= 92 && _ac == "CNT")
                {

                    svn = line.substr(6, 4);
                    prn = line.substr(11, 3);
                    station = base_type_conv::trim(line.substr(15, 9));
                    if (station.size() != 0)
                    { // currently station-specific bias is not used
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    obs1 = base_type_conv::trim(line.substr(25, 4));
                    obs2 = base_type_conv::trim(line.substr(30, 4));
                    gobs1 = str2gobs(obs1);
                    gobs2 = str2gobs(obs2);
                    _beg.from_str("%Y:%j:%s", line.substr(35, 14));
                    _end.from_str("%Y:%j:%s", line.substr(50, 14));
                    units = base_type_conv::trim(line.substr(65, 4));
                    if (units == "ns")
                    {
                        factor = 1e-9 * CLIGHT;
                    }
                    else if (units == "cyc")
                    {
                        // may for phase bias, but wavelength is needed, so not recommended
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    else
                    {
                        // other units type currently not considered
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    value = base_type_conv::str2dbl(line.substr(70, 21));
                    complete = true;
                }
                else
                {
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }
                if (complete)
                {
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            shared_ptr<gnss_data_bias> pt_bias = make_shared<gnss_data_bias>(_spdlog);
                            // std::cout << value << factor << value * factor << endl;
                            pt_bias->set(_beg, _end, value * factor, gobs1, gobs2);
                            ((gnss_all_bias *)it->second)->add(_ac, _beg, prn, pt_bias);
                        }
                        it++;
                    }
                    consume += base_coder::_consume(tmpsize);
                }
            }
#ifdef OLD
            if ((line.substr(1, 3) == "DSB" || line.substr(1, 3) == "DCB") && base_type_conv::trim(line.substr(12, 2)) != "")
            {

                double dcb, std;
                dcb = std = 0.0;

                std::istringstream istr(line);
                string str1, str2, str3, str4,
                    strobs1, strobs2, beg_doy, end_doy, prn;
                istr >> str1 >> str2 >> prn >> strobs1 >> strobs2 >> beg_doy >> end_doy >> str4 >> dcb >> std;

                if (!istr.fail())
                {
                    complete = true;
                    if (str1 == "DSB")
                    {
                        _beg.from_str("%Y:%j", beg_doy.substr(0, 8));
                        _end.from_str("%Y:%j", end_doy.substr(0, 8));
                    }
                    else if (str1 == "DCB")
                    {
                        _beg.from_str("%y:%j", beg_doy.substr(0, 6));
                        _end.from_str("%y:%j", end_doy.substr(0, 6));
                    }
                    gobs1 = str2gobs(strobs1);
                    gobs2 = str2gobs(strobs2);
                }
                else
                {
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }
                if (complete)
                {

#ifdef DEBUG
                    std::cout << "DCB decoding: " << prn << " " << beg.str_ymdhms() << " " << end.str_ymdhms() << " " << gobs2str(gobs1) << " " << gobs2str(gobs2) << " " << dcb << endl;
                    int ooo;
                    cin >> ooo;
#endif

                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            shared_ptr<gnss_data_bias> pt_bias = make_shared<gnss_data_bias>(_spdlog);
                            pt_bias->set(_beg, _end, dcb * 1e-9 * CLIGHT, gobs1, gobs2);
                            ((gnss_all_bias *)it->second)->add(_ac, _beg, prn, pt_bias);
                        }
                        it++;
                    }
                    consume += base_coder::_consume(tmpsize);
                }
            }
            else if (line.substr(1, 3) == "OSB" && base_type_conv::trim(line.substr(12, 2)) != "")
            {

                double dcb, std;
                dcb = std = 0.0;

                std::istringstream istr(line);
                string str1, str2, str3, strobs1, beg_doy, end_doy, prn;
                istr >> str1 >> str3 >> prn >> strobs1 >> beg_doy >> end_doy >> str2 >> dcb >> std;

                if (!istr.fail())
                {
                    complete = true;
                    //_beg.from_str("%Y:%j", beg_doy.substr(0, 8));
                    //_end.from_str("%Y:%j", end_doy.substr(0, 8));
                    _beg.from_str("%Y:%j:%s", beg_doy.substr(0, 14));
                    _end.from_str("%Y:%j:%s", end_doy.substr(0, 14));
                    gobs1 = str2gobs(strobs1);
                }
                else
                {
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }
                if (complete)
                {

#ifdef DEBUG
                    std::cout << "DCB decoding: " << prn << " " << beg.str_ymdhms() << " " << end.str_ymdhms() << " " << gobs2str(gobs1) << " " << gobs2str(gobs2) << " " << dcb << endl;
                    int ooo;
                    cin >> ooo;
#endif

                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            shared_ptr<gnss_data_bias> pt_bias = make_shared<gnss_data_bias>(_spdlog);
                            pt_bias->set(_beg, _end, dcb * 1e-9 * CLIGHT, gobs1, gobs1);
                            if (gnss_data_obs(gobs1).is_phase())
                            {
                                ((gnss_all_bias *)it->second)->add(_ac + "_PHASE", _beg, prn, pt_bias);
                            }
                            else
                            {
                                ((gnss_all_bias *)it->second)->add(_ac, _beg, prn, pt_bias);
                            }
                        }
                        it++;
                    }
                    consume += base_coder::_consume(tmpsize);
                }
            }
#endif
            else
            {
                consume += base_coder::_consume(tmpsize);
            }
        }

        _mutex.unlock();
        return consume;
    }

    //jqwu add for version 0.01 sinex bias files
    int gnss_coder_biabernese::_decode_data_sinex_0(char *buff, int sz, int &cnt, vector<string> &errmsg)
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
        bool complete = false;

        GOBS gobs1, gobs2;
        gobs1 = gobs2 = X;

        while ((tmpsize = base_coder::_getline(line)) >= 0)
        {
            complete = false;
            //LX changed for OSB
            if (line.find("-BIAS/SOLUTION") != line.npos || line.find("%=ENDBIA") == 0)
            {
                _mutex.unlock();
                return 0;
            }
            else if (line.substr(1, 4) == "DSB " || line.substr(1, 4) == "ISB " || line.substr(1, 4) == "OSB ")
            {
                string svn, prn, station, obs1, obs2, units;
                double value, factor;
                if (line.size() >= 105)
                {
                    svn = line.substr(6, 4);
                    prn = line.substr(11, 3);
                    station = base_type_conv::trim(line.substr(15, 9));
                    if (station.size() != 0)
                    { // currently station-specific bias is not used
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    obs1 = base_type_conv::trim(line.substr(30, 4));
                    obs2 = base_type_conv::trim(line.substr(35, 4));
                    gobs1 = str2gobs(obs1);
                    gobs2 = str2gobs(obs2);
                    _beg.from_str("%y:%j:%s", line.substr(40, 12));
                    _end.from_str("%y:%j:%s", line.substr(53, 12));
                    units = base_type_conv::trim(line.substr(66, 4));
                    if (units == "ns")
                    {
                        factor = 1e-9 * CLIGHT;
                    }
                    else if (units == "cyc")
                    {
                        // may for phase bias, but wavelength is needed, so not recommended
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    else
                    {
                        // other units type currently not considered
                        consume += base_coder::_consume(tmpsize);
                        continue;
                    }
                    value = base_type_conv::str2dbl(line.substr(71, 21));
                    complete = true;
                }
                else
                {
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }
                if (complete)
                {
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            shared_ptr<gnss_data_bias> pt_bias = make_shared<gnss_data_bias>(_spdlog);
                            std::cout << value << factor << value * factor << endl;
                            pt_bias->set(_beg, _end, value * factor, gobs1, gobs2);
                            ((gnss_all_bias *)it->second)->add(_ac, _beg, prn, pt_bias);
                        }
                        it++;
                    }
                    consume += base_coder::_consume(tmpsize);
                }
            }
            else
            {
                consume += base_coder::_consume(tmpsize);
            }
        }

        _mutex.unlock();
        return consume;
    }

} // namespace
