#include "hwa_gnss_coder_aug.h"
#include "hwa_set_inp.h"
#include "hwa_set_gproc.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_aug::gnss_coder_aug(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
        _buffsz = 0;
        caster = dynamic_cast<set_inp *>(_set)->inp_caster("aug");
        sysall = dynamic_cast<set_gen *>(_set)->sys();
        _realtime = dynamic_cast<set_gproc *>(_set)->realtime();
        _ubuffer = new unsigned char[2048];
    }

    gnss_coder_aug::~gnss_coder_aug()
    {
        if (_ubuffer)
        {
            delete[] _ubuffer;
            _ubuffer = NULL;
        }
    }

    int gnss_coder_aug::GetMessage_VS2015(unsigned char* buffer)
    {
        unsigned char* m, * e;
        int i;
        size_t _NeedBytes;
        m = buffer + _SkipBytes;
        e = buffer + _buffsz;
        _NeedBytes = _SkipBytes = 0;
        _this_crc = false;
        while (e - m >= 3)
        {
            if (m[0] == 0xD3)
            {
                _BlockSize = ((m[1] & 3) << 8) | m[2];
                if (e - m >= static_cast<int>(_BlockSize + 6))
                {
                    if (static_cast<uint32_t>((m[3 + _BlockSize] << 16) | (m[3 + _BlockSize + 1] << 8) | (m[3 + _BlockSize + 2])) == CRC24(_BlockSize + 3, m))
                    {
                        _BlockSize += 6;
                        _SkipBytes = _BlockSize;

                        _first_crc = true;
                        _this_crc = true;
                        _last_size = _SkipBytes;
                        _tail_size = _buffsz - (m - buffer) - _SkipBytes; //last block size plus tail data

                        break;
                    }
                    else
                        ++m;
                }
                else
                {
                    _NeedBytes = _BlockSize;
                    break;
                }
            }
            else
                ++m;
        }
        if (e - m < 3)
        {
            _NeedBytes = 3;
        }

        /* copy buffer to front */
        i = m - buffer;
        if (i && m < e && (_this_crc || !_first_crc))
            memmove(buffer, m, static_cast<size_t>(_buffsz - i));
        if (_first_crc && !_this_crc)
            memmove(buffer, buffer + _last_size, _tail_size);
        _buffsz -= i;

        return !_NeedBytes ? ((buffer[3] << 4) | (buffer[4] >> 4)) : 0;
    }

    int gnss_coder_aug::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();
        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
        string tmp;
        int consume = 0;
        int tmpsize = 0;
        while ((tmpsize = base_coder::_getline(tmp)) >= 0)
        {
            consume += tmpsize;
            istringstream istr(tmp);
            if (tmp.find("SYS / # / AUG TYPES") != string::npos)
            {
                string s_tmp, str_sys, str_num;
                istr >> str_sys >> str_num;
                GSYS sys = gnss_sys::str2gsys(str_sys);
                vector<hwa_pair_augtype> augtype;
                while (istr >> s_tmp)
                {
                    AUGTYPE type(AUGTYPE::TYPE_NON);
                    string s_type = s_tmp.substr(0, 1);
                    if (s_type == "P")
                    {
                        type = AUGTYPE::TYPE_P;
                    }
                    else if (s_type == "L")
                    {
                        type = AUGTYPE::TYPE_L;
                    }
                    else if (s_type == "I")
                    {
                        type = AUGTYPE::TYPE_ION;
                    }
                    else if (s_type == "T")
                    {
                        type = AUGTYPE::TYPE_TRP;
                    }
                    else
                        break;
                    string str_band = s_tmp.substr(s_tmp.size() - 1, s_tmp.size() - 0); //lvhb test
                    GOBSBAND band = (GOBSBAND)std::atoi(str_band.c_str());
                    augtype.push_back(make_pair(type, band));
                }
                _augtype[sys] = augtype;
            }
            else if (tmp.find("MARKER NAME") != string::npos)
            {
                istr >> _site;
            }
            else if (tmp.find("END OF HEADER") != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "aug", "END OF HEADER");
                base_coder::_consume(tmpsize); //lvhb added in 202007
                _mutex.unlock();
                return -1;
            }
            base_coder::_consume(tmpsize);
        }
        _mutex.unlock();
        return consume;
    }

    int gnss_coder_aug::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        if (_realtime && !dynamic_cast<set_gproc*>(_set)->simulation())
        {
            _mutex.lock();
            bool decoded = false;
            errmsg.clear();

            int _max_save_size = 2048;
            // int _max_buff_size = 1024;

            _first_crc = false;
            _SkipBytes = 0;

            /*if (sz < 33)
            {
                _first_crc = true;
                _tail_size = _tail_size + sz;
            }*/
            while (sz && _buffsz < _max_save_size)
            {
                int l = _max_save_size - _buffsz;
                if (l > sz)
                    l = sz;
                memcpy(_ubuffer + _buffsz, buff, l);
                _buffsz += l;
                sz -= l;
                buff += l;
                int id;
                while ((id = GetMessage_VS2015(_ubuffer)))
                {
                    if (id == GREATAUGRTCM_ID)
                        DecodeGREATAUGCorrection(_ubuffer, _BlockSize);
                }
            }

            //_from_pos = _last_size + _tail_size;
            _beg_pos = _last_size;
            _buffsz = _tail_size;
            /*if (_last_size)
                memset(_ubuffer + _tail_size, '\0', _max_buff_size);*/

            _mutex.unlock();
            return decoded;
        }
        else
        {
            _mutex.lock();

            if (base_coder::_add2buffer(buff, sz) == 0)
            {
                _mutex.unlock();
                return 0;
            };

            string tmp;
            int consume = 0;
            int tmpsize = 0;

            while ((tmpsize = base_coder::_getline(tmp, 0)) >= 0)
            {
                consume += tmpsize;
                istringstream istr(tmp);
                if (tmp.substr(0, 1) == ">")
                {
                    string str;
                    int year, month, day, hour, min;
                    double second;

                    istr >> str >> year >> month >> day >> hour >> min >> second;
                    _epoch.from_ymdhms(year, month, day, hour, min, second);
                }
                else if (gnss_sys::str2gsys(tmp.substr(0, 1)) != GSYS::GNS)
                {
                    string sat;
                    istr >> sat;
                    GSYS sys = gnss_sys::sat2gsys(sat);
                    vector<hwa_pair_augtype> augtype = _augtype[sys];
                    for (auto iter = augtype.begin(); iter != augtype.end(); iter++)
                    {
                        // int pos = distance(augtype.begin(), iter);
                        //string str_value = tmp.substr(3 + pos * 12, 15 + pos * 12);
                        string str_value;
                        istr >> str_value; //lvhb changed in npp
                        if (str_value.empty())
                            continue;
                        double value = atof(str_value.c_str());

                        map<string, base_data *>::iterator it = _data.begin();
                        while (it != _data.end())
                        {
                            if (it->second->id_type() == base_data::AUG)
                            {
                                ((gnss_data_aug *)it->second)->add_data(_site, _epoch, sat, *iter, value);
                                break;
                            }
                            it++;
                        }
                    }
                }
                else
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "aug", "warning: incorrect AUG data record: " + _ss.str());
                    base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
    }
}