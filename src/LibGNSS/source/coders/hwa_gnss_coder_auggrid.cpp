#include "hwa_gnss_coder_auggrid.h"
#include "hwa_set_inp.h"
#include "hwa_set_gproc.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_auggrid::gnss_coder_auggrid(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
        _buffsz = 0;
        _realtime = dynamic_cast<set_gproc *>(_set)->realtime();
        _ubuffer = new unsigned char[2048];
    }

    gnss_coder_auggrid::~gnss_coder_auggrid()
    {
        if (_ubuffer)
        {
            delete[] _ubuffer;
            _ubuffer = NULL;
        }
    }

    int gnss_coder_auggrid::decode_head(char *buff, int sz, vector<string> &errmsg)
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
            if (tmp.find("MARKER NAME") != string::npos)
            {
                string marker;
                istr >> marker;
                _Marker = marker;
            }
            else if (tmp.find("AREA ID") != string::npos)
            {
                istr >> _ID;
            }
            else if (tmp.find("REFERENCE POINT POSITION") != string::npos)
            {
                double d_first,d_second;
                string first, second;
                istr >> d_first >> first >> d_second >> second;
                if (first.find("N") != string::npos || first.find("S") != string::npos)
                {
                    _reflat = d_first;
                    _reflon = d_second;
                }
                else
                {
                    _reflon = d_first;
                    _reflat = d_second;
                }
            }
            else if (tmp.find("GRID NODE SPACING") != string::npos)
            {
                double d_first, d_second;
                string first, second;
                istr >> d_first >> first >> d_second >> second;
                if (first.find("N") != string::npos || first.find("S") != string::npos)
                {
                    _spacelat = d_first;
                    _spacelon = d_second;
                }
                else
                {
                    _spacelon = d_first;
                    _spacelat = d_second;
                }
            }
            else if (tmp.find("GRID NODE COUNT") != string::npos)
            {
                double d_first, d_second;
                string first, second;
                istr >> d_first >> first >> d_second >> second;
                if (first.find("N") != string::npos || first.find("S") != string::npos)
                {
                    _countlat = d_first;
                    _countlon = d_second;
                }
                else
                {
                    _countlon = d_first;
                    _countlat = d_second;
                }
            }
            else if (tmp.find("SYS / # / FREQ") != string::npos)
            {
                string sys, temp1, temp2;
                int band;
                istr >> sys >> temp1 >> temp2 >> band;
                if (_sys_band.find(gnss_sys::str2gsys(sys)) == _sys_band.end())
                {
                    _sys_band[gnss_sys::str2gsys(sys)] = int2gobsband(band);
                }
            }
            else if (tmp.find("END OF HEADER") != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "aug", "END OF HEADER");
                map<string, base_data*>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::AUGGRID)
                    {
                        ((gnss_data_gridaug*)it->second)->setHeader(_countlat, _countlon, _reflat, _reflon, _spacelat, _spacelon, _ID, _Marker, _sys_band);
                        break;
                    }
                    it++;
                }
                base_coder::_consume(tmpsize); //lvhb added in 202007
                _mutex.unlock();
                return -1;
            }
            base_coder::_consume(tmpsize);
        }
        _mutex.unlock();
        return consume;
    }

    int gnss_coder_auggrid::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
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
            if (tmp.find(">") != string::npos)
            {
                string str;
                int year, month, day, hour, min;
                double second;

                istr >> str >> year >> month >> day >> hour >> min >> second >> _nsat;
                _epoch.from_ymdhms(year, month, day, hour, min, second);
            }
            else if (tmp.find("TRP") != string::npos)
            {
                string type;
                gnss_data_augtrop gridtrop;
                
                double C[4];
                int gridcount;
                istr >> type >> gridtrop.siteZWD.T[0] >> gridtrop.siteZWD.T[1] >> gridtrop.siteZWD.T[2] >> gridtrop.siteZWD.T[3];
                gridtrop.siteZWD.prn = type;
                map<string, base_data*>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::AUGGRID)
                    {
                        gridcount = ((gnss_data_gridaug*)it->second)->gridNum();
                        break;
                    }
                    it++;
                }
                for (int count = 0; count < gridcount; count++)
                {
                    istr >> gridtrop.siteZWD.res[count];
                }
                gridtrop.siteZWD.valid = 1;
                ((gnss_data_gridaug*)it->second)->addGridData(_epoch, gridtrop);

            }
            else if (gnss_sys::str2gsys(tmp.substr(0, 1)) != GSYS::GNS)
            {
                string sat;
                gnss_data_SATION gridiono;

                double C[4];
                int gridcount;
                istr >> sat;
                istr >> gridiono.C[0] >> gridiono.C[1] >> gridiono.C[2] >> gridiono.C[3];
                gridiono.prn = sat;
                gridiono.csys = gnss_sys::str2gsys(tmp.substr(0, 1));
                map<string, base_data*>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::AUGGRID)
                    {
                        gridcount = ((gnss_data_gridaug*)it->second)->gridNum();
                        break;
                    }
                    it++;
                }
                for (int count = 0; count < gridcount; count++)
                {
                    istr >> gridiono.res[count];
                }
                ((gnss_data_gridaug*)it->second)->addGridData(_epoch, gridiono,sat,_nsat);
            }
            base_coder::_consume(tmpsize);
        }
        _mutex.unlock();
        return consume;
    }

    
}