#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <memory>
#include <algorithm>
#include "hwa_gnss_coder_bnccorr.h"
#include "hwa_base_io_tcp.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"
#include "hwa_set_gtype.h"
#include "hwa_gnss_sys.h"
#include "hwa_set_gbase.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_data_upd.h"
#include "hwa_gnss_data_aug.h"
#include "hwa_gnss_data_rec.h"

using namespace std;
using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    // constructor
    gnss_coder_bnccorr::gnss_coder_bnccorr(hwa_set::set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
        _begepoch = false;
        //  _endepoch = false;

        _tt = FIRST_TIME;

        _band_index[GPS] = dynamic_cast<set_gnss *>(s)->band_index(GPS);
        _band_index[GAL] = dynamic_cast<set_gnss *>(s)->band_index(GAL);
        _band_index[GLO] = dynamic_cast<set_gnss *>(s)->band_index(GLO);
        _band_index[BDS] = dynamic_cast<set_gnss *>(s)->band_index(BDS);
    }

    // ---------
    // synchronized real-time observation in BNC structure (BNC:bnchelp.html)
    // ---------
    int gnss_coder_bnccorr::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        // no header expected, but fill the buffer
        gnss_base_coder::_add2buffer(buff, sz);
        _mutex.unlock();
        return -1;
    }

    // ---------
    // synchronized real-time observation in BNC structure (BNC:bnchelp.html)
    // ---------
    int gnss_coder_bnccorr::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        //   if( _log ) _log->comment(2,"bncobs","Start decode_data");

        // complete gcoder buffer
        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        }

        //  cout << " BUFFER : \n" << _buffer << "\n size = " << sz  << " END OF BUFFER \n\n"; cout.flush();
        string flag;
        string line;
        int tmpsize = 0; // individual reading counter
        int consume = 0; // total read counter

        while ((tmpsize = gnss_base_coder::_getline(line, 0)) >= 0)
        {
            //cout << line << endl;
            if (line.find(crlf) == string::npos)
                return 0; // end of buffer

            if (line.compare(crlf) == 0)
            { // empty line
                consume += gnss_base_coder::_consume(tmpsize);
                continue;
            }

            istringstream ss(line);

            string c, site, stream;
            int yr, mn, dd, hr, mi, indicator, size;
            int gpsw, nsat;

            float sec, gpssec;

            if (line.compare(0, 1, ">") == 0)
            { // read epoch
                ss.clear();
                //ss >> c >> flag >> yr >> mn >> dd >> hr >> mi >> sec >> indicator >> size >> stream;
                ss >> c >> flag;
                _type = str2corr(flag);
                this->available_true();

                if (_type == UPD_COR)
                    ss >> yr >> mn >> dd >> hr >> mi >> sec >> nsat;
                else if (_type == AUG_COR)
                    ss >> gpsw >> gpssec >> site;
                else
                    ss >> yr >> mn >> dd >> hr >> mi >> sec >> indicator >> size >> stream;

                //cout << line << endl;
                //cout << "gw: " << gw << " sow: " << sow << endl;
                if (!ss.fail())
                {
                    base_time t(base_time::GPS);
                    if (_type != AUG_COR)
                    {
                        t.from_ymdhms(yr, mn, dd, hr, mi, sec);
                    }
                    else
                    {
                        t.from_gws(gpsw, gpssec);
                        _decode_aug(site, t, ss);
                        cnt++;
                    }
                    _tt = t;
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLPREC)
                        {
                            if (_type == ORBIT)
                            {
                                ((gnss_all_prec *)it->second)->add_orb_interval(intv[indicator]);
                                _udorbInt = intv[indicator];
                            }
                            if (_type == CLOCK)
                            {
                                ((gnss_all_prec *)it->second)->add_clk_interval(intv[indicator]);
                                _udclkInt = intv[indicator];
                            }
                        }
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            if (_type == CODE_bias || _type == PHASE_bias)
                            {
                                _udbiaInt = intv[indicator];
                                ((gnss_all_bias *)it->second)->add_bia_intv(_udbiaInt);
                            }
                        }
                        it++;
                    }
                    consume += gnss_base_coder::_consume(tmpsize);
                    continue;
                }
            }

            if (!_validepo(_tt))
            {
                consume += gnss_base_coder::_consume(tmpsize);
                continue;
            }

            ss.clear();
            // int tmp;

            if (_type == ORBIT)
                _decode_orbit(_tt, ss);
            else if (_type == CLOCK)
                _decode_clock(_tt, ss);
            else if (_type == CODE_bias)
                _decode_cb(_tt, ss);
            else if (_type == PHASE_bias)
                _decode_pb(_tt, ss);
            else if (_type == VTEC)
                _decode_vtec(_tt, ss);
            else if (_type == UPD_COR)
                _decode_upd(_tt, ss);
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "bnc", "SSR NO " + corr2str(_type));
            }

            cnt++;
            consume += gnss_base_coder::_consume(tmpsize);

        } // while
        _mutex.unlock();
        return consume;

    } //end method

    int gnss_coder_bnccorr::_decode_orbit(const base_time &epo, istringstream &ss)
    {
        //_mutex.lock();
        int satnum;
        char satsys;
        int IOD;
        Triple dxyz(0.0, 0.0, 0.0);
        Triple dvxyz(0.0, 0.0, 0.0);
        ss >> satsys >> satnum;
        if (!ss.fail() && (satsys == 'G' ||
                           satsys == 'R' ||
                           satsys == 'E' ||
                           satsys == 'C' ||
                           satsys == 'S' ||
                           satsys == 'J'))
        {
            string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
            // GSYS gsys = gnss_sys::char2gsys(satsys);

            ss >> IOD >> dxyz[0] >> dxyz[1] >> dxyz[2] >> dvxyz[0] >> dvxyz[1] >> dvxyz[2];

            // fill single data record
            if (!_filter_gnss(prn))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "SSR ORBIT", "skip " + prn);
            }
            else
            {
                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::ALLPREC)
                    {
                        ((gnss_all_prec *)it->second)->add_delta_pos_vel(prn, epo, IOD, dxyz, dvxyz);
                        ((gnss_all_prec *)it->second)->set_end(epo, "SP3");
                    }

                    it++;
                }
            }
        }
        //_mutex.unlock();
        return 0;
    }

    int gnss_coder_bnccorr::_decode_clock(const base_time &epo, istringstream &ss)
    {
        int satnum;
        char satsys;
        int IOD;
        double C0 = UNDEFVAL_CLK, C1 = UNDEFVAL_CLK, C2 = UNDEFVAL_CLK;
        ss >> satsys >> satnum;
        if (!ss.fail() && (satsys == 'G' ||
                           satsys == 'R' ||
                           satsys == 'E' ||
                           satsys == 'C' ||
                           satsys == 'S' ||
                           satsys == 'J'))
        {
            string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
            // GSYS gsys = gnss_sys::char2gsys(satsys);

            ss >> IOD >> C0 >> C1 >> C2;

            // fill single data record
            if (!_filter_gnss(prn))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "SSR CLK", "skip " + prn);
            }
            else
            {
                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::ALLPREC)
                    {
                        ((gnss_all_prec *)it->second)->add_delta_clk(prn, epo, IOD, C0, C1, C2);
                        ((gnss_all_prec *)it->second)->set_end(epo, "CLK");
                    }

                    it++;
                }
            }
        }

        return 0;
    }

    int gnss_coder_bnccorr::_decode_cb(const base_time &epo, istringstream &ss)
    {
        int satnum;
        char satsys;
        int number;
        string channel;
        double val;
        ss >> satsys >> satnum;
        if (!ss.fail() && (satsys == 'G' ||
                           satsys == 'R' ||
                           satsys == 'E' ||
                           satsys == 'C' ||
                           satsys == 'S' ||
                           satsys == 'J'))
        {
            string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
            // GSYS gsys = gnss_sys::char2gsys(satsys);

            // fill single data record
            if (!_filter_gnss(prn))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "SSR Code Bias", "skip " + prn);
            }
            else
            {

                ss >> number;
                for (int i = 0; i < number; i++)
                {
                    shared_ptr<gnss_data_bias> code_bias = make_shared<gnss_data_bias>(_spdlog); // create first reference bias
                    ss >> channel >> val;
                    code_bias->set(epo, epo + _udbiaInt, -val, str2gobs('C' + channel));

                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            //((gnss_all_bias*)it->second)->add(prn, epo, channel, code_bias);
                            ((gnss_all_bias *)it->second)->add("RTB_A", epo, prn, code_bias);
                        }
                        it++;
                    }
                }
            }
        }

        return 0;
    }

    int gnss_coder_bnccorr::_decode_pb(const base_time &epo, istringstream &ss)
    {
        int satnum;
        char satsys;
        double yaw, dyaw;
        int number;
        string channel;
        double val;
        ss >> satsys >> satnum;
        if (!ss.fail() && (satsys == 'G' ||
                           satsys == 'R' ||
                           satsys == 'E' ||
                           satsys == 'C' ||
                           satsys == 'S' ||
                           satsys == 'J'))
        {
            string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
            // GSYS gsys = gnss_sys::char2gsys(satsys);

            // fill single data record
            if (!_filter_gnss(prn))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "SSR Phase Bias", "skip " + prn);
            }
            else
            {

                ss >> yaw >> dyaw >> number;
                // bugs
                for (int i = 0; i < number; i++)
                {
                    shared_ptr<gnss_data_bias> phase_bias = make_shared<gnss_data_bias>(_spdlog); // create first reference bias
                    ss >> channel >> val;
                    //phase_bias->set(val, str2gobs('L' + channel));
                    phase_bias->set(epo, epo + _udbiaInt, val, str2gobs('L' + channel));

                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::ALLBIAS)
                        {
                            //((gnss_all_bias*)it->second)->add(prn, epo, channel, phase_bias);
                            //((gnss_all_bias*)it->second)->add("RTB_A", epo, prn, phase_bias);
                        }

                        it++;
                    }
                }
            }
        }

        return 0;
    }

    int gnss_coder_bnccorr::_decode_vtec(const base_time &epo, istringstream &ss)
    {
        return 0;
    }

    int gnss_coder_bnccorr::_decode_upd(const base_time &epo, istringstream &ss)
    {
        int satnum;
        char satsys;
        int valid;
        double wl_val, wl_sig, wl_nrec, nl_val, nl_sig, nl_nrec;
        gnss_data_updrec upd;
        ss >> satsys >> satnum;
        if (!ss.fail() && (satsys == 'G' ||
                           satsys == 'R' ||
                           satsys == 'E' ||
                           satsys == 'C' ||
                           satsys == 'S' ||
                           satsys == 'J'))
        {
            string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
            // GSYS gsys = gnss_sys::char2gsys(satsys);

            // fill single data record
            if (!_filter_gnss(prn))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "SSR UPD Cors", "skip " + prn);
            }
            else
            {

                ss >> valid >> wl_val >> wl_sig >> wl_nrec >> nl_val >> nl_sig >> nl_nrec;

                if (valid == 1)
                {
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::UPD)
                        {
                            upd.obj = prn;
                            // WL
                            upd.sigma = wl_sig;
                            upd.value = wl_val;
                            upd.npoint = wl_nrec;
                            ((gnss_data_upd *)it->second)->add_sat_upd(UPDTYPE::WL, epo, prn, upd);

                            // NL
                            upd.sigma = nl_sig;
                            upd.value = -nl_val; // RT-UPD
                            upd.npoint = nl_nrec;
                            ((gnss_data_upd *)it->second)->add_sat_upd(UPDTYPE::NL, epo, prn, upd);

                            // set end
                            ((gnss_data_upd *)it->second)->set_end(epo);
                        }
                        it++;
                    }
                }
            }
        }

        return 0;
    }

    int gnss_coder_bnccorr::_decode_aug(const string &site, const base_time &epo, istringstream &ss)
    {
        /*
                  sgpsw  sow          site  prn      p1       l1        p2        l2
           > AUG  2103 345689.000000  HKST  G03    4.4449  -2.9724    6.8491   -5.3766
           > AUG  2103 345689.000000  HKST  G16    8.5141  -7.3809   13.6521  -12.5189
           > AUG  2103 345689.000000  HKST  E21    1.4067  -0.5427    2.1592   -1.2953
        */
        int satnum;
        char satsys;
        double p1, l1, p2, l2;

        ss >> satsys >> satnum;
        if (!ss.fail() && (satsys == 'G' ||
                           satsys == 'R' ||
                           satsys == 'E' ||
                           satsys == 'C' ||
                           satsys == 'S' ||
                           satsys == 'J'))
        {
            string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
            GSYS gsys = gnss_sys::char2gsys(satsys);

            // fill single data record
            if (!_filter_gnss(prn))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "SSR UPD Cors", "skip " + prn);
            }
            else
            {

                ss >> p1 >> l1 >> p2 >> l2;

                hwa_pair_augtype augtype;
                map<string, base_data *>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::AUG)
                    {
                        augtype = make_pair(AUGTYPE::TYPE_P, _band_index[gsys][FREQ_1]);
                        ((gnss_data_aug *)it->second)->add_data(site, epo, prn, augtype, p1);

                        augtype = make_pair(AUGTYPE::TYPE_P, _band_index[gsys][FREQ_2]);
                        ((gnss_data_aug *)it->second)->add_data(site, epo, prn, augtype, p2);

                        augtype = make_pair(AUGTYPE::TYPE_L, _band_index[gsys][FREQ_1]);
                        ((gnss_data_aug *)it->second)->add_data(site, epo, prn, augtype, l1);

                        augtype = make_pair(AUGTYPE::TYPE_L, _band_index[gsys][FREQ_2]);
                        ((gnss_data_aug *)it->second)->add_data(site, epo, prn, augtype, l2);
                    }
                    it++;
                }
            }
        }
        return 1;
    }

    bool gnss_coder_bnccorr::available(const base_time &now)
    {

        if (now < _tt || now.diff(_tt) < _udorbInt)
            return true;
        else
        {
            //return _available;
            return false;
        }
    }

    void gnss_coder_bnccorr::available_false()
    {

        //_mutex.lock();

        _available = false;

        //_mutex.unlock();
    }

    void gnss_coder_bnccorr::available_true()
    {

        //_mutex.lock();

        _available = true;

        //_mutex.unlock();
    }

    string gnss_coder_bnccorr::corr2str(const CORR_TYPE &type)
    {
        switch (type)
        {
        case ORBIT:
            return "ORBIT";
        case CLOCK:
            return "CLOCK";
        case CODE_bias:
            return "CODE_bias";
        case PHASE_bias:
            return "PHASE_bias";
        case VTEC:
            return "VTEC";
        case UPD_COR:
            return "UPD";
        case AUG_COR:
            return "AUG";
        default:
            break;
        }

        return "DEFAULT";
    }

    gnss_coder_bnccorr::CORR_TYPE gnss_coder_bnccorr::str2corr(const string &str)
    {
        string tmp = str;
        transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
        if (tmp == "ORBIT")
            return ORBIT;
        else if (tmp == "CLOCK")
            return CLOCK;
        else if (tmp == "CODE_bias")
            return CODE_bias;
        else if (tmp == "PHASE_bias")
            return PHASE_bias;
        else if (tmp == "VTEC")
            return VTEC;
        else if (tmp == "UPD")
            return UPD_COR;
        else if (tmp == "AUG")
            return AUG_COR;
        else
            return DEFAULT;
    }

    bool gnss_coder_bnccorr::_validepo(const base_time &t)
    {
        if (t > _end || t < _beg)
            return false;

        if (_filter_epoch(t))
            return true;
        else
            return false;
    }

    // Checking if a receiver object exists
    // ----------------------------------
    void gnss_coder_bnccorr::_check_recobj(string &site)
    {
        map<string, base_data *>::iterator it = _data.begin();
        for (it = _data.begin(); it != _data.end(); ++it)
        {
            if (it->second->id_type() == base_data::ALLOBJ)
            {
                gnss_all_obj *all_obj = (gnss_all_obj *)it->second;
                shared_ptr<gnss_data_obj> rec = all_obj->obj(site);
                if (rec == nullptr)
                {
                    rec = make_shared<gnss_data_rec>(_spdlog);
                    rec->id(site);
                    rec->name(site);
                    all_obj->add(rec);
                }
            }
        }
    }

} // namespace
