#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <memory>
#include "hwa_gnss_coder_bncobs.h"
#include "hwa_base_const.h"
#include "hwa_set_gtype.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_data_trn.h"
#include "hwa_gnss_data_rec.h"

using namespace std;

namespace hwa_gnss
{

    // constructor
    gnss_coder_bncobs::gnss_coder_bncobs(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
        _begepoch = false;
        //  _endepoch = false;

        _tt = FIRST_TIME;
    }

    // ---------
    // synchronized real-time observation in BNC structure (BNC:bnchelp.html)
    // ---------
    int gnss_coder_bncobs::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        // no header expected, but fill the buffer
        base_coder::_add2buffer(buff, sz);
        _mutex.unlock();
        return -1;
    }

    // ---------
    // synchronized real-time observation in BNC structure (BNC:bnchelp.html)
    // ---------
    int gnss_coder_bncobs::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        //   if( _log ) _log->comment(2,"bncobs","Start decode_data");

        // complete gcoder buffer
        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        }

        //  std::cout << " BUFFER : \n" << _buffer << "\n size = " << sz  << " END OF BUFFER \n\n"; std::cout.flush();

        string line;
        int tmpsize = 0; // individual reading counter
        int consume = 0; // total read counter

        while ((tmpsize = base_coder::_getline(line, 0)) >= 0)
        {
            //std::cout << line << endl;
            if (line.find(crlf) == string::npos)
                return 0; // end of buffer

            if (line.compare(crlf) == 0)
            { // empty line
                consume += base_coder::_consume(tmpsize);
                continue;
            }

            std::istringstream ss(line);

            string c, site;
            int gw, satnum;
            double sow;
            char satsys;

            if (line.compare(0, 1, ">") == 0)
            { // read epoch
                ss.clear();
                ss >> c >> gw >> sow;

                this->available_true();
                //std::cout << line << endl;
                //std::cout << "gw: " << gw << " sow: " << sow << endl;
                if (!ss.fail())
                {
                    base_time t(base_time::GPS);
                    t.from_gws(gw, sow);
                    _tt = t;
                    //        std::cout << _tt.str_ymdhms() << " " << ss.str() << endl;
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }
            }

            if (!_validepo(_tt))
            {
                consume += base_coder::_consume(tmpsize);
                continue;
            }

            ss.clear();
            ss >> site >> satsys >> satnum;
            if (!ss.fail() && (satsys == 'G' ||
                               satsys == 'R' ||
                               satsys == 'E' ||
                               satsys == 'C' ||
                               satsys == 'S' ||
                               satsys == 'J'))
            {
                string prn = gnss_sys::eval_sat(base_type_conv::int2str(satnum), gnss_sys::char2gsys(satsys));
                GSYS gsys = gnss_sys::char2gsys(satsys);

                // filtering REC
                string site_4ch = site.substr(0, 4);
                site = site.substr(0, 4);
                if (_rec.size() == 0 || (_rec.find(site) == _rec.end() && _rec.find(site_4ch) == _rec.end()))
                {
                    //        std::cout << "Filtering out Site " << site << endl;
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }

                shared_ptr<gnss_data_obs_manager> obs = make_shared<gnss_data_obs_manager>(_spdlog, site, prn, _tt);

                map<GOBS, double> gobs;
                map<GOBS, int> glli;

                // checking if receiver object exists
                _check_recobj(site);

                // filtering GNSS systems and satellites
                if ((_sys.size() > 0 && _sys.find(gnss_sys::char2str(satsys)) == _sys.end()) ||
                    (_sat[gsys].size() > 0 && _sat[gsys].find(prn) == _sat[gsys].end()))
                {
                    //        std::cout << "Filtering out sat/sys " << prn << endl;
                    consume += base_coder::_consume(tmpsize);
                    continue;
                }

                /*
                if (satsys == 'R'){
                  int slotnum = 999;
                  ss.clear();
                  ss >> slotnum;
                  obs->channel(slotnum);
                }
                */

                string obstyp;
                double observ;
                int lli;
                ss.clear();

                while (ss >> obstyp >> observ && !ss.fail())
                {
                    if (gsys == BDS && obstyp[1] == '1')
                        obstyp[1] = '2'; // change B1 -> B2 !!!
                    GOBS typ = str2gobs(obstyp);
                    gobs[typ] = observ;

                    if (obstyp.compare(0, 1, "L") == 0)
                    {
                        ss >> lli;
                        glli[typ] = lli;
                    }
                }

                for (auto it = gobs.begin(); it != gobs.end(); ++it)
                {
                    if (_obs[gsys].size() == 0 || // filtering signals
                        _obs[gsys].find(base_type_conv::trim(gobs2str(it->first))) != _obs[gsys].end())
                    {
                        // std::cout << "Signal passed filter " << gobs2str(it->first) << endl;
                        if (it->second != 0.0)
                        {
                            obs->addobs(it->first, it->second);
                        }
                    }
                }
                /*
                      // if BNC provides non-zero always values (cycles slips), PPP has problem
                      // ----------------------------------------------------------------------
                      for( auto it = glli.begin(); it != glli.end(); ++it )
                      {
                        if( _obs[gsys].size() == 0 ||                // filtering signals
                            _obs[gsys].find(base_type_conv::trim(gobs2str(it->first))) != _obs[gsys].end() )
                        {
                          obs->addlli(it->first, it->second );
                        }
                      }
                */
#ifdef DEBUG
                std::cout << std::fixed << setprecision(3)
                     << " DECODED: " << site
                     << " " << prn
                     << " C1C: " << setw(16) << obs->getobs(C1C)
                     << " L1C: " << setw(16) << obs->getobs(L1C)
                     << " S1C: " << setw(16) << obs->getobs(S1C)
                     << obs->epoch().str(" %Y-%m-%d %H:%M:%S[%T] %W %w %v %J %s") << "\n";
                //    int ooo; cin >> ooo;
#endif

                map<string, base_data *>::iterator it;

                // adding GLO channel number to gobsgnss
                it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() != base_data::ALLOBJ)
                    {
                        it++;
                        continue;
                    }
                    gnss_all_obj *all_obj = (gnss_all_obj *)it->second;
                    shared_ptr<gnss_data_trn> one_obj = std::dynamic_pointer_cast<gnss_data_trn>(all_obj->obj(prn));
                    if (one_obj != 0 && one_obj->id_type() == base_data::TRN)
                    {
                        int ch = one_obj->channel();
                        obs->channel(ch);
                    }
                    it++;
                }

                // fill data
                it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::ALLOBS && _validepo(_tt))
                    {
                        ((gnss_all_obs *)it->second)->addobs(obs);
                        if (SPDLOG_LEVEL_TRACE == _spdlog->level())
                        {
                            std::ostringstream ltmp;
                            double diff = base_time::current_time(base_time::GPS) - obs->epoch();
                            ltmp << "add site observations " << site << " " << obs->sat() << " "
                                 << " " << obs->epoch().str("%Y-%m-%d %H:%M:%S[%T]") << " Diff " << setprecision(2) << diff;
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, ltmp.str());
                        }
                    }
                    it++;
                }
            }
            cnt++;

            consume += base_coder::_consume(tmpsize);

        } // while
        _mutex.unlock();
        return consume;

    } //end method

    /* -------- *
     * ENCODER
     * -------- */

    int gnss_coder_bncobs::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        return 0;
    }

    // encode data
    // ---------
    int gnss_coder_bncobs::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {

        _mutex.lock();

        // fill data
        if (_ss_position == 0)
        {
            for (auto it = _data.begin(); it != _data.end(); ++it)
            {
                // data types
                if (it->second->id_type() != base_data::ALLOBS &&
                    it->second->id_group() != base_data::GRP_obsERV)
                {
                    continue;
                }

                gnss_all_obs *gallobs = dynamic_cast<gnss_all_obs *>(it->second);

                set<string> sites = gallobs->stations();
                vector<base_time> epochs = gallobs->epochs(*sites.begin());
                if (sites.size() == 0)
                {
                    continue;
                }

                if (epochs.size() == 0)
                {
                    continue;
                }

                // epochs
                for (auto itEPO = epochs.begin(); itEPO != epochs.end(); ++itEPO)
                {
                    base_time epo = *itEPO;
                    if (epo < _beg || epo > _end)
                    {
                        continue;
                    } // filter BEG/END
                    if (_out_smp > 0 && int(round(epo.sod() + epo.dsec())) % int(_out_smp) != 0)
                    {
                        continue;
                    } // output SAMPLING

                    _ss << ">" << std::fixed << setw(5) << epo.gwk()
                        << setprecision(7) << setw(15) << epo.sow() + epo.dsec()
                        << setw(22) << epo.str_ymdhms()
                        << endl;

                    // stations
                    for (auto itSIT = sites.begin(); itSIT != sites.end(); ++itSIT)
                    {
                        string site = *itSIT;

                        // satellites/observations
                        vector<hwa_spt_obsmanager> vec_obs = gallobs->obs_pt(site, epo);
                        for (auto itOBS = vec_obs.begin(); itOBS != vec_obs.end(); ++itOBS)
                        {
                            hwa_spt_obsmanager pgnss_data_obs = *itOBS;
                            string sat = pgnss_data_obs->sat();

                            _ss << site << setw(4) << sat;

                            // signals
                            vector<GOBS> vec_sig = pgnss_data_obs->obs();
                            for (auto itSIG = vec_sig.begin(); itSIG != vec_sig.end(); ++itSIG)
                            {
                                GOBS gobs = *itSIG;
                                gnss_data_obs go(gobs);
                                string obs = gobs2str(gobs);
                                if (obs[0] == 'S')
                                {
                                    _ss << setw(4) << obs << setprecision(3) << setw(9) << pgnss_data_obs->getobs(gobs);
                                } // SNR
                                else
                                {
                                    _ss << setw(4) << obs << setprecision(3) << setw(15) << pgnss_data_obs->getobs(gobs);
                                } // OBS
                                if (go.is_phase())
                                {
                                    _ss << setprecision(0) << setw(5) << pgnss_data_obs->getlli(gobs);
                                } // LLI
                            }     // signals
                            _ss << endl;
                        } // satellites/observations
                    }     // stations
                }         // epochs
                _ss << endl;
            } // data type
        }     // condition

        int size = _fill_buffer(buff, sz);

        _mutex.unlock();
        return size;
    }

    bool gnss_coder_bncobs::available(const base_time &now)
    {

        if (now <= _tt)
            return true;
        else
        {
            //return _available;
            return false;
        }
    }

    void gnss_coder_bncobs::available_false()
    {

        //_mutex.lock();

        _available = false;

        //_mutex.unlock();
    }

    void gnss_coder_bncobs::available_true()
    {

        //_mutex.lock();

        _available = true;

        //_mutex.unlock();
    }

    bool gnss_coder_bncobs::_validepo(const base_time &t)
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
    void gnss_coder_bncobs::_check_recobj(string &site)
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
