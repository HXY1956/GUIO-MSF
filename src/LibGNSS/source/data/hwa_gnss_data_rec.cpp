#include <stdio.h>
#include <math.h>
#include "hwa_gnss_data_rec.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_rec::gnss_data_rec()
        : gnss_data_obj()
    // base_data::id_type(REC) // NEFUNGUJE?
    {
        //  std::cout << "CONSTRUCTOR gnss_data_rec \n"; std::cout.flush();
        id_type(REC);
        // for sinex
        _overwrite = true;
    }

    gnss_data_rec::gnss_data_rec(base_log spdlog)
        : gnss_data_obj(spdlog)
    {
        id_type(REC);
        _overwrite = true;
    }
    /*
// copy constructor
// ----------
gnss_data_rec::gnss_data_rec(const gnss_data_rec& obj)
{
  _type = obj.id_type();
  _id   = obj.id();
  _name = obj.name();
       
  vector<base_time> vTIM = obj.rec_id();
  vector<base_time>::iterator itTIM = vTIM.begin();
  while( itTIM != vTIM.end() ){
    this->rec( *itTIM, obj.rec( *itTIM ));
    itTIM++;
  }
}
*/

    // destructor
    // ----------
    gnss_data_rec::~gnss_data_rec()
    {
        _maprec.clear();
        //  std::cout << "DESTRUCTOR gnss_data_rec \n"; std::cout.flush();
    }

    // set receiver name
    // ----------
    void gnss_data_rec::rec(string rec, const base_time &beg, const base_time &end)
    {
        _rec(rec, beg, end);
        return;
    }

    // set receiver name
    // ----------
    void gnss_data_rec::_rec(string rec, const base_time &beg, const base_time &end)
    {
        hwa_map_rec::iterator it = _maprec.find(beg);

        if (!(beg < end))
        {
            string lg = "Warning: " + _id + " not valid end time (end<beg) for receiver (beg:" + beg.str_ymdhms() + " -> end:" + end.str_ymdhms() + ")";
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, lg);
            return;
        }

        // begin record
        if (it == _maprec.end())
        { // not exists
            _maprec[beg] = rec;
        }
        else
        { // record exists
            if (it->first == LAST_TIME ||
                it->second.empty())
            {

                _maprec[beg] = rec;
            }
            else
            {
                return;
            }
        }

        // control end of record (with new beg search)
        it = _maprec.find(beg);
        it++;

        // beg was last in map (add final empty record)
        if (it == _maprec.end())
        {
            _maprec[end] = "";
        }
        else
        { // process end according to next value
            if (end < it->first)
            { // only if end is smaller then existing
                if (fabs(it->first - end) > 3600)
                { // significantly smaller!
                    if (it->second.empty())
                        _maprec.erase(it); // remove obsolete empty record
                    _maprec[end] = "";
                }
                else
                { // too close to next record
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: " + _id + " 'rec' end tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S"));
                }
            }
            else if (end != it->first)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " 'rec' " + rec + " end cut and tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S"));
            }
        }

        // remove duplicated empty records
        hwa_map_rec::iterator itNEW = _maprec.begin();
        hwa_map_rec::iterator itOLD = itNEW;
        while (itOLD != _maprec.end())
        {
            if (++itNEW != _maprec.end())
            {
                if ((itNEW->second.empty() && itOLD->second.empty()))
                //          ( itNEW->first == LAST_TIME ) )
                {
                    _maprec.erase(itNEW++);
                }
            }
            itOLD = itNEW;
        }

        return;
    }

    // get receiver name (>=t)
    // ----------
    string gnss_data_rec::rec(const base_time &t) const
    {
        string tmp = "";
        tmp = _rec(t);
        return tmp;
    }

    // get receiver name (>=t)
    // ----------
    string gnss_data_rec::_rec(const base_time &t) const
    {

        hwa_map_rec::const_iterator it = _maprec.upper_bound(t);
        if (it == _maprec.begin())
        {
            return "";
        } // not found (or not exists!)
        it--;

#ifdef DEBUG
        std::cout << " FOUND " << id()
             << " time : " << it->first.str("%Y-%m-%d %H:%M:%S[%T]")
             << " < " << t.str("%Y-%m-%d %H:%M:%S[%T]")
             << " " << it->second
             << endl;
#endif

        return it->second;

        /*        
  string tmp("");
  map<base_time,string>::const_reverse_iterator it = _maprec.rbeg();
  while( it != _maprec.rend() ){
    if( t < it->first ) break;
    tmp = it->second;
//#ifdef DEBUG
    std::cout << " REC SEARCH " << id() 
         << " time : " << it->first.str("%Y-%m-%d %H:%M:%S[%T]")
         << " < "      <<         t.str("%Y-%m-%d %H:%M:%S[%T]")
         << " "        << tmp
         << endl;
//#endif
    it++;
  }
  return tmp;
*/
    }

    // return validity for receiver at epoch t
    void gnss_data_rec::rec_validity(const base_time &t, base_time &beg, base_time &end) const
    {
        hwa_map_rec::const_iterator it = _maprec.upper_bound(t);

        if (it != _maprec.end())
        {
            if (it != _maprec.begin())
            {
                end = it->first;
                it--;
                beg = it->first;
            }
        }
        else
        {
            beg = FIRST_TIME;
            end = LAST_TIME;
        }

        return;
    }

    // get time tags
    // ----------
    vector<base_time> gnss_data_rec::rec_id() const
    {
        vector<base_time> tmp;
        hwa_map_rec::const_iterator itMAP = _maprec.begin();
        while (itMAP != _maprec.end())
        {

            tmp.push_back(itMAP->first);
            itMAP++;
        }
        return tmp;
    }

    // add rinex header
    // ----------------
    void gnss_data_rec::addhdr(const gnss_data_rnxhdr &hdr, const base_time &epo, string path)
    {
        if (_maphdr.find(epo) == _maphdr.end())
        {
            _maphdr[epo] = hdr;
            _maphdr[epo].path(path);
        }

        //  gnss_data_rec::hwa_map_hdr::iterator itHDR;
        //  for( itHDR = _maphdr.begin(); itHDR != _maphdr.end(); ++itHDR )
        //  {
        //    gnss_data_rnxhdr rnxhdr = itHDR->second;
        //  }
    }

    //xjhan
    void gnss_data_rec::changehdr(const gnss_data_rnxhdr &hdr, const base_time &epo, string path)
    {
        _maphdr.erase(_maphdr.begin(), _maphdr.end());
        _maphdr[epo] = hdr;
        _maphdr[epo].path(path);
    }

    // get all rinex headr
    // -------------------
    gnss_data_rec::hwa_map_hdr gnss_data_rec::gethdr()
    {
        return _maphdr;
    }

    // get one rinex headr
    // -------------------
    gnss_data_rnxhdr gnss_data_rec::gethdr(const base_time &epo)
    {
        gnss_data_rnxhdr rnxhdr = _gethdr(epo);
        return rnxhdr;
    }

    // check consistency
    // ----------
    void gnss_data_rec::compare(shared_ptr<gnss_data_rec> grec, const base_time &tt, string source)
    {
        //  gnss_data_rnxhdr rnxhdr = grec->gethdr(tt);
        //  grec->fill_rnxhdr(rnxhdr);
        gnss_data_obj::compare(grec, tt, source);
        string old, alt;
        old = base_type_conv::trim(_rec(tt));
        alt = base_type_conv::trim(grec->rec(tt));
        base_time beg, end;
        grec->rec_validity(tt, beg, end);

        if (old != alt && !alt.empty())
        {
            if (old.empty())
            {
                _rec(alt, beg, end);
                this->rec_validity(tt, beg, end);
            }
            else if (_overwrite)
            {
                _rec(alt, beg, end);
            }
            else
            {
            }
        }

        // add hdr from grec if not exists at TT
        if (_maphdr.find(tt) == _maphdr.end())
        {
            hwa_map_hdr oth_head = grec->gethdr();
            if (oth_head.find(tt) != oth_head.end())
            {
                gnss_data_rnxhdr head = oth_head[tt];
                _maphdr[tt] = head;
            }
        }
        return;
    }

    // fill data members form rinex header
    // ---------------------
    void gnss_data_rec::fill_rnxhdr(const gnss_data_rnxhdr &rnxhdr)
    {
        _fill_rnxhdr(rnxhdr);
        return;
    }

    // get one rinex headr
    // -------------------
    gnss_data_rnxhdr gnss_data_rec::_gethdr(const base_time &epo)
    {
        gnss_data_rnxhdr rnxhdr;
        auto it = _maphdr.upper_bound(epo);

        if (it == _maphdr.begin())
        {
            return rnxhdr;
        }

        return (--it)->second;
    }

    // fill data members form rinex header
    // ---------------------
    void gnss_data_rec::_fill_rnxhdr(const gnss_data_rnxhdr &rnxhdr)
    {
        base_time epo = rnxhdr.first();

        if (_name.empty())
            _name = rnxhdr.markname(); // NOT IF EXISTS! does not need to be the same

        _domes = rnxhdr.marknumb();
        _eccxyz(rnxhdr.antxyz(), epo);
        _eccneu(rnxhdr.antneu(), epo);
        _ant(rnxhdr.anttype(), epo);
        _rec(rnxhdr.rectype(), epo);

        Triple std(10, 10, 10); // 10 m for Rinex header aprox coordinates
        _crd(rnxhdr.aprxyz(), std, epo);
    }

} // namespace
