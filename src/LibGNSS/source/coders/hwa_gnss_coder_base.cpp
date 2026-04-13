#include "hwa_gnss_coder_BASE.h"
#include "hwa_base_coder.h"
#include "hwa_gnss_sys.h"

namespace hwa_gnss
{
    gnss_base_coder::gnss_base_coder()
    {
    }
    gnss_base_coder::gnss_base_coder(hwa_set::set_base* s, std::string version, int sz, std::string id)
        : base_coder(s,version,sz,id) 
    {
        if(s)
            _gset(s);
    }
    gnss_base_coder::gnss_base_coder(base_time beg, base_time end, hwa_set::set_base* s, std::string version, int sz, std::string id)
        : base_coder(beg,end,s,version,sz,id)
    {
    }

    gnss_base_coder::gnss_base_coder(hwa_set::set_base* s, int sz)
        : base_coder(s,sz)
    {
        if (s)
            _gset(s);
    }

    /* ----------
     * destructor
     */
    gnss_base_coder::~gnss_base_coder()
    {
        // use free instead of delete due to realocate function!
        if (_buffer)
        {
            free(_buffer);
            _buffer = NULL; // modified by zhanglong
        }

        if (_close_with_warning)
        {
            std::sort(_notes.begin(), _notes.end());
            for (auto it = _notes.begin(); it != _notes.end(); ++it)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, it->str() + " .. " + base_name(_fname));
            }
            //    if( _log ){ _log->comment(-1, "gcoder", _fname+"  "+it->str()); }
        }
    }
    /* ----------
     * read settings
     */
    int gnss_base_coder::_gset(set_base* s)
    {
        if (!s)
            return -1;

        _set = s;
        _beg = dynamic_cast<set_gen*>(_set)->beg();
        _end = dynamic_cast<set_gen*>(_set)->end();

        _hdr = dynamic_cast<set_gen*>(_set)->thin();

        if (_gnss)
            _sys = dynamic_cast<set_gen*>(_set)->sys();

        /*_rec = dynamic_cast<set_gen*>(_set)->rec();*/
        _rec = dynamic_cast<set_gen*>(_set)->rec_all(); //to do,yjqin
        // jdhuang
        if (_rec.empty())
            _rec = dynamic_cast<set_gen*>(_set)->recs();
        _int = dynamic_cast<set_gen*>(_set)->sampling();
        _scl = dynamic_cast<set_gen*>(_set)->sampling_scalefc(); // scaling 10^decimal-digits

        // individually for each GNSS
        //  hwa_map_gnss::const_iterator itGNSS;
        //  for( itGNSS = gnss_data_priority.begin(); itGNSS != gnss_data_priority.end(); ++itGNSS )

        for (auto itGNSS = _sys.begin(); itGNSS != _sys.end(); ++itGNSS)
        {
            std::string gs = *itGNSS;
            GSYS gsys = gnss_sys::str2gsys(gs);

            _sat[gsys] = dynamic_cast<set_gnss*>(_set)->sat(gsys, false); // empty if not set (to speed up!)
            _obs[gsys] = dynamic_cast<set_gnss*>(_set)->obs(gsys, false); // empty if not set (to speed up!)
            _nav[gsys] = dynamic_cast<set_gnss*>(_set)->nav(gsys, false); // empty if not set (to speed up!)

            // extend gobs list with completely defined signals
            std::set<std::string> sgobs = dynamic_cast<set_gnss*>(_set)->gobs(gsys);
            for (auto it = sgobs.begin(); it != sgobs.end(); it++)
                _obs[gsys].insert(*it);

#ifdef DEBUG
            set<std::string>::const_iterator it;
            std::ostringstream os;
            os << gs << " NAV:";
            for (it = _nav[gsys].begin(); it != _nav[gsys].end(); ++it)
                os << " " << *it;
            os << std::endl;

            os << gs << " OBS:";
            for (it = _obs[gsys].begin(); it != _obs[gsys].end(); ++it)
                os << " " << *it;
            os << std::endl;

            os << gs << " SAT:";
            for (it = _sat[gsys].begin(); it != _sat[gsys].end(); ++it)
                os << " " << *it;
            os << std::endl;

            std::cout << "GNSS filtering:\n" + os.str();
            std::cout.flush();
#endif
        }

#ifdef DEBUG
        std::cout << "GCODER settings:"
            << " _beg " << _beg.str("%Y-%m-%d %H:%M:%S")
            << " _end " << _end.str("%Y-%m-%d %H:%M:%S")
            << " _int " << _int
            << " _sys " << _sys.size()
            << " _rec " << _rec.size()
            << " _sat " << _sat.size()
            << " _nav " << _nav.size()
            << " _obs " << _obs.size() << std::endl;
        std::cout.flush();
#endif
        return 0;
    }

    // GNSS/sat filter (return true if the epoch fits gnss & sat)
   // ----------
    bool gnss_base_coder::_filter_gnss(const std::string& prn)
    {

        std::string gs = gnss_sys::char2str(prn[0]); // DONT USE SYS FROM ARG WHICH MAY BE EMPTY
        GSYS gsys = gnss_sys::str2gsys(gs);     // DONT USE SYS FROM ARG WHICH MAY BE EMPTY
        if ((_sys.size() == 0 || _sys.find(gs) != _sys.end()) &&
            (_sat[gsys].size() == 0 || _sat[gsys].find(prn) != _sat[gsys].end()))
        {
            return true;
        }
        return false;
    }

    bool gnss_base_coder::_filter_gnss_addleo(const std::string& prn) //add leo
    {

        std::string gs;
        GSYS gsys;

        //xjhan test
        if ((prn.substr(0, 1) == "2" || prn.substr(0, 1) == "3" || prn.substr(0, 1) == "4" || prn.substr(0, 1) == "5") && _sys.count("LEO") > 0)
            return true;

        gs = gnss_sys::char2str(prn[0]); // DONT USE SYS FROM ARG WHICH MAY BE EMPTY
        gsys = gnss_sys::str2gsys(gs);   // DONT USE SYS FROM ARG WHICH MAY BE EMPTY

        if ((_sys.size() == 0 || _sys.find(gs) != _sys.end()) &&
            (_sat[gsys].size() == 0 || _sat[gsys].find(prn) != _sat[gsys].end()))
        {
            return true;
        }

        return false;
    }
} // namespace