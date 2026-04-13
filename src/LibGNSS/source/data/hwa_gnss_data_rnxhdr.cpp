#include "hwa_gnss_data_rnxhdr.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_rnxhdr::gnss_data_rnxhdr()
    {
    }

    gnss_data_rnxhdr::~gnss_data_rnxhdr()
    {
    }

    void gnss_data_rnxhdr::clear()
    {
        _gtime = base_time::current_time(base_time::GPS);
        _first = FIRST_TIME;
        _last = LAST_TIME;

        _path = "";
        _rnxsys = 'G';
        _rnxver = 'x';
        _program = "";
        _runby = "";
        _markname = "";
        _marknumb = "";
        _marktype = "";
        _observer = "";
        _agency = "";
        _recnumb = "";
        _rectype = "";
        _recvers = "";
        _antnumb = "";
        _anttype = "";
        _strength = "DBHZ";
        _interval = 0;
        _leapsec = 0;
        _numsats = 0;

        _aprxyz = Triple(0, 0, 0);
        _antxyz = Triple(0, 0, 0);
        _antneu = Triple(0, 0, 0);

        _mapobs.clear();

        _comment.clear();
    }

    // overloading << operator
    // -----------------------------
    std::ostream &operator<<(std::ostream &os, const gnss_data_rnxhdr &x)
    {
        os << "program: " << x.program() << std::endl;
        os << "runby: " << x.runby() << std::endl;
        os << "gtime: " << x.gtime().str_hms() << std::endl;
        os << "markname: " << x.markname() << std::endl;
        os << "marknumb: " << x.marknumb() << std::endl;
        os << "marktype: " << x.marktype() << std::endl;
        os << "observer: " << x.observer() << std::endl;
        os << "agency: " << x.agency() << std::endl;
        os << "recnumb: " << x.recnumb() << std::endl;
        os << "rectype: " << x.rectype() << std::endl;
        os << "recvers: " << x.recvers() << std::endl;
        os << "antnumb: " << x.antnumb() << std::endl;
        os << "anttype: " << x.anttype() << std::endl;
        os << "aprxyz: " << x.aprxyz() << std::endl;
        os << "antxyz: " << x.antxyz() << std::endl;
        os << "antneu: " << x.antneu() << std::endl;
        os << "strength: " << x.strength() << std::endl;
        os << "interval: " << x.interval() << std::endl;
        os << "first: " << x.first().str_hms() << std::endl;
        os << "last: " << x.last().str_hms() << std::endl;
        os << "leapsec: " << x.leapsec() << std::endl;
        os << "numsats: " << x.numsats() << std::endl;
        return os;
    }

} // namespace
