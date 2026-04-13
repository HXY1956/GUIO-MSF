#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "hwa_gnss_sys.h"
#include "hwa_base_common.h"

using namespace hwa_set;

namespace hwa_gnss
{
    gnss_sys::gnss_sys(const GSYS &sys) : _gsys(sys)
    {
    }

    gnss_sys::gnss_sys(const std::string &sys)
        : _gsys(GNS)
    {
        if (sys.size() > 0)
            _gsys = str2gsys(sys);
    }

    gnss_sys::gnss_sys(const char &c)
    {
        _gsys = char2gsys(c);
    }

    // --------------------------------------------------------
    // STATIC FUNCTIONS
    // --------------------------------------------------------
    // get system band
    // ----------
    GOBSBAND gnss_sys::gfrq2band(const GSYS &gs, const GFRQ &gfreq)
    {
        // jdhuang

        //hwa_map_freq m = GNSS_FREQ_PRIORITY;
        if (gs == GNS || gfreq == LAST_GFRQ)
            return BAND;
        try
        {
            hwa_vec_freq sys_freq = GNSS_FREQ_PRIORITY.at(gs);
            unsigned int size = sys_freq.size();
            for (unsigned int i = 0; i < size; ++i)
            {
                if (sys_freq.at(i) == gfreq)
                    return band_priority(gs, FREQ_SEQ(i));
            }
            return BAND;
        }
        catch (...)
        {
            return BAND;
        }
    }

    // get system freq
    // ----------
    GFRQ gnss_sys::band2gfrq(const GSYS &gs, const GOBSBAND &gband)
    {
        if (gs == GNS || gband == BAND)
            return LAST_GFRQ;
        try
        {
            hwa_vec_band sys_band = GNSS_BAND_PRIORITY.at(gs);
            unsigned int size = sys_band.size();
            for (unsigned int i = 0; i < size; ++i)
            {
                if (sys_band.at(i) == gband)
                    return freq_priority(gs, FREQ_SEQ(i));
            }
            return LAST_GFRQ;
        }
        catch (...)
        {
            return LAST_GFRQ;
        }
    }

    // get system sequence
    // ----------
    FREQ_SEQ gnss_sys::gfrq2freq(const GSYS &gs, const GFRQ &gfreq)
    {
        if (gs == GNS || gfreq == LAST_GFRQ)
            return FREQ_X;
        try
        {
            hwa_vec_freq sys_freq = GNSS_FREQ_PRIORITY.at(gs);
            unsigned int size = sys_freq.size();
            for (unsigned int i = 0; i < size; ++i)
            {
                if (sys_freq.at(i) == gfreq)
                    return FREQ_SEQ(i);
            }
            return FREQ_X;
        }
        catch (...)
        {
            return FREQ_X;
        }
    }

    // get system sequence
    // ----------
    GSYS gnss_sys::gfrq2gsys(const GFRQ &freq)
    {
        std::string s = t_gfreq::gfreq2str(freq);
        return gnss_sys::char2gsys(s[0]);
    }

    FREQ_SEQ gnss_sys::str2freq(const std::string &freq)
    {
        if (freq.empty())
            return FREQ_SEQ::FREQ_X;

        char cfreq = freq[0];
        switch (cfreq)
        {
        case '0':
            return FREQ_SEQ::FREQ_X;
        case '1':
            return FREQ_SEQ::FREQ_1;
        case '2':
            return FREQ_SEQ::FREQ_2;
        case '3':
            return FREQ_SEQ::FREQ_3;
        case '4':
            return FREQ_SEQ::FREQ_4;
        case '5':
            return FREQ_SEQ::FREQ_5;
        case '6':
            return FREQ_SEQ::FREQ_6;
        case '7':
            return FREQ_SEQ::FREQ_7;
        default:
            return FREQ_SEQ::FREQ_X;
        }
    }

    std::string gnss_sys::freq2str(const FREQ_SEQ &freq)
    {
        switch (freq)
        {
        case FREQ_SEQ::FREQ_X:
            return std::string("0");
        case FREQ_SEQ::FREQ_1:
            return std::string("1");
        case FREQ_SEQ::FREQ_2:
            return std::string("2");
        case FREQ_SEQ::FREQ_3:
            return std::string("3");
        case FREQ_SEQ::FREQ_4:
            return std::string("4");
        case FREQ_SEQ::FREQ_5:
            return std::string("5");
        case FREQ_SEQ::FREQ_6:
            return std::string("6");
        case FREQ_SEQ::FREQ_7:
            return std::string("7");

        default:
            return std::string("0");
        }
    }

    // get system freq sequence
    // ----------
    FREQ_SEQ gnss_sys::band2freq(const GSYS &gs, const GOBSBAND &gband)
    {
        if (gs == GNS || gband == BAND)
            return FREQ_X;
        try
        {
            hwa_vec_band sys_band = GNSS_BAND_PRIORITY.at(gs);
            unsigned int size = sys_band.size();
            for (unsigned int i = 0; i < size; ++i)
            {
                if (sys_band.at(i) == gband)
                    return FREQ_SEQ(i);
            }
            return FREQ_X;
        }
        catch (...)
        {
            return FREQ_X;
        }
    }

    // get band selection
    // ----------
    GOBSBAND gnss_sys::band_priority(const GSYS &gs, const FREQ_SEQ &iseq) // FREQ_SEQ priority band
    {
        // jdhuang
        //// origin
        //static hwa_map_band m = GNSS_BAND_PRIORITY;

        //if (gs == GNS || iseq >= m[gs].size()) return BAND;

        //return m[gs][iseq];
        //// origin

        try
        {
            if (gs == GNS || iseq >= GNSS_BAND_PRIORITY.at(gs).size())
                return BAND;
            return GNSS_BAND_PRIORITY.at(gs).at(iseq);
        }
        catch (...)
        {
            return BAND;
        }
    }

    GOBS gnss_sys::gobs_priority(const GSYS &gs, const GOBS &gobs)
    {
        if (gs == GPS)
        {
            switch (gobs)
            {
            case C1C:
            case C1P:
            case C1W:
            {
                return C1W;
            }
            case C2C:
            case C2L:
            case C2X:
            case C2P:
            case C2W:
            {
                return C2W;
            }
            case C5Q:
            case C5X:
            {
                return C5X;
            }
            default:
                return GOBS::X;
            }
        }
        else if (gs == GLO)
        {
            switch (gobs)
            {
            case C1C:
            case C1P:
                return C1P;
            case C2C:
            case C2P:
                return C2P;
            default:
                return GOBS::X;
            }
        }
        else
        {
            return gobs;
        }
    }

    GOBS gnss_sys::gobs_priority(const GSYS &gs, const GOBSBAND &band)
    {
        if (gs == GPS)
        {
            switch (band)
            {
            case BAND_1:
            {
                return C1W;
            }
            case BAND_2:
            {
                return C2W;
            }
            case BAND_5:
            {
                return C5X;
            }
            default:
                return GOBS::X;
            }
        }
        else if (gs == GLO)
        {
            switch (band)
            {
            case BAND_1:
                return C1P;
            case BAND_2:
                return C2P;
            default:
                return GOBS::X;
            }
        }
        else if (gs == GAL)
        {
            switch (band)
            {
            case BAND_1:
                return C1X;
            case BAND_5:
                return C5X;
            case BAND_7:
                return C7X;
            default:
                return GOBS::X;
            }
        }
        else
        {
            return GOBS::X;
        }
    }

    GOBS gnss_sys::gobs_defaults(const GSYS &gs, const GOBS &gobs1, const GOBSBAND &band)
    {
        if (gs == GPS)
        {
            switch (band)
            {
            case BAND_1:
            {
                return C1W;
            }
            case BAND_2:
            {
                return C2W;
            }
            case BAND_5:
            {
                return C5X;
            }
            default:
                return GOBS::X;
            }
        }
        else if (gs == GLO)
        {
            switch (band)
            {
            case BAND_1:
                return C1P;
            case BAND_2:
                return C2P;
            default:
                return GOBS::X;
            }
        }
        else if (gs == GAL)
        {
            std::string strGOBS = gobs2str(gobs1);
            switch (band)
            {
            case BAND_1:
            {
                if (strGOBS[strGOBS.size() - 1] == 'X')
                {
                    return C1X;
                }
                else
                {
                    return C1C;
                }
            }
            case BAND_5:
            {
                if (strGOBS[strGOBS.size() - 1] == 'X')
                {
                    return C5X;
                }
                else
                {
                    return C5Q;
                }
            }
            case BAND_7:
            {
                if (strGOBS[strGOBS.size() - 1] == 'X')
                {
                    return C7X;
                }
                else
                {
                    return C7Q;
                }
            }
            default:
                return GOBS::X;
            }
        }
        else
        {
            return GOBS::X;
        }
    }

    // get freq selection
    // ----------
    GFRQ gnss_sys::freq_priority(const GSYS &gs, const FREQ_SEQ &iseq) // FREQ_SEQ priority frequency
    {
        try
        {
            if (gs == GNS || iseq >= GNSS_FREQ_PRIORITY.at(gs).size())
                return LAST_GFRQ;

            return GNSS_FREQ_PRIORITY.at(gs).at(iseq);
        }
        catch (...)
        {
            return LAST_GFRQ;
        }
    }

    // get attr selection
    // ----------
    GOBSATTR gnss_sys::attr_priority(const GSYS &gs, const GOBSBAND &gb, const GOBSTYPE &gt, const unsigned int &iseq) // iseq priority sequence
    {
        hwa_map_gnss m = gnss_data_priority();

        if (gs == GNS ||
            gt == TYPE ||
            gb == BAND || iseq > m[gs][gb][gt].size())
            return ATTR;

        return m[gs][gb][gt][iseq];
    }

    // convert GSYS enum to GSYS std::string
    // ----------
    std::string gnss_sys::gsys2str(const GSYS &sys)
    {
        // PROBLEM WITH STATIC FUNCTION
        //  boost::mutex::scoped_lock lock(_mutex);
        switch (sys)
        {
        case GPS:
            return "GPS";
        case GLO:
            return "GLO";
        case GAL:
            return "GAL";
        case BDS:
            return "BDS";
        case SBS:
            return "SBS";
        case QZS:
            return "QZS";
        case IRN:
            return "IRN";
        case LEO:
            return "LEO";
        case GNS:
            return "GNS";

        default:
        {
            std::cout << "*** warning: unknown GNSS system!\n";
            std::cout.flush();
        }
        }
        return "GNS";
    }

    // convert GSYS enum to GSYS char
    // ----------
    char gnss_sys::gsys2char(const GSYS &sys)
    {

        // PROBLEM WITH STATIC FUNCTION
        //  boost::mutex::scoped_lock lock(_mutex);

        switch (sys)
        {
        case GPS:
            return 'G';
        case GLO:
            return 'R';
        case GAL:
            return 'E';
        case BDS:
            return 'C';
        case SBS:
            return 'S';
        case QZS:
            return 'J';
        case IRN:
            return 'I';
        case LEO:
            return 'L';
        case GNS:
            return 'X';

        default:
        {
            std::cout << "*** warning: unknown GNSS system \n";
            std::cout.flush();
        }
            // cannot be while static function !!!
            // if( _log ) _log->comment(0,"gobsgnss","warning: unknown frequency code ");
            //     else
        }

        return 'X';
    }
    // convert GSYS std::string to GSYS enum
    // ----------
    std::string gnss_sys::str2strfirst(const std::string &tmp)
    {

        // PROBLEM WITH STATIC FUNCTION
        //  boost::mutex::scoped_lock lock(_mutex);
        std::string s(tmp);
        if (s.size() == 0)
        {
            std::cout << "*** warning: not defined GNSS system code [NULL]\n";
            std::cout.flush();
            return "X";
        }

        transform(s.begin(), s.end(), s.begin(), ::toupper);

        if (s == "G" || s == "GPS" || s == "NAVSTAR")
            return "G";
        else if (s == "R" || s == "GLO" || s == "GLONASS")
            return "R";
        else if (s == "E" || s == "GAL" || s == "GALILEO")
            return "E";
        else if (s == "C" || s == "COMP" || s == "COMPASS")
            return "C";
        else if (s == "C" || s == "BDS" || s == "BEIDOU")
            return "C";
        else if (s == "S" || s == "SBS" || s == "EGNOS")
            return "S";
        else if (s == "S" || s == "SBAS")
            return "S";
        else if (s == "J" || s == "QZS" || s == "JAXA")
            return "J";
        else if (s == "J" || s == "QZSS")
            return "J";
        else if (s == "I" || s == "IRN" || s == "IRNSS")
            return "I";
        else
        {
            std::cout << "*** warning: not defined GNSS system code [" << s[0] << "]\n";
            std::cout.flush();
        }

        return "X";
    }

    // convert GSYS std::string to GSYS enum
    // ----------
    GSYS gnss_sys::str2gsys(const std::string &tmp)
    {

        // PROBLEM WITH STATIC FUNCTION
        //  boost::mutex::scoped_lock lock(_mutex);
        std::string s(tmp);
        if (s.size() == 0)
        {
            std::cout << "*** warning: not defined GNSS system code [NULL]\n";
            std::cout.flush();
            return GNS;
        }

        transform(s.begin(), s.end(), s.begin(), ::toupper);

        if (s == "G" || s == "GPS" || s == "NAVSTAR")
            return GPS;
        else if (s.substr(0, 1) == "2" || s.substr(0, 1) == "3" || s.substr(0, 1) == "4" || s.substr(0, 1) == "5")
            return GPS; // add for simu leo ppp
        else if (s == "R" || s == "GLO" || s == "GLONASS")
            return GLO;
        else if (s == "E" || s == "GAL" || s == "GALILEO")
            return GAL;
        else if (s == "C" || s == "COMP" || s == "COMPASS")
            return BDS;
        else if (s == "C" || s == "BDS" || s == "BEIDOU")
            return BDS;
        else if (s == "S" || s == "SBS" || s == "EGNOS")
            return SBS;
        else if (s == "S" || s == "SBAS")
            return SBS;
        else if (s == "J" || s == "QZS" || s == "JAXA")
            return QZS;
        else if (s == "J" || s == "QZSS")
            return QZS;
        else if (s == "I" || s == "IRN" || s == "IRNSS")
            return IRN;
        else if (s == "L" || s == "LEO")
            return LEO;
        else if (s == "X" || s == "GNS" || s == "GNSS")
            return GNS;
        else if (s == "M")
            return GNS;
        else
        {
            std::cout << "*** warning: not defined GNSS system code [" << s[0] << "]\n";
            std::cout.flush();
        }

        return GNS;
    }

    GSYS gnss_sys::sat2gsys(const std::string &s)
    {
        std::string tmp = s.substr(0, 1);
        if (s.empty())
            return GNS;
        else
            return str2gsys(tmp);
    }

    // convert GSYS std::string to GSYS enum
    // ----------
    char gnss_sys::str2char(const std::string &s)
    {
        if (s.size() > 0)
            return gsys2char(str2gsys(s));
        return 'X';
    }

    // convert GSYS char to GSYS enum
    // ----------
    GSYS gnss_sys::char2gsys(char c)
    {

        if (c == 'G')
            return GPS;
        else if (c == 'R')
            return GLO;
        else if (c == 'E')
            return GAL;
        else if (c == 'C')
            return BDS;
        else if (c == 'S')
            return SBS;
        else if (c == 'J')
            return QZS;
        else if (c == 'I')
            return IRN;
        else if (c == 'M')
            return GNS;
        else if (c == 'X')
            return GNS;
        else
        {
            std::cout << "*** warning: not defined GNSS system char [" << c << "]\n";
            std::cout.flush();
        }

        return GNS;
    }

    GSYS gnss_sys::char2gsys_addleo(char c)
    {

        if (c == 'G' || c == '2' || c == '3' || c == '4' || c == '5')
            return GPS;
        else if (c == 'R')
            return GLO;
        else if (c == 'E')
            return GAL;
        else if (c == 'C')
            return BDS;
        else if (c == 'S')
            return SBS;
        else if (c == 'J')
            return QZS;
        else if (c == 'I')
            return IRN;
        else if (c == 'M')
            return GNS;
        else if (c == 'X')
            return GNS;
        else
        {
            //std::cout << "*** warning: not defined GNSS system char [" << c << "]\n"; std::cout.flush(); hxj
        }

        return GNS;
    }

    // convert GSYS char to GSYS enum
    // ----------
    std::string gnss_sys::char2str(char c)
    {
        return gsys2str(char2gsys(c));
    }

    // convert satellite name
    // ----------
    std::string gnss_sys::eval_sat(const std::string &sat, const GSYS &sys)
    {
        std::istringstream is(sat); // .substr(0,3) NE!
        GSYS gnss = sys;
        int svn = 0;
        char chr = 'G'; // if empty, use GPS
        size_t l = is.str().length();

        if (l == 0)
            return "X00";

        // Slow!!
        //if( l < 3 || sat[0] == ' ' )  is        >> svn;
        //else                          is >> chr >> svn;

        // Change by ZhengHJ
        if (l < 3 || sat[0] == ' ')
            sscanf(sat.c_str(), "%d", &svn);
        else
            sscanf(sat.c_str(), "%c%d", &chr, &svn);

        if (is.fail())
        {
            return "X00";
        }
        if (chr != 'G')
            gnss = char2gsys(chr);

        switch (gnss)
        {
        case GPS:
            chr = 'G';
            break;
        case GLO:
            chr = 'R';
            break;
        case GAL:
            chr = 'E';
            break;
        case BDS:
            chr = 'C';
            break;
        case SBS:
            chr = 'S';
            break;
        case QZS:
            chr = 'J';
            break;
        case IRN:
            chr = 'I';
            break;
        case GNS:
            chr = 'X';
            break;
        default:
            chr = 'X';
            break;
        }

        if (svn > QZS_OFFSET)
        {
            svn -= QZS_OFFSET;
        }
        if (svn > SBS_OFFSET)
        {
            svn -= SBS_OFFSET;
        }

        // SLow!!
        //std::ostringstream os;
        //os << std::setw(1) << chr << setfill('0') << std::setw(2) << svn;

        // Change By ZhengHJ
        char tmp[4];
        sprintf(tmp, "%c%02d", chr, svn);

#ifdef DEBUG3
        std::cout << "  ARGM: [" << sat << "] " << gsys2str(sys)
             << "  EVAL: " << gsys2str(gnss) << " [" << chr << "] " << svn
             << "  RESU: " << os.str() << " " << l << std::endl;
        std::cout.flush();
#endif

        //return os.str();
        std::string tmp_svn(tmp);
        return tmp_svn;
    }

    std::string gnss_sys::eval_sat_addleo(const std::string &sat, const GSYS &sys)
    {
        std::istringstream is(sat); // .substr(0,3) NE!
        GSYS gnss = sys;
        int svn = 0;
        char chr = 'G'; // if empty, use GPS
        size_t l = is.str().length();

        if (l == 0)
            return "X00";

        if (l < 3 || sat[0] == ' ')
            is >> svn;
        else
            is >> chr >> svn;

        if (chr == '2' || chr == '3' || chr == '4' || chr == '5')
        {
            return sat;
        }

        if (is.fail())
        {
            return "X00";
        }
        if (chr != 'G')
            gnss = char2gsys_addleo(chr);

        switch (gnss)
        {
        case GPS:
            chr = 'G';
            break;
        case GLO:
            chr = 'R';
            break;
        case GAL:
            chr = 'E';
            break;
        case BDS:
            chr = 'C';
            break;
        case SBS:
            chr = 'S';
            break;
        case QZS:
            chr = 'J';
            break;
        case IRN:
            chr = 'I';
            break;
        case GNS:
            chr = 'X';
            break;
        default:
            chr = 'X';
            break;
        }

        if (svn > QZS_OFFSET)
        {
            svn -= QZS_OFFSET;
        }
        if (svn > SBS_OFFSET)
        {
            svn -= SBS_OFFSET;
        }

        std::ostringstream os;
        os << std::setw(1) << chr << std::setfill('0') << std::setw(2) << svn;

#ifdef DEBUG3
        std::cout << "  ARGM: [" << sat << "] " << gsys2str(sys)
             << "  EVAL: " << gsys2str(gnss) << " [" << chr << "] " << svn
             << "  RESU: " << os.str() << " " << l << std::endl;
        std::cout.flush();
#endif
        return os.str();
    }

    // convert satellite name
    // ----------
    std::string gnss_sys::eval_sat(const int &svn, const GSYS &sys)
    {
        char chr;
        switch (sys)
        {
        case GPS:
            chr = 'G';
            break;
        case GLO:
            chr = 'R';
            break;
        case GAL:
            chr = 'E';
            break;
        case BDS:
            chr = 'C';
            break;
        case SBS:
            chr = 'S';
            break;
        case QZS:
            chr = 'J';
            break;
        case IRN:
            chr = 'I';
            break;
        case GNS:
            chr = 'X';
            break;
        default:
            chr = 'X';
            break;
        }

        int num = svn;
        if (svn > QZS_OFFSET)
        {
            num -= QZS_OFFSET;
        }
        if (svn > SBS_OFFSET)
        {
            num -= SBS_OFFSET;
        }

        std::ostringstream os;
        os << std::setw(1) << chr << std::setfill('0') << std::setw(2) << num;

        return os.str();
    }

    // std::set GSYS from std::string
    // ----------
    void gnss_sys::from_string(const std::string &sys)
    {
        _gsys = str2gsys(sys);
    }

    // std::set GSYS from enum
    // ----------
    void gnss_sys::from_gsys(const GSYS &sys)
    {
        _gsys = sys;
    }

    // overloaded equivalence operator
    // ----------
    bool gnss_sys::operator==(const std::string &sys) const
    {
        //  transform(sys.begin(), sys.end(), sys.begin(), ::toupper);
        if (_gsys == str2gsys(sys))
            return true;
        return false;
    }

    // overloaded equivalence operator
    // ----------
    bool gnss_sys::operator==(const GSYS &sys) const
    {
        //  if( _gsys == sys.gsys() ) return true;
        return false;
    }

    /*
    // decode band number to frequency number
    // -----------------------------------
    FREQ_SEQ gnss_sys::band2freqID(GSYS gs, GOBSBAND band)
    {
       

       int freq = 0;

       switch (gs) {
        case GPS :
          if(band == BAND_1) freq = 1;
          if(band == BAND_2) freq = 2;
          if(band == BAND_5) freq = 3;
          break;
        case GLO :
          if(band == BAND_1) freq = 1;
          if(band == BAND_2) freq = 2;
          if(band == BAND_3) freq = 3;
          if(band == BAND_7) freq = 3;
          if(band == BAND_5) freq = 4;
          break;
        case GAL:
          if(band == BAND_1) freq = 1;
          if(band == BAND_6) freq = 2;
          if(band == BAND_7) freq = 3;
          if(band == BAND_8) freq = 4;
          if(band == BAND_5) freq = 5;
          break;
        case BDS:
          if(band == BAND_2) freq = 1;
          if(band == BAND_6) freq = 2;
          if(band == BAND_7) freq = 3;
          break;
        case QZS:
          if(band == BAND_1) freq = 1;
          if(band == BAND_6) freq = 2;
          if(band == BAND_2) freq = 3;
          if(band == BAND_5) freq = 4;
          break;
        case SBS:
          if(band == BAND_1) freq = 1;
          if(band == BAND_2) freq = 2;
          if(band == BAND_5) freq = 3;
          break;
        default:
          break;
       }
       return freq;
    }

    // decode frequency number to band number
    // -----------------------------------
    GOBSBAND gnss_sys::freq2band(GSYS gs, int freq)
    {
       

    //   GSYS gs = this->gsys();

       GOBSBAND band = BAND;

       switch (gs) {
        case GPS :
          if(freq == 1) band = BAND_1;
          if(freq == 2) band = BAND_2;
          if(freq == 3) band = BAND_5;
          break;
        case GLO :
          if(freq == 1) band = BAND_1;
          if(freq == 2) band = BAND_2;
          if(freq == 3) band = BAND_3;
          if(freq == 4) band = BAND_5;
          break;
        case GAL:
          if(freq == 1) band = BAND_1;
          if(freq == 2) band = BAND_6;
          if(freq == 3) band = BAND_7;
          if(freq == 4) band = BAND_8;
          if(freq == 5) band = BAND_5;
          break;
        case BDS:
          if(freq == 1) band = BAND_2;
          if(freq == 2) band = BAND_6;
          if(freq == 3) band = BAND_7;
          break;
        case QZS:
          if(freq == 1) band = BAND_1;
          if(freq == 2) band = BAND_6;
          if(freq == 3) band = BAND_2;
          if(freq == 4) band = BAND_5;
          break;
        case SBS:
          if(freq == 1) band = BAND_1;
          if(freq == 2) band = BAND_2;
          if(freq == 3) band = BAND_5;
          break;
        default:
          break;
       }
       return band;
    }
    */

    // ---------------------------------------------------------------------------------
    // class FREQ
    // ---------------------------------------------------------------------------------

    // get GFRQ enum from std::string
    // ----------
    GFRQ t_gfreq::str2gfreq(const std::string &freq_const)
    {

        // PROBLEM WITH STATIC FUNCTION
        //  boost::mutex::scoped_lock lock(_mutex);
        std::string freq(freq_const);
        transform(freq.begin(), freq.end(), freq.begin(), ::toupper);

        if (freq == "G01")
            return G01;
        else if (freq == "G02")
            return G02;
        else if (freq == "G05")
            return G05;
        else if (freq == "R01")
            return R01;
        else if (freq == "R02")
            return R02;
        //else if( freq == "R01" ) return R01_CDMA;
        //else if( freq == "R02" ) return R02_CDMA;
        else if (freq == "R03")
            return R03_CDMA;
        else if (freq == "R05")
            return R05_CDMA;
        else if (freq == "E01")
            return E01;
        else if (freq == "E05")
            return E05;
        else if (freq == "E07")
            return E07;
        else if (freq == "E08")
            return E08;
        else if (freq == "E06")
            return E06;
        else if (freq == "C02")
            return C02;
        else if (freq == "C07")
            return C07; // change glfeng
        else if (freq == "C06")
            return C06; // change glfeng
        else if (freq == "C01")
            return C01; // change glfeng
        else if (freq == "C08")
            return C08; // change glfeng
        else if (freq == "C05")
            return C05; // change glfeng
        else if (freq == "C09")
            return C09; // change glfeng for bds3 B2b
        else if (freq == "J01")
            return J01;
        else if (freq == "J02")
            return J02;
        else if (freq == "J05")
            return J05;
        else if (freq == "J06")
            return J06;
        else if (freq == "S01")
            return S01;
        else if (freq == "S05")
            return S05;
        else if (freq == "I05")
            return I05;
        //else if( freq == "I09" ) return I09;
        //else if( freq.empty()        ) return LAST_GFRQ;
        else if (freq == "LAST_GFRQ")
            return LAST_GFRQ;

        //  std::cout << "*** warning: not defined frequency code [" + freq + "]\n"; std::cout.flush();

        // cannot be while static function !!!
        //  if( _log ) _log->comment(0,"gobsgnss","warning: not defined frequency code " + freq);
        //  else                      std::cout << "*** warning: not defined frequency code " + freq << std::endl;

        return LAST_GFRQ;
    }

    // get std::string from GFRQ enum
    // ----------
    std::string t_gfreq::gfreq2str(const GFRQ &freq)
    {

        // PROBLEM WITH STATIC FUNCTION
        //  boost::mutex::scoped_lock lock(_mutex);

        switch (freq)
        {
        case G01:
            return "G01";
        case G02:
            return "G02";
        case G05:
            return "G05";
        case R01:
            return "R01";
        case R02:
            return "R02";
        case R01_CDMA:
            return "R01";
        case R02_CDMA:
            return "R02";
        case R03_CDMA:
            return "R03";
        case R05_CDMA:
            return "R05";
        case E01:
            return "E01";
        case E05:
            return "E05";
        case E07:
            return "E07";
        case E08:
            return "E08";
        case E06:
            return "E06";
        case C02:
            return "C02";
        case C06:
            return "C06"; // change glfeng
        case C07:
            return "C07"; // change glfeng
        case C01:
            return "C01"; // change glfeng
        case C08:
            return "C08"; // change glfeng
        case C05:
            return "C05"; // change glfeng
        case C09:
            return "C09"; // change glfeng for Bds-3 B2b
        case J01:
            return "J01";
        case J02:
            return "J02";
        case J05:
            return "J05";
        case J06:
            return "J06";
        case S01:
            return "S01";
        case S05:
            return "S05";
        case I05:
            return "I05";
            // case I09 : return "I09";
        case LAST_GFRQ:
            return "LAST_GFRQ";

        default:
        {
            std::cout << "*** warning: unknown frequency code \n";
            std::cout.flush();
        }

            // cannot be while static function !!!
            // if( _log ) _log->comment(0,"gobsgnss","warning: unknown frequency code ");
            //     else
        }

        return "LAST_GFRQ";
    }

    // get true if BDS GEO satellite
    // ------------------------------
    bool gnss_sys::bds_geo(const std::string &sat)
    {
        std::set<std::string> geo; // prns of geost. satellites
        geo.insert("C01");
        geo.insert("C02");
        geo.insert("C03");
        geo.insert("C04");
        geo.insert("C05");
        //add by lijie BDS3-GEO
        geo.insert("C59");
        geo.insert("C60");
        if (geo.find(sat) != geo.end())
            return true;

        return false;
    }

    // added by yqyuan for CYS mode of four BDS-2 and all BDS-3 CAST M/I satellites
    // get BDS satellite following CAST CYS attitude law
    // ------------------------------
    bool gnss_sys::bds_cast(const std::string &sat)
    {
        std::set<std::string> cys;
        cys.insert("C13"); // BDS-2 IGSO-6b C-017
        cys.insert("C14"); // BDS-2 MEO-6   C015
        cys.insert("C06"); // BDS-2 IGSO-1  C005
        cys.insert("C16"); // BDS-2 IGSO-7  C019
        cys.insert("C19"); // BDS-3 MEO by CAST; 14 satellites
        cys.insert("C20");
        cys.insert("C21");
        cys.insert("C22");
        cys.insert("C23");
        cys.insert("C24");
        cys.insert("C32");
        cys.insert("C33");
        cys.insert("C36");
        cys.insert("C37");
        cys.insert("C41");
        cys.insert("C42");
        cys.insert("C45");
        cys.insert("C46");
        cys.insert("C38"); // all BDS-3 IGSO by CAST; 3 satellites
        cys.insert("C39");
        cys.insert("C40");
        //cys.insert("C25"); // BDS-3 MEO by SECM; 10 satellites
        //cys.insert("C26");
        //cys.insert("C27");
        //cys.insert("C28");
        //cys.insert("C29");
        //cys.insert("C30");
        //cys.insert("C34");
        //cys.insert("C35");
        //cys.insert("C43");
        //cys.insert("C44");
        if (cys.find(sat) != cys.end())
            return true;
        return false;
    }

    // added by yqyuan for CYS mode of 10 BDS-3 SECM M satellites
    // get BDS satellite following SECM CYS attitude law
    // ------------------------------
    bool gnss_sys::bds_secm(const std::string &sat)
    {
        std::set<std::string> cys;
        cys.insert("C25"); // BDS-3 MEO by SECM; 10 satellites
        cys.insert("C26");
        cys.insert("C27");
        cys.insert("C28");
        cys.insert("C29");
        cys.insert("C30");
        cys.insert("C34");
        cys.insert("C35");
        cys.insert("C43");
        cys.insert("C44");
        if (cys.find(sat) != cys.end())
            return true;

        return false;
    }
} // namespace
