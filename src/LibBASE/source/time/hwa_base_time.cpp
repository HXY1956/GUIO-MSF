#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <algorithm>
#include <io.h>
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"

namespace hwa_base
{
    base_time::base_time(const base_timesys &ts)
        : _mjd(0), _sod(0), _dsec(0.0), _tsys(UTC)
    {
        from_time(time(NULL), 0.0, true); // input UTC time-system
        _tsys = ts;                       // now switch to required TS

        _reset_conv(); // add glfeng
    }

    //base_time::base_time(const base_time& other)
    //{
    //    this->_mjd  = other._mjd;     // integer MJD [TAI]
    //    this->_sod  = other._sod;     // seconds of day [TAI]
    //    this->_dsec = other._dsec;    // base_type_conv::fractional seconds
    //    this->_tsys = other._tsys;    // time system
    //}

    // constructor
    // ----------
    base_time::base_time(const time_t &tt, const double &ds, const base_timesys &ts)
        : _mjd(0), _sod(0), _dsec(0.0), _tsys(UTC)
    {
        from_time(tt, ds, true); // input UTC time-system
        _tsys = ts;              // now switch to required TS

        _reset_conv(); // add glfeng
    }

    // constructor
    // ----------
    base_time::base_time(const int &yr, const int &mn,
                     const int &dd, const int &hr,
                     const int &mi, const int &sc,
                     const double &ds, const base_timesys &ts)
        : _mjd(0), _sod(0), _dsec(0.0), _tsys(ts)
    {
        from_ymdhms(yr, mn, dd, hr, mi, sc + ds, true);
        // old style
        // from_ymd(yr,mn,dd,static_cast<int>(hr*3600 + mi*60 + sc),ds,true);
    }

    // constructor
    // ----------
    base_time::base_time(const int &gw, const int &dw, const int &sd,
                     const double &ds, const base_timesys &ts)
        : _mjd(0), _sod(0), _dsec(0.0), _tsys(ts)
    {
        from_gwd(gw, dw, sd, ds, true);
    }

    // constructor
    // ----------
    base_time::base_time(const int &gw, const double &sow, const base_timesys &ts)
        : _mjd(0), _sod(0), _dsec(0.0), _tsys(ts)
    {
        from_gws(gw, sow, true);
    }

    // constructor
    // ----------
    base_time::base_time(const int &mjd, const int &sd,
                     const double &ds, const base_timesys &ts)
        : _mjd(0), _sod(0), _dsec(0.0), _tsys(ts)
    {
        from_mjd(mjd, sd, ds, true);
    }

    // destructor
    // ----------
    base_time::~base_time()
    {
    }

    // convert std::string to tsys
    // ----------
    std::string base_time::tsys2str(const base_timesys &ts)
    {
        switch (ts)
        {
        case USER:
            return "USER";
        case TAI:
            return "TAI";
        case UTC:
            return "UTC";
        case LOC:
            return "LOC";
        case GPS:
            return "GPS";
        case GLO:
            return "GLO";
        case GAL:
            return "GAL";
        case BDS:
            return "BDS";
        case TT:
            return "TT";

        default:
        {
            std::cout << "*** warning: unknown time system!\n";
            std::cout.flush();
        }
        }
        return "TAI";
    }

    // convert tsys to std::string
    // ----------
    base_time::base_timesys base_time::str2tsys(const std::string &tmp)
    {
        std::string s(tmp);
        if (s.size() == 0)
        {
            std::cout << "*** warning: not defined time system\n";
            std::cout.flush();
            return TAI;
        }

        transform(s.begin(), s.end(), s.begin(), ::toupper);

        if (s == "USER")
            return USER;
        else if (s == "TAI")
            return TAI;
        else if (s == "UTC")
            return UTC;
        else if (s == "LOC")
            return LOC;
        else if (s == "GPS")
            return GPS;
        else if (s == "GLO")
            return GLO;
        else if (s == "GAL")
            return GAL;
        else if (s == "BDS")
            return BDS;
        else if (s == "TT")
            return TT;
        else
        {
            std::cout << "*** warning: not defined correct time system [" << s[0] << "]\n";
            std::cout.flush();
        }

        return TAI;
    }

    // std::set current time // acccuracy of 1 sec!
    // ----------
    base_time base_time::current_time(const base_timesys &ts)
    {
        base_time tmp(UTC);
        tmp.from_time(time(NULL), 0.0, true);
        tmp.tsys(ts);
        return tmp;
    }

    // std::set from time
    // -----------
    int base_time::from_time(const time_t &tt, const double &ds, const bool &conv)
    {
        struct tm *tm = gmtime(&tt);
        from_ymd(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                 tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec,
                 ds, conv);

        //  if(conv) _to_tai(); --> ALREADY CONVERTED
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    // std::set from GPS week and second of week
    // -----------
    int base_time::from_gws(const int &gw, const double &sow, const bool &conv)
    {
        int dw = (int)floor(sow / 86400);
        _mjd = 44244 + 7 * gw + dw;
        _sod = (int)floor(sow - dw * 86400.0);
        _dsec = sow - dw * 86400.0 - _sod;

        if (conv)
            _to_tai(); // CONVERT
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    // std::set from GPS week and dow of week
    // -----------
    int base_time::from_gwd(const int &gw, const int &dw, const int &sd,
                          const double &ds, const bool &conv)
    {
        _mjd = 44244 + 7 * gw + dw;
        _sod = sd;
        _dsec = ds;

        if (conv)
            _to_tai(); // CONVERT
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    // std::set from year, month, day, second of day, base_type_conv::fractional seconds
    // -----------
    int base_time::from_ymd(const int &yr, const int &mn, const int &dd,
                          const int &sd, const double &ds, const bool &conv)
    {
        int year(yr);
        _norm_year(year);
        _mjd = _ymd_mjd(year, mn, dd);
        _sod = sd;
        _dsec = ds;

        if (conv)
            _to_tai(); // CONVERT
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    // std::set from year, month, day, hour, minute, float seconds
    // -----------
    int base_time::from_ymdhms(const int &yr, const int &mn,
                             const int &dd, const int &h,
                             const int &m, const double &s,
                             const bool &conv)
    {
        int year(yr);
        _norm_year(year);
        _mjd = _ymd_mjd(year, mn, dd);

        _sod = int((h * 3600.0 + m * 60.0) + floor(s));
        _dsec = s - 1.0 * (floor(s));

        if (conv)
            _to_tai(); // CONVERT
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    // std::set from MJD
    // -----------
    int base_time::from_mjd(const int &mjd, const int &sd,
                          const double &ds, const bool &conv)
    {
        _mjd = mjd;
        _sod = sd;
        _dsec = ds;

        if (conv)
            _to_tai(); // CONVERT
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    int base_time::from_dmjd(const double &dmjd, const bool &conv)
    {
        _mjd = (int)dmjd;
        _sod = (int)((dmjd - _mjd) * 86400.0);
        _dsec = (dmjd - _mjd) * 86400.0 - _sod;

        if (conv)
            _to_tai(); // CONVERT
        _norm_dsec();
        _norm_sod();

        _reset_conv();
        return 0;
    }

    // from std::string
    // ----------
    int base_time::from_str(const std::string &ifmt, const std::string &idat, const bool &conv)
    {
        int cYMD = _ymd;
        int cHMS = _hms;
        int y, b, d, h, m, s;
        y = b = d = h = m = s = 0;
        size_t idx;
        size_t fmtadd = 0;
        size_t datadd = 0;
        size_t fmtpos = 0;
        size_t datpos = 0;

        // search and process keys from left to right
        while ((fmtpos < ifmt.length()) && (idx = ifmt.find('%', fmtpos)) != std::string::npos)
        {

            fmtpos++; // a priori encrease

            for (int i = 0; i < MAX_DT; ++i)
            {
                std::string tmp = TD[i];

                // too short ifmt format std::string, skip!
                if (ifmt.length() - idx - 1 < tmp.length())
                    continue;

                // dat std::string not identified, skipped !
                if (ifmt.substr(idx + 1, tmp.length()).compare(tmp) != 0)
                    continue;

                fmtpos = idx + tmp.length() + 1; // end of format reading
                datpos = idx - fmtadd + datadd;  // sum of all idat characters
                fmtadd += tmp.length() + 1;      // sum of all ifmt characters

                if (!tmp.compare("Y"))
                {
                    y = base_type_conv::str2int(idat.substr(datpos, 4));
                    datadd += 4;
                    cYMD += t_tymd(_year);
                }
                else if (!tmp.compare("y"))
                {
                    y = base_type_conv::str2int(idat.substr(datpos, 2));
                    datadd += 2;
                    cYMD += t_tymd(_year);
                }
                else if (!tmp.compare("b"))
                {
                    b = mon(idat.substr(datpos, 3));
                    datadd += 3;
                    cYMD += t_tymd(_mon);
                }
                else if (!tmp.compare("m"))
                {
                    b = base_type_conv::str2int(idat.substr(datpos, 2));
                    datadd += 2;
                    cYMD += t_tymd(_mon);
                }
                else if (!tmp.compare("d"))
                {
                    d = base_type_conv::str2int(idat.substr(datpos, 2));
                    datadd += 2;
                    cYMD += t_tymd(_day);
                }
                else if (!tmp.compare("j"))
                {
                    b = 1;
                    cYMD += t_tymd(_mon);
                    d = base_type_conv::str2int(idat.substr(datpos, 3));
                    datadd += 3;
                    cYMD += t_tymd(_day);
                }
                else if (!tmp.compare("H"))
                {
                    h = base_type_conv::str2int(idat.substr(datpos, 2));
                    datadd += 2;
                    cHMS += t_thms(_hour);
                }
                else if (!tmp.compare("M"))
                {
                    m = base_type_conv::str2int(idat.substr(datpos, 2));
                    datadd += 2;
                    cHMS += t_thms(_min);
                }
                else if (!tmp.compare("S"))
                {
                    s = base_type_conv::str2int(idat.substr(datpos, 2));
                    datadd += 2;
                    cHMS += t_thms(_sec);
                }
                else if (!tmp.compare("W"))
                {
                    y = 1980;
                    cYMD += t_tymd(_year);
                    b = 1;
                    cYMD += t_tymd(_mon);
                    d += 6 + base_type_conv::str2int(idat.substr(datpos, 4)) * 7;
                    datadd += 4;
                }
                else if (!tmp.compare("w"))
                {
                    d += base_type_conv::str2int(idat.substr(datpos, 1));
                    datadd += 1;
                    cYMD += t_tymd(_day);
                }
                else if (!tmp.compare("v"))
                {
                    s += base_type_conv::str2int(idat.substr(datpos, 6));
                    datadd += 6;
                    cYMD += t_tymd(_day);
                    cHMS += t_thms(_hour) + t_thms(_min) + t_thms(_sec);
                }
                else if (!tmp.compare("s"))
                {
                    s = base_type_conv::str2int(idat.substr(datpos, 5));
                    datadd += 5;
                    cHMS += t_thms(_hour) + t_thms(_min) + t_thms(_sec);
                }
                else if (!tmp.compare("I"))
                {
                    y = 2000;
                    cYMD += t_tymd(_year);
                    b = 1;
                    cYMD += t_tymd(_mon);
                    d = 1 - 51544 + (base_type_conv::str2int(idat.substr(datpos, 11)));
                    cYMD += t_tymd(_day);
                    s = 86400 * (base_type_conv::str2int(idat.substr(datpos, 11)) - (base_type_conv::str2int(idat.substr(datpos, 11))));
                    datadd += 11;
                    cHMS += t_thms(_hour) + t_thms(_min) + t_thms(_sec);
                }
                else if (!tmp.compare("J"))
                {
                    y = 2000;
                    cYMD += t_tymd(_year);
                    b = 1;
                    cYMD += t_tymd(_mon);
                    d = 1 - 51544 + base_type_conv::str2int(idat.substr(datpos, 5));
                    datadd += 5;
                    cYMD += t_tymd(_day);
                }
                else if (!tmp.compare("T"))
                {
                    std::cerr << "input time-system conversion not yet supported here!\n";
                }
                else
                {
                    std::cerr << " warning : base_time - unknown date/time identifier [" << tmp << "]\n";
                }
            }
        }

        if (cYMD == (_year | _mon | _day) && cHMS == (_hour | _min | _sec))
        {

            from_ymd(y, b, d, h * 3600 + m * 60 + s, 0.0, conv);

#ifdef DEBUG
            std::cout << " from_str = " << y
                 << " " << b
                 << " " << d
                 << " " << h
                 << " " << m
                 << " " << s
                 << " " << _mjd
                 << " " << _sod
                 << "\n";
#endif

            return 0;
        }

        if (cYMD == (_year | _mon | _day))
        {

#ifdef DEBUG
            std::cout << " from_str = " << y
                 << " " << b
                 << " " << d
                 << " " << h
                 << " " << m
                 << " " << s
                 << "\n";
#endif

            from_ymd(y, b, d, 0, 0.0, conv);
            return 0;
        }

        _reset_conv();
        return 1;
    }

    // reset_dsec - reset dsec only!
    // ----------
    int base_time::reset_dsec()
    {
        _norm_dsec();
        _dsec = 0.0;

        _reset_conv();
        return 0;
    }

    // reset_sod - reset sod + dsec only!
    // ----------
    int base_time::reset_sod()
    {
        _norm_sod();
        _tai_to();
        _sod = 0;
        _to_tai();
        reset_dsec();

        _reset_conv();
        return 0;
    }

    // time zone difference (LOC - UTC) [sec]
    // ----------
    int base_time::tzdiff() const
    {
        switch (_tsys)
        {
        case USER:
            return 0;
        case TAI:
            return 0;
        case UTC:
            return 0;
        case GPS:
            return 0;
        case GLO:
            return 3 * 3600;
        case GAL:
            return 0;
        case BDS:
            return 0;
        case LOC:
        {
            time_t local, utc;
            local = time(NULL);
            utc = mktime(gmtime(&local));
            return int(local - utc);
        }
        default:
            std::cerr << " warning : base_timesys not recognized !\n";
        }
        return 0;
    }

    // day saving time (summer - winter time, e.g. CEST - CET) [sec]
    // ----------
    int base_time::dstime() const
    {
        switch (_tsys)
        {
        case USER:
            return 0;
        case TAI:
            return 0;
        case UTC:
            return 0;
        case GPS:
            return 0;
        case GLO:
            return 0;
        case BDS:
            return 0;
        case GAL:
            return 0;
        case LOC:
        {
            struct tm tm;
            ymd(tm.tm_year, tm.tm_mon, tm.tm_mday, false); // TAI !
            hms(tm.tm_hour, tm.tm_min, tm.tm_sec, false);  // TAI !

            tm.tm_year -= 1900;
            tm.tm_mon -= 1;
            tm.tm_sec -= leapsec() + tzdiff(); // convert tai to LOC !
            tm.tm_isdst = -1;

            mktime(&tm);
            return int(3600.0 * tm.tm_isdst); // [sec]
        }
        default:
            std::cerr << " warning : base_timesys not recognized !\n";
        }
        return 0;
    }

    // get TAI-UTC leap seconds [sec]
    // ----------
    int base_time::leapsec() const
    {
        int leap = 0;
        for (int i = sizeof(leapseconds) / sizeof(*leapseconds) - 1; i >= 0; --i)
        {
            double mjd_time = _mjd + _sod / 86400.0 - leapseconds[i]._leap / 86400.0;
            double mjd_leap = static_cast<double>(_ymd_mjd(leapseconds[i]._year,
                                                           leapseconds[i]._mon,
                                                           leapseconds[i]._day));
            if (mjd_leap <= mjd_time)
            {
                leap = leapseconds[i]._leap;
                break;
            }
        }
        return leap;
    }

    // get leap year
    // jqwu, Sep 2020
    // ----------
    int base_time::leapyear(const int &y) const
    {
        if (y % 4 == 0.0 && y % 100 != 0.0)
            return 1; //     leap year !
        if (y % 400 == 0.0)
            return 1; //     leap year !

        return 0; // not leap year
    }

    // get modified julian date
    // b = true (convert to TSYS)
    // ----------
    int base_time::mjd(const bool &conv) const
    {
        // do not convert
        //if (!conv || _tsys == USER || _tsys == TAI) return _mjd;
        if (!conv)
        {
            return _mjd;
        }
        else
        {
            return _mjd_conv;
        }
        // do convert
        //return _mjd_conv;
        //int mjd = _mjd +  floor( (_sod + tai_tsys(_tsys) + _dsec + tai_tsys_dsec(_tsys)) / 86400.0);// 86400.0  have to be float!
        //return mjd;
    }

    // get double modified jujian date
    // -----------------------------------
    double base_time::dmjd(const bool &conv) const
    {
        double ret;

        // jdhuang
        //ret = (double)mjd(conv) +
        //      (double)sod(conv)/24.0/60.0/60.0 + dsec(conv)/24.0/60.0/60.0;

        ret = dsec(conv) / 86400.0 + (double)sod(conv) / 86400.0 + (double)mjd(conv);
        return ret;
    }

    double base_time::dmjd_vlbi(const bool &conv) const
    {
        int m, y, b = 0;
        bool ind = false;
        if (mon() <= 2)
        {
            m = mon() + 12;
            y = year() - 1;
        }
        else
        {
            m = mon();
            y = year();
        }
        if (y <= 1582 && m <= 10 && day() <= 4)
        {
            b = -2;
            ind = true;
        }
        if (ind == false)
        {
            b = int(y / 400) - int(y / 100);
        }
        double jd = int(365.25 * y) - 2400000.5;
        //double tmjd = jd + int(30.6001 * (m + 1)) + b + 1720996.5 + day() + hour() / 24.0 + mins() / 1440.0 + secs() / 86400.0 + _dsec / 86400.0;
        double tmjd = jd + int(30.6001 * (m + 1)) + b + 1720996.5 + day() + hour() / 24.0 + mins() / 1440.0 + secs() / 86400.0 + dsec() / 86400.0;
        return tmjd;
    }

    // get seconds of day
    // b = true (convert to TSYS)
    // ----------
    int base_time::sod(const bool &conv) const
    {
        // do not convert
        //if (!conv || _tsys == USER || _tsys == TAI) return _sod;
        if (!conv)
        {
            return _sod;
        }
        else
        {
            return _sod_conv;
        }

        // do convert
        //double sod = _sod + tai_tsys(_tsys) + _dsec + tai_tsys_dsec(_tsys);

        //while (sod >= 86400) { sod -= 86400; }
        //while (sod < 0) { sod += 86400; }

        //return sod;
    }

    // get dsec
    // b = true (convert to TSYS)
    // ----------
    double base_time::dsec(const bool &conv) const
    {
        // do not convert
        //if (!conv || _tsys == USER || _tsys == TAI) return _dsec;
        if (!conv)
        {
            return _dsec;
        }
        else
        {
            return _dsec_conv;
        }

        // do convert
        //double dsec = 0.0;
        //dsec = _dsec + tai_tsys_dsec(_tsys);
        //while (dsec >= 1.0) { dsec -= 1.0; }
        //while (dsec < 0.0) { dsec += 1.0; }

        //return dsec;
    }

    // get time_t
    // ----------
    time_t base_time::tim() const
    {
        int y, b, d, h, m, s;
        ymd(y, b, d, true); // convert to UTC ?
        hms(h, m, s, true); // convert to TAI UTC ?

        struct tm t;
        t.tm_year = y - 1900;
        t.tm_mon = b - 1;
        t.tm_mday = d;
        t.tm_hour = h;
        t.tm_min = m;
        t.tm_sec = s;
        t.tm_isdst = -1;

        return mktime(&t);
    }

    // get day of year
    // ----------
    // jdhuang : mark for change
    int base_time::doy(const bool &conv) const
    {
        int y, m, d;
        // ZHJ true->false
        //ymd(y,m,d,true);
        ymd(y, m, d, conv);

        int doy = d;
        for (int i = 0; i < m && i < 13; ++i)
            doy += monthdays[i];
        if (m > 2)
            doy += leapyear(y); // count for leap year
        return doy;
    }

    // get BDS week
    // ----------
    int base_time::bwk() const
    {
        return static_cast<int>((_mjd_conv - 44244.0) / 7.0) - CONV_BWK2GWK; // return true TS
    }

    // get GPS week
    // ----------
    int base_time::gwk() const
    {
        //return static_cast<int>((mjd(true) - 44244.0) / 7.0);  // return true TS
        return static_cast<int>((mjd(true) - 44244.0) / 7.0); //xiongyun true -> false
    }

    // get day of GPS(BDS) week
    // ----------
    int base_time::dow() const
    {
        //return static_cast<int>(mjd(true) - 44244.0 - gwk() * 7);  // no problem gwk for BDS here
        return static_cast<int>(mjd(true) - 44244.0 - gwk() * 7); //xiongyun true -> false    glfeng false->true
    }

    // get seconds of GPS(BDS) week
    // ----------
    int base_time::sow() const
    {
        return static_cast<int>(dow() * 86400.0 + sod()); // no problem gwk for BDS here
    }

    // get year (4-char)
    // ----------
    int base_time::year() const
    {
        int y, m, d;
        //lijie  true -> false
        //ymd(y, m, d, true);
        ymd(y, m, d, false);

        return static_cast<int>(y);
    }

    // get year (2-char) (1980-2079 only)
    // ----------
    int base_time::yr() const
    {
        int y, m, d;
        // ZHJ true -> false
        //ymd( y, m, d, true);
        ymd(y, m, d, false);
        int yr = this->yr(y);
        return yr;
    }

    // get month
    // ----------
    int base_time::mon() const
    {
        int y, m, d;
        ymd(y, m, d, true);

        return static_cast<int>(m);
    }

    // get day
    // ----------
    int base_time::day() const
    {
        int y, m, d;
        ymd(y, m, d, true);

        return static_cast<int>(d);
    }

    // get hour
    // ----------
    int base_time::hour() const
    {
        return static_cast<int>((sod(true) % 86400) / 3600.0);
    }

    // get minutes
    // ----------
    int base_time::mins() const
    {
        return static_cast<int>((sod(true) % 3600) / 60.0);
    }

    // get seconds
    // ----------
    int base_time::secs() const
    {
        return static_cast<int>(sod(true) % 60);
    }

    // get hour
    // ----------
    void base_time::hms(int &h, int &m, int &s, bool conv) const
    {
        h = static_cast<int>((sod(conv) % 86400) / 3600.0);
        m = static_cast<int>((sod(conv) % 3600) / 60.0);
        s = static_cast<int>((sod(conv) % 60));
    }

    // MJD -> YMD
    // ----------
    void base_time::ymd(int &y, int &m, int &d, bool conv) const
    {
        int jj, mm, dd;
        long ih, ih1, ih2;
        double t1, t2, t3, t4;
        double mjd = 0.0;

        mjd = this->mjd(conv) + (this->sod(conv)) / 86400.0;

        //  DO NOT USE THIS DUE TO UNDERFLOW
        //mjd = this->mjd( conv ) + (this->sod( conv ) + this->dsec( conv ))/86400;

        t1 = 1.0 + mjd - fmod(mjd, 1.0) + 2400000.0;
        t4 = fmod(mjd, 1.0);
        ih = int((t1 - 1867216.25) / 36524.25);
        t2 = t1 + 1 + ih - ih / 4;
        t3 = t2 - 1720995.0;
        ih1 = int((t3 - 122.1) / 365.25);
        t1 = ih1 * 365.25 - fmod(ih1 * 365.25, 1.0);
        ih2 = int((t3 - t1) / 30.6001);
        dd = int(t3 - t1 - int(ih2 * 30.6001) + t4);
        mm = ih2 - 1;
        if (ih2 > 13)
            mm = ih2 - 13;
        jj = ih1;
        if (mm <= 2)
            jj = jj + 1;
        y = jj;
        m = mm;
        d = dd;
    }

    // to std::string
    // ----------
    std::string base_time::str(const std::string &ofmt, const bool &conv) const
    {
        base_time gt(*this);

        char cstr[12] = "char";
        std::string str = ofmt; // copy of requested format
        size_t idx = 0;
        int y, b, d, h, m, s;
        gt.ymd(y, b, d, conv);
        gt.hms(h, m, s, conv);
        int y2 = gt.yr(y);

        // search and process all % identificators
        while ((idx = str.find('%')) != std::string::npos && idx + 1 <= str.length())
        {

            bool replace = false;
            for (int i = 0; i < MAX_DT; ++i)
            {
                std::string tmp = TD[i];

                // dat std::string not identified, skipped !
                if (str.substr(idx + 1, tmp.length()).compare(tmp) != 0)
                    continue;

                if (!tmp.compare("Y"))
                {
                    sprintf(cstr, "%04i", y);
                }
                else if (!tmp.compare("y"))
                {
                    sprintf(cstr, "%02i", y2);
                }
                else if (!tmp.compare("b"))
                {
                    sprintf(cstr, "%3.3s", gt.mon(b).c_str());
                }
                else if (!tmp.compare("m"))
                {
                    sprintf(cstr, "%02i", b);
                }
                else if (!tmp.compare("d"))
                {
                    sprintf(cstr, "%02i", d);
                }
                else if (!tmp.compare("j"))
                {
                    sprintf(cstr, "%03i", gt.doy());
                }
                else if (!tmp.compare("H"))
                {
                    sprintf(cstr, "%02i", h);
                }
                else if (!tmp.compare("M"))
                {
                    sprintf(cstr, "%02i", m);
                }
                else if (!tmp.compare("S"))
                {
                    sprintf(cstr, "%02i", s);
                }
                else if (!tmp.compare("W"))
                {
                    sprintf(cstr, "%04i", gt.gwk());
                }
                else if (!tmp.compare("w"))
                {
                    sprintf(cstr, "%01i", gt.dow());
                }
                else if (!tmp.compare("v"))
                {
                    sprintf(cstr, "%6i", gt.sow());
                }
                else if (!tmp.compare("s"))
                {
                    sprintf(cstr, "%5i", gt.sod(conv));
                }
                else if (!tmp.compare("J"))
                {
                    sprintf(cstr, "%5i", gt.mjd(conv));
                }
                else if (!tmp.compare("I"))
                {
                    sprintf(cstr, "%11.5f", gt.mjd(conv) + gt.sod(conv) / 86400.0 + gt.dsec(conv) / 86400.0);
                }
                else if (!tmp.compare("T"))
                {
                    sprintf(cstr, "%3s", sys().c_str());
                    // add for output clk files C L K O P
                }
                else if (!tmp.compare("C"))
                {
                    sprintf(cstr, "%2i", b);
                }
                else if (!tmp.compare("L"))
                {
                    sprintf(cstr, "%2i", d);
                }
                else if (!tmp.compare("K"))
                {
                    sprintf(cstr, "%2i", h);
                }
                else if (!tmp.compare("O"))
                {
                    sprintf(cstr, "%2i", m);
                }
                else if (!tmp.compare("P"))
                {
                    sprintf(cstr, "%9.6f", static_cast<double>(s));
                }
                else if (!tmp.compare("p"))
                {
                    sprintf(cstr, "%10.7f", static_cast<double>(s) + gt.dsec(conv));
                }
                else
                {
                    std::cerr << " warning : base_time - unknown date/time identifier [" << tmp << "]\n";
                }

                str.replace(idx, 2, cstr);
                replace = true;
            }
            // replace unknown % occurance
            if (!replace)
                str.replace(idx, 1, "");
        }
        return str;
    }

    // to std::string
    // ----------
    std::string base_time::str_ymd(const std::string &str, const bool &conv) const
    {
        char cstr[12];
        int y = 0, b = 0, d = 0;
        this->ymd(y, b, d, conv);
        sprintf(cstr, "%04i-%02i-%02i", y, b, d);
        return str + " " + std::string(cstr);
    }

    // to std::string
    // ----------
    std::string base_time::str_hms(const std::string &str, const bool &conv) const
    {
        char cstr[12];
        int h = 0, m = 0, s = 0;
        this->hms(h, m, s, conv);
        sprintf(cstr, "%02i:%02i:%02i", h, m, s);
        return str + " " + std::string(cstr);
    }

    std::string base_time::str_ydoysod(const std::string &str, const bool &conv) const
    {
        char cstr[15];
        sprintf(cstr, "%04i:%03i:%05i", year(), doy(conv), sod());
        return cstr;
    }

    // to std::string
    // ----------
    std::string base_time::str_ymdhms(const std::string &str,
                               const bool &ts,
                               const bool &conv) const
    {
        char cstr[25];
        int y = 0, b = 0, d = 0, h = 0, m = 0, s = 0;
        this->ymd(y, b, d, conv);
        this->hms(h, m, s, conv);
        if (ts)
            sprintf(cstr, "%04i-%02i-%02i %02i:%02i:%02i[%3s]", y, b, d, h, m, s, sys().c_str());
        else
            sprintf(cstr, "%04i-%02i-%02i %02i:%02i:%02i", y, b, d, h, m, s);
        return str + " " + std::string(cstr);
    }

    std::string base_time::str_mjdsod(const std::string &str, const bool &ts, const bool &conv) const
    {
        char cstr[25];

        int mjd = this->mjd(ts);
        double sod = (this->dmjd(ts) - mjd) * 86400.0;
        int isod = 0;
        if (fabs(sod - (int)(sod)) < 0.5e0)
        {
            isod = round(sod);
            sprintf(cstr, "%05i %10.4f", mjd, static_cast<double>(isod));
        }
        else
        {
            sprintf(cstr, "%05i %10.4f", mjd, sod);
        }

        return str + " " + std::string(cstr);
    }

    // to std::string
    // ----------
    std::string base_time::str_yyyydoy(const bool &conv) const
    {
        int year, doy;
        std::string tmp;
        year = this->year();
        doy = this->doy(conv);
        std::string cdoy = base_type_conv::int2str(doy);
        if (doy >= 10 && doy < 100)
            cdoy = "0" + cdoy;
        if (doy < 10)
            cdoy = "00" + cdoy;
        tmp = base_type_conv::int2str(year) + cdoy;
        return tmp;
    }

    std::string base_time::str_yyyy(const bool &conv) const
    {
        return base_type_conv::int2str(this->year());
    };

    std::string base_time::str_doy(const bool &conv) const
    {
        std::string tmp = base_type_conv::int2str(this->doy());
        if (tmp.size() == 1)
            return "00" + tmp;
        else if (tmp.size() == 2)
            return "0" + tmp;
        else
            return tmp;
    };

    std::string base_time::str_gwkd(const bool &conv) const
    {
        return base_type_conv::int2str(this->gwk()) + base_type_conv::int2str(this->dow());
    }

    std::string base_time::str_gwk(const bool &conv) const
    {
        return base_type_conv::int2str(this->gwk());
    }

    std::string base_time::str_yr(const bool &conv) const
    {
        std::string tmp = base_type_conv::int2str(this->yr());
        if (tmp.size() == 1)
            return "0" + tmp;
        else
            return tmp;
    }

    std::string base_time::str_mon(const bool &conv) const
    {
        std::string tmp = base_type_conv::int2str(this->mon());
        if (tmp.size() == 1)
            return "0" + tmp;
        else
            return tmp;
    }
    std::string base_time::str_day(const bool &conv) const
    {
        std::string tmp = base_type_conv::int2str(this->day());
        if (tmp.size() == 1)
            return "0" + tmp;
        else
            return tmp;
    }

    std::string base_time::str_hour(const bool & conv) const
    {
        std::string tmp = base_type_conv::int2str(this->hour());
        if (tmp.size() == 1)
            return "0" + tmp;
        else
            return tmp;
    }

    std::string base_time::str_min(const bool & conv) const
    {
        std::string tmp = base_type_conv::int2str(this->mins());
        if (tmp.size() == 1)
            return "0" + tmp;
        else
            return tmp;
    }



    // get yr (1980-2079 only)
    // ----------
    int base_time::yr(const int &y) const
    {
        if (y <= 2079 && y >= 2000)
            return y - 2000;
        if (y < 2000 && y >= 1980)
            return y - 1900;

        return -1;
    }

    // get month 3-char std::string
    // ----------
    std::string base_time::mon(const int &m) const
    {
        if (m < 1 || m > 12)
            return "XXX";
        return MON[m];
    }

    // get month number
    // ----------
    int base_time::mon(const std::string &str) const
    {
        //for (int i = 0; i < 12; ++i) {
        //    if (str.compare(MON[i]) == 0)
        //        return i + 1;
        //}
        //return 0;

        for (int i = 1; i <= 12; ++i)
        {
            if (str.compare(MON[i]) == 0)
                return i;
        }
        return 0;
    }

    // get system identifier
    // ----------
    std::string base_time::sys() const
    {
        std::string ts(t_tstr[_tsys]);
        return ts;
    }

    // operator reduce [sec] [TAI]
    // ----------
    double base_time::operator-(const base_time &t) const
    {
        return (this->diff(t));
    }

    bool base_time::operator<(const double &t) const
    {
        return (this->sow() + this->dsec() < t);
    }

    // operator minus [TAI]
    // ----------
    base_time base_time::operator-(const double &sec) const
    {
        base_time tmp(*this);
        tmp.add_secs(-static_cast<int>(sec));
        tmp.add_dsec(-static_cast<double>(sec - static_cast<int>(sec))); // already normed
        return tmp;
    }

    // operator add [TAI]
    // ----------
    base_time base_time::operator+(const double &sec) const
    {
        base_time tmp(*this);

        // jdhuang : mark for LEO changes, not me
        int second = floor(sec);
        double dsecond = sec - second;
        tmp.add_secs(second);
        tmp.add_dsec(dsecond);
        //tmp.add_secs( + static_cast<int>(sec));
        //tmp.add_dsec( + static_cast<double>( sec - static_cast<int>(sec) ));  // already normed
        return tmp;
    }

    // operator comp [TAI]
    // ----------
    bool base_time::operator<(const base_time &t) const
    {
        return ((_mjd_conv < t._mjd_conv) || (_mjd_conv == t._mjd_conv && _sod_conv < t._sod_conv) || (_mjd_conv == t._mjd_conv && _sod_conv == t._sod_conv && _dsec_conv < t._dsec_conv));
    }

    // operator comp [TAI]
    // ----------
    bool base_time::operator<=(const base_time &t) const
    {
        return (*this < t || *this == t);
    }

    // operator comp [TAI]
    // ----------
    bool base_time::operator>=(const base_time &t) const
    {
        return (*this > t || *this == t);
    }

    // operator comp [TAI]
    // ----------
    bool base_time::operator>(const base_time &t) const
    {
        return ((_mjd_conv > t._mjd_conv) || (_mjd_conv == t._mjd_conv && _sod_conv > t._sod_conv) || (_mjd_conv == t._mjd_conv && _sod_conv == t._sod_conv && _dsec_conv > t._dsec_conv));
    }

    // operator equiv [TAI]
    // ----------
    bool base_time::operator==(const base_time &t) const
    {
        return (_mjd_conv == t._mjd_conv && _sod_conv == t._sod_conv && _dsec_conv == t._dsec_conv);
    }

    // operator not equiv [TAI]
    // ----------
    bool base_time::operator!=(const base_time &t) const
    {
        return (_mjd_conv != t._mjd_conv || _sod_conv != t._sod_conv || _dsec_conv != t._dsec_conv);
    }

    // operator =
    // ----------
    base_time base_time::operator=(const base_time &t)
    {
        _mjd = t.mjd(false);
        _sod = t.sod(false);
        _dsec = t.dsec(false);
        tsys(t.tsys());

        _reset_conv();
        return *this;
    }

    // delete seconds
    // lvhb addedi in 20210413
    void base_time::del_secs(const int &sec)
    {
        _sod -= sec;
        _norm_sod();

        _reset_conv();
    }

    // add seconds
    // ----------
    void base_time::add_secs(const int &sec)
    {
        _sod += sec;
        _norm_sod();

        _reset_conv();
    }

    // add dseconds
    // ----------
    void base_time::add_dsec(const double &dsec)
    {
        _dsec += dsec;
        _norm_dsec();
        _norm_sod();

        _reset_conv();
    }

    // OBSOLETE, because via operator - exists
    // time difference (this - t) [s]
    // ----------
    double base_time::diff(const base_time &t) const
    {
        bool b = false; // compare in TAI

        //return (    mjd(b)*86400.0 +   sod(b)  +  dsec(b)
        //       - t.mjd(b)*86400.0 - t.sod(b) - t.dsec(b) );
        return ((dsec(b) - t.dsec(b)) + (sod(b) - t.sod(b)) + (mjd(b) * 86400.0 - t.mjd(b) * 86400.0));
    }

    // -----------------------------------------------------------------------------------
    // private functions
    // -----------------------------------------------------------------------------------

    // convert to TAI from requested time
    // ----------
    void base_time::_to_tai()
    {
        _norm_dsec();
        _norm_sod();
        switch (_tsys)
        {
        case USER:
            break;
            ;
        case TAI:
            break;
            ;
        case UTC:
            _sod += leapsec();
            break;
            ;
        case GPS:
            _sod += TAI_GPS;
            break;
            ;
        case GLO:
            _sod -= tzdiff();
            _sod += leapsec();
            break;
            ;
        case GAL:
            _sod += TAI_GAL;
            break;
            ;
        case BDS:
            _sod += TAI_BDS;
            break;
            ;
        case LOC:
            _sod -= tzdiff();
            _sod -= dstime();
            _sod += leapsec();
            break;
            ;
        case TT:
            _sod -= TAI_TT;
            _dsec -= TAT_TT_DSEC;
            break;
            ;
        default:
            break;
            ;
        }
        _norm_dsec();
        _norm_sod();
    }

    void base_time::_reset_conv()
    {
        _mjd_conv = _mjd + floor((_sod + tai_tsys(_tsys) + _dsec + tai_tsys_dsec(_tsys)) / 86400.0); // 86400.0  have to be float!

        double sod_temp = _sod + tai_tsys(_tsys) + _dsec + tai_tsys_dsec(_tsys);
        while (sod_temp >= 86400)
        {
            sod_temp -= 86400;
        }
        while (sod_temp < 0)
        {
            sod_temp += 86400;
        }
        _sod_conv = int(sod_temp);

        _dsec_conv = _dsec + tai_tsys_dsec(_tsys);
        while (_dsec_conv >= 1.0)
        {
            _dsec_conv -= 1.0;
        }
        while (_dsec_conv < 0.0)
        {
            _dsec_conv += 1.0;
        }
    }

    // convert to tsys and return
    // -----------
    base_time base_time::_tai_to()
    {
        _sod += tai_tsys(_tsys);
        _dsec += tai_tsys_dsec(_tsys);
        _norm_dsec();
        _norm_sod();

        return *this;
    }

    // get TAI-tsys difference [sec]
    // ----------
    int base_time::tai_tsys(const base_timesys &ts) const
    {
        double sec = 0.0;
        switch (ts)
        {
        case USER:
            break;
        case TAI:
            break;
        case UTC:
            sec -= leapsec();
            break;
            ;
        case GPS:
            sec -= TAI_GPS;
            break;
            ;
        case GLO:
            sec -= leapsec();
            sec += tzdiff();
            break;
            ;
        case GAL:
            sec -= TAI_GAL;
            break;
            ;
        case BDS:
            sec -= TAI_BDS;
            break;
            ;
        case LOC:
            sec -= leapsec();
            sec += dstime();
            sec += tzdiff();
            break;
            ;
        case TT:
            sec += TAI_TT;
            break;
            ;
        default:
            break;
            ;
        }

        return sec;
    }

    double base_time::tai_tsys_dsec(const base_timesys &ts) const
    {
        double sec = 0.0;
        switch (ts)
        {
        case TT:
            sec += TAT_TT_DSEC;
            break;
            ;
        default:
            break;
            ;
        }
        return sec;
    }

    // normalize dsec
    // ----------
    void base_time::_norm_dsec()
    {
        while (_dsec >= 1.0)
        {
            _dsec -= 1.0;
            _sod += 1;
        }
        while (_dsec < 0.0)
        {
            _dsec += 1.0;
            _sod -= 1;
        }
    }

    // normalize sod
    // ----------
    void base_time::_norm_sod()
    {
        while (_sod >= 86400)
        {
            _sod -= 86400;
            _mjd += 1;
        }
        while (_sod < 0)
        {
            _sod += 86400;
            _mjd -= 1;
        }
    }

    // normalize year
    // ----------
    void base_time::_norm_year(int &year) const
    {
        if (year < 100.0)
            year += year < 80 ? 2000 : 1900;
    }

    // Year,Mon,Day -> MJD
    // ----------
    int base_time::_ymd_mjd(const int &yr, const int &mn, const int &dd) const
    {
        int year(yr);
        int mon(mn);
        _norm_year(year);

        double mjd = 0.0;
        if (mon <= 2)
        {
            mon = mon + 12;
            year = year - 1;
        }

        mjd = 365.25 * year - fmod(365.25 * year, 1.0) - 679006.0;
        mjd += floor(30.6001 * (mon + 1)) + 2.0 - floor(year / 100) + floor(year / 400) + dd;
        return int(mjd);
    }

    // multiplatform ms sleep function
    void base_time::gmsleep(unsigned int ms) // milisecond
    {

#if defined __linux__
        usleep(ms * 1000);
#elif defined __APPLE__
        usleep(ms * 1000);
#elif defined _WIN32 || defined _WIN64
        Sleep(ms);
#else
        clock_t goal = ms + clock();
        while (goal > clock())
            ;
#endif
    }

    // multiplatform us sleep function
    void base_time::gusleep(unsigned int us)
    {
        // this_thread::sleep_for(chrono::microseconds(us));
#if defined __linux__
        usleep(us);
#elif defined __APPLE__
        usleep(us);
#elif defined _WIN32 || defined _WIN64
        std::this_thread::sleep_for(std::chrono::microseconds(us)); // c++11 feature
                                                          //#else
                                                          //   clock_t goal = us + clock();
                                                          //   while (goal > clock());
#endif
    }

    //double base_time::gps2tdt(double inp_time)
    //{
    //    return (inp_time + 51.184 / 86400.0);
    //}

    //base_time base_time::gps2tdt(base_time inp_time)
    //{
    //    base_time out_time = inp_time + 51.184;
    //    out_time.tsys(out_time.str2tsys("TT"));
    //    return out_time;
    //}

    //base_time base_time::utc2gps(base_time utc)
    //{
    //    utc.tsys(utc.str2tsys("UTC"));
    //    utc._to_tai();
    //    utc = utc- 19.0;
    //    return utc;
    //}

    void base_time::yeardoy2monthday(int year, int doy, int *month, int *day)
    {
        int days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        days_in_month[1] = 28;
        int id;

        if ((year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0)))
        {
            days_in_month[1] = 29;
        }

        id = doy;
        for (*month = 1; *month <= 12; (*month)++) //xiongyun fix bug in *montn++
        {
            id = id - days_in_month[*month - 1];
            if (id > 0)
                continue;
            else
            {
                *day = id + days_in_month[*month - 1];
                break;
            }
        }

        return;
    }

    bool operator<(double left, const base_time &t)
    {
        return (left < t.sow() + t.dsec());
    }

    bool operator<(const base_time &t, double left)
    {
        return (t.sow() + t.dsec() < left);
    }

    base_time MAX_TIME(const base_time &A, const base_time &B)
    {
        return A > B ? A : B;
    }

    base_time MAX_TIME(const std::vector<base_time> &A)
    {
        base_time tmax;
        if (A.empty())
            return tmax;
        tmax = A[0];
        for (const auto &iter : A)
        {
            if (tmax < iter)
                tmax = iter;
        }
        return tmax;
    }

    base_time MIN_TIME(const base_time &A, const base_time &B)
    {
        return A > B ? B : A;
    }

    base_time MIN_TIME(const std::vector<base_time> &A)
    {
        base_time tmin;
        if (A.empty())
            return tmin;
        tmin = A[0];
        for (const auto &iter : A)
        {
            if (tmin > iter)
                tmin = iter;
        }
        return tmin;
    }

} // namespace
