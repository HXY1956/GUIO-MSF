#ifndef hwa_base_time_h
#define hwa_base_time_h

#include "hwa_base_mutex.h"
#include <time.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdio.h>
#include "math.h"
#include <iostream>
#include <fstream>
#include <iomanip>

namespace hwa_base
{
#if defined GTIME_TAI
#define DEFAULT_TIME TAI // default gtime TAI
#elif defined GTIME_UTC
#define DEFAULT_TIME UTC // default gtime UTC
#elif defined GTIME_USER
#define DEFAULT_TIME USER // default gtime USER
#elif defined GTIME_LOC
#define DEFAULT_TIME LOC // default gtime LOCAL
#elif defined GTIME_GPS
#define DEFAULT_TIME GPS // default gtime GPS
#else
#define DEFAULT_TIME GPS // default gtime GPS
#endif

#define MAX_DT 21 + 1 /// number of time std::string identifiers /16 -> 21/
#define MAXLEAP 50    ///< maximum number of leap second in table
#define TAI_GPS 19    ///< seconds of which GPS is ahead of TAI at 6.1.1980
#define TAI_BDS 33    ///< seconds of which BDS is ahead of TAI at 1.1.2006
#define TAI_GAL 19    ///< coincide with GPS time
#define TAI_TT 32     ///< TAI TT
#define TAT_TT_DSEC 0.184
#define CONV_BWK2GWK 1356 ///< convert BDS to GPS week

    /** @brief month std::string. */
    const static std::string MON[13] = { "   ", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                   "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

    /** @brief days in a month. */
    static const int monthdays[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

    /** @brief leap second struct. */
    struct leapsec_table
    {
        int _year; ///< year
        int _mon;  ///< month
        int _day;  ///< day
        int _leap; ///< leap second
    };

    /** @brief leap second structs. */
    const static struct leapsec_table leapseconds[] = {
        {1971, 12, 31, 11},
        {1972, 12, 31, 12},
        {1973, 12, 31, 13},
        {1974, 12, 31, 14},
        {1975, 12, 31, 15},
        {1976, 12, 31, 16},
        {1977, 12, 31, 17},
        {1978, 12, 31, 18},
        {1979, 12, 31, 19},
        {1981, 06, 30, 20},
        {1982, 06, 30, 21},
        {1983, 06, 30, 22},
        {1985, 06, 30, 23},
        {1987, 12, 31, 24},
        {1989, 12, 31, 25},
        {1990, 12, 31, 26},
        {1992, 06, 30, 27},
        {1993, 06, 30, 28},
        {1994, 06, 30, 29},
        {1995, 12, 31, 30},
        {1997, 06, 30, 31},
        {1998, 12, 31, 32},
        {2005, 12, 31, 33},
        {2008, 12, 31, 34},
        {2011, 06, 30, 35},
        {2015, 06, 30, 36},
        {2017, 01, 01, 37} };

    /** @brief supported io keys of std::fixed size ! */
    const static std::string TD[MAX_DT] = {
        "Y", /// [4] year            (1900..2099)
        "y", /// [2] 2-dig year      (80..99,00..79)
        "m", /// [2] month           (1..12+)
        "b", /// [3] month           (Jan..Dec)
        "d", /// [2] day of month    (1..31+)
        "j", /// [3] day of year     (0..365+)
        "H", /// [2] hours           (0..23+)
        "M", /// [2] minutes         (0..59+)
        "S", /// [2] seconds         (0..59+)
        "W", /// [4] GPS week        (0000..XXXX)
        "w", /// [1] day of week     (0..6)
        "v", /// [6] seconds of week (0..604800+)
        "s", /// [5] seconds of day  (0..86400+)
        "J", /// [5]  modified julian date = integer only ! (e.g. 55725)
        "I", /// [11] modified julian date                  (e.g. 55725.81712)
        "T", /// [3] time-system std::string
        "C", /// [2] month for lsqclk, when single , no zero  ( 1, 2,...,10,11,12)
        "L", /// [2] day for lsqclk, when single , no zero  ( 1, 2,...,10,...,29,..)
        "K", /// [2] hour for lsqclk, when single , no zero  ( 1, 2,...,10,...,24)
        "O", /// [2] minutes for lsqclk, when single , no zero  ( 1, 2,...,10,...,60)
        "P", /// [9] seconds for lsqclk, when single , no zero for ten digit.( 1.000000, ...,59.000000)
        "p", /// [12] seconds  double (30.0000000 )
    };

    /** @brief conversion to GAL + GLO are not implemented */
    const static char t_tstr[8][4] = { "USR", "TAI", "UTC", "LOC", "GPS", "GAL", "GLO", "BDS" };
}

namespace hwa_base
{
    /** @brief class for base_time */
    class base_time
    {
    public:
        /** @brief time types */
        enum base_timesys
        {
            USER, ///< user time
            TAI,  ///< TAI
            UTC,  ///< UTC
            LOC,  ///< local time
            GPS,  ///< GPST
            GAL,  ///< Galileo time
            GLO,  ///< Glonass time
            BDS,  ///< BDS time
            TT    ///< TT
        };

        /** @brief time types to std::string */
        static std::string tsys2str(const base_timesys &ts);

        /** @brief std::string to base_timesys*/
        static base_timesys str2tsys(const std::string &s);

        /** @brief initiate with current time !*/
        explicit base_time(const base_timesys &ts = DEFAULT_TIME);
        //base_time(const base_time& other); // jdhuang : copy constructor is called

        /**
        * @brief constructor 1.
        * @param[in]  head        ambflag file head data.
        * @return      
        */
        explicit base_time(const time_t &tt,
                         const double &dsec = 0,
                         const base_timesys &ts = DEFAULT_TIME);

        /**
        * @brief constructor 2.
        * @param[in]  head        ambflag file head data.
        * @return
        */
        base_time(const int &yr, const int &mn,
                const int &dd, const int &hr,
                const int &mi, const int &sc,
                const double &ds = 0.0, const base_timesys &ts = DEFAULT_TIME);

        /**
        * @brief constructor 3.
        * @param[in]  head        ambflag file head data.
        * @return
        */
        base_time(const int &gw, const int &dw,
                const int &sd, const double &ds = 0.0,
                const base_timesys &ts = DEFAULT_TIME);

        /**
        * @brief constructor 4.
        * @param[in]  head        ambflag file head data.
        * @return
        */
        base_time(const int &gw, const double &sow,
                const base_timesys &ts = DEFAULT_TIME);

        /**
        * @brief constructor 5.
        * @param[in]  head        ambflag file head data.
        * @return
        */
        base_time(const int &mjd, const int &sd,
                const double &ds = 0.0, const base_timesys &ts = DEFAULT_TIME);

        /** @brief default destructor. */
        ~base_time();

        // returning only static functions
        /** @brief get current time. */
        static base_time current_time(const base_timesys &ts = DEFAULT_TIME);

        // std::set functions
        /** 
        * @brief std::set from time_t,dsec. 
        * @param[in]    conv    true: different sys; false: the same sys.
        */
        int from_time(const time_t &tt,
                      const double &ds = 0.0, const bool &conv = true);

        /** @brief std::set from GPSwk,sow,dsec. */
        int from_gws(const int &gw,
                     const double &sow, const bool &conv = true);

        /** @brief std::set from GPSwk+dw,sod,dsec. */
        int from_gwd(const int &gw, const int &dw, const int &sd,
                     const double &ds = 0.0, const bool &conv = true);

        /** @brief std::set from yr+mn+dd,sod,dsec. */
        int from_ymd(const int &yr, const int &mn, const int &dd,
                     const int &sd, const double &ds = 0.0,
                     const bool &conv = true);

        /** @brief std::set from yr+mn+dd + H+M+S. */
        int from_ymdhms(const int &yr, const int &mn, const int &dd,
                        const int &h, const int &m, const double &s,
                        const bool &conv = true);

        /** @brief std::set from mjd,sod,dsec. */
        int from_mjd(const int &mjd, const int &sd,
                     const double &ds = 0.0, const bool &conv = true);

        /** @brief std::set from mjd. */
        int from_dmjd(const double &dmjd, const bool &conv = true);

        /** @brief std::set from defined std::string. */
        int from_str(const std::string &ifmt, const std::string &dat,
                     const bool &conv = true);

        int from_secs(const double Timestamp) {
            from_gws(gwk(), Timestamp);
            return 1;
        }

        /** @brief reset dsec only (in TAI)!. */
        int reset_dsec();

        /** @brief reset sod + dsec only (in TAI)!. */
        int reset_sod();

        /** @brief std::set new io time-system. */
        int tsys(const base_timesys &ts)
        {
            _tsys = ts;
            _reset_conv();
            return 0;
        }

        // get functions (default includes conversion to _tsys, if conv=false return TAI)
        int mjd(const bool &conv = true) const;          ///< get MJD
        double dmjd(const bool &conv = true) const;      ///< get MJD
        double dmjd_vlbi(const bool &conv = true) const; ///< get MJD for VLBI
        int sod(const bool &conv = true) const;          ///< get seconds of day
        double dsec(const bool &conv = true) const;      ///< get base_type_conv::fractional sec

        // get functions (always includes conversion to _tsys, for TAI use only mjd, sod and dsec)
        time_t tim() const;                     ///< get time_t
        int gwk() const;                        ///< get GPS week
        int bwk() const;                        ///< get BDS week
        int dow() const;                        ///< get integer day of GPS week
        int sow() const;                        ///< get integer Sec of Week
        int year() const;                       ///< get integer 4-char year
        int yr() const;                         ///< get integer 2-char year (1980-2079 only)
        int doy(const bool &conv = true) const; ///< get integer day of year
        int day() const;                        ///< get integer day of month
        int mon() const;                        ///< get integer day
        int hour() const;                       ///< get integer hour
        int mins() const;                       ///< get integer minute
        int secs() const;                       ///< get integer seconds
        std::string sys() const;                     ///< get time system identifier

        void hms(int &h, int &m, int &s, bool conv = true) const; ///< get hour, minute, seconds
        void ymd(int &y, int &m, int &d, bool conv = true) const; ///< get year, month, day
        std::string str(const std::string &ofmt = "%Y-%m-%d %H:%M:%S",
                   const bool &conv = true) const; ///< get any std::string
        std::string str_ymd(const std::string &str = "",
                       const bool &conv = true) const; ///< get std::fixed date std::string
        std::string str_hms(const std::string &str = "",
                       const bool &conv = true) const; ///< get std::fixed time std::string
        std::string str_ydoysod(const std::string &str = "",
                           const bool &conv = true) const; ///< get year-doy-sod
        std::string str_ymdhms(const std::string &str = "",
                          const bool &ts = true,
                          const bool &conv = true) const; ///< get std::fixed date and time std::string
        std::string str_mjdsod(const std::string &str = "",
                          const bool &ts = true,
                          const bool &conv = true) const; ///< get mjd-sod

        std::string str_yyyydoy(const bool &conv = true) const; ///< get yyyydoy add by glfeng
        std::string str_yyyy(const bool &conv = true) const;    ///< get year-std::string
        std::string str_doy(const bool &conv = true) const;     ///< get doy-std::string
        std::string str_gwkd(const bool &conv = true) const;    ///< get gpsweek and day -std::string
        std::string str_gwk(const bool &conv = true) const;     ///< get gpsweek -std::string
        std::string str_yr(const bool &conv = true) const;      ///< get 2-char year
        std::string str_mon(const bool &conv = true) const;     ///< get month -std::string
        std::string str_day(const bool &conv = true) const;     ///< get day -std::string
        std::string str_hour(const bool &conv = true) const;     ///< get hour -std::string
        std::string str_min(const bool &conv = true) const;     ///< get min -std::string
        // general conversion functions
        int yr(const int &y) const;       ///< get integer 2-char year (1980-2079 only)
        std::string mon(const int &m) const;   ///< get month std::string
        int mon(const std::string &str) const; ///< get month number

        int tzdiff() const;                           ///< get UTC-LOC difference [sec]
        int dstime() const;                           ///< get day-saving time [sec]
        int leapsec() const;                          ///< get TAI-UTC leap seconds [sec]
        int leapyear(const int &y) const;             ///< get YEAR with leapsec [0,1]
        int tai_tsys(const base_timesys &ts) const;         ///< get TAI-tsys difference [sec]
        double tai_tsys_dsec(const base_timesys &ts) const; ///< get TAI-tsys difference [dsec]
        base_timesys tsys() const { return _tsys; }         ///< get required inp/out time-system

        void del_secs(const int &sec);       ///< date/time with deleted X-seconds,lvhb added in 20210413
        void add_secs(const int &sec);       ///< date/time with added X-seconds
        void add_dsec(const double &dsec);   ///< date/time with added X-dseconds
        double diff(const base_time &t) const; ///< time difference (this - t) [s]

        /** @brief override operator. */
        bool operator<(const base_time &t) const;
        bool operator<=(const base_time &t) const;
        bool operator>=(const base_time &t) const;
        bool operator>(const base_time &t) const;
        bool operator==(const base_time &t) const;
        bool operator!=(const base_time &t) const;
        double operator-(const base_time &t) const; // [sec]
        bool operator<(const double &t) const;
        base_time operator-(const double &sec) const; // gtime
        base_time operator+(const double &sec) const; // gtime
        base_time operator=(const base_time &t);
        friend bool operator<(double left, const base_time &t);
        friend bool operator<(const base_time &t, double left);

        /** @brief multiplatform msleep function [ms]. */
        static void gmsleep(unsigned int ms);

        /** @brief multiplatform usleep function [us]. */
        static void gusleep(unsigned int us);

        /** @brief convert yeardoy to month_day. */
        void yeardoy2monthday(int year, int doy, int *month, int *day);

    protected:
    private:
        int _mjd_conv;      ///< mjd conv
        int _sod_conv;      ///< sod conv
        double _dsec_conv;  ///< dsec conv
        void _reset_conv(); ///< reset conv

        int _mjd;          ///< integer MJD [TAI]
        int _sod;          ///< seconds of day [TAI]
        double _dsec;      ///< base_type_conv::fractional seconds
        base_timesys _tsys;      ///< time system
        base_time _tai_to(); ///< conversion from TAI to TS with changing gtime !
        void _to_tai();    ///< conversion from TS to TAI with changing gtime !

        void _norm_dsec();                ///< normalize dsec (range in 0-1 sec)
        void _norm_sod();                 ///< normalize sod  (range in 0-86399 sec)
        void _norm_year(int &year) const; ///< normalize year (range 1980 - 2079)
        int _ymd_mjd(const int &yr,
                     const int &mn,
                     const int &dd) const;

        /** @brief ymd type. */
        enum t_tymd
        {
            _ymd = 0,
            _year = 1,
            _mon = 2,
            _day = 4
        }; // internal counter

        /** @brief hms type. */
        enum t_thms
        {
            _hms = 0,
            _hour = 1,
            _min = 2,
            _sec = 4
        }; // internal counter
    };

    const static base_time FIRST_TIME(44239, 0, 0.0, base_time::TAI);    ///< first valid time  "1980-01-01 00:00:00"
    const static base_time LAST_TIME(80763, 86399, 0.0, base_time::TAI); ///< last  valid time  "2079-12-31 23:59:59"

    /** @brief max/min time. */
    base_time MAX_TIME(const base_time &A, const base_time &B);
    base_time MAX_TIME(const std::vector<base_time> &A);
    base_time MIN_TIME(const base_time &A, const base_time &B);
    base_time MIN_TIME(const std::vector<base_time> &A);

} // namespace

#endif
