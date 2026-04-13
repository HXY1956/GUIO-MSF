#ifndef hwa_gnss_data_rnxhdr
#define hwa_gnss_data_rnxhdr

/**
*
* @file        grnxhdr.h
* @brief    Purpose: implements receiver class
*.
*/

#include <string>
#include <vector>

#include "hwa_base_time.h"
#include "hwa_gnss_data_obsmanager.h"

namespace hwa_gnss
{

    class gnss_data_rnxhdr
    {
    public:
        /** @brief default constructor. */
        gnss_data_rnxhdr();

        /** @brief default destructor. */
        ~gnss_data_rnxhdr();

        typedef std::vector<std::pair<GOBS, double>> hwa_vector_obs;
        typedef std::map<std::string, hwa_vector_obs> hwa_map_obs;

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string path() const { return _path; } ///< get path

        /**
         * @brief 
         * 
         * @param s 
         */
        void path(const std::string &s) { _path = s; } ///< set path

        /**
         * @brief 
         * 
         * @return char 
         */
        char rnxsys() { return _rnxsys; } ///< get rinex system

        /**
         * @brief 
         * 
         * @param s 
         */
        void rnxsys(const char &s) { _rnxsys = s; } ///< set rinex system

        /**
         * @brief 
         * 
         * @return const std::string& 
         */
        const std::string &rnxver() const { return _rnxver; } ///< get rinex version

        /**
         * @brief 
         * 
         * @param s 
         */
        void rnxver(const std::string &s) { _rnxver = s; } ///< set rinex version

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string program() const { return _program; } ///< get program

        /**
         * @brief 
         * 
         * @param pgm 
         */
        void program(const std::string &pgm) { _program = pgm; } ///< set program

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string runby() const { return _runby; } ///< get runby

        /**
         * @brief 
         * 
         * @param rnb 
         */
        void runby(const std::string &rnb) { _runby = rnb; } ///< set runby

        /**
         * @brief 
         * 
         * @return base_time 
         */
        base_time gtime() const { return _gtime; } ///< get gtime

        /**
         * @brief 
         * 
         * @param t 
         */
        void gtime(const base_time &t) { _gtime = t; } ///< set gtime

        /**
         * @brief 
         * 
         * @return std::vector<std::string> 
         */
        std::vector<std::string> comment() { return _comment; } ///< get comment

        /**
         * @brief 
         * 
         * @param cmt 
         */
        void comment(const std::vector<std::string> &cmt) { _comment = cmt; } ///< set comment

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string markname() const { return _markname; } ///< get markname

        /**
         * @brief 
         * 
         * @param mrk 
         */
        void markname(const std::string &mrk) { _markname = mrk; } ///< set markname

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string marknumb() const { return _marknumb; } ///< set mark number

        /**
         * @brief 
         * 
         * @param mnb 
         */
        void marknumb(const std::string &mnb) { _marknumb = mnb; } ///< get mark number

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string marktype() const { return _marktype; } ///< get mark type

        /**
         * @brief 
         * 
         * @param mtp 
         */
        void marktype(const std::string &mtp) { _marktype = mtp; } ///< set mark type

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string observer() const { return _observer; } ///< get observer

        /**
         * @brief 
         * 
         * @param obsrv 
         */
        void observer(const std::string &obsrv) { _observer = obsrv; } ///< set observer

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string agency() const { return _agency; } ///< get agency

        /**
         * @brief 
         * 
         * @param agn 
         */
        void agency(const std::string &agn) { _agency = agn; } ///< set agency

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string recnumb() const { return _recnumb; } ///< get recnumb

        /**
         * @brief 
         * 
         * @param rnb 
         */
        void recnumb(const std::string &rnb) { _recnumb = rnb; } ///< set recnumb

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string rectype() const { return _rectype; } ///< get rectype

        /**
         * @brief 
         * 
         * @param rtp 
         */
        void rectype(const std::string &rtp) { _rectype = rtp; } ///< set rectype

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string recvers() const { return _recvers; } ///< set recvers

        /**
         * @brief 
         * 
         * @param rvs 
         */
        void recvers(const std::string &rvs) { _recvers = rvs; } ///< get recvers

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string antnumb() const { return _antnumb; } ///< get antnumb

        /**
         * @brief 
         * 
         * @param ant 
         */
        void antnumb(const std::string &ant) { _antnumb = ant; } ///< set antnumb

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string anttype() const { return _anttype; } ///< get anttype

        /**
         * @brief 
         * 
         * @param ant 
         */
        void anttype(const std::string &ant) { _anttype = ant; } ///< set anttype

        /**
         * @brief 
         * 
         * @return Triple 
         */
        Triple aprxyz() const { return _aprxyz; } ///< get apr position

        /**
         * @brief 
         * 
         * @param apr 
         */
        void aprxyz(const Triple &apr) { _aprxyz = apr; } ///< set apr position

        /**
         * @brief 
         * 
         * @return Triple 
         */
        Triple antxyz() const { return _antxyz; } ///< get ant position

        /**
         * @brief 
         * 
         * @param ecc 
         */
        void antxyz(const Triple &ecc) { _antxyz = ecc; } ///< set ant position

        /**
         * @brief 
         * 
         * @return Triple 
         */
        Triple antneu() const { return _antneu; }         ///< get ant neu
        void antneu(const Triple &ecc) { _antneu = ecc; } ///< set ant neu

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string strength() const { return _strength; } ///< get strength

        /**
         * @brief 
         * 
         * @param s 
         */
        void strength(const std::string &s) { _strength = s; } ///< set strength

        /**
         * @brief 
         * 
         * @return double 
         */
        double interval() const { return _interval; } ///< get interval

        /**
         * @brief 
         * 
         * @param i 
         */
        void interval(const double &i) { _interval = i; } ///< set interval

        /**
         * @brief 
         * 
         * @return base_time 
         */
        base_time first() const { return _first; } ///< get first epoch

        /**
         * @brief 
         * 
         * @param frst 
         */
        void first(const base_time &frst) { _first = frst; } ///< set first epoch

        /**
         * @brief 
         * 
         * @return base_time 
         */
        base_time last() const { return _last; } ///< get last epoch

        /**
         * @brief 
         * 
         * @param lst 
         */
        void last(const base_time &lst) { _last = lst; } ///< set last epoch

        /**
         * @brief 
         * 
         * @return int 
         */
        int leapsec() const { return _leapsec; } ///< get leap second

        /**
         * @brief 
         * 
         * @param ls 
         */
        void leapsec(int ls) { _leapsec = ls; } ///< set leap second

        /**
         * @brief 
         * 
         * @return int 
         */
        int numsats() const { return _numsats; } ///< get number of satellite

        /**
         * @brief 
         * 
         * @param nums 
         */
        void numsats(const int &nums) { _numsats = nums; } ///< set number of satellite

        /**
         * @brief 
         * 
         * @return hwa_map_obs 
         */
        hwa_map_obs mapobs() const { return _mapobs; } ///< get std::mapobs

        /**
         * @brief 
         * 
         * @param types 
         */
        void mapobs(const hwa_map_obs &types) { _mapobs = types; } ///< set std::mapobs

        /**
         * @brief 
         * 
         * @return hwa_map_obs 
         */
        hwa_map_obs mapcyc() const { return _mapcyc; } ///< get std::mapcyc

        /**
         * @brief 
         * 
         * @param types 
         */
        void mapcyc(const hwa_map_obs &types) { _mapcyc = types; } ///< set std::mapcyc

        /**
         * @brief 
         * 
         * @return hwa_map_obs 
         */
        hwa_map_obs glofrq() const { return _glofrq; } ///< get glonass frequency

        /**
         * @brief 
         * 
         * @param types 
         */
        void glofrq(const hwa_map_obs &types) { _glofrq = types; } ///<set glonass frequency

        /**
         * @brief 
         * 
         * @return hwa_vector_obs 
         */
        hwa_vector_obs globia() const { return _globia; } ///< get glonass bias

        /**
         * @brief 
         * 
         * @param types 
         */
        void globia(const hwa_vector_obs &types) { _globia = types; } ///< set glonass bias

        /**
         * @brief 
         * 
         */
        void clear();

        /**
         * @brief 
         * 
         * @param os 
         * @param x 
         * @return std::ostream& 
         */
        friend std::ostream &operator<<(std::ostream &os, const gnss_data_rnxhdr &x); ///< override operator <<

    private:
        char _rnxsys;            ///< G=GPS, R=GLO, E=GAL, S=SBAS, M=Mix
        std::string _path;            ///< rinex file path
        std::string _rnxver;          ///< rinex version
        std::string _program;         ///< name of program creating RINEX file
        std::string _runby;           ///< name of agency  creating RINEX file
        base_time _gtime;          ///< name of date and file of RINEX creation
        std::vector<std::string> _comment; ///< std::vector of comments
        std::string _markname;        ///< marker name
        std::string _marknumb;        ///< marker number
        std::string _marktype;        ///< marker type
        std::string _observer;        ///< name of observer
        std::string _agency;          ///< name of agency
        std::string _recnumb;         ///< receiver number
        std::string _rectype;         ///< receiver type
        std::string _recvers;         ///< receiver version
        std::string _antnumb;         ///< antenna number
        std::string _anttype;         ///< antenna type
        Triple _aprxyz;       ///< approximate xyz position [m]
        Triple _antxyz;       ///< antenna xyx eccentricities [m]
        Triple _antneu;       ///< antenna north/east/up eccentricities [m]
        std::string _strength;        ///< signal strength [DBHZ/...]
        double _interval;        ///< interval [sec]
        base_time _first;          ///< time of first observation
        base_time _last;           ///< time of last observation
        int _leapsec;            ///< leapseconds since 6-Jan-1980
        int _numsats;            ///< number of satellites
        hwa_map_obs _mapobs;      ///< std::map of GOBS and scaling factors
        hwa_map_obs _mapcyc;      ///< std::map of GOBS phase base_quater-cycle shifts
        hwa_map_obs _glofrq;      ///< std::map of GLONASS slot/frequency
        hwa_vector_obs _globia;     ///< vec of GLONASS obs code-phase biases
    };

} // namespace

#endif
