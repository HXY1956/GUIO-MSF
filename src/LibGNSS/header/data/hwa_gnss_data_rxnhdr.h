
/**
* @file        grxnhdr.h
* @brief    Purpose: rinexn header
*.
*/

#ifndef hwa_gnss_data_rxnhdr
#define hwa_gnss_data_rxnhdr

#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief system correct. */
    enum TSYS_CORR
    {
        TS_NONE,
        TS_GAUT,
        TS_GPUT,
        TS_SBUT,
        TS_GLUT,
        TS_GPGA,
        TS_GLGP,
        TS_QZGP,
        TS_QZUT,
        TS_BDUT,
        TS_IRUT,
        TS_IRGP
    };

    /** @brief iono correct. */
    enum IONO_CORR
    {
        IO_NONE,
        IO_GAL,
        IO_GPSA,
        IO_GPSB,
        IO_QZSA,
        IO_QZSB,
        IO_BDSA,
        IO_BDSB,
        IO_IRNA,
        IO_IRNB
    };

    /** @brief struct system correct. */
    struct TSYS_CORR_BATCH
    {
        double a0 = 0.0, a1 = 0.0;
        int T = 0, W = 0;
    };

    /** @brief struct iono correct. */
    struct gnss_data_iono_corr
    {
        double x0 = 0.0, x1 = 0.0, x2 = 0.0, x3 = 0.0;
        int T = 0, sat = 0;
    };

    /** @brief std::map system correct. */
    typedef std::map<TSYS_CORR, TSYS_CORR_BATCH> hwa_map_TSYS;

    /** @brief std::map iono correct. */
    typedef std::map<IONO_CORR, gnss_data_iono_corr> hwa_map_iono_CORR;

    /** @brief convert sys_corr to std::string. */
    std::string tsys_corr2str(TSYS_CORR c);

    /** @brief convert iono_corr to std::string. */
    std::string iono_corr2str(IONO_CORR c);

    /** @brief convert std::string to sys_corr. */
    TSYS_CORR str2tsys_corr(std::string s);

    /** @brief convert std::string to iono_corr. */
    IONO_CORR str2iono_corr(std::string s);

    /** @brief class for rinex header information. */
    class gnss_data_rxnhdr
    {
    public:
        /** @brief default constructor. */
        gnss_data_rxnhdr();

        /** @brief default destructor. */
        ~gnss_data_rxnhdr();

        /**
         * @brief 
         * 
         * @return const std::string& 
         */
        const std::string &path() const { return _path; } ///< get path

        /**
         * @brief 
         * 
         * @param s 
         */
        void path(const std::string &s) { _path = s; } ///< set path

        /**
         * @brief 
         * 
         * @return const char& 
         */
        const char &rxnsys() const { return _rxnsys; } ///< get rinex system

        /**
         * @brief 
         * 
         * @param s 
         */
        void rxnsys(const char &s) { _rxnsys = s; } ///< set rinex system

        /**
         * @brief 
         * 
         * @return const std::string& 
         */
        const std::string &rxnver() const { return _rxnver; } ///< get rinex receiver

        /**
         * @brief 
         * 
         * @param s 
         */
        void rxnver(const std::string &s) { _rxnver = s; } ///< set rinex receiver

        /**
         * @brief 
         * 
         * @return const std::string& 
         */
        const std::string &program() const { return _program; } ///< get program

        /**
         * @brief 
         * 
         * @param pgm 
         */
        void program(const std::string &pgm) { _program = pgm; } ///< set program

        /**
         * @brief 
         * 
         * @return const std::string& 
         */
        const std::string &runby() const { return _runby; } ///< get runby

        /**
         * @brief 
         * 
         * @param rnb 
         */
        void runby(const std::string &rnb) { _runby = rnb; } ///< set program

        /**
         * @brief 
         * 
         * @return const base_time& 
         */
        const base_time &gtime() const { return _gtime; } ///< get gtime

        /**
         * @brief 
         * 
         * @param t 
         */
        void gtime(const base_time &t) { _gtime = t; } ///< set gtime

        /**
         * @brief 
         * 
         * @return const std::vector<std::string>& 
         */
        const std::vector<std::string> &comment() const { return _comment; } ///< get comment

        /**
         * @brief 
         * 
         * @param cmt 
         */
        void comment(const std::vector<std::string> &cmt) { _comment = cmt; } ///< set comment

        /**
         * @brief 
         * 
         * @return const int& 
         */
        const int &leapsec() const { return _leapsec; } ///< get leap second

        /**
         * @brief 
         * 
         * @param ls 
         */
        void leapsec(const int &ls) { _leapsec = ls; } ///< set leap second

        /**
         * @brief 
         * 
         * @return set<TSYS_CORR> 
         */
        std::set<TSYS_CORR> tsys_corr() const; ///< system correct

        /**
         * @brief 
         * 
         * @return set<IONO_CORR> 
         */
        std::set<IONO_CORR> iono_corr() const; ///< iono correct

        /**
         * @brief 
         * 
         * @param c 
         * @return TSYS_CORR 
         */
        TSYS_CORR_BATCH tsys_corr(const TSYS_CORR &c) const; ///< get sys_corr

        /**
         * @brief 
         * 
         * @param c 
         * @param ts 
         */
        void tsys_corr(const TSYS_CORR &c, const TSYS_CORR_BATCH&ts); ///< set sys_corr

        /**
         * @brief 
         * 
         * @param c 
         * @return gnss_data_iono_corr 
         */
        gnss_data_iono_corr iono_corr(const IONO_CORR &c) const; ///< get iono_corr

        /**
         * @brief 
         * 
         * @param c 
         * @param io 
         */
        void iono_corr(const IONO_CORR &c, const gnss_data_iono_corr &io); ///< set iono_corr

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
        friend std::ostream &operator<<(std::ostream &os, const gnss_data_rxnhdr &x);

    private:
        char _rxnsys;            ///< G=GPS, R=GLO, E=GAL, S=SBAS, M=Mix
        std::string _path;            ///< rinex file path
        std::string _rxnver;          ///< rinex version
        std::string _program;         ///< name of program creating RINEX file
        std::string _runby;           ///< name of agency  creating RINEX file
        base_time _gtime;          ///< name of date and file of RINEX creation
        std::vector<std::string> _comment; ///< std::vector of comments
        int _leapsec;            ///< leapseconds since 6-Jan-1980
        hwa_map_TSYS _ts_corr;     ///< RINEX header tsys corrections
        hwa_map_iono_CORR _io_corr;     ///< RINEX header iono corrections
    };

} // namespace

#endif
