#ifndef hwa_base_iof_h
#define hwa_base_iof_h

#include <fstream>
#include <string>
#include "hwa_base_mutex.h"
#include "hwa_base_time.h"

namespace hwa_base
{
    /** @brief class for std::fstream. */
    class base_iof : public std::fstream
    {

    public:
        /** @brief constructor 1. */
        explicit base_iof(std::string mask = "");

        /** @brief default destructor. */
        virtual ~base_iof();

        /** @brief std::set/get file mask. */
        int mask(std::string mask);
        std::string mask() const { return _mask; }

        /** @brief get last filename. */
        std::string name() const { return _name; }

        /** @brief get irc status. */
        int irc() const { return _irc; };

        /** @brief std::set/get loop read. */
        void loop(bool l) { _loop = l; }
        bool loop() const { return _loop; }

        /** @brief std::set/get time offset [min] for the file name. */
        void toff(int i) { _toff = i; }
        int toff() const { return _toff; }

        /** @brief writting. */
        virtual int write(const char* buff, int size);
        int write(const std::string& s);

        /** @brief reading. */
        int read(char* buff, int size);

        /** @brief append mode [false/true]. */
        void append(const bool& b = true);

        void tsys(base_time::base_timesys);
        base_time::base_timesys tsys();

       /** @brief get md5sum. */
        std::string md5sum();

    protected:
        std::string _replace(); ///< replace mask to name

        int _irc;              ///  irc status OK=0, Warning>0, Error<0
        std::string _mask;          ///< original name mask
        std::string _name;          ///< actual (evaluated) name
        std::ios::openmode _omode;  ///< output open mode
        bool _repl;            ///< replace if time-specific
        int _toff;             ///< if replace, time offset [min] for the file name
        bool _loop;            ///< loop read
        base_time::base_timesys _tsys; ///< time system for replacement
        base_mutex _gmutex;

#ifdef BMUTEX
        boost::mutex _mutex; ///< mutual exlusion
#endif
    private:
    };

} // namespace

#endif
