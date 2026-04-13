#ifndef hwa_base_iofz_h
#define hwa_base_iofz_h

#include "hwa_base_time.h"
#include "hwa_base_mutex.h"
#include "hwa_base_zstream.h"

namespace hwa_base
{
    /** @brief class for base_iofz. */
    class base_iofz : public hwa_base_zstream
    {

    public:
        /** @brief constructor 1. */
        explicit base_iofz(std::string mask = "");

        /** @brief default destructor. */
        virtual ~base_iofz();

        /** @brief set/get file mask. */
        int mask(std::string mask);
        std::string mask() const { return _mask; }

        /** @brief get last filename. */
        std::string name() const { return _name; }

        /** @brief get irc status. */
        int irc() const { return _irc; };

        /** @brief set/get loop read. */
        void loop(bool l) { _loop = l; }
        bool loop() const { return _loop; }

        /** @brief set/get time offset [min] for the file name. */
        void toff(int i) { _toff = i; }
        int toff() const { return _toff; }

        /** @brief writting. */
        int write(const char *buff, int size);
        int write(const std::string &s);

        /** @brief reading. */
        int read(char *buff, int size);

        /** @brief append mode [false/true]. */
        void append(const bool &b = true);

        /** @brief set/get time system for replacement. */
        void tsys(base_time::base_timesys);
        base_time::base_timesys tsys();

    protected:
        std::string _replace(); ///< replace mask to name

        int _irc;              ///  irc status OK=0, Warning>0, Error<0
        std::string _mask;          ///< original name mask
        std::string _name;          ///< actual (evaluated) name
        std::ios::openmode _omode;  ///< output open mode
        bool _repl;            ///< replace if time-specific
        int _toff;             ///< if replace, time-offset [min] for the file name
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
