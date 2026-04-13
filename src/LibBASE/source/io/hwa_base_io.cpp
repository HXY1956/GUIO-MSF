#include <string>
#include <iostream>
#include "hwa_base_io.h"
#include "hwa_base_common.h"
#include "hwa_base_typeconv.h"

namespace hwa_base
{

    // constructor
    // ---------
    base_io::base_io()
        : _fd(-1),
          _size(BUF_SIZE),
          _path(""),
          _giof(),
          _count(0),
          _verb(0),
          _stop(0),
          _opened(0),
          _running(0)
    {
        _coder = 0;
    }
    base_io::base_io(base_log spdlog)
        : _fd(-1),
          _size(BUF_SIZE),
          _path(""),
          _giof(),
          _count(0),
          _verb(0),
          _stop(0),
          _opened(0),
          _running(0)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }

        _coder = 0;
        //  char* loc_buff = new char[size()];
    }

    // destructor
    // ---------
    base_io::~base_io()
    {
    }

    // std::set path
    // ---------
    int base_io::path(std::string str)
    {

        if (str == "")
            return -1;

        _path = str;
        return 1;
    }

    // overloading << operator
    // ---------
    std::ostream &operator<<(std::ostream &os, const base_io &n)
    {
        os << n.path();
        return os;
    }

    // overloading == operator
    // ---------
    bool base_io::operator==(const base_io &n) const
    {
        return (n.path() == _path);
    }

    void base_io::spdlog(base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }
    // overloading < operator
    // ---------
    bool base_io::operator<(const base_io &n) const
    {
        return (n.path() < _path);
    }

    // start reading (could be run in a separate thread)
    // ---------
    void base_io::run_read()
    {
        // be sure always clean decoder
        if (_coder)
        {
            _coder->clear();
        }
        // For error message
        std::vector<std::string> errmsg;

        char *loc_buff = new char[_size];

        if (init_read() < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, " warning - initialization failed");
            base_time::gmsleep(3000); // [ms]
                                    //  sleep(3);               // [s] linux only
        }
        int nbytes = 0;
        _stop = 0;
        _running = 1;

        while (((nbytes = _gio_read(loc_buff, _size)) > 0) && _stop != 1)
        {
            // archive the stream
            _locf_write(loc_buff, nbytes);
            // volatile int decoded = 0;
            if (_coder && nbytes > 0)
            {
                _coder->decode_data(loc_buff, nbytes, _count, errmsg);
                // if (_spdlog)
                //     SPDLOG_LOGGER_DEBUG(_spdlog, "READ : " + base_type_conv::int2str(nbytes) + " decoded: " + base_type_conv::int2str(decoded));
                
                if (_coder->end_epoch > hwa_base::base_time(0, 0))
                {
                    if (_coder->epoch > _coder->end_epoch)
                    {
                        break;
                    }
                }
            }
            else
            {
                // decoded = 0;
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "0 data decoded");
            }

        }

        _stop_common();
        delete[] loc_buff;
        //std::cerr << "run_read() ending: " << _path << std::endl;
        return;
    }

    // start reading (could be run in a separate thread)
    // ---------
    void base_io::run_write()
    {
        if (_coder)
        {
            _coder->clear();
        } // be sure always clean decoder

        std::vector<std::string> errmsg;
        _stop = 0;

        char *loc_buff = new char[_size];

        if (_opened != 1 && init_write() < 0)
        {
            _stop_common();
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, " warning - initialization failed");
            delete[] loc_buff;
            return;
        }

        int nbytes = 0;
        _running = 1;
        do
        {
            nbytes = 0;

            // 1. try to read from local file source
            nbytes = _locf_read(loc_buff, _size);

            // 2. try reading in a loop
            //    while( nbytes < 0 ){
            //       nbytes = _locf_read(loc_buff,_size);
            //     }

            // 3. read from encoder
            if (nbytes < 0 && _coder)
            {
                nbytes = _coder->encode_data(loc_buff, _size, _count, errmsg);
            }

            // if( nbytes > 0 )   std::cout << " nbytes = encoded : " << nbytes << " \n";
            // base_time::gmsleep(100);
            // for(int i = 0; i<nbytes ; i++){ std::cout << loc_buff[i]; }
        } while (_stop == 0 && (_gio_write(loc_buff, nbytes) > 0 || nbytes < -1));

        _stop_common();
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "end of read [stop or gio_write problem]");
        delete[] loc_buff;
        return;
    }

    // local log file source
    // ---------
    int base_io::_locf_read(char *buff, int size)
    {

        if (_giof.mask() == "")
            return -1;

        return _giof.read(buff, size);
    }

    // local log file archive
    // ---------
    int base_io::_locf_write(const char *buff, int size)
    {

        if (_giof.mask() == "")
            return -1;

        int irc = _giof.write(buff, size);
        //  if( fprintf(_locf_ptr, buff, size) == 0 ) return 0;

        //  for(int i = 0; i<size ; i++){ _locf_stream << buff[i]; }

        //  if( irc ) return -1;
        //  return size;
        return irc;
    }

    // common function for initialization
    // ---------
    int base_io::_init_common()
    {

        return 1;
    }

    // common function for socket/file close
    // ---------
    int base_io::_stop_common()
    {

        _opened = 0; // for TCP/NTRIP now, need to be checked for gfile!
        _running = 0;
        return _running;
    }

} // namespace