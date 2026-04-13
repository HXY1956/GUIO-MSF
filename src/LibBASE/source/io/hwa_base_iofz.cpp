#include <iostream>
#include "hwa_base_common.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_iofz.h"

using namespace std;

namespace hwa_base
{
    base_iofz::base_iofz(string mask)
        : _irc(0),
          _mask(mask),
          _name(""),
          _omode(ios::trunc),
          _repl(false),
          _toff(0),
          _loop(false),
          _tsys(base_time::UTC)
    {
        base_type_conv::substitute(mask, HWA_FILE_PREFIX, "");

        this->exceptions(iostream::failbit | iostream::badbit);
        if (_mask.find('%') != string::npos)
            _repl = true;
        _name = _replace();
    }

    // destructor
    // ---------
    base_iofz::~base_iofz()
    {
        if (this->is_open())
        {
            try
            {
                this->close();
            }
            catch (exception e)
            {
                cerr << "Exception closing file " << _name << ": " << e.what() << endl;
            }
        }
    }

    // set mask
    // ---------
    int base_iofz::mask(string mask)
    {
        _irc = 0;
        base_type_conv::substitute(mask, HWA_FILE_PREFIX, "");

        this->clear();
        if (this->is_open())
        {
            try
            {
#ifdef DEBUG
                std::cout << "mask: closing spdlog-file: " << _name << endl;
#endif
                this->close();
            }
            catch (exception e)
            {
                cerr << "Exception closing file " << _name << ": " << e.what() << endl;
                _irc++;
                return -1;
            }
        }
        if (mask.find('%') != string::npos)
            _repl = true;
        _mask = mask;
        _name = _replace();
        return 1;
    }

    // local spdlog file archive
    // ---------
    int base_iofz::write(const string &s)
    {

        return this->write(s.c_str(), s.size());
    }

    // local spdlog file archive
    // ---------
    int base_iofz::write(const char *buff, int size)
    {

        if (_mask == "" || size < 1)
            return -1;
        string name = _replace();

        this->clear();
        // new name || not opened
        if (!this->is_open() || name != _name)
        {

            if (this->is_open())
            {
                try
                {
                    this->close();
#ifdef DEBUG
                    std::cout << "closing spdlog-file: " << _name << " [" << name << "]\n";
#endif
                }
                catch (exception e)
                {
                    cerr << "Exception closing file " << _name << ": " << e.what() << endl;
                    _irc++;
                    return -1;
                }
            }

            try
            {
                _name = name;
                make_path(_name);
                this->open(_name.c_str(), ios::binary | ios::out | _omode);
#ifdef DEBUG
                std::cout << " opening file " << _name << endl;
#endif
            }
            catch (exception e)
            {
                cerr << "Exception opening file " << _name << ": " << e.what() << endl;
                _irc++;
                return -1;
            }
        }

        try
        {
            hwa_base_zstream::write(buff, size);
            hwa_base_zstream::flush();
        }
        catch (exception e)
        {
            cerr << "Exception writting to file " << _name << ": " << e.what() << endl;
            _irc++;
            return -1;
        }
        return size;
    }

    // local spdlog file source
    // ---------
    int base_iofz::read(char *buff, int size)
    {

        if (_mask == "")
            return -1;
        _name = _replace();

        int nbytes;
        /* we might need to read twice because the last read read exactly until eof,
   * thus nbytes/gcount() would be 0 this time , which we don't want */
        for (int i = 0; i < 2; i++)
        {

            this->clear();
            if (!this->is_open())
            {
                try
                {
                    make_path(_name);
                    this->open(_name.c_str(), ios::binary | ios::in);
                }
                catch (exception e)
                {
                    cerr << "Exception opening file " << _name << ": " << e.what() << endl;
                    _irc++;
                    return -1;
                }
            }

            nbytes = size;
            try
            {
                hwa_base_zstream::read(buff, size);
            }
            catch (exception e)
            {
                if (this->eof())
                {
                    nbytes = (int)this->gcount();

                    if (_loop)
                        this->close();

                    if (nbytes < 1)
                    {
                        // there was no data left before eof, let's try opening the file again
                        continue;
                    }
                }
                else
                {
                    cerr << "Exception reading from file " << _name << ": " << e.what() << endl;
                    _irc++;
                    return -1;
                }
            }
            break; // nbytes is more than 0, we can break
        }
#ifdef DEBUG
        std::cout << "giof_read  [" << nbytes << "] " << buff << "\n name " << _name << "\n";
        std::cout.flush();
#endif
        return nbytes;
    }

    // append mode
    // ---------
    void base_iofz::append(const bool &b)
    {

        if (b)
            _omode = ios::app;
        else
            _omode = ios::trunc;
    }

    // set time system for replacement
    // ---------
    void base_iofz::tsys(base_time::base_timesys ts)
    {
        _tsys = ts;
        return;
    }

    // get time system for replacement
    // ---------
    base_time::base_timesys base_iofz::tsys()
    {
        base_time::base_timesys tsys = _tsys;
        return tsys;
    }

    // evaluate name
    // ---------
    string base_iofz::_replace()
    {
        if (!_repl)
            return _mask;

        base_time file_tm = base_time::current_time(_tsys);
        file_tm.add_secs(_toff * 60.0);
        return file_tm.str(_mask);
    }

} // namespace
