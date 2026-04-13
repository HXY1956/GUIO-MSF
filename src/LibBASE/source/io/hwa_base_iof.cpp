#include <iostream>
#include "hwa_base_iof.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_fileconv.h"

namespace hwa_base
{
    std::string base_iof::_replace()
    {
        return _mask;
    }

    // constructor
    // ---------
    base_iof::base_iof(std::string mask)
        : _irc(0),
        _mask(mask),
        _name(""),
        _omode(std::ios::trunc),
        _repl(false),
        _toff(0),
        _loop(false)
    {
        base_type_conv::substitute(mask, HWA_FILE_PREFIX, "");

        this->exceptions(std::fstream::failbit | std::fstream::badbit);
        if (_mask.find('%') != std::string::npos)
            _repl = true;
        _name = _replace();
    }

    // destructor
    // ---------
    base_iof::~base_iof()
    {
        if (this->is_open())
        {
            try
            {
                this->close();
            }
            catch (std::exception e)
            {
                std::cerr << "Exception closing file " << _name << ": " << e.what() << std::endl;
            }
        }
    }

    // std::set mask
    // ---------
    int base_iof::mask(std::string mask)
    {
        hwa_base::base_type_conv::substitute(mask, HWA_FILE_PREFIX, "");
#if defined _WIN32 || defined _WIN32
        hwa_base::base_type_conv::substitute(mask, "/", PATH_SEPARATOR);
#else
        base_type_conv::substitute(mask, "\\", PATH_SEPARATOR);
#endif

        _irc = 0;
        this->clear();
        if (this->is_open())
        {
            try
            {
#ifdef DEBUG
                std::cout << "mask: closing spdlog-file: " << _name << std::endl;
#endif
                this->close();
            }
            catch (std::exception e)
            {
                std::cerr << "Exception closing file " << _name << ": " << e.what() << std::endl;
                _irc++;
                return -1;
            }
        }
        if (mask.find('%') != std::string::npos)
            _repl = true;
        _mask = mask;
        _name = _replace();
        return 1;
    }

    // local log file archive
    // ---------
    int base_iof::write(const std::string& s)
    {

        return this->write(s.c_str(), s.size());
    }

    // local log file archive
    // ---------
    int base_iof::write(const char* buff, int size)
    {

        if (_mask == "" || size < 1)
            return -1;
        std::string name = _replace();

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
                catch (std::exception e)
                {
                    std::cerr << "Exception closing file " << _name << ": " << e.what() << std::endl;
                    _irc++;
                    return -1;
                }
            }

            try
            {
                _name = name;
                make_path(_name);
                this->open(_name.c_str(), std::ios::binary | std::ios::out | _omode);
#ifdef DEBUG
                std::cout << " opening file " << _name << std::endl;
#endif
            }
            catch (std::exception e)
            {
                std::cerr << "Exception opening file " << _name << ": " << e.what() << std::endl;
                _irc++;
                return -1;
            }
        }

        try
        {
            std::fstream::write(buff, size);
            std::fstream::flush();
        }
        catch (std::exception e)
        {
            std::cerr << "Exception writting to file " << _name << ": " << e.what() << std::endl;
            _irc++;
            return -1;
        }
        return size;
    }

    // local log file source
    // ---------
    int base_iof::read(char* buff, int size)
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
                    this->open(_name.c_str(), std::ios::binary | std::ios::in);
                }
                catch (std::exception e)
                {
                    std::cerr << "Exception opening file " << _name << ": " << e.what() << std::endl;
                    _irc++;
                    return -1;
                }
            }

            nbytes = size;
            try
            {
                std::fstream::read(buff, size);
            }
            catch (std::exception e)
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
                    std::cerr << "Exception reading from file " << _name << ": " << e.what() << std::endl;
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
    void base_iof::append(const bool& b)
    {

        if (b)
            _omode = std::ios::app;
        else
            _omode = std::ios::trunc;
    }

    // std::set time system for replacement
    // ---------
  
    void base_iof::tsys(base_time::base_timesys ts)
    {
        _tsys = ts;
        return;
    }

    // get time system for replacement
    // ---------
    base_time::base_timesys base_iof::tsys()
    {
        base_time::base_timesys tsys = _tsys;
        return tsys;
    }
} 
