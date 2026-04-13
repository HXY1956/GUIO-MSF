#include "hwa_base_iobigf.h"
#include <assert.h>

using namespace std;

namespace hwa_base
{
    base_io_bigf::base_io_bigf(string mask, int buffer_size)
        : base_iof(mask),
          buffer_size(buffer_size),
          _current(0),
          _buffer(nullptr)
    {
        _buffer = new char[buffer_size];
    }

    base_io_bigf::~base_io_bigf()
    {
        if (_buffer)
        {
            delete[] _buffer;
            _buffer = nullptr;
        }
    }

    int base_io_bigf::write(const char *buff, int size)
    {
        if (_current + size > buffer_size)
        {
            //this->flush();
            if (this->flush() < 0)
            {
                return -1;
            }
            return (base_iof::write(buff, size));
        }

        //if (_current + size > BUF_SIZE) {
        //    this->flush();
        //}

        memcpy(&this->_buffer[_current], buff, size);
        _current += size;
        return size;
    }

    int base_io_bigf::flush()
    {
        if (_current == 0)
        {
            return 0;
        }

        int state = base_iof::write(_buffer, _current);
        if (state < 0)
        {
            return state;
        }
        _current = 0;
        return state;
        //return (base_iof::write(_buffer, _current));
    }

    base_io_READTEMP::base_io_READTEMP(string tempfilename) : _tmpfile(fopen(tempfilename.c_str(), "rb")),
                                                    _buffer(nullptr),
                                                    _current(0),
                                                    _endpos(0)
    {
        // get tempfile size
        //fseek(_tmpfile, 0, SEEK_END);
        //_filesize = _ftelli64(_tmpfile);
        // filesize must use 64bit filesize maybe bigger than 2GB
        if (!_get_filesize(_tmpfile, &_filesize))
        {
            fclose(_tmpfile);
            _tmpfile = nullptr;
            throw runtime_error("Get tempfile filesize is wrong!");
        }

        // alloc buffer
        // buffse size must less than MAX_BUFFER_SIZE,so using 32 bit is ok
        int32_t buffersize = (_filesize > MAX_BUFFER_SIZE) ? MAX_BUFFER_SIZE : _filesize;
        _buffer = new char[buffersize];
        _endpos = buffersize;

        // from end of file
        fseek(_tmpfile, -buffersize, SEEK_END);
        if(buffersize != fread(_buffer, 1, buffersize, _tmpfile))
        {
            throw std::logic_error("fread failed");
        }
        fseek(_tmpfile, -buffersize, SEEK_CUR);
        _filesize -= buffersize;
        _current = _endpos;
    }
    base_io_READTEMP::~base_io_READTEMP()
    {
        if (_buffer)
        {
            delete[] _buffer;
            _buffer = nullptr;
        }

        if (_tmpfile)
        {
            fclose(_tmpfile);
            _tmpfile = nullptr;
        }
    }
    void base_io_READTEMP::read(char *dst, int size)
    {
        assert(size > 0);
        if (_current + size > _endpos)
        {
            throw runtime_error("Read tempfile Wrong!!!");
        }

        //for (int i = 0; i < size; i++) {
        //    *(dst + i) = _buffer[_current++];
        //}
        memcpy(dst, _buffer + _current, size);
        _current += size;
    }
    void base_io_READTEMP::seekg_from_cur(int lensize)
    {
        assert(lensize < 0);
        if (_current + lensize < 0)
        {
            if (-(_current + lensize) > _filesize)
            {
                throw runtime_error("Read tempfile Wrong!!!");
            }
            long long move_size = _filesize < (_endpos - _current) ? _filesize : _endpos - _current;
            _endpos = move_size + _current;
            // move 0-current to last
            for (int i = _current - 1; i >= 0; i--)
            {
                _buffer[_endpos - _current + i] = _buffer[i];
            }
            // read movesize from file
            fseek(_tmpfile, -move_size, SEEK_CUR);
            if(move_size != fread(_buffer, 1, move_size, _tmpfile))
            {
                throw std::logic_error("fread failed");
            }
            fseek(_tmpfile, -move_size, SEEK_CUR);
            _filesize -= move_size;
            _current = _endpos;
        }
        _current += lensize;
    }

#if defined(__linux__) || defined(__unix__)
#define _FILE_OFFset_BITS 64
#endif

    bool base_io_READTEMP::_get_filesize(FILE *file, int64_t *file_byte_size)
    {

#if defined(_WIN32) || defined(_WIN64)
#if _MSC_VER >= 1400
        /***********************/
        if (_fseeki64(file, (long long)(0), SEEK_END))
        {
            return false;
        }
        *file_byte_size = _ftelli64(file);
#else
#error Visual Studio version is less than 8.0(VS 2005) !
#endif
        /***********************/
#else
        if (fseeko(file, (long long)(0), SEEK_END))
        {
            return false;
        }
        *file_byte_size = ftello(file);
        /***********************/
#endif
        return true;
    }
}