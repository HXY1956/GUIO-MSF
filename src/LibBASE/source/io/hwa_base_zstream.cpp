#include <iostream>
#include <string.h> 
#include "hwa_base_zstream.h"

namespace hwa_base
{
    hwa_base_zstream_buf *hwa_base_zstream_buf::open(const char *name, int open_mode)
    {
        if (is_open())
            return (hwa_base_zstream_buf *)0;
        mode = open_mode;
        // no append nor read/write mode
        if ((mode & std::ios::ate) || (mode & std::ios::app) || ((mode & std::ios::in) && (mode & std::ios::out)))
            return (hwa_base_zstream_buf *)0;
        char fmode[10];
        char *fmodeptr = fmode;
        if (mode & std::ios::in)
            *fmodeptr++ = 'r';
        else if (mode & std::ios::out)
            *fmodeptr++ = 'w';
        *fmodeptr++ = 'b';
        *fmodeptr = '\0';
        file = gzopen(name, fmode);
        if (file == 0)
            return (hwa_base_zstream_buf *)0;
        opened = 1;
        return this;
    }

    hwa_base_zstream_buf *hwa_base_zstream_buf::close()
    {
        if (is_open())
        {
            sync();
            opened = 0;
            if (gzclose(file) == Z_OK)
                return this;
        }
        return (hwa_base_zstream_buf *)0;
    }

    int hwa_base_zstream_buf::underflow()
    { // used for input buffer only
        if (gptr() && (gptr() < egptr()))
            return *reinterpret_cast<unsigned char *>(gptr());

        if (!(mode & std::ios::in) || !opened)
            return EOF;
        // Josuttis' implementation of inbuf
        int n_putback = gptr() - eback();
        if (n_putback > 4)
            n_putback = 4;
        memcpy(buffer + (4 - (int64_t)n_putback), gptr() - n_putback, n_putback);

        int num = gzread(file, buffer + 4, bufferSize - 4);
        if (num <= 0) // ERROR or EOF
            return EOF;

        // reset buffer pointers
        setg(buffer + (4 - (int64_t)n_putback), // beginning of putback area
             buffer + 4,                        // read position
             buffer + 4 + num);                 // end of buffer

        // return next character
        return *reinterpret_cast<unsigned char *>(gptr());
    }

    int hwa_base_zstream_buf::flush_buffer()
    {
        // Separate the writing of the buffer from overflow() and
        // sync() operation.
        int w = pptr() - pbase();
        if (gzwrite(file, pbase(), w) != w)
            return EOF;
        pbump(-w);
        return w;
    }

    int hwa_base_zstream_buf::overflow(int c)
    { // used for output buffer only
        if (!(mode & std::ios::out) || !opened)
            return EOF;
        if (c != EOF)
        {
            *pptr() = c;
            pbump(1);
        }
        if (flush_buffer() == EOF)
            return EOF;
        return c;
    }

    int hwa_base_zstream_buf::sync()
    {
        // Changed to use flush_buffer() instead of overflow( EOF)
        // which caused improper behavior with std::endl and flush(),
        // bug reported by Vincent Ricard.
        if (pptr() && pptr() > pbase())
        {
            if (flush_buffer() == EOF)
                return -1;
        }
        return 0;
    }

    // --------------------------------------
    // class hwa_base_zstream_base:
    // --------------------------------------

    hwa_base_zstream_base::hwa_base_zstream_base(const char *name, int mode)
    {
        init(&buf);
        open(name, mode);
    }

    hwa_base_zstream_base::~hwa_base_zstream_base()
    {
        buf.close();
    }

    void hwa_base_zstream_base::open(const char *name, int open_mode)
    {
        if (!buf.open(name, open_mode))
            clear(rdstate() | std::ios::badbit);
    }

    void hwa_base_zstream_base::close()
    {
        if (buf.is_open())
            if (!buf.close())
                clear(rdstate() | std::ios::badbit);
    }
}
