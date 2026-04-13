#ifndef hwa_base_zstream_h
#define hwa_base_zstream_h
#include <iostream>
#include <fstream>
#include <zlib.h>

namespace hwa_base
{
    /** @brief Internal classes to implement hwa_base_zstream derive from streambuf*/
    class hwa_base_zstream_buf : public std::streambuf
    {
    private:
        static const int bufferSize = 47 + 256; ///< size of data buff
        /// totals 512 bytes under g++ for hwa_base_izstream at the end.

        gzFile file;             ///< file handle for compressed file
        char buffer[bufferSize]; ///< data buffer
        int opened;              ///< open/close state of stream
        int mode;                ///< I/O mode

        int flush_buffer();

    public:
        /** @brief constructor. */
        hwa_base_zstream_buf() : opened(0)
        {
            setp(buffer, buffer + (bufferSize - 1));
            setg(buffer + 4,  // beginning of putback area
                 buffer + 4,  // read position
                 buffer + 4); // end position
                              // ASSERT: both input & output capabilities will not be used together
        }

        /** @brief get is opened?. */
        int is_open() { return opened; }

        /**
        * @brief open file.
        *
        * @param[in]  name            file name
        * @param[in]  open_mode        open mode
        * @return      hwa_base_zstream_buf
        */
        hwa_base_zstream_buf *open(const char *name, int open_mode);

        /** @brief close file. */
        hwa_base_zstream_buf *close();

        /** @brief destructor + close file. */
        ~hwa_base_zstream_buf() { close(); }

        /** @brief used for output buffer only. */
        virtual int overflow(int c = EOF);

        /** @brief used for input buffer only. */
        virtual int underflow();

        /** 
        *@brief Changed to use flush_buffer() instead of overflow( EOF)
        * which caused improper behavior with std::endl and flush(),
        * bug reported by Vincent Ricard.
        */
        virtual int sync();
    };

    /** @brief Internal classes for hwa_base_zstream_base derive from ios*/
    class hwa_base_zstream_base : virtual public std::ios
    {
    protected:
        hwa_base_zstream_buf buf; ///< buffer
    public:
        /** @brief constructor + init buffer. */
        hwa_base_zstream_base() { init(&buf); }

        /** 
        *@brief constructor 2. 
        *
        * @param[in]  name            file name
        * @param[in]  open_mode        open mode
        * @return     hwa_base_zstream_base
        */
        hwa_base_zstream_base(const char *name, int open_mode);

        /** @brief default destructor. */
        ~hwa_base_zstream_base();

        /** @brief open. */
        void open(const char *name, int open_mode);

        /** @brief close. */
        void close();

        /** @brief get buffer. */
        hwa_base_zstream_buf *rdbuf() { return &buf; }

        /** @brief get is_opened. */
        int is_open() { return buf.is_open(); }
    };

    /** 
    *@brief User classes. Use hwa_base_izstream and hwa_base_ozstream analogously to ifstream and
    * ofstream respectively. They read and write files based on the gz* 
    * function interface of the zlib. Files are compatible with gzip compression.
    */
    class hwa_base_izstream : public hwa_base_zstream_base, public std::istream
    {
    public:
        /** @brief constructor + init buffer. */
        hwa_base_izstream() : std::istream(&buf) {}

        /** @brief constructor + name + open_mode. */
        hwa_base_izstream(const char *name, int open_mode = std::ios::in)
            : hwa_base_zstream_base(name, open_mode), std::istream(&buf) {}

        /** @brief get rdbuffer. */
        hwa_base_zstream_buf *rdbuf() { return hwa_base_zstream_base::rdbuf(); }

        /** @brief open file. */
        void open(const char *name, int open_mode = std::ios::in)
        {
            hwa_base_zstream_base::open(name, open_mode);
        }
    };

    /** @brief Internal classes for hwa_base_ozstream derive from hwa_base_zstream_base and ostream.*/
    class hwa_base_ozstream : public hwa_base_zstream_base, public std::ostream
    {
    public:
        /** @brief constructor + init buffer. */
        hwa_base_ozstream() : std::ostream(&buf) {}

        /** @brief constructor + name + mode. */
        hwa_base_ozstream(const char *name, int mode = std::ios::out)
            : hwa_base_zstream_base(name, mode), std::ostream(&buf) {}
        hwa_base_zstream_buf *rdbuf() { return hwa_base_zstream_base::rdbuf(); }

        /** @brief open file. */
        void open(const char *name, int open_mode = std::ios::out)
        {
            hwa_base_zstream_base::open(name, open_mode);
        }
    };

    /** @brief JD 2017-08-08. */
    class hwa_base_zstream : public hwa_base_zstream_base, public std::iostream
    {
    public:
        /** @brief constructor + init buffer. */
        hwa_base_zstream() : std::iostream(&buf) {}

        /** @brief constructor + name + mode. */
        hwa_base_zstream(const char *name, int mode)
            : hwa_base_zstream_base(name, mode), std::iostream(&buf) {}
        hwa_base_zstream_buf *rdbuf() { return hwa_base_zstream_base::rdbuf(); }

        /** @brief open file. */
        void open(const char *name, int open_mode)
        {
            hwa_base_zstream_base::open(name, open_mode);
        }
    };
}

#endif
