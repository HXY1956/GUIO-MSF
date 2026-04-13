#ifndef hwa_base_io_serial_h
#define hwa_base_io_serial_h

#ifdef _WIN32
#pragma comment(lib, "WS2_32")
#endif // _WIN32

#define FC_DTRDSR 0x01
#define FC_RTSCTS 0x02
#define FC_XONXOFF 0x04
#define ASCII_BEL 0x07
#define ASCII_BS 0x08
#define ASCII_LF 0x0A
#define ASCII_CR 0x0D
#define ASCII_XON 0x11
#define ASCII_XOFF 0x13

#include <stdio.h>
#include <fstream>
#include <string>
#include <sys/types.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <windows.h>
#else
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#endif // _WIN32
#include "hwa_base_io.h"
#include "hwa_base_log.h"
#include "hwa_base_mutex.h"
#include "hwa_base_coder.h"

#define BUF_SIZE 1024

namespace hwa_base
{
    /**
    *@brief       Class for base_io_serial, derive from base_io
    */
    class base_io_serial : public base_io
    {
    public:
        /** @brief default constructor. */
        base_io_serial();

        /** @brief default constructor. */
        base_io_serial(std::shared_ptr<spdlog::logger> spdlog);

        /** 
        * @brief constructor. 
        * 
        * @param[in]  port              the port
        * @param[in]  baud              the baud
        * 
        */
        base_io_serial(int port, int baud);

        /** 
        * @brief constructor. 
        * 
        * @param[in]  port              the port
        * @param[in]  baud              the baud
        * 
        */
        base_io_serial(int port, int baud, std::shared_ptr<spdlog::logger> spdlog);

        /**
        * @brief constructor.
        * 
        * @param[in]  path              the path
        *
        */
        base_io_serial(std::string path);

        /**
        * @brief constructor.
        * 
        * @param[in]  path              the path
        *
        */
        base_io_serial(std::string path, std::shared_ptr<spdlog::logger> spdlog);

        /** @brief default destructor. */
        ~base_io_serial();

        /** @brief close. */
        bool close(void);

        /** @brief run read. */
        virtual void run_read();

    protected:
        /** @brief init common. */
        virtual int _init_common();

        /** @brief set spdlog. */
        virtual bool glog_set(std::shared_ptr<spdlog::logger> spdlog);

        /** @brief write. */
        virtual int _gio_write(const char *buff, int size) { return 1; }

        /** @brief read.  */
        virtual int _gio_read(char *buff, int size);

    protected:
        int _port; ///< the port
        int _baud; ///< the baud

#ifdef _WIN32
        HANDLE m_hIDComDev;
        OVERLAPPED m_OverlappedRead, m_OverlappedWrite;
#else
        int _fd;
        struct termios _opt;
#endif // _WIN32
    };

} // namespace hwa_gnss

#endif