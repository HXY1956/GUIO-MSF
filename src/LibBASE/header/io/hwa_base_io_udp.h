#ifndef hwa_base_io_udp_h
#define hwa_base_io_udp_h

#ifdef _WIN32
#pragma comment(lib, "WS2_32")
#endif // _WIN32

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
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <arpa/inet.h>
#endif // _WIN32
#include "hwa_base_io.h"
#include "hwa_base_log.h"
#include "hwa_base_mutex.h"
#include "hwa_base_coder.h"

#define BUF_SIZE 1024

namespace hwa_base
{
    /**
    *@brief       Class for base_io_udp, derive from base_io
    */
    class base_io_udp : public base_io
    {
    public:
        /** @brief default constructor. */
        base_io_udp();

        /** @brief default constructor. */
        base_io_udp(std::shared_ptr<spdlog::logger> spdlog);

        /** 
        * @brief constructor. 
        * @param[in]  address      the address
        */
        base_io_udp(const std::string &address);

        /** 
        * @brief constructor. 
        * @param[in]  address      the address
        */
        base_io_udp(const std::string &address, std::shared_ptr<spdlog::logger> spdlog);

        /** @brief default destructor. */
        ~base_io_udp();

        /** @brief send. */
        void run_send(const std::string &Message);

    protected:
        /**
        * @brief write.
        * 
        * @param[in]  buff            the buffer
        * @param[in]  size            the size
        * @return      int            write mode
        */
        virtual int _gio_write(const char *buff, int size);

        /**
        * @brief read.
        *
        * @param[in]  buff            the buffer
        * @param[in]  size            the size
        * @return      int            read mode
        */
        virtual int _gio_read(char *buff, int size);

        /**
        * @brief read.
        *
        * @param[in]  host            the local host
        * @return      int            the mode
        */
        virtual int _localhost(std::string &host);

        /** @brief get the user. */
        std::string user();

        /** @brief get the password. */
        std::string pwd();

        /** @brief get the host. */
        std::string host();

        /** @brief get the port. */
        std::string port();

        /** @brief set the spdlog. */
        virtual bool glog_set(std::shared_ptr<spdlog::logger> spdlog);

    protected:
        /** @brief init the server. */
        virtual bool _Server_Init();

        /** @brief close the socket. */
        virtual bool _close_socket();

    protected:
        std::string _user; ///< the user
        std::string _pwd;  ///< the password
        std::string _host; ///< the host
        std::string _port; ///< the port
        bool _using_ntrip_2_http = false;
#if defined(_WIN32) || defined(_WIN64)
        SOCKET _udp; ///< the udp
#else
        int _udp = 0;
#endif
    };

} // namespace

#endif
