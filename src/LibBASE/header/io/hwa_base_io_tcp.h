#ifndef hwa_base_io_tcp_h
#define hwa_base_io_tcp_h

#ifdef _WIN32
#pragma comment(lib, "WS2_32")
#endif

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

#include <stdio.h>
#include <fstream>
#include <string>
#include <sys/types.h>
#include "hwa_base_io.h"
#include "hwa_base_log.h"
#include "hwa_base_mutex.h"
#include "hwa_base_coder.h"

#define BUF_SIZE 1024

namespace hwa_base
{
    /**
    *@brief       Class for base_io_tcp, derive from base_io
    */
    class base_io_tcp : public base_io
    {
    public:
        /** @brief default constructor. */
        base_io_tcp();

        /** @brief default constructor. */
        base_io_tcp(std::shared_ptr<spdlog::logger> spdlog);

        /**
        * @brief constructor.
        *
        * @param[in]  port              the port
        *
        */
        base_io_tcp(const int &port);

        /**
        * @brief constructor.
        *
        * @param[in]  port              the port
        *
        */
        base_io_tcp(const int &port, std::shared_ptr<spdlog::logger> spdlog);

        /**
        * @brief constructor.
        *
        * @param[in]  address          the address
        *
        */
        base_io_tcp(const std::string &address);

        /**
        * @brief constructor.
        *
        * @param[in]  address          the address
        *
        */
        base_io_tcp(const std::string &address, std::shared_ptr<spdlog::logger> spdlog);

        /**
        * @brief constructor.
        *
        * @param[in]  address          the address
        * @param[in]  upload          the upload
        *
        */
        base_io_tcp(const std::string &address, bool upload); //add by xiongyun

        /**
        * @brief constructor.
        *
        * @param[in]  address          the address
        * @param[in]  upload          the upload
        *
        */
        base_io_tcp(const std::string &address, std::shared_ptr<spdlog::logger> spdlog, bool upload); //add by xiongyun

        /**
        * @brief constructor.
        * add by glfeng for upload caster
        *
        * @param[in]  host              the host
        * @param[in]  port              the port
        * @param[in]  user              the user
        * @param[in]  password          the password
        * @param[in]  mountpoint      the mountpoint
        * @param[in]  rate              the rate
        *
        */
        base_io_tcp(std::shared_ptr<spdlog::logger> spdlog, const std::string &host, const int &port, const std::string &user,
               const std::string &password, const std::string &mountpoint, int rate = 1);

        /**
        * @brief constructor.
        * add by glfeng for upload caster
        *
        * @param[in]  host              the host
        * @param[in]  port              the port
        * @param[in]  user              the user
        * @param[in]  password          the password
        * @param[in]  mountpoint      the mountpoint
        * @param[in]  rate              the rate
        *
        */
        base_io_tcp(const std::string &host, const int &port, const std::string &user,
               const std::string &password, const std::string &mountpoint, int rate = 1);

        /** @brief default destructor. */
        ~base_io_tcp();

        /** @brief read. */
        virtual void run_read();

        /** @brief write. */
        virtual void run_write();

        /** 
        * @brief send. 
        * add by xiongyun/zhshen
        * 
        * @param[in]  Message          the Message
        * @return      void
        */
        void run_send(const std::string &Message = "");

        /** 
        * add by glfeng
        * @brief upload. 
        * 
        * @param[in]  Message          the Message
        * @param[in]  size              the size
        * @return      void
        */
        void run_upload(const char *Message, int size);

    protected:
        /**
        * @brief gio write.
        * 
        * @param[in]  buff              the buffer
        * @param[in]  size              the size
        * @return      int              write mode
        */
        virtual int _gio_write(const char *buff, int size);

        /**
        * @brief gio write.
        *
        * @param[in]  buff              the buffer
        * @param[in]  size              the size
        * @return      int              read mode 
        */
        virtual int _gio_read(char *buff, int size);

        /**
        * @brief gio write.
        *
        * @param[in]  host              the local host
        * @return      int              
        */
        virtual int _localhost(std::string &host);

        virtual int _init_common();

        std::string user();
        std::string pwd();
        std::string host();
        std::string port();

        virtual bool glog_set(std::shared_ptr<spdlog::logger> spdlog);

    protected:
        virtual bool _Server_Init();
        virtual bool _NtripUpload_Init(); // add by glfeng
        virtual bool _NtripDownload_Init();
        virtual bool _close_socket();

    protected:
        std::string _user;
        std::string _pwd;
        std::string _host;
        std::string _port;
        std::string _mountpoint; // add by glfeng

        ///sockaddr_in saddr;
        sockaddr_in _caddr;

        int bandwidth;                    // the bandwidth of trans
        bool _using_ntrip_2_http = false; //add by xiongyun
        bool _upload = false;             //add by xiongyun
        int _udp;

// fix bug
#if defined(_WIN32) || defined(_WIN64)
        SOCKET _fd_skt;
        SOCKET _fd_acpt;
#else
        int _fd_skt;
        int _fd_acpt;
#endif
    };

} // namespace

#endif
