#include <cstring>
#include <iostream>
#include "hwa_base_io.h"
#include "hwa_base_io_udp.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_time.h"
#include "hwa_base_log.h"

using namespace std;

namespace hwa_base
{
    base_io_udp::base_io_udp() : base_io()
    {
        _user = "";
        _pwd = "";
        _host = "";
        _port = "";
        _spdlog = nullptr;
    }

    base_io_udp::base_io_udp(std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        _user = "";
        _pwd = "";
        _host = "";
        _port = "";
        _spdlog = nullptr;
    }

    base_io_udp::base_io_udp(const string &address) : base_io()
    {
        string addr = address;
        for (int i = 0; i < address.size(); i++)
        {
            if (address[i] == ':' || address[i] == '/')
                addr[i] = ' ';
        }
        stringstream ss(addr);
        string type;
        _user = "";
        _pwd = "";
        ss >> type >> _host >> _port;
        if (_host == "localhost" && _localhost(_host) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gudp", " error lcoal host !");
        }
    }

    base_io_udp::base_io_udp(const string &address, std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        string addr = address;
        for (int i = 0; i < address.size(); i++)
        {
            if (address[i] == ':' || address[i] == '/')
                addr[i] = ' ';
        }
        stringstream ss(addr);
        string type;
        _user = "";
        _pwd = "";
        ss >> type >> _host >> _port;
        if (_host == "localhost" && _localhost(_host) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gudp", " error lcoal host !");
        }
    }

    base_io_udp::~base_io_udp()
    {
    }

    bool base_io_udp::_Server_Init()
    {
        _close_socket();

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "Opening UDP socket");
        if ((_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "error opening UDP socket");
            return false;
        }

        return true;
    }

    int base_io_udp::_gio_read(char *buff, int size)
    {
        return 1;
    }

    void base_io_udp::run_send(const string &Message)
    {
        int nbytes = 0;
        if (!_opened) //zhshen
        {
            if (_Server_Init() <= 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "base_io_udp", " error - connect failed");
                _opened = 0;
                base_time::gmsleep(100); // [ms]       // [s] linux only
            }
        }
        nbytes = Message.length();

        if (_gio_write(Message.c_str(), nbytes) < 0)
            _opened = 0;
    }

    int base_io_udp::_gio_write(const char *buff, int size)
    {

        // create socket
        struct hostent *hostInfo;
        struct sockaddr_in servName;

#if defined(_WIN32) || defined(_WIN64)
        int addrlen; // remote address lenght
#else
        socklen_t addrlen; // remote address lenght
#endif

        servName.sin_family = AF_INET;
        servName.sin_port = htons(atoi(_port.c_str()));
        addrlen = sizeof(servName);

        // remote host info
        if ((hostInfo = gethostbyname(_host.c_str())) == NULL)
        {
            //fprintf(stderr, "wrong address [%s]\n", host.c_str());

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** no such host :", _host + "***");
            return false;
        }

        // complete sockaddr_in structure
        memcpy(&(servName.sin_addr), hostInfo->h_addr, hostInfo->h_length);

        // char line[1024];
        //sprintf(line, "\n%s", mesg);  // newline because of FORTRAN string

        int sz_total = size;
        int sz_ready = 0;
        int sz_sent = 0;
        // int sz_buffer = 1024;
        int wait_T = 0;
        char buffer[1024]; // sz_buffer (but ISO C++ forbids variable length array)

        // open failed
        if (_opened == 0)
            return -1;

        while (sz_total > 0)
        {

            // define size to be sent
            string tmp = buff;
            size_t ifirst = 0;
            if ((ifirst = tmp.find(crlf, sz_sent)) != string::npos)
                sz_ready = tmp.substr(sz_sent, ifirst + 1 - sz_sent).length();
            else
                sz_ready = sz_total;

            //strncpy( buffer, (in_buff + sz_sent), sz_ready );
            memcpy(buffer, (buff + sz_sent), sz_ready);

            sz_total -= sz_ready;

            if (wait_T > 0)
                base_time::gusleep(wait_T); // microseconds  (e.g. 1.2kBps -> 200000 us when sz=256)
            int sz_tmp = 0;
            string new_buffer;
            if (_using_ntrip_2_http)
            {
                ostringstream hex_num;
                // we need the number in hexa format instead of decimal
                hex_num << std::hex << sz_ready;
                // convert the stream to string
                string hex_num_str = string(hex_num.str());
                for (unsigned int i = 0; i < hex_num_str.size(); i++)
                { // capitalize a-f
                    // ascii 97 (decimal) is 'a', ascii 102 (decimal) is 'f'
                    if (hex_num_str[i] > 96 && hex_num_str[i] < 103)
                    {
                        // 'a' is 97 (decimal) ascii, 'A' is 65 (decimal) ascii, thus 32
                        hex_num_str[i] -= 32;
                    }
                }
                // add header to data being sent
                new_buffer = hex_num_str + string("\r\n") + string(buffer, sz_ready) + string("\r\n");
            }
            const int new_sz_ready = _using_ntrip_2_http ? new_buffer.size() : sz_ready;
            /* buffer slightly bigger than original (original has 256), so we have space
            * for header */
            char buffer_big[1024 + 256];
            // copy the data with header to buf for call of send()
            memcpy(buffer_big, _using_ntrip_2_http ? new_buffer.c_str() : buffer, new_sz_ready);

            if (sendto(_udp, buffer_big, new_sz_ready, 0, (struct sockaddr *)&servName, addrlen) == -1)
            { // MSG_DONTWAIT

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "gudp", " warning - sent to socket failed ");
                return -1;
            }
            else
            {
                ostringstream lg;
                lg << " sent to socket success: " << sz_ready; // << " " << buffer << endl;

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "gudp", lg.str());
            }
            /* so we have the size of data without header, if ntrip 2 not used then
            * new_sz_ready == sz_ready == sz_tmp and this changes nothing */
            sz_tmp = sz_ready;
            sz_sent += sz_tmp;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, , "gudp", " written bytes: " + int2str(sz_sent));

        return true;
    }

    int base_io_udp::_localhost(string &host)
    {
        try
        {
#ifdef _WIN32
            WORD wVersionRequested;
            WSADATA wsaData;
            int err;
            wVersionRequested = MAKEWORD(1, 1);
            err = WSAStartup(wVersionRequested, &wsaData);
            if (err != 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "base_io_tcp", "WSAStartup error ");
            }
            PHOSTENT hostinfo;
            char hostname[255] = {0};
            gethostname(hostname, sizeof(hostname));
            if ((hostinfo = gethostbyname(hostname)) == NULL)
            {
                errno = GetLastError();
                fprintf(stderr, "gethostbyname Error:%d\n", errno);
                return -1;
            }
            host = inet_ntoa(*(struct in_addr *)*hostinfo->h_addr_list);
            return 1;
#endif // _WIN32
            struct hostent *hostinfolinux;
            char hostnamelinux[255] = {0};
            gethostname(hostnamelinux, sizeof(hostnamelinux));
            if ((hostinfolinux = gethostbyname(hostnamelinux)) == NULL)
            {
                perror("base_io_tcp");
                fprintf(stderr, "gethostbyname Error:%d\n", errno);
                return -1;
            }
            host = inet_ntoa(*(struct in_addr *)*hostinfolinux->h_addr_list);
            return 1;
        }
        catch (...)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "base_io_tcp", "get local host fail! ");
            return -1;
        }
    }

    string base_io_udp::user()
    {
        return _user;
    }

    string base_io_udp::pwd()
    {
        return _pwd;
    }

    string base_io_udp::host()
    {
        return _host;
    }

    string base_io_udp::port()
    {
        return _port;
    }

    bool base_io_udp::glog_set(std::shared_ptr<spdlog::logger> spdlog)
    {
        // set spdlog
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }

        return true;
    }

    bool base_io_udp::_close_socket()
    {
#if defined(_WIN32) || defined(_WIN64)
        return closesocket(_udp);
#else
        return close(_udp);
#endif // _WIN32
    }

} // namespace