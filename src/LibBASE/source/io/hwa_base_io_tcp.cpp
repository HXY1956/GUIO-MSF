#include <cstring>
#include <iostream>
#include "hwa_base_io_tcp.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_time.h"
#include "hwa_base_log.h"

using namespace spdlog;
using namespace std;

namespace hwa_base
{

    base_io_tcp::base_io_tcp() : base_io()
    {
        _user = "";
        _pwd = "";
        _host = "";
        _port = "";
        _spdlog = nullptr;
    }

    base_io_tcp::base_io_tcp(std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        _user = "";
        _pwd = "";
        _host = "";
        _port = "";
        _spdlog = nullptr;
    }

    base_io_tcp::base_io_tcp(const int &port)
    {
        _user = "";
        _pwd = "";
        _port = base_type_conv::int2str(port);
    }

    base_io_tcp::base_io_tcp(const int &port, std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        _user = "";
        _pwd = "";
        _port = base_type_conv::int2str(port);
    }

    base_io_tcp::base_io_tcp(const string &address, std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        string addr = address;
        string type;
        string::size_type flag = address.find("@");

        for (int i = 0; i < address.size(); i++)
        {
            if (address[i] == ':' || address[i] == '/' || address[i] == '@')
                addr[i] = ' ';
        }
        stringstream ss(addr);
        if (flag == string::npos)
        {
            _user = "";
            _pwd = "";
            ss >> type >> _host >> _port;
            if (_host == "localhost" && _localhost(_host) < 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
            }
        }
        else
        {
            ss >> type >> _user >> _pwd >> _host >> _port >> _mountpoint;
        }
    }

    base_io_tcp::base_io_tcp(const string &address) : base_io()
    {
        string addr = address;
        string type;
        string::size_type flag = address.find("@");

        for (int i = 0; i < address.size(); i++)
        {
            if (address[i] == ':' || address[i] == '/' || address[i] == '@')
                addr[i] = ' ';
        }
        stringstream ss(addr);
        if (flag == string::npos)
        {
            _user = "";
            _pwd = "";
            ss >> type >> _host >> _port;
            if (_host == "localhost" && _localhost(_host) < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
            }
        }
        else
        {
            ss >> type >> _user >> _pwd >> _host >> _port >> _mountpoint;
        }
    }

    base_io_tcp::base_io_tcp(const string &address, std::shared_ptr<spdlog::logger> spdlog, bool upload) : base_io(spdlog)
    {
        _upload = upload;
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
                SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
        }
    }

    base_io_tcp::base_io_tcp(const string &host, const int &port, const string &user, const string &password,
                   const string &mountpoint, int rate)
    {
        _host = host;
        _port = base_type_conv::int2str(port);
        _user = user;
        _pwd = password;
        _mountpoint = mountpoint;

        if (_host == "")
        {
            _host = "localhost";
            _user = "";
            _pwd = "";
            if (_localhost(_host) < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
            }
        }
    }

    base_io_tcp::base_io_tcp(std::shared_ptr<spdlog::logger> spdlog, const string &host, const int &port, const string &user, const string &password,
                   const string &mountpoint, int rate) : base_io(spdlog)
    {
        _host = host;
        _port = base_type_conv::int2str(port);
        _user = user;
        _pwd = password;
        _mountpoint = mountpoint;

        if (_host == "")
        {
            _host = "localhost";
            _user = "";
            _pwd = "";
            if (_localhost(_host) < 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
            }
        }
    }

    base_io_tcp::~base_io_tcp()
    {
    }

    void base_io_tcp::run_read()
    {
        // be sure always clean decoder
        if (_coder)
        {
            _coder->clear();
        }
        // For error message
        vector<string> errmsg;

        char *loc_buff = new char[_size];

        int nbytes = 0;
        // int decoded = 0;
        _stop = 0;
        _running = 1;

        while (1)
        {
            if (init_read() < 0 || (_mountpoint != "" && _NtripDownload_Init() <= 0))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error - connect failed");
                base_time::gmsleep(100); // [ms]       // [s] linux only
                continue;
            }
            while (((nbytes = _gio_read(loc_buff, _size)) > 0) && _stop != 1)
            {
                // archive the stream
                //_locf_write(loc_buff, nbytes);

                if (_coder && nbytes > 0)
                {
                    _coder->decode_data(loc_buff, nbytes, _count, errmsg);
                }
                else
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, , "gio", "0 data decoded");
                }

                //if (_log && _log->verb() >= 9) _log->comment(9, "gio", "READ : " + int2str(nbytes)
                //    + " decoded: " + int2str(decoded));
            }
        }
        _stop_common();
        delete[] loc_buff;
    }

    void base_io_tcp::run_write()
    {
        // be sure always clean decoder
        if (_coder)
        {
            _coder->clear();
        }
        // For error message
        vector<string> errmsg;

        char *loc_buff = new char[_size];

        int nbytes = 0;
        // int decoded = 0;
        _stop = 0;
        _running = 1;

        while (1)
        {
            if (_Server_Init() <= 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error - connect failed");
                _opened = 0;
                base_time::gmsleep(100); // [ms]       // [s] linux only
                continue;
            }
            do
            {

                nbytes = 0;
                //// try to read from local file source
                //nbytes = _locf_read(loc_buff, _size);

                //if (_coder && nbytes < 0) {
                //    nbytes = _coder->encode_data(loc_buff, _size, _count, errmsg);
                //}
                string out = "abcdefg\n";
                //strcpy(loc_buff, out.c_str());
                loc_buff = (char *)out.c_str();
                _size = out.size();

                base_time::gmsleep(100);
                _stop = 0;
                nbytes = out.size();
                ;

            } while (_stop == 0 && (_gio_write(loc_buff, nbytes) > 0 || nbytes < -1));
        }
        _stop_common();
        delete[] loc_buff;
    }

    void base_io_tcp::run_send(const string &Message)
    {
        int nbytes = 0;
        if (!_opened) //zhshen
        {
            if (_Server_Init() <= 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " error - connect failed");
                _opened = 0;
                base_time::gmsleep(100); // [ms]       // [s] linux only
            }
        }
        nbytes = Message.length();

        if (_gio_write(Message.c_str(), nbytes) < 0)
            _opened = 0;
    }

    void base_io_tcp::run_upload(const char *Message, int size)
    {
        int iter = 0;
        while (iter <= 3)
        {
            iter++;
            if (!_opened)
            {
                if (_NtripUpload_Init() <= 0)
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, " error - connect failed, iter = " + base_type_conv::int2str(iter));
                    _opened = 0;
                    base_time::gmsleep(30); // [ms]       // [s] linux only
                    continue;
                }
            }

#if defined _WIN32 || defined _WIN64
            if (send(_fd_skt, Message, size, 0) < 0)
#else
            if (send(_fd_skt, Message, size, MSG_NOSIGNAL) < 0) // MSG_DONTWAIT  !!!!!!!!!!!! SO_NOSIGPIPE
#endif
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " failed in sending messages to NTRIP caster, iter = " + base_type_conv::int2str(iter));
                _opened = 0;
                continue;
            }
            else
            {
                break;
            }
        }
    }

    int base_io_tcp::_gio_write(const char *buff, int size)
    {

        int sz_total = size;
        int sz_ready = 0;
        int sz_sent = 0;
        int sz_buffer = 1024;
        int wait_T = 0;
        char buffer[1024]; // sz_buffer (but ISO C++ forbids variable length array)

        // wait-time in usec to provide kBps bandwidth
        if (bandwidth != 0)
            wait_T = (sz_buffer * 1000000 / bandwidth);

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

#if defined _WIN32 || defined _WIN64
            if (sz_ready != (sz_tmp = send(_fd_acpt, buffer_big, new_sz_ready, 0)))
            { // NOT TESTED, ONLY COMPILED
#else
            if (new_sz_ready != (sz_tmp = send(_fd_skt, buffer_big, new_sz_ready, MSG_NOSIGNAL)))
            { // MSG_DONTWAIT
#endif

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " warning - sent to socket failed ");
                return -1;
            }
            else
            {
                ostringstream lg;
                lg << _fd_acpt << " sent to socket success: " << sz_ready; // << " " << buffer << endl;

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, lg.str());
                //else        cerr << lg.str() << endl;
            }
            /* so we have the size of data without header, if ntrip 2 not used then
            * new_sz_ready == sz_ready == sz_tmp and this changes nothing */
            sz_tmp = sz_ready;
            sz_sent += sz_tmp;
        }

        //if( sz_sent == 0 ) sz_sent = -1; // to set non-zero return value!

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, , "gtcp", " written bytes: " + int2str(sz_sent));
        return sz_sent;
    }

    int base_io_tcp::_gio_read(char *buff, int size)
    {

        //_gmutex.lock();

        //int size = 1024;
        char *Buff = new char[size];
        int nBytes;

// fix bug
#if defined(_WIN32) || defined(_WIN64)
        if ((nBytes = recv(_fd_skt, buff, size - 1, 0)) >= 1)
        {
#else
        if ((nBytes = recv(_fd_skt, buff, sizeof(Buff) - 1, MSG_WAITALL)) >= 1)
        {

#endif // _WIN32

            buff[nBytes] = '\0';
            //buff = Buff;
        }
        else
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " Error any response [reading data]");
        }
        if (Buff)
        {
            delete[] Buff;
            Buff = nullptr;
        }

        //_gmutex.unlock();
        return nBytes;
    }

    int base_io_tcp::_localhost(string &host)
    {
        try
        {
#if defined(_WIN32) || defined(_WIN64)
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

    int base_io_tcp::_init_common()
    {
        struct hostent *Host;
        struct sockaddr_in addr;

#if defined(_WIN32) || defined(_WIN64)
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
#endif // _WIN32

        _fd_skt = -1;
        _fd_skt = socket(AF_INET, SOCK_STREAM, 0);
        if (socket(AF_INET, SOCK_STREAM, 0) == -1)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, strerror(errno));
        }
        if (_fd_skt < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** opening socket ***");
            return false;
        }

        // HOSTNAME
        if (!(Host = gethostbyname(_host.c_str())))
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** no such host :", _host + "***");
            return false;
        }
#if defined(_WIN32) || defined(_WIN64)
        memset((char *)&addr, 0, sizeof(addr));
#else
        bzero((char *)&addr, sizeof(addr));
#endif // _WIN32

        addr.sin_family = AF_INET;

#if defined(_WIN32) || defined(_WIN64)
        memcpy((char *)&addr.sin_addr.s_addr, (char *)Host->h_addr, Host->h_length);
#else
        bcopy((char *)Host->h_addr, (char *)&addr.sin_addr.s_addr, Host->h_length);
#endif // _WIN32

        addr.sin_port = htons(atoi(_port.c_str()));

        // CONNECT
        try
        {
            if (connect(_fd_skt, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            {
                // if(_spdlog) SPDLOG_LOGGER_ERROR(_spdlog, "***connecting to:", inet_ntoa(addr.sin_addr) + '***');
            }
        }
        catch (...)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "***connecting to: {} {}", inet_ntoa(addr.sin_addr), "***");
            return false;
        }
        return true;
    }

    string base_io_tcp::user()
    {
        return _user;
    }

    string base_io_tcp::pwd()
    {
        return _pwd;
    }

    string base_io_tcp::host()
    {
        return _host;
    }

    string base_io_tcp::port()
    {
        return _port;
    }

    bool base_io_tcp::glog_set(std::shared_ptr<spdlog::logger> spdlog)
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

    bool base_io_tcp::_Server_Init()
    {
        _close_socket();

        // struct hostent *Host;
        struct sockaddr_in saddr;

#if defined(_WIN32) || defined(_WIN64)
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
#endif // _WIN32

        _fd_skt = -1;
        _fd_skt = socket(AF_INET, SOCK_STREAM, 0);
        if (socket(AF_INET, SOCK_STREAM, 0) == -1)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, strerror(errno));
        }
        if (_fd_skt < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** opening socket ***");
            return false;
        }

        saddr.sin_family = AF_INET;

        saddr.sin_port = htons(atoi(_port.c_str()));
        //saddr.sin_addr.s_addr = htons(0);
        saddr.sin_addr.s_addr = htonl(INADDR_ANY);

        int retVal = ::bind(_fd_skt, (const struct sockaddr *)&saddr, int(sizeof(sockaddr_in)));
        if (retVal == -1)
        {
            perror("base_io_tcp");
            fprintf(stderr, "_init_common Error bind failed:%d\n", errno);
            //printf("Failed bind:%d\n", WSAGetLastError());
            return false;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** bind success ***");

        if (listen(_fd_skt, 10) == -1)
        {
            perror("base_io_tcp");
            fprintf(stderr, "listen failed:%d\n", errno);
            //printf("Listen failed:%d", WSAGetLastError());
            return false;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** listenning ***");

        int len = sizeof(_caddr);
        fprintf(stderr, "waiting for client\n");
#if defined(_WIN32) || defined(_WIN64)
        _fd_acpt = accept(_fd_skt, (sockaddr *)&_caddr, &len);
#else
        _fd_acpt = accept(_fd_skt, (sockaddr *)&_caddr, (socklen_t *)(&len));

#endif // _WIN32

        if (_fd_acpt == -1)
        {
            fprintf(stderr, "accept failed:%d\n", errno);
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** accepted ***");
        _opened = 1;

        return true;
    }

    bool base_io_tcp::_NtripUpload_Init()
    {
        if (_mountpoint.empty())
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** no mountpoint ***");
            return false;
        }
        _close_socket();

#if defined(_WIN32) || defined(_WIN64)
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
#endif // _WIN32

        // The sockaddr_in structure specifies the address family,
        // IP address, and port of the server to be connected to.
        struct sockaddr_in saddr;
        saddr.sin_family = AF_INET;
        saddr.sin_port = htons(atoi(_port.c_str()));
        saddr.sin_addr.s_addr = inet_addr(_host.c_str());

        /* Connect to ntrip server according to IP and port number */
        _fd_skt = -1;
        _fd_skt = socket(AF_INET, SOCK_STREAM, 0);
        if (_fd_skt == -1)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** opening socket ***");
            return false;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** opening socket success ***");

        /* Connect to server.*/
        int ret = connect(_fd_skt, (sockaddr *)&saddr, sizeof(saddr));
        if (ret < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** connecting to server ***");
            return false;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** connecting to server success ***");

        /* Send request data. */
        string msg = "SOURCE " + _pwd + " /" + _mountpoint + "\r\n" +
                     "Source-Agent: NTRIP NtripServerGREAT/1.0\r\n" +
                     "\r\n";
        ret = send(_fd_skt, msg.c_str(), strlen(msg.c_str()), 0);
        if (ret < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** trying send request fail ***");
            return false;
        }

        /* Wait for request to connect caster success. */
        char recvBuff[1024];
        while (true)
        {
            memset(recvBuff, 0, sizeof(recvBuff) / sizeof(char));
            ret = recv(_fd_skt, recvBuff, 1024, 0); // 0
            if (ret > 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "Ntrip return:  " + string(recvBuff));
                string return_str = string(recvBuff);
                if (return_str.find("ICY 200 OK\r\n") != string::npos)
                {
                    break;
                }
            }
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** connection opened ***");
        _opened = 1;

        return true;
    }

    const char base64_code_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    char index2chr(int index)
    {
        return base64_code_table[index];
    }

    int base64_encode(char *src, char *result)
    {
        char temp[3] = {0};
        int i = 0, j = 0, count = 0;
        int len = strlen(src);
        if (len == 0)
            return -1;

        if (len % 3 != 0)
        {
            count = 3 - len % 3;
        }

        while (i < len)
        {
            strncpy(temp, src + i, 3);
            result[j + 0] = index2chr((temp[0] & 0xFC) >> 2);
            result[j + 1] = index2chr(((temp[0] & 0x3) << 4) | ((temp[1] & 0xF0) >> 4));
            if (temp[1] == 0)
                break;
            result[j + 2] = index2chr(((temp[1] & 0xF) << 2) | ((temp[2] & 0xC0) >> 6));
            if (temp[2] == 0)
                break;
            result[j + 3] = index2chr(temp[2] & 0x3F);
            i += 3;
            j += 4;
            memset(temp, 0x0, 3);
        }

        while (count)
        {
            result[j + 4 - count] = '=';
            --count;
        }

        return 0;
    }

    bool base_io_tcp::_NtripDownload_Init()
    {
        if (_mountpoint.empty())
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** no mountpoint ***");
            return false;
        }
        _close_socket();

#if defined(_WIN32) || defined(_WIN64)
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
#endif // _WIN32

        // The sockaddr_in structure specifies the address family,
        // IP address, and port of the server to be connected to.
        struct sockaddr_in saddr;
        saddr.sin_family = AF_INET;
        saddr.sin_port = htons(atoi(_port.c_str()));
        saddr.sin_addr.s_addr = inet_addr(_host.c_str());

        /* Connect to ntrip server according to IP and port number */
        _fd_skt = -1;
        _fd_skt = socket(AF_INET, SOCK_STREAM, 0);
        if (_fd_skt == -1)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** opening socket ***");
            return false;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** opening socket success ***");

        /* Connect to server.*/
        int ret = connect(_fd_skt, (sockaddr *)&saddr, sizeof(saddr));
        if (ret < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** connecting to server ***");
            return false;
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** connecting to server success ***");

        /* Send request data. */
        char userinfo[64] = {0};
        char userinfo_raw[48] = {0};
        // snprintf(userinfo_raw, 63, "%s:%s", _user.c_str(), _pwd.c_str()); //lvhb changed for linux
        snprintf(userinfo_raw, 47, "%s:%s", _user.c_str(), _pwd.c_str()); //lvhb changed for linux
        base64_encode(userinfo_raw, userinfo);
        string msg = "GET /" + _mountpoint + " HTTP/1.1\r\n" + "Host: " + _host + "\r\n" + "User-Agent: NTRIP NtripClientGREAT/1.0\r\n"
                     //+"Accept: */*\r\n"
                     //+"Connection: close\r\n"
                     + "Authorization: Basic " + userinfo + "\r\n" + "\r\n";

        ret = send(_fd_skt, msg.c_str(), strlen(msg.c_str()), 0);
        if (ret < 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "*** trying send request fail ***");
            return false;
        }

        /* Wait for request to connect caster success. */
        char recvBuff[1024];
        //char gpgga[] = "$GPGGA,083552.00,3000.0000000,N,11900.0000000,E,1,08,1.0,0.000,M,100.000,M,,*57\r\n";
        while (true)
        {
            memset(recvBuff, 0, sizeof(recvBuff) / sizeof(char));
            ret = recv(_fd_skt, recvBuff, 1024, 0); // 0
            if (ret > 0)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "Ntrip return:  " + string(recvBuff));
                string return_str = string(recvBuff);
                //ret=send(_fd_skt, gpgga, strlen(gpgga), 0);
                if (ret > 0)
                {
                    if (return_str.find("ICY 200 OK\r\n") != string::npos)
                        break;
                }
                else
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "send gpgga data fail");
                    return false;
                }
            }
        }

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "*** connection opened ***");
        _opened = 1;
        return true;
    }

    bool base_io_tcp::_close_socket()
    {
#if defined(_WIN32) || defined(_WIN64)
        return closesocket(_fd_skt);
#else
        return close(_fd_skt);
#endif // _WIN32
    }

} // namespace