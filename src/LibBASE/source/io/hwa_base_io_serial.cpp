#include <cstring>
#include <iostream>
#include "hwa_base_io_serial.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_time.h"
#include "hwa_base_log.h"

using namespace std;

namespace hwa_base
{

    base_io_serial::base_io_serial() : base_io()
    {
        _port = 0;
        _baud = 0;
        _spdlog = nullptr;
#ifdef _WIN32
        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
        m_hIDComDev = NULL;
#else
        /// for linux
        _fd = -1;
#endif
    }

    base_io_serial::base_io_serial(std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        _port = 0;
        _baud = 0;
        _spdlog = nullptr;
#ifdef _WIN32
        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
        m_hIDComDev = NULL;
#else
        /// for linux
        _fd = -1;
#endif
    }

    base_io_serial::base_io_serial(int port, int baud) : base_io()
    {
        _port = port;
        _baud = baud;
        _spdlog = nullptr;
#ifdef _WIN32
        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
        m_hIDComDev = NULL;
#else
        /// for linux
        _fd = -1;
#endif
    }

    base_io_serial::base_io_serial(int port, int baud, std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        _port = port;
        _baud = baud;
        _spdlog = nullptr;
#ifdef _WIN32
        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
        m_hIDComDev = NULL;
#else
        /// for linux
        _fd = -1;
#endif
    }

    base_io_serial::base_io_serial(string path) : base_io()
    {
        path.erase(0, 12); /// erase 'com'
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i] == ':' || path[i] == ',' || path[i] == '/')
                path[i] = ' ';
        }
        stringstream ss(path);
        string port, baud, data_bit, stop_bit, flow_contrl;
        ss >> port >> baud >> data_bit >> stop_bit >> flow_contrl;
        _port = base_type_conv::str2int(port);
        _baud = base_type_conv::str2int(baud);
        _spdlog = nullptr;
        if (_port < 0 || _baud < 0)
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
#ifdef _WIN32
        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
        m_hIDComDev = NULL;
#else
        /// for linux
        _fd = -1;
#endif
    }

    base_io_serial::base_io_serial(string path, std::shared_ptr<spdlog::logger> spdlog) : base_io(spdlog)
    {
        path.erase(0, 12); /// erase 'com'
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i] == ':' || path[i] == ',' || path[i] == '/')
                path[i] = ' ';
        }
        stringstream ss(path);
        string port, baud, data_bit, stop_bit, flow_contrl;
        ss >> port >> baud >> data_bit >> stop_bit >> flow_contrl;
        _port = base_type_conv::str2int(port);
        _baud = base_type_conv::str2int(baud);
        _spdlog = nullptr;
        if (_port < 0 || _baud < 0)

            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, " error lcoal host !");
#ifdef _WIN32
        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
        m_hIDComDev = NULL;
#else
        /// for linux
        _fd = -1;
#endif
    }

    base_io_serial::~base_io_serial()
    {
        //close();
    }

    void base_io_serial::run_read()
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
            if (init_read() <= 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "gserial", " error - connect failed");
                base_time::gmsleep(100); // [ms]       // [s] linux only
                continue;
            }
            while (((nbytes = _gio_read(loc_buff, _size)) > 0) && _stop != 1)
            {
                if (_coder && nbytes > 0)
                {
                    _coder->decode_data(loc_buff, nbytes, _count, errmsg);
                }
                else
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, , "gio", "0 data decoded");
                }
            }
        }
        _stop_common();
        delete[] loc_buff;
    }

    int base_io_serial::_gio_read(char *buff, int size)
    {

        //

#ifdef _WIN32
        if (!_opened || m_hIDComDev == NULL)
        {
            base_time::gmsleep(100); // [ms]
            return (0);
        }
        BOOL bReadStatus;
        DWORD dwBytesRead, dwErrorFlags;
        COMSTAT ComStat;

        ClearCommError(m_hIDComDev, &dwErrorFlags, &ComStat);
        if (!ComStat.cbInQue)
            return (0);

        dwBytesRead = (DWORD)ComStat.cbInQue;
        if (size < (int)dwBytesRead)
            dwBytesRead = (DWORD)size;

        bReadStatus = ReadFile(m_hIDComDev, buff, dwBytesRead, &dwBytesRead, &m_OverlappedRead);
        if (!bReadStatus)
        {
            if (GetLastError() == ERROR_IO_PENDING)
                PurgeComm(m_hIDComDev, PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
            return (0);
        }

        //
        return ((int)dwBytesRead);

#else
        ///for linux
        int BytesRead;
        BytesRead = read(_fd, buff, size);
        return ((int)BytesRead);
#endif
    }

    bool base_io_serial::glog_set(std::shared_ptr<spdlog::logger> spdlog)
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

    int base_io_serial::_init_common()
    {
#ifdef _WIN32
        char szPort[15];
        char szComParams[50];
        DCB dcb;

        sprintf(szPort, "\\\\.\\COM%d", _port);
        m_hIDComDev = CreateFileA(szPort, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                  OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
        if (m_hIDComDev == NULL)
            return (0);

        memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
        memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));

        COMMTIMEOUTS CommTimeOuts;
        CommTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
        CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
        CommTimeOuts.ReadTotalTimeoutConstant = 0;
        CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
        CommTimeOuts.WriteTotalTimeoutConstant = 5000;
        SetCommTimeouts(m_hIDComDev, &CommTimeOuts);

        sprintf(szComParams, "COM%d:%d,n,8,1", _port, _baud);

        m_OverlappedRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        m_OverlappedWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

        dcb.DCBlength = sizeof(DCB);
        GetCommState(m_hIDComDev, &dcb);
        dcb.BaudRate = _baud;
        dcb.ByteSize = 8;
        unsigned char ucSet;
        ucSet = (unsigned char)((FC_RTSCTS & FC_DTRDSR) != 0);
        ucSet = (unsigned char)((FC_RTSCTS & FC_RTSCTS) != 0);
        ucSet = (unsigned char)((FC_RTSCTS & FC_XONXOFF) != 0);
        if (!SetCommState(m_hIDComDev, &dcb) ||
            !SetupComm(m_hIDComDev, 10000, 10000) ||
            m_OverlappedRead.hEvent == NULL ||
            m_OverlappedWrite.hEvent == NULL)
        {
            DWORD dwError = GetLastError();
            /// debug
            cout << dwError << endl;
            if (m_OverlappedRead.hEvent != NULL)
                CloseHandle(m_OverlappedRead.hEvent);
            if (m_OverlappedWrite.hEvent != NULL)
                CloseHandle(m_OverlappedWrite.hEvent);
            CloseHandle(m_hIDComDev);
            return (0);
        }
        _opened = TRUE;
        return _opened;

#else
        ///for linux
        char szPort[15];
        sprintf(szPort, "/dev/ttyUSB%d", 0);
        _fd = open(szPort, O_RDWR);
        if (_fd == -1)
            return 0;
        int DTR_flag;
        DTR_flag = TIOCM_DTR;
        ioctl(_fd, TIOCMBIS, &DTR_flag);

        // int i;
        // int status;
        struct termios options;
        if (tcgetattr(_fd, &options) != 0)
        {
            //perror("SetupSerial 1");
            return (false);
        }

        cfsetispeed(&options, B115200);
        options.c_cflag |= CLOCAL;
        options.c_cflag |= CREAD;

        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        options.c_cflag &= ~CSTOP;
        options.c_oflag &= ~OPOST;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_cc[VTIME] = 1;
        options.c_cc[VMIN] = 1;
        tcflush(_fd, TCIFLUSH);
        if (tcsetattr(_fd, TCSANOW, &options) != 0)
        {
            perror("com set error!\n");
            return (false);
        }
        _opened = true;
        return true;
#endif
    }

    bool base_io_serial::close()
    {
#ifdef _WIN32
        if (!_opened || m_hIDComDev == NULL)
            return (TRUE);

        if (m_OverlappedRead.hEvent != NULL)
            CloseHandle(m_OverlappedRead.hEvent);
        if (m_OverlappedWrite.hEvent != NULL)
            CloseHandle(m_OverlappedWrite.hEvent);
        CloseHandle(m_hIDComDev);
        _opened = FALSE;
        m_hIDComDev = NULL;
        return (TRUE);
#else
        return true;
#endif
    }
} // namespace hwa_gnss