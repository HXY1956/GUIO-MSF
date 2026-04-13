#ifndef hwa_base_io_h
#define hwa_base_io_h

#include <stdio.h>
#include <fstream>
#include <string>
#include "hwa_base_log.h"
#include "hwa_base_mutex.h"
#include "hwa_base_coder.h"
#include "hwa_base_iof.h"

#define BUF_SIZE 1024

namespace hwa_base
{
    class base_coder;
    class base_io
    {

    public:
        /** @brief default constructor. */
        base_io();

        base_io(base_log spdlog);
        /** @brief default destructor. */
        virtual ~base_io();

        /** @brief start reading (could be run in a separate thread). */
        virtual void run_read();
        //   virtual void run_read(void*){ run_read(); }

        /** @brief start writing (could be run in a separate thread). */
        virtual void run_write();
        virtual void coder(base_coder *coder) { _coder = coder; }

        /** 
        * @brief init write. 
        * @return
            @retval >0    success 
        */
        virtual int init_write() { return _opened = _init_common(); }

        /** 
        * @brief init read.
        * @return
            @retval >0    success
        */
        virtual int init_read() { return _opened = _init_common(); }

        /** 
        * @brief stop write. 
        * @return
        *    @retval =0    success
        */
        virtual int stop_write()
        {
            _stop_common();
            return 0;
        }

        /**
        * @brief read write.
        * @return
        *    @retval =0    success
        */
        virtual int stop_read()
        {
            _stop_common();
            return 0;
        }
        /**
        * @brief std::set file://dir/name.
        * @param[in]    str        file path
        * @return
        *    @retval =-1 false
        *    @retval =1    true
        */
        virtual int path(std::string str);

        /**
        * @brief get file://dir/name.
        * @return        file path
        */
        virtual std::string path() const { return _path; }

        /** 
        * @brief std::set local i/o file.
        * @param[in]    f    file name
        */
        void file(const char *f) { _giof.mask(f); }

        /**
        * @brief get local i/o file.
        * @return        file name
        */
        std::string file() { return _giof.mask(); }

        /**
        * @brief std::set verbosity.
        * @param[in]    i    verbosity
        */
        void verb(int i) { _verb = i; }

        /** 
        * @brief get verbosity.
        * @return        verbosity
        */
        int verb() { return _verb; }

        /** @brief stop. */
        void stop() { _stop = 1; }

        /**
        * @brief get size. 
        * @return        size
        */
        size_t size() { return _size; }

        /** 
        * @brief get running.
        * @return        running status 
        */
        int running() { return _running; }

        /** @brief get opened. */
        int opened() { return _opened; }

        /** @brief get opened. */
        int connected() { return _opened; }

        /** 
        * @brief std::set glog pointer. 
        * @param[in]    l    glog pointer
        */
        void spdlog(base_log spdlog);

        /**
        * @brief get glog pointer.
        * @return        glog pointer
        */
        base_log spdlog() { return _spdlog; }

        /** @brief override opreator < == <<. */
        bool operator<(const base_io &n) const;
        bool operator==(const base_io &n) const;
        friend std::ostream &operator<<(std::ostream &os, const base_io &n);

    protected:
        /**
        * @brief send data.
        * @param[in]    buff    buffer of the data
        * @param[in]    size    buffer size of the data
        * @return
            @retval >0    buffer size of the data
            @retval <=0    fail
        */
        virtual int _gio_write(const char *buff, int size) = 0;

        /**
        * @brief read data.
        * @param[in]    buff    buffer of the data
        * @param[in]    size    buffer size of the data
        * @return
            @retval >0    number of bytes read
            @retval <=0    fail
        */
        virtual int _gio_read(char *buff, int size) = 0;

        /**
        * @brief local log file archive. 
        * @param[in]    buff    buffer of the data
        * @param[in]    size    buffer size of the data
        * @return
            @retval >=0    sucess
            @retval <0    fail
        */
        virtual int _locf_write(const char *buff, int size);

        /**
        * @brief local log file source. 
        * @param[in]    buff    buffer of the data
        * @param[in]    size    buffer size of the data
        * @return
            @retval >=0    sucess
            @retval <0    fail
        */
        virtual int _locf_read(char *buff, int size);

        /** 
        * @brief common function for initialization. 
        * @return
            @retval >=0    sucess
        */
        virtual int _init_common();

        /**
        * @brief common function for socket/file close. 
        * @return    running status 
        */
        virtual int _stop_common();
        base_log _spdlog; ///< spdlog pointer
        int _fd;          ///< file descriptor
        size_t _size;     ///< buffer size
        std::string _path;     ///< URL-like path (e.g. file:///home/honza/file)
        base_iof _giof;     ///< local file
        //   char*           _loc_buff;         // local buffer
        int _count;       ///< record counter
        int _verb;        ///< verbosity
        int _stop;        ///< require a stop at run() loop
        int _opened;      ///< 1: opened/connected
        int _running;     ///< running
        base_coder *_coder; ///< decoder/encoder
        base_mutex _gmutex; ///< mutual exlusion

#ifdef BMUTEX
        boost::mutex _mutex; // mutual exlusion
#endif

    private:
    };

} // namespace

#endif
