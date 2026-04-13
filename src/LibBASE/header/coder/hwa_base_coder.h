#ifndef hwa_base_coder_h
#define hwa_base_coder_h
#include <set>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <sstream>
#include "hwa_set_base.h"
#include "hwa_base_log.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_timesync.h"
#include "hwa_base_note.h"
#include "hwa_base_time.h"
#include "hwa_base_data.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_io.h"
#include "hwa_base_mutex.h"
#include "hwa_base_common.h"
#include "hwa_base_coderbuffer.h"

#define BUFFER_INCREASE_FAC 1.5
#define DEFAULT_BUFFER_SIZE 4096
#define MAXIMUM_BUFFER_SIZE 240000 // 9000000

using namespace spdlog;

namespace hwa_base
{
    class base_io;
    /**
    *@brief       Class for decoding
    */
    class base_coder
    {

    public:
        /**
         * @brief default constructor.
         */
        base_coder();

        /**
        * @brief override constructor 1.
        *
        * @param[in]  s            setbase control
        * @param[in]  version    version of the gcoder
        * @param[in]  sz        size of the buffer
        * @param[in]  id        std::string for reporting
        */
        explicit base_coder(hwa_set::set_base* s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE, std::string id = "gcoder");

        /**
        * @brief override constructor 2.
        * @param[in]  beg        begin time
        * @param[in]  end        end time
        * @param[in]  s            setbase control
        * @param[in]  version    version of the gcoder
        * @param[in]  sz        size of the buffer
        * @param[in]  id        std::string for reporting
        */
        base_coder(base_time beg, base_time end, hwa_set::set_base* s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE, std::string id = "gcoder");

        /**
        * @brief override constructor.
        * @param[in]  s            setbase control
        * @param[in]  sz        size of the buffer
        */
        explicit base_coder(hwa_set::set_base* s, int sz = DEFAULT_BUFFER_SIZE); // for disable GNSS

        base_time beg_epoch = base_time(0, 0); ///<        begin epoch
        base_time end_epoch = base_time(0, 0); ///<        end epoch
        base_time epoch = base_time(0, 0);     ///<        epoch

        /** @brief default destructor. */
        virtual ~base_coder();


        virtual int _gset(hwa_set::set_base* s) {
            if (!s)
                return -1;
            return 0;
        };
        /**
        * @brief clear the buffer
        * @return void
        */
        virtual void clear();
        /**
        * @brief clear the buffer
        * @return void
        */
        virtual void version(const std::string &s) { _version = s; }

        /** @brief get version. */
        virtual const std::string &version() const { return _version; }

        /** @brief set/get out_length. */
        virtual void out_length(const int &len) { _out_len = len; }

        /**
         * @brief 
         * 
         * @return const int& 
         */
        virtual const int &out_length() const { return _out_len; }

        /** @brief set/get out_sample. */
        virtual void out_sample(const float &smp) { _out_smp = smp; }

        /**
         * @brief 
         * 
         * @return const float& 
         */
        virtual const float &out_sample() const { return _out_smp; }

        /** @brief set/get out_epoch. */
        virtual void out_epoch(const base_time &epo) { _out_epo = epo; }

        /**
         * @brief 
         * 
         * @return const base_time& 
         */
        virtual const base_time &out_epoch() const { return _out_epo; }

        /** @brief decode head. */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) { return 0; } // = 0;

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) { return 0; } // = 0;

        /** @brief encode head. */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) { return 0; } // = 0;

        /**
         * @brief 
         * 
         * @param buff 
         * @param sz 
         * @param cnt 
         * @param errmsg 
         * @return int 
         */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) { return 0; } // = 0;

        /** @brief get irc. */
        const int &irc() const { return _irc; }

        /** @brief set/get glog pointer. */
        void spdlog(std::shared_ptr<logger> spdlog);

        /**
         * @brief 
         * 
         * @return std::shared_ptr<logger> 
         */
        std::shared_ptr<logger> spdlog() { return _spdlog; }

        /** @brief set/get path. */
        void path(const std::string &s);

        /**
         * @brief 
         * 
         * @return const std::string& 
         */
        const std::string &path() const { return _fname; }

        /** @brief set gio_ptr. */
        void add_gio(std::weak_ptr<base_io> p) { _gio_ptr = p; }

        /** @brief add/get data. */
        int add_data(const std::string &data_id, base_data *data);

        //    int rem_data( std::string data_id );
        base_data *data(const std::string &data_id) { return _data[data_id]; }

        /** @brief set/get notes (messages/warning/errors). */
        void mesg(const base_note_type &m, const std::string &s); //

        /**
         * @brief 
         * 
         * @return const std::vector<base_note>& 
         */
        const std::vector<base_note> &mesg() const;

        /** @brief set close with warning. */
        void close_with_warning(const bool &b) { _close_with_warning = b; }

        /** @brief set pgm. */
        void pgm(const std::string &pgm) { _pgm = pgm; }

    protected:
        /** @brief get endpos/size/buffer. */
        const int &endpos() const { return _endpos; }

        /**
         * @brief 
         * 
         * @return const int& 
         */
        const int &size() const { return _endpos; } // number of char elements in buffer (excl \0)

        /**
         * @brief 
         * 
         * @return char* 
         */
        char *buffer() { return _buffer; }

        /** @brief init. */
        virtual void _init();

        /**
        * @brief add data.
        * @param[in]  id        data type
        * @param[in]  data        base_data
        * @return void
        */
        virtual void _add_data(const std::string &id, base_data *data){};

        /**
        * @brief sampling filter for epochs.
        * @param[in]  epo        current epoch
        * @return 
        *        true:the epoch fits sampling
        *        false:the epoch does not fit sampling
        */
        virtual bool _filter_epoch(const base_time &epo);

        /**
        * @brief get single line from the buffer.
        * @param[in]  str        the content of the single line
        * @param[in]  from_pos    the position in the buffer
        * @return      int 
        */
        int _getline(std::string &str, int from_pos = 0);

        /**
        * @brief get the buffer.
        * @param[in]  buff        buffer
        * @return      int
        */
        int _getbuffer(const char *&buff);

        /**
        * @brief get the buffer.
        * @param[in]  buff        buffer
        * @return      int
        */
        int _add2buffer(char *buff, int sz);

        /**
        * @brief remove from buffer.
        * @param[in]  bytes_to_eat    int
        * @return      int
        */
        int _consume(const int &bytes_to_eat);

        int _ex_consume(const int& bytes_to_eat);

        std::weak_ptr<base_io> _gio_ptr;     ///< gio pointer
        std::vector<base_note> _notes;       ///< cummative notes message/warning/error
        std::string _fname;                ///< decoded file
        std::string _class;                ///< std::string for reporting
        bool _initialized;            ///< if initialized
        bool _gnss;                   ///< if gnss definition is requested
        int _out_len;                 ///< [min] encoder data batch length
        float _out_smp;               ///< [sec] encoder data batch sample
        base_time _out_epo;             ///< encoder refrence epoch
        std::string _version;              ///< format version
        std::string _initver;              ///< format initial version
        std::map<std::string, base_data *> _data; ///< data pointer
        base_log _spdlog;             ///< spdlog pointer
        hwa_set::set_base* _set;
        char *_buffer;                ///< class buffer
        int _buffsz;                  ///< size of buffer
        int _endpos;                  ///< after last position
        int _begpos;                  ///< begin position of useful data
        int _irc;                     ///< IRC code
        bool _close_with_warning;     ///< close with warnings (desctructor)

        base_coder_char_buffer _decode_buffer;

        //    settings
        base_time _beg;     ///< default beg time
        base_time _end;     ///< default end time
        double _int;      ///< default interval
        int _scl;         ///< default scaling for decimation (if >= 1Hz)
        std::set<std::string> _sys; ///< default systems
        std::set<std::string> _rec; ///< default sites/receivers

        //    ENCODING
        int _fill_buffer(char *buff, int sz);
        std::stringstream _ss;
        long _ss_position;
        std::string _pgm;
        bool _hdr;

#ifdef BMUTEX
        boost::mutex _mutex; ///< buffer mutex
#else
        base_mutex _mutex; ///< buffer mutex
#endif
    private:
    };

} // namespace

#endif
