#include "hwa_base_coder.h"

namespace hwa_base
{

    base_coder::base_coder()
    {
    }

    // constructor
    // ----------
    base_coder::base_coder(hwa_set::set_base* s, std::string version, int sz, std::string id)
        : _class(id),
          _set(s),
          _initialized(false),
          _gnss(true),
          _initver(version),
          _close_with_warning(true)
    {
        _init();

        // after init!
        //  _version = version;
        _buffsz = sz;
        _spdlog = nullptr;
        _irc = 0;
        _hdr = false;

        _ss_position = 0;

        _out_smp = 0;
        _out_len = 0;
        _out_epo = LAST_TIME;

        // use malloc instead of new due to realocate function!
        // ====================================================
        _buffer = (char *)malloc((_buffsz + 1) * sizeof(char)); // due to realocate function!
        if (s)
            _gset(s);
    }
    base_coder::base_coder(base_time beg, base_time end, hwa_set::set_base* s, std::string version, int sz, std::string id)
        : _class(id),
          _set(s),
          _initialized(false),
          _gnss(true),
          _initver(version),
          _close_with_warning(true),
          beg_epoch(beg),
          end_epoch(end)
    {
        _init();

        // after init!
        //  _version = version;
        _buffsz = sz;
        _spdlog = nullptr;
        _irc = 0;
        _hdr = false;

        _ss_position = 0;

        _out_smp = 0;
        _out_len = 0;
        _out_epo = LAST_TIME;

        // use malloc instead of new due to realocate function!
        // ====================================================
        _buffer = (char *)malloc((_buffsz + 1) * sizeof(char)); // due to realocate function!
        //  if( data != 0 ) _data[ data_id ] = data;
        if (s)
            _gset(s);
    }

    base_coder::base_coder(hwa_set::set_base* s, int sz)
        : _set(s), 
          _initialized(false),
          _gnss(false),
          _close_with_warning(true)
    {
        _init();

        // after init!
        //  _version = version;
        _buffsz = sz;
        _spdlog = nullptr;
        _irc = 0;
        _hdr = false;

        _ss_position = 0;

        _out_smp = 0;
        _out_len = 0;
        _out_epo = LAST_TIME;

        // use malloc instead of new due to realocate function!
        // ====================================================
        _buffer = (char *)malloc((_buffsz + 1) * sizeof(char)); // due to realocate function!

        if (s)
            _gset(s);
    }

    /* ----------
     * destructor
     */
    base_coder::~base_coder()
    {
        // use free instead of delete due to realocate function!
        if (_buffer)
        {
            free(_buffer);
            _buffer = NULL; // modified by zhanglong
        }

        if (_close_with_warning)
        {
            std::sort(_notes.begin(), _notes.end());
            for (auto it = _notes.begin(); it != _notes.end(); ++it)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, it->str() + " .. " + base_name(_fname));
            }
            //    if( _log ){ _log->comment(-1, "gcoder", _fname+"  "+it->str()); }
        }
    }

    void base_coder::spdlog(std::shared_ptr<logger> spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }
    // set path
    // ----------
    void base_coder::path(const std::string &s)
    {
        std::string tmp(s);
        base_type_conv::substitute(tmp, HWA_FILE_PREFIX, "");
        _fname = tmp;
    }

    /* ----------
     * add message
     */
    void base_coder::mesg(const base_note_type &n, const std::string &s)
    {
        base_note m(n, "gcoder", s);

        //if( find(_notes.begin(), _notes.end(), m) == _notes.end() ){
        //     _notes.push_back(m);

        // eliminate repeating messages
        bool exist = false;
        std::vector<base_note>::const_iterator it;
        for (it = _notes.begin(); it != _notes.end(); ++it)
        {
            if (*it == m)
            {
                exist = true;
            }
        }

        if (!exist)
            _notes.push_back(m);
    }

    /* ----------
     * get message
     */
    const std::vector<base_note> &base_coder::mesg() const
    {
        return _notes;
    }

    /* ----------
     * initialize
     */
    void base_coder::_init()
    {

        _version = _initver;
        _endpos = 0;
    }

    /* ----------
     * clear
     */
    void base_coder::clear()
    {

        // use free instead of delete due to realocate function!
        // modified by zhanglong
        if (_buffer)
        {
            free(_buffer);
            _buffer = NULL;
        }

        _init();

        // use malloc instead of new due to realocate function!
        // ====================================================
        _buffer = (char *)malloc((_buffsz + 1) * sizeof(char)); // due to realocate function!
    }

    /* ----------
     * get single line from the buffer
     */
    int base_coder::_getline(std::string &str, int from_pos)
    {
        return _decode_buffer.getline(str, from_pos);
    }

    int base_coder::_getbuffer(const char *&buff)
    {
        return _decode_buffer.getbuffer(buff);
    }

    /* ----------
     * cummulate buffer
     */
    int base_coder::_add2buffer(char *buff, int sz)
    {

        return _decode_buffer.add(buff, sz);
    }

    /* ----------
     * remove from buffer
     */
    int base_coder::_consume(const int &bytes_to_eat)
    {

        return _decode_buffer.consume(bytes_to_eat);
    }

    int base_coder::_ex_consume(const int& bytes_to_eat)
    {

        return _decode_buffer.ex_consume(bytes_to_eat);
    }

    // add specific data structure to the coder
    // ----------
    int base_coder::add_data(const std::string &data_id, base_data *data)
    {

        if (data_id.empty() || data == 0)
            return -1;

        std::map<std::string, base_data *>::iterator it = _data.find(data_id);
        if (it != _data.end())
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "warning: structure " + data_id + " already exists !");
            return -1;
        }

#ifdef DEBUG
        std::cout << " gcoder: adding data structure: " << data_id << std::endl;
        std::cout.flush();
#endif
        _data[data_id] = data;

        // can be used to directly setup data containers in individual gcoders
        _add_data(data_id, data);

        return 1;
    }

    // remove specific data structure from the coder
    // ----------
    int base_coder::_fill_buffer(char *buff, int sz)
    {

        _ss.seekg(0, _ss.end);
        long len = (long)_ss.tellg();
        len = len - _ss_position;
        _ss.seekg(_ss_position, std::ios_base::beg);

        int size = (len < sz) ? len : sz;

        _ss.clear();
        _ss.read(buff, size);

        std::string tmp = buff;
        tmp = tmp.substr(0, size);
        size_t ifirst = 0;
        if ((ifirst = tmp.find_last_of(crlf)) != std::string::npos)
        {
            tmp = tmp.substr(0, ifirst + 1);
            size = tmp.size();
            _ss.seekg(_ss_position + size, std::ios_base::beg);
        }

        if (_ss.fail())
        {
            std::cout << "HEAD: any problem ?\n";
        }
        else if (_ss.gcount() == 0)
        {
            //  std::cout << "HEAD: finished " << sz << " " << len << " " << _ss_position << " " << ss.gcount() << std::endl;
            _ss_position = 0;
            _ss.str("");
            _ss.clear();
        }
        else
        {
            //  std::cout << "HEAD: read     " << sz << " " << len << " " << _ss_position << " "  << _ss.gcount()<< std::endl;
            _ss_position += size; // _ss.gcount();
        }

        return size;
    }

    // sampling filter for epochs (return true if the epoch fits sampling)
    // ----------
    bool base_coder::_filter_epoch(const base_time &epo)
    {
        if (time_sync(epo, _int, _scl, _spdlog))
        {
            //   std::cerr << epo.str_ymdhms("fitting: ")+base_type_conv::dbl2str(epo.dsec()) << std::endl;
            return true;
        }
        else if (time_sync(epo.diff(_beg), _int, _scl, _spdlog))
        { //Added by Wei Zhang
            return true;
        }
        return false;
    }  
} // namespace