#include "hwa_base_common.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_file.h"

using namespace std;

namespace hwa_base
{
    // constructor
    // ----------
    base_file::base_file(base_log spdlog)
        : base_io(spdlog),
          _irc(0),
          _gzip(false)
    {

        _file = 0;
        _zfile = 0;
        _size = FILEBUF_SIZE;
    }

    // destructor
    // ----------
    base_file::~base_file()
    {

        reset();
    }

    // set path
    // ----------
    int base_file::path(string str)
    {

        if (str == "")
            return -1;

        size_t idx1 = 0, idx2 = 0;
        string prefix = HWA_FILE_PREFIX;

        // check if file
        idx2 = str.find(prefix);
        if (idx2 == string::npos)
        {
            str = prefix + str;
            idx2 = 0;
        }

        // file path
        idx1 = idx2 + prefix.length();
        idx2 = str.length();
        if (idx2 != string::npos && idx2 > idx1)
        {

            string name = str.substr(idx1, idx2 - idx1);
            _set_gzip(name);

            if (_gzip && _zfile == 0)
                _zfile = new base_iofz(name.c_str());
            else if (_file == 0)
                _file = new base_iof(name.c_str());
            else
                return -1;

            if (_gzip && _zfile)
                _zfile->mask(name);
            else if (_file)
                _file->mask(name);
            else
            {
                cerr << "cannot assign file to path!\n";
                _irc++;
                return -1;
            }

            ostringstream ltmp;
            ltmp << "File: " << int(idx1) << ":" << int(idx2) << " = " << name;
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, ltmp.str());
        }
        else
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "warning: path does not contain file://dir/name  [check file://]");
            return -1;
        }

        base_io::path(str);

        return 1;
    }

    // get path
    // ----------
    string base_file::path()
    {
        return HWA_FILE_PREFIX + mask();
    }

    // get name
    // ----------
    string base_file::name()
    {

        return mask();
    }

    // irc
    // ----------
    int base_file::irc() const
    {
        if (_gzip && _zfile)
            return (_zfile->irc() + _irc);
        else if (_file)
            return (_file->irc() + _irc);

        return _irc;
    }

    // eof
    // ----------
    bool base_file::eof()
    {
        if (_gzip && _zfile)
            return _zfile->eof();
        else if (_file)
            return _file->eof();

        return true;
    }

    // mask
    // ----------
    string base_file::mask()
    {
        if (_gzip && _zfile)
            return _zfile->mask();
        else if (_file)
            return _file->mask();

        return "";
    }

    // init read function (read header)
    // ----------
    int base_file::init_read()
    {

        if (!_coder)
        {
            return 0;
        }

        char *loc_buff = new char[FILEHDR_SIZE];

#ifdef DEBUG
        std::cout << "HEADER START " << mask() << "\n";
        std::cout.flush();
#endif

        int nbytes = 0;
        vector<string> errmsg;
        //  while( ( nbytes = _gio_read( loc_buff, FILEHDR_SIZE ) ) >= 0 && _stop != 1 ){ // JD 2014-10-25
        while ((nbytes = _gio_read(loc_buff, FILEHDR_SIZE)) > 0 && _stop != 1)
        {

#ifdef DEBUG
            std::cout << "GIO_HEAD_READ :";
            for (int i = 0; i < nbytes; i++)
            {
                std::cout << loc_buff[i];
            }
            std::cout << ":END\n";
#endif

            if (_coder->decode_head(loc_buff, nbytes, errmsg) < 0)
                break;
        }

        if (nbytes < 0)
        { // header not completed properly ?
            ++_irc;
            if (_coder)
                _coder->mesg(GERROR, "Incomplete header identified.");
        }

#ifdef DEBUG
        std::cout << "HEADER END " << mask() << "\n";
#endif

        delete[] loc_buff;
        return 1;
    }

    // init write function (write header)
    // ----------
    int base_file::init_write()
    {

        if (!_coder)
        {
            return 0;
        }

        char *loc_buff = new char[FILEHDR_SIZE];

#ifdef DEBUG
        std::cout << "HEADER START " << mask() << endl;
        std::cout.flush();
#endif
        vector<string> errmsg;

        int nbytes = 0;
        do
        {
            if ((nbytes = _coder->encode_head(loc_buff, FILEHDR_SIZE, errmsg)) < 0)
                break;
        } while ((nbytes > 0) && (_gio_write(loc_buff, nbytes) > 0) && _stop != 1);

#ifdef DEBUG
        std::cout << "HEADER END " << mask() << endl;
        std::cout.flush();
#endif

        delete[] loc_buff;

        return 1;
    }

    // reset
    // ----------
    void base_file::reset()
    {

        if (_file)
        {
            delete _file;
            _file = 0;
        }
        if (_zfile)
        {
            delete _zfile;
            _zfile = 0;
        }
    }

    // read data
    // ----------
    int base_file::_gio_read(char *buff, int size)
    {

        if (mask() == "")
            return -1;
        int nbytes = this->_read(buff, size);

        if (nbytes == 0 && this->eof())
            return -1;
        return nbytes;

        //  int nbytes = 0
        //  nbytes = this->_read(buff,size);
        //  if( this->eof() || nbytes == -1 ){
        //    if( nbytes == 0 ) nbytes = -1;
        //    }
        //  }
        //  return nbytes;
    }

    // send data
    // ----------
    int base_file::_gio_write(const char *buff, int size)
    {

        if (mask() == "")
            return 0;

        this->_write(buff, size);

        return size;
    }

    // common function for file close
    // ----------
    int base_file::_stop_common()
    {

        return base_io::_stop_common();
    }

    // compressed or not ?
    // ----------
    void base_file::_set_gzip(string name)
    {

        if ( // name.compare(name.length()-2,name.length(),".Z")  == 0 || ///  NOT SUPPORTED
            name.compare(name.length() - 3, name.length(), ".gz") == 0)

            _gzip = true;
        else
            _gzip = false;
    }

    // ----------
    int base_file::_read(char *b, int s)
    {

        if (_gzip && _zfile)
            return _zfile->read(b, s);
        else if (_file)
            return _file->read(b, s);

        return -1;
    }

    // ----------
    int base_file::_write(const char *b, int s)
    {

        if (_gzip && _zfile)
            return _zfile->write(b, s);
        else if (_file)
            return _file->write(b, s);

        return -1;
    }

} // namespace
