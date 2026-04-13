#ifndef hwa_base_file_h
#define hwa_base_file_h

#include "hwa_base_io.h"
#include "hwa_base_iofz.h"

// special buffer size for file reading
// --> must be bellow gcoder maximum limit !
#define FILEBUF_SIZE 20480
#define FILEHDR_SIZE 48

namespace hwa_base
{

    /** @brief class for base_file derive from base_io. */
    class base_file : public base_io
    {

    public:
        /** @brief default constructor. */
        base_file(base_log spdlog);

        /** @brief default destructor. */
        virtual ~base_file();

        /** @brief init write/read. */
        virtual int init_write();
        virtual int init_read();

        /** @brief get irc status. */
        virtual int irc() const;

        /** @brief integrate gzip/ascii. */
        virtual bool eof();

        /** @brief integrate gzip/ascii. */
        virtual std::string mask();

        virtual bool compressed() { return _gzip; }

        /**
        * @brief set file://dir/name. 
        * @param[in]    str        file path 
        * @return 
        *    @retval =-1 false
        *    @retval =1    true
        */
        virtual int path(std::string str);
        virtual std::string path();
        virtual std::string name();

        /** @brief reset path. */
        virtual void reset();

    protected:
        /**
        * @brief send data.
        * @param[in]    buff    buffer of the data
        * @param[in]    size    buffer size of the data
        * @return 
        *    @retval >0    buffer size of the data
        *    @retval <=0    fail
        */
        virtual int _gio_write(const char *buff, int size);

        /** @brief read data.
        * @param[in]    buff    buffer of the data
        * @param[in]    size    buffer size of the data
        * @return 
            @retval >0    number of bytes read 
            @retval <=0    fail
        */
        virtual int _gio_read(char *buff, int size);

        /**
        * @brief common function for file close. 
        * @return        running status 
        */
        virtual int _stop_common(); ///< reimplementaion

        /**
        * @brief compressed or not ? 
        * @param[in]    name    file name
        */
        virtual void _set_gzip(std::string name); ///< compressed ?

        /**
        * @brief read. 
        * @param[in]    b    buffer of the data
        * @param[in]    s    buffer size of the data
        * @return
            @retval <0    fail
        */
        virtual int _read(char *b, int s); ///< integrate gzip/ascii

        /**
        * @brief write.
        * @param[in]    b    buffer of the data
        * @param[in]    s    buffer size of the data
        * @return
            @retval <0    fail
        */
        virtual int _write(const char *b, int s); ///< integrate gzip/ascii

        int _irc;        ///< irc
        bool _gzip;      ///< compressed
        base_iof *_file;   ///< ascii file
        base_iofz *_zfile; ///< gzip  file

    private:
    };

} // namespace

#endif
