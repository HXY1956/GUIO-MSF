#ifndef hwa_base_io_bigf_h
#define hwa_base_io_bigf_h

#include "hwa_base_iof.h"

#ifndef MAX_BUFFER_SIZE
#define MAX_BUFFER_SIZE 204800000
#endif

namespace hwa_base
{

    class base_io_bigf : public base_iof
    {
    public:
        /**
         * @brief Construct a new t giobigf
         * @param[in]  mask          mask
         * @param[in]  buffer_size   size of buffer
         */
        base_io_bigf(std::string mask = "", int buffer_size = 1024 * 1000 * 10);
        /**
         * @brief Destroy the t giobigf object
         */
        virtual ~base_io_bigf();
        /**
         * @brief write big file
         * @param[in]  buff      buffer
         * @param[in]  size      size of buffer
         * @return int 
         */
        int write(const char *buff, int size) override;
        /**
         * @brief Clear the buffer area
         * @return int 
         */
        int flush();

        const int buffer_size; ///< size of buffer

    private:
        char *_buffer; ///< buffer
        int _current;  ///< current location
    };

    class base_io_READTEMP
    {
    public:
        /**
         * @brief Construct a new t greadtemp object
         * @param[in]  tempfilename  tempfile's name
         */
        base_io_READTEMP(std::string tempfilename);
        /**
         * @brief Destroy the t greadtemp object
         */
        ~base_io_READTEMP();
        /**
         * @brief 
         * @param[in]  dst       buffer
         * @param[in]  size      size of dst
         */
        void read(char *dst, int size);
        /**
         * @brief Find a certain location from the current location based on length
         * @param[in]  lensize   length
         */
        void seekg_from_cur(int lensize);

    protected:
        /**
         * @brief get certain file size
         * @param[in]  file              FILE class
         * @param[in]  file_byte_size    file byte size
         * @return
         *         @retval true can get the file byte size
         *         @retval false can not get the file byte size
         */
        static bool _get_filesize(FILE *file, int64_t *file_byte_size);

    private:
        FILE *_tmpfile;    ///< tmpfile class
        char *_buffer;     ///< buffer
        int32_t _current;  ///< current location
        int32_t _endpos;   ///< end position
        int64_t _filesize; ///< file byte size
    };

}

#endif // !GIOBIGF_H