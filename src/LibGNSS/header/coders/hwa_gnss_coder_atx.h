/**
*
* @file     atx.h
* @brief    The base class used to decode Antenna Exchange format (ATX) file information.
*/
#ifndef hwa_gnss_coder_atx_H
#define hwa_gnss_coder_atx_H

#include <vector>
#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decoding the atx data derive from base_coder
    */
    class gnss_coder_atx : public gnss_base_coder
    {
    public:
        /**
        * @brief default constructor.
        *
        * @param[in]  s        setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        explicit gnss_coder_atx(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_atx(){};

        /**
        * @brief decode the header of the atx data file.
        *
        * The function is used for decoding the head of atx file.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in decoding.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  bufLen      buffer size of the data
        * @param[out] errmsg      error message of the data decoding
        * @return
        @retval >=0 consume size of header decoding
        @retval <0  finish reading
        */
        virtual int decode_head(char *buff, int bufLen, std::vector<std::string> &errmsg);

        /**
        * @brief decode the data body of the atx data file.
        *
        * decode data body of atx file, all the data read will store in the gpcv???
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  bufLen      buffer size of the data
        * @param[out] cnt          number of line
        * @param[out] errmsg      error message of the data decoding
        * @return
        @retval >=0 consume size of body decoding
        @retval <0  finish reading
        */
        virtual int decode_data(char *buff, int bufLen, int &cnt, std::vector<std::string> &errmsg);

        /**
        * @brief encode the header of the atx data file.
        *
        * The function is used for encoding the head of atx file.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in encoding.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[out] errmsg      error message of the data encoding
        * @return
        @retval >=0 consume size of header encoding
        @retval <0  finish reading
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
        * @brief encode the data body of the atx data file.
        *
        * The function is used for encoding the data body of atx file.
        * pay attention to the buff and the size of buff which may cause some trouble when
        using the wrong value in encoding.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[out] cnt         number of line
        * @param[out] errmsg      error message of the data encoding
        * @return
        @retval >=0 consume size of body encoding
        @retval <0  finish reading
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

    protected:
    private:
    };

} // namespace

#endif
