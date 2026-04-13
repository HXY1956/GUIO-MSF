/**
*
* @file     biabernese.h
* @brief    The base class used to decode biabernese file information.
*/
#ifndef hwa_gnss_CDOER_BIABERNESE_H
#define hwa_gnss_CDOER_BIABERNESE_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_base_time.h"
#include "hwa_gnss_all_Bias.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decoding the biabernese data derive from base_coder
    */
    class gnss_coder_biabernese : public gnss_base_coder
    {
    public:
        /**
        * @brief default constructor.
        *
        * @param[in]  s            setbase control
        * @param[in]  version    version of the gcoder
        * @param[in]  sz        size of the buffer
        * @param[in]  id        string for reporting
        */
        explicit gnss_coder_biabernese(set_base *s, std::string version, int sz = DEFAULT_BUFFER_SIZE, std::string id = "biabernese");

        /** @brief default destructor. */
        virtual ~gnss_coder_biabernese(){};

        /**
        * @brief decode the header of the biabernese data file.
        *
        * The function is used for decoding the head of biabernese file.
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
        * @brief decode the data body of the biabernese data file.
        *
        * decode data body of biabernese file, all the data read will store in the gbias???
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

    protected:
        /**
        * @brief decode the data body of the ??? data file.
        *
        * decode data body of ??? file, all the data read will store in the gbias???
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz        buffer size of the data
        * @param[out] cnt        number of line
        * @param[out] errmsg    error message of the data decoding
        * @return
        @retval >=0 consume size of body decoding
        @retval <0  finish reading
        */
        int _decode_data_CODE(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg); // add by glfeng

        /**
        * @brief decode the data body of the ??? data file.
        *
        * decode data body of ??? file, all the data read will store in the gbias???
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz        buffer size of the data
        * @param[out] cnt        number of line
        * @param[out] errmsg    error message of the data decoding
        * @return
        @retval >=0 consume size of body decoding
        @retval <0  finish reading
        */
        int _decode_data_sinex(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg); // add by glfeng

        /**
        * @brief decode the data body of the ??? data file.
        *
        * decode data body of ??? file, all the data read will store in the gbias???
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz        buffer size of the data
        * @param[out] cnt        number of line
        * @param[out] errmsg    error message of the data decoding
        * @return
        @retval >=0 consume size of body decoding
        @retval <0  finish reading
        */
        int _decode_data_sinex_0(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg); // add by jqwu

        gnss_all_bias *_allbias; ///< ???
        base_time _beg;         ///< add by glfeng
        base_time _end;         ///< add by glfeng

        std::string _ac;                ///< ???
        double _version;           ///< 1.00 or 0.01
        bool _is_bias = false;     ///< sinex bias file
        bool _is_absolute = false; ///< relative/absolute bias
    };

} // namespace

#endif
