/**
* @file     blq.h
* @brief    The base class used to decode ??? file information.
*/
#ifndef hwa_gnss_coder_blq_H
#define hwa_gnss_coder_blq_H

#include "hwa_set_base.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_otl.h"
#include "hwa_gnss_all_otl.h"
#include "hwa_base_time.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decoding the ??? data
    */
    class gnss_coder_blq : public gnss_base_coder
    {
    public:
        /**
        * @brief default constructor.
        *
        * @param[in]  s            setbase control
        * @param[in]  version    version of the gcoder
        * @param[in]  sz        size of the buffer
        */
        explicit gnss_coder_blq(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_blq(){};

        /**
        * @brief decode the header of the ??? data file.
        *
        * The function is used for decoding the head of ??? file.
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
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg);

        /**
        * @brief decode the data body of the ??? data file.
        *
        * decode data body of ??? file, all the data read will store in the gotl???
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  bufLen      buffer size of the data
        * @param[out] cnt          number of line
        * @param[out] errmsg      error message of the data decoding
        * @return
        @retval >=0 consume size of body decoding
        @retval <0  finish reading
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg);

    protected:
        double _lat;     ///< latitude
        double _lon;     ///< longitude
        std::string _site;    ///< site
        Matrix _blqdata; ///< blqdata

    private:
    };

} // namespace

#endif
