#ifndef hwa_gnss_coder_ionex_H
#define hwa_gnss_coder_ionex_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_ionex.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decode/encode TEC grid products in IONEX format
    */
    class gnss_coder_ionex : public gnss_base_coder
    {
    public:
        /**
        * @brief constructor.
        * @param[in]  s        std::setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        gnss_coder_ionex(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_ionex();

        /**
        * @brief decode header
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return 
            @retval int consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief decode body
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return 
            @retval int consume size of header decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

    protected:
        gnss_data_ionex_head _ionex_hd; ///< IONEX head info
        std::string _data_type;       ///< data type
        base_time _crt_time;       ///< current time
        int _ihgt, _ilat;        ///< height in i frame ,latitude in i frame
        int _ilon, _nlon;        ///< longitude in i frame ,longitude in nframe

    private:
    };

}

#endif // !ION_H
