/**
* @brief    decode and enconde obx constraint file(grtyyyydoy.obx)
*             yyyy  ---- year
*              ddd  ---- doy
*/

#ifndef hwa_gnss_coder_OBX_H
#define hwa_gnss_coder_OBX_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_obx.h"
#include "hwa_gnss_all_obx.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decode/encode  obx constraint file
    */
    class gnss_data_obx : public gnss_base_coder
    {
    public:
        /**
        * @brief constructor.
        * @param[in]  s        std::setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        gnss_data_obx(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_data_obx();

        /**
        * @brief decode header of ambcon file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval int consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief decode body of ambcon file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval int consume size of header decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of ambcon file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval int size of header encoding
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of ambcon file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval int size of data  body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

    protected:
        base_time _beg;      ///< begin time
        base_time _end;      ///< end time
        double _intv;      ///< time interval
        int _numb;         ///< number
        std::set<std::string> _sats; ///< satellite list
        std::set<std::string> _recs; ///< receiver list

    private:
    };

}

#endif // !ION_H
