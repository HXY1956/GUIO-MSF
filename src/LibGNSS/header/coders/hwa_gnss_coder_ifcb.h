#ifndef hwa_gnss_coder_ifcb_H
#define hwa_gnss_coder_ifcb_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_ifcb.h"

using namespace hwa_set;

namespace hwa_gnss
{
    class gnss_coder_ifcb : public gnss_base_coder
    {

    public:
        /**
        * @brief constructor.
        * @param[in]  s        std::setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        gnss_coder_ifcb(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_ifcb();

        /**
        * @brief decode header of ifcb file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return 
            @retval int consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief decode data of ifcb file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return 
            @retval int consume size of header decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of ifcb file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  
            @retval int size of header encoding
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode data of ifcb file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  
            @retval int size of data body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

    protected:
        std::string _ifcbmode; ///< ifcb mode EWL,WL and NL.
        base_time _epoch;   ///< current epoch
    private:
    };

} // namespace

#endif
