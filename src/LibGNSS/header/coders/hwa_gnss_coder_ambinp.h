#ifndef hwa_gnss_coder_ambinp_H
#define hwa_gnss_coder_ambinp_H

#include "hwa_set_base.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_ambinp.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /**
    *@brief       Class for decode/encode ambinp file
    */
    class gnss_coder_ambinp : public gnss_base_coder
    {
    public:
        /**
        * @brief constructor.
        * @param[in]  s        std::setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        gnss_coder_ambinp(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_ambinp();

        /**
        * @brief decode header of ambinp file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return 
        @retval int consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief decode body of ambinp file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return 
            @retval int consume size of header decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of ambinp file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  
            @retval int size of header encoding
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of ambinp file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  
            @retval int size of data  body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

    protected:
        gnss_data_ambinp_head *_ambinp_hd; ///< ambiguity head info
        int _flag;                  ///< =1:read ambigulity;=2:read neq
    private:
    };

}

#endif //AMBINP_H
