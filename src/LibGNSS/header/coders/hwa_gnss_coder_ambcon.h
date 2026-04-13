#ifndef hwa_gnss_coder_ambcon_H
#define hwa_gnss_coder_ambcon_H

#include "hwa_set_base.h"
#include "hwa_gnss_data_ambcon.h"
#include "hwa_gnss_coder_BASE.h"

namespace hwa_gnss
{
    /**
    *@brief       Class for decode/encode  amb constraint file
    */
    class gnss_coder_ambcon : virtual public gnss_base_coder
    {
    public:
        /**
        * @brief constructor.
        * @param[in]  s        std::setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        gnss_coder_ambcon(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_ambcon();

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
        virtual int encode_head(char* buff, int sz, std::vector<std::string>& errmsg) override;

        /**
        * @brief encode header of ambcon file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval int size of data  body encoding
        */
        virtual int encode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg) override;

    private:
        std::set<std::string> _sats;                    ///< satellite list
        std::set<std::string> _sites;                   ///< sites list
        std::string _mode = "D";                   ///< fix mode
        std::set<AMB_ID> _amb_ids;                 ///< list of ambiguity ids
        std::string _block;                        ///< satelilte block type
        AMB_TYPE _amb_type = AMB_TYPE::UNDEF; ///< ambiguity type
        int _idx_save = 0;                    ///< whether to save index
    };
}
#endif //AMBCON_H