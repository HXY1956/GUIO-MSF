#ifndef hwa_gnss_coder_ambflag_H
#define hwa_gnss_coder_ambflag_H

#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_all_ambflag.h"

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for decode/encode ambflag file
    */
    class gnss_coder_ambflag : public gnss_base_coder
    {

    public:
        /**
        * @brief constructor.
        * @param[in]  s        setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        explicit gnss_coder_ambflag(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_ambflag();

        /**
        * @brief decode header of ambflag file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief decode data body of ambflag file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return consume size for data body decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of ambflag file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of header encoding
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode data body of ambflag file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size for data body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

    protected:
        gnss_data_ambflag_head _ambflag_head;   ///< map for storaging ambflag data header
        gnss_data_ambflag_data _gnss_data_ambflag_data; ///< map for storaging ambflag data body
        int _max_epo;               ///< max epoch begin
    private:
    };

} // namespace

#endif
