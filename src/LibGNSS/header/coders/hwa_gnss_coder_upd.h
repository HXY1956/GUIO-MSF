#ifndef hwa_gnss_coder_upd_H
#define hwa_gnss_coder_upd_H

#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_upd.h"

using namespace std;
using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    class gnss_coder_upd : public gnss_base_coder
    {

    public:
        /**
        * @brief constructor.
        * @param[in]  s        setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        explicit gnss_coder_upd(set_base *s, string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief default destructor. */
        virtual ~gnss_coder_upd();

        /**
        * @brief decode header of upd file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, vector<string> &errmsg) override;

        /**
        * @brief decode data of upd file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return consume size of header decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg) override;

        /**
        * @brief encode header of upd file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of header encoding
        */
        virtual int encode_head(char *buff, int sz, vector<string> &errmsg) override;

        /**
        * @brief encode data of upd file
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of data body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg) override;

    protected:
        UPDTYPE _updtype; ///< upd mode EWL,WL and NL.
        base_time _epoch;   ///< current epoch
    private:
    };

} // namespace

#endif
