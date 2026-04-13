#ifndef hwa_gnss_coder_recover_H
#define hwa_gnss_coder_recover_H

#include "hwa_base_time.h"
#include "hwa_gnss_coder_BASE.h"
#include "hwa_gnss_data_recover.h"
#include "hwa_gnss_all_recover.h"

using namespace hwa_base;

namespace hwa_gnss
{

    class gnss_coder_resfile : public gnss_base_coder
    {
    public:
        const static std::string END_OF_HEADER;
        const static std::string TIME_HEADER;
        const static std::string SIGMA_HEADER;
        const static std::string SAT_HEADER;
        const static std::string SITE_HEADER;
        const static std::string OBS_DATA;
        const static std::string PAR_DATA;

    public:
        explicit gnss_coder_resfile(hwa_set::set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        virtual ~gnss_coder_resfile();

        /**
        * @brief decode header of resfile 
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return consume size of header decoding
        */
        virtual int decode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief decode data of  resfile
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return consume size of header decoding
        */
        virtual int decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode header of  resfile
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of header encoding
        */
        virtual int encode_head(char *buff, int sz, std::vector<std::string> &errmsg) override;

        /**
        * @brief encode data of  resfile
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return  size of data body encoding
        */
        virtual int encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg) override;

    protected:
        /**
         * @brief add a data
         * @param[in]  id        id_type
         * @param[in]  data      ptr of data
         */
        void _add_data(const std::string &id, base_data *data) override;

        gnss_all_recover *_recover_data; ///< ptr of recover data
    };

    gnss_data_recover_par strline2recover_par(base_log spdlog, const std::string &line);
    gnss_data_recover_equation strline2recover_equation(base_log spdlog, const std::string &line);

}

#endif // !RECOVER_H
