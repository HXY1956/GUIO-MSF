#ifndef hwa_gnss_coder_dvpteph405_H
#define hwa_gnss_coder_dvpteph405_H

#include "hwa_gnss_coder_BASE.h"
#include "hwa_set_base.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    class gnss_coder_dvpteph405 : public gnss_base_coder
    {
    public:
        /** 
         * @brief default constructor. 
         * 
         * @param[in]  s        setbase control
         * @param[in]  version  version of the gcoder
         * @param[in]  sz       size of the buffer
         */
        explicit gnss_coder_dvpteph405(set_base *s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** 
         * @brief default destructor. 
         */
        virtual ~gnss_coder_dvpteph405(){};

        /** 
         * @brief decode the header of the jpleph_de405 data file. 
         * @param[in]  buff        buffer of the data
         * @param[in]  bufLen      buffer size of the data
         * @param[in]  errmsg      error message of the data decoding
         * @return consume size of header decoding
            @retval >=0 consume size of header decoding
            @retval <0  finish reading
         */
        virtual int decode_head(char *buff, int bufLen, std::vector<std::string> &errmsg);

        /** 
         * @brief decode the data body of the jpleph_de405 data file. 
         * 
         * @param[in]  buff        buffer of the data
         * @param[in]  bufLen      buffer size of the data
         * @param[in]  errmsg      error message of the data decoding
         * @return consume size for data body decoding
            @retval >=0 consume size of body decoding
             @retval <0  finish reading
         */
        virtual int decode_data(char *buff, int bufLen, int &cnt, std::vector<std::string> &errmsg);

    protected:
    private:
        int _flag;       ///< flag for decoder
        int _version_de; ///< JPL file version
        base_time _beg_de; ///< start time for JPL data
        base_time _end_de; ///< end time for JPL data
        double _days;    ///< Sampling interval of data(unit: day)
        int _ipt[3][13]; ///< index of planets for calculating position and speed
    };
}

#endif // !DVPTEPH405_H
