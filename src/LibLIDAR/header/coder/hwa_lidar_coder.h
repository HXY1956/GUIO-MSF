#ifndef hwa_lidar_coder_h
#define hwa_lidar_coder_h

/**
* lidarfile is the lidar observation file,including TimeStamp and corresponding LIDARPATH
*
* An example of th lidar file( No File Head )
* @verbatim
    ===================================================================================
        TimeStamp        Name
        15.15           000001.txt/000001
        15.25           000002.txt/000002

        ...
        120.55          000100.txt/000100
    ===================================================================================
  @endverbatim
*/

#include "hwa_set_base.h"
#include "hwa_base_coder.h"
#include "hwa_base_string.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;
using namespace hwa_set;

namespace hwa_lidar {
    class lidar_coder : public base_coder
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  s        setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        explicit lidar_coder(set_base* s, string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /** @brief destructor. */
        ~lidar_coder() {}

        /**
        * @brief decode the header of the LIDAR data file.
        * lidar file doesn't include head block
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval >=0 consume size of header decoding
            @retval <0  finish reading
        */
        virtual int decode_head(char* buff, int sz, vector<string>& errmsg);

        /**
        * @brief decode the data body of the LIDAR data file.
        *
        * decode data body of LIDAR file, all the data read will store in the lidar_data
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval >=0 consume size of header decoding
            @retval <0  finish reading
        */
        virtual int decode_data(char* buff, int sz, int& cnt, vector<string>& errmsg);

    private:
        double _ts = 0;                ///< data interval
    };
}


#endif
