#ifndef hwa_ins_coder_h
#define hwa_ins_coder_h
#include "hwa_set_ins.h"
#include "hwa_base_coder.h"
#include "hwa_base_roughcoder.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_ins
{
    struct imu_batch {
        Triple wm;
        Triple vm;
        imu_batch(Triple _wm, Triple _vm) {
            wm = _wm;
            vm = _vm;
        }
    };
    using imu_map = std::map<double, imu_batch>;
    class ins_coder : virtual public hwa_base::base_coder
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  s        setbase control
        * @param[in]  version  version of the gcoder
        * @param[in]  sz       size of the buffer
        */
        explicit ins_coder(set_base* s, std::string version = "", int sz = DEFAULT_BUFFER_SIZE);

        /**
        * @brief destructor.
        */
        ~ins_coder() {}


        /**
        * @brief decode the header of the IMU data file.
        * imu file doesn't include head block
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval >=0 consume size of header decoding
            @retval <0  finish reading
        */
        virtual int decode_head(char* buff, int sz, std::vector<std::string>& errmsg);

        /**
        * @brief decode the data body of the IMU data file.
        * the main entry of decode IMU file and call the corresponding sub function according to 'order' in xml file
        *
        * decode data body of IMU file, all the data read will store in the ins_data
        * imu data final result -> t(s) gyro(rad) acce(m/s)
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  cnt         size of lines successfully decoded
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval >=0 consume size of data decoding
            @retval <0  finish reading
        */
        virtual int decode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);

        /**
        * @brief decode the data body of the IMU data file with unicore format.
        * order="unicore"
        * decode data body of IMU file, all the data read will store in the ins_data
        *
        * @param[in]  line        line in imu file
        * @param[in]  t              time of data epoch
        * @param[out] v           imu data(acce & gyro)

        * @return
            @retval 1   success
            @retval -1  fail
        */
        virtual int decode_unicore(const std::string& line, double& t, std::vector<double>& v);

        /**
        * @brief decode the data body of the IMU data file with great format.
        * order="great"
        * decode data body of IMU file, all the data read will store in the ins_data
        *
        * @param[in]  line        line in imu file
        * @param[in]  t              time of data epoch
        * @param[out] v           imu data(acce & gyro)

        * @return
            @retval 1   success
            @retval -1  fail
        */
        virtual int decode_great(const std::string& line, double& t, std::vector<double>& v);

        /**
        * @brief decode the data body of the IMU data file with fsas format.
        * order="imr_fsas"
        * decode data body of IMU file, all the data read will store in the ins_data
        *
        * @param[in]  line        line in imu file
        * @param[in]  t              time of data epoch
        * @param[out] v           imu data(acce & gyro)

        * @return
            @retval 1   success
            @retval -1  fail
        */
        virtual int decode_imr_fsas(const std::string& line, double& t, std::vector<double>& v);

        /**
        * @brief decode the data body of the IMU data file with starneto ascii format.
        * order="starneto"
        * decode data body of IMU file, all the data read will store in the ins_data
        *
        * @param[in]  line        line in imu file
        * @param[in]  t              time of data epoch
        * @param[out] v           imu data(gyro & acce)

        * @return
            @retval 1   success
            @retval -1  fail
        */
        virtual int decode_starneto(const std::string& line, double& t, std::vector<double>& v);

        /**
        * @brief decode the data body of the IMU data file with starneto binary format.
        * order="starneto" but maybe not used (maybe, zzwu)
        * decode data body of IMU file, all the data read will store in the ins_data
        *
        * @param[in]  line        line in imu file
        * @param[in]  t              time of data epoch
        * @param[out] v           imu data(gyro & acce)

        * @return
            @retval 1   success
            @retval -1  fail
        */
        virtual int decode_starneto(const char* block, int sz, std::vector<std::pair<double, std::vector<double>>>& v);

        /**
        * @brief judge whether IMU data is available
        * @param[in]  now         current epoch

        * @return
            @retval true   available
            @retval false  unavailable
        */
        virtual bool available(const base_time& now);

        /**
        * @brief encode the header of the IMU data file.
        * imu file doesn't include head block
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  errmsg      error message of the data decoding
        * @return
            @retval >=0 consume size of header decoding
            @retval <0  finish reading
        */
        virtual  int encode_head(char* buff, int sz, std::vector<std::string>& errmsg);

        /**
        * @brief encode the data body of the IMU data file.
        * the main entry of encode IMU file.
        *
        * encode data body of IMU file.
        *
        * @param[in]  buff        buffer of the data
        * @param[in]  sz          buffer size of the data
        * @param[in]  cnt         size of lines successfully encoded
        * @param[in]  errmsg      error message of the data encoding
        * @return
            @retval >=0 consume size of data encoding
            @retval <0  finish reading
        */
        virtual  int encode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg);

    private:
        double _tt;
        bool _complete;         /// reading is complete 
        std::string _order;         /// the imu record order
        double _ts;             /// data interval
        double _freq;         /// data interval
        double t_beg;
        double t_end;
        bool is_first;
        UNIT_TYPE _GyroUnit; /// Gyro data Unit
        UNIT_TYPE _AcceUnit; /// Acce data Unit
        UNIT_TYPE _MagUnit;  /// Mag data Unit
    };
}


#endif