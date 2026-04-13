#ifndef hwa_ins_data_h
#define hwa_ins_data_h
#include <cstdint>
#include <queue>
#include <stack>
#include "hwa_base_data.h"
#include "hwa_ins_base_utility.h"

namespace hwa_ins
{
    struct imu_package {
        double timestamp;   // 
        float gyro[3];     // rad/s
        float accel[3];    // m/s^2
        float rfu[6];       // 24~47
        int timestamp_us;   // 48~51
        int status;         // 52~55

        static constexpr int PacketSize = 7 * sizeof(float); // for UDP packet
    };

#pragma pack(push, 1)
    struct imu_raw_package {
        float gyro[3];      // 0~11
        float accel[3];     // 12~23
        float rfu[6];       // 24~47
        int timestamp_us;   // 48~51
        int status;         // 52~55
    };
#pragma pack(pop)

    struct imu_unit {
        imu_unit() { Status = false; };
        imu_unit(double t, Triple g, Triple a, Triple m = Triple::Zero()) {
            time = t;
            gyo = g;
            acc = a;
            mag = m;
            Status = true;
        }
        double time;
        Triple gyo;
        Triple acc;
        Triple mag;
        bool Status;
    };

    class ins_data : public base_data
    {
    public:
        /**
        * @brief default constructor.
        */
        explicit ins_data();

        /**
        * @brief default destructor.
        */
        explicit ins_data(base_log spdlog);

        virtual ~ins_data() {}

        /**
        * @brief add one data into vector
        * @note data includes time and wm and vm
        * @param[in] 't' is time
        * @param[in] 'wm' is Angular increment(rad)
        * @param[in] 'vm' is Velocity increment(m/s)
        * @return value reprents success or failure of add
        */
        int add_IMU(const double& t, const Triple& wm, const Triple& vm);

        int add_IMU(const double& t, const Triple& wm, const Triple& vm, const Triple& mm);

        //multi-imu
        void sort_IMU();

        /**
        * @brief set imu data interval
        * @param[in] 'ts' is interval
        * @return void
        */
        void set_ts(double ts);

        /**
        * @brief load data into wm and vm and t accroding to subsamples
        * @param[in] 't' is last time
        * @param[in] 'nSamples' is subsamples
        * @param[in] '_beg_end' is the direction
        * @param[out] 't' is current time
        * @param[out] 'wm' is Angular increment
        * @param[out] 'vm' is Velocity increment
        * @param[out] 'ts' is computing intervals
        * @return bool
        */
        bool load(std::vector<Triple>& wm, std::vector<Triple>& vm, double& t, double& ts, int nSamples, bool& status);
        bool load(int id, std::vector<Triple>& wm, std::vector<Triple>& vm, double& t, double& ts, int nSamples, bool& status);

        /**
        * @brief load data into wm and vm and t accroding to subsamples
        * @param[in] 't' is last time
        * @param[in] 'nSamples' is subsamples
        * @param[in] '_beg_end' is the direction
        * @param[out] 't' is current time
        * @param[out] 'wm' is Angular increment
        * @param[out] 'vm' is Velocity increment
        * @param[out] 'mm' is Magnetometer data(consult wh)
        * @param[out] 'ts' is computing intervals
        * @return bool
        */
        bool load(std::vector<Triple>& wm, std::vector<Triple>& vm, std::vector<Triple>& mm, double& t, double& ts, int nSamples, bool& status);

        /**
        * @brief load data without erasing
        * @param[in] 't' is last time
        * @param[in] 'nSamples' is subsamples
        * @param[in] 'offset' offset index from the head/back of the deque
        * @param[in] '_beg_end' is the direction
        * @param[out] 't' is current time
        * @param[out] 'wm' is Angular increment
        * @param[out] 'vm' is Velocity increment
        * @param[out] 'ts' is computing intervals
        * @return bool
        */
        bool get(std::vector<Triple>& wm, std::vector<Triple>& vm, double& t, double& ts, int nSamples, int offset = 0);

        /**
        * @brief erase imu data before t
        * @param[in] 't' is current time
        * @param[in] '_beg_end' is the direction
        * return base_time represent the most recent undeleted epoch time
        */
        base_time erase_bef(base_time t);
        base_time erase_bef(int id, base_time t);

        /**
        * @brief return imu data vector size
        * @param[in] '_beg_end' is the direction
        * @return int represent the size of imu data
        */
        int size();

        /**
        * @brief judge whether IMU data is available
        * @param[in]  now         current epoch
        * @param[in] '_beg_end' is the direction
        * @return
            @retval true   available
            @retval false  unavailable
        */
        virtual bool available(const base_time& now);

        /**
        * @brief get the begin time of imu data
        * @param[in] '_beg_end' is the direction
        * @return double  first epoch
        */
        virtual double beg_obs();

        /**
        * @brief get the end time of imu data
        * @param[in] '_beg_end' is the direction
        * @return double  end epoch
        */
        virtual double end_obs();

        /**
        * @brief get the begin time of imu data
        * @param[in] '_beg_end' is the direction
        * @return double  first epoch
        */
        virtual double beg_obs_mimu();

        /**
        * @brief get the end time of imu data
        * @param[in] '_beg_end' is the direction
        * @return double  end epoch
        */
        virtual double end_obs_mimu();

        virtual bool reset_mimu_data();

        std::deque<imu_unit> _imu_forward;         /// imu date stored for backward processing
        //multi-imu
        double _ts;                     /// imu interval 
        int _imu_num;

    private:
        int first, end;                 /// imu starting and ending time 
        std::map<int, int> first_mimu, end_mimu;                 /// imu starting and ending time 
        //multi-imu
        std::map<int, std::deque<imu_unit>> _imu_all_forward;
    };

}

#endif