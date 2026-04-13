#ifndef hwa_lidar_data_h
#define hwa_lidar_data_h

#include "hwa_base_data.h"
#include "hwa_lidar_utility.h"

using namespace hwa_base;

namespace hwa_lidar
{
    class lidar_data : public base_data
    {
    public:
        /** @brief default constructor. */
        lidar_data();

        /** @brief default destructor. */
        virtual ~lidar_data() {};
    
        /**
        * @brief add one data into vector
        * @note data includes time and img0_path and img1_path
        * @param[in] t                    time
        * @param[in] lidar_path            path of lidar data
        * @return
            @retval =0                    reprents success of add
        */
        int add_LIDAR(const double& t,const std::string& lidar_path);

        /**
        * @brief load data into lidar_path according to imu_t
        * @param[in]  imu_t                time of IMU
        * @param[out] lidar_t            time of Lidar
        * @param[out] lidar_path        path of lidar data
        */
        bool load(const double& imu_t, double& lidar_t, lidarPath& lidar_path);
        
        /** 
        * @brief return lidarpath data vector size
        * @return int                    represent size of lidar data vector
        */
        int size();
    
        private:
            std::vector<lidarPath> _veclidar;    ///< lidarpath data
    };

}
#endif