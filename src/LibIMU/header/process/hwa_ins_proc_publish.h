#ifndef hwa_ins_proc_publish_h
#define hwa_ins_proc_publish_h
#include "hwa_base_posetrans.h"
#include "hwa_base_globaltrans.h"
#include "hwa_ins_base_utility.h"
#include "hwa_ins_proc_viewer.h"
#include "hwa_base_eigendef.h"
#include <fstream>

using namespace hwa_base;

namespace hwa_ins
{
    class ins_publish
    {
    public:
        /** @brief default constructor. */
        ins_publish();

        /** @brief default destructor. */
        ~ins_publish();

        /**
        * @brief initialize the viewer for display
        */
        void Initialize();

        /**
        * @brief update the parameters for trajectory display
        *
        * @param[in] imu_state        store the imu information to show on the viewer
        * @param[in] init_campos    the initial value of the camera position
        */
        void UpdateNewState(const ImuState& imu_state);

# if (defined USE_vis) || (defined USE_LIDAR)
        /**
        * @brief update the parameters for trajectory display
        *
        * @param[in] imu_state        store the imu information to show on the viewer
        * @param[in] cam_states        store the cam information to show on the viewer
        * @param[in] init_campos    the initial value of the camera position
        */
        void UpdateNewState(const ImuState &imu_state,const CamStateServer &cam_states,const Triple & init_campos);

        /**
        * @brief update the parameters for trajectory display
        *
        * @param[in] imu_state        store the imu information to show on the viewer
        * @param[in] lidar_states    store the lidar information to show on the viewer
        * @param[in] init_campos    the initial value of the camera position
        */
        void UpdateNewState(const ImuState &imu_state, const LidarStateServer &lidar_states, std::vector<Triple> lidarPointcloud = std::vector<Triple>());

        /**
        * @brief update the std::map points for display
        */
        void UpdateMapPoints(const std::map<FeatureIDType, Triple> & map_points);
        void UpdateMapPoints(const std::vector<std::map<FeatureIDType, Triple>> & map_points);
#endif
        void AddMapPoints(const std::vector<Triple> & map_points);
        void UpdateVisualPoints(const std::vector<Triple> & map_points);
        void UpdatePlanePoints(const std::vector<Triple> & pcs, const std::vector<std::vector<Triple>>& _near_points, SO3 R_l_e, Triple t_l_e);

    private:
        
        ins_viewer *viewer=nullptr;                // the core class of trajectory display
        bool firstFlag_imu = true;        // A indicator to determine the value of _init_imupos
        bool firstFlag_cam = true;        // A indicator to determine the value of _init_campos,
        //bool firstFlag_lidar = true;    // A indicator to determine the value of _init_lidarpos,
                                        // maybe imgdata is later than imudata
        Triple _init_campos;    // transfer the zero of the IMU coordinate
        //Triple _init_lidarpos;    // transfer the zero of the IMU coordinate
        Triple _init_imupos;    // transfer the zero of the IMU coordinate
        SO3 R_e_n;          // rotation from ECEF frame to ENU frame for first imudata

        
    };
}
#endif