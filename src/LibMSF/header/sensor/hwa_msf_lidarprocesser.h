#ifndef hwa_msf_lidar_processer_h
#define hwa_msf_lidar_processer_h
#include "hwa_set_base.h"
#include "hwa_set_proc.h"
#include "hwa_base_Time.h"
#include "hwa_base_TimeCost.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_filter.h"
#include "hwa_base_allpar.h"
#include "hwa_base_iof.h"
#include "hwa_base_eigendef.h"
#include "hwa_lidar_base.h"
#include "hwa_lidar_data.h"
#include "hwa_lidar_coder.h"
#include "hwa_msf_baseprocesser.h"

using namespace hwa_lidar;

namespace hwa_msf {
    class lidarprocesser : public baseprocesser, public lidar_base {
    public:
        explicit lidarprocesser(const baseprocesser& B, base_data* data = nullptr);
        explicit lidarprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog = nullptr, base_data* data = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
        ~lidarprocesser() {};

        void add_lidarframe(LidarFrame& frame);
        void getAllOdoResidual(Matrix& H, Vector& r, bool use_3d, float ther = 1);
        void ProjOdoResidual(Matrix& H, Vector& r, bool use_3d, float ther = 1);
        void build_PPHR(vector<LidarFrame>& buffer, std::map<int, std::vector<int>>& indexs, Matrix& H, Vector& r);
        bool planarpatchJacobian(vector<LidarFrame>& buffer, int point_id, std::vector<int> corr_ids, Matrix& H, Vector& r);
        int updateLidarBuffer();
        bool _gatingTest(Matrix& H, Vector& r, const int& dof, bool is_scan);
        bool _gatingTest(Matrix& H, Vector& r, const int& dof, double obs_noise);
        void removeLidar(vector<LidarStateIDType>& rm_lidar_state_ids);
        void removeState(vector<LidarStateIDType>& rm_lidar_state_ids);
        void removeIDandBuffer(vector<LidarStateIDType>& rm_lidar_state_ids);
        void _meas_update(const Matrix& H, const Vector& r, bool isscan);
        void StateAugmentation();
        int ProcessOneEpoch() override;
        void AddData(base_data* data) override { _lidardata = dynamic_cast<lidar_data*>(data); };
        bool _init() override { return false; };
        void _feed_back() override;
        bool _time_valid(base_time inst) override;
        bool load_data() override;

    private:
        lidar_data* _lidardata;                               ///< store lidar data
        lidarPath _lidar_path;                                ///< get cur lidar path
        LidarFrame _lidarframe;                               ///< Point cloud operating at the current time
        LidarFrame _lastframe;                                ///< only used for point cloud distortion 
        std::vector<LidarFrame> lidar_buffer;                 ///< store the information of lidar frames in window

        lidar_proc_mapping _lidarmap;                        ///< lidar map
        lidar_map _priormap;                                 ///< prior map
        lidar_proc_odometry _lidarOdo;                           

        bool mIsFirstLidar = false;
    };
}
#endif
