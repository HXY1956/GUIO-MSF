#ifndef hwa_vis_proc_feature_h
#define hwa_vis_proc_feature_h
#include "hwa_set_vis.h"
#include "hwa_base_mutex.h"
#include "hwa_vis_proc_utility.h"
#include "hwa_base_eigendef.h"
#include <ceres/ceres.h>

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_vis
{
    class vis_feature
    {
    public:
        /** @brief default constructor. */
        vis_feature();
        vis_feature(const FeatureIDType& new_id);

        bool triangulatePoint(const CamStateServer& cam_states) const;

        bool checkMotion(const CamStateServer& cam_states) const;

        void generateInitialGuess(
            const SE3& T_c1_c2, const Eigen::Vector2d& z1,
            const Eigen::Vector2d& z2, Triple& p) const;

        void cost(const SE3& T_c0_ci,
            const Triple& x, const Eigen::Vector2d& z,
            double& e) const;

        void jacobian(const SE3& T_c0_ci,
            const Triple& x, const Eigen::Vector2d& z,
            Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
            double& w) const;


        bool initializePosition(const CamStateServer& cam_states);

        std::vector<std::pair<Triple, Triple>> getCorresponding(int frame_count_l, int frame_count_r);
     
    public:
        std::map<CamStateIDType, Eigen::Vector4d, std::less<CamStateIDType>,
            Eigen::aligned_allocator<
            std::pair<const CamStateIDType, Eigen::Vector4d> > > observations;        ///< store all the observations of the feature 

        FeatureIDType id;                ///< cur feature id
        Triple position;        ///< 3d postion of the feature in  world frame.
        bool is_initialized;            ///< A indicator to show if the 3d postion of the feature has been initialized or not.
        double initial_depth=0;            ///< initial depth from stero orb font end 
        bool is_KeyFrame;                ///< A indicator to show if feature is generated on the keyframe
        bool is_initialized_NonKey;     ///< A indicator to show if feature is initialized by nonkeyframe
        bool isLost = false;            ///< A indicator to show if feature track lost
        
        double translation_threshold=0.2;            ///< transformation matrix threshold
        double huber_epsilon=0.01;                    ///< kernel function
        double estimation_precision=0.0000005;            ///< threshold of whether the update quantity is iterative
        double initial_damping=0.001;                ///< initial lambda of Levenberg-Marquart
        int outler_loop_max_iteration=10;        ///< iterations
        int inner_loop_max_iteration=10;            ///< iterations of depth
        SE3 T_cam0_cam1;            ///< transformation matrix between cam0 and cam1
        bool stereo;                            ///< stereo or num

        int start_frame;
        int start_frame_id;
        int frame_size = 0;
        int end_frame_id;
        int end_frame;
        int endFrame() { 
            return start_frame + frame_size - 1;
        }
    };
}

#endif
