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
    class feature_per_frame {
    public:
        feature_per_frame() {
            cur_td = 0;
        }
    public:
		Eigen::Vector4d position;        ///< store the observation of the feature in the image frame
		Eigen::Vector2d velocity; 		  ///< store the velocity of the feature in the image frame
        double cur_td;
    };

    class vis_feature
    {
    public:
        /** @brief default constructor. */
        vis_feature();
        vis_feature(const FeatureIDType& new_id);

        bool triangulatePoint(const CamStateServer& cam_states) const;

        bool checkMotion(const CamStateServer& cam_states) const;

        bool generateInitialGuess(
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

        static void read_Tc0c1(const SE3& _T ) {
			T_cam0_cam1 = _T;
        }
     
    public:
        std::map<CamStateIDType, feature_per_frame, std::less<CamStateIDType>,
            Eigen::aligned_allocator<
            std::pair<const CamStateIDType, feature_per_frame> > > observations;        ///< store all the observations of the feature 

        FeatureIDType id;                ///< cur feature id
        Triple position;        ///< 3d postion of the feature in  world frame.
        bool is_initialized;            ///< A indicator to show if the 3d postion of the feature has been initialized or not.
        double initial_depth=0;            ///< initial depth from stero orb font end 
        bool is_KeyFrame;                ///< A indicator to show if feature is generated on the keyframe
        bool is_initialized_NonKey;     ///< A indicator to show if feature is initialized by nonkeyframe
        bool isLost = false;            ///< A indicator to show if feature track lost
        double inv_depth = -1;
        
        static double translation_threshold;            ///< transformation matrix threshold
        static double huber_epsilon;                    ///< kernel function
        static double estimation_precision;            ///< threshold of whether the update quantity is iterative
        static double initial_damping;                ///< initial lambda of Levenberg-Marquart
        static int outler_loop_max_iteration;        ///< iterations
        static int inner_loop_max_iteration;            ///< iterations of depth
        static SE3 T_cam0_cam1;            ///< transformation matrix between cam0 and cam1
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
