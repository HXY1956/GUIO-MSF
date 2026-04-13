#ifndef hwa_set_vis_h
#define hwa_set_vis_h
#define XMLKEY_VIS "vis"
#define XMLKEY_YOLO "yolo"
#include "hwa_set_base.h"
#include "hwa_base_eigendef.h"

namespace hwa_vis {
    enum TRACK_TYPE {
        STEREO_MATCH = 0,
        MONO_TRACK = 1,
        STEREO_DETECT = 2
    };
    enum FILTER_TYPE {
        MATCH_FILTER = 0,
        TRACK_FILTER = 1,
        TOTAL_FILTER = 2
    };

    enum DISTORTION_TYPE
    {
        radtan,
        equidistant
    };

    enum CLONE_TYPE
    {
        IMU,
        CAMERA
    };

    enum PROCESSER_TYPE {
        GPU,
        CPU
    };

    DISTORTION_TYPE str2distortion(const std::string& s);
    PROCESSER_TYPE str2processer(const std::string& s);
    std::string distortion2str(const DISTORTION_TYPE& s);
};

namespace hwa_set
{
    /**
    * @brief Class provides some parameters used in back end.
    */
    class set_vis : public virtual set_base
    {
    public:
        /** @brief default constructor. */
        set_vis();

        /** @brief default destructor. */
        ~set_vis();

        /** empty functions */
        void check();
        void checkdefault();
        /** empty functions */
        void help();

        double start(int cam_group_id = 0);
        double end(int cam_group_id = 0);

        double ts(int cam_group_id = 0);
        int freq(int cam_group_id = 0);
        int num_of_cam_group();

        hwa_vis::DISTORTION_TYPE cam0_distortion_model(int cam_group_id = 0);
        hwa_vis::DISTORTION_TYPE cam1_distortion_model(int cam_group_id = 0);
        Eigen::Vector2d cam0_resolution(int cam_group_id = 0);
        Eigen::Vector2d cam1_resolution(int cam_group_id = 0);
        Eigen::Vector4d cam0_intrinsics(int cam_group_id = 0);
        Eigen::Vector4d cam1_intrinsics(int cam_group_id = 0);
        Eigen::Vector4d cam0_distortion_coeffs(int cam_group_id = 0);
        Eigen::Vector4d cam1_distortion_coeffs(int cam_group_id = 0);

        bool stereo(int cam_group_id = 0);
        SO3 R_cam0_cam1(int cam_group_id = 0);
        Triple t_cam0_cam1(int cam_group_id = 0);
        SE3 T_cam0_cam1(int cam_group_id = 0);
        SO3 R_cam0_imu(int cam_group_id = 0);

        Triple t_cam0_imu(int cam_group_id = 0);
        SE3 T_cam0_imu(int cam_group_id = 0);
        double dt_cam0_imu(int cam_group_id = 0);
        int num_of_cam(int cam_group_id = 0);

        int grid_row(int cam_group_id = 0);
        int grid_col(int cam_group_id = 0);
        int grid_min_feature_num(int cam_group_id = 0);
        int grid_max_feature_num(int cam_group_id = 0);
        int pyramid_levels(int cam_group_id = 0);
        int patch_size(int cam_group_id = 0);
        int fast_threshold(int cam_group_id = 0);

        double ransac_threshold(int cam_group_id = 0);

        double stereo_threshold(int cam_group_id = 0);        ///< TODO
        int max_iteration(int cam_group_id = 0);
        double track_precision(int cam_group_id = 0);

        bool usingstereorecify(int cam_group_id = 0);

        int max_cnt(int cam_group_id = 0);
        int max_cam_state_size(int cam_group_id = 0);
        double minparallex(int cam_group_id = 0);
        double position_std_threshold(int cam_group_id = 0);
        double rotation_threshold(int cam_group_id = 0);
        double translation_threshold(int cam_group_id = 0);
        double feature_observation_noise(int cam_group_id = 0);
        int _equalize(int cam_group_id = 0);

        double huber_epsilon(int cam_group_id = 0);                    ///< TODO
        double estimation_precision(int cam_group_id = 0);            ///< TODO
        double initial_damping(int cam_group_id = 0);                ///< TODO
        int outler_loop_max_iteration(int cam_group_id = 0);        ///< TODO
        int inner_loop_max_iteration(int cam_group_id = 0);            ///< TODO
        bool estimate_extrinsic(int cam_group_id = 0);
        bool estimate_t(int cam_group_id = 0);
        hwa_vis::CLONE_TYPE clone_type(int cam_group_id = 0);
        hwa_vis::CLONE_TYPE str2ct(const std::string& s);
        bool estimate_extrinsic_seperately(int cam_group_id = 0);
        bool estimate_extrinsic_allcam(int cam_group_id = 0);
        bool estimate_t_allcam(int cam_group_id = 0);
        Triple initial_extrinsic_rotation_cov(int cam_group_id = 0);
        Triple initial_extrinsic_translation_cov(int cam_group_id = 0);
        double initial_t_cov(int cam_group_id = 0);

        int cam_update_skip(int cam_group_id = 0);                    ///< TODO
        /*Not used yet*/

        int num_particles(int cam_group_id = 0);
        double barrior(int cam_group_id = 0);
        double kappa_sig(int cam_group_id = 0);
        double alpha_sig(int cam_group_id = 0);
        double E0(int cam_group_id = 0);
        double G0(int cam_group_id = 0);
        int dof1(int cam_group_id = 0);
        int dof2(int cam_group_id = 0);
        int max_iter(int cam_group_id = 0);
        double Tau(int cam_group_id = 0);
        double proc_noise(int cam_group_id = 0);
        double static_threshold(int cam_group_id = 0);
        double max_res_norm(int cam_group_id = 0);
        std::string filter(int cam_group_id = 0);
        hwa_vis::PROCESSER_TYPE processer(int cam_group_id = 0);
        std::string cam_group_name(int cam_group_id = 0);
        std::string gst_path(int cam_group_id = 0);
        std::vector<int> camera_list();
        int group_number();

        bool detect();
        std::string modelpath();
        std::vector<std::string> clsname();
        std::pair<int, int> inputsize();
        int numpred();
        int classnumber();
        float conf();
        float nms();

    protected:
    };
}
#endif
