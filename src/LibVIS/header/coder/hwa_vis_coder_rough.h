#ifndef hwa_vis_coder_rough_h
#define hwa_vis_coder_rough_h
#include "hwa_set_vis.h"
#include "hwa_vis_proc_Utility.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_vis
{
    using ImgMap = std::map<double, IMG_PATH>;
    class vis_coder_rough
    {
    public:
        vis_coder_rough(hwa_set::set_base* _gset, const std::string& filepath_l, const std::string& filepath_r = "");
        ~vis_coder_rough() {}
        void add_IMG(const double& t, const std::string& img0_path, const std::string& img1_path = "");
        bool load(const double& target, IMG_PATH& img_path);
        bool TimeDiff(double t1, double t2);
        bool read();
    private:
        std::unique_ptr<ImgMap> imgs;   ///< image path data
        double _ts = 0;                     ///< data interval
        double _imu_ts = 0.01;
        std::fstream fileL;
        std::fstream fileR;
        bool stereo;
    };
}
#endif