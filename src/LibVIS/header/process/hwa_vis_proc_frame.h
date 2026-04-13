#ifndef hwa_vis_proc_frame_h
#define hwa_vis_proc_frame_h
#include "string"
#include <vector>
#include <opencv2/opencv.hpp>
#include "hwa_base_eigendef.h"
#include "hwa_set_base.h"
#include "hwa_base_mutex.h"
#include "hwa_vis_proc_utility.h"

using namespace hwa_base;

namespace hwa_vis
{
    class vis_frame
    {
    public:
        struct PointsAttri
        {
            cv::Point2f  cam0_pt;            ///< TODO    
            cv::Point2f  cam1_pt;            ///< TODO    
            cv::KeyPoint cam0_keypt;        ///< TODO    
            cv::KeyPoint cam1_keypt;        ///< TODO    
            cv::Mat cam0_descriptor;        ///< TODO    
            cv::Mat cam1_descriptor;        ///< TODO    
            float  depth = 0.0;                ///< TODO    
            int id = 0;                        ///< TODO    
            int track_cnt = 0;                ///< TODO    
            double response = 0.0;            ///< TODO    
        };
        double time=0;                            ///< timestamp of the img
        cv::Mat cam0_img;                        ///< A copy of the original observation data(camera 0)
        cv::Mat cam1_img;                        ///< A copy of the original observation data(camera 1)
        std::vector<cv::Mat> cam0_pyramid;        ///< pyramid of cam0
        std::vector<cv::Mat> cam1_pyramid;        ///< pyramid of cam1
        std::vector<PointsAttri> vpointInFrame;        ///< the feature in cur frame
    };
}
#endif // !GFRAME_H
