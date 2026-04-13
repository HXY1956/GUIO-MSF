#ifndef hwa_vis_data_h
#define hwa_vis_data_h
#include "hwa_base_data.h"
#include "hwa_set_base.h"
#include "hwa_vis_proc_utility.h"

using namespace hwa_base;

namespace hwa_vis
{
    /**
    *@brief Class for storing the IMG data,just the timestamp and imgpath
    *
    * imgfile is used for Front end tracking.
    * The data contains information about  timestamp and imgpath.
    * The gcoder vis_data corresponding to the gcoder t_imgfile.
    */
    class vis_data : public base_data
    {
    public:
        /** @brief default constructor. */
        vis_data();

        vis_data(hwa_set::set_base* s, int cam_group_id = 0);

        /** @brief default destructor. */
        virtual ~vis_data() {}

        /**
        * @brief add one data into vector
        *
        * note data includes time and img0_path and img1_path
        *
        * @param[in]  t                        time
        * @param[in]  img0_path                left image path
        * @param[in]  img1_path                right image path
        * @return
            @retval =0                        reprents success of add
        */
        int add_IMG(const double& t, const std::string& img0_path, const std::string& img1_path);

        /**
        * @brief load data into img_path according to imu_t
        *
        * @param[in]   t                    time of imu
        * @param[out]  img_t                time of image
        * @param[out]  img_path                path of image
        */
        bool load(double& imu_t, double& img_t, IMG_PATH& img_path);

        /**
        * @brief load data into img_path according to imu_t
        *
        * @param[in]   t                    time of imu
        * @param[out]  img_t                time of image
        * @param[out]  img_path                path of image
        * @return
            @retval =true                    reprents success of add
            @retval =flase                    reprents failure of add
        */
        bool load(double& imu_t, IMG_PATH& img_path);

        /**
        * @brief return imgpath data vector size
        * @return
            @retval int                        reprents the size of data vector
        */
        int size();
        std::vector<IMG_PATH> _vecimg;         ///< image path data

    private:

        std::vector<ONE_FRAME> _rtvectimg;         ///< image data for real time
        double _ts = 0;                     ///< data interval
    };
}
#endif