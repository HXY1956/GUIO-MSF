#ifndef hwa_set_fgo_h
#define hwa_set_fgo_h
#include "hwa_set_base.h"
#include "hwa_base_eigendef.h"
#define XMLKEY_FGO "fgo"

using namespace hwa_base;

namespace hwa_set
{
    class set_fgopara : public virtual set_base
    {
    public:
        set_fgopara();
        virtual ~set_fgopara() {};
        void check();
        void help();

        double pixel_error();
        double laser_cloud_error();
        double acc_n();
        double acc_w();
        double gyr_n();
        double gyr_w();
        double relative_pos_var();
        double relative_rot_var();
        int window_size();

    private:
        double _pixel_error = 1.0;				///< error of pixel
        double _laser_cloud_error = 1.0;			///< error of laser cloud
        double _acc_n = 0.05;						///< accelerometer measurement noise
        double _acc_w = 0.001;						///< accelerometer bias random work noise
        double _gyr_n = 0.005;						///< gyroscope measurement noise
        double _gyr_w = 0.0001;						///< gyroscope bias random work noise
        double _relative_pos_var = 0.1;			///< relative position variance
        double _relative_rot_var = 0.001;			///< relative rotation variance		
        int _window_size = 10;
    };
}
#endif