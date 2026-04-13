#ifndef hwa_set_tracker_h
#define hwa_set_tracker_h
#include "hwa_set_base.h"
#include "Eigen/Eigen"
#define XMLKEY_TRACKER "tracker"

namespace hwa_set
{
    class set_tracker : public virtual set_base
    {
    public:
        set_tracker() {};
        set_tracker(set_base* gset);
        virtual ~set_tracker() {};

        void check();
        void help() {};
        double rot_noise();
        double trans_noise();

        int PickRate();
        int camskip();
        double mindist();
        double canny();
        double acc();
        int minradius();
        int maxradius();
        double interval();
        std::string type();
        SO3 R_tracker_imu();
        Triple T_tracker_imu();


    protected:
        double Tracker_Rot_noise;
        double Tracker_Trans_noise;

    private:
    };
}
#endif