#include "hwa_vis_data.h"
#include "hwa_set_vis.h"

using namespace std;

hwa_vis::vis_data::vis_data() :base_data(),
_ts(0.1)
{
    id_type(CAMDATA);
}

hwa_vis::vis_data::vis_data(hwa_set::set_base* _gset, int cam_group_id) :base_data()
{
    _ts = 1.0 / dynamic_cast<hwa_set::set_vis*>(_gset)->freq();
    id_type(CAMDATA);
}

int hwa_vis::vis_data::add_IMG(const double& t, const string& img0_path, const string& img1_path)
{
    IMG_PATH tmp = { t,img0_path,img1_path };
    _vecimg.push_back(tmp);
    return 0;
}

bool hwa_vis::vis_data::load(double& imu_t, double& img_t, IMG_PATH& img_path)
{
    auto iter = _vecimg.begin();
    for (iter = _vecimg.begin(); iter != _vecimg.end(); iter++) {
        if (std::abs(iter->t - imu_t) <= _ts / 2) {
            img_path = *iter;
            img_t = iter->t;
            _vecimg.erase(_vecimg.begin(), iter + 1);
            return true;
        }
        if (iter->t - imu_t > _ts / 2) 
            return false;
    }
    return false;
}

bool hwa_vis::vis_data::load(double& imu_t, IMG_PATH& img_path)
{
    //cout << "imut_t" << setprecision(12) << imu_t << endl;
    for (auto record : _vecimg)
    {
        if (abs(record.t - imu_t) <= _ts / 2)
        {
            img_path = record;
            return true;
        }
    }
    return false;
}

int hwa_vis::vis_data::size()
{
    return _vecimg.size();
}


