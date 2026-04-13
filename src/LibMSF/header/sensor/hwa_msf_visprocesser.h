#ifndef hwa_msf_vis_processer_h
#define hwa_msf_vis_processer_h
#include "hwa_set_base.h"
#include "hwa_set_proc.h"
#include "hwa_base_Time.h"
#include "hwa_base_TimeCost.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_filter.h"
#include "hwa_base_allpar.h"
#include "hwa_base_iof.h"
#include "hwa_base_eigendef.h"
#include "hwa_vis_coder_stereousb.h"
#include "hwa_vis_base.h"
#include "hwa_vis_data.h"
#include "hwa_vis_coder.h"
#include "hwa_msf_baseprocesser.h"

using namespace hwa_vis;

namespace hwa_msf {
    class visprocesser : public baseprocesser, public vis_base {
    public:
        explicit visprocesser(const baseprocesser& B, int ID, base_data* data = nullptr);
        explicit visprocesser(std::shared_ptr<set_base> gset, std::string site, int ID, base_log spdlog = nullptr, base_data* data = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
        ~visprocesser();

        void StateAugmentation();
        void RemoveLostFeatures();
        void PruneCamState();
        bool GatingTest(const Matrix& H, const Vector& r, const int& dof);
        void meas_update(const Matrix& H, const Vector& r);
        void _write_calib();
        bool _extrinsic_init();
        bool align_vins();
        void align_feedback();
        int ProcessOneEpoch() override;
        void AddData(base_data* data) override { imgdata = dynamic_cast<vis_data*>(data); };
        bool _init() override { return false; };
        void _feed_back() override;
        bool _time_valid(base_time inst) override;

    private:
        vis_data* imgdata = nullptr;
        Triple initCamPos;
        std::string _site;
        base_iof* _fcalib = nullptr;
        int cam_group_id;
        bool mIsFirstImg;
        int imu_frequency;
        int _imgproc_count = 0;
        std::ofstream TimeCostDebugOutFile;
        bool TimeCostDebugStatus;
    };
}
#endif
