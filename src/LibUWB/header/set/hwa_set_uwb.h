#ifndef hwa_set_uwb_h
#define hwa_set_uwb_h
#include "hwa_set_base.h"
#include "hwa_base_eigendef.h"
#include "Eigen/Eigen"
#define XMLKEY_UWB "uwb"

namespace hwa_uwb {
    enum class UWB_WEIGHT { EQUAL, SNR, ANTIRANGE, SUCCESSRATE };
    enum class NAVAREA { INDOOR, OUTDOOR, TRANSITION };
};

using namespace hwa_uwb;
using namespace hwa_base;

namespace hwa_set
{
    class set_uwb : public virtual set_base
    {
    public:
        set_uwb();
        virtual ~set_uwb() {};
        void check();
        void help();

        int iter();
        int nq();
        double ts();
        double sample();
        Triple pos();
        Triple initial_pos_std();
        Triple pos_psd();
        UWB_WEIGHT wgt_type();
        UWB_WEIGHT str2uwbweight(const std::string& wg);
        std::string order();
        std::string uwbweight2str(const UWB_WEIGHT& wg);
        static std::string navarea2str(const NAVAREA& area);

        double start();
        double end();
        std::vector<std::string> anchor_list();
        std::string filter();
        bool smooth();
        bool addnoise();
        int smooth_point();
        double meas_range_std();
        double best_range_std();
        bool output_res();
        std::string result_file();
        std::string res_file();
        std::vector<std::string> indoor_anchor_list();
        std::vector<std::string> outdoor_anchor_list();
        double barrior();
        double kappa_sig();
        double alpha_sig();
        double log_parameter();
        double exp_parameter();
        double sigmoid_parameter();
        double sigmoid_threshold();
        double tolerance();
        std::string penal();
        bool SDP();
        bool interpolation();
        double interpolation_noise();
        bool posterior();
        double gama();
        double alpha();
        bool pred_constraint();
        double range_lim();
        double snr_lim();
        double max_res_norm();
        double E0();
        double G0();
        int dof1();
        int dof2();
        int max_iter();
        double Tau();
        double proc_noise();
        int num_particles();

    private:
        int _iter;
        int _nq;
        bool _move;
        double _ts;
        double _sample;
        Triple _pos, _initial_pos_std;
        double _pos_psd;
        UWB_WEIGHT _wgt_type;
        std::string _com;
        std::string _order;
        std::vector<std::string> _anchor_list;
    };
}
#endif