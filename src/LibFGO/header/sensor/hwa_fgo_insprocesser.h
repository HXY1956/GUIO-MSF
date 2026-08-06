#ifndef hwa_fgo_ins_processer_h
#define hwa_fgo_ins_processer_h

#include "hwa_set_ins.h"
#include "hwa_set_ign.h"
#include "hwa_set_proc.h"
#include "hwa_set_out.h"
#include "hwa_base_filter.h"
#include "hwa_base_preintegration.h"
#include "hwa_ins_proc_publish.h"
#include "hwa_ins_data.h"
#include "hwa_ins_coder.h"
#include "hwa_fgo_baseprocesser.h"

using namespace hwa_ins;

namespace hwa_fgo {
	class insprocesser : public baseprocesser {
	public:
		explicit insprocesser(const baseprocesser& B, base_data* data = nullptr);
		explicit insprocesser(std::shared_ptr<set_base> gset, std::string site, base_log spdlog = nullptr, base_data* data = nullptr, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME);
		~insprocesser() {
			if (_fins) delete _fins;
			if (_fcalib) delete _fcalib;
			if (_fcalibstd) delete _fcalibstd;
			if (_fkf) delete _fkf;
			if (_fpk) delete _fpk;
		}
		void time_update_flex(double kfts, int fback, double inflation);
		void set_Ft();
		void set_ins_out();
		void init_par();
		bool align_coarse();
		bool align_pva(const Triple& pos);
		bool align_vva(const Triple& vel);
		bool align_static();
		void merge_init(const Triple& pos, const Triple& lever);
		void set_posvel(Triple blh, Triple vn);
		void erase_bef(base_time t);
		bool load_data() override;
		int ProcessOneEpoch() override;
		int ProcessWithoutCovUpdate();
		void AddData(base_data* data) override {insdata = dynamic_cast<ins_data*>(data);};
		bool _init() override;
		void _addResidualBlocks(ceres::Problem& problem) override;
		void _addMarginInfo() override;
		void _fgo_vector_to_double() override { 
			baseprocesser::_fgo_vector_to_double(); 
		};
		void _fgo_double_to_vector() override { 
			baseprocesser::_fgo_double_to_vector(); 
		};
		void _feed_back(const base_posdata::data_pos& pos, const Triple RobustFixedPos);
		void MeasCrt();
		void _input_imu_data(const double& t, const std::vector<Eigen::Vector3d>& wm, const std::vector<Eigen::Vector3d>& vm);
		void slide_window() override {
			baseprocesser::slide_window();
		};
		MOTION_TYPE motion_state();
		MEAS_TYPE meas_state();
		void UpdateViewer();
		bool MimuMeas() { return _num_of_imu_axiliary > 0 && FuseType == STACK; }
		double dsec() { return _sins->t - int(_sins->t); }
		double _delay() { return _shm->delay; }
		void prt_sins(std::ostringstream& os);
		void write_sins(std::ostringstream& os) { if(_fins) _fins->write(os.str().c_str(), os.str().size()); }
		ins_data* insdata = nullptr;
		bool _aligned = false;

	private:
		std::vector<Triple> linear_acceleration_buf[(WINDOW_SIZE)];
		std::vector<Triple> angular_velocity_buf[(WINDOW_SIZE)];

		int nq;
		ImuState _imu_state;
		ins_publish _publisher;
		std::map<int, std::unique_ptr<ins_obj>> sins_mimu;
		std::map<int, ins_scheme> _shm_mimu;
		base_iof* _fins = nullptr;
		base_iof* _fcalib = nullptr;
		base_iof* _fcalibstd = nullptr;
		base_iof* _fkf = nullptr;
		base_iof* _fpk = nullptr;
		ins_fuse_mode FuseType;
		int _num_of_imu_axiliary;
		std::vector<Triple> vm_align;
		std::vector<Triple> wm_align;
		Triple wmm{ 0,0,0 }, vmm{0,0,0};
		int _align_count = 0;
		bool _first_align = true;
		Triple _first_pos, _pre_pos;
		double _yaw0 = 0.0;
		Matrix Ft;
		Matrix G;
		Triple initial_pos;
		Triple initial_vel;
		Triple initial_att;
		std::map<int, SO3> _R_imui_imu0;
		std::map<int, Triple> _p_imui_imu0;
		std::map<int, double> _t_imui_imu0;
		bool _estimate_imui_extrinsic;
		bool _estimate_imui_t;
		Triple _initial_extrinsic_rotation_cov;
		Triple _initial_extrinsic_translation_cov;
		double _initial_t_cov;
		double inflation;
		std::map<double, double> _map_yaw;
		ins_avar _avar;

	protected:
		bool R2P;
		Triple _enf_R_std;
		Triple _enf_p_std;
		std::map<int, std::vector<Triple>> _wm_mimu;
		std::map<int, std::vector<Triple>> _vm_mimu;
		std::unique_ptr<Triple> mean_ang_vel;
		std::unique_ptr<double> ts_p_c;
		int imu_frequency;
		int ImuSample;
		std::mutex ins_mutex;
	};
};

#endif
