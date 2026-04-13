#ifndef hwa_qt_insworker_h
#define hwa_qt_insworker_h
#include "hwa_ins_coder_serialreader.h"
#include "hwa_qt_global.h"
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QLabel>
#include <QTimer>
#include <atomic>
#include <vector>
#include <string>
#include <thread>
#include <memory>
#include <mutex>                           
#include <chrono>     
#include <fstream>
#include <iostream>
#include <sstream>
#include "hwa_set_ins.h"
#include "hwa_set_proc.h"
#include "hwa_set_out.h"
#include "hwa_set_ign.h"
#include "hwa_set_vis.h"
#include "hwa_set_tracker.h"
#include "hwa_base_filter.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_allpar.h"
#include "hwa_ins_proc.h"
#include "hwa_ins_proc_publish.h"
#ifndef Q_MOC_RUN
#include "hwa_vis_base.h"
#endif

using namespace hwa_base;
using namespace hwa_ins;

namespace hwa_qt {

	class qt_ins_worker : public QObject {
		Q_OBJECT
	public:
		qt_ins_worker(std::shared_ptr<hwa_set::set_base> _gset, std::string _dir, QObject* parent = nullptr);
		~qt_ins_worker() override;
		void _stop() { stop = true; }

	public slots:
		void AcceptSharedCamAttr(std::shared_ptr<hwa_vis::vis_imgproc_base> _CamAttr, int _ID);
		void Accept_tracker(_OneBatch Res);
		void Accept_vis(int cam_id, bool isStatic, double timestamp, const Matrix Hk1, const Vector Zk1, const Matrix Hk2, const Vector Zk2);
		void startWork();
		void stopWork();

	signals:
		void NewPose(double timestamp, Eigen::Quaterniond q, Triple t);
		void NewIns(Mstate _ins_states, Matrix global_variance);
		void NewAngleVel(double timestamp, Triple vel);

	protected:
		void InitState();
		void oneCycle();
		void set_Ft();
		bool _ins_init();
		void init_par();
		bool align_static();
		bool align_tracker();
		int _merge_init();
		void set_out();
		int _cam_feedback(const Vector& dx);
		void StateAugmentation();
		void PruneOld();
		bool _extrinsic_init(int cam_group_id);
		void Update(const std::map<int, std::vector<Triple>>& _wm_mimu, const std::map<int, std::vector<Triple>>& _vm_mimu, const std::map<int, hwa_ins::ins_scheme>& _scm_mimu);
		void UpdateError();
		void SetVisMeas(int cam_id, double timestamp, Matrix Hk, Vector Zk);
		void ZUPT_Update(double timestamp);
		void feedback(double timestamp);
		SO3 askew(const Triple& v);
		Eigen::Quaterniond I2E(hwa_base::base_quat q);
		hwa_base::base_quat E2I(Eigen::Quaterniond q);
		Eigen::Quaterniond V2Q(const Triple& rv);
		Eigen::Quaterniond M2Q(SO3 R);
		double floor1w(double t);
		double floor2w(double t);
		bool Iequal(double t1, double t2);
		void time_update_flex(double kfts, int fback, double inflation);

	private:
		bool m_running = false;
		std::atomic<bool> stop{ false };
		QTimer* cycleTimer = nullptr;
		std::shared_ptr<hwa_set::set_base> gset;
		std::map<int, std::unique_ptr<ins_reader>> readers;
		std::map<double, _OneIns> inswindow;
		std::string dir;
		std::queue<imu_unit> imucoder;
		std::map<int, std::vector<Triple>> _wm;
		std::map<int, std::vector<Triple>> _vm;
		std::map<int, hwa_ins::ins_scheme> _ins_scheme;
		std::shared_ptr<hwa_ins::ins_obj> sins;
		double basetime;
		double ts;
		bool aligned;
		bool aligned_track;
		hwa_base::base_updater updater;
		hwa_ins::ins_publish _publisher;
		int nq;
		int num_of_ins;
		hwa_ins::ins_scheme _shm;
		std::map<int, std::unique_ptr<hwa_ins::ins_obj>> sins_mimu;
		std::map<int, hwa_ins::ins_scheme> _shm_mimu;
		hwa_base::base_allpar param_of_sins;
		hwa_base::base_allpar _param;
		hwa_set::set_base* _set;
		hwa_base::base_iof* _fins;
		hwa_base::base_iof* _fcalib;
		hwa_base::base_iof* _fcalibstd;
		hwa_base::base_iof* _fkf;
		hwa_base::base_iof* _fpk;
		hwa_ins::ins_fuse_mode FuseType;
		Estimator EstimatorType;
		std::string _name;
		int _num_of_imu_axiliary;
		std::vector<Triple> vm_align;
		std::vector<Triple> wm_align;
		Triple inipos;
		Matrix Ft;
		Matrix G;
		Triple initial_pos_ecef;
		std::map<int, SO3> _R_imui_imu0;
		std::map<int, Triple> _p_imui_imu0;
		std::map<int, double> _t_imui_imu0;
		bool _estimate_imui_extrinsic;
		bool _estimate_imui_t;
		Triple _initial_extrinsic_rotation_cov;
		Triple _initial_extrinsic_translation_cov;
		double _initial_t_cov;
		hwa_ins::MEAS_TYPE Flag;
		double inflation;
		bool R2P;
		Triple _enf_R_std;
		Triple _enf_p_std;
		std::unique_ptr<Triple> mean_ang_vel;
		std::unique_ptr<double> ts_p_c;
		std::mutex ins_mutex;
		std::mutex window_mutex;
		std::vector<std::thread> threads;
		std::vector<int> cam_list;
		std::map<int, std::shared_ptr<hwa_vis::vis_imgproc_base>> CamAttr;

		SO3 R_t_i;
		Triple t_t_i;
		double Tracker_Rot_noise;
		double Tracker_Trans_noise;

		std::mutex EmitMutex;
		std::queue<_OneBatch> EmitQueue;
		SO3 R_r_e;
		Eigen::Quaterniond q_r_e;
		Triple t_r_e;
		double interval;
		double vis_interval;
		int frame_count;
		Mstate imu_states;
		int vis_size;
		double ZUPT_Noise;
		double VIS_Noise;
		int WindowSize;
		Matrix MSCKF_Hk;
		Matrix MSCKF_Rk;
		Vector MSCKF_Zk;
	};
}

#endif