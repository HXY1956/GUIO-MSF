#ifndef hwa_qt_global_h
#define hwa_qt_global_h
#include "hwa_base_eigendef.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <set>
#include "hwa_base_sharedresource.h"

using namespace hwa_base;

namespace hwa_qt {
	using Mpts = std::shared_ptr<std::map<double, std::vector<cv::Point2f>>>;
	using MMpts = std::map<int, Mpts>;
	using Vpts = std::vector<cv::Point2f>;
	using ImgMap = std::shared_ptr<std::map<double, cv::Mat>>;

	extern std::map<int, double> translation_threshold;        ///< transformation matrix threshold
	extern std::map<int, double> huber_epsilon;               ///< kernel function
	extern std::map<int, double> estimation_precision;   ///< threshold of whether the update quantity is iterative
	extern std::map<int, double>initial_damping;            ///< initial lambda of Levenberg-Marquart
	extern std::map<int, int> outler_loop_max_iteration;        ///< iterations
	extern std::map<int, int> inner_loop_max_iteration;         ///< iterations of depth
	extern std::map<int, SE3> T_cam0_cam1;     ///< transformation matrix between cam0 and cam1
	extern std::map<int, bool> stereo;                ///< stereo or num

	struct CamInfo
	{
		int ID;
		double ts;
		Eigen::Quaterniond orientation;
		Triple position;
	};

	using Mstate = std::map<double, CamInfo>;

	struct _OneBatch {
		Eigen::Quaterniond Q;
		Triple T;
		double Time;
		_OneBatch() {};
		_OneBatch(Eigen::Quaterniond _Q, Triple _T, double _Time) {
			Q = _Q;
			T = _T;
			Time = _Time;
		};
	};

	struct _PreIntegration {
		Triple delta_p;
		Triple delta_v;
		Eigen::Quaterniond delta_q;
		Matrix Ft;
		Matrix Qt;
		SO3 jacobian_bg;
		SO3 jacobian_ba;
		SO3 jacobian_pos_ba;
		SO3 jacobian_pos_bg;
		_PreIntegration() {
			delta_p = Triple::Zero();
			delta_v = Triple::Zero();
			delta_q = Eigen::Quaterniond::Identity();
			Ft = Matrix::Identity(15, 15);
			Qt = Matrix::Zero(15, 15);
			jacobian_bg = SO3::Zero(3, 3);
			jacobian_ba = SO3::Zero(3, 3);
			jacobian_pos_ba = SO3::Zero(3, 3);
			jacobian_pos_bg = SO3::Zero(3, 3);
		}
	};

	struct _OneIns {
		Matrix global_variance;
		Vector Xk;
		Eigen::Quaterniond qeb;
		Vector ve;
		Triple pos_ecef;
		Triple gcc;
		Matrix Cen;
		int nq;
		double ts;
		_PreIntegration Attribute;
		_OneIns() {};
		_OneIns(int _nq, Matrix _global_variance, Eigen::Quaterniond _qeb, Triple _pos_ecef, Vector _ve, Triple _gcc, Matrix _Cen) {
			ts = 0;
			nq = _nq;
			global_variance = _global_variance;
			qeb = _qeb;
			pos_ecef = _pos_ecef;
			ve = _ve;
			gcc = _gcc;
			Cen = _Cen;
			Xk = Vector::Zero(nq);
		};
	};

	struct USBInformation {
		std::string devicepathlist;
		int framerate;
		std::pair<int, int> framesize;
		std::string usbname;
		std::string dir;
		int ID;
		int camskip;
		USBInformation(std::string _devicepathlist,
			int _framerate,
			std::pair<int, int> _framesize,
			std::string _usbname,
			std::string _dir,
			int _ID,
			int _camskip) {
			devicepathlist = _devicepathlist;
			framerate = _framerate;
			framesize = _framesize;
			usbname = _usbname;
			dir = _dir;
			ID = _ID;
			camskip = _camskip;
		};
		USBInformation() {};
	};


	class _OneFeature
	{
	public:
		bool triangulatePoint(const Mstate& cam_states) const;
		bool checkMotion(std::set<int> Intersection) const;
		bool checkMotion() const;
		void generateInitialGuess(
			const SE3& T_c1_c2, const Eigen::Vector2d& z1,
			const Eigen::Vector2d& z2, Triple& p) const;
		void cost(const SE3& T_c0_ci,
			const Triple& x, const Eigen::Vector2d& z,
			double& e) const;
		void jacobian(const SE3& T_c0_ci,
			const Triple& x, const Eigen::Vector2d& z,
			Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
			double& w) const;
		bool initializePosition(const Mstate& cam_states);

		std::map<int, Eigen::Vector4d, std::less<int>,
			Eigen::aligned_allocator<std::pair<const int, Eigen::Vector4d>>> observations;
		int id;
		int usbID;
		Triple position;
		bool is_initialized;
		bool is_initialized_NonKey;
		bool isLost = false;
	};

	typedef std::map<int, _OneFeature> _MapServer;
}

#endif