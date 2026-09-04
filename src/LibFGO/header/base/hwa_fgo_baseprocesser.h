#ifndef hwa_fgo_base_processer_h
#define hwa_fgo_base_processer_h

#include "hwa_ins_proc.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_allpar.h"
#include "hwa_base_mutex.h" 
#include "hwa_base_time.h"
#include "hwa_base_data.h"
#include "hwa_base_posdata.h"
#include "hwa_base_filter.h"
#include "hwa_base_preintegration.h"
#include "hwa_set_base.h"
#include "hwa_set_ign.h"
#include "hwa_set_fgo.h"

#include "hwa_fgo_factor_imu_preintegration.h"
#include "hwa_fgo_factor_initial_bias.h"
#include "hwa_fgo_factor_initial_pose.h"
#include "hwa_fgo_factor_marginalization.h"
#include "hwa_fgo_factor_pose_local_parameterization.h"

#define DELAY 0.001
#define WINDOW_SIZE 20
#define NUM_OF_F 1000
#define NUM_OF_ARC 10000  //for ambiguity arc 


using namespace hwa_base;
using namespace hwa_set;
using namespace hwa_ins;

namespace hwa_fgo {
    struct MEAS_INFO {
        double tmeas, MeasYaw, MeasVf, MeasHgt;        /// meas time,meas yaw,meas velocity foward,meas height
        Triple MeasVel, MeasPos, MeasAtt;  /// meas velosity and position  & attitude addwh
        Triple _Cov_MeasVn, _Cov_MeasPos, _Cov_MeasAtt;  /// meas velosity and position convariance
        Triple _Cov_MeasNHC, _Cov_MeasZUPT;
        double _Cov_MeasZIHR, _Cov_MeasOdo, _Cov_MeasYaw;
    };

    enum SENSOR_NODE {
        GNSS_NODE,
        UWB_NODE,
        VIS_NODE,
        LIDAR_NODE,
		DEFAULT_NODE
    };

    enum SOLVER_FLAG {
        INITIAL,
        NON_LINEAR
    };

    inline std::string sensor_node2str(SENSOR_NODE node) {
        switch (node)
        {
        case GNSS_NODE:
            return "GNSS";
        case UWB_NODE:
            return "UWB";
        case VIS_NODE:
            return "VIS";
        case LIDAR_NODE:
			return "LIDAR";
        case DEFAULT_NODE:
            return "DEFAULT";
        default:
            return "UNKNOWN";
        }
    }

    enum SIZE_PARAMETERIZATION
    {
        SIZE_POSE = 7,
        SIZE_SPEEDBIAS = 9,
        SIZE_FEATURE = 1,
        SIZE_AMB = 1,
        SIZE_CRD = 3,
        SIZE_SPEED = 3
    };


    inline void saveMatrix(const Matrix& M,
        const std::string& filename)
    {
        std::ofstream file(filename);

        for (int i = 0; i < M.rows(); i++)
        {
            for (int j = 0; j < M.cols(); j++)
            {
                file << M(i, j);

                if (j != M.cols() - 1)
                    file << ",";
            }

            file << "\n";
        }
    }

    class fgo_info {

    public:
        fgo_info() {};
        fgo_info(set_base* _set) {
            _pixel_error = dynamic_cast<set_fgopara*>(_set)->pixel_error();
            _laser_cloud_error = dynamic_cast<set_fgopara*>(_set)->laser_cloud_error();
            _acc_n = dynamic_cast<set_fgopara*>(_set)->acc_n();
            _acc_w = dynamic_cast<set_fgopara*>(_set)->acc_w();
            _gyr_n = dynamic_cast<set_fgopara*>(_set)->gyr_n();
            _gyr_w = dynamic_cast<set_fgopara*>(_set)->gyr_w();
            _relative_pos_var = dynamic_cast<set_fgopara*>(_set)->relative_pos_var();
            _relative_rot_var = dynamic_cast<set_fgopara*>(_set)->relative_rot_var();
            _window_size = dynamic_cast<set_fgopara*>(_set)->window_size();
        }
        bool time_to_margin() {
			return rover_count >= _window_size;
        }
        bool _get_imu_interval(double t0, double t1, std::vector<std::pair<double, Eigen::Vector3d>>& accVector,
            std::vector<std::pair<double, Eigen::Vector3d>>& gyrVector)
        {
            if (_accBuf.empty())
            {
                printf("not receive imu\n");
                return false;
            }

            while (_accBuf.front().first <= t0)
            {
                _accBuf.pop();
                _gyrBuf.pop();
            }
            while (_accBuf.front().first < t1)
            {
                accVector.push_back(_accBuf.front());
                _accBuf.pop();
                gyrVector.push_back(_gyrBuf.front());
                _gyrBuf.pop();
                if (_accBuf.empty() || _gyrBuf.empty())  break;
            }
            if (!_accBuf.empty() && !_gyrBuf.empty())
            {
                accVector.push_back(_accBuf.front());
                gyrVector.push_back(_gyrBuf.front());
            }

            assert(accVector.size() == gyrVector.size());
            return true;
        }

        bool _pre_integration(double t1, double t2)
        {
            if (t1 < 0 || t1 > t2) return false;

            std::vector<std::pair<double, Eigen::Vector3d>> accVector, gyrVector;
            if (!_get_imu_interval(t1, t2, accVector, gyrVector)) return false;

            for (size_t i = 0; i < accVector.size(); i++)
            {
                double dt;
                if (i == 0)
                    dt = accVector[i].first - t1;
                else if (i == accVector.size() - 1)
                    dt = t2 - accVector[i - 1].first;
                else
                    dt = accVector[i].first - accVector[i - 1].first;

                double imu_ti = accVector[i].first;

                Eigen::Vector3d linear_acceleration = accVector[i].second;
                Eigen::Vector3d angular_velocity = gyrVector[i].second;

                if (!_first_imu)
                {
                    _first_imu = true;
                    _acc_0 = linear_acceleration;
                    _gyr_0 = angular_velocity;
                }
                if (!_pre_integrations[rover_count])
                {
                    _pre_integrations[rover_count] = new IntegrationBase{ _acc_0, _gyr_0, _Bas[rover_count], _Bgs[rover_count] };
                    _pre_integrations[rover_count]->init_ins(_acc_n, _acc_w, _gyr_n, _gyr_w, _gravity);
                }
                if (rover_count != 0)
                {
                    _pre_integrations[rover_count]->push_back(dt, linear_acceleration, angular_velocity);
                    _dt_buf[rover_count].push_back(dt);
                    _linear_acceleration_buf[rover_count].push_back(linear_acceleration);
                    _angular_velocity_buf[rover_count].push_back(angular_velocity);
                }
                _acc_0 = linear_acceleration;
                _gyr_0 = angular_velocity;
            }

            return true;
        }
        void slide_window() {
            if (rover_count == _window_size)
            {
                for (int i = 0; i < rover_count - 1; i++)
                {
                    _Time[i] = _Time[i + 1];
                    _Rs[i].swap(_Rs[i + 1]);
                    _Ps[i].swap(_Ps[i + 1]);

                    std::swap(_pre_integrations[i], _pre_integrations[i + 1]);

                    _dt_buf[i].swap(_dt_buf[i + 1]);
                    _linear_acceleration_buf[i].swap(_linear_acceleration_buf[i + 1]);
                    _angular_velocity_buf[i].swap(_angular_velocity_buf[i + 1]);

                    _Vs[i].swap(_Vs[i + 1]);
                    _Bas[i].swap(_Bas[i + 1]);
                    _Bgs[i].swap(_Bgs[i + 1]);
                }

                _Rs[rover_count - 1] = _Rs[rover_count - 2];
                _Ps[rover_count - 1] = _Ps[rover_count - 2];
                _Vs[rover_count - 1] = _Vs[rover_count - 2];
                _Bas[rover_count - 1] = _Bas[rover_count - 2];
                _Bgs[rover_count - 1] = _Bgs[rover_count - 2];

                if(_pre_integrations[rover_count - 1])
                    delete _pre_integrations[rover_count - 1];
                _pre_integrations[rover_count - 1] = new IntegrationBase{ _acc_0, _gyr_0, _Bas[rover_count - 1], _Bgs[rover_count - 1] };
                _pre_integrations[rover_count - 1]->init_ins(_acc_n, _acc_w, _gyr_n, _gyr_w, _gravity);
                _dt_buf[rover_count - 1].clear();
                _linear_acceleration_buf[rover_count - 1].clear();
                _angular_velocity_buf[rover_count - 1].clear();


                while (_map_motion.rbegin()->first - _map_motion.begin()->first > 60 * 10)
                {
                    _map_motion.erase(std::begin(_map_motion));
                }

                rover_count--;
            }
        }

        void printSlidingWindowStates() const
        {
            std::cout << std::fixed << std::setprecision(10);

            std::cout << "\n================ Sliding Window States ================\n";

            for (int i = 0; i < rover_count; ++i)
            {
                std::cout << "Frame [" << i << "]\n";

                std::cout << "  P  = ("
                    << _para_pose[i][0] << ", "
                    << _para_pose[i][1] << ", "
                    << _para_pose[i][2] << ")\n";

                std::cout << "  Q  = ("
                    << _para_pose[i][3] << ", "
                    << _para_pose[i][4] << ", "
                    << _para_pose[i][5] << ", "
                    << _para_pose[i][6] << ")\n";

                std::cout << "  V  = ("
                    << _para_speed_bias[i][0] << ", "
                    << _para_speed_bias[i][1] << ", "
                    << _para_speed_bias[i][2] << ")\n";

                std::cout << "  Ba = ("
                    << _para_speed_bias[i][3] << ", "
                    << _para_speed_bias[i][4] << ", "
                    << _para_speed_bias[i][5] << ")\n";

                std::cout << "  Bg = ("
                    << _para_speed_bias[i][6] << ", "
                    << _para_speed_bias[i][7] << ", "
                    << _para_speed_bias[i][8] << ")\n";

                std::cout << "-------------------------------------------------------\n";
            }

            std::cout << "=======================================================\n";
        }

    public:
        std::map<SENSOR_NODE, std::vector<int>> _Node_index;
        Eigen::Vector3d        _Ps[(WINDOW_SIZE)];							///< position of frame
        Eigen::Vector3d        _Vs[(WINDOW_SIZE)];							///< velocity of frame
        Eigen::Matrix3d        _Rs[(WINDOW_SIZE)];							///< rotation of frame
        Eigen::Vector3d        _Bas[(WINDOW_SIZE)];						///< bias of acceleration
        Eigen::Vector3d        _Bgs[(WINDOW_SIZE)];
		double  		       _Time[(WINDOW_SIZE)];						///< time of frame
        IntegrationBase* _pre_integrations[(WINDOW_SIZE)] = {};
        std::vector<double> _dt_buf[(WINDOW_SIZE)];
        std::vector<Eigen::Vector3d> _linear_acceleration_buf[(WINDOW_SIZE)];	///< buffer to store linear_acceleration info
        std::vector<Eigen::Vector3d> _angular_velocity_buf[(WINDOW_SIZE)];
        std::map<base_time, MOTION_TYPE> _map_motion;
        std::map<base_time, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> _map_pose;
        bool new_state_inserted = false;
        double _cur_node_time = -1;
        double _last_pre_integration_time = -1;             ///last pre integration  time
        double cost = 0;

    public:
        std::map<long, double*> addr_shift;
        MarginalizationInfo* marginalization_info = nullptr;
        MarginalizationInfo* _last_marginalization_info = nullptr;			/// info of last marginalized frame
        std::vector<double*> _last_marginalization_parameter_blocks;	/// block to store info of last marginalized parameters
        /// parameters  blocks for Ceres solver 
        double _para_pose[WINDOW_SIZE][SIZE_POSE];				///< pose 
        double _para_speed_bias[WINDOW_SIZE][SIZE_SPEEDBIAS];	///< speed and bias
        double _para_feature[NUM_OF_F][SIZE_FEATURE];				///< feature inverse depth
        double _para_ex_pose[2][SIZE_POSE];							///< external pose 
        double _para_retrive_pose[SIZE_POSE];						///< unused
        double _para_td[1][1];										///< time delay
        double _para_tr[1][1];										///< unused    

        std::vector<double*> t_array; ///<for global pose graph optimization,the translation variables are stored in it
        std::vector<double*> q_array; ///<for global pose graph optimization,the rotation variables are stored in it
        double _para_CRD[WINDOW_SIZE][SIZE_CRD];                     ///<ECEF coordinate (XYZ) in sliding window
        double _para_SPEED[WINDOW_SIZE][SIZE_SPEED];                 ///<ECEF Velocity (Vx,Vy,Vz) in sliding window
        double _para_amb[NUM_OF_ARC][SIZE_AMB];                    ///carrier-phase ambiguity arc tracked in sliding window 

        Eigen::Vector3d _acc_0, _gyr_0;
        std::queue<std::pair<double, Eigen::Vector3d>> _accBuf;
        std::queue<std::pair<double, Eigen::Vector3d>> _gyrBuf;

        int rover_count = 0;
        double _pixel_error = 1.0;				///< error of pixel
        double _laser_cloud_error = 1.0;			///< error of laser cloud
        double _acc_n = 0.05;						///< accelerometer measurement noise
        double _acc_w = 0.001;						///< accelerometer bias random work noise
        double _gyr_n = 0.005;						///< gyroscope measurement noise
        double _gyr_w = 0.0001;						///< gyroscope bias random work noise
        double _relative_pos_var = 0.1;			///< relative position variance
        double _relative_rot_var = 0.001;			///< relative rotation variance		
        int _window_size = 10;
        Eigen::Vector3d _gravity;				///< gravity
        bool _first_imu = false;
		SOLVER_FLAG _solver_flag = INITIAL;
    };

    class baseprocesser {
    public:
        baseprocesser() {};
        explicit baseprocesser(const baseprocesser& B): 
        _gset(B._gset), _sins(B._sins), param_of_sins(B.param_of_sins),
        _spdlog(B._spdlog), _fgo_info(B._fgo_info), _name(B._name), TimeStamp(B.TimeStamp), beg(B.beg),
        end(B.end), _shm(B._shm) 
        {
        };
        explicit baseprocesser(const baseprocesser& B, SENSOR_NODE node_type) :
            _gset(B._gset), _fgo_info(B._fgo_info), _sins(B._sins), param_of_sins(B.param_of_sins),
            _spdlog(B._spdlog), _name(B._name), TimeStamp(B.TimeStamp), beg(B.beg),
			end(B.end), _shm(B._shm), _node(node_type)
        {
        };
        explicit baseprocesser(std::shared_ptr<set_base> gset, base_log spdlog, std::string name, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME) :
            _gset(gset), _spdlog(spdlog), _name(name), beg(_beg), end(_end), TimeStamp(_beg),
            _sins(std::make_shared<hwa_ins::ins_obj>(gset.get())),
            _shm(std::make_shared<hwa_ins::ins_scheme>(gset.get())),
            _fgo_info(std::make_shared<fgo_info>(gset.get())),
            param_of_sins(std::make_shared<base_allpar>())
        {
        };
        explicit baseprocesser(std::shared_ptr<set_base> gset, base_log spdlog, std::string name, SENSOR_NODE node_type, base_time _beg = FIRST_TIME, base_time _end = LAST_TIME) :
            _gset(gset), _spdlog(spdlog), _name(name), beg(_beg), end(_end), TimeStamp(_beg), _node(node_type),
            _sins(std::make_shared<hwa_ins::ins_obj>(gset.get())),
            _shm(std::make_shared<hwa_ins::ins_scheme>(gset.get())),
			_fgo_info(std::make_shared<fgo_info>(gset.get())),
            param_of_sins(std::make_shared<base_allpar>())
        {
        };
        ~baseprocesser() {};
        double dTime() { return TimeStamp.sow() + TimeStamp.dsec(); };
        base_time& Time() { return TimeStamp;}
        base_time& _beg() { return beg; }
        base_time& _end() { return end; }
        virtual int ProcessOneEpoch() { return 1; };
        virtual void AddData(base_data* data) {};
        virtual void timesynchronization(base_time t) {};
        virtual bool _time_valid(base_time time) { return true; };
        virtual MEAS_TYPE _getPOS(base_time inst, base_posdata::data_pos& pos, MEAS_INFO& m) { return NO_MEAS; };
        virtual bool _init() { return true; };
        virtual bool load_data() { return true; };
        virtual void _addResidualBlocks(ceres::Problem& problem) {};
        virtual void _addMarginInfo() {};
        template <class T1, class T2>
        static void m_out(T1 const& name, T2 const& matrix)
        {
            std::cout << name << std::endl;
            std::cout << std::fixed << std::setprecision(6) << std::setw(15) << matrix << std::endl;
            return;
        }
        bool timecheck() {
            return TimeStamp <= end && TimeStamp >= beg;
        };
        bool check_same_node() {
            double curr_t = dTime();
            if (_fgo_info->rover_count != 0 && fabs(curr_t - _fgo_info->_Time[_fgo_info->rover_count - 1]) < 0.0025) 
                return true;
            return false;
        }
        Matrix _getPx() {
            return _sins->Pk;
		} 
        bool new_node_inserted(){ 
            return _fgo_info->new_state_inserted;
        }
        bool _time_to_margin() {
			return _fgo_info->time_to_margin();
        }
        void reset_status(){
            _fgo_info->new_state_inserted = false;
        }
        void set_solver_flag(SOLVER_FLAG flag) {
            _fgo_info->_solver_flag = flag;
		}
        SOLVER_FLAG _get_solver_flag() {
            return _fgo_info->_solver_flag;
		}

        void _set_frame_pose()
        {
            if (check_same_node()) {
                _fgo_info->_Node_index[_node].push_back(_fgo_info->rover_count - 1);
                return;
            }
            _fgo_info->_Node_index[_node].push_back(_fgo_info->rover_count);

            Eigen::Quaterniond e_q = Eigen::Quaterniond(_sins->qeb.q0, _sins->qeb.q1, _sins->qeb.q2, _sins->qeb.q3);
            e_q.normalized();
            _fgo_info->_Rs[_fgo_info->rover_count] = e_q.toRotationMatrix();
            _fgo_info->_Ps[_fgo_info->rover_count] = _sins->pos_ecef;
            _fgo_info->_Vs[_fgo_info->rover_count] = _sins->ve;
            _fgo_info->_Bas[_fgo_info->rover_count] = _sins->db;
            _fgo_info->_Bgs[_fgo_info->rover_count] = _sins->eb;
            _fgo_info->_Time[_fgo_info->rover_count] = dTime();
            _fgo_info->new_state_inserted = true;

            _fgo_info->_pre_integration(_fgo_info->_last_pre_integration_time, dTime() + _fgo_info->_para_td[0][0]);
            _fgo_info->_last_pre_integration_time = dTime() + _fgo_info->_para_td[0][0];

            _fgo_info->rover_count++;
        }
        virtual void _fgo_vector_to_double()
        {
            for (int i = 0; i <= _fgo_info->rover_count - 1; i++)
            {
                _fgo_info->_para_pose[i][0] = _fgo_info->_Ps[i].x();
                _fgo_info->_para_pose[i][1] = _fgo_info->_Ps[i].y();
                _fgo_info->_para_pose[i][2] = _fgo_info->_Ps[i].z();

                Eigen::Quaterniond q{ _fgo_info->_Rs[i] };
                _fgo_info->_para_pose[i][3] = q.x();
                _fgo_info->_para_pose[i][4] = q.y();
                _fgo_info->_para_pose[i][5] = q.z();
                _fgo_info->_para_pose[i][6] = q.w();

                _fgo_info->_para_speed_bias[i][0] = _fgo_info->_Vs[i].x();
                _fgo_info->_para_speed_bias[i][1] = _fgo_info->_Vs[i].y();
                _fgo_info->_para_speed_bias[i][2] = _fgo_info->_Vs[i].z();

                _fgo_info->_para_speed_bias[i][3] = _fgo_info->_Bas[i].x();
                _fgo_info->_para_speed_bias[i][4] = _fgo_info->_Bas[i].y();
                _fgo_info->_para_speed_bias[i][5] = _fgo_info->_Bas[i].z();

                _fgo_info->_para_speed_bias[i][6] = _fgo_info->_Bgs[i].x();
                _fgo_info->_para_speed_bias[i][7] = _fgo_info->_Bgs[i].y();
                _fgo_info->_para_speed_bias[i][8] = _fgo_info->_Bgs[i].z();
            }
        }

        virtual void _fgo_double_to_vector()
        {
            for (int i = 0; i <= _fgo_info->rover_count - 1; i++)
            {
                _fgo_info->_Rs[i] = Eigen::Quaterniond(_fgo_info->_para_pose[i][6], _fgo_info->_para_pose[i][3], _fgo_info->_para_pose[i][4], _fgo_info->_para_pose[i][5]).normalized().toRotationMatrix();
                _fgo_info->_Ps[i] = Eigen::Vector3d(_fgo_info->_para_pose[i][0], _fgo_info->_para_pose[i][1], _fgo_info->_para_pose[i][2]);
                _fgo_info->_Vs[i] = Eigen::Vector3d(_fgo_info->_para_speed_bias[i][0],
                    _fgo_info->_para_speed_bias[i][1],
                    _fgo_info->_para_speed_bias[i][2]);

                _fgo_info->_Bas[i] = Eigen::Vector3d(_fgo_info->_para_speed_bias[i][3],
                    _fgo_info->_para_speed_bias[i][4],
                    _fgo_info->_para_speed_bias[i][5]);

                _fgo_info->_Bgs[i] = Eigen::Vector3d(_fgo_info->_para_speed_bias[i][6],
                    _fgo_info->_para_speed_bias[i][7],
                    _fgo_info->_para_speed_bias[i][8]);
            }
        }
        void printSlidingWindowStates() const {
			_fgo_info->printSlidingWindowStates();
        }
        void motion_insert(base_time time, MOTION_TYPE T) {
            _fgo_info->_map_motion.insert(std::make_pair(time, T));
        }
        virtual void slide_window() {
            _fgo_info->slide_window();
        }
        const std::vector<int>& _node_index_copy() {
            return _fgo_info->_Node_index[_node];
        }
        std::shared_ptr<fgo_info> _fgo() { return _fgo_info; }
        std::vector<int>& _node_index() {
            return _fgo_info->_Node_index[_node];
        }
        SENSOR_NODE _get_node_type() {
            return _node;
		}

    protected:
        std::shared_ptr<set_base> _gset;
        std::shared_ptr<ins_obj> _sins;
        std::shared_ptr<ins_scheme> _shm;
        std::shared_ptr<base_allpar> param_of_sins;
		std::shared_ptr<fgo_info> _fgo_info;
        std::string _name;
        SENSOR_NODE _node = DEFAULT_NODE;
        base_time TimeStamp = FIRST_TIME;
        base_time beg = FIRST_TIME;
        base_time end = LAST_TIME;
        base_log _spdlog;
    };
}


#endif
