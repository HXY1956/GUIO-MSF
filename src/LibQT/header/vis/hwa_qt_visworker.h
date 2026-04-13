#ifndef hwa_qt_visworker_h
#define hwa_qt_visworker_h
#include "hwa_qt_global.h"
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QLabel>
#include <QTimer>
#include "hwa_set_base.h"
#include "hwa_set_proc.h"
#include "hwa_set_ign.h"
#include "hwa_base_iof.h"
#include "hwa_base_posetrans.h"
#include "hwa_base_filter.h"
#include "hwa_base_allpar.h"
#include "hwa_base_sharedresource.h"
#ifndef Q_MOC_RUN
#include "hwa_vis_coder_stereousb.h"
#include "hwa_vis_base.h"
#endif

using namespace hwa_ins;

namespace hwa_qt {
    class qt_vis_worker : public QObject {
        Q_OBJECT
    public:
        qt_vis_worker(hwa_set::set_base* gset, int ID, QObject* parent = nullptr);
        ~qt_vis_worker();
        static Mstate Imu2Cam(const Mstate& imu_states, const SO3 R_c_i, const Triple t_c_i);
        bool checkStaticMotion(const hwa_vis::PointCloud& msg);
        void addFeatureObservations(double timestamp, const hwa_vis::PointCloud& msg);
        void measurementJacobianEX(
            const int& cam_state_id,
            const int& feature_id,
            Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 6>& H_x_ex, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r);
        void measurementJacobian(
            const int& cam_state_id,
            const int& feature_id,
            Eigen::Matrix<double, 4, 6>& H_x, Eigen::Matrix<double, 4, 3>& H_f, Eigen::Vector4d& r);
        void measurementJacobianEX(
            const int& cam_state_id,
            const int& feature_id,
            Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 6>& H_x_ex, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r);
        void measurementJacobian(
            const int& cam_state_id,
            const int& feature_id,
            Eigen::Matrix<double, 2, 6>& H_x, Eigen::Matrix<double, 2, 3>& H_f, Eigen::Vector2d& r);
        bool featureJacobian(const int feature_id,
            const std::vector<int>& cam_state_ids,
            Matrix& H_x, Vector& r);
        bool RemoveLostFeatures(Matrix& H_x, Vector& r);
        bool PruneCamState(Matrix& H_x, Vector& r);
        bool GatingTest(const Matrix& H, const Vector& r, const int& dof);
        void meas_update(Matrix& H, Vector& r);
        void readChisquare_test();
        static std::set<int> JudgeArea(_OneFeature feature, std::vector<int> a);
        static Mstate ExtractFromMap(Mstate _cam_states, std::set<int> intersection);

    public slots:
        void Accept_pts(double timestamp, hwa_vis::PointCloud msg);
        void Accept_ins(Mstate _ins_states, Matrix Pk);
        void Accept_OpenStatus(bool flag);

    signals:
        void SharedCamAttr(std::shared_ptr<hwa_vis::vis_imgproc_base> CamAttr, int _ID);
        void NewUpdate(int cam_id, bool isStatic, double timestamp, Matrix Hk1 = Matrix::Zero(3, 3), Vector Zk1 = Vector::Zero(3, 1), Matrix Hk2 = Matrix::Zero(3, 3), Vector Zk2 = Vector::Zero(3, 1));

    protected:
        int vis_size;
        int cam_group_id;
        bool mIsFirstImg;
        Triple initCamPos;
        std::string _site;
        Estimator EstimatorType;
        double TimeStamp;
        double static_threshold;
        int imu_frequency;
        int cam_state_id;
        std::ofstream TimeCostDebugOutFile;
        bool TimeCostDebugStatus;
        bool isOpen = false;
        hwa_base::base_iof* _fcalib;
        _MapServer map_server;
        std::map<double, hwa_vis::PointCloud> Clouds;
        Mstate ins_states;
        Mstate cam_states;
        double vis_interval;
        std::shared_ptr<hwa_vis::vis_imgproc_base> CamAttr;
        std::map<int, double> chi_squared_test_table;
        Matrix _global_variance;
        std::mutex map_mtx;
    };
}

#endif