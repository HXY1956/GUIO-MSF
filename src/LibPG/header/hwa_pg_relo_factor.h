#ifndef HWA_PG_RELO_FACTOR_H
#define HWA_PG_RELO_FACTOR_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace hwa_pg {

// Local parameterization for a 7-dim SE3 pose [x,y,z,qx,qy,qz,qw]
// with right-multiplied quaternion perturbation (same convention as the FGO).
class PGQuatLocalParameterization : public ceres::LocalParameterization {
public:
    virtual ~PGQuatLocalParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
        Eigen::Map<const Eigen::Vector3d> p(x);
        Eigen::Map<const Eigen::Quaterniond> q(x + 3);
        Eigen::Map<const Eigen::Vector3d> dp(delta);
        Eigen::Quaterniond dq;
        dq.w() = 1.0;
        dq.x() = delta[3] / 2.0;
        dq.y() = delta[4] / 2.0;
        dq.z() = delta[5] / 2.0;
        Eigen::Map<Eigen::Vector3d> pp(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> pq(x_plus_delta + 3);
        pp = p + dp;
        pq = (q * dq).normalized();
        return true;
    }
    virtual bool ComputeJacobian(const double*, double* jacobian) const {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.setZero();
        j.topRows<6>().setIdentity();
        return true;
    }
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};

// Reprojection factor of a fixed 3D landmark (ECEF) into the current camera.
// Parameters: pose = [x,y,z,qx,qy,qz,qw] body->ECEF.
// This anchors the sliding window to the loop keyframe's map (fast relocalization).
struct ReloReprojectionError {
    ReloReprojectionError(const Eigen::Vector3d& p_w, const Eigen::Vector2d& obs,
                          const Eigen::Matrix3d& R_cb, const Eigen::Vector3d& t_cb)
        : p_w_(p_w), obs_(obs), R_cb_(R_cb), t_cb_(t_cb) {}

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(pose);
        Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);

        Eigen::Matrix<T, 3, 1> pw(static_cast<T>(p_w_[0]),
                                  static_cast<T>(p_w_[1]),
                                  static_cast<T>(p_w_[2]));
        Eigen::Matrix<T, 3, 1> d = pw - t;
        Eigen::Quaternion<T> vq(T(0), d[0], d[1], d[2]);
        Eigen::Quaternion<T> rq = q.conjugate() * vq * q;
        Eigen::Matrix<T, 3, 1> pb(rq.x(), rq.y(), rq.z());
        Eigen::Matrix<T, 3, 1> tcb(static_cast<T>(t_cb_[0]),
                                   static_cast<T>(t_cb_[1]),
                                   static_cast<T>(t_cb_[2]));
        Eigen::Matrix<T, 3, 1> pc = R_cb_.template cast<T>().transpose() * (pb - tcb);

        residuals[0] = pc[0] / pc[2] - static_cast<T>(obs_[0]);
        residuals[1] = pc[1] / pc[2] - static_cast<T>(obs_[1]);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p_w, const Eigen::Vector2d& obs,
                                       const Eigen::Matrix3d& R_cb, const Eigen::Vector3d& t_cb) {
        return new ceres::AutoDiffCostFunction<ReloReprojectionError, 2, 7>(
            new ReloReprojectionError(p_w, obs, R_cb, t_cb));
    }

    Eigen::Vector3d p_w_;
    Eigen::Vector2d obs_;
    Eigen::Matrix3d R_cb_;
    Eigen::Vector3d t_cb_;
};

} // namespace hwa_pg

#endif
