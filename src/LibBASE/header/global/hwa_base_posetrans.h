#ifndef hwa_base_posetrans_h
#define hwa_base_posetrans_h
#include "hwa_base_quaternion.h"
#include "hwa_base_earth.h"

namespace hwa_base
{
	SO3 askew(const Triple& v);
	Eigen::Matrix4d m2m4(const Triple& v);
	Eigen::Matrix4d m2m4_(const Triple& v);
	SO3 Cen(const Triple& pos);
	void symmetry(Matrix& m);
	SO3 dGeod2Cart(const base_earth& eth, const Triple& blh);
	Triple product(const Triple& vec, const SO3& mat);
	void delrowcol(Matrix& M, int i);
	void delrow(Matrix& M, int i);
	void delcol(Matrix& M, int i);
	void delrow(Vector& V, int i);
	//void move(int& a);
	base_quat Qbase2eigen(Eigen::Quaterniond q);

	class base_att_trans
	{
	public:
		static SO3 a2mat(const Triple& att);
		static Triple m2att(const SO3& m);
		static base_quat a2qua(const Triple& att);
		static Triple q2att(const base_quat& qnb);
		static base_quat rv2q(const Triple& rv);
		static Triple q2rv(const base_quat& q);
		static base_quat m2qua(const SO3& Cnb);
		static SO3 q2mat(const base_quat& qnb);
		static SO3 rv2m(const Triple& rv);
		static SO3 dv2mat(const Triple& vb1, const Triple& vb2,
			const Triple& vn1, const Triple& vn2);
		template <typename Derived>
		static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived>& theta)
		{
			typedef typename Derived::Scalar Scalar_t;

			Eigen::Quaternion<Scalar_t> dq;
			Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
			half_theta /= static_cast<Scalar_t>(2.0);
			dq.w() = static_cast<Scalar_t>(1.0);
			dq.x() = half_theta.x();
			dq.y() = half_theta.y();
			dq.z() = half_theta.z();
			return dq;
		}
		template <typename Derived>
		static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived>& q)
		{
			Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
			ans << typename Derived::Scalar(0), -q(2), q(1),
				q(2), typename Derived::Scalar(0), -q(0),
				-q(1), q(0), typename Derived::Scalar(0);
			return ans;
		}
		template <typename Derived>
		static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived>& q)
		{
			Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
			Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
			ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
			ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
			return ans;
		}

		template <typename Derived>
		static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived>& p)
		{
			Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
			Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
			ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
			ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
			return ans;
		}

		template <typename Derived>
		static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived>& q)
		{
			//printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
			//Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
			//printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
			//return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
			return q;
		}
	};
}

#endif