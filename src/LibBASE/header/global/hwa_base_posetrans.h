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
	void move(int& a);
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
	};
}

#endif