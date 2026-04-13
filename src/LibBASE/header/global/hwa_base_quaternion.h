#ifndef hwa_base_quaternion_h
#define hwa_base_quaternion_h
#include "hwa_base_eigendef.h"
#include "hwa_base_const.h"

namespace hwa_base
{
    class base_quat
    {
    public:
        base_quat(double q0 = 1.0, double q1 = 0.0, double q2 = 0.0, double q3 = 0.0);
        base_quat(const Eigen::Vector4d& m);
        int sign(double d) const;
        static base_quat Identity();
        base_quat operator+(const base_quat& q) const;
        base_quat operator+(const Triple& phi)const;
        base_quat operator-(const Triple& phi) const;
        base_quat operator*(const base_quat& q) const;
        Triple operator-(const base_quat& base_quat) const;
        Triple operator*(const Triple& v) const;
        base_quat& operator*=(const base_quat& q);
        static void normlize(base_quat& q);
        static base_quat normalize(const base_quat& q);
        static base_quat inverse(const base_quat& q);
        static void fabs(base_quat& q);
        static base_quat conj(const base_quat& q);
    public:
        double q0, q1, q2, q3;
    };

    const base_quat qI = base_quat();
}

#endif