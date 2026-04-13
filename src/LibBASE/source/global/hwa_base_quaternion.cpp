#include "hwa_base_posetrans.h"

using namespace hwa_base;

base_quat::base_quat(double q0, double q1, double q2, double q3)
{
    this->q0 = q0; this->q1 = q1;
    this->q2 = q2; this->q3 = q3;
}

base_quat base_quat::Identity() {
    return base_quat(1, 0, 0, 0);

}

base_quat::base_quat(const Eigen::Vector4d& m)
{
    q0 = m(0), q1 = m(1), q2 = m(2), q3 = m(3);
}

int base_quat::sign(double d) const
{
    if (d > glv.EPS)
        return 1;
    else if (d < -glv.EPS)
        return -1;
    else
        return 0;
}

base_quat base_quat::operator+(const base_quat& q) const
{
    return base_quat(q0 + q.q0, q1 + q.q1, q2 + q.q2, q3 + q.q3);
}

base_quat base_quat::operator+(const Triple& phi) const
{
    base_quat qtmp = hwa_base::base_att_trans::rv2q(-phi);
    return qtmp * (*this);
}

base_quat base_quat::operator-(const Triple& phi) const
{
    base_quat qtmp = hwa_base::base_att_trans::rv2q(phi);
    return qtmp * (*this);
}

base_quat base_quat::operator*(const base_quat& q) const
{
    base_quat qtmp;
    qtmp.q0 = q0 * q.q0 - q1 * q.q1 - q2 * q.q2 - q3 * q.q3;
    qtmp.q1 = q0 * q.q1 + q1 * q.q0 + q2 * q.q3 - q3 * q.q2;
    qtmp.q2 = q0 * q.q2 + q2 * q.q0 + q3 * q.q1 - q1 * q.q3;
    qtmp.q3 = q0 * q.q3 + q3 * q.q0 + q1 * q.q2 - q2 * q.q1;
    return qtmp;
}

Triple base_quat::operator-(const base_quat& q0) const
{
    base_quat dq = q0 * (base_quat::conj(*this));
    if (dq.q0 < 0)
    {
        dq.q0 = -dq.q0, dq.q1 = -dq.q1, dq.q2 = -dq.q2, dq.q3 = -dq.q3;
    }
    double n2 = acos(dq.q0), f;
    if (sign(n2) != 0)
    {
        f = 2.0 / (sin(n2) / n2);
    }
    else
    {
        f = 2.0;
    }
    return Triple(dq.q1, dq.q2, dq.q3) * f;
}

Triple base_quat::operator*(const Triple& v) const
{
    base_quat qtmp;
    Triple vtmp;
    qtmp.q0 = -q1 * v(0) - q2 * v(1) - q3 * v(2);
    qtmp.q1 = q0 * v(0) + q2 * v(2) - q3 * v(1);
    qtmp.q2 = q0 * v(1) + q3 * v(0) - q1 * v(2);
    qtmp.q3 = q0 * v(2) + q1 * v(1) - q2 * v(0);
    vtmp(0) = -qtmp.q0 * q1 + qtmp.q1 * q0 - qtmp.q2 * q3 + qtmp.q3 * q2;
    vtmp(1) = -qtmp.q0 * q2 + qtmp.q2 * q0 - qtmp.q3 * q1 + qtmp.q1 * q3;
    vtmp(2) = -qtmp.q0 * q3 + qtmp.q3 * q0 - qtmp.q1 * q2 + qtmp.q2 * q1;
    return vtmp;
}

base_quat& base_quat::operator*=(const base_quat& q)
{
    return (*this = *this * q);
}

void base_quat::normlize(base_quat& q)
{
    double nq = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    q.q0 /= nq, q.q1 /= nq, q.q2 /= nq, q.q3 /= nq;
}

base_quat base_quat::normalize(const base_quat& q)
{
    base_quat qq;
    double nq = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    qq.q0 = q.q0 / nq; qq.q1 = q.q1 / nq; qq.q2 = q.q2 / nq; qq.q3 = q.q3 / nq;
    return qq;
}

base_quat base_quat::inverse(const base_quat& q)
{
    base_quat qq = base_quat::normalize(base_quat::conj(q));
    return qq;
}

void base_quat::fabs(base_quat& q)
{
    if (q.q0 < 0)
    {
        q.q0 = -q.q0, q.q1 = -q.q1, q.q2 = -q.q2, q.q3 = -q.q3;
    }
}

base_quat base_quat::conj(const base_quat& q)
{
    return base_quat(q.q0, -q.q1, -q.q2, -q.q3);
}
