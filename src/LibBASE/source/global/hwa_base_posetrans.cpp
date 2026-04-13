#include "hwa_base_posetrans.h"

hwa_base::SO3 hwa_base::base_att_trans::a2mat(const Triple& att)
{
    double sp = sin(att(0)), cp = cos(att(0));
    double sr = sin(att(1)), cr = cos(att(1));
    double sy = sin(att(2)), cy = cos(att(2));

    SO3 m;
    m << cy * cr - sy * sp * sr, -sy * cp, cy* sr + sy * sp * cr,
        sy* cr + cy * sp * sr, cy* cp, sy* sr - cy * sp * cr,
        -cp * sr, sp, cp* cr;
    return m;
}

hwa_base::Triple hwa_base::base_att_trans::m2att(const SO3& m)
{
    Triple att;
    att(0) = asin(m(2, 1));
    att(1) = atan2(-m(2, 0), m(2, 2));
    att(2) = atan2(-m(0, 1), m(1, 1));
    return att;
}

hwa_base::base_quat hwa_base::base_att_trans::a2qua(const Triple& att)
{
    //double yaw = att(0) / 2.0, pitch = att(1) / 2.0, roll = att(2) / 2.0;
    double pitch = att(0) / 2.0, roll = att(1) / 2.0, yaw = att(2) / 2.0;
    double    sp = sin(pitch), sr = sin(roll), sy = sin(yaw),
        cp = cos(pitch), cr = cos(roll), cy = cos(yaw);
    base_quat qnb;
    qnb.q0 = cp * cr * cy - sp * sr * sy;
    qnb.q1 = sp * cr * cy - cp * sr * sy;
    qnb.q2 = cp * sr * cy + sp * cr * sy;
    qnb.q3 = cp * cr * sy + sp * sr * cy;
    return qnb;
}

hwa_base::Triple hwa_base::base_att_trans::q2att(const hwa_base::base_quat& qnb)
{
    double    q11 = qnb.q0 * qnb.q0, q12 = qnb.q0 * qnb.q1, q13 = qnb.q0 * qnb.q2, q14 = qnb.q0 * qnb.q3,
        q22 = qnb.q1 * qnb.q1, q23 = qnb.q1 * qnb.q2, q24 = qnb.q1 * qnb.q3,
        q33 = qnb.q2 * qnb.q2, q34 = qnb.q2 * qnb.q3,
        q44 = qnb.q3 * qnb.q3;
    Triple att;

    att(0) = asin(2 * (q34 + q12));
    att(1) = atan2(-2 * (q24 - q13), q11 - q22 - q33 + q44);
    att(2) = atan2(-2 * (q23 - q14), q11 - q22 + q33 - q44);

    return att;
}

hwa_base::base_quat hwa_base::base_att_trans::rv2q(const Triple& rv)
{
#define F1    (   2 * 1)    
#define F2    (F1*2 * 2)
#define F3    (F2*2 * 3)
#define F4    (F3*2 * 4)
#define F5    (F4*2 * 5)
    double c, f, n2 = rv.norm() * rv.norm();
    if (n2 < (glv.PI / 180.0 * glv.PI / 180.0))
    {
        double n4 = n2 * n2;
        c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
        f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
    }
    else
    {
        double n_2 = sqrt(n2) / 2.0;
        c = cos(n_2);
        f = sin(n_2) / n_2 * 0.5;
    }
    return base_quat(c, f * rv(0), f * rv(1), f * rv(2));
}

hwa_base::Triple hwa_base::base_att_trans::q2rv(const hwa_base::base_quat& q)
{
    base_quat dq;
    dq = q;
    if (dq.q0 < 0) { dq.q0 = -dq.q0, dq.q1 = -dq.q1, dq.q2 = -dq.q2, dq.q3 = -dq.q3; }
    if (dq.q0 > 1.0) dq.q0 = 1.0;
    double n2 = acos(dq.q0), f;
    if (n2 > 1.0e-20)
    {
        f = 2.0 / (sin(n2) / n2);
    }
    else
    {
        f = 2.0;
    }
    return Triple(dq.q1, dq.q2, dq.q3) * f;
}

hwa_base::base_quat hwa_base::base_att_trans::m2qua(const SO3& Cnb)
{
    double q0, q1, q2, q3, qq4;
    if (Cnb(0, 0) >= Cnb(1, 1) + Cnb(2, 2))
    {
        q1 = 0.5 * sqrt(1 + Cnb(0, 0) - Cnb(1, 1) - Cnb(2, 2));  qq4 = 4 * q1;
        q0 = (Cnb(2, 1) - Cnb(1, 2)) / qq4; q2 = (Cnb(0, 1) + Cnb(1, 0)) / qq4; q3 = (Cnb(0, 2) + Cnb(2, 0)) / qq4;
    }
    else if (Cnb(1, 1) >= Cnb(0, 0) + Cnb(2, 2))
    {
        q2 = 0.5 * sqrt(1 - Cnb(0, 0) + Cnb(1, 1) - Cnb(2, 2));  qq4 = 4 * q2;
        q0 = (Cnb(0, 2) - Cnb(2, 0)) / qq4; q1 = (Cnb(0, 1) + Cnb(1, 0)) / qq4; q3 = (Cnb(1, 2) + Cnb(2, 1)) / qq4;
    }
    else if (Cnb(2, 2) >= Cnb(0, 0) + Cnb(1, 1))
    {
        q3 = 0.5 * sqrt(1 - Cnb(0, 0) - Cnb(1, 1) + Cnb(2, 2));  qq4 = 4 * q3;
        q0 = (Cnb(1, 0) - Cnb(0, 1)) / qq4; q1 = (Cnb(0, 2) + Cnb(2, 0)) / qq4; q2 = (Cnb(1, 2) + Cnb(2, 1)) / qq4;
    }
    else
    {
        q0 = 0.5 * sqrt(1 + Cnb(0, 0) + Cnb(1, 1) + Cnb(2, 2));  qq4 = 4 * q0;
        q1 = (Cnb(2, 1) - Cnb(1, 2)) / qq4; q2 = (Cnb(0, 2) - Cnb(2, 0)) / qq4; q3 = (Cnb(1, 0) - Cnb(0, 1)) / qq4;
    }
    double nq = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;
    return base_quat(q0, q1, q2, q3);
}

hwa_base::SO3 hwa_base::base_att_trans::q2mat(const hwa_base::base_quat& qnb)
{
    double    q11 = qnb.q0 * qnb.q0, q12 = qnb.q0 * qnb.q1, q13 = qnb.q0 * qnb.q2, q14 = qnb.q0 * qnb.q3,
        q22 = qnb.q1 * qnb.q1, q23 = qnb.q1 * qnb.q2, q24 = qnb.q1 * qnb.q3,
        q33 = qnb.q2 * qnb.q2, q34 = qnb.q2 * qnb.q3,
        q44 = qnb.q3 * qnb.q3;
    SO3 Cnb;
    Cnb(0, 0) = q11 + q22 - q33 - q44, Cnb(0, 1) = 2 * (q23 - q14), Cnb(0, 2) = 2 * (q24 + q13),
        Cnb(1, 0) = 2 * (q23 + q14), Cnb(1, 1) = q11 - q22 + q33 - q44, Cnb(1, 2) = 2 * (q34 - q12),
        Cnb(2, 0) = 2 * (q24 - q13), Cnb(2, 1) = 2 * (q34 + q12), Cnb(2, 2) = q11 - q22 - q33 + q44;
    return Cnb;
}

hwa_base::SO3 hwa_base::base_att_trans::rv2m(const Triple& rv)
{
    double n2 = rv.norm(), a, b, n;
    if (n2 < (glv.PI / 180.0 * glv.PI / 180.0))
    {
        a = 1 - n2 * (1 / 6 - n2 / 120);
        b = 0.5 - n2 * (1 / 24 - n2 / 720);
    }
    else
    {
        n = sqrt(n2);
        a = sin(n) / n;
        b = (1 - cos(n)) / n2;
    }
    SO3 rx = askew(rv);
    SO3 DCM = SO3::Identity() + a * rx + b * rx * rx;
    return DCM;

}

hwa_base::SO3 hwa_base::base_att_trans::dv2mat(const Triple& vb1, const Triple& vb2, const Triple& vn1, const Triple& vn2)
{
    Triple vb = vb1.cross(vb2), vn = vn1.cross(vn2);
    Triple vbb = vb.cross(vb1), vnn = vn.cross(vn1);
    SO3 Mb, Mn;
    Mb.block(0, 0, 1, 3) = vb1 / vb1.norm(); Mn.block(0, 0, 1, 3) = vn1 / vn1.norm();
    Mb.block(0, 0, 1, 3) = vb / vb.norm();  Mn.block(0, 0, 1, 3) = vn / vn.norm();
    Mb.block(0, 0, 1, 3) = vbb / vbb.norm(); Mn.block(0, 0, 1, 3) = vnn / vnn.norm();
    return Mb.transpose() * Mn;
}

hwa_base::SO3 hwa_base::askew(const Triple& v)
{
    SO3 vnx;
    vnx << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return vnx;
}

Eigen::Matrix4d hwa_base::m2m4(const Triple& v)
{
    Eigen::Matrix4d M;
    M << 0.0, -v(0), -v(1), -v(2),
        v(0), 0.0, -v(2), v(1),
        v(1), v(2), 0.0, -v(0),
        v(2), -v(1), v(0), 0.0;
    return M;
}

Eigen::Matrix4d hwa_base::m2m4_(const Triple& v)
{
    Eigen::Matrix4d M;
    M << 0.0, -v(0), -v(1), -v(2),
        v(0), 0.0, v(2), -v(1),
        v(1), -v(2), 0.0, v(0),
        v(2), v(1), -v(0), 0.0;
    return M;
}

hwa_base::SO3 hwa_base::Cen(const Triple& pos)
{
    double sb = sin(pos(0)), cb = cos(pos(0)), sl = sin(pos(1)), cl = cos(pos(1));
    SO3 cen;
    cen << -sl, -sb * cl, cb* cl, cl, -sb * sl, cb* sl, 0, cb, sb;
    //std::cerr << cen;
    return cen;
}

void hwa_base::symmetry(Matrix& m)
{
    m = (m + m.transpose()) / 2.0;
}

hwa_base::SO3 hwa_base::dGeod2Cart(const hwa_base::base_earth& eth, const Triple& blh)
{
    SO3 res;
    double B, L, H, N, A;
    B = blh(0); L = blh(1); H = blh(2);
    double a = eth.a, e2 = eth.e2;
    A = a * e2 * sin(B) * cos(B) * pow((1 - e2 * sin(B) * sin(B)), -1.5);
    N = a / sqrt(1 - e2 * sin(B) * sin(B));

    double A11, A12, A13, A21, A22, A23, A31, A32, A33;
    A11 = A * cos(B) * cos(L) - (N + H) * sin(B) * cos(L); A12 = -(N + H) * cos(B) * sin(L); A13 = cos(B) * cos(L);
    A21 = A * cos(B) * sin(L) - (N + H) * sin(B) * sin(L); A22 = (N + H) * cos(B) * cos(L); A23 = cos(B) * sin(L);
    A31 = A * (1 - e2) * sin(B) + (N * (1 - e2) + H) * cos(B); A32 = 0; A33 = sin(B);

    res << A11, A12, A13,
        A21, A22, A23,
        A31, A32, A33;
    return res;
}

hwa_base::Triple hwa_base::product(const Triple& vec, const SO3& mat)
{
    Triple res;
    for (int i = 0; i < 3; i++)
    {
        res(i) = vec(0) * mat(0, i) + vec(1) * mat(1, i) + vec(2) * mat(2, i);
    }
    return res;
}

void hwa_base::delrowcol(Matrix& M, int i)
{
    Matrix TMP = M;
    int m = TMP.rows(), n = TMP.cols();
    M = Matrix::Zero(m - 1, n - 1);
    M.block(0, 0, i, i) = TMP.block(0, 0, i, i);
    M.block(0, i, i, n - i - 1) = TMP.block(0, i + 1, i, n - i - 1);
    M.block(i, 0, m - i - 1, i) = TMP.block(i + 1, 0, m - i - 1, i);
    M.block(i, i, m - i - 1, n - i - 1) = TMP.block(i + 1, i + 1, m - i - 1, n - i - 1);
}

void hwa_base::delrow(Matrix& M, int i)
{
    Matrix TMP = M;
    int m = TMP.rows(), n = TMP.cols();
    M = Matrix::Zero(m - 1, n);
    M.block(0, 0, i, n) = TMP.block(0, 0, i, n);
    M.block(i, 0, m - i - 1, n) = TMP.block(i + 1, 0, m - i - 1, n);
}

void hwa_base::delcol(Matrix& M, int i)
{
    Matrix TMP = M;
    int m = TMP.rows(), n = TMP.cols();
    M = Matrix::Zero(m, n - 1);
    M.block(0, 0, m, i) = TMP.block(0, 0, m, i);
    M.block(0, i, m, n - i - 1) = TMP.block(0, i + 1, m, n - i - 1);
}

void hwa_base::delrow(Vector& V, int i)
{
    Vector TMP = V;
    int m = TMP.rows();
    V = Vector::Zero(m - 1);
    V.block(0, 0, i, 1) = TMP.block(0, 0, i, 1);
    V.block(i, 0, m - i - 1, 1) = TMP.block(i + 1, 0, m - i - 1, 1);
}

void hwa_base::move(int& a)
{
    a <<= 1;
}

hwa_base::base_quat hwa_base::Qbase2eigen(Eigen::Quaterniond input) {
    base_quat q;
    q.q0 = input.w();
    q.q1 = input.x();
    q.q2 = input.y();
    q.q3 = input.z();
    return base_quat::normalize(q);
}