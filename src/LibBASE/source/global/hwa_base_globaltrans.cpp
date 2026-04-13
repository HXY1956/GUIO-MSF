#include <cmath>
#include <iostream>
#include <iomanip>
#include "hwa_base_globaltrans.h"

int hwa_base::ell2xyz(const double* Ell, double* XYZ, bool degrees)
{
    const double bell = Aell * (1.0 - 1.0 / Finv);
    const double e2 = (Aell * Aell - bell * bell) / (Aell * Aell);

    double nn;
    double hsl = Ell[2]; // [m] above mean sea level

    if (degrees)
    {
        nn = Aell / sqrt(1.0 - e2 * sin(Ell[0] * D2R) * sin(Ell[0] * D2R));
        XYZ[0] = (nn + hsl) * cos(Ell[0] * D2R) * cos(Ell[1] * D2R);
        XYZ[1] = (nn + hsl) * cos(Ell[0] * D2R) * sin(Ell[1] * D2R);
        XYZ[2] = ((1 - e2) * nn + hsl) * sin(Ell[0] * D2R);
    }
    else
    {
        nn = Aell / sqrt(1.0 - e2 * sin(Ell[0]) * sin(Ell[0]));
        XYZ[0] = (nn + hsl) * cos(Ell[0]) * cos(Ell[1]);
        XYZ[1] = (nn + hsl) * cos(Ell[0]) * sin(Ell[1]);
        XYZ[2] = ((1 - e2) * nn + hsl) * sin(Ell[0]);
    }

    return 1;
}


int hwa_base::xyz2ell(const double* XYZ, double* Ell, bool degrees)
{
    const double bell = Aell * (1.0 - 1.0 / Finv);
    const double e2 = (Aell * Aell - bell * bell) / (Aell * Aell);
    const double e2c = (Aell * Aell - bell * bell) / (bell * bell);

    double nn, ss, zps, hOld, phiOld, theta, sin3, cos3;

    ss = sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]);

    if (double_eq(ss, 0.0))
    {
        Ell[0] = -999;
        Ell[1] = -999;
        Ell[2] = -999;
        return 1;
    }

    zps = XYZ[2] / ss;
    theta = atan((XYZ[2] * Aell) / (ss * bell));
    sin3 = sin(theta) * sin(theta) * sin(theta);
    cos3 = cos(theta) * cos(theta) * cos(theta);

    // Closed formula
    Ell[0] = atan((XYZ[2] + e2c * bell * sin3) / (ss - e2 * Aell * cos3));
    Ell[1] = atan2(XYZ[1], XYZ[0]);
    nn = Aell / sqrt(1.0 - e2 * sin(Ell[0]) * sin(Ell[0]));
    Ell[2] = ss / cos(Ell[0]) - nn;

    const int MAXITER = 100;
    for (int ii = 1; ii <= MAXITER; ii++)
    {
        nn = Aell / sqrt(1.0 - e2 * sin(Ell[0]) * sin(Ell[0]));
        hOld = Ell[2];
        phiOld = Ell[0];
        Ell[2] = ss / cos(Ell[0]) - nn;
        Ell[0] = atan(zps / (1.0 - e2 * nn / (nn + Ell[2])));

        if (fabs(phiOld - Ell[0]) <= 1.0e-11 && fabs(hOld - Ell[2]) <= 1.0e-5)
        {

            // always convert longitude to 0-360
            if (Ell[1] < 0.0)
                Ell[1] += 2 * hwa_pi;

            if (degrees)
            {
                Ell[0] *= R2D;
                Ell[1] *= R2D;
            }

            return 0;
        }
    }

    return 1;
}

// XYZ to Topocentric/Local (North, East, Up) coordinates
// ----------
int hwa_base::xyz2neu(const double* XYZ, const double* XYZ_Ref, double* neu)
{
    double ele[3];
    xyz2ell(XYZ_Ref, ele, false);
    Triple r;
    r[0] = XYZ[0] - XYZ_Ref[0];
    r[1] = XYZ[1] - XYZ_Ref[1];
    r[2] = XYZ[2] - XYZ_Ref[2];
    double sinPhi = sin(ele[0]);
    double cosPhi = cos(ele[0]);
    double sinLam = sin(ele[1]);
    double cosLam = cos(ele[1]);
    neu[0] = -sinPhi * cosLam * r[0] - sinPhi * sinLam * r[1] + cosPhi * r[2];
    neu[1] = -sinLam * r[0] + cosLam * r[1];
    neu[2] = +cosPhi * cosLam * r[0] + cosPhi * sinLam * r[1] + sinPhi * r[2];
    return 1;
}

// Topocentric/Local (NEU) to XYZ coordinates
// ----------
int hwa_base::neu2xyz(const double* Ell, const double* neu, double* xyz)
{
    double sinPhi = sin(Ell[0]);
    double cosPhi = cos(Ell[0]);
    double sinLam = sin(Ell[1]);
    double cosLam = cos(Ell[1]);

    xyz[0] = -sinPhi * cosLam * neu[0] - sinLam * neu[1] + cosPhi * cosLam * neu[2];

    xyz[1] = -sinPhi * sinLam * neu[0] + cosLam * neu[1] + cosPhi * sinLam * neu[2];

    xyz[2] = +cosPhi * neu[0] + sinPhi * neu[2];

    return 1;
}

// ELL coordinates to XYZ coordinates
int hwa_base::ell2xyz(const Triple& ell, Triple& xyz, bool degrees)
{
    double ELL[3] = { ell[0], ell[1], ell[2] };
    double XYZ[3] = { 0.0, 0.0, 0.0 };
    int irc = ell2xyz(ELL, XYZ, degrees);

    if (irc > 0)
    {
        xyz[0] = XYZ[0];
        xyz[1] = XYZ[1];
        xyz[2] = XYZ[2];
        return 1;
    }
    else
        return -1;
}

int hwa_base::xyz2ell(const Triple& crd, Triple& ell, bool degrees)
{
    const double bell = Aell * (1.0 - 1.0 / Finv);
    const double e2 = (Aell * Aell - bell * bell) / (Aell * Aell);
    const double e2c = (Aell * Aell - bell * bell) / (bell * bell);

    double nn, ss, zps, hOld, phiOld, theta, sin3, cos3;

    ss = sqrt(crd[0] * crd[0] + crd[1] * crd[1]);

    if (double_eq(ss, 0.0))
    {
        ell[0] = -999;
        ell[1] = -999;
        ell[2] = -999;
        return 1;
    }

    zps = crd[2] / ss;
    theta = atan((crd[2] * Aell) / (ss * bell));
    sin3 = sin(theta) * sin(theta) * sin(theta);
    cos3 = cos(theta) * cos(theta) * cos(theta);

    ell[0] = atan((crd[2] + e2c * bell * sin3) / (ss - e2 * Aell * cos3));
    ell[1] = atan2(crd[1], crd[0]);
    nn = Aell / sqrt(1.0 - e2 * sin(ell[0]) * sin(ell[0]));
    ell[2] = ss / cos(ell[0]) - nn;

    const int MAXITER = 100;
    for (int ii = 1; ii <= MAXITER; ii++)
    {
        nn = Aell / sqrt(1.0 - e2 * sin(ell[0]) * sin(ell[0]));
        hOld = ell[2];
        phiOld = ell[0];
        ell[2] = ss / cos(ell[0]) - nn;
        ell[0] = atan(zps / (1.0 - e2 * nn / (nn + ell[2])));

        if (fabs(phiOld - ell[0]) <= 1.0e-11 && fabs(hOld - ell[2]) <= 1.0e-5)
        {
            // always convert longitude to 0-360
            if (ell[1] < 0.0)
                ell[1] += 2 * hwa_pi;

            if (degrees)
            {
                ell[0] *= R2D;
                ell[1] *= R2D;
            }

            return 0;
        }
    }

    return 1;
}

int hwa_base::rao2xyz_rot(const Triple& pos, const Triple& vel, SO3& R)
{
    Triple along = vel / vel.norm();
    Triple cross = pos.cross(vel);
    cross /= cross.norm();
    Triple radial = along.cross(cross);
    R.col(0) = radial.matrix();
    R.col(1) = along.matrix();
    R.col(2) = cross.matrix();
    return 1;
}

// Radial/Along-track/Out-of-plane (Satell-system) to XYZ coordinates
// ---------------
int hwa_base::rao2xyz(const Triple& pos, const Triple& vel,
    const Triple& rao, Triple& xyz)
{
    Triple along = vel / vel.norm();
    Triple cross = pos.cross(vel);
    cross /= cross.norm();
    Triple radial = along.cross(cross);
    SO3 RR;
    RR.col(0) = radial.matrix();
    RR.col(1) = along.matrix();
    RR.col(2) = cross.matrix();

    xyz = RR * rao;
    SO3 R;
    rao2xyz_rot(pos, vel, R);
    return 1;
}

//  XYZ coordinates to Radial/Along-track/Out-of-plane (Satell-system)
// ---------------
int hwa_base::xyz2rao(const Triple& pos, const Triple& vel,
    Triple& xyz, Triple& rao)
{
    SO3 R;
    rao2xyz_rot(pos, vel, R);
    rao = R.transpose() * xyz;
    return 1;
}

int hwa_base::xyz2ell2(const Triple& crd, Triple& ell, bool degrees)
{
    const double A = Aell;
    const double B = A * (1.0 - 1.0 / Finv);
    const double E2 = (A * A - B * B) / (A * A);
    double P = sqrt(crd[0] * crd[0] + crd[1] * crd[1]);
    if (double_eq(P, 0.0))
    {
        ell[0] = -999;
        ell[1] = -999;
        ell[2] = -999;
        return 1;
    }

    ell[0] = atan2(crd[2], P * (1 - E2));
    ell[1] = atan2(crd[1], crd[0]);

    double V = A / sqrt(1.0 - E2 * sin(ell[0]) * sin(ell[0]));
    ell[2] = P / cos(ell[0]) - V;

#ifdef DEBUG
    cout << "XYZ2ELL: " << 0 << " " << fixed << setprecision(6) << crd[0] << " " << crd[1] << " " << crd[2]
        << "  " << setprecision(6) << ell[0] << " " << ell[1] << " " << ell[2]
        << "  v: " << V << "  p: " << P
        << endl;
#endif
    const int MAXITER = 100;
    for (int ii = 1; ii <= MAXITER; ii++)
    {
        double helOld = ell[2];
        double latOld = ell[0];

        V = A / sqrt(1.0 - E2 * sin(ell[0]) * sin(ell[0]));
        ell[2] = P / cos(ell[0]) - V;
        ell[0] = atan2(crd[2] / P, 1.0 - E2 * V / (V + ell[2]));
#ifdef DEBUG
        cout << "XYZ2ELL " << ii << " " << fixed << setprecision(6) << crd[0] << " " << crd[1] << " " << crd[2]
            << "  " << setprecision(6) << ell[0] << " " << ell[1] << " " << ell[2]
            << "  v: " << V << "  p: " << P << " helOld: " << helOld << "  latOld: " << latOld * R2D << endl;
#endif
        if (fabs(latOld - ell[0]) <= 1.0e-11 && fabs(helOld - ell[2]) <= 1.0e-5)
        {
            // always convert longitude to 0-360
            if (ell[1] < 0.0)
                ell[1] += 2 * hwa_pi;

            if (degrees)
            {
                ell[0] *= R2D;
                ell[1] *= R2D;
            }

            return 0;
        }
    }

    return 1;
}

int hwa_base::xyz2ell_vlbi(const Triple& crd, Triple& ell)
{
    double a = 6378136.6;     // m      Equatorial radius of the Earth
    double f = 1 / 298.25642; // Flattening factor of the Earth
    double e2 = 2.0 * f - f * f;
    double lon = atan2(crd[1], crd[0]);
    double lat = atan2(crd[2], sqrt(crd[0] * crd[0] + crd[1] * crd[1]));
    double h;
    for (int j = 1; j <= 6; j++)
    {
        double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
        h = sqrt(crd[0] * crd[0] + crd[1] * crd[1]) / cos(lat) - N;
        lat = atan2(crd[2] * (N + h), sqrt(crd[0] * crd[0] + crd[1] * crd[1]) * ((1 - e2) * N + h));
    }
    ell[0] = lat;
    ell[1] = lon;
    ell[2] = h;
    return 0;
}

//void xyz2neu(Triple &crd, Triple &crd_site, Triple &neu)
void hwa_base::xyz2neu(Triple& ell, Triple& xyz, Triple& neu)
{
    double sinPhi = sin(ell[0]);
    double cosPhi = cos(ell[0]);
    double sinLam = sin(ell[1]);
    double cosLam = cos(ell[1]);

    neu[0] = -sinPhi * cosLam * xyz[0] - sinPhi * sinLam * xyz[1] + cosPhi * xyz[2];

    neu[1] = -sinLam * xyz[0] + cosLam * xyz[1];

    neu[2] = + cosPhi * cosLam * xyz[0] + cosPhi * sinLam * xyz[1] + sinPhi * xyz[2];
}

int hwa_base::xyz2neu(Triple& xyz, Symmetric& Q_xyz, Symmetric& Q_neu)
{

    Triple ele(0, 0, 0);
    xyz2ell(xyz, ele, false);

    double sinPhi = sin(ele[0]);
    double cosPhi = cos(ele[0]);
    double sinLam = sin(ele[1]);
    double cosLam = cos(ele[1]);

    int n_par = Q_xyz.cols();
    Matrix R(n_par, n_par);
    for (int i = 0; i < n_par; i++)
    {
        R(i, i) = 1;
    }

    R(0, 0) = -sinPhi * cosLam;
    R(0, 1) = -sinPhi * sinLam;
    R(0, 2) = cosPhi;
    R(1, 0) = -sinLam;
    R(1, 1) = cosLam;
    R(1, 2) = 0;
    R(2, 0) = cosPhi * cosLam;
    R(2, 1) = cosPhi * sinLam;
    R(2, 2) = sinPhi;

    Q_neu.matrixW() = R * Q_xyz.matrixR() * R.transpose();

    return 1;
}

void hwa_base::neu2xyz(Triple& ell, Triple& neu, Triple& xyz)
{
    double sinPhi = sin(ell[0]);
    double cosPhi = cos(ell[0]);
    double sinLam = sin(ell[1]);
    double cosLam = cos(ell[1]);

    xyz[0]= -sinPhi * cosLam * neu[0] - sinLam * neu[1] + cosPhi * cosLam * neu[2];
    xyz[1] = -sinPhi * sinLam * neu[0] + cosLam * neu[1] + cosPhi * sinLam * neu[2];
    xyz[2] = +cosPhi * neu[0] + sinPhi * neu[2];
}

hwa_base::SO3 hwa_base::rotm(double angle, int type)
{
    SO3 R;
    double ca = cos(angle);
    double sa = sin(angle);
    switch (type)
    {
    case 1:
        R(0, 0) = 1;
        R(1, 1) = ca;
        R(1, 2) = sa;
        R(2, 1) = -sa;
        R(2, 2) = ca;
        break;
    case 2:
        R(0, 0) = ca;
        R(0, 2) = -sa;
        R(1, 1) = 1;
        R(2, 0) = sa;
        R(2, 2) = ca;
        break;
    case 3:
        R(0, 0) = ca;
        R(0, 1) = sa;
        R(1, 0) = -sa;
        R(1, 1) = ca;
        R(2, 2) = 1;
        break;
    }
    return R;
}

hwa_base::SO3 hwa_base::drotm(double angle, int type)
{
    hwa_base::SO3 R;
    double ca = cos(angle);
    double sa = sin(angle);
    switch (type)
    {
    case 1:
        R(0, 0) = 0.0;
        R(1, 1) = -sa;
        R(1, 2) = ca;
        R(2, 1) = -ca;
        R(2, 2) = -sa;
        break;
    case 2:
        R(0, 0) = -sa;
        R(0, 2) = -ca;
        R(1, 1) = 0.0;
        R(2, 0) = ca;
        R(2, 2) = -sa;
        break;
    case 3:
        R(0, 0) = -sa;
        R(0, 1) = ca;
        R(1, 0) = -ca;
        R(1, 1) = -sa;
        R(2, 2) = 0.0;
        break;
    }
    return R;
}

hwa_base::Triple hwa_base::Cart2Geod(const Triple& CartPos, bool b)
{
    double XYZ[3] = { CartPos(0), CartPos(1), CartPos(2) }, ell[3];
    xyz2ell(XYZ, ell, b);
    return Triple(ell);
}

hwa_base::Triple hwa_base::Geod2Cart(const Triple& GeodPos, bool b)
{
    double ell[3] = { GeodPos(0), GeodPos(1), GeodPos(2) }, XYZ[3];
    ell2xyz(ell, XYZ, b);
    return Triple(XYZ);
}

hwa_base::Triple hwa_base::XYZ2ENU(const Triple& xyz, const Triple& xyz_ref)
{
    double XYZ[3] = { xyz(0), xyz(1), xyz(2) }, XYZ_REF[3] = { xyz_ref(0), xyz_ref(1), xyz_ref(2) }, neu[3];
    xyz2neu(XYZ, XYZ_REF, neu);
    return Triple(neu[1], neu[0], neu[2]);
}