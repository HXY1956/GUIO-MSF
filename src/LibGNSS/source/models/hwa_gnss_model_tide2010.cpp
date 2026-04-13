#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include "hwa_base_globaltrans.h"
#include "hwa_gnss_model_tide2010.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_interp.h"

using namespace hwa_base;
using namespace std;

namespace hwa_gnss
{
    gnss_model_tide2010::gnss_model_tide2010(base_log spdlog) : gnss_model_tide(spdlog)
    {
        _D.resize(6);
        _DD.resize(6);
    }

    // destructor
    // ----------
    gnss_model_tide2010::~gnss_model_tide2010()
    {
    }

    Triple cart2sph(const Triple &xyz)
    {
        Triple res;
        double hypotxy = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
        double r = sqrt(hypotxy * hypotxy + xyz[2] * xyz[2]);
        double elev = atan2(xyz[2], hypotxy);
        double az = atan2(xyz[1], xyz[0]);
        res[0] = az;
        res[1] = elev;
        res[2] = r;
        return res;
    }

    double cart2phigd(const Triple &cart)
    {
        double phigd;
        double Re = 6378136.55;
        double f = 1 / 300.0;
        double e12 = 2 * f - f * f;
        double e22 = e12 / (1 - e12);

        double rsp = sqrt(cart[0] * cart[0] + cart[1] * cart[1]);
        double teta = atan2(cart[2], (rsp * (1 - f)));

        phigd = atan2((cart[2] + e22 * Re * sin(teta) * sin(teta) * sin(teta) * (1 - f)), (rsp - e12 * Re * cos(teta) * cos(teta) * cos(teta)));
        return phigd;
    }

    Matrix Doodarg(double mjd, double delta)
    {
        Matrix da(6, 1);
        double t1 = (mjd - 51544.5 + delta / 86400.0) / 36525.0;
        double fhr = (mjd - (int)mjd) * 24;
        double s, tau, pr, h, p, zns, ps;
        s = 218.31664563 + 481267.88194 * t1 - 0.0014663889 * t1 * t1 + 0.00000185139 * t1 * t1 * t1;
        tau = fhr * 15 + 280.4606184 + 36000.7700536 * t1 + 0.00038793 * t1 * t1 - 0.0000000258 * t1 * t1 * t1 - s;
        pr = 1.396971278 * t1 + 0.000308889 * t1 * t1 + 0.000000021 * t1 * t1 * t1 + 0.000000007 * t1 * t1 * t1 * t1;
        s = s + pr;
        h = 280.46645 + 36000.7697489 * t1 + 0.00030322222 * t1 * t1 + 0.000000020 * t1 * t1 * t1 - 0.00000000654 * t1 * t1 * t1 * t1;
        p = 83.35324312 + 4069.01363525 * t1 - 0.01032172222 * t1 * t1 - 0.0000124991 * t1 * t1 * t1 + 0.00000005263 * t1 * t1 * t1 * t1;
        zns = 234.95544499 + 1934.13626197 * t1 - 0.00207561111 * t1 * t1 - 0.00000213944 * t1 * t1 * t1 + 0.00000001650 * t1 * t1 * t1 * t1;
        ps = 282.93734098 + 1.71945766667 * t1 + 0.00045688889 * t1 * t1 - 0.00000001778 * t1 * t1 * t1 - 0.00000000334 * t1 * t1 * t1 * t1;
        da(0, 0) = tau;
        da(1, 0) = s;
        da(2, 0) = h;
        da(3, 0) = p;
        da(4, 0) = zns;
        da(5, 0) = ps;
        return da;
    }

    // piecewise cubic Hermite interpolation
    Matrix pwch(vector<double> x, vector<double> y, Matrix s, vector<double> dx, vector<double> dy)
    {
        vector<double> dzzdx, dzdxdx;
        for (unsigned int i = 0; i < dy.size(); i++)
        {
            dzzdx.push_back((dy[i] - s(1, i + 1)) / dx[i]);
            dzdxdx.push_back((s(1, i + 2) - dy[i]) / dx[i]);
        }
        Matrix pc(dx.size(), 4);
        for (unsigned int i = 0; i < dx.size(); i++)
        {
            pc(i, 0) = (dzdxdx[i] - dzzdx[i]) / dx[i];
            pc(i, 1) = 2 * dzzdx[i] - dzdxdx[i];
            pc(i, 2) = s(0, i);
            pc(i, 3) = y[i];
        }
        return pc;
    }
    double ppval(Matrix pp, vector<double> x, double xx)
    {
        double xs = int(xx * 100 / 25) * 0.25;
        vector<double>::iterator it = find(x.begin(), x.end(), xs);
        int index = it - x.begin();
        xs = xx - x[index];
        double v = pp(index, 0);
        for (int i = 1; i < pp.cols(); i++)
            v = xs * v + pp(index, i);
        return v;
    }
    //double gnss_model_tide2010::splineinterp(vector<double> x, vector<double> y, double xq)
    //{
    //    int n = x.size();
    //    vector<double> dx, dy, endslopes;
    //    Matrix pp;
    //    for (int i = 0; i < n - 1; i++)
    //    {
    //        dx.push_back(x[i + 1] - x[i]);
    //        dy.push_back((y[i + 1] - y[i]) / dx[i]);
    //    }
    //    if (y.size() == x.size())
    //    {
    //
    //    }
    //    else if (y.size() == x.size() + 2)
    //    {
    //        // need to supplement referring to chckxy
    //    }
    //    else
    //    {
    //        // need an error tag
    //    }
    //    if (n == 2)
    //    {
    //        if (endslopes.empty())
    //        {
    //            // the interpolant is a strainght line reference spline.m -- a Matlab function
    //        }
    //        else
    //        {
    //            //the interpolant is the cubic Hermite polynomial reference spline.m
    //        }
    //    }
    //    else if (n == 3 && endslopes.empty())
    //    {
    //        // thee interpolant is a parabola reference spline.m
    //    }
    //    else
    //    {
    //        vector<double> b;
    //        double x31, xn;
    //        b.push_back(0.0);
    //        for (int i = 1; i < n - 1; i++)
    //        {
    //            b.push_back(3 * (dx[i] * dy[i - 1] + dx[i - 1] * dy[i]));
    //        }
    //        b.push_back(0.0);
    //        if (endslopes.empty())
    //        {
    //            x31 = x[2] - x[0];
    //            xn = x[n - 1] - x[n - 3];
    //            b[0] = ((dx[0] + 2 * x31) * dx[1] * dy[0] + dx[0] * dx[0] * dy[1]) / x31;
    //            b[n - 1] = (dx[n - 2] * dx[n - 2] * dy[n - 3] + (2 * xn + dx[n - 2]) * dx[n - 3] * dy[n - 2]) / xn;
    //        }
    //        else
    //        {
    //            x31 = 0;
    //            xn = 0;
    //            b.insert(b.begin(), dx[1] * endslopes[0]);
    //            b.push_back(dx[n - 3] * endslopes[1]);
    //        }
    //        Matrix partmp(n, 3);
    //        partmp(1, 1) = x31;
    //        partmp(1, 2) = dx[1];
    //        partmp(1, 3) = 0.0;
    //        for (int j = 0; j < n - 2; j++)
    //        {
    //            partmp(j + 2, 1) = dx[j];
    //            partmp(j + 2, 2) = 2 * (dx[j + 1] + dx[j]);
    //            partmp(j + 2, 3) = dx[j + 1];
    //        }
    //        partmp(n, 1) = 0.0;
    //        partmp(n, 2) = dx[n - 3];
    //        partmp(n, 3) = xn;
    //        vector<int> vectmp;
    //        vectmp.push_back(-1); vectmp.push_back(0); vectmp.push_back(1);
    //        /*std::cout << partmp(1, 1) << partmp(1, 2) << partmp(1, 3)
    //            << partmp(2, 1) << partmp(2, 2) << partmp(2, 3)
    //            << partmp(3, 1) << partmp(3, 2) << partmp(3, 3)
    //            << partmp(4, 1) << partmp(4, 2) << partmp(4, 3)
    //            << partmp(5, 1) << partmp(5, 2) << partmp(5, 3)
    //            << partmp(6, 1) << partmp(6, 2) << partmp(6, 3)
    //            << partmp(7, 1) << partmp(7, 2) << partmp(7, 3)
    //            << partmp(8, 1) << partmp(8, 2) << partmp(8, 3)<< std::endl;*/
    //        Matrix c = spdiags(partmp, vectmp, n, n);
    //        Matrix bm(1, n);
    //        for (int i = 0; i < b.size(); i++)
    //            bm(1, i + 1) = b[i];
    //       /* std::cout << c(0, 0) << " " << c(0, 1) << " " << c(0, 2) << " " << c(0, 3) << " " << c(0, 4) << " " << c(0, 5) << " " << c(0, 6) << " " << c(0, 7)
    //            << c(1, 0) << " " << c(1, 1) << " " << c(1, 2) << " " << c(1, 3) << " " << c(1, 4) << " " << c(1, 5) << " " << c(1, 6) << " " << c(1, 7)
    //            << c(2, 0) << " " << c(2, 1) << " " << c(2, 2) << " " << c(2, 3) << " " << c(2, 4) << " " << c(2, 5) << " " << c(2, 6) << " " << c(2, 7)
    //            << c(3, 0) << " " << c(3, 1) << " " << c(3, 2) << " " << c(3, 3) << " " << c(3, 4) << " " << c(3, 5) << " " << c(3, 6) << " " << c(3, 7)
    //            << c(4, 0) << " " << c(4, 1) << " " << c(4, 2) << " " << c(4, 3) << " " << c(4, 4) << " " << c(4, 5) << " " << c(4, 6) << " " << c(4, 7)
    //            << c(5, 0) << " " << c(5, 1) << " " << c(5, 2) << " " << c(5, 3) << " " << c(5, 4) << " " << c(5, 5) << " " << c(5, 6) << " " << c(5, 7)
    //            << c(6, 0) << " " << c(6, 1) << " " << c(6, 2) << " " << c(6, 3) << " " << c(6, 4) << " " << c(6, 5) << " " << c(6, 6) << " " << c(6, 7)
    //            << c(7, 0) << " " << c(7, 1) << " " << c(7, 2) << " " << c(7, 3) << " " << c(7, 4) << " " << c(7, 5) << " " << c(7, 6) << " " << c(7, 7) << std::endl;*/
    //
    //        Matrix tmpmat = c.i();
    //        /*tmpmat = c.inverse();
    //        std::cout << tmpmat(1, 1) << tmpmat(1, 2) << tmpmat(2, 1) << tmpmat(2, 2) << tmpmat(2, 3) << tmpmat(3, 2) << tmpmat(3, 3) << tmpmat(3, 4) << std::endl;*/
    //        Matrix s = bm * c.i();  //c.toDense();
    //        //std::cout << s(1, 1) << s(1, 2) << s(1, 3) << s(1, 4) << s(1, 5) << s(1, 6) << s(1, 7) << s(1, 8) << std::endl;
    //        pp = pwch(x, y, s, dx, dy);
    //    }
    //    return ppval(pp, x, xq);
    //}
    // WARNING!!! Still IERS 2003. 2010 needs to be implemented
    // solid base_earth tides
    // ----------
    Triple gnss_model_tide2010::tide_earth(const base_time &epoch, Triple &crd, Triple xsun, Triple xmoon)
    {
        _mutex.lock();

        Triple dxyz(0.0, 0.0, 0.0);
        Triple dren12(0.0, 0.0, 0.0);              // for VLBI stations, IERS 2010 ?!
        Triple dren13(0.0, 0.0, 0.0);              // for VLBI stations
        Triple dren14(0.0, 0.0, 0.0);              // for VLBI stations
        Triple dren15(0.0, 0.0, 0.0);              // for VLBI stations
        Triple dr16(0.0, 0.0, 0.0);                // for VLBI stations
        Triple dt16(0.0, 0.0, 0.0);                // for VLBI stations
        Triple dren16(0.0, 0.0, 0.0);              // for VLBI stations
        Triple dr17(0.0, 0.0, 0.0);                // for VLBI stations
        Triple dt17(0.0, 0.0, 0.0);                // for VLBI stations
        Triple dren17(0.0, 0.0, 0.0);              // for VLBI stations
        Triple dx12, dx13, dx14, dx15, dx16, dx17; // for VLBI stations
        Triple r(1.0, 0.0, 0.0);                   // for VLBI stations
        Triple e(0.0, 1.0, 0.0);                   // for VLBI stations
        Triple n(0.0, 0.0, 1.0);                   // for VLBI stations

        Triple rs;
        Triple rm;
        Triple tmpell; // for VLBI stations

        Triple gxsun(xsun);
        tmpell = cart2sph(gxsun); // for VLBI stations
        double Ls = tmpell[0];
        double Bs = tmpell[1];
        double Rs = tmpell[2];
        rs = xsun / Rs;

        Triple gxmoon(xmoon);
        tmpell = cart2sph(gxmoon); // for VLBI stations
        double Lm = tmpell[0];
        double Bm = tmpell[1];
        double Rm = tmpell[2];
        rm = xmoon / Rm;

        Triple xyz = crd;
        Triple gxyz = xyz;
        tmpell = cart2sph(gxyz);
        double lam = tmpell[0];
        double phi = tmpell[1];
        double Ra = tmpell[2];
        Triple ra = xyz / Ra;

        // Love's, Shida's Numbers
        // --------------
        const double h_0 = 0.6078;  // Love number h0 of degree 2
        const double h_2 = -0.0006; // Love number h2 of degree 2
        const double h3 = 0.292;    // Love number of degree 3
        const double l_0 = 0.0847;  // Shida number l0 of degree 2
        const double l_2 = 0.0002;  // Shida number l0 of degree 2
        const double l3 = 0.015;    // Shide number of degree 3

        const double l_11 = 0.0012;   // diurnal band, for VLBI stations
        const double l_12 = 0.0024;   // semidiurnal band, for VLBI stations
        const double hI_21 = -0.0025; // for VLBI stations
        const double lI_21 = -0.0007; // for VLBI stations
        const double hI_22 = -0.0022; // for VLBI stations
        const double lI_22 = -0.0007; //for VLBI stations

        double h = h_0 + h_2 * (3 * sin(phi) * sin(phi) - 1) / 2;
        double l = l_0 + l_2 * (3 * sin(phi) * sin(phi) - 1) / 2;

        const double MS2E = 332946.0;   // Ratio of mass Sun to Earth
        const double MM2E = 0.01230002; // Ratio of mass Moon to Earth

        // Tidal Displacement
        // ------------------
        double scsun = ra.dot(rs);
        double scmon = ra.dot(rm);
        double MRm = 0.012300034000000;
        double MRs = 3.329459430620000e+05;
        double Re = 6.378136550000000e+06;
        double f2m = MRm * pow(Re, 4) / pow(Rm, 3);
        double f2s = MRs * pow(Re, 4) / pow(Rs, 3);
        double f3m = f2m * Re / Rm;
        double f3s = f2s * Re / Rs;
        double P2_sphi = 1.5 * sin(phi) * sin(phi) - 0.5;
        double P2_cm = 1.5 * scmon * scmon - 0.5;
        double P2_cs = 1.5 * scsun * scsun - 0.5;
        double P3_cm = 2.5 * pow(scmon, 3) - 1.5 * scmon;
        double P3_cs = 2.5 * pow(scsun, 3) - 1.5 * scsun;
        double P21_sBm = 3.0 * cos(Bm) * sin(Bm);
        double P21_sBs = 3.0 * cos(Bs) * sin(Bs);
        double P22_sBm = 3.0 * cos(Bm) * cos(Bm);
        double P22_sBs = 3.0 * cos(Bs) * cos(Bs);
        double h2 = h_0 + h_2 * P2_sphi;
        double l2 = l_0 + l_2 * P2_sphi;

        // Computing temporary vectors
        Triple tmpSun, tmpMoon;
        for (int i = 0; i < 3; i++)
        {
            tmpSun[i] = rs[i] - scsun * ra[i];
            tmpMoon[i] = rm[i] - scmon * ra[i];
        }

        Matrix da = Doodarg(epoch.dmjd(), epoch.leapsec() + 32.184); // for VLBI stations
        Matrix diu(11, 10), lband(5, 9), thetafd, thetafl;           // for VLBI stations
        diu
            << 1 , 3 , 5 , 6 , 5 , 5 , -0.08 , 0.00 , -0.01 , 0.01
            , 1 , 4 , 5 , 5 , 4 , 5 , -0.10 , 0.00 , 0.00 , 0.00
            , 1 , 4 , 5 , 5 , 5 , 5 , -0.51 , 0.00 , -0.02 , 0.03
            , 1 , 5 , 5 , 6 , 5 , 5 , 0.06 , 0.00 , 0.00 , 0.00
            , 1 , 6 , 2 , 5 , 5 , 6 , -0.06 , 0.00 , 0.00 , 0.00
            , 1 , 6 , 3 , 5 , 5 , 5 , -1.23 , -0.07 , 0.06 , 0.01
            , 1 , 6 , 5 , 5 , 4 , 5 , -0.22 , 0.01 , 0.01 , 0.00
            , 1 , 6 , 5 , 5 , 5 , 5 , 12.00 , -0.78 , -0.67 , -0.03
            , 1 , 6 , 5 , 5 , 6 , 5 , 1.73 , -0.12 , -0.10 , 0.00
            , 1 , 6 , 6 , 5 , 5 , 4 , -0.50 , -0.01 , 0.03 , 0.00
            , 1 , 6 , 7 , 5 , 5 , 5 , -0.11 , -0.11 , 0.01 , 0.00;
        lband
            << 5 , 5 , 5 , 6 , 5 , 0.47 , 0.16 , 0.23 , 0.07
            , 5 , 7 , 5 , 5 , 5 , -0.20 , -0.11 , -0.12 , -0.05
            , 6 , 5 , 4 , 5 , 5 , -0.11 , -0.09 , -0.08 , -0.04
            , 7 , 5 , 5 , 5 , 5 , -0.13 , -0.15 , -0.11 , -0.07
            , 7 , 5 , 5 , 6 , 5 , -0.05 , -0.06 , -0.05 , -0.03;
        thetafd = (diu.col(0).array() * da(0, 0) + (diu.col(1).array() - 5) * da(1, 0) + (diu.col(2).array() - 5) * da(2, 0) + (diu.col(3).array() - 5) * da(3, 0) + (diu.col(4).array() - 5) * da(4, 0) + (diu.col(5).array() - 5) * da(5, 0)).matrix();
        thetafd = D2R * thetafd;
        thetafl = ((lband.col(0).array() - 5) * da(1, 0) + (lband.col(1).array() - 5) * da(2, 0) + (lband.col(2).array() - 5) * da(3, 0) + (lband.col(3).array() - 5) * da(4, 0) + (lband.col(4).array() - 5) * da(5, 0)).matrix();
        thetafl = D2R * thetafl;

        Triple dx9_m, dx9_s, dx9, dx10_m, dx10_s, dx10, dt12_m, dt12_s, dt13_m, dt13_s, dr14_m, dr14_s, dr14, dt14_m, dt14_s, dt14,
            dr15_m, dr15_s, dr15, dt15_m, dt15_s, dt15;

        for (int i = 0; i < 3; i++)
        {
            dxyz[i] = MS2E * (pow(EARTH_R * 1000, 4) / pow(Rs, 3)) * (h * ra[i] * (3.0 / 2.0 * scsun * scsun - 0.5) + 3.0 * l * scsun * tmpSun[i]) +
                      MM2E * (pow(EARTH_R * 1000, 4) / pow(Rm, 3)) * (h * ra[i] * (3.0 / 2.0 * scmon * scmon - 0.5) + 3.0 * l * scmon * tmpMoon[i]);

            dxyz[i] += MM2E * (pow(EARTH_R * 1000, 5) / pow(Rm, 4)) *
                           (h3 * ra[i] * (5.0 / 2.0 * scmon * scmon * scmon - 3.0 / 2.0 * scmon) +
                            l3 * (15.0 / 2.0 * scmon * scmon - 3.0 / 2.0) * tmpMoon[i]) //; for GPS stations, but for VLBI statons more items need to consider
                       + MS2E * (pow(EARTH_R * 1000, 5) / pow(Rs, 4)) *
                             (h3 * ra[i] * (5.0 / 2.0 * scsun * scsun * scsun - 3.0 / 2.0 * scsun) +
                              l3 * (15.0 / 2.0 * scsun * scsun - 3.0 / 2.0) * tmpSun[i]); // for VLBI stations

            dx9_m[i] = f2m * (h2 * ra[i] * P2_cm + 3.0 * l2 * scmon * (rm[i] - scmon * ra[i]));
            dx9_s[i] = f2s * (h2 * ra[i] * P2_cs + 3.0 * l2 * scsun * (rs[i] - scsun * ra[i]));
            dx9[i] = dx9_m[i] + dx9_s[i];

            dx10_m[i] = f3m * (h3 * ra[i] * P3_cm + l3 * (7.5 * scmon * scmon - 1.5) * (rm[i] - scmon * ra[i]));
            dx10_s[i] = f3s * (h3 * ra[i] * P3_cs + l3 * (7.5 * scsun * scsun - 1.5) * (rs[i] - scsun * ra[i]));
            dx10[i] = dx10_m[i] + dx10_s[i];

            dt12_m[i] = -l_11 * sin(phi) * f2m * P21_sBm * (sin(phi) * cos(lam - Lm) * n[i] - cos(2 * phi) * sin(lam - Lm) * e[i]);
            dt12_s[i] = -l_11 * sin(phi) * f2s * P21_sBs * (sin(phi) * cos(lam - Ls) * n[i] - cos(2 * phi) * sin(lam - Ls) * e[i]);
            dren12[i] = 0.0 * r[i] + dt12_m[i] + dt12_s[i];

            dt13_m[i] = -0.5 * l_12 * sin(phi) * cos(phi) * f2m * P22_sBm * (cos(2 * (lam - Lm)) * n[i] + sin(phi) * sin(2 * (lam - Lm)) * e[i]);
            dt13_s[i] = -0.5 * l_12 * sin(phi) * cos(phi) * f2s * P22_sBs * (cos(2 * (lam - Ls)) * n[i] + sin(phi) * sin(2 * (lam - Ls)) * e[i]);
            dren13[i] = 0.0 * r[i] + dt13_m[i] + dt13_s[i];

            dr14_m[i] = -3.0 / 4.0 * hI_21 * f2m * sin(2 * Bm) * sin(2 * phi) * sin(lam - Lm);
            dr14_s[i] = -3.0 / 4.0 * hI_21 * f2s * sin(2 * Bs) * sin(2 * phi) * sin(lam - Ls);
            dr14[i] = dr14_m[i] + dr14_s[i];

            dt14_m[i] = -1.5 * lI_21 * f2m * sin(2 * Bm) * (cos(2 * phi) * sin(lam - Lm) * n[i] + sin(phi) * cos(lam - Lm) * e[i]);
            dt14_s[i] = -1.5 * lI_21 * f2s * sin(2 * Bs) * (cos(2 * phi) * sin(lam - Ls) * n[i] + sin(phi) * cos(lam - Ls) * e[i]);
            dt14[i] = dt14_m[i] + dt14_s[i];
            dren14[i] = dr14[i] * r[i] + dt14[i];

            dr15_m[i] = -3.0 / 4.0 * hI_22 * f2m * cos(Bm) * cos(Bm) * cos(phi) * cos(phi) * sin(2 * (lam - Lm));
            dr15_s[i] = -3.0 / 4.0 * hI_22 * f2s * cos(Bs) * cos(Bs) * cos(phi) * cos(phi) * sin(2 * (lam - Ls));
            dr15[i] = dr15_m[i] + dr15_s[i];

            dt15_m[i] = 3.0 / 4.0 * lI_22 * f2m * cos(Bm) * cos(Bm) * (sin(2 * phi) * sin(2 * (lam - Lm)) * n[i] - 2 * cos(phi) * cos(2 * (lam - Lm)) * e[i]);
            dt15_s[i] = 3.0 / 4.0 * lI_22 * f2s * cos(Bs) * cos(Bs) * (sin(2 * phi) * sin(2 * (lam - Ls)) * n[i] - 2 * cos(phi) * cos(2 * (lam - Ls)) * e[i]);
            dt15[i] = dt15_m[i] + dt15_s[i];
            dren15[i] = dr15[i] * r[i] + dt15[i];

            for (int j = 1; j < 12; j++)
            {
                dr16[i] += 0.001 * (diu(j, 7) * sin(thetafd(j, 1) + lam) + diu(j, 8) * cos(thetafd(j, 1) + lam)) * sin(2 * phi) * r[i];
                dt16[i] += 0.001 * ((diu(j, 9) * cos(thetafd(j, 1) + lam) - diu(j, 10) * sin(thetafd(j, 1) + lam)) * sin(phi) * e[i] + (diu(j, 9) * sin(thetafd(j, 1) + lam) + diu(j, 10) * cos(thetafd(j, 1) + lam)) * cos(2 * phi) * n[i]);
            } // for VLBI stations
            dren16[i] = dr16[i] + dt16[i];
            for (int j = 1; j < 6; j++)
            {
                dr17[i] += 0.001 * ((1.5 * sin(phi) * sin(phi) - 0.5) * (lband(j, 6) * cos(thetafl(j, 1)) + lband(j, 7) * sin(thetafl(j, 1))) * r[i]);
                dt17[i] += 0.001 * ((lband(j, 8) * cos(thetafl(j, 1)) + lband(j, 9) * sin(thetafl(j, 1))) * sin(2 * phi) * n[i]);
            }
            dren17[i] = dr17[i] + dt17[i]; // for VLBI stations
        }
        Triple neu;
        Triple ell;
        ell[0] = cart2phigd(crd);
        ell[1] = lam;
        ell[2] = 0.0;
        neu[0] =  dren12[2];
        neu[1] =  dren12[1];
        neu[2] =  dren12[0];
        neu2xyz(ell, neu, dx12);
        neu[0] =  dren13[2];
        neu[1] =  dren13[1];
        neu[2] =  dren13[0];
        neu2xyz(ell, neu, dx13);
        neu[0] =  dren14[2];
        neu[1] =  dren14[1];
        neu[2] =  dren14[0];
        neu2xyz(ell, neu, dx14);
        neu[0] =  dren15[2];
        neu[1] =  dren15[1];
        neu[2] =  dren15[0];
        neu2xyz(ell, neu, dx15);
        neu[0] =  dren16[2];
        neu[1] =  dren16[1];
        neu[2] =  dren16[0];
        neu2xyz(ell, neu, dx16);
        neu[0] =  dren17[2];
        neu[1] =  dren17[1];
        neu[2] =  dren17[0];
        neu2xyz(ell, neu, dx17);
        dxyz = dx9 + dx10 + dx12 + dx13 + dx14 + dx15 + dx16 + dx17;

        //std::cout << "return tide: " << dxyz[0] << " " << dxyz[1] << " " << dxyz[2] << std::endl;
        _mutex.unlock();
        return dxyz;
    }

    //base_pair meanpole(double t, modeofmeanpole mode)
    //{
    //    double t0 = 2000.0;
    //    double x0, x1, x2, x3, y0, y1, y2, y3;
    //    base_pair xypm;
    //    switch (mode)
    //    {
    //    case modeofmeanpole::linear:
    //        x0 = 55.0;    // mas
    //        x1 = 1.677;   // mas/year
    //        y0 = 320.5;   // mas
    //        y1 = 3.460;   // mas/year
    //        xypm[0] = x0 + x1 * (t - t0);
    //        xypm[1] = y0 + y1 * (t - t0);
    //        break;
    //    case modeofmeanpole::cubic:
    //        if (t < 2010)
    //        {
    //            x0 = 55.974;    // mas
    //            x1 = 1.8243;    // mas/y
    //            x2 = 0.18413;   // mas/y^2
    //            x3 = 0.007024;  // mas/y^3
    //
    //            y0 = 346.346;
    //            y1 = 1.7896;
    //            y2 = -0.10729;
    //            y3 = -0.000908;
    //        }
    //        else
    //        {
    //            x0 = 23.513;
    //            x1 = 7.6141;
    //            x2 = 0.0;
    //            x3 = 0.0;
    //
    //            y0 = 358.891;
    //            y1 = -0.6287;
    //            y2 = 0.0;
    //            y3 = 0.0;
    //        }
    //        xypm[0] = x0 + x1 * (t - t0) + x2 * (t - t0) * (t - t0) + x3 * (t - t0) * (t - t0) * (t - t0);
    //        xypm[1] = y0 + y1 * (t - t0) + y2 * (t - t0) * (t - t0) + y3 * (t - t0) * (t - t0) * (t - t0);
    //        break;
    //    case modeofmeanpole::cmp2015:
    //        Matrix ierstab;
    //        ierstab
    //            << 1970.00 << 0.002117 << 0.219438
    //            << 1971.00 << 0.005376 << 0.225514
    //            << 1972.00 << 0.008752 << 0.231446
    //            << 1973.00 << 0.012200 << 0.237249
    //            << 1974.00 << 0.015675 << 0.242939
    //            << 1975.00 << 0.019134 << 0.248524
    //            << 1976.00 << 0.022536 << 0.254008
    //            << 1977.00 << 0.025841 << 0.259390
    //            << 1978.00 << 0.029016 << 0.264667
    //            << 1979.00 << 0.032029 << 0.269833
    //            << 1980.00 << 0.034850 << 0.274883
    //            << 1981.00 << 0.037453 << 0.279812
    //            << 1982.00 << 0.039815 << 0.284619
    //            << 1983.00 << 0.041917 << 0.289304
    //            << 1984.00 << 0.043746 << 0.293872
    //            << 1985.00 << 0.045297 << 0.298326
    //            << 1986.00 << 0.046571 << 0.302667
    //            << 1987.00 << 0.047582 << 0.306895
    //            << 1988.00 << 0.048352 << 0.311002
    //            << 1989.00 << 0.048912 << 0.314978
    //            << 1990.00 << 0.049304 << 0.318804
    //            << 1991.00 << 0.049576 << 0.322462
    //            << 1992.00 << 0.049783 << 0.325930
    //            << 1993.00 << 0.049983 << 0.329189
    //            << 1994.00 << 0.050238 << 0.332221
    //            << 1995.00 << 0.050608 << 0.335012
    //            << 1996.00 << 0.051151 << 0.337554
    //            << 1997.00 << 0.051918 << 0.339844
    //            << 1998.00 << 0.052955 << 0.341881
    //            << 1999.00 << 0.054299 << 0.343670
    //            << 2000.00 << 0.055981 << 0.345218
    //            << 2001.00 << 0.058022 << 0.346532
    //            << 2002.00 << 0.060436 << 0.347619
    //            << 2003.00 << 0.063229 << 0.348490
    //            << 2004.00 << 0.066403 << 0.349153
    //            << 2005.00 << 0.069953 << 0.349618
    //            << 2006.00 << 0.073868 << 0.349899
    //            << 2007.00 << 0.078133 << 0.350008
    //            << 2008.00 << 0.082731 << 0.349963
    //            << 2009.00 << 0.087643 << 0.349779
    //            << 2010.00 << 0.092851 << 0.349474
    //            << 2011.00 << 0.098339 << 0.349062
    //            << 2012.00 << 0.104098 << 0.348553
    //            << 2013.00 << 0.110120 << 0.347955
    //            << 2014.00 << 0.116401 << 0.347271
    //            << 2015.00 << 0.122942 << 0.346502;
    //        int ti = int(t);
    //        double tinc = t - ti;
    //        int ind;
    //        for (ind = 1; ind <= ierstab.nrows(); ind++)
    //        {
    //            if (ierstab(ind, 1) == ti)
    //                break;
    //        }
    //        if (ind == ierstab.nrows())
    //        {
    //            xypm[0] = (ierstab(ind, 2) + tinc * (ierstab(ind, 2) - ierstab(ind - 1, 2))) * 1000;
    //            xypm[1] = (ierstab(ind, 3) + tinc * (ierstab(ind, 3) - ierstab(ind - 1, 3))) * 1000;
    //        }
    //        else
    //        {
    //            xypm[0] = (ierstab(ind, 2) + tinc * (ierstab(ind + 1, 2) - ierstab(ind, 2))) * 1000;
    //            xypm[1] = (ierstab(ind, 3) + tinc * (ierstab(ind + 1, 3) - ierstab(ind, 3))) * 1000;
    //        }
    //        break;
    //    }
    //    return xypm;
    //}

    // pole tides
    // ----------
    //Triple gnss_model_tide2010::tide_pole(const base_time& epo, const Triple& xRec, double xp, double yp, modeofmeanpole mode)
    //{
    //   _mutex.lock();
    //
    //  Triple dxyz(0.0,0.0,0.0);
    //  Triple ell;
    //  xyz2ell_vlbi(xRec, ell);
    //  double clt = 3.141592653589793 / 2 - ell[0];
    //  double yr, doy, h, m, s;   // the seconds of the epoch is int ? what if it is float ?
    //  yr = epo.year();
    //  doy = epo.doy();
    //  h = epo.hour();
    //  m = epo.mins();
    //  s = epo.secs();
    //  int IFAC = 365;
    //  if ((int(yr) % 4) == 0) IFAC = 366;
    //  double t = yr + (doy + h / 23.93447 + m / 1440 + s / 86400) / (IFAC + 0.2422);
    //  base_pair xypm = meanpole(t, mode);
    //  double m1 = R2AS * xp - xypm[0] / 1000;
    //  double m2 = -(R2AS * yp - xypm[1] / 1000);
    //  double h2 = 0.6207;
    //  double l2 = 0.0836;
    //  double omega = 7.292115e-5;  // rad/s
    //  double re = 6.378e6;         // m
    //  double g = 9.7803278;        // m/s^2
    //  double dR_m = h2 / g * (-omega * omega * re * re / 2);   // m
    //  double dR = dR_m * hwa_pi / 180.0 / 3600.0;                // m/as
    //  double dT_m = 2 * l2 / g * (-omega * omega * re * re / 2); // m
    //  double dT = dT_m * hwa_pi / 180.0 / 3600.0;                 // m/as
    //  double dr = dR * sin(2 * clt) * (m1 * cos(ell[1]) + m2 * sin(ell[1]));  // m
    //  double de = -dT * cos(clt) * (m1 * sin(ell[1]) - m2 * cos(ell[1]));     // m
    //  double dn = -dT * cos(2 * clt) * (m1 * cos(ell[1]) + m2 * sin(ell[1])); // m
    //  Triple dneu(dn, de, dr);
    //  neu2xyz(ell, dneu, dxyz);
    //
    //  _mutex.unlock();
    //  return dxyz;
    //}

    //Triple gnss_model_tide2010::tide_oceanpole(const base_time& epo, const Triple& xRec, double xp, double yp, modeofmeanpole mode)
    //{
    //    _mutex.lock();
    //    Triple dxyz(0.0, 0.0, 0.0);
    //    Triple ell, ell1;
    //    Triple Rren, Iren;
    //    xyz2ell_vlbi(xRec, ell);
    //    if (!_gopl || _gopl->data(xRec, Rren, Iren) < 0) {
    //        if (_log) _log->comment(0, "gtide2010", "WARNING: Site not found in atmosphere non tidal loading file!");
    //        _mutex.unlock(); return dxyz;
    //    }
    //
    //    int yr = epo.year();
    //    double doy = epo.doy();
    //    double h = epo.hour();
    //    double m = epo.mins();
    //    double s = epo.secs();
    //    int IFAC = 365;
    //    if ((yr % 4) == 0) IFAC = 366;
    //    double t = double(yr) + (doy + h / 23.93447 + m / 1440.0 + s / 86400.0) / (IFAC + 0.2422);
    //    base_pair xypm = meanpole(t, mode);
    //    double m1 = R2AS * xp - xypm[0] / 1000;
    //    double m2 = -(R2AS * yp - xypm[1] / 1000);
    //    double ga2R = 0.6870;
    //    double ga2I = 0.0036;
    //    double m1m2ga2 = m1 * ga2R + m2 * ga2I;
    //    double m2m1ga2 = m2 * ga2R - m1 * ga2I;
    //    m1m2ga2 = m1m2ga2 / 3600.0 / 180.0 * hwa_pi;
    //    m2m1ga2 = m2m1ga2 / 3600.0 / 180.0 * hwa_pi;
    //    Triple uren;
    //    for (int i = 0; i < 3; i++)
    //    {
    //        uren[i] = K_OPL * (m1m2ga2 * Rren[i] + m2m1ga2 * Iren[i]);
    //    }
    //    double tmp = uren[2];
    //    uren[2] = uren[1];
    //    uren[1] = tmp;
    //    xyz2ell_vlbi(xRec, ell1);
    //    Triple neu(uren[2], uren[1], uren[0]);
    //    /*if (ell1[1] > hwa_pi)
    //        ell1[1] = ell1[1] - 2 * hwa_pi;*/
    //    neu2xyz(ell1, neu, dxyz);
    //
    //  _mutex.unlock();
    //  return dxyz;
    //}

    // ---------------------------------------------------------------------
    // <0, 2pi)
    // ---------------------------------------------------------------------
    double null2pi_degree(double angle)
    {
        while (angle > 360.0)
            angle -= 360.0;
        while (angle < 0)
            angle += 360.0;
        return angle;
    }

    double null2pi_rad(double angle)
    {
        while (angle > 2 * hwa_pi)
            angle -= 2 * hwa_pi;
        while (angle < 0)
            angle += 2 * hwa_pi;
        return angle;
    }

    // ---------------------------------------------------------------------
    // Astronomical arguments and fundamental frequencies
    // ---------------------------------------------------------------------
    Matrix DD_arguments(double mjd, double delta)
    {
        Matrix Kij(5, 5), tM(5, 1);                                                                                                                                            // general
        Matrix Delauney(6, 1), l(1, 5), ls(1, 5), F(1, 5), D(1, 5), Om(1, 5), fund_freqDel(5, 1);                                                                              // Delauney
        Matrix dood(6, 3), Doodson0(6, 5), Doodson(6, 5), Doodson_vec(6, 1), Doodson_vecf(12, 1), s(1, 5), h(1, 5), p(1, 5), Ns(1, 5), ps(1, 5), T(1, 5), fund_freqDood(6, 1); // Doodson
        double t0, t1, t2, t3, t4;

        // nulling
        dood.setZero();
        Doodson0.setZero();
        Delauney.setZero();
        Doodson_vec.setZero();

        t0 = 1.0;
        t1 = (mjd - 51544.5 + delta / 86400.0) / 36525.0; // 51544.5 vs. 51545.0.... double T = (dmjd - 51545.0 + delta/86400.0)/36525.0;
        t2 = pow(t1, 2);
        t3 = pow(t1, 3);
        t4 = pow(t1, 4);
        tM << t0 , t1 , t2 , t3 , t4;

        // IERS 2010, p.67 ... l, l', F, D, Omega (F1 ... F5)
        Kij << 134.96340251 , 357.52910918 , 93.27209062 , 297.85019547 , 125.04455501
            , 1717915923.2178 / 3600 , 129596581.0481 / 3600 , 1739527262.8478 / 3600 , 1602961601.2090 / 3600 , -6962890.5431 / 3600 // /3600;
            , 31.8792 / 3600 , -0.5532 / 3600 , -12.7512 / 3600 , -6.3706 / 3600 , 7.4722 / 3600                                      // /3600;
            , 0.051635 / 3600 , 0.000136 / 3600 , -0.001037 / 3600 , 0.006593 / 3600 , 0.007702 / 3600                                // /3600;
            , -0.00024470 / 3600 , -0.00001149 / 3600 , 0.00000417 / 3600 , -0.00003169 / 3600 , -0.00005939 / 3600;                  // /3600;

        fund_freqDel << Kij(1, 0) , Kij(1, 1) , Kij(1, 2) , Kij(1, 3) , Kij(1, 4); // in [degree/Julcent]

        // Delauney arguments, IERS 1000 p.38
        l << Kij(0, 0) , Kij(1, 0) , Kij(2, 0) , Kij(3, 0) , Kij(4, 0);
        ls << Kij(0, 1) , Kij(1, 1) , Kij(2, 1) , Kij(3, 1) , Kij(4, 1);
        F << Kij(0, 2) , Kij(1, 2) , Kij(2, 2) , Kij(3, 2) , Kij(4, 2);
        D << Kij(0, 3) , Kij(1, 3) , Kij(2, 3) , Kij(3, 3) , Kij(4, 3);
        Om << Kij(0, 4) , Kij(1, 4) , Kij(2, 4) , Kij(3, 4) , Kij(4, 4);

        //  Doodson arguments, Dehant (2015) et al., p.178
        s = F + Om;            // mean lunar node
        h = F + Om - D;        // angle of the mean tropic year
        p = -l + F + Om;       // mean lunar perigee
        Ns = -Om;              // angle of the mean lunar node
        ps = -ls + F + Om - D; // angle of perihelion - ps/p'
        T = -s;                // -s [+ gmst + pi] ... added below

        // Delauney
        Delauney << T.row(0).dot(tM.col(0)) , l.row(0).dot(tM.col(0)), ls.row(0).dot(tM.col(0)), F.row(0).dot(tM.col(0)), D.row(0).dot(tM.col(0)), Om.row(0).dot(tM.col(0));
        Delauney = Delauney * D2R;

        // Doodson0 ... T is not ready here!
        Doodson0.row(0) = T;
        Doodson0.row(1) = s;
        Doodson0.row(2) = h;
        Doodson0.row(3) = p;
        Doodson0.row(4) = Ns;
        Doodson0.row(5) = ps;

        // Doodson fundamental frequencies
        fund_freqDood = Doodson0.col(1) / 36525.0 / 24.0; // in [degree/hour] in mean Solar day
        fund_freqDood.row(0) = Eigen::RowVectorXd::Constant(fund_freqDood.cols(), 15.0) - fund_freqDood.row(1) + fund_freqDood.row(2);

        // Doodson ... final modification
        Doodson = Doodson0;
        // convert to <0,360)
        Doodson(0, 0) = null2pi_degree(Doodson(0, 0));
        Doodson(1, 0) = null2pi_degree(Doodson(1, 0));
        Doodson(2, 0) = null2pi_degree(Doodson(2, 0));
        Doodson(3, 0) = null2pi_degree(Doodson(3, 0));
        Doodson(4, 0) = null2pi_degree(Doodson(4, 0));
        Doodson(5, 0) = null2pi_degree(Doodson(5, 0));

        // make a vector
        Doodson_vec = Doodson * tM;

        // get GMST
        gnss_model_ephplan epoGMST;
        double gmst_i = R2D * epoGMST.gmst(mjd);

        // update T for [+ gmst + pi]!
        Doodson_vec(0, 0) = Doodson_vec(0, 0) + gmst_i + 070.0;

        // convert to <0,250) and put to Doodson_vecf
        Doodson_vecf(0, 0) = D2R * null2pi_degree(Doodson_vec(0, 0)); // zde se diky starsi funkci GMST v gephplan.h lisime na 7 des. miste (nema vliv na vysledek)
        Doodson_vecf(1, 0) = D2R * null2pi_degree(Doodson_vec(1, 0));
        Doodson_vecf(2, 0) = D2R * null2pi_degree(Doodson_vec(2, 0));
        Doodson_vecf(3, 0) = D2R * null2pi_degree(Doodson_vec(3, 0));
        Doodson_vecf(4, 0) = D2R * null2pi_degree(Doodson_vec(4, 0));
        Doodson_vecf(5, 0) = D2R * null2pi_degree(Doodson_vec(5, 0));

        // add frequencies
        Doodson_vecf(6, 0) = fund_freqDood(0, 0);
        Doodson_vecf(7, 0) = fund_freqDood(1, 0);
        Doodson_vecf(8, 0) = fund_freqDood(2, 0);
        Doodson_vecf(9, 0) = fund_freqDood(3, 0);
        Doodson_vecf(9, 0) = fund_freqDood(4, 0);
        Doodson_vecf(10, 0) = fund_freqDood(5, 0);

        return Doodson_vecf;
    }

    Matrix dnn_table()
    {
        Matrix dnntab(342, 7);
        dnntab // first 6. cols = Doodson number, 7. col = tide amplitude from TGP
            << 2 , 0 , 0 , 0 , 0 , 0 , 0.632208
            , 2 , 2 , -2 , 0 , 0 , 0 , 0.294107
            , 2 , -1 , 0 , 1 , 0 , 0 , 0.121046
            , 2 , 2 , 0 , 0 , 0 , 0 , 0.079915
            , 2 , 2 , 0 , 0 , 1 , 0 , 0.023818
            , 2 , 0 , 0 , 0 , -1 , 0 , -0.023589
            , 2 , -1 , 2 , -1 , 0 , 0 , 0.022994
            , 2 , -2 , 2 , 0 , 0 , 0 , 0.019333
            , 2 , 1 , 0 , -1 , 0 , 0 , -0.017871
            , 2 , 2 , -3 , 0 , 0 , 1 , 0.017192
            , 2 , -2 , 0 , 2 , 0 , 0 , 0.016018
            , 2 , -3 , 2 , 1 , 0 , 0 , 0.004671
            , 2 , 1 , -2 , 1 , 0 , 0 , -0.004662
            , 2 , -1 , 0 , 1 , -1 , 0 , -0.004519
            , 2 , 3 , 0 , -1 , 0 , 0 , 0.004470
            , 2 , 1 , 0 , 1 , 0 , 0 , 0.004467
            , 2 , 2 , 0 , 0 , 2 , 0 , 0.002589
            , 2 , 2 , -1 , 0 , 0 , -1 , -0.002455
            , 2 , 0 , -1 , 0 , 0 , 1 , -0.002172
            , 2 , 1 , 0 , 1 , 1 , 0 , 0.001972
            , 2 , 3 , 0 , -1 , 1 , 0 , 0.001947
            , 2 , 0 , 1 , 0 , 0 , -1 , 0.001914
            , 2 , 0 , -2 , 2 , 0 , 0 , -0.001898
            , 2 , -3 , 0 , 3 , 0 , 0 , 0.001802
            , 2 , -2 , 3 , 0 , 0 , -1 , 0.001304
            , 2 , 4 , 0 , 0 , 0 , 0 , 0.001170
            , 2 , -1 , 1 , 1 , 0 , -1 , 0.001130
            , 2 , -1 , 3 , -1 , 0 , -1 , 0.001061
            , 2 , 2 , 0 , 0 , -1 , 0 , -0.001022
            , 2 , -1 , -1 , 1 , 0 , 1 , -0.001017
            , 2 , 4 , 0 , 0 , 1 , 0 , 0.001014
            , 2 , -3 , 4 , -1 , 0 , 0 , 0.000901
            , 2 , -1 , 2 , -1 , -1 , 0 , -0.000857
            , 2 , 3 , -2 , 1 , 0 , 0 , 0.000855
            , 2 , 1 , 2 , -1 , 0 , 0 , 0.000855
            , 2 , -4 , 2 , 2 , 0 , 0 , 0.000772
            , 2 , 4 , -2 , 0 , 0 , 0 , 0.000741
            , 2 , 0 , 2 , 0 , 0 , 0 , 0.000741
            , 2 , -2 , 2 , 0 , -1 , 0 , -0.000721
            , 2 , 2 , -4 , 0 , 0 , 2 , 0.000698
            , 2 , 2 , -2 , 0 , -1 , 0 , 0.000658
            , 2 , 1 , 0 , -1 , -1 , 0 , 0.000654
            , 2 , -1 , 1 , 0 , 0 , 0 , -0.000653
            , 2 , 2 , -1 , 0 , 0 , 1 , 0.000633
            , 2 , 2 , 1 , 0 , 0 , -1 , 0.000626
            , 2 , -2 , 0 , 2 , -1 , 0 , -0.000598
            , 2 , -2 , 4 , -2 , 0 , 0 , 0.000590
            , 2 , 2 , 2 , 0 , 0 , 0 , 0.000544
            , 2 , -4 , 4 , 0 , 0 , 0 , 0.000479
            , 2 , -1 , 0 , -1 , -2 , 0 , -0.000464
            , 2 , 1 , 2 , -1 , 1 , 0 , 0.000413
            , 2 , -1 , -2 , 3 , 0 , 0 , -0.000390
            , 2 , 3 , -2 , 1 , 1 , 0 , 0.000373
            , 2 , 4 , 0 , -2 , 0 , 0 , 0.000366
            , 2 , 0 , 0 , 2 , 0 , 0 , 0.000366
            , 2 , 0 , 2 , -2 , 0 , 0 , -0.000360
            , 2 , 0 , 2 , 0 , 1 , 0 , -0.000355
            , 2 , -3 , 3 , 1 , 0 , -1 , 0.000354
            , 2 , 0 , 0 , 0 , -2 , 0 , 0.000329
            , 2 , 4 , 0 , 0 , 2 , 0 , 0.000328
            , 2 , 4 , -2 , 0 , 1 , 0 , 0.000319
            , 2 , 0 , 0 , 0 , 0 , 2 , 0.000302
            , 2 , 1 , 0 , 1 , 2 , 0 , 0.000279
            , 2 , 0 , -2 , 0 , -2 , 0 , -0.000274
            , 2 , -2 , 1 , 0 , 0 , 1 , -0.000272
            , 2 , -2 , 1 , 2 , 0 , -1 , 0.000248
            , 2 , -1 , 1 , -1 , 0 , 1 , -0.000225
            , 2 , 5 , 0 , -1 , 0 , 0 , 0.000224
            , 2 , 1 , -3 , 1 , 0 , 1 , -0.000223
            , 2 , -2 , -1 , 2 , 0 , 1 , -0.000216
            , 2 , 3 , 0 , -1 , 2 , 0 , 0.000211
            , 2 , 1 , -2 , 1 , -1 , 0 , 0.000209
            , 2 , 5 , 0 , -1 , 1 , 0 , 0.000194
            , 2 , -4 , 0 , 4 , 0 , 0 , 0.000185
            , 2 , -3 , 2 , 1 , -1 , 0 , -0.000174
            , 2 , -2 , 1 , 1 , 0 , 0 , -0.000171
            , 2 , 4 , 0 , -2 , 1 , 0 , 0.000159
            , 2 , 0 , 0 , 2 , 1 , 0 , 0.000131
            , 2 , -5 , 4 , 1 , 0 , 0 , 0.000127
            , 2 , 0 , 2 , 0 , 2 , 0 , 0.000120
            , 2 , -1 , 2 , 1 , 0 , 0 , 0.000118
            , 2 , 5 , -2 , -1 , 0 , 0 , 0.000117
            , 2 , 1 , -1 , 0 , 0 , 0 , 0.000108
            , 2 , 2 , -2 , 0 , 0 , 2 , 0.000107
            , 2 , -5 , 2 , 3 , 0 , 0 , 0.000105
            , 2 , -1 , -2 , 1 , -2 , 0 , -0.000102
            , 2 , -3 , 5 , -1 , 0 , -1 , 0.000102
            , 2 , -1 , 0 , 0 , 0 , 1 , 0.000099
            , 2 , -2 , 0 , 0 , -2 , 0 , -0.000096
            , 2 , 0 , -1 , 1 , 0 , 0 , 0.000095
            , 2 , -3 , 1 , 1 , 0 , 1 , -0.000089
            , 2 , 3 , 0 , -1 , -1 , 0 , -0.000085
            , 2 , 1 , 0 , 1 , -1 , 0 , -0.000084
            , 2 , -1 , 2 , 1 , 1 , 0 , -0.000081
            , 2 , 0 , -3 , 2 , 0 , 1 , -0.000077
            , 2 , 1 , -1 , -1 , 0 , 1 , -0.000072
            , 2 , -3 , 0 , 3 , -1 , 0 , -0.000067
            , 2 , 0 , -2 , 2 , -1 , 0 , 0.000066
            , 2 , -4 , 3 , 2 , 0 , -1 , 0.000064
            , 2 , -1 , 0 , 1 , -2 , 0 , 0.000063
            , 2 , 5 , 0 , -1 , 2 , 0 , 0.000063
            , 2 , -4 , 5 , 0 , 0 , -1 , 0.000063
            , 2 , -2 , 4 , 0 , 0 , -2 , 0.000062
            , 2 , -1 , 0 , 1 , 0 , 2 , 0.000062
            , 2 , -2 , -2 , 4 , 0 , 0 , -0.000060
            , 2 , 3 , -2 , -1 , -1 , 0 , 0.000056
            , 2 , -2 , 5 , -2 , 0 , -1 , 0.000053
            , 2 , 0 , -1 , 0 , -1 , 1 , 0.000051
            , 2 , 5 , -2 , -1 , 1 , 0 , 0.000050
            , 1 , 1 , 0 , 0 , 0 , 0 , 0.368645
            , 1 , -1 , 0 , 0 , 0 , 0 , -0.262232
            , 1 , 1 , -2 , 0 , 0 , 0 , -0.121995
            , 1 , -2 , 0 , 1 , 0 , 0 , -0.050208
            , 1 , 1 , 0 , 0 , 1 , 0 , 0.050031
            , 1 , -1 , 0 , 0 , -1 , 0 , -0.049470
            , 1 , 2 , 0 , -1 , 0 , 0 , 0.020620
            , 1 , 0 , 0 , 1 , 0 , 0 , 0.020613
            , 1 , 3 , 0 , 0 , 0 , 0 , 0.011279
            , 1 , -2 , 2 , -1 , 0 , 0 , -0.009530
            , 1 , -2 , 0 , 1 , -1 , 0 , -0.009469
            , 1 , -3 , 2 , 0 , 0 , 0 , -0.008012
            , 1 , 0 , 0 , -1 , 0 , 0 , 0.007414
            , 1 , 1 , 0 , 0 , -1 , 0 , -0.007300
            , 1 , 3 , 0 , 0 , 1 , 0 , 0.007227
            , 1 , 1 , -3 , 0 , 0 , 1 , -0.007131
            , 1 , -3 , 0 , 2 , 0 , 0 , -0.006644
            , 1 , 1 , 2 , 0 , 0 , 0 , 0.005249
            , 1 , 0 , 0 , 1 , 1 , 0 , 0.004137
            , 1 , 2 , 0 , -1 , 1 , 0 , 0.004087
            , 1 , 0 , 2 , -1 , 0 , 0 , 0.003944
            , 1 , 2 , -2 , 1 , 0 , 0 , 0.003943
            , 1 , 3 , -2 , 0 , 0 , 0 , 0.003420
            , 1 , -1 , 2 , 0 , 0 , 0 , 0.003418
            , 1 , 1 , 1 , 0 , 0 , -1 , 0.002885
            , 1 , 1 , -1 , 0 , 0 , 1 , 0.002884
            , 1 , 4 , 0 , -1 , 0 , 0 , 0.002160
            , 1 , -4 , 2 , 1 , 0 , 0 , -0.001936
            , 1 , 0 , -2 , 1 , 0 , 0 , 0.001934
            , 1 , -2 , 2 , -1 , -1 , 0 , -0.001798
            , 1 , 3 , 0 , -2 , 0 , 0 , 0.001690
            , 1 , -1 , 0 , 2 , 0 , 0 , 0.001689
            , 1 , -1 , 0 , 0 , -2 , 0 , 0.001516
            , 1 , 3 , 0 , 0 , 2 , 0 , 0.001514
            , 1 , -3 , 2 , 0 , -1 , 0 , -0.001511
            , 1 , 4 , 0 , -1 , 1 , 0 , 0.001383
            , 1 , 0 , 0 , -1 , -1 , 0 , 0.001372
            , 1 , 1 , -2 , 0 , -1 , 0 , 0.001371
            , 1 , -3 , 0 , 2 , -1 , 0 , -0.001253
            , 1 , 1 , 0 , 0 , 2 , 0 , -0.001075
            , 1 , 1 , -1 , 0 , 0 , -1 , 0.001020
            , 1 , -1 , -1 , 0 , 0 , 1 , 0.000901
            , 1 , 0 , 2 , -1 , 1 , 0 , 0.000865
            , 1 , -1 , 1 , 0 , 0 , -1 , -0.000794
            , 1 , -1 , -2 , 2 , 0 , 0 , 0.000788
            , 1 , 2 , -2 , 1 , 1 , 0 , 0.000782
            , 1 , -4 , 0 , 3 , 0 , 0 , -0.000747
            , 1 , -1 , 2 , 0 , 1 , 0 , -0.000745
            , 1 , 3 , -2 , 0 , 1 , 0 , 0.000670
            , 1 , 2 , 0 , -1 , -1 , 0 , -0.000603
            , 1 , 0 , 0 , 1 , -1 , 0 , -0.000597
            , 1 , -2 , 2 , 1 , 0 , 0 , 0.000542
            , 1 , 4 , -2 , -1 , 0 , 0 , 0.000542
            , 1 , -3 , 3 , 0 , 0 , -1 , -0.000541
            , 1 , -2 , 1 , 1 , 0 , -1 , -0.000469
            , 1 , -2 , 3 , -1 , 0 , -1 , -0.000440
            , 1 , 0 , -2 , 1 , -1 , 0 , 0.000438
            , 1 , -2 , -1 , 1 , 0 , 1 , 0.000422
            , 1 , 4 , -2 , 1 , 0 , 0 , 0.000410
            , 1 , -4 , 4 , -1 , 0 , 0 , -0.000374
            , 1 , -4 , 2 , 1 , -1 , 0 , -0.000365
            , 1 , 5 , -2 , 0 , 0 , 0 , 0.000345
            , 1 , 3 , 0 , -2 , 1 , 0 , 0.000335
            , 1 , -5 , 2 , 2 , 0 , 0 , -0.000321
            , 1 , 2 , 0 , 1 , 0 , 0 , -0.000319
            , 1 , 1 , 3 , 0 , 0 , -1 , 0.000307
            , 1 , -2 , 0 , 1 , -2 , 0 , 0.000291
            , 1 , 4 , 0 , -1 , 2 , 0 , 0.000290
            , 1 , 1 , -4 , 0 , 0 , 2 , -0.000289
            , 1 , 5 , 0 , -2 , 0 , 0 , 0.000286
            , 1 , -1 , 0 , 2 , 1 , 0 , 0.000275
            , 1 , -2 , 1 , 0 , 0 , 0 , 0.000271
            , 1 , 4 , -2 , 1 , 1 , 0 , 0.000263
            , 1 , -3 , 4 , -2 , 0 , 0 , -0.000245
            , 1 , -1 , 3 , 0 , 0 , -1 , 0.000225
            , 1 , 3 , -3 , 0 , 0 , 1 , 0.000225
            , 1 , 5 , -2 , 0 , 1 , 0 , 0.000221
            , 1 , 1 , 2 , 0 , 1 , 0 , -0.000202
            , 1 , 2 , 0 , 1 , 1 , 0 , -0.000200
            , 1 , -5 , 4 , 0 , 0 , 0 , -0.000199
            , 1 , -2 , 0 , -1 , -2 , 0 , 0.000192
            , 1 , 5 , 0 , -2 , 1 , 0 , 0.000183
            , 1 , 1 , 2 , -2 , 0 , 0 , 0.000183
            , 1 , 1 , -2 , 2 , 0 , 0 , 0.000183
            , 1 , -2 , 2 , 1 , 1 , 0 , -0.000170
            , 1 , 0 , 3 , -1 , 0 , -1 , 0.000169
            , 1 , 2 , -3 , 1 , 0 , 1 , 0.000168
            , 1 , -2 , -2 , 3 , 0 , 0 , 0.000162
            , 1 , -1 , 2 , -2 , 0 , 0 , 0.000149
            , 1 , -4 , 3 , 1 , 0 , -1 , -0.000147
            , 1 , -4 , 0 , 3 , -1 , 0 , -0.000141
            , 1 , -1 , -2 , 2 , -1 , 0 , 0.000138
            , 1 , -2 , 0 , 3 , 0 , 0 , 0.000136
            , 1 , 4 , 0 , -3 , 0 , 0 , 0.000136
            , 1 , 0 , 1 , 1 , 0 , -1 , 0.000127
            , 1 , 2 , -1 , -1 , 0 , 1 , 0.000127
            , 1 , 2 , -2 , 1 , -1 , 0 , -0.000126
            , 1 , 0 , 0 , -1 , -2 , 0 , -0.000121
            , 1 , 2 , 0 , 1 , 2 , 0 , -0.000121
            , 1 , 2 , -2 , -1 , -1 , 0 , 0.000117
            , 1 , 0 , 0 , 1 , 2 , 0 , -0.000116
            , 1 , 0 , 1 , 0 , 0 , 0 , -0.000114
            , 1 , 2 , -1 , 0 , 0 , 0 , -0.000114
            , 1 , 0 , 2 , -1 , -1 , 0 , -0.000114
            , 1 , -1 , -2 , 0 , -2 , 0 , 0.000114
            , 1 , -3 , 1 , 0 , 0 , 1 , 0.000113
            , 1 , 3 , -2 , 0 , -1 , 0 , 0.000109
            , 1 , -1 , -1 , 0 , -1 , 1 , 0.000108
            , 1 , 4 , -2 , -1 , 1 , 0 , 0.000106
            , 1 , 2 , 1 , -1 , 0 , -1 , -0.000106
            , 1 , 0 , -1 , 1 , 0 , 1 , -0.000106
            , 1 , -2 , 4 , -1 , 0 , 0 , 0.000105
            , 1 , 4 , -4 , 1 , 0 , 0 , 0.000104
            , 1 , -3 , 1 , 2 , 0 , -1 , -0.000103
            , 1 , -3 , 3 , 0 , -1 , -1 , -0.000100
            , 1 , 1 , 2 , 0 , 2 , 0 , -0.000100
            , 1 , 1 , -2 , 0 , -2 , 0 , -0.000100
            , 1 , 3 , 0 , 0 , 3 , 0 , 0.000099
            , 1 , -1 , 2 , 0 , -1 , 0 , -0.000098
            , 1 , -2 , 1 , -1 , 0 , 1 , 0.000093
            , 1 , 0 , -3 , 1 , 0 , 1 , 0.000093
            , 1 , -3 , -1 , 2 , 0 , 1 , 0.000090
            , 1 , 2 , 0 , -1 , 2 , 0 , -0.000088
            , 1 , 6 , -2 , -1 , 0 , 0 , 0.000083
            , 1 , 2 , 2 , -1 , 0 , 0 , -0.000083
            , 1 , -1 , 1 , 0 , -1 , -1 , -0.000082
            , 1 , -2 , 3 , -1 , -1 , -1 , -0.000081
            , 1 , -1 , 0 , 0 , 0 , 2 , -0.000079
            , 1 , -5 , 0 , 4 , 0 , 0 , -0.000077
            , 1 , 1 , 0 , 0 , 0 , -2 , -0.000075
            , 1 , -2 , 1 , 1 , -1 , -1 , -0.000075
            , 1 , 1 , -1 , 0 , 1 , 1 , -0.000075
            , 1 , 1 , 2 , 0 , 0 , -2 , 0.000071
            , 1 , -3 , 1 , 1 , 0 , 0 , 0.000071
            , 1 , -4 , 4 , -1 , -1 , 0 , -0.000071
            , 1 , 1 , 0 , -2 , -1 , 0 , 0.000068
            , 1 , -2 , -1 , 1 , -1 , 1 , 0.000068
            , 1 , -3 , 2 , 2 , 0 , 0 , 0.000065
            , 1 , 5 , -2 , -2 , 0 , 0 , 0.000065
            , 1 , 3 , -4 , 2 , 0 , 0 , 0.000064
            , 1 , 1 , -2 , 0 , 0 , 2 , 0.000064
            , 1 , -1 , 4 , -2 , 0 , 0 , 0.000064
            , 1 , 2 , 2 , -1 , 1 , 0 , -0.000064
            , 1 , -5 , 2 , 2 , -1 , 0 , -0.000060
            , 1 , 1 , -3 , 0 , -1 , 1 , 0.000056
            , 1 , 1 , 1 , 0 , 1 , -1 , 0.000056
            , 1 , 6 , -2 , -1 , 1 , 0 , 0.000053
            , 1 , -2 , 2 , -1 , -2 , 0 , 0.000053
            , 1 , 4 , -2 , 1 , 2 , 0 , 0.000053
            , 1 , -6 , 4 , 1 , 0 , 0 , -0.000053
            , 1 , 5 , -4 , 0 , 0 , 0 , 0.000053
            , 1 , -3 , 4 , 0 , 0 , 0 , 0.000053
            , 1 , 1 , 2 , -2 , 1 , 0 , 0.000052
            , 1 , -2 , 1 , 0 , -1 , 0 , 0.000050
            , 0 , 2 , 0 , 0 , 0 , 0 , -0.066607
            , 0 , 1 , 0 , -1 , 0 , 0 , -0.035184
            , 0 , 0 , 2 , 0 , 0 , 0 , -0.030988
            , 0 , 0 , 0 , 0 , 1 , 0 , 0.027929
            , 0 , 2 , 0 , 0 , 1 , 0 , -0.027616
            , 0 , 3 , 0 , -1 , 0 , 0 , -0.012753
            , 0 , 1 , -2 , 1 , 0 , 0 , -0.006728
            , 0 , 2 , -2 , 0 , 0 , 0 , -0.005837
            , 0 , 3 , 0 , -1 , 1 , 0 , -0.005286
            , 0 , 0 , 1 , 0 , 0 , -1 , -0.004921
            , 0 , 2 , 0 , -2 , 0 , 0 , -0.002884
            , 0 , 2 , 0 , 0 , 2 , 0 , -0.002583
            , 0 , 3 , -2 , 1 , 0 , 0 , -0.002422
            , 0 , 1 , 0 , -1 , -1 , 0 , 0.002310
            , 0 , 1 , 0 , -1 , 1 , 0 , 0.002283
            , 0 , 4 , -2 , 0 , 0 , 0 , -0.002037
            , 0 , 1 , 0 , 1 , 0 , 0 , 0.001883
            , 0 , 0 , 3 , 0 , 0 , -1 , -0.001811
            , 0 , 4 , 0 , -2 , 0 , 0 , -0.001687
            , 0 , 3 , -2 , 1 , 1 , 0 , -0.001004
            , 0 , 3 , -2 , -1 , 0 , 0 , -0.000925
            , 0 , 4 , -2 , 0 , 1 , 0 , -0.000844
            , 0 , 0 , 2 , 0 , 1 , 0 , 0.000766
            , 0 , 1 , 0 , 1 , 1 , 0 , 0.000766
            , 0 , 4 , 0 , -2 , 1 , 0 , -0.000700
            , 0 , 3 , 0 , -1 , 2 , 0 , -0.000495
            , 0 , 5 , -2 , -1 , 0 , 0 , -0.000492
            , 0 , 1 , 2 , -1 , 0 , 0 , 0.000491
            , 0 , 1 , -2 , 1 , -1 , 0 , 0.000483
            , 0 , 1 , -2 , 1 , 1 , 0 , 0.000437
            , 0 , 2 , -2 , 0 , -1 , 0 , -0.000416
            , 0 , 2 , -3 , 0 , 0 , 1 , -0.000384
            , 0 , 2 , -2 , 0 , 1 , 0 , 0.000374
            , 0 , 0 , 2 , -2 , 0 , 0 , -0.000312
            , 0 , 1 , -3 , 1 , 0 , 1 , -0.000288
            , 0 , 0 , 0 , 0 , 2 , 0 , -0.000273
            , 0 , 0 , 1 , 0 , 0 , 1 , 0.000259
            , 0 , 1 , 2 , -1 , 1 , 0 , 0.000245
            , 0 , 3 , 0 , -3 , 0 , 0 , -0.000232
            , 0 , 2 , 1 , 0 , 0 , -1 , 0.000229
            , 0 , 1 , -1 , -1 , 0 , 1 , -0.000216
            , 0 , 1 , 0 , 1 , 2 , 0 , 0.000206
            , 0 , 5 , -2 , -1 , 1 , 0 , -0.000204
            , 0 , 2 , -1 , 0 , 0 , 1 , -0.000202
            , 0 , 2 , 2 , -2 , 0 , 0 , 0.000200
            , 0 , 1 , -1 , 0 , 0 , 0 , 0.000195
            , 0 , 5 , 0 , -3 , 0 , 0 , -0.000190
            , 0 , 2 , 0 , -2 , 1 , 0 , 0.000187
            , 0 , 1 , 1 , -1 , 0 , -1 , 0.000180
            , 0 , 3 , -4 , 1 , 0 , 0 , -0.000179
            , 0 , 0 , 2 , 0 , 2 , 0 , 0.000170
            , 0 , 2 , 0 , -2 , -1 , 0 , 0.000153
            , 0 , 4 , -3 , 0 , 0 , 1 , -0.000137
            , 0 , 3 , -1 , -1 , 0 , 1 , -0.000119
            , 0 , 0 , 2 , 0 , 0 , -2 , -0.000119
            , 0 , 3 , -3 , 1 , 0 , 1 , -0.000112
            , 0 , 2 , -4 , 2 , 0 , 0 , -0.000110
            , 0 , 4 , -2 , -2 , 0 , 0 , -0.000110
            , 0 , 3 , 1 , -1 , 0 , -1 , 0.000107
            , 0 , 5 , -4 , 1 , 0 , 0 , -0.000095
            , 0 , 3 , -2 , -1 , -1 , 0 , -0.000095
            , 0 , 3 , -2 , 1 , 2 , 0 , -0.000091
            , 0 , 4 , -4 , 0 , 0 , 0 , -0.000090
            , 0 , 6 , -2 , -2 , 0 , 0 , -0.000081
            , 0 , 5 , 0 , -3 , 1 , 0 , -0.000079
            , 0 , 4 , -2 , 0 , 2 , 0 , -0.000079
            , 0 , 2 , 2 , -2 , 1 , 0 , 0.000077
            , 0 , 0 , 4 , 0 , 0 , -2 , -0.000073
            , 0 , 3 , -1 , 0 , 0 , 0 , 0.000069
            , 0 , 3 , -3 , -1 , 0 , 1 , -0.000067
            , 0 , 4 , 0 , -2 , 2 , 0 , -0.000066
            , 0 , 1 , -2 , -1 , -1 , 0 , 0.000065
            , 0 , 2 , -1 , 0 , 0 , -1 , 0.000064
            , 0 , 4 , -4 , 2 , 0 , 0 , -0.000062
            , 0 , 2 , 1 , 0 , 1 , -1 , 0.000060
            , 0 , 3 , -2 , -1 , 1 , 0 , 0.000059
            , 0 , 4 , -3 , 0 , 1 , 1 , -0.000056
            , 0 , 2 , 0 , 0 , 3 , 0 , 0.000055
            , 0 , 6 , -4 , 0 , 0 , 0 , -0.000051;
        return dnntab;
    }

    base_pair libiers_tdfrph(Matrix IDOOD, double mjd, double leap)
    {
        double T = (mjd - 51544.5 + (leap + 32.184) / 86400.0) / 36525.0;
        double DAYFR = mjd - int(mjd);
        double F1 = 134.9634025100 + T * (477198.8675605000 + T * (0.0088553333 + T * (0.0000143431 + T * (-0.0000000680))));
        double F2 = 357.5291091806 + T * (35999.0502911389 + T * (-0.0001536667 + T * (0.0000000378 + T * (-0.0000000032))));
        double F3 = 93.2720906200 + T * (483202.0174577222 + T * (-0.0035420000 + T * (-0.0000002881 + T * (0.0000000012))));
        double F4 = 297.8501954694 + T * (445267.1114469445 + T * (-0.0017696111 + T * (0.0000018314 + T * (-0.0000000088))));
        double F5 = 125.0445550100 + T * (-1934.1362619722 + T * (0.0020756111 + T * (0.0000021394 + T * (-0.0000000165))));
        double D[6];
        D[0] = 360.0 * DAYFR - F4;
        D[1] = F3 + F5;
        D[2] = D[1] - F4;
        D[3] = D[1] - F1;
        D[4] = -F5;
        D[5] = D[2] - F2;
        double FD1 = 0.0362916471 + 0.0000000013 * T;
        double FD2 = 0.0027377786;
        double FD3 = 0.0367481951 - 0.0000000005 * T;
        double FD4 = 0.0338631920 - 0.0000000003 * T;
        double FD5 = -0.0001470938 + 0.0000000003 * T;
        double DD[6];
        DD[0] = 1.0 - FD4;
        DD[1] = FD3 + FD5;
        DD[2] = DD[1] - FD4;
        DD[3] = DD[1] - FD1;
        DD[4] = -FD5;
        DD[5] = DD[2] - FD2;
        double FREQ = 0.0, PHASE = 0.0;
        for (int I = 0; I < 6; I++)
        {
            FREQ = FREQ + IDOOD(0, I) * DD[I];
            PHASE = PHASE + IDOOD(0, I) * D[I];
        }
        PHASE = fmod(PHASE, 360.0);
        if (PHASE < 0)
            PHASE = PHASE + 360.0;
        base_pair ret;
        ret[0] = FREQ;
        ret[1] = PHASE;
        return ret;
    }

    Matrix libiers_admint_part1(Matrix TAMP, Matrix TPH, double TAMPT[], Matrix IDT, double mjd, double leap)
    {
        Matrix RL(3, 11), AIM(3, 11), RF(1, 11);

        base_pair frpr;
        for (int c = 0; c < 11; c++)
        {
            RL(0, c) = TAMP(0, c) * cos(TPH(0, c) * D2R) / abs(TAMPT[c]);
            AIM(0, c) = TAMP(0, c) * sin(TPH(0, c) * D2R) / abs(TAMPT[c]);
            RL(1, c) = TAMP(1, c) * cos(TPH(1, c) * D2R) / abs(TAMPT[c]);
            AIM(1, c) = TAMP(1, c) * sin(TPH(1, c) * D2R) / abs(TAMPT[c]);
            RL(2, c) = TAMP(2, c) * cos(TPH(2, c) * D2R) / abs(TAMPT[c]);
            AIM(2, c) = TAMP(2, c) * sin(TPH(2, c) * D2R) / abs(TAMPT[c]);
            frpr = libiers_tdfrph(IDT.row(c), mjd, leap);
            RF(0, c) = frpr[0];
        }
        int key[11];
        double min = 1e5;
        int ind = 0;
        Matrix tmp = RF;
        for (int i = 0; i < 11; i++)
        {
            for (int j = 0; j < 11; j++)
            {
                if (min >= tmp(0, j))
                {
                    min = tmp(0, j);
                    ind = j;
                }
            }
            tmp(0, ind) = 1e5 + 0.1;
            min = 1e5;
            key[i] = ind;
        }
        Matrix ret(7, 11);
        for (int c = 0; c < 11; c++)
        {
            ret.block(0, c, 1, 1) = RF.col(key[c]);
            ret.block(1, c, 3, 1) = RL.col(key[c]);
            ret.block(4, c, 3, 1) = AIM.col(key[c]);
        }
        return ret;
    }

    vector<double> libiers_spline(int NN, Matrix X, Matrix U, Matrix A)
    {
        int N = abs(NN);
        vector<double> S;
        if (N <= 3)
        {
            for (int I = 1; I <= N; I++)
            {
                S.push_back(0.0);
            }
            return S;
        }
        double U1 = U(0, 1) - U(0, 0);
        double X1 = X(0, 1) - X(0, 0);
        double U2 = U(0, 2) - U(0, 0);
        double X2 = X(0, 2) - X(0, 0);
        double Q1 = (U1 / X1 / X1 - U2 / X2 / X2) / (1.0 / X1 - 1.0 / X2);
        U1 = U(0, N - 2) - U(0, N - 1);
        X1 = X(0, N - 2) - X(0, N - 1);
        U2 = U(0, N - 3) - U(0, N - 1);
        X2 = X(0, N - 3) - X(0, N - 1);
        double QN = (U1 / X1 / X1 - U2 / X2 / X2) / (1.0 / X1 - 1.0 / X2);
        if (NN <= 0)
        {
            Q1 = S[0];
            QN = S[1];
        }
        S.push_back(6.0 * ((U(0, 1) - U(0, 0)) / (X(0, 1) - X(0, 0)) - Q1));
        int N1 = N - 1;
        for (int I = 1; I < N1; I++)
        {
            S.push_back((U(0, I - 1) / (X(0, I) - X(0, I - 1)) - U(0, I) * (1.0 / (X(0, I) - X(0, I - 1)) + 1.0 / (X(0, I + 1) - X(0, I))) + U(0, I + 1) / (X(0, I + 1) - X(0, I))) * 6.0);
        }
        S.push_back(6.0 * (QN + (U(0, N1 - 1) - U(0, N - 1)) / (X(0, N - 1) - X(0, N1 - 1))));
        A(0, 0) = 2.0 * (X(0, 1) - X(0, 0));
        A(0, 1) = 1.5 * (X(0, 1) - X(0, 0)) + 2.0 * (X(0, 2) - X(0, 1));
        S[1] = S[1] - 0.5 * S[0];
        double C;
        for (int I = 2; I < N1; I++)
        {
            C = (X(0, I) - X(0, I - 1)) / A(0, I - 1);
            A(0, I) = 2.0 * (X(0, I + 1) - X(0, I - 1)) - C * (X(0, I) - X(0, I - 1));
            S[I] = S[I] - C * S[I - 1];
        }
        C = (X(0, N - 1) - X(0, N1 - 1)) / A(0, N1 - 1);
        A(0, N - 1) = (2.0 - C) * (X(0, N - 1) - X(0, N1 - 1));
        S[N - 1] = S[N - 1] - C * S[N1 - 1];
        S[N - 1] = S[N - 1] / A(0, N - 1);
        for (int J = 1; J <= N1; J++)
        {
            int I = N - J;
            S[I - 1] = (S[I - 1] - (X(0, I) - X(0, I - 1)) * S[I]) / A(0, I - 1);
        }
        return S;
    }

    Matrix libiers_eval(vector<double> Y, int NN, Matrix X, Matrix U, vector<double> S)
    {
        unsigned int nt = Y.size();
        Matrix EVAL(nt, 1);
        EVAL.setZero();
        vector<int> id1;
        for (unsigned int I = 0; I < nt; I++)
        {
            if (Y[I] <= X(0, 0))
                id1.push_back(I);
        }
        if (!id1.empty())
            for (unsigned int I = 0; I < id1.size(); I++)
                EVAL(id1[I], 0) = U(0, 0);

        vector<int> id2;
        for (unsigned int I = 0; I < nt; I++)
        {
            if (Y[I] >= X(0, NN - 1))
                id2.push_back(I);
        }
        if (!id2.empty())
            for (unsigned int I = 0; I < id2.size(); I++)
                EVAL(id2[I], 0) = U(0, NN - 1);

        vector<double> YY;
        bool findid1 = false, findid2 = false;
        for (unsigned int i = 0; i < Y.size(); i++)
        {
            for (unsigned int m = 0; m < id1.size(); m++)
            {
                if (id1[m] == i)
                {
                    findid1 = true;
                    break;
                }
            }
            if (findid1 == false)
            {
                for (unsigned int m = 0; m < id2.size(); m++)
                {
                    if (id2[m] == i)
                    {
                        findid2 = true;
                        break;
                    }
                }
            }
            if (findid1 == false && findid2 == false)
                YY.push_back(Y[i]);
            else
            {
                findid1 = false;
                findid2 = false;
            }
        }

        int lY = YY.size();
        Matrix K1(1, lY), K2(1, lY);
        K1.setZero(); K2.setZero();
        for (unsigned int i = 0; i < lY; i++)
        {
            for (int K = 1; K < NN; K++)
            {
                if (X(0, K - 1) < YY[i] && X(0, K) >= YY[i])
                {
                    K1(0, i) = K - 1;
                    K2(0, i) = K;
                }
            }
        }

        vector<double> DY, DY1, DK, DELI, FF1, FF2, F1, F2, F3, EVAL1;
        for (unsigned int I = 0; I < YY.size(); I++)
        {
            DY.push_back(X(0, int(K2(0, I))) - YY[I]);
            DY1.push_back(YY[I] - X(0, int(K1(0, I))));
            DK.push_back(X(0, int(K2(0, I))) - X(0, int(K1(0, I))));
            DELI.push_back(1.0 / (6.0 * DK[I]));
            FF1.push_back(S[int(K1(0, I))] * pow(DY[I], 3));
            FF2.push_back(S[int(K2(0, I))] * pow(DY1[I], 3));
            F1.push_back((FF1[I] + FF2[I]) * DELI[I]);
            F2.push_back(DY1[I] * (U(0, int(K2(0, I))) / DK[I] - S[int(K2(0, I))] * DK[I] / 6.0));
            F3.push_back(DY[I] * (U(0, int(K1(0, I))) / DK[I] - S[int(K1(0, I))] * DK[I] / 6.0));
            EVAL1.push_back(F1[I] + F2[I] + F3[I]);
        }

        findid1 = false;
        findid2 = false;
        unsigned int n1 = 0;
        for (unsigned int i = 0; i < nt; i++)
        {
            for (unsigned int m = 0; m < id1.size(); m++)
            {
                if (id1[m] == i)
                {
                    findid1 = true;
                    break;
                }
            }
            if (findid1 == false)
            {
                for (unsigned int m = 0; m < id2.size(); m++)
                {
                    if (id2[m] == i)
                    {
                        findid2 = true;
                        break;
                    }
                }
            }
            if (findid1 == false && findid2 == false)
            {
                EVAL(i, 0) = EVAL1[n1];
                n1++;
            }
            else
            {
                findid1 = false;
                findid2 = false;
            }
        }

        return EVAL;
    }

    pair<Matrix, Matrix> libiers_admint(Matrix RF, Matrix RL, Matrix AIM, Matrix F, Matrix P, Matrix TAMP, Matrix IDD1)
    {
        int NLP = 0, NDI = 0, NSD = 0;
        vector<double> ZDR, ZDI, DR, DI, SDR, SDI;
        int NIN = RF.cols();
        Matrix SCR = AIM;
        for (int I = 0; I < NIN; I++)
        {
            if (RF(0, I) < 0.5)
                NLP = NLP + 1;
            if (RF(0, I) < 1.5 && RF(0, I) > 0.5)
                NDI = NDI + 1;
            if (RF(0, I) < 2.5 && RF(0, I) > 1.5)
                NSD = NSD + 1;
        }

        if (NLP != 0.0)
        {
            ZDR = libiers_spline(NLP, RF, RL, SCR);
            ZDI = libiers_spline(NLP, RF, AIM, SCR);
        }


        DR = libiers_spline(NDI, RF.block(0, NLP, RF.rows(), RF.cols() - NLP), RL.block(0, NLP, RL.rows(), RL.cols() - NLP), SCR);
        DI = libiers_spline(NDI, RF.block(0, NLP, RF.rows(), RF.cols() - NLP), AIM.block(0, NLP, AIM.rows(), AIM.cols() - NLP), SCR);
        SDR = libiers_spline(NSD, RF.block(0, NLP + NDI, RF.rows(), RF.cols() - NLP - NDI), RL.block(0, NLP + NDI, RL.rows(), RL.cols() - NLP - NDI), SCR);
        SDI = libiers_spline(NSD, RF.block(0, NLP + NDI, RF.rows(), RF.cols() - NLP - NDI), AIM.block(0, NLP + NDI, AIM.rows(), AIM.cols() - NLP - NDI), SCR);
        vector<double> SF;
        for (int I = 0; I < IDD1.size(); I++)
        {
            if (IDD1(I, 0) == 0)
            {
                SF.push_back(F(0, I));
            }
        }
        Matrix RE_lp = libiers_eval(SF, NLP, RF, RL, ZDR);
        Matrix AM_lp = libiers_eval(SF, NLP, RF, AIM, ZDI);
        SF.clear();
        for (int I = 0; I < IDD1.size(); I++)
        {
            if (IDD1(I, 0) == 1)
            {
                SF.push_back(F(0, I));
            }
        }
        Matrix RE_diu = libiers_eval(SF, NDI, RF.block(0, NLP, RF.rows(), RF.cols() - NLP), RL.block(0, NLP, RL.rows(), RL.cols() - NLP), DR);
        Matrix AM_diu = libiers_eval(SF, NDI, RF.block(0, NLP, RF.rows(), RF.cols() - NLP), AIM.block(0, NLP, AIM.rows(), AIM.cols() - NLP), DI);

        SF.clear();
        for (int I = 0; I < IDD1.size(); I++)
        {
            if (IDD1(I, 0) == 2)
            {
                SF.push_back(F(0, I));
            }
        }
        Matrix RE_smd = libiers_eval(SF, NSD, RF.block(0, NLP + NDI, RF.rows(), RF.cols() - NLP - NDI), RL.block(0, NLP + NDI, RL.rows(), RL.cols() - NLP - NDI), SDR);
        Matrix AM_smd = libiers_eval(SF, NSD, RF.block(0, NLP + NDI, RF.rows(), RF.cols() - NLP - NDI), AIM.block(0, NLP + NDI, AIM.rows(), AIM.cols() - NLP - NDI), SDI);

        Matrix AMP1(RE_smd.rows(), 1), AMP2(RE_diu.rows(), 1), AMP3(RE_lp.rows(), 1);
        int n1 = 0, n2 = 0, n3 = 0;
        for (int I = 0; I < IDD1.size(); I++)
        {
            if (IDD1(I, 0) == 2)
            {
                AMP1(n1, 0) = TAMP(0, I) * sqrt(RE_smd(n1, 0) * RE_smd(n1, 0) + AM_smd(n1, 0) * AM_smd(n1, 0));
                n1++;
            }
            if (IDD1(I, 0) == 1)
            {
                AMP2(n2, 0) = TAMP(0, I) * sqrt(RE_diu(n2, 0) * RE_diu(n2, 0) + AM_diu(n2, 0) * AM_diu(n2, 0));
                n2++;
            }
            if (IDD1(I, 0) == 0)
            {
                AMP3(n3, 0) = TAMP(0, I) * sqrt(RE_lp(n3, 0) * RE_lp(n3, 0) + AM_lp(n3, 0) * AM_lp(n3, 0));
                n3++;
            }
        }
        Matrix AMP(1, TAMP.cols());

        AMP.block(0, 0, AMP.rows(), AMP1.rows()) = AMP1.transpose();

        AMP.block(0, AMP1.rows(), AMP.rows(), AMP2.rows()) = AMP2.transpose();

        AMP.block(0, AMP1.rows() + AMP2.rows(), AMP.rows(), AMP3.rows()) = AMP3.transpose();

        Matrix dP(1, TAMP.cols());
        for (int i = 0; i < AM_smd.rows(); i++)
        {
            dP(0, i) = atan2(AM_smd(i, 0), RE_smd(i, 0)) * R2D;
        }
        for (int i = 0; i < AM_diu.rows(); i++)
        {
            dP(0, i + AM_smd.rows()) = atan2(AM_diu(i, 0), RE_diu(i, 0)) * R2D;
        }
        for (int i = 0; i < AM_lp.rows(); i++)
        {
            dP(0, i + AM_smd.rows() + AM_diu.rows()) = atan2(AM_lp(i, 0), RE_lp(i, 0)) * R2D;
        }
        for (int i = 0; i < TAMP.cols(); i++)
        {
            if (IDD1(i, 0) == 0)
                P(0, i) = P(0, i) + 180.0;
            if (IDD1(i, 0) == 1)
                P(0, i) = P(0, i) + 90.0;
        }
        P = P + dP;
        for (int i = 0; i < P.cols(); i++)
        {
            if (P(0, i) > 180.0)
                P(0, i) = P(0, i) - 360.0;
        }
        pair<Matrix, Matrix> ret(AMP, P);
        return ret;
    }

    Matrix libiers_tdfrph_call(double mjd, double leap)
    {
        Matrix TAMP(1, 342);
        Matrix IDD(342, 6);
        TAMP
            << .632208 , .294107 , .121046 , .079915 , .023818 , -.023589 , .022994
            , .019333 , -.017871 , .017192 , .016018 , .004671 , -.004662 , -.004519
            , .004470 , .004467 , .002589 , -.002455 , -.002172 , .001972 , .001947
            , .001914 , -.001898 , .001802 , .001304 , .001170 , .001130 , .001061
            , -.001022 , -.001017 , .001014 , .000901 , -.000857 , .000855 , .000855
            , .000772 , .000741 , .000741 , -.000721 , .000698 , .000658 , .000654
            , -.000653 , .000633 , .000626 , -.000598 , .000590 , .000544 , .000479
            , -.000464 , .000413 , -.000390 , .000373 , .000366 , .000366 , -.000360
            , -.000355 , .000354 , .000329 , .000328 , .000319 , .000302 , .000279
            , -.000274 , -.000272 , .000248 , -.000225 , .000224 , -.000223 , -.000216
            , .000211 , .000209 , .000194 , .000185 , -.000174 , -.000171 , .000159
            , .000131 , .000127 , .000120 , .000118 , .000117 , .000108 , .000107
            , .000105 , -.000102 , .000102 , .000099 , -.000096 , .000095 , -.000089
            , -.000085 , -.000084 , -.000081 , -.000077 , -.000072 , -.000067 , .000066
            , .000064 , .000063 , .000063 , .000063 , .000062 , .000062 , -.000060
            , .000056 , .000053 , .000051 , .000050 , .368645 , -.262232 , -.121995
            , -.050208 , .050031 , -.049470 , .020620 , .020613 , .011279 , -.009530
            , -.009469 , -.008012 , .007414 , -.007300 , .007227 , -.007131 , -.006644
            , .005249 , .004137 , .004087 , .003944 , .003943 , .003420 , .003418
            , .002885 , .002884 , .002160 , -.001936 , .001934 , -.001798 , .001690
            , .001689 , .001516 , .001514 , -.001511 , .001383 , .001372 , .001371
            , -.001253 , -.001075 , .001020 , .000901 , .000865 , -.000794 , .000788
            , .000782 , -.000747 , -.000745 , .000670 , -.000603 , -.000597 , .000542
            , .000542 , -.000541 , -.000469 , -.000440 , .000438 , .000422 , .000410
            , -.000374 , -.000365 , .000345 , .000335 , -.000321 , -.000319 , .000307
            , .000291 , .000290 , -.000289 , .000286 , .000275 , .000271 , .000263
            , -.000245 , .000225 , .000225 , .000221 , -.000202 , -.000200 , -.000199
            , .000192 , .000183 , .000183 , .000183 , -.000170 , .000169 , .000168
            , .000162 , .000149 , -.000147 , -.000141 , .000138 , .000136 , .000136
            , .000127 , .000127 , -.000126 , -.000121 , -.000121 , .000117 , -.000116
            , -.000114 , -.000114 , -.000114 , .000114 , .000113 , .000109 , .000108
            , .000106 , -.000106 , -.000106 , .000105 , .000104 , -.000103 , -.000100
            , -.000100 , -.000100 , .000099 , -.000098 , .000093 , .000093 , .000090
            , -.000088 , .000083 , -.000083 , -.000082 , -.000081 , -.000079 , -.000077
            , -.000075 , -.000075 , -.000075 , .000071 , .000071 , -.000071 , .000068
            , .000068 , .000065 , .000065 , .000064 , .000064 , .000064 , -.000064
            , -.000060 , .000056 , .000056 , .000053 , .000053 , .000053 , -.000053
            , .000053 , .000053 , .000052 , .000050 , -.066607 , -.035184 , -.030988
            , .027929 , -.027616 , -.012753 , -.006728 , -.005837 , -.005286 , -.004921
            , -.002884 , -.002583 , -.002422 , .002310 , .002283 , -.002037 , .001883
            , -.001811 , -.001687 , -.001004 , -.000925 , -.000844 , .000766 , .000766
            , -.000700 , -.000495 , -.000492 , .000491 , .000483 , .000437 , -.000416
            , -.000384 , .000374 , -.000312 , -.000288 , -.000273 , .000259 , .000245
            , -.000232 , .000229 , -.000216 , .000206 , -.000204 , -.000202 , .000200
            , .000195 , -.000190 , .000187 , .000180 , -.000179 , .000170 , .000153
            , -.000137 , -.000119 , -.000119 , -.000112 , -.000110 , -.000110 , .000107
            , -.000095 , -.000095 , -.000091 , -.000090 , -.000081 , -.000079 , -.000079
            , .000077 , -.000073 , .000069 , -.000067 , -.000066 , .000065 , .000064
            , -.000062 , .000060 , .000059 , -.000056 , .000055 , -.000051;
        IDD
            << 2 , 0 , 0 , 0 , 0 , 0 , 2 , 2 , -2 , 0 , 0 , 0 , 2 , -1 , 0 , 1 , 0 , 0
            , 2 , 2 , 0 , 0 , 0 , 0 , 2 , 2 , 0 , 0 , 1 , 0 , 2 , 0 , 0 , 0 , -1 , 0
            , 2 , -1 , 2 , -1 , 0 , 0 , 2 , -2 , 2 , 0 , 0 , 0 , 2 , 1 , 0 , -1 , 0 , 0
            , 2 , 2 , -3 , 0 , 0 , 1 , 2 , -2 , 0 , 2 , 0 , 0 , 2 , -3 , 2 , 1 , 0 , 0
            , 2 , 1 , -2 , 1 , 0 , 0 , 2 , -1 , 0 , 1 , -1 , 0 , 2 , 3 , 0 , -1 , 0 , 0
            , 2 , 1 , 0 , 1 , 0 , 0 , 2 , 2 , 0 , 0 , 2 , 0 , 2 , 2 , -1 , 0 , 0 , -1
            , 2 , 0 , -1 , 0 , 0 , 1 , 2 , 1 , 0 , 1 , 1 , 0 , 2 , 3 , 0 , -1 , 1 , 0
            , 2 , 0 , 1 , 0 , 0 , -1 , 2 , 0 , -2 , 2 , 0 , 0 , 2 , -3 , 0 , 3 , 0 , 0
            , 2 , -2 , 3 , 0 , 0 , -1 , 2 , 4 , 0 , 0 , 0 , 0 , 2 , -1 , 1 , 1 , 0 , -1
            , 2 , -1 , 3 , -1 , 0 , -1 , 2 , 2 , 0 , 0 , -1 , 0 , 2 , -1 , -1 , 1 , 0 , 1
            , 2 , 4 , 0 , 0 , 1 , 0 , 2 , -3 , 4 , -1 , 0 , 0 , 2 , -1 , 2 , -1 , -1 , 0
            , 2 , 3 , -2 , 1 , 0 , 0 , 2 , 1 , 2 , -1 , 0 , 0 , 2 , -4 , 2 , 2 , 0 , 0
            , 2 , 4 , -2 , 0 , 0 , 0 , 2 , 0 , 2 , 0 , 0 , 0 , 2 , -2 , 2 , 0 , -1 , 0
            , 2 , 2 , -4 , 0 , 0 , 2 , 2 , 2 , -2 , 0 , -1 , 0 , 2 , 1 , 0 , -1 , -1 , 0
            , 2 , -1 , 1 , 0 , 0 , 0 , 2 , 2 , -1 , 0 , 0 , 1 , 2 , 2 , 1 , 0 , 0 , -1
            , 2 , -2 , 0 , 2 , -1 , 0 , 2 , -2 , 4 , -2 , 0 , 0 , 2 , 2 , 2 , 0 , 0 , 0
            , 2 , -4 , 4 , 0 , 0 , 0 , 2 , -1 , 0 , -1 , -2 , 0 , 2 , 1 , 2 , -1 , 1 , 0
            , 2 , -1 , -2 , 3 , 0 , 0 , 2 , 3 , -2 , 1 , 1 , 0 , 2 , 4 , 0 , -2 , 0 , 0
            , 2 , 0 , 0 , 2 , 0 , 0 , 2 , 0 , 2 , -2 , 0 , 0 , 2 , 0 , 2 , 0 , 1 , 0
            , 2 , -3 , 3 , 1 , 0 , -1 , 2 , 0 , 0 , 0 , -2 , 0 , 2 , 4 , 0 , 0 , 2 , 0
            , 2 , 4 , -2 , 0 , 1 , 0 , 2 , 0 , 0 , 0 , 0 , 2 , 2 , 1 , 0 , 1 , 2 , 0
            , 2 , 0 , -2 , 0 , -2 , 0 , 2 , -2 , 1 , 0 , 0 , 1 , 2 , -2 , 1 , 2 , 0 , -1
            , 2 , -1 , 1 , -1 , 0 , 1 , 2 , 5 , 0 , -1 , 0 , 0 , 2 , 1 , -3 , 1 , 0 , 1
            , 2 , -2 , -1 , 2 , 0 , 1 , 2 , 3 , 0 , -1 , 2 , 0 , 2 , 1 , -2 , 1 , -1 , 0
            , 2 , 5 , 0 , -1 , 1 , 0 , 2 , -4 , 0 , 4 , 0 , 0 , 2 , -3 , 2 , 1 , -1 , 0
            , 2 , -2 , 1 , 1 , 0 , 0 , 2 , 4 , 0 , -2 , 1 , 0 , 2 , 0 , 0 , 2 , 1 , 0
            , 2 , -5 , 4 , 1 , 0 , 0 , 2 , 0 , 2 , 0 , 2 , 0 , 2 , -1 , 2 , 1 , 0 , 0
            , 2 , 5 , -2 , -1 , 0 , 0 , 2 , 1 , -1 , 0 , 0 , 0 , 2 , 2 , -2 , 0 , 0 , 2
            , 2 , -5 , 2 , 3 , 0 , 0 , 2 , -1 , -2 , 1 , -2 , 0 , 2 , -3 , 5 , -1 , 0 , -1
            , 2 , -1 , 0 , 0 , 0 , 1 , 2 , -2 , 0 , 0 , -2 , 0 , 2 , 0 , -1 , 1 , 0 , 0
            , 2 , -3 , 1 , 1 , 0 , 1 , 2 , 3 , 0 , -1 , -1 , 0 , 2 , 1 , 0 , 1 , -1 , 0
            , 2 , -1 , 2 , 1 , 1 , 0 , 2 , 0 , -3 , 2 , 0 , 1 , 2 , 1 , -1 , -1 , 0 , 1
            , 2 , -3 , 0 , 3 , -1 , 0 , 2 , 0 , -2 , 2 , -1 , 0 , 2 , -4 , 3 , 2 , 0 , -1
            , 2 , -1 , 0 , 1 , -2 , 0 , 2 , 5 , 0 , -1 , 2 , 0 , 2 , -4 , 5 , 0 , 0 , -1
            , 2 , -2 , 4 , 0 , 0 , -2 , 2 , -1 , 0 , 1 , 0 , 2 , 2 , -2 , -2 , 4 , 0 , 0
            , 2 , 3 , -2 , -1 , -1 , 0 , 2 , -2 , 5 , -2 , 0 , -1 , 2 , 0 , -1 , 0 , -1 , 1
            , 2 , 5 , -2 , -1 , 1 , 0 , 1 , 1 , 0 , 0 , 0 , 0 , 1 , -1 , 0 , 0 , 0 , 0
            , 1 , 1 , -2 , 0 , 0 , 0 , 1 , -2 , 0 , 1 , 0 , 0 , 1 , 1 , 0 , 0 , 1 , 0
            , 1 , -1 , 0 , 0 , -1 , 0 , 1 , 2 , 0 , -1 , 0 , 0 , 1 , 0 , 0 , 1 , 0 , 0
            , 1 , 3 , 0 , 0 , 0 , 0 , 1 , -2 , 2 , -1 , 0 , 0 , 1 , -2 , 0 , 1 , -1 , 0
            , 1 , -3 , 2 , 0 , 0 , 0 , 1 , 0 , 0 , -1 , 0 , 0 , 1 , 1 , 0 , 0 , -1 , 0
            , 1 , 3 , 0 , 0 , 1 , 0 , 1 , 1 , -3 , 0 , 0 , 1 , 1 , -3 , 0 , 2 , 0 , 0
            , 1 , 1 , 2 , 0 , 0 , 0 , 1 , 0 , 0 , 1 , 1 , 0 , 1 , 2 , 0 , -1 , 1 , 0
            , 1 , 0 , 2 , -1 , 0 , 0 , 1 , 2 , -2 , 1 , 0 , 0 , 1 , 3 , -2 , 0 , 0 , 0
            , 1 , -1 , 2 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , -1 , 1 , 1 , -1 , 0 , 0 , 1
            , 1 , 4 , 0 , -1 , 0 , 0 , 1 , -4 , 2 , 1 , 0 , 0 , 1 , 0 , -2 , 1 , 0 , 0
            , 1 , -2 , 2 , -1 , -1 , 0 , 1 , 3 , 0 , -2 , 0 , 0 , 1 , -1 , 0 , 2 , 0 , 0
            , 1 , -1 , 0 , 0 , -2 , 0 , 1 , 3 , 0 , 0 , 2 , 0 , 1 , -3 , 2 , 0 , -1 , 0
            , 1 , 4 , 0 , -1 , 1 , 0 , 1 , 0 , 0 , -1 , -1 , 0 , 1 , 1 , -2 , 0 , -1 , 0
            , 1 , -3 , 0 , 2 , -1 , 0 , 1 , 1 , 0 , 0 , 2 , 0 , 1 , 1 , -1 , 0 , 0 , -1
            , 1 , -1 , -1 , 0 , 0 , 1 , 1 , 0 , 2 , -1 , 1 , 0 , 1 , -1 , 1 , 0 , 0 , -1
            , 1 , -1 , -2 , 2 , 0 , 0 , 1 , 2 , -2 , 1 , 1 , 0 , 1 , -4 , 0 , 3 , 0 , 0
            , 1 , -1 , 2 , 0 , 1 , 0 , 1 , 3 , -2 , 0 , 1 , 0 , 1 , 2 , 0 , -1 , -1 , 0
            , 1 , 0 , 0 , 1 , -1 , 0 , 1 , -2 , 2 , 1 , 0 , 0 , 1 , 4 , -2 , -1 , 0 , 0
            , 1 , -3 , 3 , 0 , 0 , -1 , 1 , -2 , 1 , 1 , 0 , -1 , 1 , -2 , 3 , -1 , 0 , -1
            , 1 , 0 , -2 , 1 , -1 , 0 , 1 , -2 , -1 , 1 , 0 , 1 , 1 , 4 , -2 , 1 , 0 , 0
            , 1 , -4 , 4 , -1 , 0 , 0 , 1 , -4 , 2 , 1 , -1 , 0 , 1 , 5 , -2 , 0 , 0 , 0
            , 1 , 3 , 0 , -2 , 1 , 0 , 1 , -5 , 2 , 2 , 0 , 0 , 1 , 2 , 0 , 1 , 0 , 0
            , 1 , 1 , 3 , 0 , 0 , -1 , 1 , -2 , 0 , 1 , -2 , 0 , 1 , 4 , 0 , -1 , 2 , 0
            , 1 , 1 , -4 , 0 , 0 , 2 , 1 , 5 , 0 , -2 , 0 , 0 , 1 , -1 , 0 , 2 , 1 , 0
            , 1 , -2 , 1 , 0 , 0 , 0 , 1 , 4 , -2 , 1 , 1 , 0 , 1 , -3 , 4 , -2 , 0 , 0
            , 1 , -1 , 3 , 0 , 0 , -1 , 1 , 3 , -3 , 0 , 0 , 1 , 1 , 5 , -2 , 0 , 1 , 0
            , 1 , 1 , 2 , 0 , 1 , 0 , 1 , 2 , 0 , 1 , 1 , 0 , 1 , -5 , 4 , 0 , 0 , 0
            , 1 , -2 , 0 , -1 , -2 , 0 , 1 , 5 , 0 , -2 , 1 , 0 , 1 , 1 , 2 , -2 , 0 , 0
            , 1 , 1 , -2 , 2 , 0 , 0 , 1 , -2 , 2 , 1 , 1 , 0 , 1 , 0 , 3 , -1 , 0 , -1
            , 1 , 2 , -3 , 1 , 0 , 1 , 1 , -2 , -2 , 3 , 0 , 0 , 1 , -1 , 2 , -2 , 0 , 0
            , 1 , -4 , 3 , 1 , 0 , -1 , 1 , -4 , 0 , 3 , -1 , 0 , 1 , -1 , -2 , 2 , -1 , 0
            , 1 , -2 , 0 , 3 , 0 , 0 , 1 , 4 , 0 , -3 , 0 , 0 , 1 , 0 , 1 , 1 , 0 , -1
            , 1 , 2 , -1 , -1 , 0 , 1 , 1 , 2 , -2 , 1 , -1 , 0 , 1 , 0 , 0 , -1 , -2 , 0
            , 1 , 2 , 0 , 1 , 2 , 0 , 1 , 2 , -2 , -1 , -1 , 0 , 1 , 0 , 0 , 1 , 2 , 0
            , 1 , 0 , 1 , 0 , 0 , 0 , 1 , 2 , -1 , 0 , 0 , 0 , 1 , 0 , 2 , -1 , -1 , 0
            , 1 , -1 , -2 , 0 , -2 , 0 , 1 , -3 , 1 , 0 , 0 , 1 , 1 , 3 , -2 , 0 , -1 , 0
            , 1 , -1 , -1 , 0 , -1 , 1 , 1 , 4 , -2 , -1 , 1 , 0 , 1 , 2 , 1 , -1 , 0 , -1
            , 1 , 0 , -1 , 1 , 0 , 1 , 1 , -2 , 4 , -1 , 0 , 0 , 1 , 4 , -4 , 1 , 0 , 0
            , 1 , -3 , 1 , 2 , 0 , -1 , 1 , -3 , 3 , 0 , -1 , -1 , 1 , 1 , 2 , 0 , 2 , 0
            , 1 , 1 , -2 , 0 , -2 , 0 , 1 , 3 , 0 , 0 , 3 , 0 , 1 , -1 , 2 , 0 , -1 , 0
            , 1 , -2 , 1 , -1 , 0 , 1 , 1 , 0 , -3 , 1 , 0 , 1 , 1 , -3 , -1 , 2 , 0 , 1
            , 1 , 2 , 0 , -1 , 2 , 0 , 1 , 6 , -2 , -1 , 0 , 0 , 1 , 2 , 2 , -1 , 0 , 0
            , 1 , -1 , 1 , 0 , -1 , -1 , 1 , -2 , 3 , -1 , -1 , -1 , 1 , -1 , 0 , 0 , 0 , 2
            , 1 , -5 , 0 , 4 , 0 , 0 , 1 , 1 , 0 , 0 , 0 , -2 , 1 , -2 , 1 , 1 , -1 , -1
            , 1 , 1 , -1 , 0 , 1 , 1 , 1 , 1 , 2 , 0 , 0 , -2 , 1 , -3 , 1 , 1 , 0 , 0
            , 1 , -4 , 4 , -1 , -1 , 0 , 1 , 1 , 0 , -2 , -1 , 0 , 1 , -2 , -1 , 1 , -1 , 1
            , 1 , -3 , 2 , 2 , 0 , 0 , 1 , 5 , -2 , -2 , 0 , 0 , 1 , 3 , -4 , 2 , 0 , 0
            , 1 , 1 , -2 , 0 , 0 , 2 , 1 , -1 , 4 , -2 , 0 , 0 , 1 , 2 , 2 , -1 , 1 , 0
            , 1 , -5 , 2 , 2 , -1 , 0 , 1 , 1 , -3 , 0 , -1 , 1 , 1 , 1 , 1 , 0 , 1 , -1
            , 1 , 6 , -2 , -1 , 1 , 0 , 1 , -2 , 2 , -1 , -2 , 0 , 1 , 4 , -2 , 1 , 2 , 0
            , 1 , -6 , 4 , 1 , 0 , 0 , 1 , 5 , -4 , 0 , 0 , 0 , 1 , -3 , 4 , 0 , 0 , 0
            , 1 , 1 , 2 , -2 , 1 , 0 , 1 , -2 , 1 , 0 , -1 , 0 , 0 , 2 , 0 , 0 , 0 , 0
            , 0 , 1 , 0 , -1 , 0 , 0 , 0 , 0 , 2 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0
            , 0 , 2 , 0 , 0 , 1 , 0 , 0 , 3 , 0 , -1 , 0 , 0 , 0 , 1 , -2 , 1 , 0 , 0
            , 0 , 2 , -2 , 0 , 0 , 0 , 0 , 3 , 0 , -1 , 1 , 0 , 0 , 0 , 1 , 0 , 0 , -1
            , 0 , 2 , 0 , -2 , 0 , 0 , 0 , 2 , 0 , 0 , 2 , 0 , 0 , 3 , -2 , 1 , 0 , 0
            , 0 , 1 , 0 , -1 , -1 , 0 , 0 , 1 , 0 , -1 , 1 , 0 , 0 , 4 , -2 , 0 , 0 , 0
            , 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , 3 , 0 , 0 , -1 , 0 , 4 , 0 , -2 , 0 , 0
            , 0 , 3 , -2 , 1 , 1 , 0 , 0 , 3 , -2 , -1 , 0 , 0 , 0 , 4 , -2 , 0 , 1 , 0
            , 0 , 0 , 2 , 0 , 1 , 0 , 0 , 1 , 0 , 1 , 1 , 0 , 0 , 4 , 0 , -2 , 1 , 0
            , 0 , 3 , 0 , -1 , 2 , 0 , 0 , 5 , -2 , -1 , 0 , 0 , 0 , 1 , 2 , -1 , 0 , 0
            , 0 , 1 , -2 , 1 , -1 , 0 , 0 , 1 , -2 , 1 , 1 , 0 , 0 , 2 , -2 , 0 , -1 , 0
            , 0 , 2 , -3 , 0 , 0 , 1 , 0 , 2 , -2 , 0 , 1 , 0 , 0 , 0 , 2 , -2 , 0 , 0
            , 0 , 1 , -3 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , 2 , 0 , 0 , 0 , 1 , 0 , 0 , 1
            , 0 , 1 , 2 , -1 , 1 , 0 , 0 , 3 , 0 , -3 , 0 , 0 , 0 , 2 , 1 , 0 , 0 , -1
            , 0 , 1 , -1 , -1 , 0 , 1 , 0 , 1 , 0 , 1 , 2 , 0 , 0 , 5 , -2 , -1 , 1 , 0
            , 0 , 2 , -1 , 0 , 0 , 1 , 0 , 2 , 2 , -2 , 0 , 0 , 0 , 1 , -1 , 0 , 0 , 0
            , 0 , 5 , 0 , -3 , 0 , 0 , 0 , 2 , 0 , -2 , 1 , 0 , 0 , 1 , 1 , -1 , 0 , -1
            , 0 , 3 , -4 , 1 , 0 , 0 , 0 , 0 , 2 , 0 , 2 , 0 , 0 , 2 , 0 , -2 , -1 , 0
            , 0 , 4 , -3 , 0 , 0 , 1 , 0 , 3 , -1 , -1 , 0 , 1 , 0 , 0 , 2 , 0 , 0 , -2
            , 0 , 3 , -3 , 1 , 0 , 1 , 0 , 2 , -4 , 2 , 0 , 0 , 0 , 4 , -2 , -2 , 0 , 0
            , 0 , 3 , 1 , -1 , 0 , -1 , 0 , 5 , -4 , 1 , 0 , 0 , 0 , 3 , -2 , -1 , -1 , 0
            , 0 , 3 , -2 , 1 , 2 , 0 , 0 , 4 , -4 , 0 , 0 , 0 , 0 , 6 , -2 , -2 , 0 , 0
            , 0 , 5 , 0 , -3 , 1 , 0 , 0 , 4 , -2 , 0 , 2 , 0 , 0 , 2 , 2 , -2 , 1 , 0
            , 0 , 0 , 4 , 0 , 0 , -2 , 0 , 3 , -1 , 0 , 0 , 0 , 0 , 3 , -3 , -1 , 0 , 1
            , 0 , 4 , 0 , -2 , 2 , 0 , 0 , 1 , -2 , -1 , -1 , 0 , 0 , 2 , -1 , 0 , 0 , -1
            , 0 , 4 , -4 , 2 , 0 , 0 , 0 , 2 , 1 , 0 , 1 , -1 , 0 , 3 , -2 , -1 , 1 , 0
            , 0 , 4 , -3 , 0 , 1 , 1 , 0 , 2 , 0 , 0 , 3 , 0 , 0 , 6 , -4 , 0 , 0 , 0;
        base_pair FP;
        Matrix F(1, IDD.rows()), P(1, IDD.rows());
        for (int i = 0; i < IDD.rows(); i++)
        {
            FP = libiers_tdfrph(IDD.row(i), mjd, leap);
            F(0, i) = FP[0];
            P(0, i) = FP[1];
            //std::cout << P(1, i) << std::endl;
        }
        Matrix ret(4, 342);
        ret.row(0) = F;
        ret.row(1) = P;
        ret.row(2) = TAMP;
        ret.row(3) = IDD.col(0).transpose();
        return ret;
    }

    //Triple gnss_model_tide2010::load_ocean_vlbi(const base_time& epoch, const string& site, const Triple& xRec)
    //{
    //    _mutex.lock();
    //    Matrix otl;
    //    Triple dxyz;
    //
    //    Matrix IDT(11, 6);
    //    IDT << 2 << 0 << 0 << 0 << 0 << 0
    //        << 2 << 2 << -2 << 0 << 0 << 0
    //        << 2 << -1 << 0 << 1 << 0 << 0
    //        << 2 << 2 << 0 << 0 << 0 << 0
    //        << 1 << 1 << 0 << 0 << 0 << 0
    //        << 1 << -1 << 0 << 0 << 0 << 0
    //        << 1 << 1 << -2 << 0 << 0 << 0
    //        << 1 << -2 << 0 << 1 << 0 << 0
    //        << 0 << 2 << 0 << 0 << 0 << 0
    //        << 0 << 1 << 0 << -1 << 0 << 0
    //        << 0 << 0 << 2 << 0 << 0 << 0;
    //    double TAMPT[11] = { 0.6322, 0.2941, 0.1210, 0.0799, 0.3686, -0.2622, -0.1220, -0.0502, -0.0666, -0.0352, -0.0310 };
    //
    //    if (!_gotl || _gotl->data(otl, site) < 0) {
    //        if (_log) _log->comment(0, "gtide2010", "WARNING: Site " + site + " not found in ocean tide BLQ file!");
    //        _mutex.unlock(); return dxyz;
    //    }
    //    Matrix FPTAMPIDD = libiers_tdfrph_call(epoch.dmjd(), epoch.leapsec());
    //    Matrix TAMP = otl.submatrix(1, 3, 1, 11);
    //    Matrix TPH = -1.0 * otl.submatrix(4, 6, 1, 11);
    //    Matrix RFRLAIM = libiers_admint_part1(TAMP, TPH, TAMPT, IDT, epoch.dmjd(), epoch.leapsec());
    //    pair<Matrix, Matrix> AZPZ, AWPW, ASPS;
    //    AZPZ = libiers_admint(RFRLAIM.row(1), RFRLAIM.row(2), RFRLAIM.row(5), FPTAMPIDD.row(1), FPTAMPIDD.row(2), FPTAMPIDD.row(3), FPTAMPIDD.row(4).transpose());
    //    AWPW = libiers_admint(RFRLAIM.row(1), RFRLAIM.row(3), RFRLAIM.row(6), FPTAMPIDD.row(1), FPTAMPIDD.row(2), FPTAMPIDD.row(3), FPTAMPIDD.row(4).transpose());
    //    ASPS = libiers_admint(RFRLAIM.row(1), RFRLAIM.row(4), RFRLAIM.row(7), FPTAMPIDD.row(1), FPTAMPIDD.row(2), FPTAMPIDD.row(3), FPTAMPIDD.row(4).transpose());
    //
    //    Matrix PZ, PS, PW;
    //    PZ = AZPZ.second * D2R;
    //    PS = ASPS.second * D2R;
    //    PW = AWPW.second * D2R;
    //    Triple cto_rsw(0.0, 0.0, 0.0);
    //    for (int i = 1; i <= AZPZ.first.ncols(); i++)
    //    {
    //        cto_rsw[0] = cto_rsw[0] + AZPZ.first(1, i) * cos(PZ(1, i));
    //    }
    //    for (int i = 1; i <= ASPS.first.ncols(); i++)
    //    {
    //        cto_rsw[1] = cto_rsw[1] + ASPS.first(1, i) * cos(PS(1, i));
    //    }
    //    for (int i = 1; i <= AWPW.first.ncols(); i++)
    //    {
    //        cto_rsw[2] = cto_rsw[2] + AWPW.first(1, i) * cos(PW(1, i));
    //    }
    //    Triple neu(-cto_rsw[1], -cto_rsw[2], cto_rsw[0]);
    //    Triple ell;
    //    Triple tmp = xRec;
    //    xyz2ell_vlbi(tmp, ell);
    //    neu2xyz(ell, neu, dxyz);
    //
    //    _mutex.unlock(); return dxyz;
    //}

    // ocean tide loading
    // ----------
    Triple gnss_model_tide2010::load_ocean(const base_time &epoch, const string &site, const Triple &xRec)
    {
        _mutex.lock();

        Matrix otl, Acjcos, AcjSin;
        Matrix DD, A(2, 3), P(2, 3); // ff stands for fundamental frequencies
        Matrix it;
        Matrix dnn, dn;
        Matrix votl;
        Matrix itt(342, 6);
        Vector ff, fjc(342), fjs(342);
        fjc.setZero();
        fjs.setZero();
        double f, fp1, fp2, K, Kp1, Kp2, pp;
        int i1, i2, i1b, i2b;
        double delta = 32.184 + epoch.leapsec();

        //                 NOTE !!!
        // pivot indices for each wave in dnn and dnnBLQ - if you modify "dnn" or dnnBLQ (e.g., flipping rows), you HAVE TO change iind accordingly!!!
        // 1. and 2. col determines position of both pivots (if any) in dnn(342,7), 3. and 4. determines position of the pivots (if any) in dnnBLQ (11,6)
        // if iind(i,1)=0 and iind(i,2)=0 and iind(i,1)=iind(i,2) ... pivot from BLQ
        // if iind(i,1)=0 or iind(i,2)=0                             ... one-side interpolation of the minor wave
        // if iind(i,1)~=0 or iind(i,2)~=0                           ... two-side interpolation of the minor wave

        int iind[342][4] = {
            {0, 0, 1, 1}, {0, 0, 2, 2}, {0, 0, 3, 3}, {0, 0, 4, 4}, {4, 0, 4, 0}, {3, 1, 3, 1}, {3, 1, 3, 1}, {3, 0, 3, 0}, {1, 2, 1, 2}, {1, 2, 1, 2}, {3, 0, 3, 0}, {3, 0, 3, 0}, {1, 2, 1, 2}, {3, 0, 3, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {4, 0, 4, 0}, {2, 4, 2, 4}, {3, 1, 3, 1}, {1, 2, 1, 2}, {4, 0, 4, 0}, {1, 2, 1, 2}, {3, 1, 3, 1}, {3, 0, 3, 0}, {3, 0, 3, 0}, {4, 0, 4, 0}, {3, 1, 3, 1}, {3, 1, 3, 1}, {2, 4, 2, 4}, {3, 0, 3, 0}, {4, 0, 4, 0}, {3, 0, 3, 0}, {3, 1, 3, 1}, {4, 0, 4, 0}, {1, 2, 1, 2}, {3, 0, 3, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {3, 0, 3, 0}, {1, 2, 1, 2}, {1, 2, 1, 2}, {1, 2, 1, 2}, {3, 1, 3, 1}, {2, 4, 2, 4}, {4, 0, 4, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {4, 0, 4, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {1, 2, 1, 2}, {3, 0, 3, 0}, {4, 0, 4, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {1, 2, 1, 2}, {1, 2, 1, 2}, {3, 0, 3, 0}, {3, 1, 3, 1}, {4, 0, 4, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {1, 2, 1, 2}, {3, 1, 3, 1}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 1, 3, 1}, {4, 0, 4, 0}, {1, 2, 1, 2}, {3, 0, 3, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {4, 0, 4, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {3, 0, 3, 0}, {1, 2, 1, 2}, {3, 1, 3, 1}, {4, 0, 4, 0}, {1, 2, 1, 2}, {2, 4, 2, 4}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 1, 3, 1}, {3, 0, 3, 0}, {4, 0, 4, 0}, {1, 2, 1, 2}, {3, 1, 3, 1}, {3, 1, 3, 1}, {1, 2, 1, 2}, {3, 0, 3, 0}, {3, 1, 3, 1}, {3, 0, 3, 0}, {3, 0, 3, 0}, {4, 0, 4, 0}, {3, 0, 3, 0}, {3, 0, 3, 0}, {3, 1, 3, 1}, {3, 0, 3, 0}, {4, 0, 4, 0}, {3, 0, 3, 0}, {3, 1, 3, 1}, {4, 0, 4, 0}, {0, 0, 5, 5}, {0, 0, 6, 6}, {0, 0, 7, 7}, {0, 0, 8, 8}, {110, 0, 5, 0}, {113, 111, 8, 6}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {113, 111, 8, 6}, {113, 0, 8, 0}, {113, 0, 8, 0}, {111, 112, 6, 7}, {112, 110, 7, 5}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 0, 8, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {112, 110, 7, 5}, {110, 0, 5, 0}, {113, 0, 8, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {110, 0, 5, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {111, 112, 6, 7}, {113, 0, 8, 0}, {110, 0, 5, 0}, {112, 110, 7, 5}, {113, 111, 8, 6}, {111, 112, 6, 7}, {111, 112, 6, 7}, {113, 111, 8, 6}, {110, 0, 5, 0}, {113, 0, 8, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {110, 0, 5, 0}, {113, 0, 8, 0}, {113, 111, 8, 6}, {113, 111, 8, 6}, {111, 112, 6, 7}, {113, 0, 8, 0}, {110, 0, 5, 0}, {113, 0, 8, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {110, 0, 5, 0}, {113, 0, 8, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {113, 0, 8, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {112, 110, 7, 5}, {113, 111, 8, 6}, {111, 112, 6, 7}, {110, 0, 5, 0}, {113, 0, 8, 0}, {111, 112, 6, 7}, {113, 0, 8, 0}, {113, 0, 8, 0}, {113, 111, 8, 6}, {113, 111, 8, 6}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {111, 112, 6, 7}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {113, 0, 8, 0}, {110, 0, 5, 0}, {113, 111, 8, 6}, {110, 0, 5, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {110, 0, 5, 0}, {113, 0, 8, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {111, 112, 6, 7}, {113, 0, 8, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {111, 112, 6, 7}, {113, 111, 8, 6}, {111, 112, 6, 7}, {113, 0, 8, 0}, {112, 110, 7, 5}, {113, 111, 8, 6}, {112, 110, 7, 5}, {110, 0, 5, 0}, {113, 0, 8, 0}, {113, 0, 8, 0}, {112, 110, 7, 5}, {113, 0, 8, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {110, 0, 5, 0}, {112, 110, 7, 5}, {111, 112, 6, 7}, {110, 0, 5, 0}, {113, 0, 8, 0}, {111, 112, 6, 7}, {110, 0, 5, 0}, {110, 0, 5, 0}, {113, 111, 8, 6}, {110, 0, 5, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {113, 0, 8, 0}, {110, 0, 5, 0}, {113, 111, 8, 6}, {0, 0, 9, 9}, {0, 0, 10, 10}, {0, 0, 11, 11}, {266, 0, 11, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 265, 11, 10}, {265, 264, 10, 9}, {264, 0, 9, 0}, {266, 0, 11, 0}, {265, 264, 10, 9}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 265, 11, 10}, {265, 264, 10, 9}, {264, 0, 9, 0}, {265, 264, 10, 9}, {266, 265, 11, 10}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 265, 11, 10}, {265, 264, 10, 9}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {265, 264, 10, 9}, {266, 265, 11, 10}, {266, 265, 11, 10}, {265, 264, 10, 9}, {265, 264, 10, 9}, {265, 264, 10, 9}, {266, 0, 11, 0}, {266, 265, 11, 10}, {266, 0, 11, 0}, {266, 0, 11, 0}, {265, 264, 10, 9}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 265, 11, 10}, {265, 264, 10, 9}, {264, 0, 9, 0}, {265, 264, 10, 9}, {264, 0, 9, 0}, {266, 265, 11, 10}, {264, 0, 9, 0}, {265, 264, 10, 9}, {265, 264, 10, 9}, {264, 0, 9, 0}, {266, 265, 11, 10}, {265, 264, 10, 9}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 0, 11, 0}, {264, 0, 9, 0}, {265, 264, 10, 9}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 265, 11, 10}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {266, 265, 11, 10}, {265, 264, 10, 9}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}, {264, 0, 9, 0}};

        Matrix dnnBLQ(11, 6);
        dnnBLQ << 2 , 0 , 0 , 0 , 0 , 0
               , 2 , 2 , -2 , 0 , 0 , 0
               , 2 , -1 , 0 , 1 , 0 , 0
               , 2 , 2 , 0 , 0 , 0 , 0
               , 1 , 1 , 0 , 0 , 0 , 0
               , 1 , -1 , 0 , 0 , 0 , 0
               , 1 , 1 , -2 , 0 , 0 , 0
               , 1 , -2 , 0 , 1 , 0 , 0
               , 0 , 2 , 0 , 0 , 0 , 0
               , 0 , 1 , 0 , -1 , 0 , 0
               , 0 , 0 , 2 , 0 , 0 , 0;

        // load dMJD
        double mjd_i = epoch.dmjd(); // [day]

        // input for the atronomical argument for each epoch
        DD = DD_arguments(mjd_i, delta);

        // load all waves for the interpolation
        dnn = dnn_table();

        // load BLQ for a particular site
        Triple dxyz(0.0, 0.0, 0.0);
        if (!_gotl || _gotl->data(otl, site) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "WARNING: Site " + site + " not found in ocean tide BLQ file!");
            _mutex.unlock();
            return dxyz;
        }

        otl.block(3, 0, 3, 11) = otl.block(3, 0, 3, 11) * D2R;

        // ff is the same for all waves and all epochs
        ff = DD.block(6, 0, 6, 1);

        Kp2 = 0.0;
        Kp1 = 0.0;
        pp = 0.0;

        for (int i = 0; i <= 341; i++)
        {

            dn = dnn.block(i, 0, 1, 6);
            K = dnn(i, 6);
            f = Vector(dn).dot(ff);
            i1 = iind[i][0]; // indeces wrt dnn, +1 because dnn and otl is Newmat Matrix
            i2 = iind[i][1];
            i1b = iind[i][2]; // indeces wrt BLQ
            i2b = iind[i][3];
            A.setZero();
            P.setZero();

            // BLQ - pivots not needed at all
            if (i1 == 0 && i2 == 0)
            {
                fp1 = 0.0; // frequency of the first pivot
                fp2 = 0.0; // frequency of the seoond pivot
                Kp1 = K;
                Kp2 = 1.0; // because of denominator
                pp = 0.0;

                A(0, 0) = otl(0, i1b - 1);
                A(0, 1) = otl(1, i1b - 1);
                A(0, 2) = otl(2, i1b - 1);

                P(0, 0) = otl(3, i1b - 1);
                P(0, 1) = otl(4, i1b - 1);
                P(0, 2) = otl(5, i1b - 1); //% up,ew,ns
            }

            // One-side interpolation ... pivot 2 non-existing
            if (i1 != 0 && i2 == 0)
            {
                Vector tmp = dnn.block(i1 - 1, 0, 1, 6);
                fp1 = tmp.dot(ff);
                fp2 = 0.0; // frequency of the second pivot
                Kp1 = dnn(i1 - 1, 6);
                Kp2 = 1.0; // because of denominator
                pp = 0.0;

                A(0, 0) = otl(0, i1b - 1);
                A(0, 1) = otl(1, i1b - 1);
                A(0, 2) = otl(2, i1b - 1);

                P(0, 0) = otl(3, i1b - 1);
                P(0, 1) = otl(4, i1b - 1);
                P(0, 2) = otl(5, i1b - 1); //% up,ew,ns
            }

            // One-side interpolation ... pivot 1 non-existing
            if (i1 == 0 && i2 != 0)
            {
                fp2 = 0.0; // frequency of the first pivot
                Vector tmp = dnn.block(i2 - 1, 0, 1, 6);

                fp1 = tmp.dot(ff);
                Kp2 = 1.0; // because of denominator
                Kp1 = dnn(i2 - 1, 6);
                pp = 0.0;

                A(0, 0) = otl(0, i1b - 1);
                A(0, 1) = otl(1, i1b - 1);
                A(0, 2) = otl(2, i1b - 1);

                P(0, 0) = otl(3, i1b - 1);
                P(0, 1) = otl(4, i1b - 1);
                P(0, 2) = otl(5, i1b - 1); //% up,ew,ns
            }

            // Two-side interpolation
            if (i1 != 0 && i2 != 0)
            {
                Vector tmp = dnn.block(i1 - 1, 0, 1, 6);
                fp1 = tmp.dot(ff); // frequency of the first pivot
                tmp = dnn.block(i2 - 1, 0, 1, 6);
                fp2 = tmp.dot(ff); // frequency of the second pivot

                // flip all if fp1 > fp2
                if (fp1 > fp2)
                {
                    i1 = iind[i][1]; // indeces wrt dnn, +1 because dnn and otl is Newmat Matrix
                    i2 = iind[i][0];
                    i1b = iind[i][3]; // indeces wrt BLQ
                    i2b = iind[i][2];
                    tmp = dnn.block(i2 - 1, 0, 1, 6);
                    fp1 = tmp.dot(ff); // frequency of the first pivot
                    tmp = dnn.block(i1- 1, 0, 1, 6);
                    fp2 = tmp.dot(ff); // frequency of the second pivot
                }
                Kp1 = dnn(i1 - 1, 6); // amplitude of the first pivot
                Kp2 = dnn(i2 - 1, 6); // amplitude of the second pivot
                pp = (f - fp1) / (fp2 - fp1);

                A(0, 0) = otl(0, i1b - 1);
                P(0, 0) = otl(3, i1b - 1);
                A(0, 1) = otl(1, i1b - 1);
                P(0, 1) = otl(4, i1b - 1);
                A(0, 2) = otl(2, i1b - 1);
                P(0, 2) = otl(5, i1b - 1); //% up,ew,ns

                A(1, 0) = otl(0, i2b - 1);
                P(1, 0) = otl(3, i2b - 1);
                A(1, 1) = otl(1, i2b - 1);
                P(1, 1) = otl(4, i2b - 1);
                A(1, 2) = otl(2, i2b - 1);
                P(1, 2) = otl(5, i2b - 1); //% up,ew,ns
            }

            // ==== Conditions for Phase and Amplitude

            //if (K>0.0 && dn(1,1)==0){ // ADMINTF degree 0 ... likely not needed
            // P = P + hwa_pi;       }

            if (K > 0.0 && abs(dn(0, 0) - 1.0) < 0.001)
            { //ADMINTF degree 1
                P = P + Matrix::Constant(P.rows(), P.cols(), hwa_pi / 2.0);
                A = -1.0 * A;
            }

            if (K < 0.0 && abs(dn(0, 0) - 1.0) < 0.001)
            { //ADMINTF degree 1
                P = P - Matrix::Constant(P.rows(), P.cols(), hwa_pi / 2.0);
                A = -1.0 * A;
            }

            // table itt - interpolation of all tides (for major tides - pivots K/Kp1=1 and pp=0)
            for (unsigned ii = 0; ii < 3; ii++)
            {
                itt(i, ii) = A(0, ii) * cos(P(0, ii)) * K / Kp1 * (1.0 - pp) + A(1, ii) * cos(P(1, ii)) * K / Kp2 * pp;
                itt(i, ii + 3) = A(0, ii) * sin(P(0, ii)) * K / Kp1 * (1.0 - pp) + A(1, ii) * sin(P(1, ii)) * K / Kp2 * pp;
            }

        } // ends loop over tide waves

        for (unsigned j = 0; j < 342; j++)
        {
            Vector tmp1 = dnn.block(j, 0, 1, 6);
            Vector tmp2 = DD.block(0, 0, 6, 1);
            fjc(j, 0) = cos(null2pi_rad(tmp1.dot(tmp2)));
            fjs(j, 0) = sin(null2pi_rad(tmp1.dot(tmp2)));
        }

        double up, ns, ew;
        up = fjc.dot(itt.col(0)) + fjs.dot(itt.col(3));
        /*ns = dot(fjc,itt.Column(2)) + dot(fjs,itt.Column(5));
     ew = dot(fjc,itt.Column(3)) + dot(fjs,itt.Column(6));*/
        // corrected according to the VLBI MATLAB code ?
        ew = fjc.dot(itt.col(1)) + fjs.dot(itt.col(4));
        ns = fjc.dot(itt.col(2)) + fjs.dot(itt.col(5));

        //    std::cout << setw(12) <<up <<" "<< ns << " " << ew << std::endl;

        Triple dneu;
        dneu[0] = -ns;
        dneu[1] = -ew;
        dneu[2] = up;

        Triple ell;
        xyz2ell_vlbi(xRec, ell);
        neu2xyz(ell, dneu, dxyz);

        _mutex.unlock();
        return dxyz;
    }

    // atmospheric tide loading
    // ----------
    //Triple gnss_model_tide2010::load_atmosph(const base_time& epoch, const string& site, const Triple& xRec)
    //{
    //   _mutex.lock();
    //
    //  Matrix atl;
    //
    //  double mjd_i = epoch.dmjd(); // [day]
    //  double utj = (mjd_i - int(mjd_i)) * 2 * 3.141592653589793;
    //  double dr, dn, de;
    //  Triple dxyz(0.0, 0.0, 0.0), dneu;
    //  if (!_gatl || _gatl->data(atl, site) < 0) {
    //      if (_log) _log->comment(0, "gtide2010", "WARNING: Site " + site + " not found in tidal atmosphere loading file!");
    //      _mutex.unlock(); return dxyz;
    //  }
    //
    //  dr = atl(1, 1) * cos(utj) + atl(1, 2) * sin(utj) + atl(1, 3) * cos(utj * 2) + atl(1, 4) * sin(utj * 2);
    //  dn = atl(1, 5) * cos(utj) + atl(1, 6) * sin(utj) + atl(1, 7) * cos(utj * 2) + atl(1, 8) * sin(utj * 2);
    //  de = atl(1, 9) * cos(utj) + atl(1, 10) * sin(utj) + atl(1, 11) * cos(utj * 2) + atl(1, 12) * sin(utj * 2);
    //  dneu[0] =  dn / 1000);
    //  dneu[1] =  de / 1000);
    //  dneu[2] =  dr / 1000);
    //  Triple ell;
    //  ell[0] = cart2phigd(xRec);
    //  ell[1] = atan2(xRec[1], xRec[0]);
    //  ell[2] = 0.0;
    //  neu2xyz(ell, dneu, dxyz);
    //
    //  _mutex.unlock(); return dxyz;
    //}

    // atmospheric non tide loading
    // ----------
    //Triple gnss_model_tide2010::load_notidatmosph(const base_time& epoch, const string& site, const Triple& xRec)
    //{
    //   _mutex.lock();
    //
    //    double mjd_i = epoch.dmjd(); // [day]
    //    double indtim = int(mjd_i * 100 / 25) * 0.25 - 0.75;
    //    base_time indepo(base_time::UTC);
    //    vector<double> cnta1, cnta2, cnta3, cnta4;
    //    Triple dxyz(0.0, 0.0, 0.0), apl;
    //    Triple ell, xyz, neu;
    //    ell[0] = cart2phigd(xRec);
    //    ell[1] = atan2(xRec[1], xRec[0]);
    //    ell[2] = 0.0;
    //    for (int i = 1; i < 9; i++)
    //    {
    //        indepo.from_dmjd(indtim);
    //        if (!_gantl || _gantl->data(apl, site, indepo) < 0) {
    //            if (_log) _log->comment(0, "gtide2010", "WARNING: Site " + site + " not found in atmosphere non tidal loading file!");
    //            _mutex.unlock(); return dxyz;
    //        }
    //        neu[0] = apl[2];
    //        neu[1] = apl[1];
    //        neu[2] = apl[0];
    //        neu2xyz(ell, neu, xyz);
    //        cnta1.push_back(indtim);  // cnta(i, 1) = indtim;
    //        cnta2.push_back(xyz[0]);  // cnta(i, 2) = apl[0];
    //        cnta3.push_back(xyz[1]);  // cnta(i, 3) = apl[1];
    //        cnta4.push_back(xyz[2]);  // cnta(i, 4) = apl[2];
    //        indtim = indtim + 0.25;
    //    }
    //
    //    // SPLINE Cubic spline data interpolation
    //    dxyz[0] = splineinterp(cnta1, cnta2, mjd_i);
    //    dxyz[1] = splineinterp(cnta1, cnta3, mjd_i);
    //    dxyz[2] = splineinterp(cnta1, cnta4, mjd_i);
    //    _mutex.unlock(); return dxyz;
    //}

    Triple gnss_model_tide2010::load_atmosregrecoef(const Triple &xRec, double press)
    {
        //
        double press0, temp, undu;
        Triple ell, dxyz;
        xyz2ell_vlbi(xRec, ell);
        // VIEVS -- the MATLAB code is read a gpt3_5.grd file to load the gpt ???
        _ggpt.gpt_v1(51544.0, ell[0], ell[1], ell[2], press0, temp, undu);
        double pcrg_h = press - press0;
        Triple tmp;
        tmp[0] = pcrg_h;
        tmp[1] = 0.0;
        tmp[2] = 0.0;
        neu2xyz(ell, tmp, dxyz);
        return dxyz;
    }

    // PROTECTED
    //======================
} // namespace
