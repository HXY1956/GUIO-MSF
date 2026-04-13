#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "hwa_gnss_data_interp.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{
    // constructor
    // ----------
    gnss_data_interp::gnss_data_interp()
        : _interp_1d(LINEAR),
          _interp_2d(BILINEAR),
          _interp_3d(VER2HOR),
          _interp_ht(INTERPOLATE)
    {
    }

    gnss_data_interp::gnss_data_interp(base_log spdlog)
        : base_data(spdlog),
          _interp_1d(LINEAR),
          _interp_2d(BILINEAR),
          _interp_3d(VER2HOR),
          _interp_ht(INTERPOLATE)
    {
    }
    // destructor
    // ----------
    gnss_data_interp::~gnss_data_interp() {}

    // ----------
    gnss_data_interp::INTERP_1D gnss_data_interp::str_to_interp_1d(const string &str)
    {
        if (str.compare("LINEAR") == 0)
            return LINEAR;
        else if (str.compare("SPLINE") == 0)
            return SPLINE;
        else
            cerr << "# warning - unknown 1D interpolation method: " << str << endl;

        return LINEAR;
    }

    // ----------
    gnss_data_interp::INTERP_2D gnss_data_interp::str_to_interp_2d(const string &str)
    {
        if (str.compare("BILINEAR") == 0)
            return BILINEAR;
        else if (str.compare("IDW") == 0)
            return IDW;
        else if (str.compare("TPS") == 0)
            return TPS;
        else
            cerr << "# warning - unknown 2D interpolation method: " << str << endl;

        return BILINEAR;
    }

    // ----------
    gnss_data_interp::INTERP_3D gnss_data_interp::str_to_interp_3d(const string &str)
    {
        if (str.compare("HOR2VER") == 0)
            return HOR2VER;
        else if (str.compare("VER2HOR") == 0)
            return VER2HOR;
        else
            cerr << "# warning - unknown 3D interpolation method: " << str << endl;

        return VER2HOR;
    }

    // ----------
    gnss_data_interp::INTERP_HT gnss_data_interp::str_to_interp_ht(const string &str)
    {
        if (str.compare("INTERPOLATE") == 0)
            return INTERPOLATE;
        else if (str.compare("SCALE") == 0)
            return SCALE;
        else
            cerr << "# warning - unknown HT interpolation method: " << str << endl;

        return INTERPOLATE;
    }

    // ----------
    string gnss_data_interp::interp_1d_to_str(const INTERP_1D &typ)
    {
        switch (typ)
        {
        case LINEAR:
            return "LINEAR";
            break;
        case SPLINE:
            return "SPLINE";
            break;
        default:
            cerr << "# warning - unknown 1D interpolation method\n";
        }
        return "LINEAR";
    }

    // ----------
    string gnss_data_interp::interp_2d_to_str(const INTERP_2D &typ)
    {
        switch (typ)
        {
        case BILINEAR:
            return "BILINEAR";
            break;
        case IDW:
            return "IDW";
            break;
        case TPS:
            return "TPS";
            break;
        default:
            cerr << "# warning - unknown 2D interpolation method\n";
        }
        return "BILINEAR";
    }

    // ----------
    string gnss_data_interp::interp_3d_to_str(const INTERP_3D &typ)
    {
        switch (typ)
        {
        case HOR2VER:
            return "HOR2VER";
            break;
        case VER2HOR:
            return "VER2HOR";
            break;
        default:
            cerr << "# warning - unknown 3D interpolation method\n";
        }
        return "VER2HOR";
    }

    // ----------
    string gnss_data_interp::interp_ht_to_str(const INTERP_HT &typ)
    {
        switch (typ)
        {
        case INTERPOLATE:
            return "INTERPOLATE";
            break;
        case SCALE:
            return "SCALE";
            break;
        default:
            cerr << "# warning - unknown HT interpolation method\n";
        }
        return "INTERPOLATE";
    }

    // ----------------------------
    // Linear Interpolation (double)
    // ----------------------------
    int gnss_data_interp::linear(map<double, double> &data, double val, double &fval)
    {
        map<double, double>::iterator it2, it1;
        it2 = data.lower_bound(val);

        if (it2 != data.end() && double_eq(it2->first, val))
        {
            fval = it2->second;
            return 0;
        }
        if (it2 == data.begin() || it2 == data.end())
        {
#ifdef DEBUG
            cerr << "# warning: extrapolation is not allowed " << it2->first << " " << val << "!\n";
#endif
            return -1;
        }

        it1 = it2;
        it1--;

        Matrix x(2, 2), y(2, 1);
        x << it1->first, 1
          ,it2->first, 1;
        y << it1->second
          , it2->second;

        Matrix coef(2, 1);
        coef = x.inverse() * y;

        fval = coef(0, 0) * val + coef(1, 0);

#ifdef DEBUG
        cerr << "gnss_data_interp::linear: "
             << "x = " << val << " fx = " << fval << endl;
#endif
        return 0;
    }

    // ----------------------------
    // Spline Interpolation (double)
    // ----------------------------
    int gnss_data_interp::spline(map<double, double> &data, double val, double &fval)
    {
        if (data.size() == 1)
        {
            cerr << "Warning: For using of Cubic Spline Interpolation Method, the dimension of input data vector equal to (at least): dimm = 3, is requested " << endl;
            return -1;
        }
        else if (data.size() == 2)
        {
            gnss_data_interp CallLin(_spdlog);
            CallLin.linear(data, val, fval);
        }
        else if (data.size() > 2)
        {
            //    map<double, double>::const_iterator I = data.upper_bound( val );
            map<double, double>::const_iterator II;

            vector<double> X, Y;
            for (II = data.begin(); II != data.end(); ++II)
            {
                X.push_back(II->first);
                Y.push_back(II->second);
            }

#ifdef DEBUG
            for (II = data.begin(); II != data.end(); ++II)
                std::cout << II->first << "  :  " << II->second << endl;

            std::cout << "Interpolated value " << val << endl;
#endif

            double e = val;

            vector<double> h, lambda, eta, B, C, D, g, CoefA, CoefB, CoefC, CoefD, AA, BB, FI;

            // vector h: OK
            for (unsigned int i = 0; i < X.size() - 1; i++)
            {
                h.push_back(X[i + 1] - X[i]);
            }
            // lambda: OK, eta: OK, g
            for (unsigned int i = 0; i < X.size() - 2; i++)
            {
                lambda.push_back(h[i] / (h[i] + h[i + 1]));
                eta.push_back(1.0 - lambda[i]);
                B.push_back(2); // (4.6.12.) diag = 2;
            }
            for (unsigned int j = 1; j < X.size() - 2; j++)
            {
                C.push_back(lambda[j]);
            }
            for (unsigned int j = 0; j < X.size() - 3; j++)
            {
                D.push_back(eta[j]);
            }

            Matrix A(X.size() - 2, X.size() - 2);
            Vector M(X.size() - 2);
            A.setZero();
            for (unsigned int i = 0; i < X.size() - 2; i++)
                for (unsigned int j = 0; j < X.size() - 2; j++)
                {
                    A(i, i) = B[j]; // ok
                }
            for (unsigned int i = 0; i < X.size() - 3; i++)
                for (unsigned int j = 0; j < X.size() - 3; j++)
                {
                    if (j == i)
                    {
                        A(i, j + 1) = D[j];
                        A(i + 1, j) = C[j];
                    }
                }

            Vector G(X.size() - 2), MM(X.size());
            for (unsigned int i = 0; i < X.size() - 2; i++) // ok
            {
                g.push_back((6.0 / (h[i] + h[i + 1])) * (((Y[i + 2] - Y[i + 1]) / h[i + 1]) - ((Y[i + 1] - Y[i]) / h[i])));
            }

            for (unsigned int i = 0; i < g.size(); i++)
            {
                G(i) = g[i];
            }

            // Solving a linear equations
            M = A.inverse() * G;
            double M0 = 0.0;
            double Mn = 0.0;

            MM(0) = M0;
            for (unsigned int i = 1; i < X.size() - 1; i++) // ok
            {
                MM(i) = M(i - 1);
            }
            MM(X.size() - 1) = Mn;

            // Coeficients a, b, c, d
            for (unsigned int i = 0; i < X.size() - 1; i++) // ok
            {
                CoefA.push_back(Y[i]);
                CoefB.push_back(((Y[i + 1] - Y[i]) / h[i]) - (2 * MM[i] + MM[i + 1]) * h[i] / 6);
                CoefC.push_back(0.5 * MM[i]);
                CoefD.push_back((MM[i + 1] - MM[i]) / (6 * h[i]));

                // Interpolation

                AA.push_back(((Y[i + 1] - Y[i]) / h[i]) - (h[i] / 6) * (MM[i + 1] - MM[i]));
                BB.push_back(Y[i] - MM[i] * (pow(h[i], 2) / 6));
                FI.push_back(MM[i] * (pow((X[i + 1] - e), 3) / (6 * h[i])) + MM[i + 1] * ((pow((e - X[i]), 3)) / (6 * h[i])) + AA[i] * (e - X[i]) + BB[i]);
                if (e == X[i])
                {
                    fval = Y[i];
                }
                else if (X[i] < e && e < X[i + 1])
                {
                    fval = FI[i];
                }
            }
        }
        else
        {
            cerr << "Warning (ginterp-spline): something wrong with input data!" << endl;
        }

        return 0;
    }

    // ----------------------------
    // Linear Interpolation (time)
    // ----------------------------
    int gnss_data_interp::linear(const map<base_time, double> &data, const base_time &epo, double &fval)
    {
        if (data.size() == 0)
        {
            return -1;
        }
        else if (epo == data.begin()->first)
        { // data.size() == 1 && ){

            fval = data.begin()->second;
            return 0;
        }
        else if (epo == data.rbegin()->first)
        { // data.size() == 1 && ){

            fval = data.rbegin()->second;
            return 0;
        }
        else if (data.size() == 2)
        {
            map<base_time, double>::const_iterator itDAT;
            map<double, double> tmp;

            for (itDAT = data.begin(); itDAT != data.end(); ++itDAT)
            {
                base_time dt(itDAT->first);
                double gt = (dt.mjd() - epo.mjd()) * 86400 + dt.sod() - epo.sod() + dt.dsec() - epo.dsec(); // all in sec

                tmp[gt] = itDAT->second;
                //      std::cout << " seconds = " << itDAT->first.str_ymdhms() << " " << gt << endl;
            }

            int irc = linear(tmp, 0.0, fval);
            return irc;
        }
        else
        {
            cerr << "# warning: a problem in time linear interpolation! [#node:"
                 << data.size() << "] " << epo.str_ymdhms() << endl;
            return -1;
        }

        return 0;
    }

    // ----------------------------
    // Spline Interpolation (time)
    // ----------------------------
    int gnss_data_interp::spline(const map<base_time, double> &data, const base_time &epo, double &fval)
    {
        map<base_time, double>::const_iterator I;
        map<double, double> tmp;
        for (I = data.begin(); I != data.end(); ++I)
        {
            base_time dt(I->first);
            double gt = (dt.mjd() - epo.mjd()) * 86400 + dt.sod() - epo.sod() + dt.dsec() - epo.dsec(); // all in sec

            tmp[gt] = I->second;
        }

        int irc = spline(tmp, 0.0, fval);
        return irc;

        return 0;
    }

    // ----------------------------
    // bilinear interpolation
    //
    // 0 .. 11 (bottom-left)         21 *---------* 22
    // 1 .. 12 (bottom-right)           |         |
    // 2 .. 21 (top-left)               |         |
    // 3 .. 22 (top-right)           11 *---------* 12
    // ----------------------------
    int gnss_data_interp::bilinear(const map<base_pair, double> &data, const base_pair &req_pos, double &fval)
    {
        vector<base_pair> v_pt;
        map<base_pair, double>::const_iterator itDAT = data.begin();
        for (; itDAT != data.end(); ++itDAT)
        {
            v_pt.push_back(itDAT->first);
#ifdef DEBUG
            std::cout << "# interp:" << std::fixed << setprecision(0)
                 << " " << itDAT->first[0]
                 << ":" << itDAT->first[1]
                 << " " << setprecision(3)
                 << " " << data.at(v_pt.at(v_pt.size() - 1))
                 << endl;
#endif
        }

        // no interpolation (1 point)
        if (v_pt.size() == 1)
        {
            fval = data.at(v_pt[0]);

            // linear interpolation (2 points)
        }
        else if (v_pt.size() == 2)
        {
            double x = 0.0;
            double a = 0.0;
            double b = 0.0;

            if (v_pt[1][0] - v_pt[0][0] == 0)
            {
                x = req_pos[1]; // lat=const
                a = v_pt[0][1];
                b = v_pt[1][1];
            }
            else
            {
                x = req_pos[0]; // lon=const
                a = v_pt[0][0];
                b = v_pt[1][0];
            }

            if (a - b == 0)
            {
                std::cout << "# warning: Bilinear interpolation failed (2 points) \n";
                fval = 0.0;
                return -1;
            }
            else
            {
                fval = data.at(v_pt[0]) + (data.at(v_pt[1]) - data.at(v_pt[0])) * (x - a) / (a - b);
            }

            // bilinear interpolation (4 points)
        }
        else if (v_pt.size() == 4)
        {

#ifdef DEBUG
            std::cout << "# Linear interpolation (4): " << std::fixed << setprecision(0)
                 << "  " << v_pt[0][0] << ":" << v_pt[0][1]
                 << "  " << v_pt[1][0] << ":" << v_pt[1][1]
                 << "  " << v_pt[2][0] << ":" << v_pt[2][1]
                 << "  " << v_pt[3][0] << ":" << v_pt[3][1]
                 << " ." << setprecision(3)
                 << "  " << req_pos[0] << "-" << req_pos[1]
                 << "  " << data.at(v_pt[0])
                 << "  " << data.at(v_pt[1])
                 << "  " << data.at(v_pt[2])
                 << "  " << data.at(v_pt[3])
                 << endl;
#endif

            double a = data.at(v_pt[0]) * (v_pt[3][0] - req_pos[0]) * (v_pt[3][1] - req_pos[1]);
            double b = data.at(v_pt[1]) * (req_pos[0] - v_pt[2][0]) * (v_pt[2][1] - req_pos[1]);
            double c = data.at(v_pt[2]) * (v_pt[1][0] - req_pos[0]) * (req_pos[1] - v_pt[1][1]);
            double d = data.at(v_pt[3]) * (req_pos[0] - v_pt[0][0]) * (req_pos[1] - v_pt[0][1]);

            if ((v_pt[2][0] - v_pt[1][0]) * (v_pt[1][1] - v_pt[0][1]) == 0)
            {
                //       std::cout << "# warning: Bilinear interpolation failed (4 points)\n";
                fval = 0.0;
                return -1;
            }
            else
            {
                fval = (a + b + c + d) / ((v_pt[2][0] - v_pt[1][0]) * (v_pt[1][1] - v_pt[0][1]));
            }
        }
        else
        {
            fval = 0.0;
            //     std::cout << "# warning: Bilinear interpolation failed (X points ?)\n";
            return -1;
        }

        return 0;
    }

    int gnss_data_interp::idw(const map<base_pair, double> &data, const base_pair &req_pos, double &fval)
    {
        if (data.size() < 3)
        {

            fval = 0.0;
            std::cout << "Warning: Input data's size is not appropriate for IDW spatial interpolation technique!" << endl;
            return -1;
        }

        double sumDistances = _processSumOfDistances(data, req_pos);
        double sumVals = 0.0;
        double sumWghts = 0.0;

        for (map<base_pair, double>::const_iterator I = data.begin(); I != data.end(); ++I)
        {

#ifdef DEBUG
            std::cout
                << " CRDS: " << I->first[0] << "  " << I->first[1]
                << " REQP: " << req_pos[0] << "  " << req_pos[1]
                << " VALS: " << I->second
                << endl;
#endif

            //wghts
            base_pair iPos(I->first[0], I->first[1]);

            double iDistance = _iDistance(iPos, req_pos);
            double iWght = iDistance / sumDistances;
            sumWghts = sumWghts + iWght;

            //vals
            sumVals = sumVals + (iWght * I->second);
        }

        fval = sumVals;

        return 0;
    }

    int gnss_data_interp::tps(map<base_pair, double> &data, const base_pair &req_pos, double &alpha, double &fval)
    {
        vector<double> xcrd;
        vector<double> ycrd;
        vector<double> zcrd;
        for (map<base_pair, double>::iterator j = data.begin(); j != data.end(); j++)
        {
            //std::cout << std::fixed << setprecision(4) << "INP: " << j->first[0] << "  " << j->first[1] << "  " << j->second << endl;
            xcrd.push_back(j->first[0] * 1e-0);
            ycrd.push_back(j->first[1] * 1e-0);
            zcrd.push_back(j->second * 1e-0);
        }

        double xreq = req_pos[0];
        double yreq = req_pos[1];
        //std::cout << " REQP: " << xreq << "  " << yreq << endl;

        int nData = data.size();

        vector<double> r;
        for (int i = 0; i < nData; i++)
        {
            double temp = (pow((xreq - xcrd[i]), 2.0) + pow((yreq - ycrd[i]), 2.0));
            r.push_back(temp);
        }

        //A
        Matrix A(nData, nData);
        A.setZero();
        for (int i = 0; i < nData; i++) // row
        {
            for (int j = 0; j < nData; j++) // col
            {
                if (i == j)
                {
                    A(i, j) = 0.0;
                }
                else
                {
                    double Rij = (pow((xcrd[i] - xcrd[j]), 2.0) + pow((ycrd[i] - ycrd[j]), 2.0));
                    A(i, j) = Rij * log(sqrt(Rij));
                }
            }
        }

        // alp*I
        double s = A.sum();
        double S = s / pow(nData, 2);

        Diag I(nData);
        I.setIdentity();

        Matrix K = A + (I.matrixR() * pow(S, 2) * alpha);

        // T
        Matrix T(nData, 3);
        for (int i = 0; i < nData; i++)
        {
            T(i, 0) = 1;
            T(i, 1) = xcrd[i];
            T(i, 2) = ycrd[i];
        }

        // T.transpose();
        Matrix Tt = T.transpose();

        // Z
        Matrix Z(3, 3);
        Z.setZero();

        // W
        Matrix W(nData + 3, 1);
        for (int i = 0; i < nData; i++)
        {
            W(i, 0) = zcrd[i];
        }

        // BLOCK M
        Matrix BL(nData + 3, nData + 3);
        BL.setZero();

        for (int i = 0; i < nData + 3; i++)
        {

            int idx = 0;

            for (int j = 0; j < nData + 3; j++)
            {
                if (i < nData && j < nData)
                {
                    BL(i, j) = K(i, j);
                }
                else if (i < nData && j >= nData)
                {
                    BL(i, j) = T(i, idx);
                    idx++;
                }
                else if (i >= nData && j < nData)
                {
                    BL(i, j) = Tt(i - nData, j);
                }
            }
        }

        //std::cout << BL << endl;

        // LAM
        Matrix LAM(nData + 3, 1);
        LAM = BL.inverse() * W;

        // vec lam
        vector<double> lam;
        for (int l = 0; l < nData; l++)
        {
            lam.push_back(LAM(l, 0));
        }

        double v00 = LAM(nData, 0);
        double v10 = LAM(nData + 1, 0);
        double v20 = LAM(nData + 2, 0);

        double temp_A = 0.0;
        for (int m = 0; m < nData; m++)
        {
            temp_A += lam[m] * r[m] * log(r[m]);
        }

        fval = (0.5 * temp_A) + v00 + (v10 * xreq) + (v20 * yreq);

        return 0;
    }

    double gnss_data_interp::_processSumOfDistances(const map<base_pair, double> &data, const base_pair &req)
    {

        map<base_pair, double>::const_iterator it;
        double sumDist = 0.0;

        for (it = data.begin(); it != data.end(); ++it)
        {

            double dist = 1.0 / sqrt(pow(abs((it->first[0] - req[0])), 1.0) +
                                     pow(abs((it->first[1] - req[1])), 1.0));

#ifdef DEBUG
            std::cout
                << it->first[0] << "  " << req[0] << " dif " << it->first[0] - req[0] << " pow " << pow((it->first[0] - req[0]), 3.0)
                << "  " << it->first[1] << "  " << req[1] << " dif " << it->first[1] - req[1] << " pow " << pow((it->first[1] - req[1]), 3)
                << "  " << dist << endl;
#endif

            sumDist = sumDist + dist;
        }

        return sumDist;
    }

    double gnss_data_interp::_iDistance(const base_pair &pos, const base_pair &req)
    {

        double dist = 1.0 / sqrt(pow(abs((pos[0] - req[0])), 1.0) +
                                 pow(abs((pos[1] - req[1])), 1.0));

        return dist;
    }

} // namespace
