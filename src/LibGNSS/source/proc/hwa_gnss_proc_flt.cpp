#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <chrono>

#include "hwa_gnss_proc_flt.h"
#include "hwa_gnss_all_fltmat.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{

    // Constructors
    gnss_proc_flt::gnss_proc_flt()
    {
    }
    void gnss_proc_flt::add_data(const base_allpar &param, const Vector &dx, const Symmetric &Qx, const double &sigma0, const Symmetric &Qx0)
    {
        try
        {
            _param = param;
            _dx = dx;
            _Qx = Qx;
            _sigma0 = sigma0;
            _Qx0 = Qx0;
        }
        catch (...)
        {
        }
    }
    void gnss_proc_flt::add_data(const Matrix &A, const Symmetric &P, const Vector &l)
    {
        try
        {
            _A = A;
            _P = P;
            _l = l;
            _A_virtual.resize(0, 0);
            _P_virtual.resize(0);
            _l_virtual.resize(0);
        }
        catch (...)
        {
        }
    }
    void gnss_proc_flt::add_data(const double &vtpv, const int &nobs_total, const int &npar_number)
    {
        _vtpv = vtpv;
        _nobs_total = nobs_total;
        _npar_number = npar_number;
    }

    void gnss_proc_flt::add_virtual_obs(const Matrix &A, const Symmetric &P, const Vector &l)
    {
        int virtual_obs_num = l.size();
        ResizeKeep(_A, _nobs_total + virtual_obs_num, _npar_number);
        _P.ResizeKeep(_nobs_total + virtual_obs_num);
        ResizeKeep(_l, _nobs_total + virtual_obs_num);

        int virtual_total_obs_num = _l_virtual.size();
        ResizeKeep(_A_virtual, virtual_total_obs_num + virtual_obs_num, _npar_number);
        _P_virtual.ResizeKeep(virtual_total_obs_num + virtual_obs_num);
        ResizeKeep(_l_virtual,virtual_total_obs_num + virtual_obs_num);

        // Float + Fixed
        _A.block(_nobs_total, 0,  virtual_obs_num, _npar_number) = A;
        _P.matrixW().block(_nobs_total, _nobs_total, virtual_obs_num, virtual_obs_num) = P.matrixR();
        _l.block(_nobs_total, 0, virtual_obs_num, 1) = l;
        _nobs_total += virtual_obs_num;

        // Fixed Only
        _A_virtual.block(virtual_total_obs_num, 0, virtual_obs_num, _npar_number) = A;
        _P_virtual.matrixW().block(virtual_total_obs_num, virtual_total_obs_num, virtual_obs_num, virtual_obs_num) = P.matrixR();
        _l_virtual.block(virtual_total_obs_num, 0, virtual_obs_num, 1) = l;
    }

    void gnss_proc_flt::get_virtual_obs(Matrix &A, Symmetric &P, Vector &l)
    {
        A = _A_virtual;
        P = _P_virtual;
        l = _l_virtual;
    }

    void gnss_proc_flt::change_Qx(int row, int col, double xx)
    {
        _Qx.set(xx, row, col);
    }
    void gnss_proc_flt::change_dx(int n, double xx)
    {
        if (n < 0 || n >= _dx.size())
        {
            _dx.setConstant(xx);
        }
        else
        {
            _dx(n) = xx;
        }
    }
    void gnss_proc_flt::change_stdx(int n, double xx)
    {
    }
    void gnss_proc_flt::change_sigma0(double sigma)
    {
        _sigma0 = sigma;
    }
    void gnss_proc_flt::change_vtpv(double xx)
    {
        _vtpv = xx;
    }

    Vector gnss_proc_flt::stdx()
    {
        _stdx.resize(_Qx.rows());
        _stdx.setZero();
        for (int i = 0; i < _Qx.rows(); i++)
        {
            _stdx(i) = sqrt(_Qx(i, i)) * _sigma0;
        }
        return _stdx;
    }

    gnss_proc_kalman::gnss_proc_kalman()
    {
    }

    gnss_proc_srf::gnss_proc_srf()
    {
    }

    gnss_proc_srif::gnss_proc_srif()
    {
    }

    // Destructors
    gnss_proc_flt::~gnss_proc_flt()
    {
    }

    gnss_proc_kalman::~gnss_proc_kalman()
    {
    }

    gnss_proc_srf::~gnss_proc_srf()
    {
    }

    gnss_proc_srif::~gnss_proc_srif()
    {
    }

    bool gnss_proc_flt::outlierDetect(const Matrix &A, const Symmetric &Q, const Vector &dx, const Vector &l, Symmetric &P)
    {
        bool res = false;
        double k0 = 3, k1 = 7;
        int n = l.rows();
        Vector v_post = l - A * dx;
        Matrix Qz = A * Q.matrixR() * A.transpose() + P.matrixR().inverse();
        Vector v_norm;
        v_norm.resize(n);
        v_norm.setZero();
        int idx = -1;
        double max_res = 0.0;
        double gamma = 1.0;
        //Vector gamma; gamma.resize(n); gamma = 1.0;
        for (int i = 0; i < n; i++)
        {
            v_norm(i) = sqrt(1.0 / Qz(i, i)) * abs(v_post(i));
            if (v_norm(i) > max_res)
            {
                max_res = v_norm(i);
                idx = i;
            }
        }

        if (max_res <= k0)
            gamma = 1.0;
        else if (max_res > k0 && max_res <= k1)
        {
            gamma = max_res / k0 * (k1 - k0) / (k1 - max_res);
        }
        else // when max_res > k1
        {
            gamma = 1e2;
        }
        if (gamma > 1.0)
            res = true;
        double tmp = P(idx, idx) / gamma;
        P.set(tmp, idx, idx);
        return res;
    }

    void gnss_proc_flt::resetQ()
    {
        _Qx = _Qx0;
    }

    void gnss_proc_kalman::update()
    {
        gnss_proc_kalman::update(_A, _P, _l, _dx, _Qx);
    }
    // Methods
    //
    // Update filter
    // ---------------
    // Input arguments:
    // A  ... first design matrix
    // Ql ... variance-covariance matrix of measurements
    // l  ... reduced measurements
    // dx ... state std::vector
    // Qx ... variance-covariance matrix of state
    //
    // Output arguments:
    // dx ... state std::vector
    // Qx ... variance-covariance matrix of state
    //
    void gnss_proc_kalman::update(const Matrix &A, const Diag &Pl, const Vector &l, Vector &dx, Symmetric &Qx)
    {

        //  high_resolution_clock::time_point t1 = high_resolution_clock::now();

        Matrix K; // Kalman gain
        Matrix NN;

        NN = Pl.matrixR().inverse() + A * Qx.matrixR() * A.transpose();

        K = Qx.matrixR() * A.transpose() * NN.inverse();
        //  std::cout  << "Qx" << std::endl << Qx << std::endl << "KAQx" << std::endl << K*A*Qx << std::endl << "K" << K << std::endl;

        Matrix I = Matrix::Identity(Qx.rows(), Qx.rows());
        Matrix KA = K * A;
        Matrix I_KA = I - K * A;

        // ZhouYuxuan modified. "K * ( l - A * dx )" => "K * l"

        dx = K * l;                                      // update state std::vector
        Qx.matrixW() = I_KA * Qx.matrixR() * I_KA.transpose() + K * Pl.matrixR().inverse() * K.transpose(); // update variance-covariance matrix of state
                                                         //Qx << Qx - K * A * Qx;               // update variance-covariance matrix of state
        //  std::cout << "Qx_upd" << std::endl << Qx << std::endl;
        //  high_resolution_clock::time_point t2 = high_resolution_clock::now();
        //  auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        //  std::cout << duration << std::endl;
    }
    //for rtk filter
    void gnss_proc_kalman::update(const Matrix &A, const Symmetric &Pl, const Vector &l, Vector &dx, Symmetric &Qx)
    {
        Matrix K;
        Matrix NN;
        Symmetric Pli;
        Pli.matrixW() = Pl.matrixR().inverse();

        NN = Pli.matrixR() + A * Qx.matrixR() * A.transpose();
        K = Qx.matrixR() * A.transpose() * NN.inverse();

        Matrix I = Matrix::Identity(Qx.rows(), Qx.rows());
        Matrix KA = K * A;
        Matrix I_KA = I - K * A;

        dx = K * l;
        Qx.matrixW() = I_KA * Qx.matrixR() * I_KA.transpose() + K * Pli.matrixR() * K.transpose();
    }

    void gnss_proc_kalman::update(const Matrix& A, const Matrix& R, const Vector& l, Vector& dx, Symmetric& Qx)
    {
        Matrix K;
        Matrix NN;

        NN = R + A * Qx.matrixR() * A.transpose();
        K = Qx.matrixR() * A.transpose() * NN.inverse();

        Matrix I = Matrix::Identity(Qx.rows(), Qx.rows());
        Matrix KA = K * A;
        Matrix I_KA = I - K * A;

        dx = K * l;
        Qx.matrixW() = I_KA * Qx.matrixR() * I_KA.transpose() + K * R * K.transpose();

        //std::cout << "A" << std::endl << std::fixed << std::setprecision(6) << std::setw(15) << A << std::endl;
        //std::cout << "R" << std::endl << std::fixed << std::setprecision(6) << std::setw(15) << R << std::endl;
        //std::cout << "L" << std::endl << std::fixed << std::setprecision(6) << std::setw(15) << l << std::endl;
        //std::cout << "dx" << std::endl << std::fixed << std::setprecision(6) << std::setw(15) << dx << std::endl;
    }

    void gnss_proc_srf::update()
    {
        gnss_proc_srf::update(_A, _P, _l, _dx, _Qx);
    }
    void gnss_proc_srf::update(const Matrix &A, const Diag &Pl, const Vector &l, Vector &dx, Symmetric &Qx)
    {

        //  high_resolution_clock::time_point t1 = high_resolution_clock::now();

        int nObs = A.rows();
        int nPar = A.cols();

        Matrix SS;
        try
        {
            Eigen::LLT<Matrix> llt(Qx.matrixR());
            if (llt.info() == Eigen::NumericalIssue) {
                return;
            }
            SS = llt.matrixL().transpose();
        }
        catch (...)
        {
            return;
        }

        Matrix SA = SS * A.transpose();
        Matrix SRF(nObs + nPar, nObs + nPar);
        SRF.setZero();
        for (int ii = 0; ii < nObs; ++ii)
        {
            SRF(ii, ii) = 1.0 / sqrt(Pl(ii, ii));
        }

        SRF.block(nObs ,0, nPar, nObs) = SA;
        SRF.block(nObs, nObs, nPar, nPar) = SS;

        Eigen::HouseholderQR<Matrix> qr(SRF);
        Matrix UU = qr.matrixQR().triangularView<Eigen::Upper>();

        SS = UU.block(nObs, nObs, nPar, nPar);
        Matrix SH_rt = UU.block(0, 0, nObs, nObs);
        Matrix YY = UU.block(0, nObs, nObs, nPar);
        Matrix SHi = SH_rt.inverse();

        Matrix KT = SHi * YY;
        Symmetric Hi;
        Hi.matrixW() = SHi * SHi.transpose();

        dx = KT.transpose() * l;
        Qx.matrixW() = (SS.transpose() * SS);

        //  high_resolution_clock::time_point t2 = high_resolution_clock::now();
        //  auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        //  std::cout << duration << std::endl;
    }

    void gnss_proc_srf::update(const Matrix &A, const Symmetric &Pl, const Vector &l, Vector &dx, Symmetric &Qx)
    {

        //  high_resolution_clock::time_point t1 = high_resolution_clock::now();

        int nObs = A.rows();
        int nPar = A.cols();

        Matrix SS;
        try
        {
            Eigen::LLT<Matrix> llt(Qx.matrixR());
            if (llt.info() == Eigen::NumericalIssue) {
                return;
            }
            SS = llt.matrixL().transpose();
        }
        catch (...)
        {
            return;
        }

        Matrix SA = SS * A.transpose();

        Matrix SRF(nObs + nPar, nObs + nPar);
        SRF.setZero();

        // Maybe can be simplified.
        Symmetric Pli;
        Pli.matrixW() = Pl.matrixR().inverse();

        Eigen::LLT<Matrix> llt1(Pli.matrixR());
        if (llt1.info() == Eigen::NumericalIssue) {
            return;
        }
        Matrix SPl = llt1.matrixL().transpose();

        SRF.block(0, 0, nObs, nObs) = SPl;
        SRF.block(nObs, 0, nPar, nObs) = SA;
        SRF.block(nObs, nObs, nPar, nPar) = SS;

        Eigen::HouseholderQR<Matrix> qr(SRF);
        Matrix UU = qr.matrixQR().triangularView<Eigen::Upper>();

        SS = UU.block(nObs, nObs, nPar, nPar);
        Matrix SH_rt = UU.block(0, 0, nObs, nObs);
        Matrix YY = UU.block(0, nObs, nObs, nPar);
        Matrix SHi = SH_rt.inverse();

        Matrix KT = SHi * YY;
        Symmetric Hi;
        Hi.matrixW() = SHi * SHi.transpose();

        dx = KT.transpose() * l;
        Qx.matrixW() = (SS.transpose() * SS);

        //  high_resolution_clock::time_point t2 = high_resolution_clock::now();
        //  auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        //  std::cout << duration << std::endl;
    }

    // needs debugging
    void gnss_proc_srif::update(const Matrix &A, const Diag &Pl, const Vector &l, Vector &dx, Symmetric &Qx)
    {
        int nObs = A.rows();
        int nPar = A.cols();
        Symmetric INF(nPar);
        INF.setZero();
        INF.matrixW() = Qx.matrixR().inverse();


        Eigen::LLT<Matrix> llt(INF.matrixR());
        if (llt.info() == Eigen::NumericalIssue) {
            return;
        }
        Matrix R = llt.matrixL().transpose();

        Vector z(nPar);
        z.setZero();

        Matrix SRIF(nObs + nPar, nPar + 1);
        SRIF.setZero();

        SRIF.block(0 ,0, nPar, nPar) = R;
        SRIF.block(nPar, 0, nObs, nPar) = A;
        SRIF.block(0, nPar, nPar, 1) = z;
        SRIF.block(nPar, nPar, nObs, 1) = l;

        /* SRIF.SubMatrix(1, nObs, 1, nPar) = A; */
        /* SRIF.SubMatrix(nObs+1, nPar+nObs, 1, nPar) = R;    */
        /* SRIF.SubMatrix(1, nObs, nPar+1, nPar+1) = l; */
        /* SRIF.SubMatrix(nObs+1, nPar+nObs, nPar+1, nPar+1) = z; */

        Eigen::HouseholderQR<Matrix> qr(SRIF);
        Matrix MU = qr.matrixQR().triangularView<Eigen::Upper>();

        R.setZero();
        z.setZero();
        R = MU.block(0, 0, nPar, nPar);
        z = MU.block(0, nPar, nPar, 1);

        INF.matrixW() = R * R.transpose();

        dx = R.inverse() * z;
        Qx.matrixW() = INF.matrixR().inverse();
        //   std::cout << "nPar = " << nPar << std::endl << "nObs = " << nObs << std::endl;
        //   std::cout << "MU dim = (" << MU.rows() << ", " << MU.cols() << " )" << std::endl;
        //  std::cout << Qx << std::endl;
        //  std::cout << dx << std::endl;
        //  int ooo; cin >> ooo;
    }

} // namespace
