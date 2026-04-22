#include "hwa_base_filter.h"

hwa_base::Updater hwa_base::str2updater(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(),
        [](unsigned char c) { return std::toupper(c); });
    if (str == "EKF") return EKF;
    else if (str == "UKF") return UKF;
    else if (str == "CKF") return CKF;
    else if (str == "PF") return PF;
    else if (str == "VBAKF") return VBAKF;
    else if (str == "GSTM") return GSTM;
    else if (str == "GSTM_1") return GSTM_1;
    else if (str == "GSTM_2") return GSTM_2;
    else return EKF;
}

double hwa_base::VecNorm(Vector x) {
    double a = x.transpose() * x;
    return sqrt(a);
}

int hwa_base::base_updater::_meas_update(Matrix& Hk, Vector& Zk, Matrix& Rk, Vector& Xk, Matrix& Pk) {
    int res = -1;
    switch (filter) {
    case GSTM:
    case GSTM_1:
    case GSTM_2:
        res = _meas_update_gstm(Hk, Zk, Rk, Xk, Pk);
        break;
    case EKF:
        res = _meas_update_ekf(Hk, Zk, Rk, Xk, Pk);
        break;
    case VBAKF:
        res = _meas_update_vbakf(Hk, Zk, Rk, Xk, Pk);
        break;
    case UKF:
        res = _meas_update_ukf(Xk, Pk, Rk);
        break;
    case PF:
        set_particles(Pk);
        res = _meas_update_pf(Xk, Pk, Rk);
        set_particles(Pk);
        break;
    default:
        res = _meas_update_ekf(Hk, Zk, Rk, Xk, Pk);
        break;
    }
    curr_iter++;
    return res;
}

int hwa_base::base_updater::_meas_update_gstm(Matrix& Hk, Vector& Zk, Matrix& Rk, Vector& Xk, Matrix& Pk)
{
    try {
        double logbw1 = boost::math::digamma(g0) - boost::math::digamma(1);
        double logbw2 = boost::math::digamma(e0) - boost::math::digamma(1);
        double logconjbw1 = boost::math::digamma(1 - g0) - boost::math::digamma(1);
        double logconjbw2 = boost::math::digamma(1 - e0) - boost::math::digamma(1);
        double bv1 = 1, bv2 = 1;
        double gv1 = 1, gv2 = 1;
        double loggv1 = 0, loggv2 = 0;
        int df1, df2;
        int dim = Pk.rows();
        int GSTMiter = 0;
        int nr = Zk.size();

        Vector SavXk;
        Matrix TempPk, TempRk;
        Matrix Ak, Bk;
        Vector C_logbw2 = Vector::Constant(Rk.rows(), boost::math::digamma(e0) - boost::math::digamma(1));
        Vector C_logconjbw2 = Vector::Constant(Rk.rows(), boost::math::digamma(1 - e0) - boost::math::digamma(1));
        Vector C_bv2 = Vector::Ones(Rk.rows());
        Vector C_gv2 = Vector::Ones(Rk.rows());
        Vector C_loggv2 = Vector::Zero(Rk.rows());
        Vector C_Bk = Vector::Zero(Rk.rows());

        Vector X_sav = Xk;
        Matrix Pk_Sav = Pk;
        Matrix Rk_sav = Rk;

        if (filter == GSTM_2 && curr_iter == 0) {
            Matrix Pk1 = Matrix::Zero(Rk.rows() + Pk.rows(), Rk.rows() + Pk.rows());
            Matrix Hk1 = Matrix::Zero(Hk.rows() + Pk.rows(), Hk.cols());
            Matrix Zk1 = Matrix::Zero(Zk.rows() + Pk.rows(), Zk.cols());

            Pk1.block(0, 0, Rk.rows(), Rk.cols()) = Rk.matrix();
            Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(Pk);
            Matrix eigenvectors = eigenSolver.eigenvectors().real();
            Pk1.block(Rk.rows(), Rk.cols(), Pk.rows(), Pk.cols()) = (eigenvectors.transpose() * Pk * eigenvectors).matrix();
            Hk1.block(0, 0, Hk.rows(), Hk.cols()) = Hk.matrix();
            Hk1.block(Hk.rows(), 0, Pk.rows(), Hk.cols()) = eigenvectors.transpose().matrix();
            Zk1.block(0, 0, Zk.rows(), Zk.cols()) = Zk.matrix();

            C_logbw2 = Vector::Constant(Pk1.rows(), boost::math::digamma(e0) - boost::math::digamma(1));
            C_logconjbw2 = Vector::Constant(Pk1.rows(), boost::math::digamma(1 - e0) - boost::math::digamma(1));
            C_bv2 = Vector::Ones(Pk1.rows());
            C_gv2 = Vector::Ones(Pk1.rows());
            C_loggv2 = Vector::Zero(Pk1.rows());
            C_Bk = Vector::Zero(Pk1.rows());

            Hk = Hk1;
            Zk = Zk1;
            Rk = Pk1;
        }

        do {
            df1 = Xk.size();
            df2 = Rk.rows();
            GSTMiter++;

            SavXk = Xk;

            // Update Stochasitc Model
            if (filter == GSTM) {
                TempPk = Pk_Sav / (bv1 + (1 - bv1) * gv1);
                TempRk = Rk / (bv2 + (1 - bv2) * gv2);
            }
            else {
                TempPk = Pk_Sav;
                TempRk = Rk;
                for (int i = 0; i < Rk.rows(); i++) {
                    TempRk(i, i) /= (C_bv2(i) + (1 - C_bv2(i)) * C_gv2(i));
                }
            }

            //Measurement Update
            if (filter == GSTM - 2) {
                Matrix Qr = (Hk.transpose() * TempRk.inverse() * Hk).inverse();
                Xk = Qr * Hk.transpose() * TempRk.inverse() * Zk;
                Pk = Qr;
            }
            else {
                Matrix Him = Hk.transpose();
                Matrix Pxzm = TempPk * Him;
                Matrix Pz0m = Hk * Pxzm;
                Matrix Pzzm = Pz0m + TempRk;
                Matrix Kkm = Pxzm * Pzzm.inverse();
                Xk = Kkm * Zk;
                Matrix I_KH = Matrix::Identity(Kkm.rows(), Hk.cols()) - Kkm * Hk;
                Pk = I_KH * TempPk * I_KH.transpose() + Kkm * Kkm.transpose() * proc_noise;
            }

            //Variational Bayes
            if (filter == GSTM) {
                Ak = (Xk - X_sav) * (Xk - X_sav).transpose() + Pk;
                Bk = (Zk - Hk * Xk) * (Zk - Hk * Xk).transpose() + Hk * Pk * Hk.transpose();

                bv1 = 1 / (1 + exp(logconjbw1 + dim * loggv1 / 2 - logbw1 + (1 - gv1) * (Ak * Pk_Sav.inverse()).trace() / 2));
                bv2 = 1 / (1 + exp(logconjbw2 + nr * loggv2 / 2 - logbw2 + (1 - gv2) * (Bk * Rk.inverse()).trace() / 2));

                double shape1 = 0.5 * dim * (1 - bv1) + 0.5 * df1;
                double size1 = 0.5 * (Ak * Pk_Sav.inverse()).trace() * (1 - bv1) + 0.5 * df1;
                double shape2 = 0.5 * nr * (1 - bv2) + 0.5 * df2;
                double size2 = 0.5 * (Bk * Rk.inverse()).trace() * (1 - bv2) + 0.5 * df2;
                gv1 = shape1 / size1;
                gv2 = shape2 / size2;
                loggv1 = boost::math::digamma(shape1) - log(size1);
                loggv2 = boost::math::digamma(shape2) - log(size2);

                double temp11 = g0 + bv1;
                double temp12 = 2 - g0 - bv1;
                double temp21 = e0 + bv2;
                double temp22 = 2 - e0 - bv2;
                logbw1 = boost::math::digamma(temp11) - boost::math::digamma(temp11 + temp12);
                logconjbw1 = boost::math::digamma(temp12) - boost::math::digamma(temp11 + temp12);
                logbw2 = boost::math::digamma(temp21) - boost::math::digamma(temp21 + temp22);
                logconjbw2 = boost::math::digamma(temp22) - boost::math::digamma(temp21 + temp22);
            }
            else {
                Ak = (Xk - X_sav) * (Xk - X_sav).transpose() + Pk;
                bv1 = 1 / (1 + exp(logconjbw1 + dim * loggv1 / 2 - logbw1 + (1 - gv1) * (Ak * Pk_Sav.inverse()).trace() / 2));
                for (int i = 0; i < Rk.rows(); i++) {
                    C_Bk(i) = pow(Zk(i) - (Hk.row(i) * Xk).value(), 2) + (Hk.row(i) * Pk * Hk.row(i).transpose()).value();
                }
                for (int i = 0; i < Rk.rows(); i++) {
                    C_bv2(i) = 1 / (1 + exp(C_logconjbw2(i) + nr * C_loggv2(i) / 2 - C_logbw2(i) + (1 - C_gv2(i)) * C_Bk(i) / (2 * Rk(i, i))));
                }

                double shape1 = 0.5 * dim * (1 - bv1) + 0.5 * df1;
                double size1 = 0.5 * (Ak * Pk_Sav.inverse()).trace() * (1 - bv1) + 0.5 * df1;
                gv1 = shape1 / size1;
                loggv1 = boost::math::digamma(shape1) - log(size1);
                double temp11 = g0 + bv1;
                double temp12 = 2 - g0 - bv1;
                logbw1 = boost::math::digamma(temp11) - boost::math::digamma(temp11 + temp12);
                logconjbw1 = boost::math::digamma(temp12) - boost::math::digamma(temp11 + temp12);

                for (int i = 0; i < Rk.rows(); i++) {
                    double shape2 = 0.5 * nr * (1 - C_bv2[i]) + 0.5 * df2;
                    double size2 = 0.5 * C_Bk[i] / Rk(i, i) * (1 - C_bv2[i]) + 0.5 * df2;
                    C_gv2[i] = shape2 / size2;
                    C_loggv2[i] = boost::math::digamma(shape2) - log(size2);

                    double temp21 = e0 + C_bv2[i];
                    double temp22 = 2 - e0 - C_bv2[i];
                    C_logbw2[i] = boost::math::digamma(temp21) - boost::math::digamma(temp21 + temp22);
                    C_logconjbw2[i] = boost::math::digamma(temp22) - boost::math::digamma(temp21 + temp22);
                }
            }
        } while (VecNorm(SavXk - Xk) / VecNorm(Xk) > 0.00001 && GSTMiter < max_iter);
        return 1;
    }
    catch (...) {
        return -1;
    }
}

int  hwa_base::base_updater::_meas_update_ekf(const Matrix& Hk, const Vector& Zk, const Matrix& Rk, Vector& Xk, Matrix& Pk) {
    try {
        Xk = Vector::Zero(Pk.rows());
        Matrix Him = Hk.transpose();
        Matrix Pxzm = Pk * Him;
        Matrix Pz0m = Hk * Pxzm;
        Matrix rm = Zk - Him.transpose() * Xk;
        Matrix Pzzm = Pz0m + Rk;
        Matrix Kkm = Pxzm * Pzzm.inverse();
        Xk = Kkm * rm;
        Matrix I_KH = Matrix::Identity(Kkm.rows(), Hk.cols()) - Kkm * Hk;
        //Pk = I_KH * Pk * I_KH.transpose() + Kkm * Kkm.transpose() * proc_noise;
        Pk = I_KH * Pk * I_KH.transpose() + Kkm * Rk * Kkm.transpose();
        return 1;
    }
    catch (...) {
        return -1;
    }
}

int  hwa_base::base_updater::_meas_update_vbakf(const Matrix& Hk, const Vector& Zk, const Matrix& Rk, Vector& Xk, Matrix& Pk) {
    try {
        Matrix Pk_Sav = Pk;
        for (int i = 0; i < 5; i++) {
            Matrix Am = Pk + Xk * Xk.transpose();
            Matrix Tm = Am + tau * Pk_Sav;
            Pk = Tm / (tau + 1);

            Matrix Him = Hk.transpose();
            Matrix Pxzm = Pk * Him;
            Matrix Pz0m = Hk * Pxzm;

            Matrix rm = Zk;
            Matrix Pzzm = Pz0m + Rk;
            Matrix Kkm = Pxzm * Pzzm.inverse();
            Xk = Kkm * rm;

            Matrix I_KH = Matrix::Identity(Kkm.rows(), Hk.cols()) - Kkm * Hk;
            Pk = I_KH * Pk * I_KH.transpose() + Kkm * Kkm.transpose() * proc_noise;
            return 1;
        }
    }
    catch (...) {
        return -1;
    }
}

int  hwa_base::base_updater::_meas_update_ukf(Vector& Xk, Matrix& Pk, Matrix& Rk)
{
    try{
        if (CarPos.size() == 0 || IniRange.size() == 0 || AnchorPos.size() == 0) {
            std::cout << " Should Introduce Information If Use UKF." << "\n";
            return -1;
        }
        int n = Xk.size(), obs_crt = 0, l = AnchorPos.size();
        Matrix Kk = Matrix::Zero(n, AnchorPos.size());
        double alpha = alpha_sig, beta = 2, kappa = kappa_sig,
            lambda = pow(alpha, 2) * (n + kappa) - n,
            gamma = sqrt(n / 1.0 + lambda);
        Vector Wm = Vector::Zero(2 * n + 1),
            Wc = Vector::Zero(2 * n + 1);

        Matrix xSig = Matrix::Zero(n, 2 * n + 1);
        Matrix zSig = Matrix::Zero(l, 2 * n + 1);
        Vector zSigBar = Vector::Zero(l);
        Matrix cSig = Matrix::Zero(l, l);
        Matrix kSig = Matrix::Zero(n, l);
        Vector Xkm = Vector::Zero(n);
        xSig.block(0, 0, n, 1) = Xk.matrix();
        Wm(0) = lambda / pow(gamma, 2);
        Wc(0) = Wm(0) + 1 - pow(alpha, 2) + beta;
        Matrix Sk = Pk.llt().matrixL();
        Sk = Sk * gamma;

        for (int iSig = 0; iSig < 2 * n + 1; iSig++)
        {
            if (iSig > 0)
            {
                Wm(iSig) = 0.5 / pow(gamma, 2);
                Wc(iSig) = Wm(iSig);
                if (iSig < n + 1)
                    xSig.block(0, iSig, n, 1) = Xk + Sk.block(0, iSig - 1, n, 1);
                else
                    xSig.block(0, iSig, n, 1) = Xk - Sk.block(0, iSig - 1 - n, n, 1);
            }
            obs_crt = 0;
            for (auto it = AnchorPos.begin(); it != AnchorPos.end(); it++)
            {
                Triple base_crd_vector = *it;
                double dist = (CarPos - xSig.block(6, iSig, 3, 1) - base_crd_vector).norm();
                zSig(obs_crt, iSig) = dist;
                obs_crt++;
            }
            zSigBar = zSigBar + Wm(iSig) * zSig.block(0, iSig, l, 1);
            Xkm = Xkm + Wm(iSig) * xSig.block(0, iSig, n, 1);
        }

        Xk = Xkm;

        Vector zerror(n), xerror(n);
        for (int iSig = 0; iSig < 2 * n + 1; iSig++)
        {
            zerror = zSig.block(0, iSig, l, 1) - zSigBar.matrix();
            xerror = xSig.block(0, iSig, n, 1) - Xk.matrix();
            cSig = cSig + Wc(iSig) * zerror * zerror.transpose();   //zz
            kSig = kSig + Wc(iSig) * xerror * zerror.transpose();   //zx
        }

        cSig = cSig + Rk;
        Kk = kSig * cSig.inverse();
        Xk = Xk + Kk * (IniRange - zSigBar);
        Pk = Pk - Kk * cSig * Kk.transpose();
        return 1;
    }
    catch (...) {
        return -1;
    }
}

int  hwa_base::base_updater::_meas_update_pf(Vector& Xk, Matrix& Pk, Matrix& Rk)
{
    try{
        if (CarPos.size() == 0 || IniRange.size() == 0 || AnchorPos.size() == 0) {
            std::cout << " Should Introduce Information If Use UKF." << "\n";
            return -1;
        }
        int obs_crt = 0, n = Xk.size();
        Matrix zSig = Matrix::Zero(AnchorPos.size(), _num_particles);
        Vector weight = Vector::Zero(_num_particles);
        Vector cum_weight = Vector::Zero(_num_particles);

        Matrix Kk = Matrix::Zero(n, AnchorPos.size());

        for (int idx = 0; idx < _num_particles; idx++)
        {
            obs_crt = 0;
            for (int i = 0; i < AnchorPos.size(); i++)
            {
                Triple base_crd_vector = AnchorPos[i];
                double dist = (CarPos + _particles.block(0, idx, n, 1) - base_crd_vector).norm();
                zSig(obs_crt, idx) = dist;
                obs_crt++;
            }
            Vector res = IniRange - zSig.block(0, idx, AnchorPos.size(), 1);

            double vtpv = res.transpose() * (Rk / Rk.norm()).inverse() * res;
            weight(idx) = exp(-vtpv / 2.0);
            if (idx == 0)
                cum_weight(idx) = weight(idx);
            else
                cum_weight(idx) = cum_weight(idx - 1) + weight(idx);
        }
        cum_weight = cum_weight / weight.sum();
        weight = weight / weight.sum();

        Eigen::VectorXi indexes = Eigen::VectorXi::Zero(_num_particles);
        residual_resample(weight, indexes);

        Matrix new_particles = Matrix::Zero(n, _num_particles);
        for (int idx = 0; idx < _num_particles; idx++)
        {
            new_particles.block(0, idx, n, 1) = _particles.block(0, indexes(idx), n, 1);
        }

        _particles = new_particles;
        Xk = _particles.rowwise().sum() / _num_particles;
        new_particles = Matrix::Ones(n, _num_particles);

        for (int i = 0; i < n; i++)
        {
            new_particles.row(i) = Xk(i) * new_particles.row(i);
        }
        Pk = ((_particles - new_particles) * (_particles - new_particles).transpose()) / _num_particles;
        return 1;
    }
    catch (...) {
        return -1;
    }
}

void hwa_base::base_updater::residual_resample(const Vector& weights, Vector_T<int>& indexes)
{
    //Vector num_copies = weights * _num_particles, cumulative_sum = indexes.cast<double>();
    Vector num_copies = weights * _num_particles;
    Vector cumulative_sum = Vector::Zero(_num_particles);

    for (int i = 0; i < num_copies.size(); i++)
        num_copies(i) = floor(num_copies(i));

    int k = 0;
    for (int i = 0; i < _num_particles; i++)
    {
        for (int j = 0; j < num_copies(i); j++)
        {
            indexes(k) = i;
            k = k + 1;
        }
    }

    Vector residual = weights - num_copies;
    residual = residual / residual.sum();

    cumulative_sum(0) = residual(0);
    for (int i = 1; i < _num_particles; i++)
    {
        cumulative_sum(i) = cumulative_sum(i - 1) + residual(i);
    }
    cumulative_sum(_num_particles - 1) = 1.0;

    for (int i = k; i < _num_particles; i++)
    {
        std::random_device e;
        std::uniform_real_distribution<double> u(0, 1);
        double r = u(e);
        int idx = 0;
        for (int j = 1; j < _num_particles; j++)
        {
            if (cumulative_sum(j - 1) < r && cumulative_sum(j) >= r)
            {
                idx = j;
            }
        }
        indexes(i) = idx;
    }

    return;
}

void hwa_base::base_updater::set_particles(Matrix& Pk)
{
    if (filter != PF || _num_particles <= 0)
        return;

    for (int row = 0; row < Pk.rows(); row++)
    {
        for (int col = 0; col < _num_particles; col++)
        {
            _particles(row, col) = sqrt(Pk(row, row)) * _particles_distribution(_particles_engine);
        }
    }

    weights_nonnormalized.fill(1.0 / _num_particles);
    weights_normalized.fill(1.0 / _num_particles);

    if (!_particles_init)
        _particles_init = true;


    return;
}
