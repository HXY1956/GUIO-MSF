#include "hwa_ins_proc.h"
#include <Eigen/src/Core/util/DisableStupidWarnings.h>
using namespace hwa_ins;
using namespace hwa_set;
using namespace hwa_base;

ins_obj::ins_obj(const base_quat& qnb0, const Triple& vn0, const Triple& pos0, double t0)
{
    nq = 15;
    _global_variance = Ft = Pk = Matrix::Zero(nq, nq);
    Hk = Matrix::Zero(nr, nq);
    Qt = Pmin = Xk = Vector::Zero(nq);
    G = Matrix::Zero(nq, nq - 3);
    Xf = Matrix::Zero(5, 5);
    Rt = Zk = Vector::Zero(nr);  rts = Vector::Ones(nr);
    Rk = Matrix::Zero(nr, nr);
    Rmax = Vector::Ones(nr) * glv.INF; Rmin = Rb = Vector::Zero(nr); Rbeta = Vector::Ones(nr);
    FBTau = FBMax = Vector::Ones(nq) * glv.INF; FBXk = FBTotal = Vector::Zero(nq);
    Phik = Matrix::Identity(nq, nq);
    qnb = qnb0; vn = vn0; pos = pos0; t = t0;
    eth.Update(pos0, vn0);
    Cnb = hwa_base::base_att_trans::q2mat(qnb); att = hwa_base::base_att_trans::q2att(qnb); Cbn = Cnb.transpose(); vb = Cbn * vn;
    Kg = Ka = Triple::Ones();
    an = eb = db = _tauG = _tauA = _tauGScale = _tauAScale = Triple::Zero(); _tauOdo = 0;
    wib = web = wnb = fb = fn = an = Triple::Zero();
    Ft = Matrix::Zero(nq, nq);
    err_mat();
}

ins_obj::ins_obj(hwa_set::set_base* set)
{
    nq = 15;
    Triple Cart = dynamic_cast<set_ins*>(set)->pos();
    pos = Cart2Geod(Cart, false);
    vn = Cen(pos).transpose() * (dynamic_cast<set_ins*>(set)->vel());
    att = dynamic_cast<set_ins*>(set)->att();
    eth.Update(pos, vn);
    qnb = base_att_trans::a2qua(att);
    Cnb = base_att_trans::q2mat(qnb); Cbn = Cnb.transpose(); vb = Cbn * vn;
    eb = dynamic_cast<set_ins*>(set)->gyro_bias() * glv.dph;
    db = dynamic_cast<set_ins*>(set)->acce_bias() * glv.mg;
    wib = web = wnb = fb = fn = an = Triple::Zero();
    Kg = Ka = Triple::Ones();
    eb = db = _tauG = _tauA = _tauGScale = _tauAScale = Triple::Zero(); _tauOdo = 0;
    Eigen::Vector3d installation_att = dynamic_cast<set_ins*>(set)->imu_installation_rotation();
    qvb = base_att_trans::a2qua(installation_att); Cvb = base_att_trans::q2mat(qvb); Cbv = Cvb.transpose();
    _order = dynamic_cast<set_ins*>(set)->out_order();
    output_freq = dynamic_cast<set_ins*>(set)->out_freq();
    output_intsec = dynamic_cast<set_ins*>(set)->out_intsec();

    _global_variance = Ft = Pk = Matrix::Zero(nq, nq);
    Qt = Pmin = Xk = Vector::Zero(nq);
    G = Matrix::Zero(nq, nq - 3);
    Ft = Matrix::Zero(nq, nq);
    Xf = Matrix::Zero(5, 5);
    Rt = Zk = Vector::Zero(nr);  rts = Vector::Ones(nr);
    Rmax = Vector::Ones(nr) * glv.INF; Rmin = Rb = Vector::Zero(nr); Rbeta = Vector::Ones(nr);
    FBTau = FBMax = Vector::Ones(nq) * glv.INF; FBXk = FBTotal = Vector::Zero(nq);
    Phik = Matrix::Identity(nq, nq);
    err_mat();
}

ins_fuse_mode hwa_ins::str2ins_fuse_mode(std::string s) {
    if (s == "virtual") {
        return VIRTUAL;
    }
    else if (s == "stack") {
        return STACK;
    }
    else {
        return VIRTUAL;
    }
}

void ins_obj::set_posvel(const Triple& pos0, const Triple& vn0)
{
    pos = pos0;
    ell2xyz(pos, pos_ecef, true);
    vn = vn0;
    eth.Update(pos, vn);
}

void ins_obj::set_posvel(const Triple& pos0, const Triple& vn0, const base_quat q)
{
    pos = pos0; 
    ell2xyz(pos, pos_ecef, true);
    vn = vn0; 
    qnb = q; 
    Cnb = base_att_trans::q2mat(qnb);
    eth.Update(pos, vn);
}

void ins_obj::set_tau(const Triple& tauG, const Triple& tauA)
{
    _tauG(0) = tauG(0) > glv.INF / 2 ? 0.0 : 1.0 / tauG(0);
    _tauG(1) = tauG(1) > glv.INF / 2 ? 0.0 : 1.0 / tauG(1);
    _tauG(2) = tauG(2) > glv.INF / 2 ? 0.0 : 1.0 / tauG(2);
    _tauA(0) = tauA(0) > glv.INF / 2 ? 0.0 : 1.0 / tauA(0);
    _tauA(1) = tauA(1) > glv.INF / 2 ? 0.0 : 1.0 / tauA(1);
    _tauA(2) = tauA(2) > glv.INF / 2 ? 0.0 : 1.0 / tauA(2);
}

Triple ins_obj::align_coarse(const Triple& wmm, const Triple& vmm)
{
    double latitude = pos(0);
    double T11, T12, T13, T21, T22, T23, T31, T32, T33;
    double cb = cos(latitude), tb = tan(latitude), nn;
    Triple wbib = wmm / wmm.norm(), fb = vmm / vmm.norm();
    T31 = fb(0), T32 = fb(1), T33 = fb(2);
    T21 = wbib(0) / cb - T31 * tb, T22 = wbib(1) / cb - T32 * tb, T23 = wbib(2) / cb - T33 * tb;
    nn = sqrt(T21 * T21 + T22 * T22 + T23 * T23);  T21 /= nn, T22 /= nn, T23 /= nn;
    T11 = T22 * T33 - T23 * T32, T12 = T23 * T31 - T21 * T33, T13 = T21 * T32 - T22 * T31;
    nn = sqrt(T11 * T11 + T12 * T12 + T13 * T13);  T11 /= nn, T12 /= nn, T13 /= nn;
    SO3 Cnb;
    Cnb << T11, T12, T13, T21, T22, T23, T31, T32, T33;
    return hwa_base::base_att_trans::m2att(Cnb);
}

Triple ins_obj::acc_align(const Triple& wmm)
{
    return Triple();
}

// Virtual IMU Proporgation
void ins_obj::Update(const std::map<int, std::vector<Triple>>& _wm_mimu, const std::map<int, std::vector<Triple>>& _vm_mimu, const std::map<int, ins_scheme>& _scm_mimu)
{
    if (_wm_mimu.size() == 0) {
        std::cerr << " Error Occur Because of No IMU Data" << std::endl;
        return;
    }
    nts = _scm_mimu.at(0).ts;
    nts = abs(nts);
    double nts_2 = nts / 2.0;
    if (_mimu._num_of_imu_axiliary == 0) {
        Update(_wm_mimu.at(0), _vm_mimu.at(0), _scm_mimu.at(0));
        return;
    }
    int imu_number = _mimu._num_of_imu_axiliary + 1;
    Matrix R = Matrix::Zero(imu_number * 3, 3);
    Vector L_g = Vector::Zero(imu_number * 3);
    std::map<int, imu> mimu_data;

    //Project the auxiliary IMU on the virtual IMU
    for (int i = 0; i < imu_number; i++) {
        if (!_scm_mimu.at(i).Status) continue;
        mimu_data[i].Update(_wm_mimu.at(i), _vm_mimu.at(i), _scm_mimu.at(i));
        R.block(i * 3, 0, 3, 3) = _mimu._R_imui_imu0[i].transpose();
        L_g.block(i * 3, 0, 3, 1) = mimu_data[i].phim;
    }
    Triple virtual_wm = (R.transpose() * R).inverse() * R.transpose() * L_g / nts;

    Vector Temp_L_a = Vector::Zero(imu_number * 3);
    for (int i = 0; i < imu_number; i++) {
        if (!_scm_mimu.at(i).Status) continue;
        Temp_L_a.block(i * 3, 0, 3, 1) = mimu_data[i].dvbm / nts - _mimu._R_imui_imu0[i].transpose() * hwa_base::askew(virtual_wm) * hwa_base::askew(virtual_wm) * _mimu._p_imui_imu0[i];
    }

    //Null Space Projection
    Matrix temp_matrix = Matrix::Zero(imu_number * 3, 3);
    for (int i = 0; i < imu_number; i++) {
        if (!_scm_mimu.at(i).Status) continue;
        temp_matrix.block(i * 3, 0, 3, 3) = _mimu._R_imui_imu0[i].transpose() * hwa_base::askew(_mimu._p_imui_imu0[i]);
    }
    //std::cout << "R: " << std::fixed << std::setprecision(6) << std::setw(15) << R << std::endl;
    //std::cout << "L_g: " << std::fixed << std::setprecision(6) << std::setw(15) << L_g << std::endl;
    //std::cout << "L_a: " << std::fixed << std::setprecision(6) << std::setw(15) << Temp_L_a << std::endl;
    //std::cout << "R_T: " << std::fixed << std::setprecision(6) << std::setw(15) << temp_matrix << std::endl;

    Eigen::JacobiSVD<Matrix> svd_helper(temp_matrix, Eigen::ComputeFullU | Eigen::ComputeThinV);
    Matrix A = svd_helper.matrixU().rightCols(
        imu_number * 3 - 3);

    if (temp_matrix == Matrix::Zero(imu_number * 3, 3)) {
        A = Matrix::Identity(imu_number * 3, imu_number * 3);
    }
    Vector L_a = A.transpose() * Temp_L_a;
    Matrix R_a = A.transpose() * R;
    Matrix design_matrix = (R_a.transpose() * R_a).inverse() * R_a.transpose();
    Triple virtual_vm = design_matrix * L_a;
    //std::cout << "total_wm: " << virtual_wm.transpose() << "; total_vm: " << virtual_vm.transpose() << "\n";
    _imu.phim = virtual_wm * nts;
    _imu.dvbm = virtual_vm * nts;
    obs_wib = _imu.phim / nts;
    obs_fb = _imu.dvbm / nts;
    _imu.phim = Kg.asDiagonal() * _imu.phim - eb * nts; _imu.dvbm = Ka.asDiagonal() * _imu.dvbm - db * nts;
    Triple vn1_2 = vn + an * nts_2, pos1_2 = pos + eth.v2dp(vn1_2, nts_2);
    eth.Update(pos1_2, vn1_2);
    wib = _imu.phim / nts;
    fb = _imu.dvbm / nts;
    web = wib - Cbn * eth.wnie;
    wnb = wib - base_quat::conj(qnb * base_att_trans::rv2q(_imu.phim / 2)) * eth.wnin;
    fn = qnb * fb;
    an = base_att_trans::rv2q(-eth.wnin * nts_2) * fn + eth.gcc;
    gcc_b = -Cbn * eth.gcc;
    Triple vn1 = vn + an * nts;
    pos = pos + eth.v2dp(vn + vn1, nts_2);    vn = vn1;
    qnb = base_att_trans::rv2q(-eth.wnin * nts) * qnb * base_att_trans::rv2q(_imu.phim);
    Cnb = base_att_trans::q2mat(qnb); att = base_att_trans::m2att(Cnb); Cbn = Cnb.transpose(); vb = Cbn * vn;
    eth.Update(pos, vn); pos_ecef = Geod2Cart(pos, false); 
    std::cout<<t << " att: " << att.transpose() << "; Vn: " << vn.transpose() << " POS: " << std::fixed << std::setprecision(6) << pos_ecef.transpose() << "\n";
    Ceb = eth.Cen * Cnb; Cbe = Ceb.transpose(); qeb = base_att_trans::m2qua(Ceb); ve = eth.Cen * vn; ae = eth.Cen * an;
    pure_ins_time += nts; qbe = base_att_trans::m2qua(Cbe);

    orientation = Ceb.transpose();
    velocity = ve;
    position = Geod2Cart(pos, false);

    Xf.block(0, 0, 3, 3) = Ceb;
    Xf.block(0, 3, 3, 1) = ve;
    Xf.block(0, 4, 3, 1) = pos_ecef - initial_pos;

    // Caculating jacobian_v_bg
    Matrix temp_matrix1 = Matrix::Zero(imu_number * 3, 3);
    for (int i = 0; i < imu_number; i++) {
        temp_matrix1.block(i * 3, 0, 3, 3) = _mimu._R_imui_imu0[i].transpose() * askew(virtual_wm) * askew(_mimu._p_imui_imu0[i]) + askew(askew(virtual_wm) * _mimu._p_imui_imu0[i]);

    }
    jacobian_v_bg = Ceb * design_matrix * A.transpose() * temp_matrix1;
    jacobian_v_etag = jacobian_v_bg;
}

void ins_obj::Update(const std::vector<Triple>& wm, const std::vector<Triple>& vm, const ins_scheme& scm)
{
    nts = scm.ts;
    t = scm.t;
    nts = abs(nts);
    double nts_2 = nts / 2.0;
    _imu.Update(wm, vm, scm);
    obs_wib = _imu.phim / nts;
    obs_fb = _imu.dvbm / nts;
    _imu.phim = Kg.asDiagonal() * _imu.phim - eb * nts; _imu.dvbm = Ka.asDiagonal() * _imu.dvbm - db * nts;
    Triple vn1_2 = vn + an * nts_2, pos1_2 = pos + eth.v2dp(vn1_2, nts_2);
    eth.Update(pos1_2, vn1_2);
    wib = _imu.phim / nts;
    fb = _imu.dvbm / nts;
    web = wib - Cbn * eth.wnie;
    wnb = wib - base_quat::conj(qnb * hwa_base::base_att_trans::rv2q(_imu.phim / 2)) * eth.wnin;
    //wnb = wib - (qnb * hwa_base::base_att_trans::rv2q(imu.phim / 2)) * eth.wnin;
    fn = qnb * fb;
    an = hwa_base::base_att_trans::rv2q(-eth.wnin * nts_2) * fn + eth.gcc;
    gcc_b = -Cbn * eth.gcc;
    // an = hwa_base::base_att_trans::rv2q(-eth.wnin * nts) * fn + eth.gcc;

    Triple vn1 = vn + an * nts;
    pos = pos + eth.v2dp(vn + vn1, nts_2);    vn = vn1;
    qnb = hwa_base::base_att_trans::rv2q(-eth.wnin * nts) * qnb * hwa_base::base_att_trans::rv2q(_imu.phim);
    Cnb = hwa_base::base_att_trans::q2mat(qnb); att = hwa_base::base_att_trans::m2att(Cnb); Cbn = Cnb.transpose(); vb = Cbn * vn;
    eth.Update(pos, vn); pos_ecef = Geod2Cart(pos, false);
    Ceb = eth.Cen * Cnb; Cbe = Ceb.transpose(); qeb = hwa_base::base_att_trans::m2qua(Ceb); ve = eth.Cen * vn; ae = eth.Cen * an;
    pure_ins_time += nts; qbe = hwa_base::base_att_trans::m2qua(Cbe);

    orientation = Ceb.transpose();
    velocity = ve;
    position = Geod2Cart(pos, false);

    Xf.block(0, 0, 3, 3) = Ceb;
    Xf.block(0, 3, 3, 1) = ve;
    Xf.block(0, 4, 3, 1) = pos_ecef - initial_pos;

    //std::cout << t <<" "<<std::setiosflags(ios::fixed) << std::setprecision(4) << pos_ecef.transpose() << std::endl;
}

void ins_obj::preintergration(std::vector<IMU_MSG> imu_msg, Triple& pos, Triple& vel, base_quat& q) {
    double ts = 0.005;
    for (auto iter = imu_msg.begin(); iter < imu_msg.end(); iter++) {
        Triple phim = Kg.asDiagonal() * iter->wm - eb * ts;
        Triple dvbm = Ka.asDiagonal() * iter->vm - db * ts; dvbm /= ts;
        if (iter + 1 < imu_msg.end()) {
            Triple un_dvbm = Ka.asDiagonal() * (iter + 1)->vm - db * ts; un_dvbm /= ts;
            Triple mi_dvbm = (dvbm + un_dvbm) / 2;
            q = q * hwa_base::base_att_trans::rv2q(phim);
            vel += q * mi_dvbm * ts;
            pos += q * mi_dvbm * ts * ts / 2;
            continue;
        }
        q = q * hwa_base::base_att_trans::rv2q(phim);
        vel += q * dvbm * ts;
        pos += q * dvbm * ts * ts / 2;
    }
}

void ins_obj::get_gcc(Triple gcc) {
    eth.gcc = gcc;
}

void ins_obj::Update_ipn(const std::vector<Triple>& wm, const std::vector<Triple>& vm, const ins_scheme& scm, const Triple& gravity)
{
    // int n = wm.size();
    nts = scm.ts;
    t = scm.t;
    nts = abs(nts);
    double nts_2 = nts / 2.0;
    _imu.Update(wm, vm, scm);
    _imu.phim = Kg.asDiagonal() * _imu.phim - eb * nts; _imu.dvbm = Ka.asDiagonal() * _imu.dvbm - db * nts;
    // Triple vn1_2 = vn + an * nts_2, pos1_2 = pos + vn1_2 * nts_2;
    wib = _imu.phim / nts; fb = _imu.dvbm / nts;
    fn = qnb * fb;

    an = fn + gravity;
    Triple vn1 = vn + an * nts;
    pos = pos + (vn + vn1) * nts_2;    vn = vn1;
    qnb = qnb * hwa_base::base_att_trans::rv2q(_imu.phim);
    Cnb = hwa_base::base_att_trans::q2mat(qnb);
    att = hwa_base::base_att_trans::m2att(Cnb); Cbn = Cnb.transpose(); vb = Cbn * vn;
    pure_ins_time += nts;
}

void ins_obj::err_mat()
{
    double tb = eth.tb, secb = 1.0 / eth.cb, secb2 = secb * secb, scb = eth.sb * eth.cb,
        wN = eth.wnie(1), wU = eth.wnie(2), vE = vn(0), vN = vn(1);
    double f_RMh = eth.f_RMh, f_RNh = eth.f_RNh, f_cbRNh = eth.f_cbRNh,
        f_RMh2 = f_RMh * f_RMh, f_RNh2 = f_RNh * f_RNh;
    SO3 F1, F2, F3;
    F1 << 0, 0, 0, -wU, 0, 0, wN, 0, 0;
    F2 << 0, 0, vN* f_RMh2, 0, 0, -vE * f_RNh2, vE* secb2* f_RNh, 0, -vE * tb * f_RNh2;
    F3 << 0, 0, 0, 0, 0, 0, -glv.g0 * (5.27094e-3 * 2 * scb + 2.32718e-5 * 4 * eth.sb2 * scb), 0, 3.086e-6;
    Faa = hwa_base::askew(-eth.wnin);
    Fav << 0, -f_RMh, 0, f_RNh, 0, 0, tb* f_RNh, 0, 0;
    Fap = F1 + F2;
    Fva = hwa_base::askew(fn);
    Fvv = hwa_base::askew(vn) * Fav - hwa_base::askew(eth.wnie + eth.wnin);
    Fvp = hwa_base::askew(vn) * (2 * F1 + F2) + F3;
    Fpv << 0, f_RMh, 0, f_cbRNh, 0, 0, 0, 0, 1;
    Fpp << 0, 0, -vN * f_RMh2, vE* tb* f_cbRNh, 0, -vE * secb * f_RNh2, 0, 0, 0;
    //Fng << 0, eth.RMh, 0, eth.cbRNh, 0, 0, 0, 0, 1;
}

void ins_obj::prt_header(std::ostringstream& os)
{
    // the first line
    os << "# ";
    os << std::setw(15) << "Seconds of Week";
    switch (_order)
    {
    case XYZ_XYZ_PRY_NW:
        os << std::setw(18) << "X-ECEF" <<
            std::setw(18) << "Y-ECEF" <<
            std::setw(18) << "Z-ECEF";
        os << std::setw(10) << "VX" <<
            std::setw(10) << "VY" <<
            std::setw(10) << "VZ";
        os << std::setw(10) << "Pitch" <<
            std::setw(10) << "Roll" <<
            std::setw(10) << "Yaw";
        break;

    case BLH_ENU_PRY_NW:
        os << std::setw(18) << "Latitude" <<
            std::setw(18) << "Longitude" <<
            std::setw(18) << "Altitude";
        os << std::setw(10) << "Ve" <<
            std::setw(10) << "Vn" <<
            std::setw(10) << "Vu";
        os << std::setw(10) << "Pitch" <<
            std::setw(10) << "Roll" <<
            std::setw(10) << "Yaw";
        break;

    case HOLO_ODO:
        os << std::setw(18) << "Longitude" <<
            std::setw(18) << "Latitude" <<
            std::setw(18) << "Altitude";
        os << std::setw(10) << "Pitch" <<
            std::setw(10) << "Roll" <<
            std::setw(10) << "Yaw";
        os << std::setw(10) << "Ve" <<
            std::setw(10) << "Vn" <<
            std::setw(10) << "Vu";
        break;

    default:
        break;
    }
    os << std::setw(15) << "GyroBiasX" <<
        std::setw(15) << "GyroBiasY" <<
        std::setw(15) << "GyroBiasZ" <<
        std::setw(15) << "AcceBiasX" <<
        std::setw(15) << "AcceBiasY" <<
        std::setw(15) << "AcceBiasZ";
    os << std::setw(12) << "MeasType" <<
        std::setw(7) << "Nsat" <<
        std::setw(7) << "Nuwb" <<
        std::setw(7) << "PDOP" <<
        std::setw(12) << "AmbStatus";
    os << "         " << std::endl;

    os << "# ";
    os << std::setw(15) << "(s)";
    switch (_order)
    {
    case XYZ_XYZ_PRY_NW:
        os << std::setw(18) << "(m)" <<
            std::setw(18) << "(m)" <<
            std::setw(18) << "(m)";
        os << std::setw(10) << "(m/s)" <<
            std::setw(10) << "(m/s)" <<
            std::setw(10) << "(m/s)";
        os << std::setw(10) << "(deg)" <<
            std::setw(10) << "(deg)" <<
            std::setw(10) << "(deg)";
        break;

    case BLH_ENU_PRY_NW:
        os << std::setw(18) << "(deg)" <<
            std::setw(18) << "(deg)" <<
            std::setw(18) << "(m)";
        os << std::setw(10) << "(m/s)" <<
            std::setw(10) << "(m/s)" <<
            std::setw(10) << "(m/s)";
        os << std::setw(10) << "(deg)" <<
            std::setw(10) << "(deg)" <<
            std::setw(10) << "(deg)";
        break;

    case HOLO_ODO:
        os << std::setw(18) << "(deg)" <<
            std::setw(18) << "(deg)" <<
            std::setw(18) << "(m)";
        os << std::setw(10) << "(deg)" <<
            std::setw(10) << "(deg)" <<
            std::setw(10) << "(deg)";
        os << std::setw(10) << "(m/s)" <<
            std::setw(10) << "(m/s)" <<
            std::setw(10) << "(m/s)";
        break;
    default:
        break;
    }
    os << std::setw(15) << "(deg/h)";
    os << std::setw(15) << "(deg/h)";
    os << std::setw(15) << "(deg/h)";
    os << std::setw(15) << "(mg)";
    os << std::setw(15) << "(mg)";
    os << std::setw(15) << "(mg)";
    os << std::setw(12) << " " <<
        std::setw(7) << "#" <<
        std::setw(7) << "#" <<
        std::setw(7) << "#" <<
        std::setw(12) << " ";
    os << std::endl;

}

void ins_obj::prt_sins(std::ostringstream& os)
{
    Triple att1 = att / glv.deg;
    Triple eb1 = eb / glv.deg * glv.hur;
    Triple db1 = db / glv.mg;
    // t_gbase_quat base_quat1 = hwa_base::base_att_trans::a2qua(hwa_base::base_att_trans::m2att(hwa_base::Cen(pos)*hwa_base::base_att_trans::a2mat(att)));
    Triple Car_pos = Geod2Cart(pos, false);
    Triple Geo_pos = Triple(pos(0) / glv.deg, pos(1) / glv.deg, pos(2));
    Triple Ve = hwa_base::Cen(pos) * (vn);
    // Triple Vb = Cbn * vn;

    Triple pos_out, vel_out, att_out;

    switch (_order)
    {
        /// the att_out is positive if defination is from north to west[-Pi,Pi]
    case XYZ_XYZ_PRY_NW:
        pos_out = Car_pos;
        vel_out = Ve;
        att_out = att1;
        att_out(2) = -att_out(2);
        break;

        /// the att_out is positive if defination is from north to west[-Pi,Pi]
    case BLH_ENU_PRY_NW:
        pos_out = Geo_pos;
        vel_out = vn;
        att_out = att1;
        att_out(2) = -att_out(2);
        break;

        /// the att_out is positive if defination is from north to west[-Pi,Pi]
        /// HOLO format is latitude,longitude,altitude,Roll,Pitch,Yaw,Ve,Vn,Vu
    case HOLO_ODO:
        pos_out = Triple(Geo_pos(0), Geo_pos(1), Geo_pos(2));
        // exchange vel_out and att_out for HOLO format
        vel_out = att1;
        att_out = vn;
        vel_out(2) = -vel_out(2);
        break;

    default:
        break;
    }

    os << std::fixed << std::setprecision(4) << std::setw(18) << static_cast<double>(int(t));
    os << std::fixed << std::setprecision(3) <<
        std::setw(18) << pos_out(0) <<
        std::setw(18) << pos_out(1) <<
        std::setw(18) << pos_out(2);
    os << std::fixed << std::setprecision(3) <<
        std::setw(10) << vel_out(0) <<
        std::setw(10) << vel_out(1) <<
        std::setw(10) << vel_out(2);
    os << std::fixed << std::setprecision(4) <<
        std::setw(10) << att_out(0) <<
        std::setw(10) << att_out(1) <<
        std::setw(10) << att_out(2);
    os << std::fixed << std::setprecision(4) <<
        std::setw(15) << eb1(0) <<
        std::setw(15) << eb1(1) <<
        std::setw(15) << eb1(2);
    os << std::fixed << std::setprecision(4) <<
        std::setw(15) << db1(0) <<
        std::setw(15) << db1(1) <<
        std::setw(15) << db1(2);
    //os << std::fixed << std::setprecision(8) <<
    //    std::setw(15) << base_quat1.q0 <<
    //    std::setw(15) << base_quat1.q1 <<
    //    std::setw(15) << base_quat1.q2 <<
    //    std::setw(15) << base_quat1.q3;
    // os << std::endl;
}

void ins_obj::prt_sins_ipn(std::ostringstream& os)
{
    Triple att1 = att / glv.deg;
    Triple eb1 = eb / glv.deg * glv.hur;
    Triple db1 = db / glv.mg;

    Triple Geo_pos = Triple(pos(0) / glv.deg, pos(1) / glv.deg, pos(2));
    Triple pos_out, vel_out, att_out;
    switch (_order)
    {
    case XYZ_XYZ_PRY_NW:
        pos_out = pos;
        vel_out = vn;
        att_out = att1;
        att_out(2) = -att_out(2);
        break;

    case BLH_ENU_PRY_NW:
        pos_out = Geo_pos;
        vel_out = vn;
        att_out = att1;
        att_out(2) = -att_out(2);
        break;

    case HOLO_ODO:
        pos_out = Triple(Geo_pos(0), Geo_pos(1), Geo_pos(2));
        // exchange vel_out and att_out for HOLO format
        vel_out = att1;
        att_out = vn;
        vel_out(2) = -vel_out(2);
        break;

    default:
        break;
    }

    os << std::fixed << std::setprecision(6) << std::setw(18) << t;
    os << std::fixed << std::setprecision(3) <<
        std::setw(18) << pos_out(0) <<
        std::setw(18) << pos_out(1) <<
        std::setw(18) << pos_out(2);
    os << std::fixed << std::setprecision(3) <<
        std::setw(10) << vel_out(0) <<
        std::setw(10) << vel_out(1) <<
        std::setw(10) << vel_out(2);
    os << std::fixed << std::setprecision(4) <<
        std::setw(10) << att_out(0) <<
        std::setw(10) << att_out(1) <<
        std::setw(10) << att_out(2);
    os << std::fixed << std::setprecision(4) <<
        std::setw(15) << eb1(0) <<
        std::setw(15) << eb1(1) <<
        std::setw(15) << eb1(2);
    os << std::fixed << std::setprecision(4) <<
        std::setw(15) << db1(0) <<
        std::setw(15) << db1(1) <<
        std::setw(15) << db1(2);
}

void ins_obj::int_sec(bool _beg_end)
{
    double out_nts = 1.0 / output_freq;
    double dt = fmod(t, out_nts);
    auto p = vec_qnb.begin();
    base_quat q1; //q1 = vec_qnb[0];
    q1 = p->second;  p++;
    base_quat q2; //q2 = vec_qnb[1];
    q2 = p->second;
    q1.fabs(q1); q2.fabs(q2);
    if (_beg_end)
    {
        q1 = q1 * (dt / nts);
        q2 = q2 * ((nts - dt) / nts);
        t = t - dt;
    }
    else
    {
        dt = out_nts - dt;
        q1 = q1 * ((nts - dt) / nts);
        q2 = q2 * (dt / nts);
        t = t + dt;
    }
    Triple vn1 = vn - an * dt;
    /*std::cerr << "\n" << "t " << std::fixed << std::setprecision(4) << t
        << std::fixed << std::setprecision(6) << " an " << an.transpose()
        << std::fixed << std::setprecision(6) << " an*dt " << (an * dt).transpose();*/
        /*std::cerr<<"\n"<<"vn "<< std::fixed << std::setprecision(4) <<vn.transpose()<< " vn1 "<< vn1.transpose()
            <<" dt "<<dt<<" dpos " << std::fixed << std::setprecision(6) << eth.v2dp(vn + vn1, dt / 2).transpose()<<std::endl;*/
    pos = pos - eth.v2dp(vn + vn1, dt / 2);    vn = vn1;
    //qnb = hwa_base::base_att_trans::rv2q(-eth.wnin*nts)*qnb*hwa_base::base_att_trans::rv2q(imu.phim);

    qnb = q1 + q2;
    qnb.normlize(qnb);
    Cnb = hwa_base::base_att_trans::q2mat(qnb); att = hwa_base::base_att_trans::m2att(Cnb); Cbn = Cnb.transpose(); vb = Cbn * vn;
    /*std::cerr << "pos0 " << std::fixed << std::setprecision(8) << pos_ecef.transpose() << std::endl;*/
    eth.Update(pos, vn); pos_ecef = Geod2Cart(pos, false);
    /*std::cerr << "pos " << std::fixed << std::setprecision(8) << pos_ecef.transpose() << std::endl;*/
    Ceb = eth.Cen * Cnb; Cbe = Ceb.transpose(); qeb = hwa_base::base_att_trans::m2qua(Ceb); ve = eth.Cen * vn; ae = eth.Cen * an;
}

void ins_obj::_debug_ins_info()
{
    std::cout << "ins info : \n"
        << "     time: " << std::setw(16) << t << "\n"
        << "     att: " << std::setw(10) << att.transpose() / glv.deg << "\n"
        << "     vel: " << std::setw(10) << ve.transpose() << "\n"
        << "     pos: " << std::setw(10) << pos_ecef.transpose() << "\n"
        << "     an: " << std::setw(10) << an.transpose() << "\n"
        << "     ae: " << std::setw(10) << ae.transpose() << "\n"
        << "     web: " << std::setw(10) << web.transpose() / glv.dps << "\n"
        << "     wnb: " << std::setw(10) << wnb.transpose() / glv.dps << "\n"
        << std::endl;
}

imu::imu()
{
    phim = dvbm = wm_1 = vm_1 = Triple::Zero();
    pf1 = pf2 = 1;
}

Triple imu::phi_cone_crt(const std::vector<Triple>& wm)
{
    Triple dphim;
    int n = wm.size();
    if (n == 1)
        dphim = Triple::Zero();
    else if (n == 2)
        dphim = 2. / 3 * wm[0].cross(wm[1]);
    else if (n == 3)
        dphim = 27. / 40 * wm[1].cross(wm[2]) + 9. / 20 * wm[0].cross(wm[2]) + 27. / 40 * wm[0].cross(wm[1]);
    else if (n == 4)
        dphim = 232. / 315 * wm[2].cross(wm[3]) + 46. / 105 * wm[1].cross(wm[3]) +
        18. / 35 * wm[0].cross(wm[3]) + 178. / 315 * wm[1].cross(wm[2]) +
        46. / 105 * wm[0].cross(wm[2]) + 232. / 315 * wm[0].cross(wm[1]);
    else if (n == 5)
        dphim = 18575. / 24192 * wm[3].cross(wm[4]) + 2675. / 6048 * wm[2].cross(wm[4]) +
        11225. / 24192 * wm[1].cross(wm[4]) + 125. / 252 * wm[0].cross(wm[4]) +
        2575. / 6048 * wm[2].cross(wm[3]) + 425. / 672 * wm[1].cross(wm[3]) +
        13975. / 24192 * wm[0].cross(wm[3]) + 1975. / 3024 * wm[1].cross(wm[2]) +
        325. / 1512 * wm[0].cross(wm[2]) + 21325. / 24192 * wm[0].cross(wm[1]);
    else
        dphim = Triple::Zero();
    return dphim;
}

Triple imu::phi_poly_crt(const std::vector<Triple>& wm)
{
    Triple dphim;
    int n = wm.size();
    if (n == 1)
    {
        // One Plus Previous
        if (pf1 == 1) { wm_1 = wm[0]; pf1 = 0; }
        dphim = 1. / 12 * wm_1.cross(wm[0]);
    }
    else if (n == 2)
        dphim = 2. / 3 * wm[0].cross(wm[1]);
    else if (n == 3)
        dphim = 33. / 80 * wm[0].cross(wm[2]) + 57. / 80 * wm[1].cross(wm[2] - wm[0]);
    else if (n == 4)
        dphim = 736. / 945 * (wm[0].cross(wm[1]) + wm[2].cross(wm[3])) +
        334. / 945 * (wm[0].cross(wm[2]) + wm[1].cross(wm[3])) +
        526. / 945 * wm[0].cross(wm[3]) + 654. / 945 * wm[1].cross(wm[2]);
    else if (n == 5)
        dphim = 123425. / 145152 * (wm[0].cross(wm[1]) + wm[3].cross(wm[4])) +
        34875. / 145152 * (wm[0].cross(wm[2]) + wm[2].cross(wm[4])) +
        90075. / 145152 * (wm[0].cross(wm[3] + wm[1].cross(wm[4]))) +
        66625. / 145152 * wm[0].cross(wm[4]) +
        103950. / 145152 * (wm[1].cross(wm[2]) + wm[2].cross(wm[3])) +
        55400. / 145152 * (wm[1].cross(wm[3]));
    else
        dphim = Triple::Zero();
    return dphim;
}

Triple imu::vm_cone_crt(const std::vector<Triple>& wm, const std::vector<Triple>& vm)
{
    Triple scullm;
    int n = wm.size();
    if (n == 1)
        scullm = Triple::Zero();
    else if (n == 2)
        scullm = 2. / 3 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    else if (n == 3)
        scullm = 27. / 40 * (wm[1].cross(vm[2]) + vm[1].cross(wm[2])) +
        9. / 20 * (wm[0].cross(vm[2]) + vm[0].cross(wm[2])) +
        27. / 40 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    else if (n == 4)
        scullm = 232. / 315 * (wm[2].cross(vm[3]) + vm[2].cross(wm[3])) +
        46. / 105 * (wm[1].cross(vm[3]) + vm[1].cross(wm[3])) +
        18. / 35 * (wm[0].cross(vm[3]) + vm[0].cross(wm[3])) +
        178. / 315 * (wm[1].cross(vm[2]) + vm[1].cross(wm[2])) +
        46. / 105 * (wm[0].cross(vm[2]) + vm[0].cross(wm[2])) +
        232. / 315 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    return scullm;
}


Triple imu::vm_poly_crt(const std::vector<Triple>& wm, const std::vector<Triple>& vm)
{
    Triple scullm;
    int n = wm.size();
    if (n == 1)
    {
        // One Plus Previous
        if (pf2 == 1) { wm_1 = wm[0]; vm_1 = vm[0]; pf2 = 0; }
        scullm = 1. / 12 * (wm_1.cross(vm[0]) + vm_1.cross(wm[0]));
        wm_1 = wm[0]; vm_1 = vm[0];
    }
    else if (n == 2)
        scullm = 2. / 3 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]));
    else if (n == 3)
        scullm = 33. / 80 * (wm[0].cross(vm[2]) + vm[0].cross(wm[2])) +
        57. / 80 * (wm[0].cross(vm[1]) + vm[0].cross(wm[1]) + wm[1].cross(vm[2] + vm[1].cross(wm[2])));
    else if (n == 4)
        scullm = 736. / 945 * (wm[0].cross(vm[1]) + wm[2].cross(vm[3]) + vm[0].cross(wm[1]) + vm[2].cross(wm[3])) +
        334. / 945 * (wm[0].cross(vm[2]) + wm[1].cross(vm[3]) + vm[0].cross(wm[2]) + vm[1].cross(wm[3])) +
        526. / 945 * (wm[0].cross(vm[3]) + vm[0].cross(wm[3])) +
        654. / 945 * (wm[1].cross(vm[2]) + vm[1].cross(wm[2]));
    else
        scullm = Triple::Zero();
    return scullm;

}

void imu::Update(const std::vector<Triple>& wm, const std::vector<Triple>& vm, const ins_scheme& scm)
{
    Triple dphim, scullm, wmm = Triple::Zero(), vmm = Triple::Zero();
    for (size_t i = 0; i < wm.size(); i++)
    {
        wmm += wm[i];
        vmm += vm[i];
    }

    if (scm.Cps == 0)
    {
        dphim = phi_cone_crt(wm);
        scullm = vm_cone_crt(wm, vm);
    }
    else if (scm.Cps == 2)
    {
        dphim = phi_poly_crt(wm);
        scullm = vm_poly_crt(wm, vm);
    }
    Triple rotm = 1. / 2 * wmm.cross(vmm);
    phim = wmm + dphim;
    dvbm = vmm + rotm + scullm;
}