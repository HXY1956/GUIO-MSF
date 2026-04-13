#include "hwa_uwb_proc.h"
#include "hwa_set_uwb.h"
#include "hwa_set_ign.h"
#include <corecrt_io.h>

using namespace hwa_uwb;
using namespace hwa_set;

hwa_uwb::uwb_proc::uwb_proc(set_base* gset, std::string mark, base_data* data):
    _is_first(true),
    _intv(0.0),
    _site(mark),
    uwbdata(dynamic_cast<uwb_data*>(data))
{
    _iter = dynamic_cast<set_uwb*>(gset)->iter();
    _dim = dynamic_cast<set_uwb*>(gset)->nq();
    Ft_uwb = Pk_uwb = Pk_Sav_uwb = Matrix::Zero(_dim, _dim);
    Hk_uwb = Matrix::Zero(_dim, _dim);
    Qt_uwb = Pmin_uwb = Xk_uwb = Vector::Zero(_dim);  Pmax_uwb = Vector::Ones(_dim) * glv.INF;
    Rt_uwb = Zk_uwb = Vector::Zero(_dim);  rts_uwb = Vector::Ones(_dim);
    Rk_uwb = Matrix::Zero(_dim, _dim);
    Rmax_uwb = Vector::Ones(_dim) * glv.INF; Rmin_uwb = Rb_uwb = Vector::Zero(_dim); Rbeta_uwb = Vector::Ones(70);
    FBTau_uwb = FBMax_uwb = Vector::Ones(_dim) * glv.INF; FBXk_uwb = FBTotal_uwb = Vector::Zero(_dim);
    _wgt_type = dynamic_cast<set_uwb*>(gset)->wgt_type();
    _ts = dynamic_cast<set_uwb*>(gset)->ts();
    _sample = dynamic_cast<set_uwb*>(gset)->sample();
    _pos = dynamic_cast<set_uwb*>(gset)->pos();
    _initial_pos_std = dynamic_cast<set_uwb*>(gset)->initial_pos_std();
    _pos_psd = dynamic_cast<set_uwb*>(gset)->pos_psd();
    Pk_uwb = _initial_pos_std.array().abs2().array().matrix().asDiagonal();
    _init_Pk = Pk_uwb; Pk_Sav_uwb = Pk_uwb;
    Qt_uwb = _pos_psd.array().abs2();
    _anchor_list = dynamic_cast<set_uwb*>(gset)->anchor_list();
    std::string tmp;
    tmp = dynamic_cast<set_uwb*>(gset)->result_file();
    _fuwb = new base_iof(tmp);
    _fuwb->mask(tmp);
    output_res = dynamic_cast<set_uwb*>(gset)->output_res();
    if (output_res)
    {
        tmp = dynamic_cast<set_uwb*>(gset)->res_file();
        _fuwbres = new base_iof(tmp);
        _fuwbres->mask(tmp);
    }
    _print_head();
    nq = _dim;
    _valid_node_num = 0;
    filter = dynamic_cast<set_uwb*>(gset)->filter();
    smooth = dynamic_cast<set_uwb*>(gset)->smooth();
    smooth_point = dynamic_cast<set_uwb*>(gset)->smooth_point();
    meas_range_std = dynamic_cast<set_uwb*>(gset)->meas_range_std();
    best_range_std = dynamic_cast<set_uwb*>(gset)->best_range_std();
    barrior = dynamic_cast<set_uwb*>(gset)->barrior();
    kappa_sig = dynamic_cast<set_uwb*>(gset)->kappa_sig();
    alpha_sig = dynamic_cast<set_uwb*>(gset)->alpha_sig();
    log_parameter = dynamic_cast<set_uwb*>(gset)->log_parameter();
    exp_parameter = dynamic_cast<set_uwb*>(gset)->exp_parameter();
    sigmoid_parameter = dynamic_cast<set_uwb*>(gset)->sigmoid_parameter();
    sigmoid_threshold = dynamic_cast<set_uwb*>(gset)->sigmoid_threshold();
    tolerance = dynamic_cast<set_uwb*>(gset)->tolerance();
    penal = str2Penal(dynamic_cast<set_uwb*>(gset)->penal());
    SDP = dynamic_cast<set_uwb*>(gset)->SDP();
    interpolation = dynamic_cast<set_uwb*>(gset)->interpolation();
    interpolation_noise = dynamic_cast<set_uwb*>(gset)->interpolation_noise();
    gama = dynamic_cast<set_uwb*>(gset)->gama();
    alpha = dynamic_cast<set_uwb*>(gset)->alpha();
    pred_constraint = dynamic_cast<set_uwb*>(gset)->pred_constraint();
    range_lim = dynamic_cast<set_uwb*>(gset)->range_lim();
    snr_lim = dynamic_cast<set_uwb*>(gset)->snr_lim();
    posterior = dynamic_cast<set_uwb*>(gset)->posterior();
    max_res_norm = dynamic_cast<set_uwb*>(gset)->max_res_norm();
    g0 = dynamic_cast<set_uwb*>(gset)->G0();
    e0 = dynamic_cast<set_uwb*>(gset)->E0();
    dof1 = dynamic_cast<set_uwb*>(gset)->dof1();
    dof2 = dynamic_cast<set_uwb*>(gset)->dof2();
    max_iter = dynamic_cast<set_uwb*>(gset)->max_iter();
    tau = dynamic_cast<set_uwb*>(gset)->Tau();
    proc_noise = dynamic_cast<set_uwb*>(gset)->proc_noise();
    proc_noise *= proc_noise;
    make_dir("output\\initial_range.txt");
    make_dir("output\\SDP.txt");
    std::ofstream("output\\initial_range.txt", std::ios::out | std::ios::trunc);
    std::ofstream("output\\SDP.txt", std::ios::out | std::ios::trunc);
    _outfile["initial_range"].open("output\\initial_range.txt", std::ios::app);
    _outfile["SDP"].open("output\\SDP.txt", std::ios::app);
}

hwa_uwb::uwb_proc::~uwb_proc()
{
    if (_fuwb != nullptr)delete _fuwb; _fuwb = nullptr;
    if (_fuwbres != nullptr)delete _fuwbres; _fuwbres = nullptr;
    _outfile["initial_range"].close();
    _outfile["SDP"].close();
}

PenalType hwa_uwb::str2Penal(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return std::toupper(c); });
    if (s == "SIGMOID") return SIGMOID;
    if (s == "EXP") return EXP;
    if (s == "LOG") return LOG;
    if (s == "LINEAR") return LINEAR;
}

int hwa_uwb::uwb_proc::ProcessOneEpoch(const base_time& now)
{
    _preprocess();
    time_update(_get_ts());
    set_particles();
    if (obs_update(now) == NO_MEAS)
        return -1;
    _write();
    return 0;
}

int hwa_uwb::uwb_proc::ProcessBatch(const base_time& beg, const base_time& end)
{
    int sign = 1;

    std::string flag = "(forward direction) ";
    bool prtOut = true;
    _now_uwb = beg;

    std::cerr << "Start UWB Processing Filtering: " << _now_uwb.str_ymdhms() << " " << end.str_ymdhms() << std::endl;
    bool time_loop = true;
    while (time_loop)
    {
        if (_now_uwb >= end)
        {
            time_loop = false; break;
        }

        ProcessOneEpoch(_now_uwb);
        if (_ts < 1e-5)
            _now_uwb = _now_uwb + _intv;
        else
            _now_uwb = _now_uwb + _ts;
    }
    return 0;
}

void hwa_uwb::uwb_proc::time_update(double kfts)
{
    Matrix Fk = Matrix::Identity(_dim, _dim);
    Phik_uwb = Fk * Phik_uwb;
    Xk_uwb = Fk * Xk_uwb;
    Matrix Qk = (Qt_uwb * kfts).array().matrix().asDiagonal();
    Pk_uwb = Fk * Pk_uwb * (Fk.transpose());
    Pk_uwb.block(0, 0, _dim, _dim) += Qk;
    return;
}

int hwa_uwb::uwb_proc::obs_update(const base_time& now)
{
    Vector Pxz = Vector::Zero(_dim);
    Vector Kk = Vector::Zero(_dim);
    Vector Hi = Vector::Zero(_dim);
    Vector Xki = Vector::Zero(_dim);

    SO3 Qenu = SO3::Zero();
    Triple Geo_pos = Triple::Zero();
    SO3 Cne = SO3::Zero();
    int valid_num = 0;

    Vector v_norm = Vector::Zero(uwb_epo.size());
    double vtpv = 0.0;
    int outlier = -1;
    Matrix Pk_Sav = Pk_uwb;
    Matrix T0 = tau * Pk_Sav;

    Rk_uwb = Matrix::Zero(uwb_epo.size(), uwb_epo.size());
    bool isfirst = true;
    nr = uwb_epo.size();

    int dim = Xk_uwb.size();
    Vector Delx = Vector::Zero(dim);
    Triple pos_store = this->_pos;
    Matrix P_store = Pk_uwb;
    Vector Xk_store = Xk_uwb;
    Matrix rm;
    Xk_store(0) -= 1;
    int iter = 0;

    for (unsigned int iter = 0; iter < _iter; iter++)
    {
        Xk_store = Xk_uwb;
        if (isfirst && iter > 0)
            isfirst = false;
        valid_num = hwa_uwb::uwb_proc::_setMeas(isfirst);
      
        if (outlier >= 0)
        {
            Rk_uwb(outlier, outlier) = Rk_uwb(outlier, outlier) * barrior;
            outlier = -1;
        }

        Matrix Kkm = _particles;

        if (valid_num < 3)
            return NO_MEAS;

        if (filter == "EKF" || filter == "UKF" || filter == "GSTM" || filter == "GSTM-1" || filter == "GSTM-2")
        {
            Matrix Him = Hk_uwb.transpose();
            Matrix Pxzm = P_store * Him;
            Matrix Pz0m = Hk_uwb * Pxzm;
            rm = Zk_uwb + Hk_uwb * Xk_uwb;
            Matrix Pzzm = Pz0m + Rk_uwb;
            Matrix Kkm = Pxzm * Pzzm.inverse();
            Xk_uwb = Kkm * rm;
            Pk_uwb = P_store - Kkm * Pxzm.transpose();
        }
        else if (filter == "CKF")
        {
            hwa_uwb::uwb_proc::_meas_updata_ckf(_pos, Xk_uwb, Pk_uwb, Zk_uwb, Rk_uwb);
        }
        else if (filter == "PF")
        {
            hwa_uwb::uwb_proc::_meas_updata_pf(_pos, Xk_uwb, Pk_uwb, Zk_uwb, Rk_uwb);
        }

        _posterioriTest(Hk_uwb, Rk_uwb, rm, Xk_uwb, Pk_uwb, v_norm, vtpv);
        if (_outlierDetect(v_norm, outlier) >= 0); Pk_uwb = P_store;

        set_particles();

        this->_pos = pos_store - Xk_uwb.block(0, 0, _dim, 1);

        if (outlier < 0 && (Xk_store - Xk_uwb).norm() < 0.0001 /*&& Xki.norm() > 1e-10 && Xki.norm() < 1e-2*/)
        {
            Phik_uwb = Matrix::Identity(_dim, _dim);
            this->_pos = pos_store - Xk_uwb.block(0, 0, _dim, 1);
            _valid_node_num = valid_num;
            //cal dop
            double cof = pow(Rk_uwb.determinant(), 1.0 / _valid_node_num);
            Qenu = Hk_uwb.block(0, 0, valid_num, _dim).transpose() * Rk_uwb.block(0, 0, valid_num, valid_num).inverse() * Hk_uwb.block(0, 0, valid_num, _dim);
            Geo_pos = Cart2Geod(_pos, false);
            Cne(0, 0) = -sin(Geo_pos(1)); Cne(0, 1) = cos(Geo_pos(1)); Cne(0, 2) = 0.0;
            Cne(1, 0) = -sin(Geo_pos(0)) * cos(Geo_pos(1)); Cne(1, 1) = -sin(Geo_pos(0)) * sin(Geo_pos(1)); Cne(1, 2) = cos(Geo_pos(0));
            Cne(2, 0) = cos(Geo_pos(0)) * cos(Geo_pos(1)); Cne(2, 1) = cos(Geo_pos(0)) * sin(Geo_pos(1)); Cne(2, 2) = sin(Geo_pos(0));
            Qenu = Cne * Qenu.inverse() * Cne.transpose();
            Qenu = Qenu / cof;
            hdop = sqrt(Qenu(0, 0) + Qenu(1, 1));
            vdop = sqrt(Qenu(2, 2));
            pdop = sqrt(Qenu(0, 0) + Qenu(1, 1) + Qenu(2, 2));
            break;
        }
    }
    if (/*filter == "EKF" && */output_res)
    {
        Vector v = Zk_uwb - Hk_uwb * Xk_uwb;
        _res_output(v);
    }
    //if (_is_first)
    //    _is_first = false;
    std::cout << std::fixed << std::setprecision(10) << now.str_ymdhms() << "   end: " << Pk_uwb(0, 0) << " " << Pk_uwb(1, 1) << " " << Pk_uwb(2, 2) << " " << std::endl;

    Pk_Sav_uwb = Pk_uwb;

    return UWB_MEAS;
}

int hwa_uwb::uwb_proc::_uwb_init()
{
    _dim = 3;
    Pk_uwb = _init_Pk;
    Qt_uwb = _pos_psd.array().abs2();
    return 1;
}

int hwa_uwb::uwb_proc::_avaliable(const base_time& crt)
{
    // todo: 
    return 0;
}

Triple hwa_uwb::uwb_proc::_get_pos()
{
    return _pos;
}

void hwa_uwb::uwb_proc::_set_pos(const Triple& pos)
{
    _pos = pos;
    return;
}

Triple hwa_uwb::uwb_proc::_get_pos_psd()
{
    return _pos_psd;
}

Triple hwa_uwb::uwb_proc::_get_pos_std()
{
    Triple v;
    v(0) = sqrt(Pk_uwb(0, 0)); v(1) = sqrt(Pk_uwb(1, 1)); v(2) = sqrt(Pk_uwb(2, 2));
    return v;
}

SO3 hwa_uwb::uwb_proc::_get_pos_var()
{
    return Pk_uwb.block(0, 0, 3, 3);
}

Matrix& hwa_uwb::uwb_proc::_get_Pk() {
    return Pk_uwb;
}

base_time hwa_uwb::uwb_proc::_get_uwb_time()
{
    return _now_uwb;
}

hwa_map_id_uwbnode hwa_uwb::uwb_proc::_get_uwb_epo()
{
    return uwb_epo;
}

int hwa_uwb::uwb_proc::_get_iter()
{
    return _iter;
}

std::map<std::string, Triple> hwa_uwb::uwb_proc::_get_anchor_info()
{
    return _anchor_info;
}

UWB_WEIGHT hwa_uwb::uwb_proc::_get_wgt_type()
{
    return _wgt_type;
}

double hwa_uwb::uwb_proc::_get_ts()
{
    return _ts;
}

void hwa_uwb::uwb_proc::_remove_anchor(const std::string& name)
{
    uwb_epo.erase(name);
    return;
}

void hwa_uwb::uwb_proc::_remove_anchor(const bool& indoor, const bool& outdoor)
{

    for (auto a = uwb_epo.begin(); a != uwb_epo.end();)
    {
        if (indoor)
        {
            if (find(_indoor_anchor_list.begin(), _indoor_anchor_list.end(), a->first) != _indoor_anchor_list.end())
            {
                a = uwb_epo.erase(a);
                continue;
            }
        }
        if (outdoor)
        {
            if (find(_outdoor_anchor_list.begin(), _outdoor_anchor_list.end(), a->first) != _outdoor_anchor_list.end())
            {
                a = uwb_epo.erase(a);
                continue;
            }
        }
        ++a;
    }
    return;
}

void hwa_uwb::uwb_proc::_print_head()
{
    std::ostringstream os;
    //os << std::fixed << std::setprecision(6) << std::setw(18) << t;
    os << std::fixed << std::setw(15) << "#Seconds of Week" <<
        std::setw(18) << "X-ECEF" <<
        std::setw(18) << "Y-ECEF" <<
        std::setw(18) << "Z-ECEF" <<
        std::setw(18) << "valid_num" <<
        std::setw(18) << "nlos_num" <<
        std::setw(18) << "hdop" <<
        std::setw(18) << "vdop" << std::endl;

    _fuwb->write(os.str().c_str(), os.str().size());
    os.str("");
}

void hwa_uwb::uwb_proc::_write()
{
    std::ostringstream os;
    //os << std::fixed << std::setprecision(6) << std::setw(18) << t;
    os << std::fixed << std::setprecision(3) << std::setw(15) << _now_uwb.str_ymdhms() <<
        std::setw(18) << _pos(0) <<
        std::setw(18) << _pos(1) <<
        std::setw(18) << _pos(2) <<
        std::setw(18) << _valid_node_num <<
        std::setw(18) << _nlos.size() <<
        std::setw(18) << hdop <<
        std::setw(18) << vdop << std::endl;

    _fuwb->write(os.str().c_str(), os.str().size());
    os.str("");
}

void hwa_uwb::uwb_proc::_recoverPk()
{
    Pk_uwb = Pk_Sav_uwb;
    return;
}

void hwa_uwb::uwb_proc::_restorePk()
{
    Pk_Sav_uwb = Pk_uwb;
    return;
}

void hwa_uwb::uwb_proc::_reTime(const base_time& now)
{
    _now_uwb = now;
    //_now = now;
    return;
}

void hwa_uwb::uwb_proc::_res_output(const Vector& v)
{
    if (!output_res || _fuwbres == nullptr)
        return;
    std::ostringstream os;
    os << valid_node.size() << "\t";
    int i = 0;
    for (auto anchor : valid_node)
    {
        os << "\t" << anchor << "\t" << std::fixed << std::setprecision(4) << v(i) << "\t" << uwb_epo[anchor].range << "\t" << uwb_epo[anchor].fpRSSI << "\t" << uwb_epo[anchor].rxRSSI;
        i++;
    }
    os << std::endl;
    _fuwbres->write(os.str().c_str(), os.str().size());
    os.str("");
    return;
}

void hwa_uwb::uwb_proc::_error_compensation(Vector v)
{
    int i = 0;
    for (auto a : valid_node)
    {
        uwb_epo[a].range = uwb_epo[a].range - v(i);
        i++;
    }
    return;
}

bool hwa_uwb::uwb_proc::_prepareData(const base_time& now)
{
    _now_uwb = now;
    if (!uwbdata->loadUWB(_now_uwb.sow() + _now_uwb.dsec(), _ts, uwb_epo))
    {
        return false;
    }
    _preprocess();
    return true;
}

void hwa_uwb::uwb_proc::_preprocess()
{
    _nlos_detect();
    for (auto a = uwb_epo.begin(); a != uwb_epo.end();)
    {
        if (a->second.SNR < snr_lim || a->second.range > range_lim)
        {
            a = uwb_epo.erase(a);
            continue;
        }
        ++a;
    }
    for (auto b = _nlos.begin(); b != _nlos.end(); b++) {
        auto iter = uwb_epo.find(*b);
        if (iter != uwb_epo.end()) {
            uwb_epo.erase(iter);
        }
    }
    _smooth();
    return;
}

void hwa_uwb::uwb_proc::_nlos_detect()
{
    _nlos.clear();
    for (auto a : _anchor_list)
    {
        auto iter = uwb_epo.find(a);
        if (iter != uwb_epo.end())
        {
            if (iter->second.rxRSSI - iter->second.fpRSSI > 10)
            {
                _nlos.push_back(iter->first);
            }
        }
    }
    return;
}

void hwa_uwb::uwb_proc::_smooth()
{
    if (!smooth)
        return;
    int point = smooth_point, valid_point = 0;
    double t = _now_uwb.sow() + _now_uwb.dsec(), dis_smt = 0.0, snr_smt = 0.0, fp_smt = 0.0, rx_smt = 0.0, noise_smt = 0.0;
    hwa_map_id_uwbnode point_smt;
    bool front = false, back = false;

    point = point - 1 + point % 2;  //force to odd, zzwu

    for (auto a : uwb_epo)
    {
        valid_point = 1;
        dis_smt = a.second.range;
        snr_smt = a.second.SNR;
        fp_smt = a.second.fpRSSI;
        rx_smt = a.second.rxRSSI;
        noise_smt = a.second.noise;
        front = back = false;

        for (int it = 1; it <= point / 2; it++)
        {
            //if (!uwbdata->loadUWB(t - it * 0.1, _ts, point_smt) || point_smt.find(a.first) == point_smt.end())
            if (!uwbdata->loadUWB(t - it * 1.0 / _sample, _ts, point_smt) || point_smt.find(a.first) == point_smt.end())
            {
                //break;
            }
            else
            {
                front = true;
                dis_smt = dis_smt + point_smt.find(a.first)->second.range;
                snr_smt = snr_smt + point_smt.find(a.first)->second.SNR;
                fp_smt = fp_smt + point_smt.find(a.first)->second.fpRSSI;
                rx_smt = rx_smt + point_smt.find(a.first)->second.rxRSSI;
                noise_smt = noise_smt + point_smt.find(a.first)->second.noise;
                valid_point++;
            }


            //if (!uwbdata->loadUWB(t + it * 0.1, _ts, point_smt) || point_smt.find(a.first) == point_smt.end())
            if (!uwbdata->loadUWB(t + it * 1.0 / _sample, _ts, point_smt) || point_smt.find(a.first) == point_smt.end())
            {
                //break;
            }
            else
            {
                back = true;
                dis_smt = dis_smt + point_smt.find(a.first)->second.range;
                snr_smt = snr_smt + point_smt.find(a.first)->second.SNR;
                fp_smt = fp_smt + point_smt.find(a.first)->second.fpRSSI;
                rx_smt = rx_smt + point_smt.find(a.first)->second.rxRSSI;
                noise_smt = noise_smt + point_smt.find(a.first)->second.noise;
                valid_point++;
            }
        }
        if (front || back)
        {
            dis_smt = dis_smt / (valid_point / 1.0);
            snr_smt = snr_smt / (valid_point / 1.0);
            fp_smt = fp_smt / (valid_point / 1.0);
            rx_smt = rx_smt / (valid_point / 1.0);
            noise_smt = noise_smt / (valid_point / 1.0);
        }

        if (front && back)
        {
            uwb_epo.find(a.first)->second.range = dis_smt;
            uwb_epo.find(a.first)->second.SNR = snr_smt;
            uwb_epo.find(a.first)->second.fpRSSI = fp_smt;
            uwb_epo.find(a.first)->second.rxRSSI = rx_smt;
            uwb_epo.find(a.first)->second.noise = noise_smt;
        }  
    }

    return;
}

void hwa_uwb::uwb_proc::set_particles()
{
    if (filter != "PF" || _num_particles <= 0)
        return;

    for (int row = 0; row < nq; row++)
    {
        for (int col = 0; col < _num_particles; col++)
        {
            _particles(row, col) = sqrt(Pk_uwb(row, row)) * _particles_distribution(_particles_engine);
        }
    }

    weights_nonnormalized.fill(1.0 / _num_particles);
    weights_normalized.fill(1.0 / _num_particles);

    if (!_particles_init)
        _particles_init = true;

   
    return;
}

void hwa_uwb::uwb_proc::residual_resample(const Vector& weights, Vector& indexes)
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
        //rand [0, 1]
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

void hwa_uwb::uwb_proc::_resetMatrix()
{
    int obs_num = uwb_epo.size();
    valid_node.clear();

    Xk_uwb = Matrix::Zero(_dim, 1);
    Zk_uwb = Matrix::Zero(obs_num, 1);
    Hk_uwb = Matrix::Zero(obs_num, _dim);
    //Rk_uwb = Matrix::Zero(obs_num, obs_num);
    return;
}

int hwa_uwb::uwb_proc::_setMeas(const bool& isfirst)
{
    int valid_num = 0, obs_crt = 0;
    double sum = 0.0;
    for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
    {
        Triple base_crd_vector = _anchor_info.find(it->first)->second;
        double dist = (this->_pos - base_crd_vector).norm();
        Matrix H0 = Matrix::Zero(1, 3);
        H0 = (this->_pos - base_crd_vector).transpose() / dist;

        Hk_uwb.block(obs_crt, 0, 1, 3) = -H0;
        Zk_uwb(obs_crt) = it->second.range - dist;

        // weight
        if (isfirst)
        {
            switch (_wgt_type)
            {
            case UWB_WEIGHT::EQUAL:
                //Rk_uwb(obs_crt, obs_crt) = 9e-2;
                Rk_uwb(obs_crt, obs_crt) = pow(meas_range_std, 2);
                break;
            case UWB_WEIGHT::SNR:
                Rk_uwb(obs_crt, obs_crt) = pow(meas_range_std * sqrt(pow(10.0, -(it->second.SNR) / 10.0)), 2);

                break;
            case UWB_WEIGHT::ANTIRANGE:
                Rk_uwb(obs_crt, obs_crt) = 1e-2;
                sum += 1.0 / pow(it->second.range, 2);
                break;
            case UWB_WEIGHT::SUCCESSRATE:
                Rk_uwb(obs_crt, obs_crt) = 1 / pow(it->second.successrate * 5, 2);
                break;
            default:
                break;
            }
            //adjust var
            if (find(_nlos.begin(), _nlos.end(), it->first) != _nlos.end())
            {
                Rk_uwb(obs_crt, obs_crt) = Rk_uwb(obs_crt, obs_crt) * 9;
            }
        }
        obs_crt++;
        valid_num++;
        valid_node.push_back(it->first);
    }
    return valid_num;
}

bool hwa_uwb::uwb_proc::_ut(Triple& param, Vector& Xk, Matrix& Pk)
{
    int n = Xk.size(), obs_crt = 0;
    double alpha = 1e-3, beta = 2, kappa = 0,
        lambda = pow(alpha, 2) * (n + kappa) - n,
        gamma = sqrt(n / 1.0 + lambda);

    Vector Wm = Vector::Zero(2 * n + 1),
        Wc = Vector::Zero(2 * n + 1);

    xSig = Matrix::Zero(n, 2 * n + 1);
    zSig = Matrix::Zero(uwb_epo.size(), 2 * n + 1);
    zSigBar = Vector::Zero(uwb_epo.size());
    cSig = Matrix::Zero(uwb_epo.size(), uwb_epo.size());
    kSig = Matrix::Zero(n, uwb_epo.size());
    xSig.block(0, 0, n, 1) = Xk;
    Wm(0) = lambda / pow(gamma, 2);
    Wc(0) = Wm(0) + (1 - pow(alpha, 2) + beta);
    Sk = Pk.llt().matrixL(); Sk = Sk * gamma;

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
        for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
        {
            Triple base_crd_vector = _anchor_info.find(it->first)->second;
            double dist = (param + xSig.block(0, iSig, n, 1) - base_crd_vector).norm();
            zSig(obs_crt, iSig) = dist;
            obs_crt++;
        }
        zSigBar = zSigBar + Wm(iSig) * zSig.block(0, iSig, uwb_epo.size(), 1);
    }

    Vector zerror(n), xerror(n);
    for (int iSig = 0; iSig < 2 * n + 1; iSig++)
    {
        zerror = zSig.block(0, iSig, uwb_epo.size(), 1) - zSigBar;
        xerror = xSig.block(0, iSig, n, 1) - Xk;
        cSig = cSig + Wc(iSig) * zerror * zerror.transpose();   //zz
        kSig = kSig + Wc(iSig) * xerror * zerror.transpose();   //zx
    }
    return true;
}

bool hwa_uwb::uwb_proc::_meas_updata_ukf(Triple& param, Vector& Xk, Matrix& Pk, Vector& Zk, Matrix& Rk)
{
    int obs_crt = 0, n = Xk.size();
    Matrix Kk = Matrix::Zero(n, uwb_epo.size());

    for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
    {
        Zk(obs_crt) = it->second.range;
        obs_crt++;
    }

    if (hwa_uwb::uwb_proc::_ut(param, Xk, Pk))
    {
        cSig = cSig + Rk;
        Kk = kSig * cSig.inverse();
        Xk = Xk + Kk * (Zk - zSigBar);
        Zk = Zk - zSigBar;
        Pk = Pk - Kk * cSig * Kk.transpose();
    }
    return true;
}

bool hwa_uwb::uwb_proc::_meas_updata_ckf(Triple& param, Vector& Xk, Matrix& Pk, Vector& Zk, Matrix& Rk)
{
    int n = Xk.size(), obs_crt = 0;

    Vector Wm = Vector::Zero(2 * n),
        Wc = Vector::Zero(2 * n);

    xSig = Matrix::Zero(n, 2 * n);
    zSig = Matrix::Zero(uwb_epo.size(), 2 * n);
    zSigBar = Vector::Zero(uwb_epo.size());
    cSig = Matrix::Zero(uwb_epo.size(), uwb_epo.size());
    kSig = Matrix::Zero(n, uwb_epo.size());
    Sk = Pk.llt().matrixL();

    xSig.block(0, 0, n, n) = sqrt(n / 1.0) * Matrix::Identity(n, n);
    xSig.block(0, n, n, n) = -sqrt(n / 1.0) * Matrix::Identity(n, n);
    xSig = Sk * xSig;
    //std::cerr << xSig << std::endl;

    for (int iSig = 0; iSig < 2 * n; iSig++)
    {
        xSig.block(0, iSig, n, 1) = Xk + xSig.block(0, iSig, n, 1);

        obs_crt = 0;
        for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
        {
            Triple base_crd_vector = _anchor_info.find(it->first)->second;
            double dist = (param + xSig.block(0, iSig, n, 1) - base_crd_vector).norm();
            zSig(obs_crt, iSig) = dist;
            obs_crt++;
        }
        cSig = cSig + zSig.block(0, iSig, uwb_epo.size(), 1) * zSig.block(0, iSig, uwb_epo.size(), 1).transpose();
        kSig = kSig + xSig.block(0, iSig, n, 1) * zSig.block(0, iSig, uwb_epo.size(), 1).transpose();
    }
    cSig = 1.0 / (2 * n) * cSig; kSig = 1.0 / (2 * n) * kSig;

    zSigBar = 1.0 / (2 * n) * (zSig.rowwise().sum());
    cSig = cSig - zSigBar * zSigBar.transpose() + Rk;
    kSig = kSig - Xk * zSigBar.transpose();

    Matrix Kk = Matrix::Zero(n, uwb_epo.size());
    Kk = kSig * cSig.inverse();

    obs_crt = 0;
    for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
    {
        Zk(obs_crt) = it->second.range;
        obs_crt++;
    }

    Xk = Xk + Kk * (Zk - zSigBar);
    Zk = Zk - zSigBar;
    Pk = Pk - Kk * cSig * Kk.transpose();

    return true;
}

bool hwa_uwb::uwb_proc::_meas_updata_pf(Triple& param, Vector& Xk, Matrix& Pk, Vector& Zk, Matrix& Rk)
{
    int obs_crt = 0, n = Xk.size();
    zSig = Matrix::Zero(uwb_epo.size(), _num_particles);
    Vector weight = Vector::Zero(_num_particles);
    Vector cum_weight = Vector::Zero(_num_particles);

    Matrix Kk = Matrix::Zero(n, uwb_epo.size());

    for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
    {
        Zk(obs_crt) = it->second.range;
        obs_crt++;
    }

    for (int idx = 0; idx < _num_particles; idx++)
    {
        obs_crt = 0;
        for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
        {
            Triple base_crd_vector = _anchor_info.find(it->first)->second;
            double dist = (param + _particles.block(0, idx, n, 1) - base_crd_vector).norm();
            zSig(obs_crt, idx) = dist;
            obs_crt++;
        }
        Vector res = Zk - zSig.block(0, idx, uwb_epo.size(), 1);

        double vtpv = res.transpose() * (Rk / Rk.norm()).inverse() * res;
        weight(idx) = exp(-vtpv / 2.0);
        if (idx == 0)
            cum_weight(idx) = weight(idx);
        else
            cum_weight(idx) = cum_weight(idx - 1) + weight(idx);
    }
    cum_weight = cum_weight / weight.sum();
    weight = weight / weight.sum();

    Vector indexes = Vector::Zero(_num_particles);
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

    obs_crt = 0;
    for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
    {
        Triple base_crd_vector = _anchor_info.find(it->first)->second;
        double dist = (param + Xk - base_crd_vector).norm();
        Zk(obs_crt) = Zk(obs_crt) - dist;
        obs_crt++;
    }

    return true;
}

int hwa_uwb::uwb_proc::_outlierDetect(const Vector& v, int& outlier)
{
    int nobs = v.rows();
    double max = 0.0;
    int idx = -1;
    for (int i = 0; i < nobs; i++)
    {
        if (fabs(v(i)) > max && fabs(v(i)) > max_res_norm)
        {
            max = fabs(v(i));
            idx = i;
        }
    }
    if (idx >= 0)
    {
        outlier = idx;
    }
    else
        outlier = -1;
    return idx;
}

int uwb_proc::_outlierDetect(const Vector& v, int& outlier, double& lambda)
{
    int nobs = v.rows();
    double max = 0.0;
    int idx = -1;

    for (int i = 0; i < nobs; i++)
    {
        if (fabs(v(i)) > max && fabs(v(i)) > max_res_norm)
        {
            max = fabs(v(i));
            idx = i;
        }
    }

    if (idx >= 0)
    {
        outlier = idx;
        lambda = fabs(v(idx)) / max_res_norm;
    }
    else
        outlier = -1;
    return idx;
}

void uwb_proc::_posterioriTest(const Matrix& A, const Matrix& P, const Vector& l,
    const Vector& dx, const Matrix& Q, Vector& v_norm, double& vtpv)
{
    double sig_unit = 0.0;
    Vector v_orig, v_test;
    Vector dxxx = dx;
    for (int i = 0; i < 3; i++)
        dxxx(i) = 0;
    v_orig = A * dx - l;
    Matrix Qv = P + A * Q * A.transpose();
    v_norm.resize(v_orig.size());
    for (int i = 0; i < v_norm.rows(); i++)
    {
        if (v_orig(i) == 0)
        {
            v_norm(i) = 0.0;
            continue;
        }
        v_norm(i) = sqrt(1 / abs(Qv(i, i))) * abs(v_orig(i));
    }

    int freedom = A.rows() - A.cols();
    if (freedom < 1)
    {
        freedom = 1;
    }

    Vector vtPv = v_orig.transpose() * P.inverse() * v_orig;
    sig_unit = vtPv(0) / freedom;
    vtpv = vtPv(0);
    return;
}

int uwb_proc::nlos_est(std::vector<double>& mu, Triple x_b) {
    Triple x_store = x_b;
    if (AnchorPos.size() == 0) return 0;

    Triple x_b_hat = x_b;
    Vector Residuals_Original = Vector::Zero(AnchorPos.size());
    std::vector<std::vector<double>> NLOS;

    //GENERATE GROUPS OF INITIAL VALUES OF NLOS
    for (int i = 0; i < AnchorPos.size(); ++i) {
        double residual = abs(IniRange[i] - (x_b - AnchorPos.at(i)).norm());
        Residuals_Original(i) = residual;
        NLOS.push_back(generateArithmeticSequence(tolerance, residual));
    }
    std::vector<std::vector<double>> all_combinations;
    std::vector<double> current_combination(NLOS.size(), 0.0);
    generateAllCombinations(NLOS, current_combination, 0, all_combinations);

    //FIND THE MINIMUM COST SOLUTION
    Vector Residuals = Residuals_Original / Residuals_Original.maxCoeff();
    double Cost = 1;
    double Cost_final = 1e6;
    std::vector<double> final_combination(AnchorPos.size(), 0.0);
    for (auto& combination : all_combinations) {
        Triple x_b_hat = x_store;
        std::vector<double> mu_hat = combination;
        Optimization(Cost, x_b_hat, AnchorPos, mu_hat, Residuals_Original, IniRange);
        if (Cost < Cost_final) {
            Cost_final = Cost;
            mu = mu_hat;
            x_b = x_b_hat;
        }
    }
    std::cout << "Base Number: " << AnchorPos.size() << std::endl << "Time: " << _now_uwb.sod() << std::endl 
        << "Add: " << (x_store - x_b).transpose() << std::endl;
    std::cout << "Cost: " << Cost_final << std::endl;
    std::cout << "Gama_transformed: ";
    double cost = 0;
    for (int i = 0; i < mu.size(); i++) {
        double gama_transformed;
        double ratio = Residuals_Original(i) / sigmoid_threshold;
        if (Residuals_Original(i) < sigmoid_threshold) {
            gama_transformed = gama * exp(sigmoid_parameter / ratio - sigmoid_parameter);
            gama_transformed = std::min(1e4, gama_transformed);
        }
        else {
            gama_transformed = gama * exp(-sigmoid_parameter * ratio + sigmoid_parameter);
            gama_transformed = std::max(1e-4, gama_transformed);
        }
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(4) << gama_transformed << " ";
    }
    std::cout << std::endl << "NLOS: ";
    for (int i = 0; i < mu.size(); i++) {
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(4) << mu[i] << " ";
    }
    std::cout << std::endl;

    if (_outfile["SDP"].is_open()) {
        for (int i = 0; i < mu.size(); ++i) {
            _outfile["SDP"] << std::setiosflags(std::ios::fixed) << std::setprecision(4) << _now_uwb.sod() << " " 
                << anchorname.at(i) << " " << mu[i] << " " << std::endl;
        }
    }
}

void uwb_proc::_extract_value() {
    AnchorPos.clear();
    IniRange = Vector::Zero(uwb_epo.size());
    int i = 0;
    for (auto it = uwb_epo.begin(); it != uwb_epo.end(); it++)
    {
        Triple base_crd_vector = _anchor_info.find(it->first)->second;
        AnchorPos.push_back(base_crd_vector);
        IniRange[i++] = it->second.range;
    }
}

std::vector<double> uwb_proc::generateArithmeticSequence(double r, double Z) {
    std::vector<double> sequence;
    if (Z < 0) {
        for (double current = 0; current >= Z; current -= r) {
            sequence.push_back(current);
        }
    }
    else {
        for (double current = 0; current <= Z; current += r) {
            sequence.push_back(current);
        }
    }
    return sequence;
}

void uwb_proc::generateAllCombinations(
    const std::vector<std::vector<double>>& NLOS,
    std::vector<double>& current_combination,
    int index,
    std::vector<std::vector<double>>& all_combinations) {
    if (index == NLOS.size()) {
        all_combinations.push_back(current_combination);
        return;
    }
    for (double value : NLOS[index]) {
        current_combination[index] = value;
        generateAllCombinations(NLOS, current_combination, index + 1, all_combinations);
    }
}

void uwb_proc::Optimization(double& Cost, Triple& x_b, const std::vector<Triple> x_u, std::vector<double>& mu, const Vector Residuals_Original, Vector r) {
    Vector Residuals = Residuals_Original / Residuals_Original.maxCoeff();
    Triple pos_uwb_ecef = x_b;
    if (pred_constraint) {
        pos_uwb_ecef[0] += 0.001;
        pos_uwb_ecef[1] += 0.001;
        pos_uwb_ecef[2] += 0.001;
    }
    ceres::Problem problem;

    if (penal == hwa_uwb::EXP) {
        for (size_t i = 0; i < x_u.size(); ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 1>(
                    new CostFunctor(r[i], x_u[i], gama * exp(-exp_parameter * Residuals(i))));
            problem.AddResidualBlock(cost_function, nullptr, x_b.data(), &mu[i]);
        }
    }
    else if (penal == hwa_uwb::LINEAR) {
        for (size_t i = 0; i < x_u.size(); ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 1>(
                    new CostFunctor(r[i], x_u[i], gama / Residuals(i)));
            problem.AddResidualBlock(cost_function, nullptr, x_b.data(), &mu[i]);
        }

        if (pred_constraint) {
            ceres::CostFunction* cost_function_xb =
                new ceres::AutoDiffCostFunction<CostFunctor_xb, 1, 3>(
                    new CostFunctor_xb(pos_uwb_ecef, alpha));
            problem.AddResidualBlock(cost_function_xb, nullptr, x_b.data());
        }
    }

    else if (penal == hwa_uwb::LOG) {
        for (size_t i = 0; i < x_u.size(); ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 1>(
                    new CostFunctor(r[i], x_u[i], gama / log(1 + Residuals(i) + log_parameter)));
            problem.AddResidualBlock(cost_function, nullptr, x_b.data(), &mu[i]);
        }

        if (pred_constraint) {
            ceres::CostFunction* cost_function_xb =
                new ceres::AutoDiffCostFunction<CostFunctor_xb, 1, 3>(
                    new CostFunctor_xb(pos_uwb_ecef, alpha));
            problem.AddResidualBlock(cost_function_xb, nullptr, x_b.data());
        }
    }
    else if (penal == hwa_uwb::SIGMOID) {
        for (size_t i = 0; i < x_u.size(); ++i) {
            double gama_transformed;
            double ratio = Residuals_Original(i) / sigmoid_threshold;
            if (Residuals_Original(i) < sigmoid_threshold) {
                gama_transformed = gama * exp(sigmoid_parameter / ratio - sigmoid_parameter);
                gama_transformed = std::min(1e4, gama_transformed);
            }
            else {
                gama_transformed = gama * exp(-sigmoid_parameter * ratio + sigmoid_parameter);
                gama_transformed = std::max(1e-4, gama_transformed);
            }

            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 1>(
                    new CostFunctor(r[i], x_u[i], gama_transformed));
            problem.AddResidualBlock(cost_function, nullptr, x_b.data(), &mu[i]);
            //problem.SetParameterLowerBound(&mu[i], 0, 0.0);
        }

        if (pred_constraint) {
            ceres::CostFunction* cost_function_xb =
                new ceres::AutoDiffCostFunction<CostFunctor_xb, 1, 3>(
                    new CostFunctor_xb(pos_uwb_ecef, alpha));
            problem.AddResidualBlock(cost_function_xb, nullptr, x_b.data());
        }
    }

    ceres::Solver::Options options;


    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100000;
    options.gradient_tolerance = 1e-10;
    options.function_tolerance = 1e-10;
    options.parameter_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    Cost = summary.final_cost;
}

void uwb_proc::_nlos_detectbypos(Triple pos_uwb) {
    for (auto a : _anchor_list)
    {
        auto iter = uwb_epo.find(a);
        Triple base_crd_vector = _anchor_info.find(iter->first)->second;
        double dist = (pos_uwb - base_crd_vector).norm();
        double diff = dist - iter->second.range;
        if (iter != uwb_epo.end())
        {
            if (diff > 1)
            {
                _nlos.push_back(iter->first);
            }
        }
    }
}