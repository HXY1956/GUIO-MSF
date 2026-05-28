#include"hwa_set_lidar.h"
using namespace hwa_lidar;
using namespace std;

hwa_set::set_lidar::set_lidar() : set_base()
{
    _set.insert(XMLKEY_LIDAR);
    _e0 = 0.85;
    _g0 = 0.85;
    _max_res_norm = 3.0;
    _barrior = 9;
    _kappa_sig = 0;
    _alpha_sig = 0.001;
    _dof1 = 1;
    _dof2 = 15;
    _tau = 1000;
    _proc_noise = 0.05;
    _max_iter = 20;
    _num_particles = 500;
}

hwa_set::set_lidar::~set_lidar()
{
}

void hwa_set::set_lidar::check()
{
    xml_node parent = _doc.child(XMLKEY_ROOT);
    xml_node node = _default_node(parent, XMLKEY_LIDAR);
    _default_attr(node, "filter", "EKF");
    _default_attr(node, "kappa_sig", 0);
    _default_attr(node, "alpha_sig", 0.001);
    _default_attr(node, "tau", 1000);
    _default_attr(node, "proc_noise", 0.05);
    _default_attr(node, "g0", 0.85);
    _default_attr(node, "e0", 0.85);
    _default_attr(node, "max_iter", 20);
    _default_attr(node, "_num_particles", 500);
    _default_attr(node, "barrior", 9);
    _default_attr(node, "dof1", 1);
    _default_attr(node, "dof2", 15);
    _default_attr(node, "max_res_norm", 3);
}

void hwa_set::set_lidar::help()
{
}

double hwa_set::set_lidar::start()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("start");
    str_erase(tmp);
    double tmp_double = 0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double hwa_set::set_lidar::end()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("end");
    str_erase(tmp);
    double tmp_double = 0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double hwa_set::set_lidar::ts()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("freq");
    str_erase(tmp);
    int tmp_int = 10; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    double res = 1.0 / tmp_int;
    return res;
}

double hwa_set::set_lidar::imu_ts()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("imu_freq");
    str_erase(tmp);
    int tmp_int = 100; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    double res = 1.0 / tmp_int;
    return res;
}

int hwa_set::set_lidar::freq()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("freq");
    str_erase(tmp);
    int tmp_int = 10; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

int hwa_set::set_lidar::imu_freq()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("imu_freq");
    str_erase(tmp);
    int tmp_int = 100; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

SO3 hwa_set::set_lidar::R_lidar_imu()
{
    string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("R_lidar_imu");
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == ',')str[i] = ' ';
    }
    stringstream ss(str);
    SO3 res;
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
    ss >> R11 >> R12 >> R13 >> R21 >> R22 >> R23 >> R31 >> R32 >> R33;
    res << R11, R12, R13, R21, R22, R23, R31, R32, R33;

    return res;
}

Triple hwa_set::set_lidar::t_lidar_imu()
{
    string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("t_lidar_imu");
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == ',')str[i] = ' ';
    }
    stringstream ss(str);
    SO3 res;
    double X, Y, Z;
    ss >> X >> Y >> Z;

    return Triple(X, Y, Z);
}

Eigen::Isometry3d hwa_set::set_lidar::T_lidar_imu()
{
    SO3 tmp_R_lidar_imu = R_lidar_imu();
    Triple tmp_t_lidar_imu = t_lidar_imu();
    Eigen::Isometry3d tmp_T_lidar_imu=Eigen::Isometry3d::Identity();
    tmp_T_lidar_imu.linear() = tmp_R_lidar_imu;
    tmp_T_lidar_imu.translation() = tmp_t_lidar_imu;
    return tmp_T_lidar_imu;
}

int hwa_set::set_lidar::n_scans()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("n_scans");
    str_erase(tmp);
    int tmp_int = 16; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

int hwa_set::set_lidar::window_size()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("window_size");
    str_erase(tmp);
    int tmp_int = 10; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

bool hwa_set::set_lidar::use_scan()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("use_scan");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

bool hwa_set::set_lidar::use_pp()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("use_pp");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

bool hwa_set::set_lidar::build_map()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("build_map");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

bool hwa_set::set_lidar::use_segmenter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("use_segmenter");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

bool hwa_set::set_lidar::use_map()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("use_map");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

double hwa_set::set_lidar::prior_map_resolution()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("prior_map_resolution");
    str_erase(tmp);
    double tmp_double = 0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double hwa_set::set_lidar::map_resolution_sharp()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("map_resolution_sharp");
    str_erase(tmp);
    double tmp_double = 0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double hwa_set::set_lidar::map_resolution_surf()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("map_resolution_surf");
    str_erase(tmp);
    double tmp_double = 0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double hwa_set::set_lidar::scan_observation_noise()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("scan_observation_noise");
    str_erase(tmp);
    double tmp_double = 0.05; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double hwa_set::set_lidar::map_observation_noise()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("map_observation_noise");
    str_erase(tmp);
    double tmp_double = 0.02; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

bool hwa_set::set_lidar::distortion()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("distortion");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

double hwa_set::set_lidar::close_threshold()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("close_threshold");
    str_erase(tmp);
    double tmp_double = 0.3; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

bool hwa_set::set_lidar::estimate_extrinsic()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("estimate_extrinsic");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

Triple hwa_set::set_lidar::initial_lidar_extrinsic_rotation_cov()
{
    string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("initial_extrinsic_rotation_cov");

    if (str.empty()) return Triple(3.0462e-4, 3.0462e-4, 3.0462e-4);
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == ',')str[i] = ' ';
    }
    stringstream ss(str);
    Triple res;
    double s1, s2, s3;
    ss >> s1 >> s2 >> s3;
    res << s1, s2, s3;
    return res;
}

Triple hwa_set::set_lidar::initial_lidar_extrinsic_translation_cov()
{
    string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("initial_extrinsic_translation_cov");

    if (str.empty()) return Triple(1e-4, 1e-4, 1e-4);
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == ',')str[i] = ' ';
    }
    stringstream ss(str);
    Triple res;
    double s1, s2, s3;
    ss >> s1 >> s2 >> s3;
    res << s1, s2, s3;
    return res;
}

double hwa_set::set_lidar::barrior()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("barrior");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _barrior; //default value
    return tmp_double;
}

double hwa_set::set_lidar::kappa_sig()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("kappa_sig");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _kappa_sig; //default value
    return tmp_double;
}

double hwa_set::set_lidar::alpha_sig()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("alpha_sig");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _alpha_sig; //default value
    return tmp_double;
}

double hwa_set::set_lidar::E0()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("e0");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _e0; //default value
    return tmp_double;
}

double hwa_set::set_lidar::G0()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("g0");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _g0; //default value
    return tmp_double;
}

int hwa_set::set_lidar::dof1()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("dof1");
    str_erase(tmp);
    int tmp_int;
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    else
        tmp_int = _dof1; //default value
    return tmp_int;
}

int hwa_set::set_lidar::dof2()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("dof2");
    str_erase(tmp);
    int tmp_int;
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    else
        tmp_int = _dof2; //default value
    return tmp_int;
}

int hwa_set::set_lidar::max_iter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("max_iter");
    str_erase(tmp);
    int tmp_int;
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    else
        tmp_int = _max_iter; //default value
    return tmp_int;
}

int hwa_set::set_lidar::num_particles()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("num_particles");
    str_erase(tmp);
    int tmp_int;
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    else
        tmp_int = _num_particles; //default value
    return tmp_int;
}

double hwa_set::set_lidar::Tau()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("tau");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _tau; //default value
    return tmp_double;
}

double hwa_set::set_lidar::proc_noise()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("proc_noise");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _proc_noise; //default value
    return tmp_double;
}

double hwa_set::set_lidar::max_res_norm()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("max_res_norm");
    str_erase(tmp);
    double tmp_double;
    if (tmp != "")
        tmp_double = std::stod(tmp);
    else
        tmp_double = _max_res_norm; //default value
    return tmp_double;
}

std::string hwa_set::set_lidar::filter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("filter");
    str_erase(tmp);
    transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
    if (tmp.empty())tmp = "EKF";
    return tmp;
}

std::string hwa_set::set_lidar::proc_mode()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_LIDAR).child_value("proc_mode");
    str_erase(tmp);
    if (tmp.empty())
    {
        tmp = "equal_noise";
    }
    return tmp;
}