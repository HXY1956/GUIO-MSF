#include "hwa_set_vis.h"

hwa_set::set_vis::set_vis() : hwa_set::set_base()
{
    _set.insert(XMLKEY_VIS);
    checkdefault();
}

hwa_set::set_vis::~set_vis()
{
}

hwa_vis::DISTORTION_TYPE hwa_vis::str2distortion(const std::string& s)
{
    std::string tmp = s;
    std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
    if (tmp == "radtan")return radtan;
    if (tmp == "equidistant")return equidistant;

    return radtan;
}

hwa_vis::PROCESSER_TYPE hwa_vis::str2processer(const std::string& s) {
    std::string tmp = s;
    std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
    if (tmp == "GPU")return GPU;
    if (tmp == "CPU")return CPU;
    return CPU;
}

hwa_vis::CLONE_TYPE hwa_set::set_vis::str2ct(const std::string& s) {
    std::string tmp = s;
    transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
    if (tmp == "IMU") return hwa_vis::IMU;
    else if (tmp == "CAMERA") return hwa_vis::CAMERA;
    return hwa_vis::IMU;
}

std::string hwa_vis::distortion2str(const hwa_vis::DISTORTION_TYPE& s)
{
    //std::string res;
    if (s == 0)
        return std::string("radtan");
    else if (s == 1)
        return std::string("equidistant");
    return std::string("radtan");
}

void hwa_set::set_vis::checkdefault()
{
    xml_node parent = _doc.child(XMLKEY_ROOT);
    xml_node node = _default_node(parent, XMLKEY_VIS);
    _default_attr(node, "start", 0);
    _default_attr(node, "end", 1000000);
    _default_attr(node, "max_cam_state_size", 10);
    _default_attr(node, "position_std_threshold", 8);
    _default_attr(node, "rotation_threshold", 0.2618);
    _default_attr(node, "translation_threshold", 0.5);
    _default_attr(node, "feature_observation_noise1", 0.005);
    _default_attr(node, "feature_observation_noise", 0.008);
    _default_attr(node, "maxcnt", 350);
    _default_attr(node, "maxcnt1", "700");
    _default_attr(node, "equalize", 0);
    _default_attr(node, "cam_update_skip", 0);
    _default_attr(node, "multi_thread_imgproc", 0);
    _default_attr(node, "num_of_cam_group", 1);
    _default_attr(node, "static_threshold", "1.2");
    _default_attr(node, "Use_vis_For_Initial", false);
    _default_attr(node, "check_static", false);
    _default_attr(node, "clone_type", "CAMERA");
    _default_attr(node, "minparallex", 15);
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
    _default_attr(node, "filter", "EKF");
    _default_attr(node, "max_res_norm", 3);
}

void hwa_set::set_vis::check()
{
}

void hwa_set::set_vis::help()
{
}

int hwa_set::set_vis::num_of_cam_group()
{
    int num_of_cam_group = 1;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).attribute("num_of_cam_group").empty())
        num_of_cam_group = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).attribute("num_of_cam_group").as_int();
    return num_of_cam_group;
}

double hwa_set::set_vis::start(int cam_group_id)
{
    double res = 0.0;
    std::string tmp;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("start");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::end(int cam_group_id)
{
    double res = 100000.0;
    std::string tmp;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("end");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::ts(int cam_group_id)
{
    double res = 20;
    std::string tmp;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("freq");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    res = 1 / res;
    return res;
}

int hwa_set::set_vis::freq(int cam_group_id)
{
    int res = 20;
    std::string tmp;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("freq");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

bool hwa_set::set_vis::stereo(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("stereo");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

hwa_vis::DISTORTION_TYPE hwa_set::set_vis::cam0_distortion_model(int cam_group_id)
{
    std::string tmp = "radtan";
    str_erase(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam0_distortion_model");
        str_erase(tmp2);
        if (tmp2 != "")
            tmp = tmp2;
    }
    return hwa_vis::str2distortion(tmp);
}

hwa_vis::DISTORTION_TYPE hwa_set::set_vis::cam1_distortion_model(int cam_group_id)
{
    std::string tmp = "radtan";
    str_erase(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam1_distortion_model");
        str_erase(tmp2);
        if (tmp2 != "")
            tmp = tmp2;
    }
    return hwa_vis::str2distortion(tmp);
}

Eigen::Vector2d hwa_set::set_vis::cam0_resolution(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("cam0_resolution");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam0_resolution");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0;
    ss >> X >> Y;
    return Eigen::Vector2d(X, Y);
}

Eigen::Vector2d hwa_set::set_vis::cam1_resolution(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("cam1_resolution");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam1_resolution");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0;
    ss >> X >> Y;
    return Eigen::Vector2d(X, Y);
}

Eigen::Vector4d hwa_set::set_vis::cam0_intrinsics(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("cam0_intrinsics");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam0_intrinsics");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    ss >> fx >> fy >> cx >> cy;
    return Eigen::Vector4d(fx, fy, cx, cy);
}

Eigen::Vector4d hwa_set::set_vis::cam1_intrinsics(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("cam1_intrinsics");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam1_intrinsics");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    ss >> fx >> fy >> cx >> cy;
    return Eigen::Vector4d(fx, fy, cx, cy);
}

Eigen::Vector4d hwa_set::set_vis::cam0_distortion_coeffs(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("cam0_distortion_coeffs");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam0_distortion_coeffs");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0;
    ss >> k1 >> k2 >> p1 >> p2;
    return Eigen::Vector4d(k1, k2, p1, p2);
}

Eigen::Vector4d hwa_set::set_vis::cam1_distortion_coeffs(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("cam1_distortion_coeffs");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam1_distortion_coeffs");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0;
    ss >> k1 >> k2 >> p1 >> p2;
    return Eigen::Vector4d(k1, k2, p1, p2);
}

SO3 hwa_set::set_vis::R_cam0_cam1(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("R_cam0_cam1");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("R_cam0_cam1");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double R11 = 0.0, R12 = 0.0, R13 = 0.0, R21 = 0.0, R22 = 0.0, R23 = 0.0, R31 = 0.0, R32 = 0.0, R33 = 0.0;
    ss >> R11 >> R12 >> R13 >> R21 >> R22 >> R23 >> R31 >> R32 >> R33;
    SO3 res;
    res << R11, R12, R13, R21, R22, R23, R31, R32, R33;
    return res;
}

Triple hwa_set::set_vis::t_cam0_cam1(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("t_cam0_cam1");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("t_cam0_cam1");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0, Z = 0.0;
    ss >> X >> Y >> Z;
    return Triple(X, Y, Z);
}

SE3 hwa_set::set_vis::T_cam0_cam1(int cam_group_id)
{
    SO3 tmp_R_cam0_cam1 = R_cam0_cam1(cam_group_id);
    Triple tmp_t_cam0_cam1 = t_cam0_cam1(cam_group_id); 

    SE3 tmp_T_cam0_cam1 = SE3::Identity();
    tmp_T_cam0_cam1.linear() = tmp_R_cam0_cam1;
    tmp_T_cam0_cam1.translation() = tmp_t_cam0_cam1;
    return tmp_T_cam0_cam1;
}

SO3 hwa_set::set_vis::R_cam0_imu(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("R_cam0_imu");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("R_cam0_imu");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double R11 = 0.0, R12 = 0.0, R13 = 0.0, R21 = 0.0, R22 = 0.0, R23 = 0.0, R31 = 0.0, R32 = 0.0, R33 = 0.0;
    ss >> R11 >> R12 >> R13 >> R21 >> R22 >> R23 >> R31 >> R32 >> R33;
    SO3 res;
    res << R11, R12, R13, R21, R22, R23, R31, R32, R33;
    return res;
}

Triple hwa_set::set_vis::t_cam0_imu(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("t_cam0_imu");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("t_cam0_imu");
        if (tmp2 != "")
            tmp = tmp2;
    }

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0, Z = 0.0;
    ss >> X >> Y >> Z;
    return Triple(X, Y, Z);
}

SE3 hwa_set::set_vis::T_cam0_imu(int cam_group_id)
{
    SO3 tmp_R_cam0_imu = R_cam0_imu(cam_group_id);
    Triple tmp_t_cam0_imu = t_cam0_imu(cam_group_id);

    SE3 tmp_T_cam0_imu = SE3::Identity();
    tmp_T_cam0_imu.linear() = tmp_R_cam0_imu;
    tmp_T_cam0_imu.translation() = tmp_t_cam0_imu;
    return tmp_T_cam0_imu;
}

double hwa_set::set_vis::dt_cam0_imu(int cam_group_id)
{
    double res = 0.0;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("dt_cam0_imu");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("dt_cam0_imu");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

int hwa_set::set_vis::num_of_cam(int cam_group_id)
{
    int res = 1;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("num_of_cam");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("num_of_cam");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::grid_row(int cam_group_id)
{
    int res = 7;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("grid_row");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("grid_row");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::grid_col(int cam_group_id)
{
    int res = 8;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("grid_col");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("grid_col");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::grid_min_feature_num(int cam_group_id)
{
    int res = 3;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("grid_min_feature_num");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("grid_min_feature_num");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::grid_max_feature_num(int cam_group_id)
{
    int res = 8;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("grid_max_feature_num");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("grid_max_feature_num");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::pyramid_levels(int cam_group_id)
{
    int res = 3;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("pyramid_levels");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("pyramid_levels");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::patch_size(int cam_group_id)
{
    int res = 15;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("patch_size");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("patch_size");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::fast_threshold(int cam_group_id)
{
    int res = 5;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("fast_threshold");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("fast_threshold");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

double hwa_set::set_vis::ransac_threshold(int cam_group_id)
{
    double res = 3.0;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("ransac_threshold");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("ransac_threshold");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::stereo_threshold(int cam_group_id)
{
    double res = 5.0;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("stereo_threshold");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("stereo_threshold");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

int hwa_set::set_vis::max_iteration(int cam_group_id)
{
    int res = 30;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("max_iteration");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("max_iteration");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

double hwa_set::set_vis::track_precision(int cam_group_id)
{
    double res = 0.01;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("track_precision");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("track_precision");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

bool hwa_set::set_vis::usingstereorecify(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("usingstereorectify");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

int hwa_set::set_vis::max_cnt(int cam_group_id)
{
    int res = 350;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("maxcnt");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

int hwa_set::set_vis::max_cam_state_size(int cam_group_id)
{
    int res = 10;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("max_cam_state_size");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

double hwa_set::set_vis::minparallex(int cam_group_id)
{
    double res = 20;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("minparallex");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::position_std_threshold(int cam_group_id)
{
    double res = 5;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("position_std_threshold");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::rotation_threshold(int cam_group_id)
{
    double res = 0.2618;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("rotation_threshold");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::translation_threshold(int cam_group_id)
{
    double res = 0.5;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("translation_threshold");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::feature_observation_noise(int cam_group_id)
{
    double res = 0.008;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("feature_observation_noise");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

int hwa_set::set_vis::_equalize(int cam_group_id)
{
    int res = 0;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("equalize");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

double hwa_set::set_vis::huber_epsilon(int cam_group_id)
{
    double res = 0.01;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("huber_epsilon");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::estimation_precision(int cam_group_id)
{
    double res = 0.000009;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("estimation_precision");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

double hwa_set::set_vis::initial_damping(int cam_group_id)
{
    double res = 0.001;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("initial_damping");
        str_erase(tmp);
        if (tmp != "")
            res = std::stod(tmp);
    }
    return res;
}

int hwa_set::set_vis::outler_loop_max_iteration(int cam_group_id)
{
    int res = 10;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("outler_loop_max_iteration");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

int hwa_set::set_vis::inner_loop_max_iteration(int cam_group_id)
{
    int res = 10;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("inner_loop_max_iteration");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

bool hwa_set::set_vis::estimate_extrinsic(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("estimate_extrinsic");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

bool hwa_set::set_vis::estimate_t(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("estimate_t");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

bool hwa_set::set_vis::estimate_extrinsic_seperately(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("estimate_extrinsic_seperately");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

bool hwa_set::set_vis::estimate_extrinsic_allcam(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("estimate_extrinsic_allcam");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

bool hwa_set::set_vis::estimate_t_allcam(int cam_group_id)
{
    bool res = false;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("estimate_t_allcam");
        str_erase(tmp);
        if (tmp != "")
            res = (tmp == "true" || tmp == "1" || tmp == "yes");
    }
    return res;
}

Triple hwa_set::set_vis::initial_extrinsic_rotation_cov(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("initial_extrinsic_rotation_cov");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("initial_extrinsic_rotation_cov");

        if (tmp2 != "")
            tmp = tmp2;
    }

    if (tmp.empty()) return Triple(3.0462e-4, 3.0462e-4, 3.0462e-4);

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double s1 = 0.0, s2 = 0.0, s3 = 0.0;
    ss >> s1 >> s2 >> s3;
    Triple res;
    res << s1, s2, s3;
    return res;
}

Triple hwa_set::set_vis::initial_extrinsic_translation_cov(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("initial_extrinsic_translation_cov");

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("initial_extrinsic_translation_cov");

        if (tmp2 != "")
            tmp = tmp2;
    }

    if (tmp.empty()) return Triple(1e-4, 1e-4, 1e-4);

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double s1 = 0.0, s2 = 0.0, s3 = 0.0;
    ss >> s1 >> s2 >> s3;
    Triple res;
    res << s1, s2, s3;
    return res;
}

double hwa_set::set_vis::initial_t_cov(int cam_group_id)
{
    double res = 0.0;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("initial_t_cov");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("initial_t_cov");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

int hwa_set::set_vis::cam_update_skip(int cam_group_id)
{
    int res = 0;
    std::string tmp;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("cam_update_skip");
        str_erase(tmp);
        if (tmp != "")
            res = std::stoi(tmp);
    }
    return res;
}

hwa_vis::CLONE_TYPE hwa_set::set_vis::clone_type(int cam_group_id)
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("clone_type");
    str_erase(tmp);
    if (tmp.empty())
        tmp = "CAMERA";

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("clone_type");
        str_erase(tmp2);
        if (!tmp2.empty())
            tmp = tmp2;
    }
    return str2ct(tmp);
}

double hwa_set::set_vis::barrior(int cam_group_id)
{
    double res = 9.0;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("barrior");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("barrior");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

int hwa_set::set_vis::num_particles(int cam_group_id)
{
    int res = 500;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("num_particles");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);
    if (res < 0)
        res = -res;

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("num_particles");
        str_erase(tmp2);
        if (tmp2 != "")
        {
            res = std::stoi(tmp2);
            if (res < 0)
                res = -res;
        }
    }
    return res;
}

double hwa_set::set_vis::kappa_sig(int cam_group_id)
{
    double res = 0.0;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("kappa_sig");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("kappa_sig");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::alpha_sig(int cam_group_id)
{
    double res = 0.001;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("alpha_sig");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("alpha_sig");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::E0(int cam_group_id)
{
    double res = 0.85; // default value
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("e0");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("e0");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::G0(int cam_group_id)
{
    double res = 0.85; // default value
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("g0");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("g0");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

int hwa_set::set_vis::dof1(int cam_group_id)
{
    int res = 15;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("dof1");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("dof1");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::dof2(int cam_group_id)
{
    int res = 1;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("dof2");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("dof2");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

int hwa_set::set_vis::max_iter(int cam_group_id)
{
    int res = 20;
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("max_iter");
    str_erase(tmp);
    if (tmp != "")
        res = std::stoi(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("max_iter");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stoi(tmp2);
    }
    return res;
}

double hwa_set::set_vis::Tau(int cam_group_id)
{
    double res = 1000; // default value
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("tau");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("tau");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::proc_noise(int cam_group_id)
{
    double res = 1000; // default value
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("proc_noise");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("proc_noise");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::static_threshold(int cam_group_id)
{
    double res = 1000; // default value
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("static_threshold");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("static_threshold");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

double hwa_set::set_vis::max_res_norm(int cam_group_id)
{
    double res = 1000; // default value
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("max_res_norm");
    str_erase(tmp);
    if (tmp != "")
        res = std::stod(tmp);

    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp2 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("max_res_norm");
        str_erase(tmp2);
        if (tmp2 != "")
            res = std::stod(tmp2);
    }
    return res;
}

std::string hwa_set::set_vis::filter(int cam_group_id)
{
    std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("filter");
    str_erase(res);
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp1 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("filter");
        str_erase(tmp1);
        if (tmp1 != "")
            res = tmp1;
    }
    if (res == "") res = "ekf";
    return res;
}

hwa_vis::PROCESSER_TYPE hwa_set::set_vis::processer(int cam_group_id) {
    std::string res = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("processer");
    str_erase(res);
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp1 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("processer");
        str_erase(tmp1);
        if (tmp1 != "")
            res = tmp1;
    }
    if (res == "") res = "cpu";
    return hwa_vis::str2processer(res);
}

std::string hwa_set::set_vis::cam_group_name(int cam_group_id)
{
    std::string res;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp1 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).attribute("name").value();
        str_erase(tmp1);
        if (tmp1 != "")
            res = tmp1;
    }
    return res;
}

std::string hwa_set::set_vis::gst_path(int cam_group_id)
{
    std::string res;
    if (!_doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).empty())
    {
        std::string tmp1 = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_child_by_attribute("cam_group", "id", std::to_string(cam_group_id).c_str()).child_value("gst_path");
        str_erase(tmp1);
        if (tmp1 != "")
            res = tmp1;
    }
    return res;
}

std::vector<int> hwa_set::set_vis::camera_list() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child_value("camera_list");
    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',')
            tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    std::string node;
    std::vector<int> list;
    while (ss >> node)
        list.push_back(std::stoi(node));
    return list;
}

bool hwa_set::set_vis::detect()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("detect");
    str_erase(tmp);
    bool res = false;
    if (tmp != "")
        res = (tmp == "true" || tmp == "1" || tmp == "yes");
    return res;
}

std::string hwa_set::set_vis::modelpath()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("modelpath");
    str_erase(tmp);
    return tmp;
}

std::pair<int, int> hwa_set::set_vis::inputsize()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("inputsize");

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',')
            tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    std::string node0, node1;
    std::pair<int, int> res;
    ss >> node0 >> node1;
    res.first = std::stoi(node0);
    res.second = std::stoi(node1);
    return res;
}

std::vector<std::string> hwa_set::set_vis::clsname()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("clsname");

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',')
            tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    std::string node;
    std::vector<std::string> list;
    while (ss >> node)
        list.push_back(node);
    return list;
}

float hwa_set::set_vis::conf()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("conf");
    str_erase(tmp);
    float res = 0.0f;
    if (tmp != "")
        res = std::stof(tmp);
    return res;
}

float hwa_set::set_vis::nms()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("nms");
    str_erase(tmp);
    float res = 0.0f;
    if (tmp != "")
        res = std::stof(tmp);
    return res;
}

int hwa_set::set_vis::classnumber()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("cls");
    str_erase(tmp);
    int res = 0;
    if (tmp != "")
        res = std::stoi(tmp);
    return res;
}

int hwa_set::set_vis::numpred()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).child(XMLKEY_YOLO).child_value("numpred");
    str_erase(tmp);
    int res = 0;
    if (tmp != "")
        res = std::stoi(tmp);
    return res;
}

int hwa_set::set_vis::group_number() {
    
    return _doc.child(XMLKEY_ROOT).child(XMLKEY_VIS).find_specific_child_number("cam_group");
}