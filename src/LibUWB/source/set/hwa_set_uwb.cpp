#include "hwa_set_uwb.h"
using namespace hwa_set;

set_uwb::set_uwb() :
    set_base(), _com("COM9 COM10 COM11"),
    _nq(3), _move(true), _ts(1), _sample(10.0), _pos(Triple::Zero()),
    _initial_pos_std(Triple::Zero()), _pos_psd(0.1),
    _wgt_type(UWB_WEIGHT::EQUAL), _order("rangenet"),
    /*_dim(3),*/ _iter(20), _anchor_list()
{
    _set.insert(XMLKEY_UWB);
}

void set_uwb::check()
{
    xml_node parent = _doc.child(XMLKEY_ROOT);
    xml_node node = _default_node(parent, XMLKEY_UWB);
    std::string str = std::to_string(_pos(0)) + ' ' + std::to_string(_pos(1)) + ' ' + std::to_string(_pos(2));
    _default_attr(node, "pos", str);

    str.clear();  str = "";
    str = std::to_string(_initial_pos_std(0)) + ' ' + std::to_string(_initial_pos_std(1)) + ' ' + std::to_string(_initial_pos_std(2));
    _default_attr(node, "initial_pos_std", str);
    _default_attr(node, "iter", _iter);
    _default_attr(node, "nq", _nq);
    _default_attr(node, "move", _move);
    _default_attr(node, "ts", _ts);
    _default_attr(node, "sample", _sample);
    _default_attr(node, "pos_psd", _pos_psd);
    _default_attr(node, "weight", uwbweight2str(_wgt_type));
    _default_attr(node, "com", _com);
    _default_attr(node, "order", _order);

    _default_attr(node, "filter", "EKF");
    _default_attr(node, "smooth", "false");
    _default_attr(node, "smooth_point", 5);
    _default_attr(node, "meas_range_std", 0.3);
    _default_attr(node, "best_range_std", 0.1);
    _default_attr(node, "output_res", false);
    _default_attr(node, "range_lim", 100);
    _default_attr(node, "snr_lim", -14);
    _default_attr(node, "max_res_norm", 3);

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
}

void set_uwb::help()
{
}

UWB_WEIGHT set_uwb::str2uwbweight(const std::string& wg) {
    if (wg == "EQUAL") {
        return UWB_WEIGHT::EQUAL;
    }
    else if (wg == "SNR") {
        return UWB_WEIGHT::SNR;
    }
    else if (wg == "ANTIRANGE") {
        return UWB_WEIGHT::ANTIRANGE;
    }
    else if (wg == "SUCCESSRATE") {
        return UWB_WEIGHT::SUCCESSRATE;
    }
    else {
        return UWB_WEIGHT::EQUAL;
    }
}

std::string set_uwb::uwbweight2str(const UWB_WEIGHT& wg)
{
    std::string str;
    switch (wg)
    {
    case UWB_WEIGHT::EQUAL:
        str = "EQUAL"; break;
    case UWB_WEIGHT::SNR:
        str = "SNR"; break;
    case UWB_WEIGHT::ANTIRANGE:
        str = "ANTIRANGE"; break;
    case UWB_WEIGHT::SUCCESSRATE:
        str = "SUCCESSRATE"; break;
    default:
        str = ""; break;
    }
    return str;
}

std::string set_uwb::navarea2str(const NAVAREA& area)
{
    std::string str;
    switch (area)
    {
    case NAVAREA::INDOOR:
        str = "indoor"; break;
    case NAVAREA::OUTDOOR:
        str = "outdoor"; break;
    case NAVAREA::TRANSITION:
        str = "transition"; break;
    default:
        str = ""; break;
    }
    return str;
}

int set_uwb::iter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("iter");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

int set_uwb::nq()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("nq");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

double set_uwb::ts()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("ts");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::sample()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("sample");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

int set_uwb::num_particles()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("num_particles");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    if (tmp_int < 0)
        tmp_int = -tmp_int;
    return tmp_int;
}

Triple set_uwb::pos()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("pos");

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0, Z = 0.0; // default values
    ss >> X >> Y >> Z;
    return Triple(X, Y, Z);
}

Triple set_uwb::initial_pos_std()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("initial_pos_std");

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0, Z = 0.0; // default values
    ss >> X >> Y >> Z;
    return Triple(X, Y, Z);
}

Triple set_uwb::pos_psd()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("pos_psd");

    for (int i = 0; i < tmp.size(); i++)
    {
        if (tmp[i] == ',') tmp[i] = ' ';
    }
    std::stringstream ss(tmp);
    double X = 0.0, Y = 0.0, Z = 0.0; // default values
    ss >> X >> Y >> Z;
    return Triple(X, Y, Z);
}

UWB_WEIGHT set_uwb::wgt_type()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("weight");
    str_erase(tmp);
    transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
    UWB_WEIGHT WG = str2uwbweight(tmp);
    return WG;
}

std::string set_uwb::order()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("order");
    str_erase(tmp);
    return tmp;
}

std::vector<std::string> set_uwb::anchor_list()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("anchor_list");

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

std::vector<std::string> set_uwb::indoor_anchor_list()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("indoor_anchor_list");

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

std::vector<std::string> set_uwb::outdoor_anchor_list()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("outdoor_anchor_list");

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

double set_uwb::start()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("start");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::end()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("end");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

std::string set_uwb::filter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("filter_uwb");
    str_erase(tmp);
    if (tmp.empty())
    {
        tmp = "EKF";
    }
    return tmp;
}

bool set_uwb::smooth()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("smooth");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

bool set_uwb::addnoise()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("addnoise");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

int set_uwb::smooth_point()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("smooth_point");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

double set_uwb::meas_range_std()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("meas_range_std");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::best_range_std()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("best_range_std");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

bool set_uwb::output_res()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("output_res");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

double set_uwb::barrior()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("barrior");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::kappa_sig()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("kappa_sig");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::alpha_sig()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("alpha_sig");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::log_parameter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("log_parameter");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::exp_parameter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("exp_parameter");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::sigmoid_parameter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("sigmoid_parameter");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::sigmoid_threshold()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("sigmoid_threshold");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::tolerance()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("tolerance");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

std::string set_uwb::penal()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("penal");
    str_erase(tmp);
    if (tmp.empty())
    {
        tmp = "";
    }
    return tmp;
}

bool set_uwb::SDP()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("estimate_nlos");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

bool set_uwb::interpolation()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("interpolation");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

double set_uwb::interpolation_noise()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("interpolation_noise");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

bool set_uwb::posterior()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("posterior");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

double set_uwb::gama()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("gama");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::alpha()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("alpha");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

bool set_uwb::pred_constraint()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("pred_constraint");
    str_erase(tmp);
    bool tmp_bool = false; // default value
    if (tmp != "")
        tmp_bool = (tmp == "true" || tmp == "1" || tmp == "yes");
    return tmp_bool;
}

std::string set_uwb::result_file()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("result_file");
    str_erase(tmp);
    if (tmp.empty())
    {
        tmp = "uwb.txt";
    }
    return tmp;
}

std::string set_uwb::res_file()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("res_file");
    str_erase(tmp);
    if (tmp.empty())
    {
        tmp = "res.txt";
    }
    return tmp;
}

double set_uwb::range_lim()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("range_lim");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::snr_lim()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("snr_lim");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::E0()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("e0");
    str_erase(tmp);
    double tmp_double = 0.85; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::G0()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("g0");
    str_erase(tmp);
    double tmp_double = 0.85; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

int set_uwb::dof1()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("dof1");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

int set_uwb::dof2()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("dof2");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

double set_uwb::max_res_norm()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("max_res_norm");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

int set_uwb::max_iter()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("max_iter");
    str_erase(tmp);
    int tmp_int = 0; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

double set_uwb::Tau()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("tau");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_uwb::proc_noise()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_UWB).child_value("proc_noise_uwb");
    str_erase(tmp);
    double tmp_double = 0.0; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

