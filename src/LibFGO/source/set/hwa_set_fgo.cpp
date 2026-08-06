#include "hwa_set_fgo.h"
using namespace hwa_set;

set_fgopara::set_fgopara() :
    set_base()
{
    _pixel_error = 1.0;
    _laser_cloud_error = 1.0;		
    _acc_n = 0.05;	
    _acc_w = 0.001;			
    _gyr_n = 0.005;					
    _gyr_w = 0.0001;			
    _relative_pos_var = 0.1;		
    _relative_rot_var = 0.001;			
    _window_size = 10;
    _set.insert(XMLKEY_FGO);
}

void set_fgopara::check()
{
    xml_node parent = _doc.child(XMLKEY_ROOT);
    xml_node node = _default_node(parent, XMLKEY_FGO);

    _default_attr(node, "pixel_error", _pixel_error);
    _default_attr(node, "laser_cloud_error", _laser_cloud_error);
    _default_attr(node, "acc_n", _acc_n);
    _default_attr(node, "acc_w", _acc_w);
    _default_attr(node, "gyr_n", _gyr_n);
    _default_attr(node, "gyr_w", _gyr_w);
    _default_attr(node, "relative_pos_var", _relative_pos_var);
    _default_attr(node, "relative_rot_var", _relative_rot_var);
    _default_attr(node, "window_size", _window_size);
}

void set_fgopara::help()
{
}

double set_fgopara::pixel_error() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("pixel_error");
    str_erase(tmp);
    double tmp_double = _pixel_error; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::laser_cloud_error() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("laser_cloud_error");
    str_erase(tmp);
    double tmp_double = _laser_cloud_error; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::acc_n() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("acc_n");
    str_erase(tmp);
    double tmp_double = _acc_n; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::acc_w() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("acc_w");
    str_erase(tmp);
    double tmp_double = _acc_w; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::gyr_n() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("gyr_n");
    str_erase(tmp);
    double tmp_double = _gyr_n; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::gyr_w() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("gyr_w");
    str_erase(tmp);
    double tmp_double = _gyr_w; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::relative_pos_var() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("relative_pos_var");
    str_erase(tmp);
    double tmp_double = _relative_pos_var; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

double set_fgopara::relative_rot_var() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("relative_rot_var");
    str_erase(tmp);
    double tmp_double = _relative_rot_var; // default value
    if (tmp != "")
        tmp_double = std::stod(tmp);
    return tmp_double;
}

int set_fgopara::window_size() {
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_FGO).child_value("window_size");
    str_erase(tmp);
    int tmp_int = _window_size; // default value
    if (tmp != "")
        tmp_int = std::stoi(tmp);
    return tmp_int;
}

