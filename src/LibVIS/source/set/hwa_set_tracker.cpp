#include "hwa_set_tracker.h"
using namespace hwa_set;

set_tracker::set_tracker(hwa_set::set_base* gset) :
    hwa_set::set_base()
{
    Tracker_Rot_noise = dynamic_cast<set_tracker*>(gset)->rot_noise();
    Tracker_Trans_noise = dynamic_cast<set_tracker*>(gset)->trans_noise();
}

void set_tracker::check()
{
    xml_node parent = _doc.child(XMLKEY_ROOT);
    xml_node node = _default_node(parent, XMLKEY_TRACKER);
    _default_attr(node, "rot_noise", 1e-5);
    _default_attr(node, "trans_noise", 1e-4);
}

int set_tracker::PickRate()
{
    int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("pickrate").as_int();
    return res;
}

int set_tracker::camskip()
{
    int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("camskip").as_int();
    return res;
}

double set_tracker::rot_noise()
{
    double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("rot_noise").as_double();
    return res;
}

double set_tracker::trans_noise()
{
    double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("trans_noise").as_double();
    return res;
}

double set_tracker::mindist()
{
    double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("mindist").as_double();
    return res;
}

double set_tracker::interval()
{
    double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("interval").as_double();
    return res;
}

double set_tracker::canny()
{
    double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("canny").as_double();
    return res;
}

double set_tracker::acc()
{
    double res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("acc").as_double();
    return res;
}

int set_tracker::minradius()
{
    int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("minradius").as_int();
    return res;
}

int set_tracker::maxradius()
{
    int res = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("maxradius").as_int();
    return res;
}

std::string set_tracker::type()
{
    std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("type").value();
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    return str;
}


SO3 set_tracker::R_tracker_imu()
{
    std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("R_t_i").value();
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == ',')str[i] = ' ';
    }
    std::stringstream ss(str);
    SO3 res;
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
    ss >> R11 >> R12 >> R13 >> R21 >> R22 >> R23 >> R31 >> R32 >> R33;
    res << R11, R12, R13, R21, R22, R23, R31, R32, R33;
    return res;
}

Triple set_tracker::T_tracker_imu()
{
    std::string str = _doc.child(XMLKEY_ROOT).child(XMLKEY_TRACKER).attribute("t_t_i").value();
    for (int i = 0; i < str.size(); i++)
    {
        if (str[i] == ',')str[i] = ' ';
    }
    std::stringstream ss(str);
    SO3 res;
    double X, Y, Z;
    ss >> X >> Y >> Z;
    return Triple(X, Y, Z);
}