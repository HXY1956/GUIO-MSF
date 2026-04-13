#include "hwa_set_proc.h"
using namespace hwa_set;

set_proc::set_proc(hwa_set::set_base* gset) :
    hwa_set::set_base()
{
    checkdefault();
}

void set_proc::checkdefault()
{
    xml_node parent = _doc.child(XMLKEY_ROOT);
    xml_node node = _default_node(parent, XMLKEY_PROCESS);
}

bool set_proc::TimeCostDebug()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_PROCESS).child_value("timecost");
    str_erase(tmp);
    bool res = false;
    if (tmp != "")
        res = (tmp == "true" || tmp == "1" || tmp == "yes");
    return res;
}

double set_proc::insinterval()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_PROCESS).child_value("InsInterval");
    str_erase(tmp);
    double res = 0.0;
    if (tmp != "")
        res = std::stod(tmp);
    return res;
}

double set_proc::visinterval()
{
    std::string tmp = _doc.child(XMLKEY_ROOT).child(XMLKEY_PROCESS).child_value("VisInterval");
    str_erase(tmp);
    double res = 0.0;
    if (tmp != "")
        res = std::stod(tmp);
    return res;
}