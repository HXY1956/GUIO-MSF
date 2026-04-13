#include "hwa_set_param.h"

hwa_set::set_par::set_par() : set_base()
{
    _set.insert(XMLKEY_PARS);
}

hwa_set::set_par::~set_par()
{
}

double hwa_set::set_par::sigCX()
{
    //get rkf model node
    std::string sigCX = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_GEO).attribute("sigCX").value();

#ifdef DEBUG
    std::cout << "The sigma of CX is : " << sigCX << std::endl;
#endif // DEBUG

    if (sigCX.empty())
    {
        return _default_sigCX;
    }
    else
    {
        return hwa_base::base_type_conv::str2dbl(sigCX);
    }
}

double hwa_set::set_par::sigCY()
{
    //get rkf model node
    std::string sigCY = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_GEO).attribute("sigCY").value();

#ifdef DEBUG
    std::cout << "The sigma of CY is : " << sigCY << std::endl;
#endif // DEBUG

    if (sigCY.empty())
    {

        return _default_sigCY;
    }
    else
    {
        return base_type_conv::str2dbl(sigCY);
    }
}

double hwa_set::set_par::sigCZ()
{
    //get rkf model node
    std::string sigCZ = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_GEO).attribute("sigCZ").value();

#ifdef DEBUG
    std::cout << "The sigma of CZ is : " << sigCZ << std::endl;
#endif // DEBUG

    if (sigCZ.empty())
    {
        return _default_sigCZ;
    }
    else
    {
        return base_type_conv::str2dbl(sigCZ);
    }
}

double hwa_set::set_par::sigXpole()
{
    //get rkf model node
    std::string sigXPOLE = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigXPOLE").value();

#ifdef DEBUG
    std::cout << "The sigma of XPOLE is : " << sigXPOLE << std::endl;
#endif // DEBUG

    if (sigXPOLE.empty())
    {
        return _default_sigXPOLE;
    }
    else
    {
        return base_type_conv::str2dbl(sigXPOLE);
    }
}

double hwa_set::set_par::sigYpole()
{
    //get rkf model node
    std::string sigYPOLE = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigYPOLE").value();

#ifdef DEBUG
    std::cout << "The sigma of YPOLE is : " << sigYPOLE << std::endl;
#endif // DEBUG

    if (sigYPOLE.empty())
    {
        return _default_sigYPOLE;
    }
    else
    {
        return base_type_conv::str2dbl(sigYPOLE);
    }
}

double hwa_set::set_par::sigDxpole()
{
    //get rkf model node
    std::string sigDXPOLE = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigDXPOLE").value();

#ifdef DEBUG
    std::cout << "The sigma of DXPOLE is : " << sigDXPOLE << std::endl;
#endif // DEBUG

    if (sigDXPOLE.empty())
    {
        return _default_sigDXPOLE;
    }
    else
    {
        return base_type_conv::str2dbl(sigDXPOLE);
    }
}

double hwa_set::set_par::sigDypole()
{
    //get rkf model node
    std::string sigDYPOLE = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigDYPOLE").value();

#ifdef DEBUG
    std::cout << "The sigma of DYPOLE is : " << sigDYPOLE << std::endl;
#endif // DEBUG

    if (sigDYPOLE.empty())
    {
        return _default_sigDXPOLE;
    }
    else
    {
        return base_type_conv::str2dbl(sigDYPOLE);
    }
}

double hwa_set::set_par::sigUt1()
{
    //get rkf model node
    std::string sigUT1 = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigUT1").value();

#ifdef DEBUG
    std::cout << "The sigma of UT1 is : " << sigUT1 << std::endl;
#endif // DEBUG

    if (sigUT1.empty())
    {
        return _default_sigUT1;
    }
    else
    {
        return base_type_conv::str2dbl(sigUT1);
    }
}

double hwa_set::set_par::sigUt1_vlbi()
{
    //get rkf model node
    std::string sigUT1_vlbi = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigUT1").value();

#ifdef DEBUG
    std::cout << "The sigma of UT1 is : " << sigUT1 << std::endl;
#endif // DEBUG

    if (sigUT1_vlbi.empty())
    {
        return _default_sigUT1_vlbi;
    }
    else
    {
        return base_type_conv::str2dbl(sigUT1_vlbi);
    }
}

double hwa_set::set_par::sigDX_vlbi()
{
    //get rkf model node
    std::string sigUT1_vlbi = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigDX").value();

#ifdef DEBUG
    std::cout << "The sigma of UT1 is : " << sigUT1 << std::endl;
#endif // DEBUG

    if (sigUT1_vlbi.empty())
    {
        return _default_sigDX;
    }
    else
    {
        return base_type_conv::str2dbl(sigUT1_vlbi);
    }
}

double hwa_set::set_par::sigDY_vlbi()
{
    //get rkf model node
    std::string sigUT1_vlbi = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigDY").value();

#ifdef DEBUG
    std::cout << "The sigma of UT1 is : " << sigUT1 << std::endl;
#endif // DEBUG

    if (sigUT1_vlbi.empty())
    {
        return _default_sigDY;
    }
    else
    {
        return base_type_conv::str2dbl(sigUT1_vlbi);
    }
}

double hwa_set::set_par::sigDut1()
{
    //get rkf model node
    std::string sigDUT1 = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_ERP).attribute("sigDUT1").value();

#ifdef DEBUG
    std::cout << "The sigma of DUT1 is : " << sigDUT1 << std::endl;
#endif // DEBUG

    if (sigDUT1.empty())
    {
        return _default_sigDUT1;
    }
    else
    {
        return base_type_conv::str2dbl(sigDUT1);
    }
}

double hwa_set::set_par::sigZtd(std::string sta_name)
{
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", sta_name.c_str());
    std::string sigZTD = station.attribute("sigZTD").value();

#ifdef DEBUG
    std::cout << "The sigma of ZTD is : " << sigZTD << std::endl;
#endif // DEBUG

    if (sigZTD.empty())
    {
        return _default_sigZTD;
    }
    else
    {
        return base_type_conv::str2dbl(sigZTD);
    }
}

double hwa_set::set_par::sigSion(std::string sta_name)
{
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", sta_name.c_str());
    std::string sigSION = station.attribute("sigSION").value();

#ifdef DEBUG
    std::cout << "The sigma of ZTD is : " << sigSION << std::endl;
#endif // DEBUG

    if (sigSION.empty())
    {
        return _default_sigSION;
    }
    else
    {
        return base_type_conv::str2dbl(sigSION);
    }
}

double hwa_set::set_par::sigVion(std::string sta_name)
{
    return 0.0;
}

double hwa_set::set_par::sigTropPd(std::string sta_name)
{
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", sta_name.c_str());
    std::string sigTropPd = station.attribute("sigTropPd").value();

#ifdef DEBUG
    std::cout << "The sigma of TropPd is : " << sigTropPd << std::endl;
#endif // DEBUG

    if (sigTropPd.empty())
    {
        return _default_sigTropPd;
    }
    else
    {
        return base_type_conv::str2dbl(sigTropPd);
        //    std::cerr<< base_type_conv::str2dbl(sigTropPd);
    }
}

double hwa_set::set_par::sigIonoPd(std::string rec)
{
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", rec.c_str());
    std::string sigIonoPd = station.attribute("sigIonoPd").value();

#ifdef DEBUG
    std::cout << "The sigma of IonoPd is : " << sigIonoPd << std::endl;
#endif // DEBUG

    if (sigIonoPd.empty())
    {
        return _default_sigIonoPd;
    }
    else
    {
        return base_type_conv::str2dbl(sigIonoPd);
    }
}

double hwa_set::set_par::sigGRD(std::string rec)
{
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", rec.c_str());
    std::string sigGRD = station.attribute("sigGRD").value();

#ifdef DEBUG
    std::cout << "The sigma of gradient is : " << sigGRD << std::endl;
#endif // DEBUG

    if (sigGRD.empty())
    {
        return _default_sigGRD;
    }
    else
    {
        return base_type_conv::str2dbl(sigGRD);
    }
}

double hwa_set::set_par::sigGrdPd(std::string rec)
{
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", rec.c_str());
    std::string sigGrdPd = station.attribute("sigGrdPd").value();

#ifdef DEBUG
    std::cout << "The sigma of GrdPd is : " << sigGrdPd << std::endl;
#endif // DEBUG

    if (sigGrdPd.empty())
    {
        return _default_sigGrdPd;
    }
    else
    {
        return base_type_conv::str2dbl(sigGrdPd);
    }
}

double hwa_set::set_par::sigAmb()
{
    //get rkf model node
    std::string sigAMB = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_AMB).attribute("sigAMB").value();

#ifdef DEBUG
    std::cout << "The sigma of AMB is : " << sigAMB << std::endl;
#endif // DEBUG

    if (sigAMB.empty())
    {
        return _default_sigAMB;
    }
    else
    {
        return base_type_conv::str2dbl(sigAMB);
    }
}

double hwa_set::set_par::sigRecCLK(std::string sta_name)
{
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", sta_name.c_str());
    std::string sigCLK = station.attribute("sigCLK").value();

#ifdef DEBUG
    std::cout << "The sigma of CLK is : " << sigCLK << std::endl;
#endif // DEBUG

    if (sigCLK.empty())
    {
        return _default_sta_sigCLK;
    }
    else
    {
        return base_type_conv::str2dbl(sigCLK);
    }
}

double hwa_set::set_par::sigRB()
{
    std::string sta_name = "SLR";
    //get rkf model node
    xml_node station = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", sta_name.c_str());
    std::string sigRB = station.attribute("sigRB").value();

#ifdef DEBUG
    std::cout << "The sigma of SLR is : " << sigRB << std::endl;
#endif // DEBUG

    if (sigRB.empty())
    {
        return _default_sta_sigRB;
    }
    else
    {
        return base_type_conv::str2dbl(sigRB);
    }
}

double hwa_set::set_par::sigSatCLK(std::string sat_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", sat_name.c_str());
    std::string sigCLK = satellite.attribute("sigCLK").value();

#ifdef DEBUG
    std::cout << "The sigma of CLK is : " << sigCLK << std::endl;
#endif // DEBUG

    if (sigCLK.empty())
    {
        xml_node common_node = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute("ID", "XXX");
        std::string common_sigCLK = common_node.attribute("sigCLK").value();

        if (!common_sigCLK.empty())
            return base_type_conv::str2dbl(common_sigCLK);
        return _default_sat_sigCLK;
    }
    else
    {
        return base_type_conv::str2dbl(sigCLK);
    }
}

std::map<std::string, double> hwa_set::set_par::sigRecPos(std::string sta_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_STA, "ID", sta_name.c_str());
    std::string sigPOS = satellite.attribute("sigPOS").value();

#ifdef DEBUG
    std::cout << "The sigma of POS is : " << sigPOS << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigXYZ;
    if (!sigPOS.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigPOS), "_", sigmas);
        if (sigmas.size() == 3)
        {
            sigXYZ.clear();
            sigXYZ.insert(std::pair<std::string, double>("sigPX", base_type_conv::str2dbl(sigmas[0])));
            sigXYZ.insert(std::pair<std::string, double>("sigPY", base_type_conv::str2dbl(sigmas[1])));
            sigXYZ.insert(std::pair<std::string, double>("sigPZ", base_type_conv::str2dbl(sigmas[2])));
            return sigXYZ;
        }
    }


    sigXYZ.clear();
    sigXYZ.insert(std::pair<std::string, double>("sigPX", _default_sta_sigPOS));
    sigXYZ.insert(std::pair<std::string, double>("sigPY", _default_sta_sigPOS));
    sigXYZ.insert(std::pair<std::string, double>("sigPZ", _default_sta_sigPOS));

    return sigXYZ;
}

std::map<std::string, double> hwa_set::set_par::sigSatPos(std::string sat_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_SAT, "ID", sat_name.c_str());
    std::string sigPOS = satellite.attribute("sigPOS").value();

#ifdef DEBUG
    std::cout << "The sigma of POS is : " << sigPOS << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigXYZ;
    if (!sigPOS.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigPOS), "_", sigmas);
        if (sigmas.size() == 3)
        {
            sigXYZ.clear();
            sigXYZ.insert(std::pair<std::string, double>("sigPX", base_type_conv::str2dbl(sigmas[0])));
            sigXYZ.insert(std::pair<std::string, double>("sigPY", base_type_conv::str2dbl(sigmas[1])));
            sigXYZ.insert(std::pair<std::string, double>("sigPZ", base_type_conv::str2dbl(sigmas[2])));
            return sigXYZ;
        }
    }

    sigXYZ.clear();
    sigXYZ.insert(std::pair<std::string, double>("sigPX", _default_sat_sigPOS));
    sigXYZ.insert(std::pair<std::string, double>("sigPY", _default_sat_sigPOS));
    sigXYZ.insert(std::pair<std::string, double>("sigPZ", _default_sat_sigPOS));

    return sigXYZ;
}

//add for LEO processing by zhangwei: sat_name format as site: swaa/graa
std::map<std::string, double> hwa_set::set_par::sigLeoPos(std::string sat_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_STA, "ID", sat_name.c_str());
    std::string sigPOS = satellite.attribute("sigPOS").value();

#ifdef DEBUG
    std::cout << "The sigma of POS is : " << sigPOS << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigXYZ;
    if (!sigPOS.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigPOS), "_", sigmas);
        if (sigmas.size() == 3)
        {
            sigXYZ.clear();
            sigXYZ.insert(std::pair<std::string, double>("sigPX", base_type_conv::str2dbl(sigmas[0])));
            sigXYZ.insert(std::pair<std::string, double>("sigPY", base_type_conv::str2dbl(sigmas[1])));
            sigXYZ.insert(std::pair<std::string, double>("sigPZ", base_type_conv::str2dbl(sigmas[2])));
            return sigXYZ;
        }
    }

    sigXYZ.clear();
    sigXYZ.insert(std::pair<std::string, double>("sigPX", _default_leo_sigPOS));
    sigXYZ.insert(std::pair<std::string, double>("sigPY", _default_leo_sigPOS));
    sigXYZ.insert(std::pair<std::string, double>("sigPZ", _default_leo_sigPOS));

    return sigXYZ;
}

std::map<std::string, double> hwa_set::set_par::sigSatVel(std::string sat_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_SAT, "ID", sat_name.c_str());
    std::string sigVEL = satellite.attribute("sigVEL").value();

#ifdef DEBUG
    std::cout << "The sigma of VEL is : " << sigVEL << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigV;
    if (!sigVEL.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigVEL), "_", sigmas);
        if (sigmas.size() == 3)
        {
            sigV.clear();
            sigV.insert(std::pair<std::string, double>("sigVX", base_type_conv::str2dbl(sigmas[0])));
            sigV.insert(std::pair<std::string, double>("sigVY", base_type_conv::str2dbl(sigmas[1])));
            sigV.insert(std::pair<std::string, double>("sigVZ", base_type_conv::str2dbl(sigmas[2])));
            return sigV;
        }
    }

    sigV.clear();
    sigV.insert(std::pair<std::string, double>("sigVX", _default_sat_sigVEL));
    sigV.insert(std::pair<std::string, double>("sigVY", _default_sat_sigVEL));
    sigV.insert(std::pair<std::string, double>("sigVZ", _default_sat_sigVEL));

    return sigV;
}

//add for LEO processing by zhangwei: sat_name format as site: swaa/graa
std::map<std::string, double> hwa_set::set_par::sigLeoVel(std::string sat_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_STA, "ID", sat_name.c_str());
    std::string sigVEL = satellite.attribute("sigVEL").value();

#ifdef DEBUG
    std::cout << "The sigma of VEL is : " << sigVEL << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigV;
    if (!sigVEL.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigVEL), "_", sigmas);
        if (sigmas.size() == 3)
        {
            sigV.clear();
            sigV.insert(std::pair<std::string, double>("sigVX", base_type_conv::str2dbl(sigmas[0])));
            sigV.insert(std::pair<std::string, double>("sigVY", base_type_conv::str2dbl(sigmas[1])));
            sigV.insert(std::pair<std::string, double>("sigVZ", base_type_conv::str2dbl(sigmas[2])));
            return sigV;
        }
    }

    sigV.clear();
    sigV.insert(std::pair<std::string, double>("sigVX", _default_leo_sigVEL));
    sigV.insert(std::pair<std::string, double>("sigVY", _default_leo_sigVEL));
    sigV.insert(std::pair<std::string, double>("sigVZ", _default_leo_sigVEL));

    return sigV;
}

std::map<std::string, double> hwa_set::set_par::sigSatEcom(std::string sat_name)
{
    // todo : some problem
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_SAT, "ID", sat_name.c_str());

    std::string sigECOM = satellite.attribute("sigECOM").value();

#ifdef DEBUG
    std::cout << "The sigma of ECOM is : " << sigECOM << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigPARS;
    if (!sigECOM.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigECOM), "_", sigmas);
        if (sigmas.size() == 9)
        {
            sigPARS.clear();
            sigPARS.insert(std::pair<std::string, double>("sigD0", base_type_conv::str2dbl(sigmas[0])));
            sigPARS.insert(std::pair<std::string, double>("sigDc", base_type_conv::str2dbl(sigmas[1])));
            sigPARS.insert(std::pair<std::string, double>("sigDs", base_type_conv::str2dbl(sigmas[2])));
            sigPARS.insert(std::pair<std::string, double>("sigY0", base_type_conv::str2dbl(sigmas[3])));
            sigPARS.insert(std::pair<std::string, double>("sigYc", base_type_conv::str2dbl(sigmas[4])));
            sigPARS.insert(std::pair<std::string, double>("sigYs", base_type_conv::str2dbl(sigmas[5])));
            sigPARS.insert(std::pair<std::string, double>("sigX0", base_type_conv::str2dbl(sigmas[6])));
            sigPARS.insert(std::pair<std::string, double>("sigXc", base_type_conv::str2dbl(sigmas[7])));
            sigPARS.insert(std::pair<std::string, double>("sigXs", base_type_conv::str2dbl(sigmas[8])));
            return sigPARS;
        }
    }

    sigPARS.clear();

    sigPARS.insert(std::pair<std::string, double>("sigD0", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigDc", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigDs", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigY0", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigYc", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigYs", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigX0", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigXc", _default_sat_sigECOM));
    sigPARS.insert(std::pair<std::string, double>("sigXs", _default_sat_sigECOM));

    return sigPARS;
}

std::map<std::string, double> hwa_set::set_par::sigSatAbw(std::string sat_name)
{
    // todo : some problem
    //get rkf model node
    //xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_SAT, "ID", sat_name.c_str());

    ////std::string   sigABW = satellite.attribute("sigABW").value();
    //xml_node test = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_SRP);
    std::string sigABW = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(XMLKEY_PARS_SRP).attribute("sigABW").value();
#ifdef DEBUG
    std::cout << "The sigma of ABW is : " << sigABW << std::endl;
#endif // DEBUG

    std::map<std::string, double> sigPARS;
    if (!sigABW.empty())
    {
        std::vector<std::string> sigmas;
        split(base_type_conv::trim(sigABW), "_", sigmas);
        if (sigmas.size() == 9)
        {
            sigPARS.clear();
            sigPARS.insert(std::pair<std::string, double>("sigSP", base_type_conv::str2dbl(sigmas[0])));
            sigPARS.insert(std::pair<std::string, double>("sigSB", base_type_conv::str2dbl(sigmas[1])));
            sigPARS.insert(std::pair<std::string, double>("sigY0", base_type_conv::str2dbl(sigmas[2])));
            sigPARS.insert(std::pair<std::string, double>("sigPXAD", base_type_conv::str2dbl(sigmas[3])));
            sigPARS.insert(std::pair<std::string, double>("sigPZAD", base_type_conv::str2dbl(sigmas[4])));
            sigPARS.insert(std::pair<std::string, double>("sigNZAD", base_type_conv::str2dbl(sigmas[5])));
            sigPARS.insert(std::pair<std::string, double>("sigPXR", base_type_conv::str2dbl(sigmas[6])));
            sigPARS.insert(std::pair<std::string, double>("sigPZR", base_type_conv::str2dbl(sigmas[7])));
            sigPARS.insert(std::pair<std::string, double>("sigNZR", base_type_conv::str2dbl(sigmas[8])));
            return sigPARS;
        }
    }

    sigPARS.clear();

    sigPARS.insert(std::pair<std::string, double>("sigSP", _default_sat_sigABW_SY));
    sigPARS.insert(std::pair<std::string, double>("sigSB", _default_sat_sigABW_SY));
    sigPARS.insert(std::pair<std::string, double>("sigY0", _default_sat_sigABW_SY));
    sigPARS.insert(std::pair<std::string, double>("sigPXAD", _default_sat_sigABW_AD));
    sigPARS.insert(std::pair<std::string, double>("sigPZAD", _default_sat_sigABW_AD));
    sigPARS.insert(std::pair<std::string, double>("sigNZAD", _default_sat_sigABW_AD));
    sigPARS.insert(std::pair<std::string, double>("sigPXR", _default_sat_sigABW_R));
    sigPARS.insert(std::pair<std::string, double>("sigPZR", _default_sat_sigABW_R));
    sigPARS.insert(std::pair<std::string, double>("sigNZR", _default_sat_sigABW_R));

    return sigPARS;
}

// add for LEO processing
double hwa_set::set_par::sigLeoDyn(std::string sat_name)
{
    //get rkf model node
    xml_node satellite = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(XMLKEY_PARS_STA, "ID", sat_name.c_str());
    std::string sigDYN = satellite.attribute("sigDyn").value();

#ifdef DEBUG
    std::cout << "The sigma of LEO dynamic parameters is : " << sigDYN << std::endl;
#endif // DEBUG

    double sigPAR;
    if (!sigDYN.empty())
    {
        sigPAR = base_type_conv::str2dbl(sigDYN);
        return sigPAR;
    }

    sigPAR = _default_leo_sigDYN;
    return sigPAR;
}

void hwa_set::set_par::help()
{
    std::cerr << "<!--> One example for this block : <!-->                       " << std::endl
         << "<!--> The index of sigECOM is sigD0,sigDc,sigDs,sigY0,sigYC,sigYs,sigB0,sigBc,sigBs. <!-->" << std::endl
         << "<parameters>" << std::endl
         << "<GEO   sigCX    = \"0.001\"  sigCY    = \"0.001\" sigCZ  = \"0.001\" / >" << std::endl
         << "<ERP   sigXPOLE = \"0.300\"  sigYPOLE = \"0.030\" sigUT1 = \"0.030\"  sigDXPOLE = \"0.0001\" sigDYPOLE = \"0.002\"  sigDUT1 = \"0.002\" />     " << std::endl
         << "<station   ID = \"AIRA\" sigCLK = \"9000\" sigZTD = \"0.201\"    sigION = \"9000\"     sigPOS = \"0.1_0.1_0.1\" / >" << std::endl
         << "<satellite ID = \"G01\"  sigCLK = \"5000\" sigPOS = \"10_10_10\" sigVEL = \"10_10_10\"  sigECOM = \"0.1_0.1_0.1_0.1_0.1_0.1_0.1_0.1_0.1\"/>  " << std::endl
         << "< / parameters>                          " << std::endl
         << std::endl;
}

std::string hwa_set::set_par::_child_value(const std::string &child)
{
    return _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child_value(child.c_str());
}

std::string hwa_set::set_par::_attribute_value(const std::string &index_name, const std::string &index_value, const std::string &attribute)
{
    xml_node index = _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).find_child_by_attribute(index_name.c_str(), index_value.c_str());
    if (index)
    {
        return index.attribute(attribute.c_str()).value();
    }
    else
    {
        return std::string();
    }
}

std::string hwa_set::set_par::_child_attribute_value(const std::string &child, const std::string &attribute)
{
    return _doc.child(XMLKEY_ROOT).child(XMLKEY_PARS).child(child.c_str()).attribute(attribute.c_str()).value();
}

void hwa_set::set_par::split(const std::string &s, std::string delim, std::vector<std::string> &ret)
{
    size_t last = 0;
    size_t index = s.find_first_of(delim, last);

    while (index != std::string::npos)
    {
        ret.push_back(s.substr(last, index - last));
        last = index + 1;
        index = s.find_first_of(delim, last);
    }
    if (index - last > 0)
    {
        ret.push_back(s.substr(last, index - last));
    }
}
