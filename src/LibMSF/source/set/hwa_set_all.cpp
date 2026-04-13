#include "hwa_set_all.h"

hwa_set::set_msf::set_msf() :
    hwa_set::set_ins(),
    hwa_set::set_uwb(),
    hwa_set::set_vis(),
    hwa_set::set_ppp(),
    hwa_set::set_tracker(),
    hwa_set::set_proc(),
	hwa_set::set_lidar(),
    hwa_set::set_ign()
{
    _IFMT_supported.insert(IFMT::RSSIMSG_INP);
    _IFMT_supported.insert(IFMT::UWBMSG_INP);
    _IFMT_supported.insert(IFMT::IMU_INP);
    _IFMT_supported.insert(IFMT::IMU_POS_INP);
    _IFMT_supported.insert(IFMT::ODO_INP);
    _IFMT_supported.insert(IFMT::RSSIMAP_INP);
    _IFMT_supported.insert(IFMT::ATT_INP);
    _IFMT_supported.insert(IFMT::IMAGE_INP);

    _OFMT_supported.insert(INS_OUT);
    _OFMT_supported.insert(INSKF_OUT);
    _OFMT_supported.insert(INSKFPK_OUT);
    _OFMT_supported.insert(RSSIMAP_OUT);
    _OFMT_supported.insert(RSSIMATCH_OUT);
}

hwa_set::set_msf::~set_msf()
{
}

void hwa_set::set_msf::check()
{
    hwa_set::set_ins::check();
    hwa_set::set_uwb::check();
    hwa_set::set_vis::check();
    hwa_set::set_ppp::check();
    hwa_set::set_lidar::check();
    hwa_set::set_tracker::check();
    hwa_set::set_proc::check();
    hwa_set::set_ign::check();
}

void hwa_set::set_msf::help()
{
    hwa_set::set_ins::help();
    hwa_set::set_uwb::help();
    hwa_set::set_vis::help();
    hwa_set::set_ppp::help();
   // hwa_set::set_lidar::help();
    hwa_set::set_tracker::help();
    hwa_set::set_proc::help();
    hwa_set::set_ign::help();
}
