#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "hwa_gnss_data_pcv.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_data_interp.h"
#include "hwa_gnss_model_ppp.h"
#include "hwa_base_eigendef.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_pcv::gnss_data_pcv()
        : base_data(),
          _trans(true), // transmitter (default yes)
          _anten(""),   // antenna type
          _ident(""),   // antenna identification
          _svcod(""),   // SVN code
          _method(""),  // calibartion method
          _source(""),  // source of calibration
          _snxcod("")   // sinex code
    {
        _beg.tsys(base_time::GPS);
        _end.tsys(base_time::GPS);
        id_type(base_data::PCV);
        _pcv_noazi = false;
    }

    gnss_data_pcv::gnss_data_pcv(base_log spdlog)
        : base_data(spdlog),
          _trans(true), // transmitter (default yes)
          _anten(""),   // antenna type
          _ident(""),   // antenna identification
          _svcod(""),   // SVN code
          _method(""),  // calibartion method
          _source(""),  // source of calibration
          _snxcod("")   // sinex code
    {
        _beg.tsys(base_time::GPS);
        _end.tsys(base_time::GPS);
        id_type(base_data::PCV);
        _pcv_noazi = false;
    }
    // destructor
    // ----------
    gnss_data_pcv::~gnss_data_pcv() {}

    // return correction ARP/CoM -> phase center
    // zen [rad] - for receiver antenna, it is zenith angle to the satellite
    //           - for satellite antenna, it is nadir angle (receiver zenith minus
    //             geocentric (space) angle btw XYZsat and XYZrec)
    // azi [rad] - for receiver antenna -> need to be implemented !
    //           - for satellite antenna not interesting (only if horizontal eccentricity exists!)
    // ----------
    double gnss_data_pcv::pco(const double &zen, const double &azi, const GFRQ &f)
    {
        double corr = 0.0;
        if (zen > hwa_pi)
        {
            std::ostringstream lg;
            lg << "not valid zenith angle:" << zen * R2D << std::endl;
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, lg.str());
            return corr;
        }

        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        // ---> ONLY TEMPORARY FUNCTION
        //  if( _mappco.find(f) == _mappco.end() ){
        //    GSYS gsys = gnss_sys::gfrq2gsys(f);
        //    if(_gnote) _gnote->mesg(GWARNING,"gpcv","no REC PCV for freq["+t_gfreq::gfreq2str(f)+"], base_type_conv::substitute GPS L1/L2");
        //    f = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys,f) <= 2) ? (gnss_sys::gfrq2freq(gsys,f)) : FREQ_2 );
        //  }
        // ==================================================

        // satellite only PCO (Z-offset) mapped to rec-sat direction (approximated)
        if (_mappco.find(f) != _mappco.end())
        {
            corr = _mappco[f][2] * cos(zen); // for satellite zen should to be zen-alfa
        }

        // SATELLITE HORIZONTAL ECCENTRICITIES NOT YET IMPLEMENTED !!!!
        // RECEIVER PCO NOT YET IMPLEMENTED (azimut/zenith dependent)
        return corr;
    }

    int gnss_data_pcv::pcoS(gnss_data_sats &satdata, Triple &pco, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2)
    {
        switch (lc)
        {
        case OBSCOMBIN::IONO_FREE:
            return pcoS_cmb(satdata, pco, b1, b2);
            break;
        case OBSCOMBIN::RAW_ALL:
        case OBSCOMBIN::RAW_MIX:
        case OBSCOMBIN::RAW_SINGLE:
            return pcoS_raw(satdata, pco, b1);
            break;
        default:
            return -1;
            break;
        }
    }

    int gnss_data_pcv::pcoR(gnss_data_sats &satdata, Triple &pco, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2)
    {
        switch (lc)
        {
        case OBSCOMBIN::IONO_FREE:
            return pcoR_cmb(satdata, pco, b1, b2);
            break;
        case OBSCOMBIN::RAW_ALL:
        case OBSCOMBIN::RAW_MIX:
        case OBSCOMBIN::RAW_SINGLE:
            return pcoR_raw(satdata, pco, b1);
            break;
        default:
            return -1;
            break;
        }
    }

    int gnss_data_pcv::pcvS(double &corr, gnss_data_sats &satdata, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2, Triple &site)
    {
        switch (lc)
        {
        case OBSCOMBIN::IONO_FREE:
            return pcvS_cmb(corr, satdata, b1, b2, site);
            break;
        case OBSCOMBIN::RAW_ALL:
        case OBSCOMBIN::RAW_MIX:
        case OBSCOMBIN::RAW_SINGLE:
            return pcvS_raw(corr, satdata, b1, site);
            break;
        default:
            return -1;
            break;
        }
    }

    int gnss_data_pcv::pcvR(double &corr, gnss_data_sats &satdata, OBSCOMBIN lc, GOBSBAND &b1, GOBSBAND &b2)
    {
        switch (lc)
        {
        case OBSCOMBIN::IONO_FREE:
            return pcvR_cmb(corr, satdata, b1, b2);
            break;
        case OBSCOMBIN::RAW_ALL:
        case OBSCOMBIN::RAW_MIX:
        case OBSCOMBIN::RAW_SINGLE:
            return pcvR_raw(corr, satdata, b1);
            break;
        default:
            return -1;
            break;
        }
    }

    int gnss_data_pcv::pcoS_cmb(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1, GOBSBAND &b2)
    {

        GSYS gsys = satdata.gsys();
        std::string sat = satdata.sat();

        Triple apcf1, apcf2, apcLC;

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);
        GFRQ f2 = gnss_sys::band2gfrq(gsys, b2);

        // GFRQ def_f2 = gnss_sys::freq_priority(gsys, FREQ_2);

        // FREQ_SEQ freq1 = gnss_sys::gfrq2freq(gsys, f1);
        // FREQ_SEQ freq2 = gnss_sys::gfrq2freq(gsys, f2);

        if (_mappco.find(f1) == _mappco.end())
        {
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mappco.find(f2) == _mappco.end())
        {
            f2 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }

        //TODO COMMENT
        if (_mappco.find(f1) != _mappco.end() && _mappco.find(f2) != _mappco.end())
        {
            apcf1 = _mappco.at(f1);
            apcf2 = _mappco.at(f2);
        }
        else
        {
            if (_gnote)
            {
                if (_mappco.find(f1) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], using hardwired values");
                if (_mappco.find(f2) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], using hardwired values");
            }

            hwa_map_gsys_pco offs = gnss_pco_offsets();
            hwa_map_gsys_pco::iterator itSYS = offs.find(gsys);
            if (itSYS != offs.end())
            {
                hwa_map_band_atx::iterator itb1 = itSYS->second.find(b1);
                hwa_map_band_atx::iterator itb2 = itSYS->second.find(b2);

                if (itb1 != itSYS->second.end())
                {
                    apcf1[0] = itb1->second[0];
                    apcf1[1] = itb1->second[1];
                    apcf1[2] = itb1->second[2];
                }

                if (itb2 != itSYS->second.end())
                {
                    apcf2[0] = itb2->second[0];
                    apcf2[1] = itb2->second[1];
                    apcf2[2] = itb2->second[2];
                }
            }
        }

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
            apcf2[i] /= 1000.0;
        }

        // PCO for linear combination

        double koef1 = 0.0;
        double koef2 = 0.0;
        satdata.coef_ionofree(b1, koef1, b2, koef2);
        apcLC = apcf1 * koef1 + apcf2 * koef2;

        pco = apcLC;

#ifdef DEBUG
        std::cout << "PCO calculation for " << satdata.sat() << " " << anten() << " " << epo.str_hms() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl;
        //   int ooo; cin >> ooo;
#endif

        return 1;
    }

    int gnss_data_pcv::pcoR_cmb(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1, GOBSBAND &b2)
    {
        Triple apcf1;
        Triple apcf2;

        GSYS gsys = satdata.gsys();

        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);
        GFRQ f2 = gnss_sys::band2gfrq(gsys, b2);

        // ==================================================
        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }
        if (_mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
            f2 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }

        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }
        if (_mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
            f2 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }
        // ==================================================

        if (_mappco.find(f1) == _mappco.end() || _mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
            {
                if (_mappco.find(f1) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "]");
                }
                if (_mappco.find(f2) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "]");
                }
            }
            return -1;
        }

        apcf1 = _mappco[f1];
        apcf2 = _mappco[f2];

        //  std::cout <<  "JD: pcoR mapS " << anten()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << apcf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << apcf2 << std::endl;

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
            apcf2[i] /= 1000.0;
        }

        // PCO for linear combination

        double koef1 = 0.0;
        double koef2 = 0.0;
        satdata.coef_ionofree(b1, koef1, b2, koef2);
        pco = apcf1 * koef1 + apcf2 * koef2;

#ifdef DEBUG
        std::cout << "PCO calculation for " << anten() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl
#endif
        return 1;
    }

    int gnss_data_pcv::pcvS_cmb(double &corr, gnss_data_sats &satdata, GOBSBAND &b1, GOBSBAND &b2, Triple &site)
    {
        corr = 0.0;
        GSYS gsys = satdata.gsys();

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);
        GFRQ f2 = gnss_sys::band2gfrq(gsys, b2);

        // GFRQ def_f2 = gnss_sys::freq_priority(gsys, FREQ_2);

        // FREQ_SEQ freq1 = gnss_sys::gfrq2freq(gsys, f1);
        // FREQ_SEQ freq2 = gnss_sys::gfrq2freq(gsys, f2);

        if (_mapzen.find(f1) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GSYS L1");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mapzen.find(f2) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GSYS L1");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }

        if (_mapzen.find(f1) == _mapzen.end() || _mapzen.find(f2) == _mapzen.end())
        {
            if (_gnote)
            {
                _gnote->mesg(GWARNING, "gpcv", "no SAT PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "]");
            }
            return -1;
        }

        std::map<double, double> mapDataf1;
        mapDataf1 = _mapzen[f1];
        std::map<double, double> mapDataf2;
        mapDataf2 = _mapzen[f2];

        double eleS = satdata.ele();
        double rS = satdata.satcrd().norm();
        double rR = site.norm();

        double sinz = (rR / rS) * cos(eleS);
        double zen = asin(sinz) * R2D;

        double corrf1 = 0.0;
        double corrf2 = 0.0;

        // jdhuang : fix for L3 and more
        gnss_data_interp interp(_spdlog);

        if (interp.linear(mapDataf1, zen, corrf1) < 0 || interp.linear(mapDataf2, zen, corrf2) < 0)
        {
            return -1;
        }

        //  std::cout <<  "JD: pcvS mapS " << satdata.sat()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << corrf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << corrf2 << std::endl;

        // [mm] -> [m]
        corrf1 /= 1000.0;
        corrf2 /= 1000.0;

        // PCV for linear combination

        double koef1 = 0.0;
        double koef2 = 0.0;
        satdata.coef_ionofree(b1, koef1, b2, koef2);
        corr = corrf1 * koef1 + corrf2 * koef2;

#ifdef DEBUG
        std::cout << "Sat PCV interpolation" << std::endl;
        std::cout << "Nadir = " << zen
             << " corrf1 =  " << corrf1 * 1000 << " mm, "
             << " corrf2 =  " << corrf2 * 1000 << " mm, "
             << " corrLC =  " << corrLC * 1000 << " mm, "
             << " PRN: " << anten() << std::endl;
#endif
        return 1;
    }

    int gnss_data_pcv::pcvR_cmb(double &corr, gnss_data_sats &satdata, GOBSBAND &b1, GOBSBAND &b2)
    {
        GSYS gsys = satdata.gsys();

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);
        GFRQ f2 = gnss_sys::band2gfrq(gsys, b2);

        //GFRQ def_f2 = gnss_sys::freq_priority(gsys, FREQ_2);

        //FREQ_SEQ freq1 = gnss_sys::gfrq2freq(gsys, f1);
        //FREQ_SEQ freq2 = gnss_sys::gfrq2freq(gsys, f2);

        // ==================================================
        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        // jdhuang : fix for muti-GNSS
        if (_mapzen.find(f1) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mapzen.find(f2) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
            f2 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }

        if (_mapzen.find(f1) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mapzen.find(f2) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
            f2 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }

        if (_mapzen.find(f1) == _mapzen.end())
        {
            return -1;
        }
        if (_mapzen.find(f2) == _mapzen.end())
        {
            return -1;
        }

        // ==================================================

        double zen = hwa_pi / 2.0 - satdata.ele();
        zen *= R2D;

        double azi = satdata.azi();
        azi *= R2D;

        double corrf1 = 0.0;
        double corrf2 = 0.0;

        if (_azi_dependent(f1) && _azi_dependent(f2))
        {
            // AZI-dependant calibration available
            base_pair p_az(azi, zen);
            std::map<base_pair, double> mapDataf1;
            std::map<base_pair, double> mapDataf2;

            for (std::map<GFRQ, hwa_map_A>::iterator itGFRQ = _mapazi.begin(); itGFRQ != _mapazi.end(); itGFRQ++)
            {
                GFRQ f = itGFRQ->first;

                // jdhuang
                // 2020.04.07 fix a bug :change  || to &&
                if (f != f1 && f != f2)
                    continue;

                std::map<double, hwa_map_Z>::iterator itA1 = itGFRQ->second.lower_bound(azi);
                if (itA1 == itGFRQ->second.end() || itA1 == itGFRQ->second.begin())
                {
                    return -1;
                }
                std::map<double, hwa_map_Z>::iterator itA2 = itA1;
                itA2--;

                std::map<double, double>::iterator itZ1 = itA1->second.lower_bound(zen);
                if (itZ1 == itA1->second.end() || itZ1 == itA1->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ2 = itZ1;
                itZ2--;

                std::map<double, double>::iterator itZ3 = itA2->second.lower_bound(zen);
                if (itZ3 == itA2->second.end() || itZ3 == itA2->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ4 = itZ3;
                itZ4--;

                base_pair p1(itA1->first, itZ1->first);
                base_pair p2(itA1->first, itZ2->first);
                base_pair p3(itA2->first, itZ3->first);
                base_pair p4(itA2->first, itZ4->first);

                if (f == f1)
                {
                    mapDataf1[p1] = itZ1->second; //std::cout << p1[0] << " : " << p1[1] << "  " << itZ1->second << std::endl;
                    mapDataf1[p2] = itZ2->second; //std::cout << p2[0] << " : " << p2[1] << "  " << itZ2->second << std::endl;
                    mapDataf1[p3] = itZ3->second; //std::cout << p3[0] << " : " << p3[1] << "  " << itZ3->second << std::endl;
                    mapDataf1[p4] = itZ4->second; //std::cout << p4[0] << " : " << p4[1] << "  " << itZ4->second << std::endl;
                }
                else if (f == f2)
                {
                    mapDataf2[p1] = itZ1->second;
                    mapDataf2[p2] = itZ2->second;
                    mapDataf2[p3] = itZ3->second;
                    mapDataf2[p4] = itZ4->second;
                }
                else
                {
                    continue;
                }
            }

            // jdhuang
            gnss_data_interp interp(_spdlog);
            if (interp.bilinear(mapDataf1, p_az, corrf1) < 0 || interp.bilinear(mapDataf2, p_az, corrf2) < 0)
            {
                return -1;
            }
        }
        else
        { // AZI-dependant calibration NOT available (only NOAZI)

            if (_gnote)
            {
                _gnote->mesg(GWARNING, "gpcv", "no REC AZI PCV [" + _anten + "/freq:" + t_gfreq::gfreq2str(f1) + "], just used NOAZI");
            }

            std::map<double, double> mapDataf1;
            std::map<double, double> mapDataf2;

            if (_mapzen.find(f1) == _mapzen.end() || _mapzen.find(f2) == _mapzen.end())
            {
                return -1;
            }

            mapDataf1 = _mapzen[f1];
            mapDataf2 = _mapzen[f2];
            gnss_data_interp interp(_spdlog);
            if (interp.linear(mapDataf1, zen, corrf1) < 0 || interp.linear(mapDataf2, zen, corrf2) < 0)
            {
                return -1;
            }
        }

        corrf1 /= 1000.0;
        corrf2 /= 1000.0;

        // PCV for linear combination
        double koef1 = 0.0;
        double koef2 = 0.0;
        satdata.coef_ionofree(b1, koef1, b2, koef2);
        corr = corrf1 * koef1 + corrf2 * koef2;

#ifdef DEBUG
        std::cout << "Rec PCV interpolation" << std::endl;
        std::cout << "Zenith = " << zen
             << " corrf1 =  " << corrf1 * 1000 << " mm, "
             << " corrf2 =  " << corrf2 * 1000 << " mm, "
             << " corrLC =  " << corrLC * 1000 << " mm, "
             << " Ant: " << anten() << " "
             << " Sat: " << satdata.sat() << std::endl;
        //   int ooo; cin >> ooo;
#endif
        return 1;
    }

    int gnss_data_pcv::pcoS_raw(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1)
    {

        GSYS gsys = satdata.gsys();
        std::string sat = satdata.sat();

        Triple apcf1;

        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);

        // û��pco�Ļ�����ͳһ�õڶ�Ƶ�ʵ�pco����
        if (_mappco.find(f1) == _mappco.end())
        {
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mappco.find(f1) != _mappco.end())
        {
            apcf1 = _mappco.at(f1);
        }
        else
        {
            if (_gnote)
            {
                if (_mappco.find(f1) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], using hardwired values");
            }

            hwa_map_gsys_pco offs = gnss_pco_offsets();
            hwa_map_gsys_pco::iterator itSYS = offs.find(gsys);
            if (itSYS != offs.end())
            {
                hwa_map_band_atx::iterator itb1 = itSYS->second.find(b1);

                if (itb1 != itSYS->second.end())
                {
                    apcf1[0] = itb1->second[0];
                    apcf1[1] = itb1->second[1];
                    apcf1[2] = itb1->second[2];
                }
            }
        }

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
        }

        // PCO for linear combination
        pco = apcf1;

#ifdef DEBUG
        std::cout << "PCO calculation for " << satdata.sat() << " " << anten() << " " << epo.str_hms() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl;
        //   int ooo; cin >> ooo;
#endif

        return 1;
    }

    int gnss_data_pcv::pcoR_raw(gnss_data_sats &satdata, Triple &pco, GOBSBAND &b1)
    {

        Triple apcf1, apcLC;
        GSYS gsys = satdata.gsys();

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);
        // FREQ_SEQ freq = gnss_sys::gfrq2freq(gsys, f1);

        // fix for muti-GNSS
        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L2");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L2");
            f1 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        // ==================================================
        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
            {
                if (_mappco.find(f1) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "]");
                }
            }
            return -1;
        }

        apcf1 = _mappco[f1];

        //  std::cout <<  "JD: pcoR mapS " << anten()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << apcf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << apcf2 << std::endl;

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
        }

        pco = apcf1;

#ifdef DEBUG
        std::cout << "PCO calculation for " << anten() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl
#endif

            return 1;
    }

    int gnss_data_pcv::pcvS_raw(double &corr, gnss_data_sats &satdata, GOBSBAND &b1, Triple &site)
    {

        GSYS gsys = satdata.gsys();

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);

        // GFRQ def_f2 = gnss_sys::freq_priority(gsys, FREQ_2);

        // FREQ_SEQ freq1 = gnss_sys::gfrq2freq(gsys, f1);

        if (_mapzen.find(f1) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GSYS L1");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        // jdhuang : fix for RAW ALL
        if (_mapzen.find(f1) == _mapzen.end())
        {

            return -1;
        }

        // for zenith angle at satellite-side, yqyuan
        double eleS = satdata.ele();
        double rS = satdata.satcrd().norm();
        double rR = site.norm();
        double sinz = (rR / rS) * cos(eleS);
        double zen = asin(sinz) * R2D;
        // for possible azimuth amgle at satellite-side, yqyuan for Galileo and QZSS
        double azi = satdata.azi_sat() * R2D;

        // azimuth and zenith denpendent PCV at satellite, yqyuan for Galileo and QZSS
        double corrf1 = 0.0;
        if (_azi_dependent(f1))
        {
            base_pair p_az(azi, zen);
            std::map<base_pair, double> mapDataf1;

            for (std::map<GFRQ, hwa_map_A>::iterator itGFRQ = _mapazi.begin(); itGFRQ != _mapazi.end(); itGFRQ++)
            {
                GFRQ f = itGFRQ->first;

                if (f != f1)
                    continue;

                std::map<double, hwa_map_Z>::iterator itA1 = itGFRQ->second.lower_bound(azi);
                if (itA1 == itGFRQ->second.end() || itA1 == itGFRQ->second.begin())
                {
                    return -1;
                }
                std::map<double, hwa_map_Z>::iterator itA2 = itA1;
                itA2--;

                std::map<double, double>::iterator itZ1 = itA1->second.lower_bound(zen);
                if (itZ1 == itA1->second.end() || itZ1 == itA1->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ2 = itZ1;
                itZ2--;

                std::map<double, double>::iterator itZ3 = itA2->second.lower_bound(zen);
                if (itZ3 == itA2->second.end() || itZ3 == itA2->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ4 = itZ3;
                itZ4--;

                base_pair p1(itA1->first, itZ1->first);
                base_pair p2(itA1->first, itZ2->first);
                base_pair p3(itA2->first, itZ3->first);
                base_pair p4(itA2->first, itZ4->first);

                if (f == f1)
                {
                    mapDataf1[p1] = itZ1->second; //std::cout << p1[0] << " : " << p1[1] << "  " << itZ1->second << std::endl;
                    mapDataf1[p2] = itZ2->second; //std::cout << p2[0] << " : " << p2[1] << "  " << itZ2->second << std::endl;
                    mapDataf1[p3] = itZ3->second; //std::cout << p3[0] << " : " << p3[1] << "  " << itZ3->second << std::endl;
                    mapDataf1[p4] = itZ4->second; //std::cout << p4[0] << " : " << p4[1] << "  " << itZ4->second << std::endl;
                }
                else
                {
                    continue;
                }
            }
            // jdhuang
            gnss_data_interp interp(_spdlog);
            if (interp.bilinear(mapDataf1, p_az, corrf1) < 0)
            {
                return -1;
            }
        }
        // only zenith denpendent PCV at satellite, yqyuan
        else
        {
            std::map<double, double> mapDataf1;
            mapDataf1 = _mapzen.at(f1);

            // jdhuang : fix for L3 and more
            gnss_data_interp interp(_spdlog);
            if (interp.linear(mapDataf1, zen, corrf1) < 0)
            {
                return -1;
            }
        }
        /*double eleS = satdata.ele();
    double rS = satdata.satcrd().norm();
    double rR = site.norm();*/
        //double sinz = (rR / rS)*cos(eleS);
        //double zen = asin(sinz) * R2D;
        //double corrf1 = 0.0;
        //// jdhuang : fix for L3 and more
        //gnss_data_interp interp;
        //if (interp.linear(mapDataf1, zen, corrf1) < 0)
        //{
        //    return -1;
        //}
        //  std::cout <<  "JD: pcvS mapS " << satdata.sat()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << corrf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << corrf2 << std::endl;

        // [mm] -> [m]
        corrf1 /= 1000.0;

        // PCV for linear combination
        corr = corrf1;

#ifdef DEBUG
        std::cout << "Sat PCV interpolation" << std::endl;
        std::cout << "Nadir = " << zen
             << " corrf1 =  " << corrf1 * 1000 << " mm, "
             << " corrf2 =  " << corrf2 * 1000 << " mm, "
             << " corrLC =  " << corrLC * 1000 << " mm, "
             << " PRN: " << anten() << std::endl;
#endif

        return 1;
    }

    int gnss_data_pcv::pcvR_raw(double &corr, gnss_data_sats &satdata, GOBSBAND &b1)
    {
        GSYS gsys = satdata.gsys();

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::band2gfrq(gsys, b1);

        // ==================================================
        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        // jdhuang : fix for Raw all
        if (_mapzen.find(f1) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(gsys, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mapzen.find(f1) == _mapzen.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f1) <= 2) ? gnss_sys::gfrq2freq(gsys, f1) : FREQ_2);
        }

        if (_mapzen.find(f1) == _mapzen.end())
        {
            return -1;
        }

        // ==================================================

        double zen = hwa_pi / 2.0 - satdata.ele();
        zen *= R2D;

        double azi = satdata.azi();
        azi *= R2D;

        double corrf1 = 0.0;
        // double corrf2 = 0.0;
        // double corrf3 = 0.0;
        // double corrf4 = 0.0;
        // double corrf5 = 0.0;

        if (_azi_dependent(f1))
        {
            // AZI-dependant calibration available
            base_pair p_az(azi, zen);
            std::map<base_pair, double> mapDataf1;
            for (std::map<GFRQ, hwa_map_A>::iterator itGFRQ = _mapazi.begin(); itGFRQ != _mapazi.end(); itGFRQ++)
            {
                GFRQ f = itGFRQ->first;

                // jdhuang
                // 2020.04.07 fix a bug :change  || to &&
                if (f != f1)
                    continue;

                // if(f != f1 && f != f2) continue;
                std::map<double, hwa_map_Z>::iterator itA1 = itGFRQ->second.lower_bound(azi);
                if (itA1 == itGFRQ->second.end() || itA1 == itGFRQ->second.begin())
                {
                    return -1;
                }
                std::map<double, hwa_map_Z>::iterator itA2 = itA1;
                itA2--;

                std::map<double, double>::iterator itZ1 = itA1->second.lower_bound(zen);
                if (itZ1 == itA1->second.end() || itZ1 == itA1->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ2 = itZ1;
                itZ2--;

                std::map<double, double>::iterator itZ3 = itA2->second.lower_bound(zen);
                if (itZ3 == itA2->second.end() || itZ3 == itA2->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ4 = itZ3;
                itZ4--;

                base_pair p1(itA1->first, itZ1->first);
                base_pair p2(itA1->first, itZ2->first);
                base_pair p3(itA2->first, itZ3->first);
                base_pair p4(itA2->first, itZ4->first);

                if (f == f1)
                {
                    mapDataf1[p1] = itZ1->second; //std::cout << p1[0] << " : " << p1[1] << "  " << itZ1->second << std::endl;
                    mapDataf1[p2] = itZ2->second; //std::cout << p2[0] << " : " << p2[1] << "  " << itZ2->second << std::endl;
                    mapDataf1[p3] = itZ3->second; //std::cout << p3[0] << " : " << p3[1] << "  " << itZ3->second << std::endl;
                    mapDataf1[p4] = itZ4->second; //std::cout << p4[0] << " : " << p4[1] << "  " << itZ4->second << std::endl;
                }
                else
                {
                    continue;
                }
            }

            // jdhuang
            gnss_data_interp interp(_spdlog);
            if (interp.bilinear(mapDataf1, p_az, corrf1) < 0)
            {
                return -1;
            }
        }
        else
        { // AZI-dependant calibration NOT available (only NOAZI)

            if (_gnote)
            {
                _gnote->mesg(GWARNING, "gpcv", "no REC AZI PCV [" + _anten + "/freq:" + t_gfreq::gfreq2str(f1) + "], just used NOAZI");
            }

            std::map<double, double> mapDataf1;
            if (_mapzen.find(f1) != _mapzen.end())
            {
                mapDataf1 = _mapzen[f1];
            }
            else
            {
                return -1;
            }

            gnss_data_interp interp(_spdlog);
            if (interp.linear(mapDataf1, zen, corrf1) < 0)
            {
                return -1;
            }
        }

        //  std::cout <<  "JD: pcvR mapS " << anten()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << corrf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << corrf2 << std::endl;

        // [mm] -> [m]
        corrf1 /= 1000.0;

        corr = corrf1;

#ifdef DEBUG
        std::cout << "Rec PCV interpolation" << std::endl;
        std::cout << "Zenith = " << zen
             << " corrf1 =  " << corrf1 * 1000 << " mm, "
             << " corrf2 =  " << corrf2 * 1000 << " mm, "
             << " corrLC =  " << corrLC * 1000 << " mm, "
             << " Ant: " << anten() << " "
             << " Sat: " << satdata.sat() << std::endl;
        //   int ooo; cin >> ooo;
#endif

        return 1;
    }

    // Update satellite coordinates according to pco
    // -----------------------------------------------
    int gnss_data_pcv::pcoS(gnss_data_sats &satdata, Triple &pco, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2)
    {
        base_time epo = satdata.epoch();
        GSYS gsys = satdata.gsys();
        std::string sat = satdata.sat();

        Triple apcf1, apcf2, apcf3, apcf4, apcf5, apcLC;

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GOBSBAND b1 = gnss_sys::band_priority(gsys, FREQ_1);
        GFRQ f1 = gnss_sys::freq_priority(gsys, FREQ_1);
        GOBSBAND b2 = gnss_sys::band_priority(gsys, FREQ_2);
        GFRQ f2 = gnss_sys::freq_priority(gsys, FREQ_2);
        GOBSBAND b3 = gnss_sys::band_priority(gsys, FREQ_3);
        GFRQ f3 = gnss_sys::freq_priority(gsys, FREQ_3);
        GOBSBAND b4 = gnss_sys::band_priority(gsys, FREQ_4);
        GFRQ f4 = gnss_sys::freq_priority(gsys, FREQ_4);
        GOBSBAND b5 = gnss_sys::band_priority(gsys, FREQ_5);
        GFRQ f5 = gnss_sys::freq_priority(gsys, FREQ_5);

        if (_mappco.find(f1) != _mappco.end() && _mappco.find(f2) != _mappco.end())
        {
            apcf1 = _mappco.at(f1);
            apcf2 = _mappco.at(f2);
            apcf3 = (_mappco.find(f3) != _mappco.end()) ? _mappco.at(f3) : _mappco.at(f2);
            apcf4 = (_mappco.find(f4) != _mappco.end()) ? _mappco.at(f4) : _mappco.at(f2);
            apcf5 = (_mappco.find(f5) != _mappco.end()) ? _mappco.at(f5) : _mappco.at(f2);
            //    std::cout <<  "JD: pcoS mapS " << satdata.sat()
            //                       << " " << t_gfreq::gfreq2str(f1) << " " << apcf1
            //                       << " " << t_gfreq::gfreq2str(f2) << " " << apcf2 << std::endl;
        }
        else
        {
            if (_gnote)
            {
                if (_mappco.find(f1) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], using hardwired values");
                if (_mappco.find(f2) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], using hardwired values");
                if (_mappco.find(f3) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f3) + "], using hardwired values");
                if (_mappco.find(f4) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f4) + "], using hardwired values");
                if (_mappco.find(f5) != _mappco.end())
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCO [" + _anten + "/" + t_gfreq::gfreq2str(f5) + "], using hardwired values");
            }

            hwa_map_gsys_pco offs = gnss_pco_offsets();
            hwa_map_gsys_pco::iterator itSYS = offs.find(gsys);
            if (itSYS != offs.end())
            {
                hwa_map_band_atx::iterator itb1 = itSYS->second.find(b1);
                hwa_map_band_atx::iterator itb2 = itSYS->second.find(b2);
                hwa_map_band_atx::iterator itb3 = itSYS->second.find(b3);
                hwa_map_band_atx::iterator itb4 = itSYS->second.find(b4);
                hwa_map_band_atx::iterator itb5 = itSYS->second.find(b5);

                if (itb1 != itSYS->second.end())
                {
                    apcf1[0] = itb1->second[0];
                    apcf1[1] = itb1->second[1];
                    apcf1[2] = itb1->second[2];
                }

                if (itb2 != itSYS->second.end())
                {
                    apcf2[0] = itb2->second[0];
                    apcf2[1] = itb2->second[1];
                    apcf2[2] = itb2->second[2];
                }

                if (itb3 != itSYS->second.end())
                {
                    apcf3[0] = itb3->second[0];
                    apcf3[1] = itb3->second[1];
                    apcf3[2] = itb3->second[2];
                }

                if (itb4 != itSYS->second.end())
                {
                    apcf4[0] = itb4->second[0];
                    apcf4[1] = itb4->second[1];
                    apcf4[2] = itb4->second[2];
                }

                if (itb5 != itSYS->second.end())
                {
                    apcf5[0] = itb5->second[0];
                    apcf5[1] = itb5->second[1];
                    apcf5[2] = itb5->second[2];
                }
            }
        }

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
            apcf2[i] /= 1000.0;
            apcf3[i] /= 1000.0;
            apcf4[i] /= 1000.0;
            apcf5[i] /= 1000.0;
        }

        // PCO for linear combination
        if (lc == LC_IF)
        {
            double koef1 = 0.0;
            double koef2 = 0.0;
            // modified by glfeng for multi sysyem
            // in gnut GAL noly have BAND_1/5/6/7/8 BDS only have BAND_2/7/6
            //satdata.coef_ionofree(BAND_1, koef1, BAND_2, koef2);
            satdata.coef_ionofree(k1, koef1, k2, koef2);
            apcLC = apcf1 * koef1 + apcf2 * koef2;

            // jdhuang:
            // add L3 PCO
        }
        else if (lc == LC_L1)
        {
            apcLC = apcf1;
        }
        else if (lc == LC_L2)
        {
            apcLC = apcf2;
        }
        else if (lc == LC_L3)
        {
            apcLC = apcf3;
        }
        else if (lc == LC_L4)
        {
            apcLC = apcf4;
        }
        else if (lc == LC_L5)
        {
            apcLC = apcf5;
        }
        else
        {
            apcLC = apcf2;
        }

        pco = apcLC;

#ifdef DEBUG
        std::cout << "PCO calculation for " << satdata.sat() << " " << anten() << " " << epo.str_hms() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl;
//   int ooo; cin >> ooo;
#endif

        return 1;
    }

    // Receiver pco
    // -----------------------------------------------
    int gnss_data_pcv::pcoR(gnss_data_sats &satdata, Triple &pco, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2)
    {
        Triple apcf1, apcf2, apcLC;

        GSYS gsys = satdata.gsys();

        // Temporary - GAL site antenna calibration not available
        //  if(gsys == GAL) gsys = GPS; // JD 2018/09: was valid only for Galileo, below for any GNSS

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::freq_priority(gsys, FREQ_1);
        GFRQ f2 = gnss_sys::freq_priority(gsys, FREQ_2);

        // ==================================================
        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        _gnote = nullptr;
        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(GPS, (FREQ_1));
        }
        if (_mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
            f2 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }
        // ==================================================

        if (_mappco.find(f1) == _mappco.end() || _mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
            {
                if (_mappco.find(f1) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "]");
                }
                if (_mappco.find(f2) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "]");
                }
            }
            return -1;
        }

        apcf1 = _mappco[f1];
        apcf2 = _mappco[f2];

        //  std::cout <<  "JD: pcoR mapS " << anten()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << apcf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << apcf2 << std::endl;

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
            apcf2[i] /= 1000.0;
        }

        // PCO for linear combination
        if (lc == LC_IF)
        {
            double koef1 = 0.0;
            double koef2 = 0.0;
            // modified by glfeng for multi sysyem
            // in gnut GAL noly have BAND_1/5/6/7/8 BDS only have BAND_2/7/6
            //satdata.coef_ionofree(BAND_1, koef1, BAND_2, koef2);

            satdata.coef_ionofree(k1, koef1, k2, koef2);
            apcLC = apcf1 * koef1 + apcf2 * koef2;
        }
        else if (lc == LC_L1)
        {
            apcLC = apcf1;
        }
        else if (lc == LC_L2)
        {
            apcLC = apcf2;
        }
        else
            apcLC = apcf2; // jdhuang : when L3, use the pco for L2

        pco = apcLC;

#ifdef DEBUG
        std::cout << "PCO calculation for " << anten() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl
#endif
        return 1;
    }

    // Receiver pco
    // -----------------------------------------------
    int gnss_data_pcv::pcoR(gnss_data_sats &satdata, Triple &dx, Triple &site, GOBS_LC lc)
    {
        Triple apcf1, apcf2, apcLC;

        GSYS gsys = satdata.gsys();

        // Temporary - GAL site antenna calibration not available
        //  if(gsys == GAL) gsys = GPS; // JD 2018/09: was valid only for Galileo, below for any GNSS

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::freq_priority(gsys, FREQ_1);
        GFRQ f2 = gnss_sys::freq_priority(gsys, FREQ_2);
        GOBSBAND b1, b2; //add by glfeng

        // ==================================================
        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        if (_mappco.find(f1) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
            f1 = gnss_sys::freq_priority(GPS, (FREQ_1));
        }
        if (_mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
                _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
            f2 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
        }
        // ==================================================

        if (_mappco.find(f1) == _mappco.end() || _mappco.find(f2) == _mappco.end())
        {
            if (_gnote)
            {
                if (_mappco.find(f1) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "]");
                }
                if (_mappco.find(f2) == _mappco.end())
                {
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCO [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "]");
                }
            }
            return -1;
        }

        apcf1 = _mappco[f1];
        apcf2 = _mappco[f2];

        //  std::cout <<  "JD: pcoR mapS " << anten()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << apcf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << apcf2 << std::endl;

        for (int i = 0; i <= 2; i++)
        {
            apcf1[i] /= 1000.0;
            apcf2[i] /= 1000.0;
        }

        // PCO for linear combination
        if (lc == LC_IF)
        {
            double koef1 = 0.0;
            double koef2 = 0.0;
            // modified by glfeng for multi sysyem
            // in gnut GAL noly have BAND_1/5/6/7/8 BDS only have BAND_2/7/6
            //satdata.coef_ionofree(BAND_1, koef1, BAND_2, koef2);
            b1 = gnss_sys::band_priority(gsys, FREQ_1);
            b2 = gnss_sys::band_priority(gsys, FREQ_2);
            satdata.coef_ionofree(b1, koef1, b2, koef2);
            apcLC = apcf1 * koef1 + apcf2 * koef2;
        }
        else if (lc == LC_L1)
        {
            apcLC = apcf1;
        }
        else if (lc == LC_L2)
        {
            apcLC = apcf2;
        }
        else
            apcLC = apcf2;

        Triple ell(0.0, 0.0, 0.0);
        xyz2ell(site, ell, false);
        neu2xyz(ell, apcLC, dx);

#ifdef DEBUG
        std::cout << "PCO calculation for " << anten() << std::endl;
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pco f1: " << apcf1 << " m" << std::endl
             << "pco f2: " << apcf2 << " m" << std::endl
             << "pco lc: " << apcLC << " m" << std::endl
             << std::fixed << std::setprecision(5)
             << "Sat dX:\n"
             << dx << std::endl;
#endif
        return 1;
    }

    // return type
    // ---------------
    int gnss_data_pcv::pco_proj(double &corr, gnss_data_sats &satdata, Triple &site, Triple &dx)
    {

        Triple satcrd = satdata.satcrd();
        Triple SatRec(0.0, 0.0, 0.0);
        SatRec = satcrd - site;
        SatRec /= SatRec.norm();

        corr = dx.dot(SatRec);

        return 1;
    }

    // return type
    // ----------
    int gnss_data_pcv::pcvS(double &corrLC, gnss_data_sats &satdata, Triple &site, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2)
    {
        GSYS gsys = satdata.gsys();

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::freq_priority(gsys, FREQ_1);
        GFRQ f2 = gnss_sys::freq_priority(gsys, FREQ_2);
        GFRQ f3 = gnss_sys::freq_priority(gsys, FREQ_3); // jdhuang add L1, L2, L3
        GFRQ f4 = gnss_sys::freq_priority(gsys, FREQ_4);
        GFRQ f5 = gnss_sys::freq_priority(gsys, FREQ_5);

        // jdhuang : fix for RAW ALL
        if (lc == GOBS_LC::LC_IF)
        {
            if (_mapzen.find(f1) == _mapzen.end() || _mapzen.find(f2) == _mapzen.end())
            {
                if (_gnote)
                {
                    _gnote->mesg(GWARNING, "gpcv", "no SAT PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "]");
                }
                return -1;
            }
        }

        // jdhuang : fix for RAW ALL
        if (lc == GOBS_LC::LC_L1 || lc == GOBS_LC::LC_L2 || lc == GOBS_LC::LC_L3 || lc == GOBS_LC::LC_L4 || lc == GOBS_LC::LC_L5)
        {
            switch (lc)
            {
            case GOBS_LC::LC_L1:
                if (_mapzen.find(f1) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L2:
                if (_mapzen.find(f2) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L3:
                if (_mapzen.find(f3) == _mapzen.end())
                    f3 = f2;
                if (_mapzen.find(f3) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L4:
                if (_mapzen.find(f4) == _mapzen.end())
                    f4 = f2;
                if (_mapzen.find(f4) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L5:
                if (_mapzen.find(f5) == _mapzen.end())
                    f5 = f2;
                if (_mapzen.find(f5) == _mapzen.end())
                {
                    return -1;
                }
                break;
            default:
            {
                return -1;
            }
            }
        }

        std::map<double, double> mapDataf1;
        mapDataf1 = _mapzen[f1];
        std::map<double, double> mapDataf2;
        mapDataf2 = _mapzen[f2];
        std::map<double, double> mapDataf3;
        mapDataf3 = _mapzen[f3];
        std::map<double, double> mapDataf4;
        mapDataf4 = _mapzen[f4];
        std::map<double, double> mapDataf5;
        mapDataf5 = _mapzen[f5];

        double eleS = satdata.ele();
        double rS = satdata.satcrd().norm();
        double rR = site.norm();

        double sinz = (rR / rS) * cos(eleS);
        double zen = asin(sinz) * R2D;

        double corrf1 = 0.0;
        double corrf2 = 0.0;
        double corrf3 = 0.0;
        double corrf4 = 0.0;
        double corrf5 = 0.0;

        // jdhuang : fix for L3 and more
        gnss_data_interp interp(_spdlog);
        if (lc == GOBS_LC::LC_IF)
        {
            if (interp.linear(mapDataf1, zen, corrf1) < 0 || interp.linear(mapDataf2, zen, corrf2) < 0)
            {
                return -1;
            }
        }
        else
        {
            switch (lc)
            {
            case GOBS_LC::LC_L1:
                if (interp.linear(mapDataf1, zen, corrf1) < 0)
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L2:
                if (interp.linear(mapDataf2, zen, corrf2) < 0)
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L3:
                if (interp.linear(mapDataf3, zen, corrf3) < 0)
                {
                    mapDataf3 = mapDataf2;
                }
                if (interp.linear(mapDataf3, zen, corrf3) < 0)
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L4:
                if (interp.linear(mapDataf4, zen, corrf4) < 0)
                {
                    mapDataf4 = mapDataf2;
                }
                if (interp.linear(mapDataf4, zen, corrf4) < 0)
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L5:
                if (interp.linear(mapDataf5, zen, corrf5) < 0)
                {
                    mapDataf5 = mapDataf2;
                }
                if (interp.linear(mapDataf5, zen, corrf5) < 0)
                {
                    return -1;
                }
                break;
            default:
            {
                return -1;
            }
            }
        }

        //  std::cout <<  "JD: pcvS mapS " << satdata.sat()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << corrf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << corrf2 << std::endl;

        // [mm] -> [m]
        corrf1 /= 1000.0;
        corrf2 /= 1000.0;
        corrf3 /= 1000.0;
        corrf4 /= 1000.0;
        corrf5 /= 1000.0;

        // PCV for linear combination
        if (lc == LC_IF)
        {
            double koef1 = 0.0;
            double koef2 = 0.0;
            // modified by glfeng for multi sysyem
            // in gnut GAL noly have BAND_1/5/6/7/8 BDS only have BAND_2/7/6
            //satdata.coef_ionofree(BAND_1, koef1, BAND_2, koef2);

            satdata.coef_ionofree(k1, koef1, k2, koef2);
            corrLC = corrf1 * koef1 + corrf2 * koef2;
        }
        else if (lc == LC_L1)
        {
            corrLC = corrf1;
        }
        else if (lc == LC_L2)
        {
            corrLC = corrf2;
        }
        else if (lc == LC_L3)
        {
            corrLC = corrf3;
        }
        else if (lc == LC_L4)
        {
            corrLC = corrf4;
        }
        else if (lc == LC_L5)
        {
            corrLC = corrf5;
        }
        else
            corrLC = corrf2;

#ifdef DEBUG
        std::cout << "Sat PCV interpolation" << std::endl;
        std::cout << "Nadir = " << zen
             << " corrf1 =  " << corrf1 * 1000 << " mm, "
             << " corrf2 =  " << corrf2 * 1000 << " mm, "
             << " corrLC =  " << corrLC * 1000 << " mm, "
             << " PRN: " << anten() << std::endl;
#endif
        return 1;
    }

    // return type
    // ----------
    int gnss_data_pcv::pcvR(double &corrLC, gnss_data_sats &satdata, GOBS_LC lc, GOBSBAND k1, GOBSBAND k2)
    {
        GSYS gsys = satdata.gsys();

        // Temporary - GAL site antenna calibration not available
        //   if(gsys == GAL) gsys = GPS; // JD 2018/09 JUST FOR GALILEO, BELOW FOR ANY

        // JD: New flexible way of defining L3 frequency for multi-GNSS
        GFRQ f1 = gnss_sys::freq_priority(gsys, FREQ_1);
        GFRQ f2 = gnss_sys::freq_priority(gsys, FREQ_2);
        GFRQ f3 = gnss_sys::freq_priority(gsys, FREQ_3);
        GFRQ f4 = gnss_sys::freq_priority(gsys, FREQ_4);
        GFRQ f5 = gnss_sys::freq_priority(gsys, FREQ_5);

        // ==================================================
        // JD 2018/09: DEFAULT SUBSTITUTION FOR NON-EXISTENT
        // ==================================================
        // jdhuang : fix for Raw all
        if (lc == GOBS_LC::LC_IF)
        {
            if (_mapzen.find(f1) == _mapzen.end())
            {
                if (_gnote)
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
                f1 = gnss_sys::freq_priority(GPS, (FREQ_1));
            }
            if (_mapzen.find(f2) == _mapzen.end())
            {
                if (_gnote)
                    _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
                f2 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
            }

            if (_mapzen.find(f1) == _mapzen.end())
            {
                return -1;
            }
            if (_mapzen.find(f2) == _mapzen.end())
            {
                return -1;
            }
        }
        else
        {
            switch (lc)
            {
            case GOBS_LC::LC_L1:
                if (_mapzen.find(f1) == _mapzen.end())
                {
                    if (_gnote)
                        _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f1) + "], base_type_conv::substituted with GPS L1");
                    f1 = gnss_sys::freq_priority(GPS, (FREQ_1));
                }
                if (_mapzen.find(f1) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L2:
                if (_mapzen.find(f2) == _mapzen.end())
                {
                    if (_gnote)
                        _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
                    f2 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
                }
                if (_mapzen.find(f2) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L3:
                if (_mapzen.find(f3) == _mapzen.end())
                {
                    if (_gnote)
                        _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
                    f3 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
                }
                if (_mapzen.find(f3) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L4:
                if (_mapzen.find(f4) == _mapzen.end())
                {
                    if (_gnote)
                        _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
                    f4 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
                }
                if (_mapzen.find(f4) == _mapzen.end())
                {
                    return -1;
                }
                break;
            case GOBS_LC::LC_L5:
                if (_mapzen.find(f5) == _mapzen.end())
                {
                    if (_gnote)
                        _gnote->mesg(GWARNING, "gpcv", "no REC PCV [" + _anten + "/" + t_gfreq::gfreq2str(f2) + "], base_type_conv::substituted with GPS L2");
                    f5 = gnss_sys::freq_priority(GPS, (gnss_sys::gfrq2freq(gsys, f2) <= 2) ? gnss_sys::gfrq2freq(gsys, f2) : FREQ_2);
                }
                if (_mapzen.find(f5) == _mapzen.end())
                {
                    return -1;
                }
                break;
            default:
                return -1;
            }
        }

        // ==================================================

        double zen = hwa_pi / 2.0 - satdata.ele();
        zen *= R2D;

        double azi = satdata.azi();
        azi *= R2D;

        double corrf1 = 0.0;
        double corrf2 = 0.0;
        double corrf3 = 0.0;
        double corrf4 = 0.0;
        double corrf5 = 0.0;

        bool isAzi = false;
        if (lc == GOBS_LC::LC_IF && _azi_dependent(f1) && _azi_dependent(f2))
            isAzi = true;
        else if (lc == GOBS_LC::LC_L1 && _azi_dependent(f1))
            isAzi = true;
        else if (lc == GOBS_LC::LC_L2 && _azi_dependent(f2))
            isAzi = true;
        else if (lc == GOBS_LC::LC_L3 && _azi_dependent(f3))
            isAzi = true;
        else if (lc == GOBS_LC::LC_L4 && _azi_dependent(f4))
            isAzi = true;
        else if (lc == GOBS_LC::LC_L5 && _azi_dependent(f5))
            isAzi = true;
        else
            isAzi = _azi_dependent(f2);

        if (isAzi)
        {
            // AZI-dependant calibration available
            base_pair p_az(azi, zen);

            std::map<base_pair, double> mapDataf1;
            std::map<base_pair, double> mapDataf2;
            std::map<base_pair, double> mapDataf3;
            std::map<base_pair, double> mapDataf4;
            std::map<base_pair, double> mapDataf5;

            for (std::map<GFRQ, hwa_map_A>::iterator itGFRQ = _mapazi.begin(); itGFRQ != _mapazi.end(); itGFRQ++)
            {
                GFRQ f = itGFRQ->first;

                // jdhuang
                // 2020.04.07 fix a bug :change  || to &&
                if (lc == GOBS_LC::LC_IF && (f != f1 && f != f2))
                    continue;
                if (lc == GOBS_LC::LC_L1 && (f != f1))
                    continue;
                if (lc == GOBS_LC::LC_L2 && (f != f2))
                    continue;
                if (lc == GOBS_LC::LC_L3 && (f != f3))
                    continue;
                if (lc == GOBS_LC::LC_L4 && (f != f4))
                    continue;
                if (lc == GOBS_LC::LC_L5 && (f != f5))
                    continue;

                // if(f != f1 && f != f2) continue;
                std::map<double, hwa_map_Z>::iterator itA1 = itGFRQ->second.lower_bound(azi);
                if (itA1 == itGFRQ->second.end() || itA1 == itGFRQ->second.begin())
                {
                    return -1;
                }
                std::map<double, hwa_map_Z>::iterator itA2 = itA1;
                itA2--;

                std::map<double, double>::iterator itZ1 = itA1->second.lower_bound(zen);
                if (itZ1 == itA1->second.end() || itZ1 == itA1->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ2 = itZ1;
                itZ2--;

                std::map<double, double>::iterator itZ3 = itA2->second.lower_bound(zen);
                if (itZ3 == itA2->second.end() || itZ3 == itA2->second.begin())
                {
                    return -1;
                }
                std::map<double, double>::iterator itZ4 = itZ3;
                itZ4--;

                base_pair p1(itA1->first, itZ1->first);
                base_pair p2(itA1->first, itZ2->first);
                base_pair p3(itA2->first, itZ3->first);
                base_pair p4(itA2->first, itZ4->first);

                if (f == f1)
                {
                    mapDataf1[p1] = itZ1->second; //std::cout << p1[0] << " : " << p1[1] << "  " << itZ1->second << std::endl;
                    mapDataf1[p2] = itZ2->second; //std::cout << p2[0] << " : " << p2[1] << "  " << itZ2->second << std::endl;
                    mapDataf1[p3] = itZ3->second; //std::cout << p3[0] << " : " << p3[1] << "  " << itZ3->second << std::endl;
                    mapDataf1[p4] = itZ4->second; //std::cout << p4[0] << " : " << p4[1] << "  " << itZ4->second << std::endl;
                }
                else if (f == f2)
                {
                    mapDataf2[p1] = itZ1->second;
                    mapDataf2[p2] = itZ2->second;
                    mapDataf2[p3] = itZ3->second;
                    mapDataf2[p4] = itZ4->second;
                }
                else if (f == f3)
                {
                    mapDataf3[p1] = itZ1->second;
                    mapDataf3[p2] = itZ2->second;
                    mapDataf3[p3] = itZ3->second;
                    mapDataf3[p4] = itZ4->second;
                }
                else if (f == f4)
                {
                    mapDataf4[p1] = itZ1->second;
                    mapDataf4[p2] = itZ2->second;
                    mapDataf4[p3] = itZ3->second;
                    mapDataf4[p4] = itZ4->second;
                }
                else if (f == f5)
                {
                    mapDataf5[p1] = itZ1->second;
                    mapDataf5[p2] = itZ2->second;
                    mapDataf5[p3] = itZ3->second;
                    mapDataf5[p4] = itZ4->second;
                }
                else
                {
                    continue;
                }
            }

            // jdhuang
            gnss_data_interp interp(_spdlog);
            if (lc == GOBS_LC::LC_IF)
            {
                if (interp.bilinear(mapDataf1, p_az, corrf1) < 0 || interp.bilinear(mapDataf2, p_az, corrf2) < 0)
                {
                    return -1;
                }
            }
            else
            {
                switch (lc)
                {
                case GOBS_LC::LC_L1:
                    if (interp.bilinear(mapDataf1, p_az, corrf1) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L2:
                    if (interp.bilinear(mapDataf2, p_az, corrf2) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L3:
                    if (interp.bilinear(mapDataf3, p_az, corrf3) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L4:
                    if (interp.bilinear(mapDataf4, p_az, corrf4) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L5:
                    if (interp.bilinear(mapDataf5, p_az, corrf5) < 0)
                    {
                        return -1;
                    }
                    break;
                default:
                {
                    return -1;
                }
                }
            }
        }
        else
        { // AZI-dependant calibration NOT available (only NOAZI)

            if (_gnote)
            {
                _gnote->mesg(GWARNING, "gpcv", "no REC AZI PCV [" + _anten + "/freq:" + t_gfreq::gfreq2str(f1) + "], just used NOAZI");
            }

            std::map<double, double> mapDataf1;
            std::map<double, double> mapDataf2;
            std::map<double, double> mapDataf3;
            std::map<double, double> mapDataf4;
            std::map<double, double> mapDataf5;

            if (lc == GOBS_LC::LC_IF)
            {
                if (_mapzen.find(f1) == _mapzen.end() || _mapzen.find(f2) == _mapzen.end())
                {
                    return -1;
                }

                mapDataf1 = _mapzen[f1];
                mapDataf2 = _mapzen[f2];
                gnss_data_interp interp(_spdlog);
                if (interp.linear(mapDataf1, zen, corrf1) < 0 || interp.linear(mapDataf2, zen, corrf2) < 0)
                {
                    return -1;
                }
            }
            else
            {
                if (_mapzen.find(f1) != _mapzen.end())
                {
                    mapDataf1 = _mapzen[f1];
                }
                if (_mapzen.find(f2) != _mapzen.end())
                {
                    mapDataf2 = _mapzen[f2];
                }
                if (_mapzen.find(f3) != _mapzen.end())
                {
                    mapDataf3 = _mapzen[f3];
                }
                if (_mapzen.find(f4) != _mapzen.end())
                {
                    mapDataf4 = _mapzen[f4];
                }
                if (_mapzen.find(f5) != _mapzen.end())
                {
                    mapDataf5 = _mapzen[f5];
                }

                gnss_data_interp interp(_spdlog);

                switch (lc)
                {
                case GOBS_LC::LC_L1:
                    if (interp.linear(mapDataf1, zen, corrf1) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L2:
                    if (interp.linear(mapDataf2, zen, corrf2) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L3:
                    if (interp.linear(mapDataf3, zen, corrf3) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L4:
                    if (interp.linear(mapDataf4, zen, corrf4) < 0)
                    {
                        return -1;
                    }
                    break;
                case GOBS_LC::LC_L5:
                    if (interp.linear(mapDataf5, zen, corrf5) < 0)
                    {
                        return -1;
                    }
                    break;
                default:
                {
                    return -1;
                }
                }
            }
        }

        //  std::cout <<  "JD: pcvR mapS " << anten()
        //                     << " " << t_gfreq::gfreq2str(f1) << " " << corrf1
        //                     << " " << t_gfreq::gfreq2str(f2) << " " << corrf2 << std::endl;

        // [mm] -> [m]
        corrf1 /= 1000.0;
        corrf2 /= 1000.0;
        corrf3 /= 1000.0;
        corrf4 /= 1000.0;
        corrf5 /= 1000.0;

        // PCV for linear combination
        if (lc == LC_IF)
        {
            double koef1 = 0.0;
            double koef2 = 0.0;
            // modified by glfeng for multi sysyem
            // in gnut GAL noly have BAND_1/5/6/7/8 BDS only have BAND_2/7/6
            //satdata.coef_ionofree(BAND_1, koef1, BAND_2, koef2);

            satdata.coef_ionofree(k1, koef1, k2, koef2);
            corrLC = corrf1 * koef1 + corrf2 * koef2;
        }
        else if (lc == LC_L1)
        {
            corrLC = corrf1;
        }
        else if (lc == LC_L2)
        {
            corrLC = corrf2;
        }
        else if (lc == LC_L3)
        {
            corrLC = corrf3;
        }
        else if (lc == LC_L4)
        {
            corrLC = corrf4;
        }
        else if (lc == LC_L5)
        {
            corrLC = corrf5;
        }
        else
            corrLC = corrf2;

#ifdef DEBUG
        std::cout << "Rec PCV interpolation" << std::endl;
        std::cout << "Zenith = " << zen
             << " corrf1 =  " << corrf1 * 1000 << " mm, "
             << " corrf2 =  " << corrf2 * 1000 << " mm, "
             << " corrLC =  " << corrLC * 1000 << " mm, "
             << " Ant: " << anten() << " "
             << " Sat: " << satdata.sat() << std::endl;
//   int ooo; cin >> ooo;
#endif
        return 1;
    }

    // does the calibration contain azi-depenedant data?
    bool gnss_data_pcv::_azi_dependent(GFRQ f)
    {
        hwa_map_azi::iterator it = _mapazi.find(f);

        bool ret = false;

        if (it == _mapazi.end())
        {
            ret = false;
        }
        else
        {

            int size = _mapazi[f].size();
            if (size > 0)
            {
                ret = true;
            }
            else
            {
                ret = false;
            }
        }
        if (_pcv_noazi)
            ret = false;
        return ret;
    }

} // namespace
