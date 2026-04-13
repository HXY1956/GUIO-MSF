#include "hwa_gnss_model_tideiers.h"
#include "hwa_base_const.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_gnss
{
    gnss_model_tideiers::gnss_model_tideiers() : gnss_model_tide()
    {
        _mean_pole_model = modeofmeanpole::cubic;
    }

    gnss_model_tideiers::gnss_model_tideiers(base_log spdlog) : gnss_model_tide(spdlog)
    {
        _mean_pole_model = modeofmeanpole::cubic;
    }

    gnss_model_tideiers::gnss_model_tideiers(gnss_all_otl *otl) : gnss_model_tide()
    {
        _gotl = otl;
        _mean_pole_model = modeofmeanpole::cubic;
    }

    gnss_model_tideiers::gnss_model_tideiers(base_log spdlog, gnss_all_otl *otl) : gnss_model_tide(spdlog)
    {
        _gotl = otl;
    }

    Triple gnss_model_tideiers::tide_solid(const base_time &epo, Triple &xyz, Matrix &rot_trs2crs, gnss_data_navde *nav_planet)
    {
        // See the computation ins_scheme in IERS Conventions 2010 page 103
        //const double h20 = 0.6026e0;
        const double h20 = 0.6078e0;
        const double dh2 = -0.0006e0;
        //const double l20 = 0.0831e0;
        const double l20 = 0.0847e0;
        const double dl2 = 0.0002e0;
        //const double h3 = 0.293e0;
        const double h3 = 0.292e0;
        const double l3 = 0.015e0;
        Vector sun_pos(3), moon_pos(3);

        // get sun pos in J2000
        base_time epo_tt = epo;
        epo_tt.tsys(base_time::TT);
        double mjd = epo_tt.dmjd();
        nav_planet->get_pos(mjd, "SUN", sun_pos);
        nav_planet->get_pos(mjd, "MOON", moon_pos);

        // change to TRS, unit: km
        sun_pos = rot_trs2crs.transpose() * sun_pos;
        //Triple sun_pos_trs(sun_pos);

        moon_pos = rot_trs2crs.transpose() * moon_pos;
        //Triple moon_pos_trs(moon_pos);

        double rSun = sqrt(sun_pos.dot(sun_pos));
        double rMoon = sqrt(moon_pos.dot(moon_pos));

        // unit std::vector for site
        Vector unit_site_pos = xyz / xyz.norm();
        // get BLH
        Triple blh;
        xyz2ell(xyz, blh, false);
        // double lat = blh[0];
        // double lon = blh[1];
        // double height = blh[2];
        // double colat = hwa_pi / 2 - blh[0];
        double lsta = xyz.norm();
        double cosphi = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]) / lsta;

        // calculate
        Triple dx(0.0, 0.0, 0.0);
        for (int i = 0; i < 2; i++)
        {
            Triple unit_pos(0.0, 0.0, 0.0);
            //double GM_body = 0.0, R_body, dotl = 0.0, scale = 0.0;
            double mass_ratio = 0.0, R_body, dotl = 0.0, scale = 0.0;
            switch (i)
            {
            case 0:
                unit_pos = sun_pos / rSun;
                dotl = unit_pos.dot(Triple(unit_site_pos));
                //GM_body = SUN_GM;
                mass_ratio = _MASS_RATIO_SUN;
                R_body = rSun;
                break;
            case 1:
                unit_pos = moon_pos / rMoon;
                dotl = unit_pos.dot(Triple(unit_site_pos));
                //GM_body = MOON_GM;
                mass_ratio = _MASS_RATIO_MOON;
                R_body = rMoon;
                break;
            }

            //scale = mass_ratio * EARTH_R * pow((EARTH_R / R_body), 3);
            scale = mass_ratio * _EARTH_R * pow((_EARTH_R / R_body), 3);

            //double h2 = h20 + dh2 * (3.0 * sin(lat) * sin(lat) - 1.0) / 2.0;
            double h2 = h20 + dh2 * (2.0 - 3.0 * cosphi * cosphi) / 2.0;
            //double l2 = l20 + dl2 * (3.0 * sin(lat) * sin(lat) - 1.0) / 2.0;
            double l2 = l20 + dl2 * (2.0 - 3.0 * cosphi * cosphi) / 2.0;
            double usite_part = (h2 * (1.5 * dotl * dotl - 0.50) - 3.0 * l2 * dotl * dotl) * scale;
            double ubody_part = 3.e0 * l2 * dotl * scale;

            dx = dx + usite_part * unit_site_pos + ubody_part * unit_pos;

            scale = scale * (_EARTH_R / R_body);
            ubody_part = (7.5 * dotl * dotl - 1.5) * l3;
            usite_part = (h3 * (2.5 * dotl * dotl - 1.5) * dotl - ubody_part * dotl) * scale;
            ubody_part = ubody_part * scale;
            dx = dx + usite_part * unit_site_pos + ubody_part * unit_pos;
        }

        double FAC2SUN = _MASS_RATIO_SUN * _EARTH_R * pow((_EARTH_R / rSun), 3) * 1000;
        double FAC2MON = _MASS_RATIO_MOON * _EARTH_R * pow((_EARTH_R / rMoon), 3) * 1000;
        // double FAC3SUN = FAC2SUN * (_EARTH_R / rSun);
        // double FAC3MON = FAC2MON * (_EARTH_R / rMoon);
        // FIRST, FOR THE DIURNAL BAND
        Triple XCORSTA_DIURNAL = _st1idiu(xyz, sun_pos, moon_pos, FAC2SUN, FAC2MON);
        dx = dx + XCORSTA_DIURNAL;

        // SECOND, FOR THE SEMI-DIURNAL BAND
        Triple XCORSTA_SEMI = _st1isem(xyz, sun_pos, moon_pos, FAC2SUN, FAC2MON);
        dx = dx + XCORSTA_SEMI;

        // CORRECTIONS FOR THE LATITUDE DEPENDENCE OF LOVE NUMBERS (PART L^(1) )
        Triple XCORSTA_LATI = _st1l1(xyz, sun_pos, moon_pos, FAC2SUN, FAC2MON);
        dx = dx + XCORSTA_LATI;

        Triple dx_diurnal = tide_solid_frequency_diurnal(epo_tt, xyz);
        Triple dx_longperiod = tide_solid_frequency_longperiod(epo_tt, xyz);
        //std::cout << dx_diurnal[0] * 1000 << "  " << dx_diurnal[1] * 1000 << "  " << dx_diurnal[2] * 1000 << std::endl;
        //std::cout << dx_longperiod[0] * 1000 << "  " << dx_longperiod[1] * 1000 << "  " << dx_longperiod[2] * 1000 << std::endl;
        dx = dx + dx_diurnal + dx_longperiod;
        Triple tmp = dx;

        return tmp;
    }

    Triple gnss_model_tideiers::tide_solid(const base_time &epo, Triple &xyz, Matrix &rot_trs2crs, Vector &sun_pos, Vector &moon_pos)
    {
        // See the computation ins_scheme in IERS Conventions 2010 page 103
        //const double h20 = 0.6026e0;
        const double h20 = 0.6078e0;
        const double dh2 = -0.0006e0;
        //const double l20 = 0.0831e0;
        const double l20 = 0.0847e0;
        const double dl2 = 0.0002e0;
        //const double h3 = 0.293e0;
        const double h3 = 0.292e0;
        const double l3 = 0.015e0;

        // get sun pos in J2000
        base_time epo_tt = epo;
        epo_tt.tsys(base_time::TT);

        // change to TRS, unit: km
        sun_pos = rot_trs2crs.transpose() * sun_pos;
        //Triple sun_pos_trs(sun_pos);

        moon_pos = rot_trs2crs.transpose() * moon_pos;
        //Triple moon_pos_trs(moon_pos);

        double rSun = sqrt(sun_pos.dot(sun_pos));
        double rMoon = sqrt(moon_pos.dot(moon_pos));

        // unit std::vector for site
        Triple unit_site_pos = xyz / xyz.norm();
        // get BLH
        Triple blh;
        xyz2ell(xyz, blh, false);
        // double lat = blh[0];
        // double lon = blh[1];
        // double height = blh[2];
        // double colat = hwa_pi / 2 - blh[0];
        double lsta = xyz.norm();
        double cosphi = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]) / lsta;

        // calculate
        Triple dx(0, 0, 0);
        for (int i = 0; i < 2; i++)
        {
            Triple unit_pos;
            // double GM_body = 0.0, R_body, dotl = 0.0, scale = 0.0;
            double mass_ratio = 0.0, R_body, dotl = 0.0, scale = 0.0;
            switch (i)
            {
            case 0:
                unit_pos = sun_pos / rSun;
                dotl = unit_pos.dot(unit_site_pos);
                // GM_body = SUN_GM;
                mass_ratio = _MASS_RATIO_SUN;
                R_body = rSun;
                break;
            case 1:
                unit_pos = moon_pos / rMoon;
                dotl = unit_pos.dot(unit_site_pos);
                // GM_body = MOON_GM;
                mass_ratio = _MASS_RATIO_MOON;
                R_body = rMoon;
                break;
            }

            // scale = GM_body / EARTH_GM * EARTH_R * pow((EARTH_R / R_body), 3);
            scale = mass_ratio * _EARTH_R * pow((_EARTH_R / R_body), 3);

            // double h2 = h20 + dh2 * (3.0 * sin(lat) * sin(lat) - 1.0) / 2.0;
            double h2 = h20 + dh2 * (2.0 - 3.0 * cosphi * cosphi) / 2.0;
            // double l2 = l20 + dl2 * (3.0 * sin(lat) * sin(lat) - 1.0) / 2.0;
            double l2 = l20 + dl2 * (2.0 - 3.0 * cosphi * cosphi) / 2.0;
            double usite_part = (h2 * (1.5 * dotl * dotl - 0.50) - 3.0 * l2 * dotl * dotl) * scale;
            double ubody_part = 3.e0 * l2 * dotl * scale;

            dx = dx + usite_part * unit_site_pos + ubody_part * unit_pos;

            scale = scale * (EARTH_R / R_body);
            ubody_part = (7.5 * dotl * dotl - 1.5) * l3;
            usite_part = (h3 * (2.5 * dotl * dotl - 1.5) * dotl - ubody_part * dotl) * scale;
            ubody_part = ubody_part * scale;
            dx = dx + usite_part * unit_site_pos + ubody_part * unit_pos;
        }

        double FAC2SUN = _MASS_RATIO_SUN * _EARTH_R * pow((_EARTH_R / rSun), 3) * 1000;
        double FAC2MON = _MASS_RATIO_MOON * _EARTH_R * pow((_EARTH_R / rMoon), 3) * 1000;
        // double FAC3SUN = FAC2SUN * (_EARTH_R / rSun);
        // double FAC3MON = FAC2MON * (_EARTH_R / rMoon);
        // FIRST, FOR THE DIURNAL BAND
        // added by yqyuan 2021/11/25
        // Reference: IERS Conventions 2010 v1.0, Eq (7.10)
        Triple XCORSTA_DIURNAL = _st1idiu(xyz, sun_pos, moon_pos, FAC2SUN, FAC2MON);
        dx = dx + XCORSTA_DIURNAL;

        // SECOND, FOR THE SEMI-DIURNAL BAND
        // added by yqyuan 2021/11/25
        // Reference: IERS Conventions 2010 v1.0, Eq (7.11)
        Triple XCORSTA_SEMI = _st1isem(xyz, sun_pos, moon_pos, FAC2SUN, FAC2MON);
        dx = dx + XCORSTA_SEMI;

        // CORRECTIONS FOR THE LATITUDE DEPENDENCE OF LOVE NUMBERS (PART L^(1) )
        // added by yqyuan 2021/11/25
        // Reference: IERS Conventions 2010 v1.0, Eq (7.7-7.9)
        Triple XCORSTA_LATI = _st1l1(xyz, sun_pos, moon_pos, FAC2SUN, FAC2MON);
        dx = dx + XCORSTA_LATI;

        Triple dx_diurnal = tide_solid_frequency_diurnal(epo_tt, xyz);
        Triple dx_longperiod = tide_solid_frequency_longperiod(epo_tt, xyz);
        //std::cout << dx_diurnal[0] * 1000 << "  " << dx_diurnal[1] * 1000 << "  " << dx_diurnal[2] * 1000 << std::endl;
        //std::cout << dx_longperiod[0] * 1000 << "  " << dx_longperiod[1] * 1000 << "  " << dx_longperiod[2] * 1000 << std::endl;
        Triple tmp = dx + dx_diurnal + dx_longperiod;
        return tmp;
    }

    /** added by kkzhang 2021/11/25 */
    /** Reference: IERS Conventions 2010 v1.0, Eq (7.12); STEP2DIU.F from IERS 2010 Conventions software collection */
    Triple gnss_model_tideiers::tide_solid_frequency_diurnal(const base_time &epo, Triple &xyz)
    {
        double rsta, sinphi, cosphi, cosla, sinla, zla;
        Triple dx; dx.setZero();

        //Compute centuries since J2000 and base_type_conv::fractional hours in the day
        double t = (epo.dmjd() - 51544.5) / 36525;
        double hr = (epo.dmjd() - int(epo.dmjd())) * 24;

        //Compute the phase angles in degrees.
        double s = 218.31664563 + (481267.88194 + (-0.0014663889 + (0.00000185139) * t) * t) * t;

        double tau = hr * 15 + 280.4606184 + (36000.7700536 + (0.00038793 + (-0.0000000258) * t) * t) * t - s;

        double pr = (1.396971278 + (0.000308889 + (0.000000021 + 0.000000007 * t) * t) * t) * t;

        s = s + pr;

        double h = 280.46645 + (36000.7697489 + (0.00030322222 + (0.000000020 + (-0.00000000654) * t) * t) * t) * t;

        double p = 83.35324312 + (4069.01363525 + (-0.01032172222 + (-0.0000124991 + 0.00000005263 * t) * t) * t) * t;

        double zns = 234.95544499 + (1934.13626197 + (-0.00207561111 + (-0.00000213944 + 0.00000001650 * t) * t) * t) * t;

        double ps = 282.93734098 + (1.71945766667 + (0.00045688889 + (-0.00000001778 + (-0.00000000334) * t) * t) * t) * t;

        //Reduce angles to between the range 0 and 360.
        s = s - int(s / 360) * 360;
        tau = tau - int(tau / 360) * 360;
        h = h - int(h / 360) * 360;
        p = p - int(p / 360) * 360;
        zns = zns - int(zns / 360) * 360;
        ps = ps - int(ps / 360) * 360;

        rsta = xyz.norm();
        sinphi = xyz[2] / rsta;
        cosphi = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]) / rsta;

        cosla = xyz[0] / cosphi / rsta;
        sinla = xyz[1] / cosphi / rsta;
        zla = atan2(xyz[1], xyz[0]);

        // Calculate corretion
        double thetaf, dr, dn, de;
        for (int i = 0; i < 31; i++)
        {
            thetaf = (tau + _solid_diurnal_tide[i][0] * s + _solid_diurnal_tide[i][1] * h + _solid_diurnal_tide[i][2] * p +
                      _solid_diurnal_tide[i][3] * zns + _solid_diurnal_tide[i][4] * ps) *
                     D2R;

            dr = _solid_diurnal_tide[i][5] * 2 * sinphi * cosphi * sin(thetaf + zla) +
                 _solid_diurnal_tide[i][6] * 2 * sinphi * cosphi * cos(thetaf + zla);

            dn = _solid_diurnal_tide[i][7] * (cosphi * cosphi - sinphi * sinphi) * sin(thetaf + zla) +
                 _solid_diurnal_tide[i][8] * (cosphi * cosphi - sinphi * sinphi) * cos(thetaf + zla);

            de = _solid_diurnal_tide[i][7] * sinphi * cos(thetaf + zla) -
                 _solid_diurnal_tide[i][8] * sinphi * sin(thetaf + zla);

            dx[0] = dx[0] + dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
            dx[1] = dx[1] + dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
            dx[2] = dx[2] + dr * sinphi + dn * cosphi;
        }
        //Unit: from mm to km
        dx = dx / 1000000;
        return dx;
    }

    /** added by kkzhang 2021/11/25 */
    /** Reference: IERS Conventions 2010 v1.0, Eq (7.13); STEP2LON.F from IERS 2010 Conventions software collection */
    Triple gnss_model_tideiers::tide_solid_frequency_longperiod(const base_time &epo, Triple &xyz)
    {
        double rsta, sinphi, cosphi, cosla, sinla;
        Triple dx(0.0, 0.0, 0.0);
        //Compute centuries since J2000
        double t = (epo.dmjd() - 51544.5) / 36525;

        //Compute the phase angles in degrees.
        double s = 218.31664563 + (481267.88194 + (-0.0014663889 + (0.00000185139) * t) * t) * t;

        double pr = (1.396971278 + (0.000308889 + (0.000000021 + 0.000000007 * t) * t) * t) * t;

        s = s + pr;

        double h = 280.46645 + (36000.7697489 + (0.00030322222 + (0.000000020 + (-0.00000000654) * t) * t) * t) * t;

        double p = 83.35324312 + (4069.01363525 + (-0.01032172222 + (-0.0000124991 + 0.00000005263 * t) * t) * t) * t;

        double zns = 234.95544499 + (1934.13626197 + (-0.00207561111 + (-0.00000213944 + 0.00000001650 * t) * t) * t) * t;

        double ps = 282.93734098 + (1.71945766667 + (0.00045688889 + (-0.00000001778 + (-0.00000000334) * t) * t) * t) * t;

        rsta = xyz.norm();
        sinphi = xyz[2] / rsta;
        cosphi = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]) / rsta;

        cosla = xyz[0] / cosphi / rsta;
        sinla = xyz[1] / cosphi / rsta;

        //Reduce angles to between the range 0 and 360.
        s = s - int(s / 360) * 360;
        h = h - int(h / 360) * 360;
        p = p - int(p / 360) * 360;
        zns = zns - int(zns / 360) * 360;
        ps = ps - int(ps / 360) * 360;

        // Calculate corretion
        double thetaf, dr, dn, de;
        for (int i = 0; i < 5; i++)
        {
            thetaf = (_solid_long_tide[i][0] * s + _solid_long_tide[i][1] * h + _solid_long_tide[i][2] * p +
                      _solid_long_tide[i][3] * zns + _solid_long_tide[i][4] * ps) *
                     D2R;

            dr = _solid_long_tide[i][5] * (3 * sinphi * sinphi - 1) / 2 * cos(thetaf) +
                 _solid_long_tide[i][7] * (3 * sinphi * sinphi - 1) / 2 * sin(thetaf);

            dn = _solid_long_tide[i][6] * (cosphi * sinphi * 2) * cos(thetaf) +
                 _solid_long_tide[i][8] * (cosphi * sinphi * 2) * sin(thetaf);

            de = 0.0;

            dx[0] = dx[0] + dr * cosla * cosphi - de * sinla - dn * sinphi * cosla;
            dx[1] = dx[1] + dr * sinla * cosphi + de * cosla - dn * sinphi * sinla;
            dx[2] = dx[2] + dr * sinphi + dn * cosphi;
        }

        //Unit: from mm to km
        dx = dx / 1000000;

        return dx;
    }

    /** @brief the out-of-phase corrections induced by mantle anelasticity in the diurnal band.*/
    /** added by yqyuan 2021/11/25 */
    /** Reference: IERS Conventions 2010 v1.0, Eq (7.10) */
    /** Remeber: SIN(PHIj)=Zj/Rj; SIN(LAMBDAj)=Yj/COS(PHIj)/Rj; COS(LAMBDAj)=Xj/COS(PHIj)/Rj */
    Triple gnss_model_tideiers::_st1idiu(Triple &XSTA, Vector &XSUN, Vector &XMON, double &FAC2SUN, double &FAC2MON)
    {
        double DHI = -0.0025e0;
        double DLI = -0.0007e0;
        // Compute the normalized position std::vector of the IGS station.
        double RSTA = sqrt(XSTA[0] * XSTA[0] + XSTA[1] * XSTA[1] + XSTA[2] * XSTA[2]);
        double SINPHI = XSTA[2] / RSTA;
        double COSPHI = sqrt(XSTA[0] * XSTA[0] + XSTA[1] * XSTA[1]) / RSTA;
        double COS2PHI = COSPHI * COSPHI - SINPHI * SINPHI;
        double SINLA = XSTA[1] / COSPHI / RSTA;
        double COSLA = XSTA[0] / COSPHI / RSTA;

        //Compute the normalized position std::vector of the Moon.
        double RMON = sqrt(XMON[0] * XMON[0] + XMON[1] * XMON[1] + XMON[2] * XMON[2]);
        // Compute the normalized position std::vector of the Sun.
        double RSUN = sqrt(XSUN[0] * XSUN[0] + XSUN[1] * XSUN[1] + XSUN[2] * XSUN[2]);

        // Remeber: SIN(PHIj)=Zj/Rj; SIN(LAMBDAj)=Yj/COS(PHIj)/Rj; COS(LAMBDAj)=Xj/COS(PHIj)/Rj
        double DRSUN = -3.0 * DHI * SINPHI * COSPHI * FAC2SUN * XSUN[2] * (XSUN[0] * SINLA - XSUN[1] * COSLA) / RSUN / RSUN;
        double DRMON = -3.0 * DHI * SINPHI * COSPHI * FAC2MON * XMON[2] * (XMON[0] * SINLA - XMON[1] * COSLA) / RMON / RMON;
        double DNSUN = -3.0 * DLI * COS2PHI * FAC2SUN * XSUN[2] * (XSUN[0] * SINLA - XSUN[1] * COSLA) / RSUN / RSUN;
        double DNMON = -3.0 * DLI * COS2PHI * FAC2MON * XMON[2] * (XMON[0] * SINLA - XMON[1] * COSLA) / RMON / RMON;
        double DESUN = -3.0 * DLI * SINPHI * FAC2SUN * XSUN[2] * (XSUN[0] * COSLA + XSUN[1] * SINLA) / RSUN / RSUN;
        double DEMON = -3.0 * DLI * SINPHI * FAC2MON * XMON[2] * (XMON[0] * COSLA + XMON[1] * SINLA) / RMON / RMON;

        double DR = DRSUN + DRMON;
        double DN = DNSUN + DNMON;
        double DE = DESUN + DEMON;

        //Compute the corrections for the station.
        Triple XCORSTA;
        XCORSTA(0) = DR * COSLA * COSPHI - DE * SINLA - DN * SINPHI * COSLA;
        XCORSTA(1) = DR * SINLA * COSPHI + DE * COSLA - DN * SINPHI * SINLA;
        XCORSTA(2) = DR * SINPHI + DN * COSPHI;

        XCORSTA = XCORSTA / 1000;
        return XCORSTA;
    }

    /** @brief the out-of-phase corrections induced by mantle anelasticity in the semi-diurnal band.*/
    /** added by yqyuan 2021/11/25 */
    /** Reference: IERS Conventions 2010 v1.0, Eq (7.11) */
    /** Remeber: SIN(PHIj)=Zj/Rj; SIN(LAMBDAj)=Yj/COS(PHIj)/Rj; COS(LAMBDAj)=Xj/COS(PHIj)/Rj */
    Triple gnss_model_tideiers::_st1isem(Triple &XSTA, Vector &XSUN, Vector &XMON, double &FAC2SUN, double &FAC2MON)
    {
        double DHI = -0.0022;
        double DLI = -0.0007;

        double RSTA = sqrt(XSTA[0] * XSTA[0] + XSTA[1] * XSTA[1] + XSTA[2] * XSTA[2]);
        double SINPHI = XSTA[2] / RSTA;
        double COSPHI = sqrt(XSTA[0] * XSTA[0] + XSTA[1] * XSTA[1]) / RSTA;
        double SINLA = XSTA[1] / COSPHI / RSTA;
        double COSLA = XSTA[0] / COSPHI / RSTA;
        double COSTWOLA = COSLA * COSLA - SINLA * SINLA;
        double SINTWOLA = 2.0 * COSLA * SINLA;

        // Compute the normalized position std::vector of the Moon.
        double RMON = sqrt(XMON[0] * XMON[0] + XMON[1] * XMON[1] + XMON[2] * XMON[2]);
        // Compute the normalized position std::vector of the Sun.
        double RSUN = sqrt(XSUN[0] * XSUN[0] + XSUN[1] * XSUN[1] + XSUN[2] * XSUN[2]);

        double DRSUN = -3.0 / 4.0 * DHI * COSPHI * COSPHI * FAC2SUN * ((XSUN[0] * XSUN[0] - XSUN[1] * XSUN[1]) * SINTWOLA - 2.0 * XSUN[0] * XSUN[1] * COSTWOLA) / RSUN / RSUN;
        double DRMON = -3.0 / 4.0 * DHI * COSPHI * COSPHI * FAC2MON * ((XMON[0] * XMON[0] - XMON[1] * XMON[1]) * SINTWOLA - 2.0 * XMON[0] * XMON[1] * COSTWOLA) / RMON / RMON;
        double DNSUN = 3.0 / 2.0 * DLI * SINPHI * COSPHI * FAC2SUN * ((XSUN[0] * XSUN[0] - XSUN[1] * XSUN[1]) * SINTWOLA - 2.0 * XSUN[0] * XSUN[1] * COSTWOLA) / RSUN / RSUN;
        double DNMON = 3.0 / 2.0 * DLI * SINPHI * COSPHI * FAC2MON * ((XMON[0] * XMON[0] - XMON[1] * XMON[1]) * SINTWOLA - 2.0 * XMON[0] * XMON[1] * COSTWOLA) / RMON / RMON;
        double DESUN = -3.0 / 2.0 * DLI * COSPHI * FAC2SUN * ((XSUN[0] * XSUN[0] - XSUN[1] * XSUN[1]) * COSTWOLA + 2.0 * XSUN[0] * XSUN[1] * SINTWOLA) / RSUN / RSUN;
        double DEMON = -3.0 / 2.0 * DLI * COSPHI * FAC2MON * ((XMON[0] * XMON[0] - XMON[1] * XMON[1]) * COSTWOLA + 2.0 * XMON[0] * XMON[1] * SINTWOLA) / RMON / RMON;
        double DR = DRSUN + DRMON;
        double DN = DNSUN + DNMON;
        double DE = DESUN + DEMON;

        Triple XCORSTA;
        XCORSTA(0) = DR * COSLA * COSPHI - DE * SINLA - DN * SINPHI * COSLA;
        XCORSTA(1) = DR * SINLA * COSPHI + DE * COSLA - DN * SINPHI * SINLA;
        XCORSTA(2) = DR * SINPHI + DN * COSPHI;
        XCORSTA = XCORSTA / 1000;

        return XCORSTA;
    }

    /** @brief the corrections induced by the latitude dependence given by L^1 in Mathews et al. 1991.*/
    /** added by yqyuan 2021/11/25*/
    /** Reference: IERS Conventions 2010 v1.0, Eq (7.7-7.9)*/
    Triple gnss_model_tideiers::_st1l1(Triple &XSTA, Vector &XSUN, Vector &XMON, double &FAC2SUN, double &FAC2MON)
    {
        double L1D = 0.0012;
        double L1SD = 0.0024;
        // Compute the normalized position std::vector of the IGS station.
        double RSTA = sqrt(XSTA[0] * XSTA[0] + XSTA[1] * XSTA[1] + XSTA[2] * XSTA[2]);
        double SINPHI = XSTA[2] / RSTA;
        double COSPHI = sqrt(XSTA[0] * XSTA[0] + XSTA[1] * XSTA[1]) / RSTA;
        double SINLA = XSTA[1] / COSPHI / RSTA;
        double COSLA = XSTA[0] / COSPHI / RSTA;
        // Compute the normalized position std::vector of the Moon.
        double RMON = sqrt(XMON[0] * XMON[0] + XMON[1] * XMON[1] + XMON[2] * XMON[2]);
        // Compute the normalized position std::vector of the Sun.
        double RSUN = sqrt(XSUN[0] * XSUN[0] + XSUN[1] * XSUN[1] + XSUN[2] * XSUN[2]);
        // Compute the station corrections for the diurnal band.
        double L1 = L1D;
        double DNSUN = -L1 * SINPHI * SINPHI * FAC2SUN * XSUN[2] * (XSUN[0] * COSLA + XSUN[1] * SINLA) / RSUN / RSUN;
        double DNMON = -L1 * SINPHI * SINPHI * FAC2MON * XMON[2] * (XMON[0] * COSLA + XMON[1] * SINLA) / RMON / RMON;
        double DESUN = L1 * SINPHI * (COSPHI * COSPHI - SINPHI * SINPHI) * FAC2SUN * XSUN[2] * (XSUN[0] * SINLA - XSUN[1] * COSLA) / RSUN / RSUN;
        double DEMON = L1 * SINPHI * (COSPHI * COSPHI - SINPHI * SINPHI) * FAC2MON * XMON[2] * (XMON[0] * SINLA - XMON[1] * COSLA) / RMON / RMON;
        //std::cout << DNSUN << DNMON << DESUN << DEMON;
        double DE = 3.0 * (DESUN + DEMON);
        double DN = 3.0 * (DNSUN + DNMON);

        Triple XCORSTA;
        XCORSTA(0) = -DE * SINLA - DN * SINPHI * COSLA;
        XCORSTA(1) = DE * COSLA - DN * SINPHI * SINLA;
        XCORSTA(2) = DN * COSPHI;

        L1 = L1SD;
        double COSTWOLA = COSLA * COSLA - SINLA * SINLA;
        double SINTWOLA = 2.0 * COSLA * SINLA;
        DNSUN = -L1 / 2.0 * SINPHI * COSPHI * FAC2SUN * ((XSUN[0] * XSUN[0] - XSUN[1] * XSUN[1]) * COSTWOLA + 2.0 * XSUN[0] * XSUN[1] * SINTWOLA) / RSUN / RSUN;
        DNMON = -L1 / 2.0 * SINPHI * COSPHI * FAC2MON * ((XMON[0] * XMON[0] - XMON[1] * XMON[1]) * COSTWOLA + 2.0 * XMON[0] * XMON[1] * SINTWOLA) / RMON / RMON;
        DESUN = -L1 / 2.0 * SINPHI * SINPHI * COSPHI * FAC2SUN * ((XSUN[0] * XSUN[0] - XSUN[1] * XSUN[1]) * SINTWOLA - 2.0 * XSUN[0] * XSUN[1] * COSTWOLA) / RSUN / RSUN;
        DEMON = -L1 / 2.0 * SINPHI * SINPHI * COSPHI * FAC2MON * ((XMON[0] * XMON[0] - XMON[1] * XMON[1]) * SINTWOLA - 2.0 * XMON[0] * XMON[1] * COSTWOLA) / RMON / RMON;

        DE = 3.0 * (DESUN + DEMON);
        DN = 3.0 * (DNSUN + DNMON);

        XCORSTA(0) = XCORSTA(0) - DE * SINLA - DN * SINPHI * COSLA;
        XCORSTA(1) = XCORSTA(1) + DE * COSLA - DN * SINPHI * SINLA;
        XCORSTA(2) = XCORSTA(2) + DN * COSPHI;
        XCORSTA = XCORSTA / 1000;

        return XCORSTA;
    }

    Triple gnss_model_tideiers::tide_pole()
    {
        return Triple();
    }

    // the
    Triple gnss_model_tideiers::tide_pole_pod(const base_time &epo, double xpole, double ypole, Triple &xyz)
    {
        Triple blh;
        xyz2ell(xyz, blh, false);
        // double lat = blh[0];
        // double height = blh[2];
        double colat = hwa_pi / 2 - blh[0];

        double mean_xpole = 0.0, mean_ypole = 0.0;
        getMeanPole(epo.dmjd(), mean_xpole, mean_ypole);

        mean_xpole = xpole - mean_xpole;
        mean_ypole = -(ypole - mean_ypole);

        Triple dxi(0.0, 0.0, 0.0);

        double lon = blh[1];
        dxi[1] = 9.0 * cos(colat) * (mean_xpole * sin(lon) - mean_ypole * cos(lon));
        dxi[0] = 9.0 * cos(2.0 * colat) * (mean_xpole * cos(lon) + mean_ypole * sin(lon));
        dxi[2] = -33.0 * sin(2.0 * colat) * (mean_xpole * cos(lon) + mean_ypole * sin(lon));

        Triple dx(0.0, 0.0, 0.0);
        //enu neu
        neu2xyz(blh, dxi, dx);

        //!!rotation matrix from east - north - radial to x - y - z
        //dx = dx + dxi / 1.e6;
        dx = dx / 1.e6;
        return dx;
    }

    Triple gnss_model_tideiers::load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec)
    {
        // calcute angular argument
        const double dtr = 0.174532925199e-1;
        const double speed[11] = {1.40519e-4, 1.45444e-4, 1.37880e-4, 1.45842e-4, 0.72921e-4, 0.67598e-4,
                                  0.72523e-4, 0.64959e-4, 0.053234e-4, 0.026392e-4, 0.003982e-4};

        const double angfac[4][11] = {{2.0, 0.0, 2.0, 2.0, 1.0, 1.0, -1.0, 1.0, 0.0, 0.0, 2.0},
                                      {-2.0, 0.0, -3.0, 0.0, 0.0, -2.0, 0.0, -3.0, 2.0, 1.0, 0.0},
                                      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0},
                                      {0.0, 0.0, 0.0, 0.0, 0.25, -0.25, -0.25, -0.25, 0.0, 0.0, 0.0}};

        if (!_gotl)
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "No ocean load file, check if you need it!");
            return Triple(0.0, 0.0, 0.0);
        }
        int capd = epoch.doy() + 365 * (epoch.year() - 1975) + (epoch.year() - 1973) / 4;
        double capt = (27392.500528e0 + 1.000000035e0 * capd) / 36525.0;
        double h0 = (279.69668e0 + (36000.768930485e0 + 3.03e-4 * capt) * capt) * dtr;
        double s0 = (((1.9e-6 * capt - 0.001133) * capt + 481267.88314137) * capt + 270.434358) * dtr;
        double p0 = (((-1.2e-5 * capt - 0.010325) * capt + 4069.0340329577) * capt + 334.329653) * dtr;

        double angle[11] = {0.0};
        double sod = epoch.sod() + epoch.dsec();
        for (int i = 0; i < 11; i++)
        {
            angle[i] = speed[i] * sod + angfac[0][i] * h0 + angfac[1][i] * s0 + angfac[2][i] * p0 + angfac[3][i] * hwa_pi * 2;
            angle[i] = fmod(angle[i], hwa_pi * 2);
            if (angle[i] < 0)
                angle[i] = angle[i] + hwa_pi * 2;
        }

        //get oceanload data
        Triple ell;
        xyz2ell(xRec, ell, true);
        Matrix coef_oceanload;
        int isexist = _gotl->data(coef_oceanload, ell[1], ell[0]);
        if (isexist < 0)
        {
            coef_oceanload.resize(6, 11);
            coef_oceanload.setZero();
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Not found {} in ocean load file!", site);
        }

        //compute NEU for 11 components
        Triple dx{0, 0, 0};
        for (int i = 0; i < 11; i++)
        {
            dx[0] = dx[0] - coef_oceanload(2, i) * cos(angle[i] - coef_oceanload(5, i) * D2R);
            dx[1] = dx[1] - coef_oceanload(1, i) * cos(angle[i] - coef_oceanload(4, i) * D2R);
            dx[2] = dx[2] + coef_oceanload(0, i) * cos(angle[i] - coef_oceanload(3, i) * D2R);
        }
        Triple blh, xyz;
        xyz2ell(xRec, blh, false);
        neu2xyz(blh, dx, xyz);
        xyz = xyz / 1.e3;
        return xyz;
    }
    Triple gnss_model_tideiers::load_atmosph()
    {
        return Triple();
    }

    // added by yqyuan for atmospheric pressure loading; 2021/11/30
    // References: [1] IERS Conventions 2010, Sect. 7.1.3
    //             [2] https://geophy.uni.lu/displacementgrids/
    //             [3] /gamit/model/etide.f
    Triple gnss_model_tideiers::atmospheric_loading(const base_time &epoch, const Triple &xyz)
    {
        Triple blh;
        xyz2ell(xyz, blh, false);
        double lat = blh[0] * R2D;
        double lon = blh[1] * R2D;
        Matrix atmo_coef;
        atmo_coef.resize(3, 4);
        atmo_coef.setZero();
        if (!_atm_grid->get_coef(lon, lat, atmo_coef))
        {
            return Triple();
        }
        /*std::cout << atmo_coef(1, 1) << atmo_coef(1, 2) << atmo_coef(1, 3) << atmo_coef(1, 4) << std::endl;
        std::cout << atmo_coef(2, 1) << atmo_coef(2, 2) << atmo_coef(2, 3) << atmo_coef(2, 4) << std::endl;
        std::cout << atmo_coef(3, 1) << atmo_coef(3, 2) << atmo_coef(3, 3) << atmo_coef(3, 4) << std::endl;
        double amp1du = sqrt(pow(atmo_coef(1, 1), 2) + pow(atmo_coef(1, 2), 2));
        double amp2du = sqrt(pow(atmo_coef(1, 3), 2) + pow(atmo_coef(1, 4), 2));
        double amp1dn = sqrt(pow(atmo_coef(2, 1), 2) + pow(atmo_coef(2, 2), 2));
        double amp2dn = sqrt(pow(atmo_coef(2, 3), 2) + pow(atmo_coef(2, 4), 2));
        double amp1de = sqrt(pow(atmo_coef(3, 1), 2) + pow(atmo_coef(3, 2), 2));
        double amp2de = sqrt(pow(atmo_coef(3, 3), 2) + pow(atmo_coef(3, 4), 2));
        double pha1du = atan2(atmo_coef(1, 2), atmo_coef(1, 1)) / D2R;
        double pha2du = atan2(atmo_coef(1, 4), atmo_coef(1, 3)) / D2R;
        double pha1dn = atan2(atmo_coef(2, 2), atmo_coef(2, 1)) / D2R;
        double pha2dn = atan2(atmo_coef(2, 4), atmo_coef(2, 3)) / D2R;
        double pha1de = atan2(atmo_coef(3, 2), atmo_coef(3, 1)) / D2R;
        double pha2de = atan2(atmo_coef(3, 4), atmo_coef(3, 3)) / D2R;
        if (pha1du < 0) pha1du += 360.0;
        if (pha2du < 0) pha2du += 360.0;
        if (pha1dn < 0) pha1dn += 360.0;
        if (pha2dn < 0) pha2dn += 360.0;
        if (pha1de < 0) pha1de += 360.0;
        if (pha2de < 0) pha2de += 360.0;
        std::cout << amp1du << amp2du << amp1dn << amp2dn << amp1de << amp2de << std::endl;
        std::cout << pha1du << pha2du << pha1dn << pha2dn << pha1de << pha2de << std::endl;*/

        // displacement S1 = Amp1C * cos(freq1 * t) + Amp1S * sin(freq1 *t)
        // displacement S2 = Amp2C * cos(freq2 * t) + Amp2S * sin(freq2 *t)
        double amp1cdu = atmo_coef(0, 0);
        double amp1sdu = atmo_coef(0, 1);
        double amp2cdu = atmo_coef(0, 2);
        double amp2sdu = atmo_coef(0, 3);
        double amp1cdn = atmo_coef(1, 0);
        double amp1sdn = atmo_coef(1, 1);
        double amp2cdn = atmo_coef(1, 2);
        double amp2sdn = atmo_coef(1, 3);
        double amp1cde = atmo_coef(2, 0);
        double amp1sde = atmo_coef(2, 1);
        double amp2cde = atmo_coef(2, 2);
        double amp2sde = atmo_coef(2, 3);
        double dt = epoch.sod(false) + epoch.dsec(false) - epoch.leapsec(); // UTC = TAI - LS
        Triple dneu = Triple();
        dneu[0] = amp1cdn * cos(_atm_freq[0] * dt) + amp1sdn * sin(_atm_freq[0] * dt) + amp2cdn * cos(_atm_freq[1] * dt) + amp2sdn * sin(_atm_freq[1] * dt);
        dneu[1] = amp1cde * cos(_atm_freq[0] * dt) + amp1sde * sin(_atm_freq[0] * dt) + amp2cde * cos(_atm_freq[1] * dt) + amp2sde * sin(_atm_freq[1] * dt);
        dneu[2] = amp1cdu * cos(_atm_freq[0] * dt) + amp1sdu * sin(_atm_freq[0] * dt) + amp2cdu * cos(_atm_freq[1] * dt) + amp2sdu * sin(_atm_freq[1] * dt);
        Triple dxyz;
        neu2xyz(blh, dneu, dxyz);
        dxyz = dxyz / 1.e6; // mm -> km
        return dxyz;
    }

    Triple gnss_model_tideiers::tide_freq(const std::string &site, const Triple &xRec, double gast)
    {
        Triple blh;
        xyz2ell(xRec, blh, false);
        double lat = blh[0];
        double lon = blh[1];
        // double height = blh[2];

        double dotl = -0.0000253 * sin(lat) * cos(lat) * sin(gast + lon);
        Triple xyz = xRec * dotl / xRec.norm();

        return xyz;
    }

    void gnss_model_tideiers::getMeanPole(double mjd, double &xpm, double &ypm)
    {
        // 51544 is the mjd2000
        // 55197 is the mjd2010
        double mjd_2000 = 51544;
        double mjd_2010 = 55197;
        double dt = (mjd - mjd_2000) / 365.25;
        double xpole = 0.0, ypole = 0.0;
        switch (_mean_pole_model)
        {
        case modeofmeanpole::linear:
            xpole = 55.0 + 1.677 * dt;
            ypole = 320.5 + 3.460 * dt;
            break;
        case modeofmeanpole::cubic:
            if (mjd > mjd_2010)
            {
                xpole = 23.513 + 7.6141 * dt;
                ypole = 358.891 - 0.6287 * dt;
            }
            else
            {
                xpole = 55.974 + (1.8243 + (0.18413 + 0.007024 * dt) * dt) * dt;
                // -0.000908 -> +0.000908 by yqyuan, 2021/12/03
                //ypole = 346.346 + (1.7896 - (0.10729 - 0.000908 * dt) * dt) * dt;
                ypole = 346.346 + (1.7896 - (0.10729 + 0.000908 * dt) * dt) * dt;
            }
            break;
        }
        //mas to seconds
        xpm = xpole * (1e-3);
        ypm = ypole * (1e-3);
    }

    void gnss_model_tideiers::set_atm_grid(gnss_all_atmloading *atm_grid)
    {
        _atm_grid = atm_grid;
    }

    gnss_all_atmloading *gnss_model_tideiers::get_atm_grid()
    {
        return _atm_grid;
    }

    void gnss_model_tideiers::set_opl_grid(gnss_all_opl *opl_grid)
    {
        _opl = opl_grid;
    }

    Triple gnss_model_tideiers::load_oceanpole(const base_time &epo, const Triple &xRec, const double xp, const double yp)
    {
        Triple dxyz(0.0, 0.0, 0.0);
        Triple ell;
        Triple Rren, Iren;

        xyz2ell(xRec, ell, false);
        if (!_opl || _opl->data(xRec, Rren, Iren) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, "WARNING: Site not found in atmosphere non tidal loading file!");
            _mutex.unlock();
            return dxyz;
        }
        double xpole, ypole;
        getMeanPole(epo.dmjd(), xpole, ypole);
        double m1 = xp - xpole;
        double m2 = -(yp - ypole);
        double ga2R = 0.6870;
        double ga2I = 0.0036;
        double m1m2ga2 = m1 * ga2R + m2 * ga2I;
        double m2m1ga2 = m2 * ga2R - m1 * ga2I;
        m1m2ga2 = m1m2ga2 / 3600.0 / 180.0 * hwa_pi;
        m2m1ga2 = m2m1ga2 / 3600.0 / 180.0 * hwa_pi;
        Triple neu;
        neu[0] = K_OPL * (m1m2ga2 * Rren[1] + m2m1ga2 * Iren[1]);
        neu[1] = K_OPL * (m1m2ga2 * Rren[2] + m2m1ga2 * Iren[2]);
        neu[2] = K_OPL * (m1m2ga2 * Rren[0] + m2m1ga2 * Iren[0]);
        neu2xyz(ell, neu, dxyz);
        dxyz /= 1000.0;
        return dxyz;
    }

    void gnss_model_tideiers::set_mean_pole_model(modeofmeanpole mean_pole_model)
    {
        _mean_pole_model = mean_pole_model;
    }

    /*inline void gnss_model_tideiers::setDE(gnss_data_navde * nav_planet)
    {
        _nav_planet = nav_planet;
    }*/

    /*inline void gnss_model_tideiers::set_trs2crs(Matrix trs2crs)
    {
        _rot_trs2crs = trs2crs;
    }*/
}
