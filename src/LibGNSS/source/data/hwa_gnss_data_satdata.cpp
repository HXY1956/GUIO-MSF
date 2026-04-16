#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_navbds.h"
#include "hwa_gnss_data_navgps.h"
#include "hwa_gnss_data_navgal.h"
#include "hwa_gnss_data_navglo.h"
#include "hwa_gnss_data_navqzs.h"
#include "hwa_gnss_data_navsbs.h"
#include "hwa_gnss_data_navirn.h"

using namespace hwa_base;

namespace hwa_gnss
{
    gnss_data_sats::gnss_data_sats() : gnss_data_obs_manager()
    {
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }
    gnss_data_sats::gnss_data_sats(base_log spdlog) : gnss_data_obs_manager(spdlog)
    {
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }
    // -----------
    gnss_data_sats::gnss_data_sats(base_log spdlog, const std::string &site, const std::string &sat, const base_time &t)
        : gnss_data_obs_manager(spdlog, site, sat, t),
          _satcrd(0.0, 0.0, 0.0),
          _satpco(0.0, 0.0, 0.0),
          _dloudx(0.0, 0.0, 0.0),
          _drate(0.0),
          _clk(0.0),
          _dclk(0.0),
          _ele(0.0),
          _azi_rec(0.0),
          _rho(0.0),
          _eclipse(false),
          _mfH(0.0),
          _mfW(0.0),
          _mfG(0.0),
          _wind(0.0),
          _low_prec(false),
          _slipf(false),
          _beta_val(999),
          _orb_angle_val(999),
          _yaw(999)

    {
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }

    // xjhan
    gnss_data_sats::gnss_data_sats(base_log spdlog,
                           const std::string &site,
                           const std::string &sat,
                           const base_time &t,
                           const bool &isleo)
        : gnss_data_obs_manager(spdlog, site, sat, t, isleo),
          _satcrd(0.0, 0.0, 0.0),
          _satpco(0.0, 0.0, 0.0),
          _dloudx(0.0, 0.0, 0.0),
          _drate(0.0),
          _clk(0.0),
          _dclk(0.0),
          _ele(0.0),
          _azi_rec(0.0),
          _rho(0.0),
          _eclipse(false),
          _mfH(0.0),
          _mfW(0.0),
          _mfG(0.0),
          _wind(0.0),
          _low_prec(false),
          _slipf(false),
          _beta_val(999),
          _orb_angle_val(999),
          _yaw(999)

    {
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }

    // -----------
    gnss_data_sats::gnss_data_sats(const gnss_data_obs_manager &obs)
        : gnss_data_obs_manager(obs), // gnss_data_obs_manager(obs.site() ,obs.sat(), obs.epoch() ),
          _satcrd(0.0, 0.0, 0.0),
          _satpco(0.0, 0.0, 0.0),
          _clk(0.0),
          _ele(0.0),
          _azi_rec(0.0),
          _rho(0.0),
          _eclipse(false),
          _mfH(0.0),
          _mfW(0.0),
          _mfG(0.0),
          _wind(0.0),
          _low_prec(false),
          _slipf(false),
          _beta_val(999),
          _orb_angle_val(999),
          _drate(0.0),
          _yaw(999)
    {
        _spdlog = (obs.spdlog());
        id_type(base_data::SATDATA);
        id_group(base_data::GRP_obsERV);
    }

    // -----------
    gnss_data_sats::~gnss_data_sats()
    {
#ifdef DEBUG
        std::cout << "GSATDATA - destruct POCAT: " << site() << " " << sat() << " "
             << epoch().str("  %Y-%m-%d %H:%M:%S[%T] ") << std::fixed << std::setprecision(3) << std::endl;

        std::vector<GOBS> v_obs = this->obs();
        std::vector<GOBS>::iterator itOBS = v_obs.begin();
        for (; itOBS != v_obs.end(); ++itOBS)
            std::cout << " " << gnss_data_obs::gobs2str(*itOBS) << ":" << this->getobs(*itOBS);
        std::cout << std::endl;

        std::cout << "GSATDATA - destruct KONEC: " << site() << " " << sat() << " "
             << epoch().str("  %Y-%m-%d %H:%M:%S[%T] ");
        std::cout.flush();
#endif
    }

    // -----------
    void gnss_data_sats::addpco(const Triple &pco)
    {
        _satpco = pco;
        return;
    }

    // -----------
    void gnss_data_sats::addcrd(const Triple &crd)
    {
        _satcrd = crd;
        return;
    }

    void gnss_data_sats::addcrdcrs(const Triple &crd)
    {
        _satcrdcrs = crd;
        return;
    }

    // -----------
    void gnss_data_sats::addvel(const Triple &vel)
    {
        _satvel = vel;
        return;
    }

    // -----------
    void gnss_data_sats::addvel_crs(const Triple &vel)
    {
        _satvel_crs = vel;
        return;
    }

    // TODO:Single Freq Set Option
    //----------
    int gnss_data_sats::addprd(gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {

        //

        _low_prec = false;

        int irc = this->_addprd(gnav, corrTOT, msk_health);

        //
        return irc;
    }

    int gnss_data_sats::addprd_realtime(gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {

        //

        _low_prec = false;

        int irc = this->_addprd_realtime_new(gnav, corrTOT, msk_health);

        //
        return irc;
    }

    int gnss_data_sats::addprd(const base_time &T_sat, gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {
        _low_prec = false;

        int irc = this->_addprd(T_sat, gnav, corrTOT, msk_health);
        return irc;
    }

    //----------
    int gnss_data_sats::addprd_nav(gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {
        _low_prec = true;

        int irc = this->_addprd(gnav, corrTOT, msk_health);
        return irc;
    }

    // Compute rho, ele, azi ...
    // -----------------------
    int gnss_data_sats::cmpVal(const Triple &xyz)
    {

        if (double_eq(_satcrd[0], 0.0) ||
            double_eq(_satcrd[1], 0.0) ||
            double_eq(_satcrd[2], 0.0))
        {
            std::cerr << "Satellite position has not been calculated" << std::endl;
            return -1;
        }

        if (double_eq(xyz[0], 0.0) ||
            double_eq(xyz[1], 0.0) ||
            double_eq(xyz[2], 0.0))
        {
            std::cerr << "Station position has not been calculated" << std::endl;
            return -1;
        }

        Triple neu_sat;
        Triple ell_sit;
        xyz2ell(xyz, ell_sit, false);
        Triple xyz_rho = _satcrd - xyz;

        xyz2neu(ell_sit, xyz_rho, neu_sat);

        // Correct Earth rotation
        Vector xRec(3);
        double rho0 = (_satcrd - xyz).norm();
        double dPhi = OMEGA * rho0 / CLIGHT;

        xRec(0) = xyz[0] * cos(dPhi) - xyz[1] * sin(dPhi);
        xRec(1) = xyz[1] * cos(dPhi) + xyz[0] * sin(dPhi);
        xRec(2) = xyz[2];
        
        double tmp = (_satcrd - xRec).norm();
        _rho = tmp;

        double NE2 = neu_sat[0] * neu_sat[0] + neu_sat[1] * neu_sat[1];
        double ele = acos(sqrt(NE2) / _rho);
        if (neu_sat[2] < 0.0)
        {
            ele *= -1.0;
        }
        if (sqrt(NE2) / _rho > 1.0)
            _ele = 0.0;
        else
            _ele = ele;

        double azi = atan2(neu_sat[1], neu_sat[0]);
        if (azi < 0)
            azi += 2 * hwa_pi;
        _azi_rec = azi;

#ifdef DEBUG
        std::cout << site() << " " << xyz << std::endl
             << sat() << " " << _satcrd << std::endl
             << "ele: " << ele * R2D << " azi: " << azi * R2D << std::endl
             << std::endl;
#endif

        return 1;
    }

    // -----------
    void gnss_data_sats::addclk(const double &clk)
    {
        _clk = clk;
    }

    // -----------
    void gnss_data_sats::addreldelay(const double &rel)
    {
        _reldelay = rel;
    }

    void gnss_data_sats::addreccrd(const Triple &crd)
    {
        _reccrd = crd;
    }
    void gnss_data_sats::addreccrdcrs(const Triple &crd)
    {
        _reccrdcrs = crd;
    }
    void gnss_data_sats::addsat2reccrs(const Triple &crd)
    {
        _sat2reccrs = crd;
    }

    void gnss_data_sats::addTRS2CRS(const Matrix &rotmat, const Matrix &drdxpole, const Matrix &drdypole, const Matrix &drdut1)
    {
        _rotmat = rotmat;
        _drdxpole = drdxpole;
        _drdypole = drdypole;
        _drdut1 = drdut1;
    }

    void gnss_data_sats::addSCF2CRS(const Matrix &scf2crs, const Matrix &scf2trs)
    {
        _scf2crs = scf2crs;
        _scf2trs = scf2trs;
    }

    void gnss_data_sats::addorbfunct(const Matrix &orbfunct)
    {
        _orbfunct = orbfunct;
    }

    void gnss_data_sats::adddrate(const double &drate)
    {
        _drate = drate;
    }

    void gnss_data_sats::adddloudx(const Triple &unit)
    {
        _dloudx = unit;
    }

    void gnss_data_sats::addsatindex(const int &idx)
    {
        _satindex = idx;
    }

    void gnss_data_sats::addrecTime(const base_time &recTime)
    {
        _TR = recTime;
    }
    void gnss_data_sats::addsatTime(const base_time &satTime)
    {
        _TS = satTime;
    }

    // -----------
    void gnss_data_sats::addele(const double &ele)
    {
        _ele = ele;
    }

    void gnss_data_sats::addele_leo(const double &ele)
    {
        _ele_leo = ele;
    }

    // -----------
    void gnss_data_sats::addazi_rec(const double &azi)
    {
        _azi_rec = azi;
    }

    void gnss_data_sats::addzen_rec(const double &zen)
    {
        _zen_rec = zen;
    }

    void gnss_data_sats::addazi_sat(const double &azi_sat)
    {
        _azi_sat = azi_sat;
    }

    void gnss_data_sats::addzen_sat(const double &zen_sat)
    {
        _zen_sat = zen_sat;
    }

    void gnss_data_sats::addnadir(const double &nadir)
    {
        _nadir = nadir;
    }

    // -----------
    void gnss_data_sats::addrho(const double &rho)
    {
        _rho = rho;
    }

    // -----------
    void gnss_data_sats::addmfH(const double &mfH)
    {
        _mfH = mfH;
    }

    // -----------
    void gnss_data_sats::addmfW(const double &mfW)
    {
        _mfW = mfW;
    }

    // -----------
    void gnss_data_sats::addmfG(const double &mfG)
    {
        _mfG = mfG;
    }

    // -----------
    const Triple &gnss_data_sats::satcrd() const
    {
        return _satcrd;
    }

    const Triple &gnss_data_sats::satcrdcrs() const
    {
        return _satcrdcrs;
    }
    // -----------
    const Triple &gnss_data_sats::satpco() const
    {
        return _satpco;
    }

    // -----------
    const Triple &gnss_data_sats::satvel() const
    {
        return _satvel;
    }

    // -----------
    const Triple &gnss_data_sats::satvel_crs() const
    {
        return _satvel_crs;
    }

    // -----------
    const double &gnss_data_sats::clk() const
    {
        return _clk;
    }

    const double &gnss_data_sats::dclk() const
    {
        return _dclk;
    }

    // -----------
    const double &gnss_data_sats::reldelay() const
    {
        return _reldelay;
    }

    const double &gnss_data_sats::drate() const
    {
        return _drate;
    }

    const Triple &gnss_data_sats::dloudx() const
    {
        return _dloudx;
    }

    // -----------
    const Matrix &gnss_data_sats::orbfunct() const
    {
        return _orbfunct;
    }
    // -----------
    int gnss_data_sats::satindex()
    {
        return _satindex;
    }
    // -----------
    const Triple &gnss_data_sats::reccrd() const
    {
        return _reccrd;
    }
    const Triple &gnss_data_sats::reccrdcrs() const
    {
        return _reccrdcrs;
    }
    const Triple &gnss_data_sats::sat2reccrs() const
    {
        return _sat2reccrs;
    }
    // -----------
    const Matrix &gnss_data_sats::rotmat() const
    {
        return _rotmat;
    }

    // -----------
    const Matrix &gnss_data_sats::drdxpole() const
    {
        return _drdxpole;
    }

    // -----------
    const Matrix &gnss_data_sats::drdypole() const
    {
        return _drdypole;
    }

    // -----------
    const Matrix &gnss_data_sats::drdut1() const
    {
        return _drdut1;
    }
    const Matrix &gnss_data_sats::scf2crs() const
    {
        return _scf2crs;
    }
    const Matrix &gnss_data_sats::scf2trs() const
    {
        return _scf2trs;
    }
    // -----------
    const base_time &gnss_data_sats::recTime() const
    {
        return _TR;
    }

    // -----------
    const base_time &gnss_data_sats::satTime() const
    {
        return _TS;
    }

    // -----------
    const double &gnss_data_sats::ele() const
    {
        return _ele;
    }

    const double &gnss_data_sats::ele_leo() const
    {
        return _ele_leo;
    }

    // -----------
    double gnss_data_sats::ele_deg() const
    {
        double tmp = _ele * 180.0 / hwa_pi;
        return tmp;
    }

    double gnss_data_sats::ele_leo_deg() const
    {
        double tmp = _ele_leo * 180.0 / hwa_pi;
        return tmp;
    }

    // -----------
    const double &gnss_data_sats::azi() const
    {
        return _azi_rec;
    }

    const double &gnss_data_sats::azi_sat() const
    {

        return _azi_sat;
    }

    const double &gnss_data_sats::nadir() const
    {

        return _nadir;
    }

    // -----------
    const double &gnss_data_sats::rho() const
    {
        return _rho;
    }

    // valid
    // ----------
    bool gnss_data_sats::valid()
    {
        bool tmp = this->_valid();
        return tmp;
    }

    bool gnss_data_sats::is_carrier_range(const GOBSBAND& band) const
    {
        if (_is_carrier_range.find(band) == _is_carrier_range.end())
            return false;
        else
            return _is_carrier_range.at(band);
    }

    // -----------
    void gnss_data_sats::addslip(const bool &flag)
    {
        _slipf = flag;
    }

    // -----------
    const bool &gnss_data_sats::islip() const
    {
        return _slipf;
    }

    double gnss_data_sats::beta()
    {
        double tmp = _b();
        return tmp;
    }

    double gnss_data_sats::orb_angle()
    {
        double tmp = _orb_angle();
        return tmp;
    }

    // clean data
    // ----------
    void gnss_data_sats::clear()
    {
        this->_clear();
        return;
    }

    void gnss_data_sats::is_carrier_range(const GOBSBAND & band, const bool & b)
    {
         _is_carrier_range[band] = b; ;
    }

    //----------
    int gnss_data_sats::_addprd(gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {

        std::string satname(_satid);

        GSYS gs = this->gsys();

        GOBSBAND b1, b2;
        b1 = b2 = BAND;

        // automatic selection of two bands for IF LC
        std::set<GOBSBAND> bands = _band_avail_code();
        auto itBAND = bands.begin();
        if (corrTOT)
        {
            if (bands.size() < 1)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "At least two bands are necessary for TOT correction in sat pos/clk calculation!");
                return -1;
            }
            else if (bands.size() < 2)
            {
                b1 = *itBAND;
            }
            else
            {
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
            }
        }
        else
        {
            b1 = *itBAND;
        }

        double P3 = 0.0;
        if (b1 != BAND && b2 != BAND)
            P3 = this->P3(b1, b2);
        if (double_eq(P3, 0.0))
            P3 = this->obs_C(gnss_data_band(b1, GOBSATTR::ATTR));

        //test for observations availability
        if (gnav == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms("  gnss_all_nav pointer is not available "));
            //      std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " gnss_all_nav pointer is not available" << std::endl;
            return -1;
        }

        //test for observations availability
        if (double_eq(P3, 0.0) && corrTOT)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms(" P3 = 0!"));
            //                   std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " P3 = 0;" << std::endl;
            return -1;
        }

        double xyz[3] = {0.0, 0.0, 0.0};
        double vel[3] = {0.0, 0.0, 0.0};
        double var[3] = {0.0, 0.0, 0.0};
        double clk = 0.0;
        double dclk = 0.0;
        double clkrms = 0.0;

        if (satname.substr(0, 1) != "G" &&
            satname.substr(0, 1) != "R" &&
            satname.substr(0, 1) != "E" &&
            satname.substr(0, 1) != "J" &&
            //       satname.substr(0,1) != "S" &&
            satname.substr(0, 1) != "C")
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" Undefined satellite system! "));
            return -1;
        }

        base_time epoT(base_time::GPS);
        double satclk = 0.0;
        double satclk2 = 1.0;
        int cnt = 0;

        if (corrTOT)
        {
            while (fabs(satclk - satclk2) > 1.e-3 / CLIGHT)
            {
                satclk2 = satclk;
                epoT = _epoch - P3 / CLIGHT - satclk;

                int irc = gnav->clk(satname, epoT, &clk, &clkrms, &dclk, msk_health);

#ifdef DEBUG
                std::cout << "gsatdata: " << _epoch.str_ymdhms() << " " << satname << std::fixed << std::setprecision(3)
                     << " " << satclk << " " << satclk2 << " " << (satclk - satclk2) * CLIGHT
                     << " " << clk << " " << clkrms << " " << dclk << " " << P3 << " " << irc << std::endl;
#endif
                if (irc < 0 || cnt++ > 25)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" clocks not calculated (irc|iter) for epoch: "));
                    //        std::cerr << satname + _epoch.str_ymdhms(" clocks not calculated (irc|iter) for epoch: ") << std::endl;
                    return -1;
                }
                satclk = clk;
            }
        }
        else
        {
            epoT = _epoch;
            int irc = gnav->clk(satname, epoT, &satclk, &clkrms, &dclk, msk_health);
            if (irc < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" clocks not calculated for epoch "));
                //                 std::cout << satname + _epoch.str_ymdhms(" clocks not calculated for epoch ") << std::endl;
                return -1;
            }
        }

        int irc = 0;
        irc = gnav->pos(satname, epoT, xyz, var, vel, msk_health);

        //   if (_low_prec) irc = gnav->nav( satname, epoT, xyz, var, vel ); // sometimes problem !?
        //   else           irc = gnav->pos( satname, epoT, xyz, var, vel ); // OK!
        // =======================================================================
        // TADY JE OBCAS PROBLEM S NAV() PRO VYPOCET ELEVACE/AZIMUTU V ANUBIS!!!!
        // =======================================================================

        if (irc < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates not calculated for epoch "));
            //    std::cout << satname + _epoch.str_ymdhms(" coordinates not calculated for epoch ") << std::endl;
            return -1;
        }

        Triple txyz = fromarray(xyz);
        Triple tvel = fromarray(vel);

        // relativistic correction
        // WARNING: GLONASS clk already include the correction if broadcast eph are used !!!!!
        if (gs != GLO ||
            (gs == GLO && gnav->id_type() == base_data::ALLPREC))
        {
            double rel = 2.0 * (txyz[0] * vel[0] + txyz[1] * vel[1] + txyz[2] * vel[2]) / CLIGHT / CLIGHT; //default
            std::shared_ptr<gnss_data_eph> eph = gnav->find(satname, epoT);
            if (gs == BDS && eph && gnav->id_type() == base_data::ALLRTCM)
            {
                std::shared_ptr<gnss_data_navbds> gnavb = std::dynamic_pointer_cast<gnss_data_navbds>(eph);
                double clk0 = 0.0, dclk0 = 0.0, clkrms0 = 0.0;
                gnavb->clk(epoT, &clk0, &clkrms0, &dclk0, msk_health);
                rel = gnavb->rel();
            }
            else if (gs == GAL && eph && gnav->id_type() == base_data::ALLRTCM)
            {
                std::shared_ptr<gnss_data_navgal> gnave = std::dynamic_pointer_cast<gnss_data_navgal>(eph);
                double clk0 = 0.0, dclk0 = 0.0, clkrms0 = 0.0;
                gnave->clk(epoT, &clk0, &clkrms0, &dclk0, msk_health);
                rel = gnave->rel();
            }

            if (rel == 0.0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" relativity correction not calculated for epoch "));
                return -1;
            }

            satclk -= rel;
            _TS = epoT;
            _reldelay = rel * CLIGHT;
        }

        // filling gsatdata

        _satcrd = txyz;
        _satvel = tvel;
        _clk = satclk * CLIGHT;
        _dclk = dclk * CLIGHT;

#ifdef DEBUG
        std::ostringstream os;
        os << "gsatdata " << satname
           << " CRD " << std::fixed << std::setprecision(3)
           << "  " << epoT.str_ymdhms()
           << " X " << std::setw(14) << txyz[0]
           << " Y " << std::setw(14) << txyz[1]
           << " Z " << std::setw(14) << txyz[2]
           << " T " << std::setw(14) << satclk * 1000.0 * 1000.0;
        //if (_log) _log->comment(2, "gsatdata", os.str());
        std::cout << os.str() << std::endl;
#endif
        return 1;
    }

    int gnss_data_sats::_addprd_realtime(gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {

        std::string satname(_satid);

        GSYS gs = this->gsys();

        GOBSBAND b1, b2;
        b1 = b2 = BAND;

        // automatic selection of two bands for IF LC
        std::set<GOBSBAND> bands = _band_avail_code();
        auto itBAND = bands.begin();

        if (corrTOT)
        {
            if (bands.size() < 1)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "At least two bands are necessary for TOT correction in sat pos/clk calculation!");
                return -1;
            }
            else if (bands.size() < 2)
            {
                b1 = *itBAND;
            }
            else
            {
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
            }
        }
        else
        {
            b1 = *itBAND;
        }

        double P3 = this->P3(b1, b2);
        if (double_eq(P3, 0.0))
            P3 = this->obs_C(gnss_data_band(b1, GOBSATTR::ATTR));

        //test for observations availability
        if (gnav == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms("  gnss_all_nav pointer is not available "));
            //      std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " gnss_all_nav pointer is not available" << std::endl;
            return -1;
        }

        //test for observations availability
        if (double_eq(P3, 0.0) && corrTOT)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms(" P3 = 0!"));
            //                   std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " P3 = 0;" << std::endl;
            return -1;
        }

        double xyz[3] = {0.0, 0.0, 0.0};
        double xyz_corr[3] = {0.0, 0.0, 0.0};
        double vel[3] = {0.0, 0.0, 0.0};
        double vel_corr[3] = {0.0, 0.0, 0.0};
        double var[3] = {0.0, 0.0, 0.0};
        double clk = 0.0;
        double clk_corr = 0.0;
        double dclk = 0.0;
        double dclk_corr = 0.0;
        double clkrms = 0.0;

        if (satname.substr(0, 1) != "G" &&
            satname.substr(0, 1) != "R" &&
            satname.substr(0, 1) != "E" &&
            satname.substr(0, 1) != "J" &&
            //       satname.substr(0,1) != "S" &&
            satname.substr(0, 1) != "C")
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" Undefined satellite system! "));
            return -1;
        }

        base_time epoT(base_time::GPS);
        double satclk = 0.0;
        double satclk2 = 1.0;
        int cnt = 0;

        if (corrTOT)
        {
            while (fabs(satclk - satclk2) > 1.e-3 / CLIGHT)
            {
                satclk2 = satclk;
                epoT = _epoch - P3 / CLIGHT - satclk;

                int irc = gnav->clk(satname, epoT, &clk, &clkrms, &dclk, msk_health);

#ifdef DEBUG
                std::cout << "gsatdata: " << _epoch.str_ymdhms() << " " << satname << std::fixed << std::setprecision(3)
                     << " " << satclk << " " << satclk2 << " " << (satclk - satclk2) * CLIGHT
                     << " " << clk << " " << clkrms << " " << dclk << " " << P3 << " " << irc << std::endl;
#endif
                if (irc < 0 || cnt++ > 25)
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" clocks not calculated (irc|iter) for epoch: "));
                    //        std::cerr << satname + _epoch.str_ymdhms(" clocks not calculated (irc|iter) for epoch: ") << std::endl;
                    return -1;
                }
                satclk = clk;
            }
        }
        else
        {
            epoT = _epoch;
            int irc = gnav->clk(satname, epoT, &satclk, &clkrms, &dclk, msk_health);
            if (irc < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" clocks not calculated for epoch "));
                //                 std::cout << satname + _epoch.str_ymdhms(" clocks not calculated for epoch ") << std::endl;
                return -1;
            }
        }

        int irc = 0;
        irc = gnav->pos(satname, epoT, xyz, var, vel, msk_health);

        if (irc < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates not calculated for epoch "));
            //    std::cout << satname + _epoch.str_ymdhms(" coordinates not calculated for epoch ") << std::endl;
            return -1;
        }

        //// get the eph iod issue
        //int iod = gnav->get_iod(satname, epoT);

        //irc = gnav->get_pos_clk_correction(satname, epoT, iod, xyz_corr, vel_corr, clk_corr, dclk_corr);
        //if (irc < 0) {
        //    if (_log)
        //        _log->comment(2, "gsatdata", " satelite " + satname
        //            + _epoch.str_ymdhms(" coordinates and clock correction is not exist for epoch") + "(eph_iod: " + base_type_conv::int2str(iod) + " )");
        //    return -1;
        //}

        std::pair<int, int> iods = gnav->get_iod(satname, epoT);
        
        irc = dynamic_cast<gnss_all_prec*>(gnav)->get_pos_clk_correction(satname, epoT, iods.first, xyz_corr, vel_corr, clk_corr, dclk_corr);
        //irc = gnav->get_pos_clk_correction(satname, epoT, iods.first, xyz_corr, vel_corr, clk_corr, dclk_corr);
        if (irc < 0)
        {
            if (gnav->get_pos_clk_correction(satname, epoT, iods.second, xyz_corr, vel_corr, clk_corr, dclk_corr) < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates and clock correction is not exist for ephemeris") + "(LastEph_IOD: " + base_type_conv::int2str(iods.first) + ", PrevEph_IOD: " + base_type_conv::int2str(iods.second) + " )");
                return -1;
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates and clock correction is exist for prev-ephemeris") + "(LastEph_IOD: " + base_type_conv::int2str(iods.first) + ", PrevEph_IOD: " + base_type_conv::int2str(iods.second) + " )");
            }
        }

        irc = this->_correction(xyz, vel, satclk, dclk, xyz_corr, vel_corr, clk_corr, dclk_corr);

        Triple txyz = fromarray(xyz);
        Triple tvel = fromarray(vel);

        // relativistic correction
        // WARNING: GLONASS clk already include the correction if broadcast eph are used !!!!!
        if (gs != GLO || (gs == GLO && gnav->id_type() == base_data::ALLPREC))
        {
            double rel = 2.0 * (txyz[0] * vel[0] + txyz[1] * vel[1] + txyz[2] * vel[2]) / CLIGHT / CLIGHT; //default
            std::shared_ptr<gnss_data_eph> eph = gnav->find(satname, epoT);
            if (gs == BDS && eph && gnav->id_type() == base_data::ALLRTCM)
            {
                std::shared_ptr<gnss_data_navbds> gnavb = std::dynamic_pointer_cast<gnss_data_navbds>(eph);
                double clk0 = 0.0, dclk0 = 0.0, clkrms0 = 0.0;
                gnavb->clk(epoT, &clk0, &clkrms0, &dclk0, msk_health);
                rel = gnavb->rel();
            }
            else if (gs == GAL && eph && gnav->id_type() == base_data::ALLRTCM)
            {
                std::shared_ptr<gnss_data_navgal> gnave = std::dynamic_pointer_cast<gnss_data_navgal>(eph);
                double clk0 = 0.0, dclk0 = 0.0, clkrms0 = 0.0;
                gnave->clk(epoT, &clk0, &clkrms0, &dclk0, msk_health);
                rel = gnave->rel();
            }

            if (rel == 0.0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" relativity correction not calculated for epoch "));
                return -1;
            }

            satclk -= rel;
            _TS = epoT;
            _reldelay = rel * CLIGHT;
        }

        // filling gsatdata

        _satcrd = txyz;
        _satvel = tvel;
        _clk = satclk * CLIGHT;
        _dclk = dclk * CLIGHT;

#ifdef DEBUG
        std::ostringstream os;
        os << "gsatdata " << satname
           << " CRD " << std::fixed << std::setprecision(3)
           << "  " << epoT.str_ymdhms()
           << " X " << std::setw(14) << txyz[0]
           << " Y " << std::setw(14) << txyz[1]
           << " Z " << std::setw(14) << txyz[2]
           << " T " << std::setw(14) << satclk * 1000.0 * 1000.0;
        //if (_log) _log->comment(2, "gsatdata", os.str());
        std::cout << os.str() << std::endl;
#endif
        return 1;
    }

    int gnss_data_sats::_addprd_realtime_new(gnss_all_nav* gnav, bool corrTOT, bool msk_health)
    {

        std::string satname(_satid);

        GSYS gs = this->gsys();

        GOBSBAND b1, b2;
        b1 = b2 = BAND;

        // automatic selection of two bands for IF LC
        std::set<GOBSBAND> bands = _band_avail_code();
        auto itBAND = bands.begin();

        if (corrTOT) {
            if (bands.size() < 1) {
                std::cout << "gnss_data_sats bands.size() < 2" << std::endl;
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "At least two bands are necessary for TOT correction in sat pos/clk calculation!");
                return -1;
            }
            else if (bands.size() < 2)
            {
                b1 = *itBAND;
            }
            else
            {
                b1 = *itBAND;
                itBAND++;
                b2 = *itBAND;
            }
        }
        else
        {
            b1 = *itBAND;
        }

        double P3 = this->P3(b1, b2);
        if (double_eq(P3, 0.0)) P3 = this->obs_C(gnss_data_band(b1, GOBSATTR::ATTR));


        //test for observations availability
        if (gnav == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms("  gnss_all_nav pointer is not available "));
            //      std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " gnss_all_nav pointer is not available" << std::endl;
            return -1;
        }

        //test for observations availability
        if (double_eq(P3, 0.0) && corrTOT)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms(" P3 = 0!"));
            //                   std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " P3 = 0;" << std::endl;
            return -1;
        }

        double xyz[3] = { 0.0, 0.0, 0.0 };
        double xyz_corr[3] = { 0.0, 0.0, 0.0 };
        double vel[3] = { 0.0, 0.0, 0.0 };
        double vel_corr[3] = { 0.0, 0.0, 0.0 };
        double var[3] = { 0.0, 0.0, 0.0 };
        double clk = 0.0;
        double clk_corr = 0.0;
        double dclk = 0.0;
        double dclk_corr = 0.0;
        double clkrms = 0.0;

        if (satname.substr(0, 1) != "G" &&
            satname.substr(0, 1) != "R" &&
            satname.substr(0, 1) != "E" &&
            satname.substr(0, 1) != "J" &&
            //       satname.substr(0,1) != "S" &&       
            satname.substr(0, 1) != "C")
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" Undefined satellite system! "));
            return -1;
        }

        base_time epoT(base_time::GPS);
        double satclk = 0.0;
        double satclk2 = 1.0;
        int cnt = 0;
        //int pv_iod = -1;
        //int clk_iod = -1;

        if (dynamic_cast<gnss_all_prec*>(gnav)->get_ssr_iod(satname, _epoch, _pv_iod, _clk_iod) < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates and clock correction is not exist for ephemeris ")
                ++ "(PV_IOD: " + base_type_conv::int2str(_pv_iod) + ", CLK_IOD: " + base_type_conv::int2str(_clk_iod) + " )");
            return -1;
        }
        if (_pv_iod != _clk_iod)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" PC_IOD and CLK_IOD is not equal for ephemeris ")
                    ++ "(PV_IOD: " + base_type_conv::int2str(_pv_iod) + ", CLK_IOD: " + base_type_conv::int2str(_clk_iod) + " )");
            return -1;
        }

        if (corrTOT) {
            while (fabs(satclk - satclk2) > 1.e-3 / CLIGHT) {
                satclk2 = satclk;
                epoT = _epoch - P3 / CLIGHT - satclk;

                // tyx debug
                //if (satname == "G27")
                //{
                //    std::cerr << std::endl;
                //}

                int irc = gnav->clk(satname, _pv_iod, epoT, &clk, &clkrms, &dclk, msk_health);
                //int irc = gnav->clk(satname, epoT, &clk, &clkrms, &dclk, msk_health);

#ifdef DEBUG
                std::cout << "gsatdata: " << _epoch.str_ymdhms() << " " << satname << std::fixed << std::setprecision(3)
                    << " " << satclk << " " << satclk2 << " " << (satclk - satclk2) * CLIGHT
                    << " " << clk << " " << clkrms << " " << dclk << " " << P3 << " " << irc << std::endl;
#endif     
                if (irc < 0 || cnt++ > 25) {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" clocks not calculated (irc|iter) for epoch: "));
                    //        std::cerr << satname + _epoch.str_ymdhms(" clocks not calculated (irc|iter) for epoch: ") << std::endl;
                    return -1;
                }
                satclk = clk;
            }
        }
        else {
            epoT = _epoch;

            int irc = gnav->clk(satname, _pv_iod, epoT, &satclk, &clkrms, &dclk, msk_health);
            //int irc = gnav->clk(satname, epoT, &clk, &clkrms, &dclk, msk_health);
            if (irc < 0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" clocks not calculated for epoch "));
                //                 std::cout << satname + _epoch.str_ymdhms(" clocks not calculated for epoch ") << std::endl;
                return -1;
            }
        }


        int irc = 0;

        //if (satname[0] == 'E')
        //{
        //    std::cerr << std::endl;
        //}

        irc = gnav->pos(satname, _pv_iod, epoT, xyz, var, vel, msk_health);

        if (irc < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates not calculated for epoch "));
            //    std::cout << satname + _epoch.str_ymdhms(" coordinates not calculated for epoch ") << std::endl;
            return -1;
        }

        irc = gnav->get_pos_clk_correction(satname, epoT, _pv_iod, xyz_corr, vel_corr, clk_corr, dclk_corr);

        std::ostringstream os;
        os << "************************* new  sat  prn " << satname << "*************************" << std::endl;
        os << "gsatdata " << satname
            << " CRD " << std::fixed << std::setprecision(3)
            << "  " << epoT.str_ymdhms()
            << "  " << epoT.sow() + epoT.dsec()
            << " PV_IOD " << std::setw(5) << _pv_iod
            << " X before correction " << std::setw(14) << xyz[0]
            << " Y before correction " << std::setw(14) << xyz[1]
            << " Z before correction " << std::setw(14) << xyz[2]
            << " sat clk before correction " << std::setw(14) << satclk * CLIGHT << std::endl;

        irc = this->_correction(xyz, vel, satclk, dclk, xyz_corr, vel_corr, clk_corr, dclk_corr);

        if (irc < 0) {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" coordinates and clock correction is not exist for ephemeris") + "(LastEph_IOD: " + base_type_conv::int2str(iods.first) + ", PrevEph_IOD: " + base_type_conv::int2str(iods.second) + " )");
            return -1;
        }


        os << " X correction " << std::setw(14) << xyz_corr[0]
            << " Y correction " << std::setw(14) << xyz_corr[1]
            << " Z correction " << std::setw(14) << xyz_corr[2]
            << " sat clk correction " << std::setw(14) << clk_corr * CLIGHT
            << " X after correction " << std::setw(14) << xyz[0]
            << " Y after correction " << std::setw(14) << xyz[1]
            << " Z after correction " << std::setw(14) << xyz[2]
            << " sat clk after correction " << std::setw(14) << satclk * CLIGHT << std::endl;

        

        // tyx debug
        std::cout << os.str() << std::endl;

        Triple txyz = fromarray(xyz);
        Triple tvel = fromarray(vel);

        // relativistic correction
        // WARNING: GLONASS clk already include the correction if broadcast eph are used !!!!!
        if (gs != GLO || (gs == GLO && gnav->id_type() == base_data::ALLPREC))
        {
            double rel = 2.0 * (txyz[0] * vel[0] + txyz[1] * vel[1] + txyz[2] * vel[2]) / CLIGHT / CLIGHT; //default
            std::shared_ptr<gnss_data_eph> eph = gnav->find(satname, epoT);
            if (gs == BDS && eph && gnav->id_type() == base_data::ALLRTCM) {
                std::shared_ptr <gnss_data_navbds> gnavb = std::dynamic_pointer_cast <gnss_data_navbds>(eph);
                double clk0 = 0.0, dclk0 = 0.0, clkrms0 = 0.0;
                gnavb->clk(epoT, &clk0, &clkrms0, &dclk0, msk_health);
                rel = gnavb->rel();
            }
            else if (gs == GAL && eph && gnav->id_type() == base_data::ALLRTCM) {
                std::shared_ptr <gnss_data_navgal> gnave = std::dynamic_pointer_cast <gnss_data_navgal>(eph);
                double clk0 = 0.0, dclk0 = 0.0, clkrms0 = 0.0;
                gnave->clk(epoT, &clk0, &clkrms0, &dclk0, msk_health);
                rel = gnave->rel();
            }

            if (rel == 0.0)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" relativity correction not calculated for epoch "));
                return -1;
            }

            satclk -= rel;
            _TS = epoT;
            _reldelay = rel * CLIGHT;
        }

        // filling gsatdata

        _satcrd = txyz;
        _satvel = tvel;
        _clk = satclk * CLIGHT;
        _dclk = dclk * CLIGHT;


#ifdef DEBUG   
        std::ostringstream os;
        os << "gsatdata " << satname
            << " CRD " << std::fixed << std::setprecision(3)
            << "  " << epoT.str_ymdhms()
            << " X " << std::setw(14) << txyz[0]
            << " Y " << std::setw(14) << txyz[1]
            << " Z " << std::setw(14) << txyz[2]
            << " T " << std::setw(14) << satclk * 1000.0 * 1000.0;
        //if (_log) _log->comment(2, "gsatdata", os.str());
        std::cout << os.str() << std::endl;
#endif
        return 1;
    }

    // update sat clk
    int gnss_data_sats::_addprd(const base_time &T_sat, gnss_all_nav *gnav, const bool &corrTOT, const bool &msk_health)
    {

        std::string satname(_satid);

        // GSYS gs = this->gsys();

        //test for observations availability
        if (gnav == 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satellite " + satname + _epoch.str_ymdhms("  gnss_all_nav pointer is not available "));
            //      std::cout << "Add prd " << epoch().str_hms() << " " << sat() << " gnss_all_nav pointer is not available" << std::endl;
            return -1;
        }

        double xyz[3] = {0.0, 0.0, 0.0};
        double vel[3] = {0.0, 0.0, 0.0};
        double var[3] = {0.0, 0.0, 0.0};
        double satclk = 0.0;
        double dclk = 0.0;
        double clkrms = 0.0;

        if (satname.substr(0, 1) != "G" &&
            satname.substr(0, 1) != "R" &&
            satname.substr(0, 1) != "E" &&
            satname.substr(0, 1) != "J" &&
            //       satname.substr(0,1) != "S" &&
            satname.substr(0, 1) != "C")
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + _epoch.str_ymdhms(" Undefined satellite system! "));
            return -1;
        }

        int irc = 0;
        irc = gnav->clk(satname, T_sat, &satclk, &clkrms, &dclk, msk_health);
        if (irc < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + T_sat.str_ymdhms(" clocks not calculated for epoch "));
            //                 std::cout << satname + _epoch.str_ymdhms(" clocks not calculated for epoch ") << std::endl;
            return -1;
        }

        irc = gnav->pos(satname, T_sat, xyz, var, vel, msk_health);

        //   if (_low_prec) irc = gnav->nav( satname, epoT, xyz, var, vel ); // sometimes problem !?
        //   else           irc = gnav->pos( satname, epoT, xyz, var, vel ); // OK!
        // =======================================================================
        // TADY JE OBCAS PROBLEM S NAV() PRO VYPOCET ELEVACE/AZIMUTU V ANUBIS!!!!
        // =======================================================================

        if (irc < 0)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, " satelite " + satname + T_sat.str_ymdhms(" coordinates not calculated for epoch "));
            //    std::cout << satname + _epoch.str_ymdhms(" coordinates not calculated for epoch ") << std::endl;
            return -1;
        }

        Triple txyz = fromarray(xyz);
        Triple tvel = fromarray(vel);

        // filling gsatdata
        _satcrd = txyz;
        _satvel = tvel;
        _TS = T_sat;

        return 1;
    }

    int gnss_data_sats::_correction(double *xyz, double *vxyz, double &clk, double &dclk, double *xyz_corr, double *vxyz_corr, double &clk_corr, double &dclk_corr)
    {
        Triple pos = fromarray(xyz); 
        Triple vel = fromarray(vxyz);
        Triple rao_vel = fromarray(vxyz_corr);
        Triple rao_pos = fromarray(xyz_corr);
        Triple out = Triple::Zero();
        
        rao2xyz(pos, vel, rao_pos, out);
        for (size_t i = 0; i < 3; i++)
        {
            xyz[i] -= out(i);
        }
        rao2xyz(pos, vel, rao_vel, out);
        for (size_t i = 0; i < 3; i++)
        {
            vxyz[i] -= out(i);
        }
        clk += clk_corr / CLIGHT;
        dclk += dclk_corr / CLIGHT;
        return 1;
    }

    // clean internal function
    // ----------
    void gnss_data_sats::_clear()
    {

        gnss_data_obs_manager::_clear();
        //  _satcrd[0] = 0.0;
        //  _satcrd[1] = 0.0;
        //  _satcrd[2] = 0.0;
        _ele = 0.0;
        _azi_rec = 0.0;
        _rho = 0.0;
        _clk = 0.0;
    }

    // clean internal function
    // ----------
    bool gnss_data_sats::_valid() const
    {

        if (_rho == 0.0 || // single validity identification for gsatdata!
            gnss_data_obs_manager::_valid())
            return false;

        return true;
    }

    // reset eclipsing flag
    // ---------------------
    void gnss_data_sats::setecl(const bool &ecl)
    {
        _eclipse = ecl;
    }

    // get eclipsing
    // ---------------------
    const bool &gnss_data_sats::ecl() const
    {

        return _eclipse;
    }

    // determi ne wheather eclipsed or not
    // -------------------------------------
    void gnss_data_sats::addecl(std::map<std::string, base_time> &lastEcl)
    {

        if (fabs(_b()) < EPS0_GPS && fabs(_orb_angle()) < EPS0_GPS)
        {
            _eclipse = true;
            lastEcl[_satid] = _epoch;
            return;
        }
        else
        {
            auto itLast = lastEcl.find(_satid);
            if (itLast != lastEcl.end())
            {
                double tdiff = _epoch.diff(itLast->second);
                if (abs(tdiff) <= POST_SHADOW)
                    _eclipse = true;
                else
                    _eclipse = false;
            }
        }
    }

    // add postfit residuals
    //------------------
    void gnss_data_sats::addres(const RESIDTYPE &restype, const GOBSTYPE &type, const double &res)
    {
        if (restype == RESIDTYPE::RES_ORIG)
        {
            if (type == TYPE_C)
                _code_res_orig.push_back(res);
            if (type == TYPE_L)
                _phase_res_orig.push_back(res);
        }

        if (restype == RESIDTYPE::RES_NORM)
        {
            if (type == TYPE_C)
                _code_res_norm.push_back(res);
            if (type == TYPE_L)
                _phase_res_norm.push_back(res);
        }
    }

    // get postfit residuals
    //------------------
    std::vector<double> gnss_data_sats::residuals(const RESIDTYPE &restype, const GOBSTYPE &type)
    {
        std::vector<double> res;

        if (restype == RESIDTYPE::RES_ORIG)
        {
            if (type == TYPE_C)
                res = _code_res_orig;
            else if (type == TYPE_L)
                res = _phase_res_orig;
        }

        if (restype == RESIDTYPE::RES_NORM)
        {
            if (type == TYPE_C)
                res = _code_res_norm;
            else if (type == TYPE_L)
                res = _phase_res_norm;
        }
        return res;
    }

    // clean residuals
    // ---------------
    void gnss_data_sats::clear_res(const RESIDTYPE &restype)
    {
        if (restype == RESIDTYPE::RES_ORIG)
        {
            _code_res_orig.clear();
            _phase_res_orig.clear();
        }

        if (restype == RESIDTYPE::RES_NORM)
        {
            _code_res_norm.clear();
            _phase_res_norm.clear();
        }
    }

    // add postfit residuals
    //------------------
    void gnss_data_sats::addwind(const double &wind)
    {
        _wind = wind;
    }

    // get stored wind up
    //------------------
    const double &gnss_data_sats::wind() const
    {
        return _wind;
    }

    // get hydrostatic mapPing factor
    //------------------
    const double &gnss_data_sats::mfH() const
    {

        return _mfH;
    }

    // get wet mapPing factor
    //------------------
    const double &gnss_data_sats::mfW() const
    {

        return _mfW;
    }

    // get tropo gradient mapPing factor
    //------------------
    const double &gnss_data_sats::mfG() const
    {

        return _mfG;
    }

    // Sun elevation relative to orbital plane
    double gnss_data_sats::_b()
    {

        // test if already calculated
        if (!double_eq(_beta_val, 999))
            return _beta_val;

        if (_satcrd.isZero())
            return 999;

        double beta = 0.0;
        double dt = 300;

        double dmjd = _epoch.dmjd();
        gnss_model_ephplan eph;
        Triple Sun = eph.sunPos(dmjd, false); //ICRF
        double gmt = eph.gmst(dmjd);
        double gmt_dt = eph.gmst(dmjd + dt / 86400.0);

        Triple Satcrd = _satcrd;
        Triple Satvel = _satvel;
        Triple Satcrd_dt = Satcrd + Satvel * dt; // 300s for extrapolation

        // prec. and nut. matrix should not change significantly in dt
        gnss_model_eop80 eop;
        Matrix prec = eop.precMatrix(dmjd);
        Matrix nut = eop.nutMatrix(dmjd);

        // ITRF -> ICRF
        Satcrd = prec.inverse() * nut.inverse() * rotZ(-gmt) * Satcrd;
        Satcrd_dt = prec.inverse() * nut.inverse() * rotZ(-gmt_dt) * Satcrd_dt;

        Triple n = Satcrd.cross(Satcrd_dt);

        n /= n.norm();

        Triple nSun = Sun / Sun.norm();

        double cosa = nSun.dot(n);
        //   if(cosa < 0) cosa *= -1;

        beta = hwa_pi / 2.0 - acos(cosa);

        _beta_val = beta;

#ifdef DEBUG
        std::cout << "Beta angle " << sat() << " " << _epoch.str_hms() << std::endl
             << " Sat pos: " << _satcrd
             << " Sat vel: " << _satvel
             << " Beta: " << beta * R2D << std::endl;
#endif

        return beta;
    }

    // Orbit angle
    double gnss_data_sats::_orb_angle()
    {

        // test if already calculated
        if (!double_eq(_orb_angle_val, 999))
            return _orb_angle_val;

        if (_satcrd.isZero())
            return 999;

        double mi = 0.0;
        double dt = 30;

        double dmjd = _epoch.dmjd();
        gnss_model_ephplan eph;
        Triple Sun = eph.sunPos(dmjd, false); //ICRF
        double gmt = eph.gmst(dmjd);
        double gmt_dt = eph.gmst(dmjd + dt / 86400.0);

        Triple Satcrd = _satcrd;
        Triple Satvel = _satvel;
        Triple Satcrd_dt = Satcrd + Satvel * dt; // 30s for extrapolation

        // prec. and nut. matrix should not change significantly in dt
        gnss_model_eop80 eop;
        Matrix prec = eop.precMatrix(dmjd);
        Matrix nut = eop.nutMatrix(dmjd);

        // ITRF -> ICRF
        Satcrd = prec.inverse() * nut.inverse() * rotZ(-gmt) * Satcrd;
        Satcrd_dt = prec.inverse() * nut.inverse() * rotZ(-gmt_dt) * Satcrd_dt;

        Triple n = Satcrd.cross(Satcrd_dt);

        Triple es = Satcrd / Satcrd.norm();

        Triple eSun = Sun / Sun.norm();

        Triple p = Sun.cross(n);
        p /= p.norm();

        double E = acos(es.dot(p));
        double SunSat = acos(es.dot(eSun));

        if (SunSat > hwa_pi / 2)
        {
            if (E <= hwa_pi / 2)
                mi = hwa_pi / 2 - E;
            else
                mi = hwa_pi / 2 - E;
        }
        else
        {
            if (E <= hwa_pi / 2)
                mi = hwa_pi / 2 + E;
            else
                mi = E - hwa_pi - hwa_pi / 2;
        }

        _orb_angle_val = mi;

#ifdef DEBUG
        std::cout << "Orbit angle " << sat() << " " << _epoch.sod() << " " //str_hms() << " "
             << " Sat pos: " << _satcrd
             << " Sat vel: " << _satvel
             << " Angle: " << mi * R2D << std::endl;
#endif

        return mi;
    }

} // namespace
