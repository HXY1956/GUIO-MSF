#include "hwa_gnss_proc_Spp.h"
#include "hwa_gnss_model_spp.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{
    // Constructor
    gnss_proc_spp::gnss_proc_spp(std::string mark, hwa_set::set_base *set)
        : _grec(nullobj),
          _gallobj(0),
          _gallbias(0),
          _weight(OBSWEIGHT::SINEL),
          _observ(OBSCOMBIN::IONO_FREE),
          _valid_crd_xml(false),
          _valid_ztd_xml(false),
          _site(mark),
          _set(set),
          _res(0),
          _gobs(0),
          _gnav(0),
          _gmet(0),
          _gion(0),
          _allprod(0),
          _phase(false),
          _doppler(false),
          _gnss(GNS),
          _initialized(false),
          _use_ecl(false)
    {
        if (nullptr == set)
        {
            std::cerr << "your set pointer is nullptr !"<<std::endl;
            throw std::logic_error("");
        }
        else
        {
            _set = set;
        }

        _crd_begStat = _crd_endStat = FIRST_TIME;
        _ztd_begStat = _ztd_endStat = FIRST_TIME;

        _valid_ztd_xml = false;

        _get_settings();

        _gModel = new gnss_model_spp(_site, _set);
    }
    gnss_proc_spp::gnss_proc_spp(string mark, set_base* set, base_log spdlog, string mode)
        : _grec(nullobj),
        _gallobj(0),
        _gallbias(0),
        _weight(OBSWEIGHT::SINEL),
        _observ(OBSCOMBIN::IONO_FREE),
        _valid_crd_xml(false),
        _valid_ztd_xml(false),
        _site(mark),
        _set(set),
        _res(0),
        _gobs(0),
        _gnav(0),
        _gmet(0),
        _gion(0),
        _allprod(0),
        _phase(false),
        _doppler(false),
        _gnss(GNS),
        _initialized(false),
        _use_ecl(false)
    {
        // set the setting pointer
        _setLog(mode);       // set log  (before using it later on!)
        if (nullptr == set)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw logic_error("");
        }
        else
        {
            _set = set;
        }
        _crd_begStat = _crd_endStat = FIRST_TIME;
        _ztd_begStat = _ztd_endStat = FIRST_TIME;
        _valid_ztd_xml = false;
        _get_settings();
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw logic_error("");
        }
        else
        {
            _gspdlog = spdlog;
        }
        _gModel = new gnss_model_spp(_gspdlog, _site, _set);
    }

    // Destructor
    gnss_proc_spp::~gnss_proc_spp()
    {
        if (_gModel)
            delete _gModel;
    }

    // Set obs, nav
    // --------------------------------
    void gnss_proc_spp::setDAT(gnss_all_obs *gobs, gnss_all_nav *gnav)
    {
        _gobs = gobs;
        _gnav = gnav;

        //this->fixObsTypes();
    }

    // set output for products
    // ----------------------------
    void gnss_proc_spp::setOUT(gnss_all_prod *products)
    {
        this->_setOut();
        _allprod = products;
    }

    // Set OBJ
    // --------------------------------
    void gnss_proc_spp::setOBJ(gnss_all_obj *gallobj)
    {
        _gallobj = gallobj;

        if (_gallobj)
        {
            _grec = _gallobj->obj(_site);
            dynamic_cast<gnss_model_spp *>(_gModel)->setrec(_grec);
        }
    }

    // set DCB products
    // ----------------------------
    void gnss_proc_spp::setDCB(gnss_all_bias *bias)
    {
        _gallbias = bias;
        _gModel->setBIAS(bias);
    }

    // set FCB products
    // ----------------------------
    void gnss_proc_spp::setFCB(gnss_all_bias *bias)
    {
        _gallfcb = bias;
    }

    void gnss_proc_spp::spdlog(base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }
    // Tropo is used or not
    // -------------------------
    void gnss_proc_spp::tropo(bool tropo)
    {
        this->_tropo_est = tropo;
    }

    // Tropo slants are provided
    // -------------------------
    void gnss_proc_spp::tropo_slant(bool slant)
    {
        this->_tropo_slant = slant;
    }

    // Tropo is used or not
    // -------------------------
    void gnss_proc_spp::phase(bool phase)
    {
        this->_phase = phase;
    }

    // set used GNSS
    // --------------------------
    void gnss_proc_spp::setgnss(GSYS sys)
    {
        this->_success = true;
        this->_gnss = sys;
    }

    // set std::map of signals used in processing
    void gnss_proc_spp::fixObsTypes()
    {
        // find out frequency of individual signals from gallnav
        gnss_all_obs::hwa_map_freq mfrq = _gobs->frqobs(_site);

        // find out most frequent signals
        for (std::map<std::string, std::map<GOBSBAND, std::map<GOBS, int>>>::iterator itPRN = mfrq.begin(); itPRN != mfrq.end(); itPRN++)
        {
            std::string prn = itPRN->first;

            for (std::map<GOBSBAND, std::map<GOBS, int>>::iterator itBAND = itPRN->second.begin(); itBAND != itPRN->second.end(); itBAND++)
            {
                GOBSBAND band = itBAND->first;

                int max = 0;
                for (std::map<GOBS, int>::iterator itGOBS = itBAND->second.begin(); itGOBS != itBAND->second.end(); itGOBS++)
                {
                    std::string str = gobs2str(itGOBS->first);
                    if (str.compare(0, 1, "C") != 0 && str.compare(0, 1, "L") != 0)
                        continue;
                    if (itGOBS->second > max)
                    {
                        max = itGOBS->second;
                        _signals[prn][band] = str2gobsattr(str);
                        //          std::cout << "Fixed Types: " << prn << " " << str << std::endl;
                    }
                }
            }
        }
    }

    // Get settings
    // -----------------------------
    int gnss_proc_spp::_get_settings()
    {
        //   _sats          = dynamic_cast<set_gnss*>(_set)->sat();
        _tropo_est = dynamic_cast<set_gproc *>(_set)->tropo();
        _iono_est = dynamic_cast<set_gproc *>(_set)->iono();
        _tropo_grad = dynamic_cast<set_gproc *>(_set)->tropo_grad();
        _tropo_slant = dynamic_cast<set_gproc *>(_set)->tropo_slant();
        _ztd_mf = dynamic_cast<set_gproc *>(_set)->tropo_mf();
        _grd_mf = dynamic_cast<set_gproc *>(_set)->grad_mf();
        _crd_est = dynamic_cast<set_gproc *>(_set)->crd_est();
        _sampling = dynamic_cast<set_gen *>(_set)->sampling();
        _scale = dynamic_cast<set_gen *>(_set)->sampling_scalefc();
        _minElev = dynamic_cast<set_gproc *>(_set)->minimum_elev();
        _sig_init_crd = dynamic_cast<set_gproc *>(_set)->sig_init_crd();
        _sig_init_vel = dynamic_cast<set_gproc *>(_set)->sig_init_vel();
        _sig_init_ztd = dynamic_cast<set_gproc *>(_set)->sig_init_ztd();
        _sig_init_vion = dynamic_cast<set_gproc *>(_set)->sig_init_vion();
        _sig_init_grd = dynamic_cast<set_gproc *>(_set)->sig_init_grd();
        _sig_init_glo = dynamic_cast<set_gproc *>(_set)->sig_init_glo();
        _sig_init_gal = dynamic_cast<set_gproc *>(_set)->sig_init_gal();
        _sig_init_bds = dynamic_cast<set_gproc *>(_set)->sig_init_bds();
        _sig_init_qzs = dynamic_cast<set_gproc *>(_set)->sig_init_qzs();

        _sigCodeGPS = dynamic_cast<set_gnss *>(_set)->sigma_C(GPS);
        _sigCodeGLO = dynamic_cast<set_gnss *>(_set)->sigma_C(GLO);
        _sigCodeGAL = dynamic_cast<set_gnss *>(_set)->sigma_C(GAL);
        _sigCodeBDS = dynamic_cast<set_gnss *>(_set)->sigma_C(BDS);
        _sigCodeQZS = dynamic_cast<set_gnss *>(_set)->sigma_C(QZS);
        _sigPhaseGPS = dynamic_cast<set_gnss *>(_set)->sigma_L(GPS);
        _sigPhaseGLO = dynamic_cast<set_gnss *>(_set)->sigma_L(GLO);
        _sigPhaseGAL = dynamic_cast<set_gnss *>(_set)->sigma_L(GAL);
        _sigPhaseBDS = dynamic_cast<set_gnss *>(_set)->sigma_L(BDS);
        _sigPhaseQZS = dynamic_cast<set_gnss *>(_set)->sigma_L(QZS);
        _sigDopplerGPS = dynamic_cast<set_gnss *>(_set)->sigma_D(GPS);
        _sigDopplerGLO = dynamic_cast<set_gnss *>(_set)->sigma_D(GLO);
        _sigDopplerGAL = dynamic_cast<set_gnss *>(_set)->sigma_D(GAL);
        _sigDopplerBDS = dynamic_cast<set_gnss *>(_set)->sigma_D(BDS);
        _sigDopplerQZS = dynamic_cast<set_gnss *>(_set)->sigma_D(QZS);

        // no implement
        //_sigDopplerGPS = dynamic_cast<set_gnss*>(_set)->sigma_D(GPS);
        //_sigDopplerGLO = dynamic_cast<set_gnss*>(_set)->sigma_D(GLO);
        //_sigDopplerGAL = dynamic_cast<set_gnss*>(_set)->sigma_D(GAL);
        //_sigDopplerBDS = dynamic_cast<set_gnss*>(_set)->sigma_D(BDS);
        //_sigDopplerQZS = dynamic_cast<set_gnss*>(_set)->sigma_D(QZS);

        _pos_kin = dynamic_cast<set_gproc *>(_set)->pos_kin();
        _weight = dynamic_cast<set_gproc *>(_set)->weighting();
        _observ = dynamic_cast<set_gproc *>(_set)->obs_combin();
        _use_ecl = dynamic_cast<set_gproc *>(_set)->use_eclipsed();

        if (_sampling == 0)
            _sampling = dynamic_cast<set_gen *>(_set)->sampling_default();

        return 1;
    }

    // Set Out
    // -----------------------------
    void gnss_proc_spp::_setOut()
    {
        std::string tmp(dynamic_cast<set_out *>(_set)->outputs("res"));
        if (!tmp.empty())
        {
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            _greslog.set_log("BASIC", spdlog::level::info, tmp);
            _res = _greslog.spdlog();
        }
    }

    // Set log
    // -----------------------------
    void gnss_proc_spp::_setLog(std::string mode)
    {
        std::string tmp(dynamic_cast<set_out *>(_set)->outputs("ppp"));
        if (!tmp.empty())
        {
            tmp = tmp.substr(7, tmp.size() - 7);
            base_type_conv::substitute(tmp, "$(rec)", _site, false);

            base_time beg = dynamic_cast<set_gen *>(_set)->beg();
            if (beg == FIRST_TIME)
            {
                beg = base_time::current_time(base_time::GPS);
            } // real-time model
            
            base_type_conv::substitute(tmp, "$(rec)", _site, false);
            base_type_conv::substitute(tmp, "$(doy)", base_type_conv::int2str(beg.doy()), false);
            base_type_conv::substitute(tmp, "$(date)", base_type_conv::int2str(beg.year()) + base_type_conv::int2str(beg.doy()), false); // add for date
            if (mode != "")
                tmp = tmp + "-" + mode;
            /*int n = 1;
            bool loop = true;
            while (loop)
            {
                std::string tmp1 = tmp + to_string(n);
                auto alog = spdlog::get(tmp);       // get no useful
                if(alog)
                {
                    n = n + 1;
                }
                else
                {
                    tmp = tmp1;
                    loop = false;
                }
            }*/
            auto log_type = dynamic_cast<set_out*>(_set)->log_type();
            auto log_level = dynamic_cast<set_out*>(_set)->log_level();
            auto log_name = dynamic_cast<set_out*>(_set)->log_name();
            _grtlog.set_log(log_type, log_level, log_name);
            _spdlog = _grtlog.spdlog();
        }
    }

} // namespace
