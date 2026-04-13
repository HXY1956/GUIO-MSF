#include "hwa_gnss_proc_ppp.h"
#include "hwa_base_const.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_model_ppp.h"

using namespace hwa_base;

namespace hwa_gnss
{

    // constructor
    // -------------
    gnss_proc_ppp::gnss_proc_ppp(std::string mark, set_base *set)
        : gnss_proc_spp(mark, set),
          _gotl(0),
          _running(false)
    {
        _tides = new gnss_model_tide2010(_spdlog);
        _gModel = new gnss_model_ppp(_site, _spdlog, _set);

        _get_settings();
    }
    gnss_proc_ppp::gnss_proc_ppp(std::string mark, set_base *set, base_log spdlog)
        : gnss_proc_spp(mark, set, spdlog),
          _gotl(0),
          _running(false)
    {
        _tides = new gnss_model_tide2010(_spdlog);
        _gModel = new gnss_model_ppp(_site, _spdlog, _set);

        _get_settings();
    }

    // destructor
    // ----------
    gnss_proc_ppp::~gnss_proc_ppp()
    {
        if (_tides)
            delete _tides;
    }

    // Set OTL
    // --------------------------------
    void gnss_proc_ppp::setOTL(gnss_all_otl *gotl)
    {
        _gotl = gotl;
        if (_tides)
            _tides->setOTL(_gotl);
    }

    // Set OBJ
    // --------------------------------
    void gnss_proc_ppp::setOBJ(gnss_all_obj *gallobj)
    {
        gnss_proc_spp::setOBJ(gallobj);

        dynamic_cast<gnss_model_ppp *>(_gModel)->setOBJ(gallobj);

        /* if(_gallobj){ */
        /*    _grec = _gallobj->obj(_site); */
        /* } */
    }

    // Test if running
    // -----------------------------
    bool gnss_proc_ppp::isRunning()
    {
        //  boost::mutex::scoped_lock lock(_mutex);   // not if processBatch
        return _running;
    }

    // Get settings
    // -----------------------------
    int gnss_proc_ppp::_get_settings()
    {
        gnss_proc_spp::_get_settings();

        _phase = dynamic_cast<set_gproc *>(_set)->phase();
        _doppler = dynamic_cast<set_gproc *>(_set)->doppler();
        _sigAmbig = dynamic_cast<set_gproc *>(_set)->sig_init_amb();

        //_setLog(); // set log  (before using it later on!)

        return 1;
    }

    // Print results
    // -----------------------------
    void gnss_proc_ppp::_prt_results(base_iof *giof, const gnss_all_rslt &rslt)
    {
        if (!giof)
            return;
        std::ostringstream os;

        // TOTO JE Z Glsq.CPP
        //  set<gnss_all_rslt::result>::iterator it_trp;
        //  for(it_trp=rslt.set_trp.begin();it_trp!=rslt.set_trp.end();it_trp++)
        //  {
        //    std::cout <<"TRP "<< it_trp->beg.str(" now: %Y-%m-%d %H:%M:%S[%T] ")
        //       << " "<< it_trp->val << std::endl;
        //  }

        std::vector<gnss_all_rslt::result>::const_iterator it;
        double ztd = 0.0;
        double ztdrms = 0.0;
        Triple xyz(0.0, 0.0, 0.0);
        Triple xyz_ref(0.0, 0.0, 0.0);
        Triple neu(0.0, 0.0, 0.0);
        Triple rms(0.0, 0.0, 0.0);

        // reference crd for xyz2neu
        it = rslt.v_rslt.begin();
        if (it->type == "CRD_X")
        {
            xyz_ref[0] = it->val;
        }
        if (it->type == "CRD_Y")
        {
            xyz_ref[1] = it->val;
        }
        if (it->type == "CRD_Z")
        {
            xyz_ref[2] = it->val;
        }

        for (it = rslt.v_rslt.begin(); it != rslt.v_rslt.end(); ++it)
        {

            //  PRINT ALL PAR-BY-PAR!
            //     std::cout << it->type << it->beg.str(" %Y-%m-%d %H:%M:%S[%T] ")
            //                      << it->end.str(" %Y-%m-%d %H:%M:%S[%T] ")
            //                      << " " << it->val
            //                      << " " << it->rms
            //                      << std::endl;

            // COLLECT COORDINATES
            if (it->type == "CRD_X")
            {
                xyz[0] = it->val;
                rms[0] = it->rms;
            }
            if (it->type == "CRD_Y")
            {
                xyz[1] = it->val;
                rms[1] = it->rms;
            }
            if (it->type == "CRD_Z")
            {
                xyz[2] = it->val;
                rms[2] = it->rms;
            }

            // TROPO-DRIVEN OUTPUT (SUCH OUTPUT IS TEMPORARY SOLUTION!)
            if (it->type == "TRP" && xyz[0] != 0.0)
            {
                ztd = it->val;
                ztdrms = it->rms;
                base_time tt = it->beg + (it->end - it->beg);

                Triple xyz_rho = xyz - xyz_ref;
                Triple ell_ref;
                xyz2ell(xyz_ref, ell_ref, false);

                xyz2neu(ell_ref, xyz_rho, neu);

                os << std::fixed << std::setprecision(3)
                   << tt.str_ymdhms()
                   << " " << std::setw(13) << xyz[0]
                   << " " << std::setw(13) << xyz[1]
                   << " " << std::setw(13) << xyz[2]
                   << " " << std::setw(7) << ztd
                   << " " << std::setw(13) << neu[0]
                   << " " << std::setw(13) << neu[1]
                   << " " << std::setw(13) << neu[2]
                   << std::fixed << std::setprecision(5)
                   << " " << std::setw(9) << rms[0]
                   << " " << std::setw(9) << rms[1]
                   << " " << std::setw(9) << rms[2]
                   << " " << std::setw(7) << ztdrms
                   << " " << std::setw(2) << 0 // DATA SIZE NOT KNOWN HERE !!!!!
                   << std::endl;

                giof->write(os.str().c_str(), os.str().size()); // _flt->flush();

                os.clear();
                os.str("");
            }
        }
    }

} // namespace
