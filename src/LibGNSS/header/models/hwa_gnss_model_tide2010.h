
/**
*.
* @file        gtide2010.h
* @brief    Purpose: implements tides
*/

#ifndef hwa_gnss_model_tide2010_H
#define hwa_gnss_model_tide2010_H

#include <vector>

#ifdef BMUTEX
#include <boost/thread/mutex.hpp>
#endif

#include "hwa_base_const.h"
#include "hwa_base_time.h"
#include "hwa_gnss_model_ephplan.h"
#include "hwa_gnss_model_gpt.h"
#include "hwa_gnss_model_tide.h"

namespace hwa_gnss
{

    /** @brief class for gnss_model_tide2010 derive from gnss_model_tide. */
    class gnss_model_tide2010 : public gnss_model_tide
    {

    public:
        /** @brief constructor 1. */
        gnss_model_tide2010(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_model_tide2010();

        /** @brief tide search. */
        virtual Triple tide_earth(const base_time &epo, Triple &xyz, Triple xsun, Triple xmoon);               // solid base_earth tides
        /*virtual Triple tide_pole(const base_time& epo, const Triple& xRec, double xp, double yp, modeofmeanpole mode); */ // pole tides
        //virtual Triple load_ocean_vlbi(const base_time& epoch, const string& site, const Triple& xRec);
        virtual Triple load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec); // ocean tide loading
        //virtual Triple load_atmosph(const base_time& epoch, const string& site, const Triple& xRec);   // atmosphere tide loading
        //virtual Triple load_notidatmosph(const base_time& epoch, const string& site, const Triple& xRec);  // Non tidal atmosphere loading
        virtual Triple load_atmosregrecoef(const Triple &xRec, double press);
        //virtual Triple tide_oceanpole(const base_time& epo, const Triple& xRec, double xp, double yp, modeofmeanpole mode);
        //double splineinterp(vector<double> x, vector<double> y, double xq);
    protected:
        // methods for ocean tides
        //int _interpolate(const base_time& epo, const Matrix& otl_in, const Matrix& doods, Matrix& otl_out);
        //int _tidefreq(const base_time& epo, const Vector& idd, double& freq, double& phase);
        base_time _epo_save; ///< epo save
        Vector _D;   ///< D
        Vector _DD;  ///< DD
        gnss_model_gpt _ggpt;       ///< gpt
    };

} // namespace

#endif
