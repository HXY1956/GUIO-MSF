
/**
*
* @file        gmodel.h
* @brief    Purpose: base abstract class for models
*.
*/

#ifndef hwa_gnss_model_H
#define hwa_gnss_model_H

#include <string>
#include <map>
#include <cmath>

#include "hwa_base_allpar.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_set_base.h"
#include "hwa_gnss_model_tropo.h"
#include "hwa_gnss_all_Bias.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model. */
    class gnss_model
    {
    public:
        gnss_model();
        gnss_model(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_model();

        /** @brief std::set site. */
        void setSite(const std::string &site);

        /** @brief std::set log. */
        void spdlog(base_log spdlog);

        /** @brief model computed range value (phase/code). */
        virtual double cmpObs(base_time &epoch, base_allpar &param, gnss_data_sats &, gnss_data_obs &gobs, bool = false) = 0;

        /** @brief model computed D range value (phase/code). */
        virtual double cmpObsD(base_time &epoch, base_allpar &param, gnss_data_sats &gsatdata, gnss_data_obs &gobs) = 0;

        /** @brief get WindUp. */
        virtual double windUp(base_time &epoch, const std::string, const Vector &, const Vector &) { return 0.0; };

        /** @brief get trop Delay. */
        virtual double tropoDelay(base_time &epoch, base_allpar &param, Triple ell, gnss_data_sats &satdata) { return 0.0; };

        /** @brief get ZHD. */
        virtual double getZHD(const std::string &site, const base_time &epo) { return 0.0; };

        /** @brief get ZWD. */
        virtual double getZWD(const std::string &site, const base_time &epo) { return 0.0; };

        /** @brief reset observation. */
        virtual void reset_observ(OBSCOMBIN obs){};

        /** @brief reset trop mf. */
        virtual void reset_tropmf(ZTDMPFUNC mf){}; //lvhb added for npp, 20200918

        /** @brief reset sat PCO. */
        virtual void reset_SatPCO(bool cal = true){}; //lvhb added for npp, 20200918

        /** @brief Outliers detection. */
        virtual int outlierDetect(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &, const Vector &) = 0;
        virtual int outlierDetect(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &, std::vector<gnss_data_sats>::iterator &) = 0;
        virtual int outlierDetect(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &) = 0;
        virtual int outlierDetect_chi(std::vector<gnss_data_sats> &data, Symmetric &Qx, const Symmetric &, const Vector &) = 0;

        /** @brief get tropoModel. */
        std::shared_ptr<gnss_model_tropo> tropoModel() { return _tropoModel; }

        /** @brief std::set allbias. */
        void setBIAS(gnss_all_bias *bia) { _gallbias = bia; }

    protected:
        std::shared_ptr<gnss_model_tropo> _tropoModel; ///< trop model
        set_base *_settings;            ///< setting
        std::string _site;                     ///< site
        base_log _spdlog;                 /// spdlog ptr
        bool _phase;                      ///< phase
        gnss_all_bias *_gallbias;            ///< allbias
    };

}

#endif //  GMODEL_H
