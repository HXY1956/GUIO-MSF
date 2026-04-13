#ifndef hwa_gnss_proc_spp_h
#define hwa_gnss_proc_spp_h

#include "hwa_set_base.h"
#include "hwa_set_out.h"
#include "hwa_set_proc.h"
#include "hwa_base_time.h"
#include "hwa_base_mutex.h"
#include "hwa_base_log.h"
#include "hwa_gnss_prod_crd.h"
#include "hwa_set_gen.h"
#include "hwa_base_iof.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_gnss_all_nav.h"
#include "hwa_gnss_all_prod.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_all_bias.h"
#include "hwa_gnss_model_bancroft.h"
#include "hwa_gnss_model.h"
#include "hwa_base_par.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /** @brief class for gnss_proc_spp. */
    class gnss_proc_spp
    {
    public:
        /** @brief constructor 1. */
        gnss_proc_spp(std::string mark, set_base *set);

        gnss_proc_spp(std::string mark, set_base* set, hwa_base::base_log spdlog, std::string mode = "");

        /** @brief default destructor. */
        virtual ~gnss_proc_spp();

        /** @brief for process. */
        virtual int processBatch(const hwa_base::base_time &beg, const hwa_base::base_time &end) = 0;

        /** @brief set for data, output product, object(like pcv model),bias and so on. */
        virtual void setDAT(gnss_all_obs *gobs, gnss_all_nav *gnav);

        /** @brief set output for products. */
        void setOUT(gnss_all_prod *products);

        /** @brief Set general log file. */
        virtual void spdlog(hwa_base::base_log spdlog);
        /** @brief set OBJ/DCB/FCB. */
        virtual void setOBJ(gnss_all_obj *gobj);
        virtual void setDCB(gnss_all_bias *gbias);
        virtual void setFCB(gnss_all_bias *gbias);

        /** 
        *@brief Setting up types of error correction.
        * Tropo is used or not
        */
        void tropo(bool tropo);

        /** @brief Tropo slants are provided. */
        void tropo_slant(bool slant);

        /** @brief phase. */
        void phase(bool phase);

        /** @brief set gnss. */
        void setgnss(GSYS sys);

        /** @brief get site. */
        std::string site() { return _site; }

        /** @brief set std::map of signals used in processing. */
        void fixObsTypes();

        double sampling() { return _sampling; }

        std::shared_ptr<gnss_data_obj> _grec; ///< Transmitter/receiver object.
        gnss_all_obj* _gallobj;      ///< Objects transmitter/receiver.
        gnss_all_bias* _gallbias;    ///< Differential code biases.
        gnss_all_bias* _gallfcb;     ///< Phase base_type_conv::fractional cycle bias.

    protected:
        CONSTRPAR _crd_est;
        hwa_base::base_time _crd_begStat;
        hwa_base::base_time _crd_endStat;
        hwa_base::base_time _ztd_begStat;
        hwa_base::base_time _ztd_endStat; ///< constraining parameters

        // a priory parameters
        double _aprox_ztd_xml; ///< Approximate ztd.
        OBSWEIGHT _weight;     ///< observation weighting
        OBSCOMBIN _observ;     ///< observation type model

        /** @brief Get settings from XML file and set local variables. */
        virtual int _get_settings();

        /** @brief Set output file (empty). */
        virtual void _setOut();

        /** @brief Set processing log file (<ppp>). */
        virtual void _setLog(std::string mode = "");
        bool _valid_crd_xml;
        bool _valid_ztd_xml;
        gnss_model *_gModel;    ///< models.
        std::string _site;         ///< Site internal ID.
        set_base *_set;     ///< Base setting.
        base_log _spdlog;     ///< Processing spdlog output.
        base_log _gspdlog;    ///< Genereal spdlog output.
        base_log _res;        ///< Genereal spdlog output.
        base_rtlog _grtlog;     ///< great log
        base_rtlog _greslog;    ///< Residuals output.
        gnss_all_obs *_gobs;     ///< Observation data.
        gnss_all_nav *_gnav;     ///< Objects for ephemerides.
        gnss_all_prod *_gmet;    ///< Troposphere products input
        gnss_all_prod *_gion;    ///< Ionosphere products input
        gnss_all_prod *_allprod; ///< Products output.
        bool _phase;          ///< Phase is used.
        bool _doppler;        ///< Doppler is used.
        bool _tropo_est;      ///< Tropo is estimated.
        bool _iono_est;       ///< Iono is estimated.
        bool _tropo_grad;     ///< Tropo horizontal gradients.
        bool _tropo_slant;    ///< Tropo slants are produced.
        GSYS _gnss;           ///< GNSS system to be used.
        ZTDMPFUNC _ztd_mf;    ///< ZTD mapPing function.
        GRDMPFUNC _grd_mf;    ///< GRD mapPing function.
        double _minElev;      ///< Elevation cut-off.
        double _sampling;     ///< Sampling interval.
        double _scale;        ///< Sampling scaling factor.
        bool _initialized;    ///< Initialized status.
        //set<std::string>   _sats;           /**< Configured satellites to be used. */
        int _nSat;      ///< Number of satellites comming into the processing
        int _nSat_excl; ///< Number of excluded satellites due to various reason

        // init sigma
        double _sig_init_crd;  ///< Initial coordinates sigma.
        double _sig_init_vel;  ///< Initial velocities sigma.
        double _sig_init_ztd;  ///< Initial ZTD sigma.
        double _sig_init_vion; ///< Initial VION sigma.
        double _sig_init_grd;  ///< Initial GRD  sigma.

        // ISB sigma
        double _sig_init_glo; ///< Initial GLONASS ISB sigma.
        double _sig_init_gal; ///< Initial Galileo ISB sigma.
        double _sig_init_bds; ///< Initial BeiDou ISB sigma.
        double _sig_init_qzs; ///< Initial QZSS ISB sigma.

        // observations sigma
        double _sigCodeGPS;    ///< Code sigma.  - GPS
        double _sigPhaseGPS;   ///< Phase sigma. - GPS
        double _sigDopplerGPS; ///< Doppler sigma. - GPS
        double _sigCodeGLO;    ///< Code sigma.  - GLONASS
        double _sigPhaseGLO;   ///< Phase sigma. - GLONASS
        double _sigDopplerGLO; ///< Doppler sigma. - GLONASS
        double _sigCodeGAL;    ///< Code sigma.  - Galileo
        double _sigPhaseGAL;   ///< Phase sigma. - Galileo
        double _sigDopplerGAL; ///< Doppler sigma. - Galileo
        double _sigCodeBDS;    ///< Code sigma.  - BeiDou
        double _sigPhaseBDS;   ///< Phase sigma. - BeiDou
        double _sigDopplerBDS; ///< Doppler sigma. - BeiDou
        double _sigCodeQZS;    ///< Code sigma.  - QZSS
        double _sigPhaseQZS;   ///< Phase sigma. - QZSS
        double _sigDopplerQZS; ///< Doppler sigma. - QZSS

        bool _pos_kin;    ///< pos kin
        bool _extern_log; ///< extern log

        bool _use_ecl; ///< use ecl
        bool _success; ///< success

#ifdef BMUTEX
        boost::mutex _mutex;
#endif
        hwa_base::base_mutex _gmutex; ///< gmutex

        // signals used for processing
        std::map<std::string, std::map<GOBSBAND, GOBSATTR>> _signals; ///< signal
    };
} // namespace

#endif
