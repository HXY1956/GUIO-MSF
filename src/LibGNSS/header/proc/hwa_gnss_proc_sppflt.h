#ifndef hwa_gnss_proc_sppflt_h
#define hwa_gnss_proc_sppflt_h

#include "hwa_gnss_proc_spp.h"
#include "hwa_gnss_model_dop.h"
#include "hwa_gnss_proc_flt.h"
#include "hwa_gnss_all_bias.h"
#include "hwa_gnss_all_prod.h"
#include "hwa_gnss_model_stochastic.h"
#include "hwa_set_npp.h"
#include "hwa_set_gflt.h"
#include "hwa_base_allpar.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{

#define SPP_MINSAT (6) ///< minimum number of satellites

    /** @brief class for gnss_proc_sppflt derive from gnss_proc_spp. */
    class gnss_proc_sppflt : public virtual gnss_proc_spp
    {
    public:
        /** @brief constructor 1. */
        gnss_proc_sppflt(std::string mark, set_base *set);

        gnss_proc_sppflt(std::string mark, set_base *set, base_log spdlog, std::string mode="");
        /** @brief default destructor. */
        virtual ~gnss_proc_sppflt();

        /** @brief Process observation batch. */
        virtual int processBatch(const base_time &beg, const base_time &end);

        /** @brief min sat. */
        void minsat(size_t minsat) { _minsat = (minsat < 5) ? 5 : minsat; }

        /** @brief get crd. */
        Triple getCrd(const base_time &time); //add by glfeng

        /** @brief get outlier sat. */
        std::vector<std::string> get_outlier_sat() { return _outlier_sat; } //added by xiongyun

        Symmetric& get_Qx() { return _Qx;}

        const Matrix& _get_Pk() { return _Qx.matrixR(); }

    protected:
        /** @brief Predict state std::vector and covariance matrix. */
        virtual void _predict();

        /** @brief Restore state and covariance matrix. */
        virtual void _restore(const Symmetric &Qsav, const base_allpar &Xsav);

        /** @brief Satelite position. */
        virtual int _satPos(base_time &, gnss_data_sats &);

        /** @brief Prepare data: filter, bancroft, members in gsatdata. */
        virtual int _prepareData();

        /** @brief Tides are not applied in SPP. */
        virtual int _apply_tides(base_time &_epoch, Triple &xRec);

        /** @brief Process one epoch. */
        virtual int _processEpoch(const base_time &runEpoch);

        /** @brief add data to A, l, P - code. */
        int _addObsP(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P);

        /** @brief add data to A, l, P - carrier phase. */
        virtual int _addObsL(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P);

        /** @brief add pseudo observations as a constrains. */
        virtual int _addPseudoZTD(unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P);

        /** @brief weight coef for observations. */
        double _weightObs(gnss_data_sats &satdata, gnss_data_obs &go);

        /** @brief Update time for RDW processes. */
        virtual void _timeUpdate(const base_time &epo);

        /** @brief Sync inter-GNSS systems bias. */
        void _syncSys();

        /** @brief Add/Remove ionosphere delay. */
        void _syncIono();

        /** @brief Add/Remove inter-freq. biases. */
        void _syncIFB();

        // Add/Remove pseudorange bias for PPP-RTK client
        void _syncRcb();

        /** @brief save observations residuals. */
        void _save_residuals(Vector &v, std::vector<gnss_data_sats> &satdata, RESIDTYPE restype);

        /** @brief Apply SICB correction. */
        double _applySICB(std::string prn, double elev, GOBSBAND freq);

        /** @brief Apply DCB correction. */
        int _applyDCB(gnss_data_sats &satdata, double &P, gnss_data_obs *gobs1, gnss_data_obs *gobs2 = 0);

        std::vector<gnss_data_sats> _data; ///< data
        std::map<std::string, int> _newAMB; ///< newAMB
        std::set<std::string> _slips;       ///< slips
        unsigned int _minsat;     ///< minsat
        double _sig_unit;         ///< sig unit
        int _frequency;           ///< frequency

        // Models
        gnss_model_random_walk *_trpStoModel; ///< trop stochastic model
        //gnss_model_white_noise*       _ionStoModel;
        gnss_model_random_walk *_ionStoModel; ///< iono stochastic model
        gnss_model_random_walk *_gpsStoModel; ///< gps stochastic model
        gnss_model_random_walk *_gloStoModel; ///< glo stochastic model
        gnss_model_random_walk *_galStoModel; ///< gal stochastic model
        gnss_model_random_walk *_bdsStoModel; ///< bds stochastic model
        gnss_model_random_walk *_qzsStoModel; ///< qzs stochastic model
        gnss_model_white_noise *_clkStoModel; ///< clk stochastic model
        gnss_model_white_noise *_crdStoModel; ///< crd stochastic model

        base_allpar _param;
        Symmetric _Qx;     ///< Parameters and covariance matrix
        Diag _Noise;   ///< Noise matrix
        base_time _epoch;          ///< Epoch time
        gnss_proc_flt *_filter;         ///< Estimation objects
        int _numSat(GSYS gsys);  ///< num sat
        Vector _vBanc;     ///< v
        gnss_model_dop _dop;             ///< dop
        int _cntrep;             ///< cntrep
        int _numcor;             ///< num cor
        bool _smooth;            ///< smooth
        unsigned int _n_NPD_flt; ///< NPD flt number
        unsigned int _n_all_flt; ///< NPD smt number
        unsigned int _n_NPD_smt; ///< NPD smt number
        unsigned int _n_all_smt; ///< ALL smt number

        RESIDTYPE _resid_type; ///< res type

        /** @brief get obs. */
        int _getgobs(std::string prn, GOBSTYPE type, GOBSBAND band, gnss_data_obs &gobs);
        std::map<std::string, int> _frqNum;      ///< frequency number
        std::map<std::string, base_time> _lastEcl; ///< last ecl

        bool _ifb3_init; ///< ifb3
        bool _ifb4_init; ///< ifb4
        bool _ifb5_init; ///< ifb5

        bool _auto_band;      ///< auto band
        CBIASCHAR _cbiaschar; ///< bias(char)

        // result  add by glfeng
        std::map<base_time, Triple> _map_crd;

        std::vector<std::string> _outlier_sat; //added by xiongyun

		std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index;
		std::map<GSYS, std::map<GOBSBAND, FREQ_SEQ>> _freq_index; // added by tyx
    };

} // namespace

#endif // GSPPFLT_H
