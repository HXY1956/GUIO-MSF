#ifndef hwa_gnss_proc_spplsq_H
#define hwa_gnss_proc_spplsq_H

#include "hwa_gnss_proc_spp.h"
#include "hwa_gnss_model_dop.h"
#include "hwa_gnss_proc_lsq.h"
#include "hwa_base_allpar.h"
#include "hwa_gnss_all_Bias.h"
#include "hwa_gnss_all_Prod.h"
#include "hwa_base_globaltrans.h"
#include "hwa_gnss_model_stochastic.h"
#include "hwa_gnss_model_spp.h"

#define SPP_MINSAT (6) // minimum number of satellites
namespace hwa_gnss
{
    /**
     * @brief class for spp
     */
    class gnss_proc_spplsq : public virtual gnss_proc_spp
    {
    public:
        // Constructor
        /**
         * @brief Construct a new t gspplsq object
         * @param[in]  mark      mark
         * @param[in]  std::set       std::setbase control
         * @param[in]  obj       object
         * @param[in]  dynamic   dynamic
         */
        gnss_proc_spplsq(std::string mark, set_base *set, gnss_all_obj *obj, base_log spdlog, bool dynamic = false);

        // Destructor
        /**
         * @brief Destroy the t gspplsq object
         */
        virtual ~gnss_proc_spplsq();

        //
        /**  Main function of processing
         * @brief 
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        virtual int processBatch(const base_time &beg, const base_time &end);
        /**
         * @brief process one epoch
         * @param[in]  now       time
         * @param[in]  vec_obs   satinfo
         * @return int 
         */
        int ProcessOneEpoch(const base_time &now, std::vector<gnss_data_sats> *vec_obs = NULL);
        /**
         * @brief Get the Rec Clk 
         * @param[in]  time      time
         * @return double station clk
         */
        double getRecClk(const base_time &time);
        /**
         * @brief get velocity
         * @param[in]  time      time
         * @return Triple velocity
         */
        Triple getvel(const base_time &time);
        /**
         * @brief get coordinate
         * @param[in]  time      time
         * @return Triple coordinate
         */
        Triple getCrd(const base_time &time);
        /**
         * @brief get average coordinate
         * @return Triple coordinate
         */
        Triple getAvergeCrd(); // for static site
        /**
         * @brief get satellite list
         * @param[in]  satdata   satinfo
         * @return std::set<std::string> satellite list
         */
        std::set<std::string> outcomsat(std::map<std::string, gnss_data_sats> &satdata);
        std::map<base_time, int> getobsnum();

    protected:
        std::map<base_time, Triple> _map_crd; ///< result

        gnss_proc_lsqbase *_glsq = nullptr; ///< Set Estimation objects
        /**
         * @brief get settings
         */
        int _get_own_settings();

        virtual void _setOut() override;
        /**
         * @brief Prepare for data
         * @return int 
         */
        //
        virtual int _prepareData();

        //
        /**
         * @brief Processing functions for each epoch
         * @param[in]  runEpoch  time
         */
        virtual int _processEpoch(const base_time &runEpoch);
        //
        /**
         * @brief Adding pseudorange observations to construct B, OMC and P Matrix
         * @param[in]  satdata   satinfo
         * @param[in]  iobs      index of obs
         * @param[in]  ell       ell
         * @param[in]  B         patrial
         * @param[in]  OMC       omc
         * @param[in]  P         weight
         * @return int 
         */
        virtual int _addObsP(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &B, Vector &OMC, Diag &P);
        //
        /**
         * @brief  Adding phase observations to construct B, OMC and P Matrix
         * @param[in]  satdata   satinfo
         * @param[in]  iobs      index of obs
         * @param[in]  ell       ell
         * @param[in]  B         patrial
         * @param[in]  OMC       omc
         * @param[in]  P         weight
         * @return int 
         */
        virtual int _addObsL(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &B, Vector &OMC, Diag &P);
        /**
         * @brief add doppler measurement to model
         * @param[in] satdata
         * @param[in] iobs
         * @param[in] XYZ
         * @param[out] A
         * @param[out] l
         * @param[out] P
         * @return int
         */
        int _addObsD(gnss_data_sats &satdata, unsigned int &iobs, Triple &XYZ, Matrix &A, Vector &l, Diag &P);

        //
        /**
         * @brief Satelite position
         * @param[in]  epo       time
         * @param[in]  gsatdata  satinfo
         * @return int 
         */
        virtual int _satPos(base_time &epo, gnss_data_sats &gsatdata);
        /**
         * @brief apply tide
         * @param[in]  _epoch    time
         * @param[in]  xRec      station coordinate
         * @return int 
         */
        virtual int _apply_tides(base_time &_epoch, Triple &xRec);
        /**
         * @brief caculate weight
         * @param[in]  satdata   satinfo
         * @param[in]  obstype   type of obs
         * @param[in]  g1        band1
         * @param[in]  g2        band2
         * @return double 
         */
        double _weightObs(gnss_data_sats &satdata, OBSCOMBIN obstype, gnss_data_obs *g1, gnss_data_obs *g2 = 0);
        /**
         * @brief print lsq
         * @param[in]  lsqestimator lsq class
         */
        virtual void _printout(const gnss_proc_lsqbase &lsqestimator);

        std::vector<gnss_data_sats> _data; ///< All the sat data

        unsigned int _minsat; ///< The minimum satellite number

        double _sig_unit; ///< Sigma in unit weight

        base_time _epoch; ///< Epoch time

        base_time _beg; ///< Begin time (in std::set)

        base_time _end; ///< End  time (in std::set)

        std::set<std::string> _sys; ///< used gnss system

        gnss_model_dop _dop; ///< For caculate the dop value, including pdop,vdop,hdop and so on

        base_iof *_result = nullptr; ///< output file
        std::map<base_time, int> _obs_num;
        // estimate velocity
        bool _dynamics = false;  ///< TODO
        bool _auto_band = false; ///< TODO
        CBIASCHAR _cbiaschar;    ///< TODO

        double _sig_ion; ///< sig of IONO

        RECEIVERTYPE _receiverType;                     ///< type of station
        std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> _band_index; ///< band index
    };

} // namespace

#endif // GSPPFLT_H
