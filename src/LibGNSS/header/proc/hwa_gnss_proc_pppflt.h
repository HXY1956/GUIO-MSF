#ifndef hwa_gnss_proc_pppflt_h
#define hwa_gnss_proc_pppflt_h

#define WIN32_LEAN_AND_MEAN
#define _WINSOCKAPI_

#include "hwa_base_io_tcp.h"
#include "hwa_gnss_proc_ppp.h"
#include "hwa_gnss_proc_sppflt.h"
#include "hwa_gnss_all_fltmat.h"
#include "hwa_base_xml.h"


namespace hwa_gnss
{
    class gnss_proc_pppflt : public gnss_proc_ppp,
                             public gnss_proc_sppflt,
                             public base_xml
    {
    public:
        explicit gnss_proc_pppflt(std::string mark, set_base *set);
        explicit gnss_proc_pppflt(std::string mark, set_base *set,base_log spdlog);
        virtual ~gnss_proc_pppflt();
        void add_coder(const std::vector<base_coder *> &coder);

        virtual int processBatch(const base_time &beg, const base_time &end, bool prtOut);
        virtual int processBatchFB(const base_time &beg, const base_time &end);
        void debugPrint();

    protected:
        // Process one epoch
        virtual int _processEpoch(const base_time &runEpoch);

        // add one satellite data to A, l, P - carrier phase
        virtual int _addObsL(gnss_data_sats &satdata, unsigned int &iobs, Triple &ell, Matrix &A, Vector &l, Diag &P);

        // Predict state std::vector and covariance matrix
        virtual void _predict();

        // Satelite position
        virtual int _satPos(base_time &, gnss_data_sats &);

        //   virtual int    _prepareData();

        // Applying tides
        virtual int _apply_tides(base_time &_epoch, Triple &xRec);

        // Synchronize ambiguity with respect to observation data.
        void _syncAmb();

        // Set output & return if smooth
        virtual void _setOut();

        // Set output & return if smooth
        void _prtOut(base_time &epo, base_allpar &X, const Symmetric &Q, std::vector<gnss_data_sats> &data, std::ostringstream &os, xml_node &node, bool saveProd = true);

        // Save apriory coordinates to products
        void _saveApr(base_time &epo, base_allpar &X, const Symmetric &Q);

        // Remove apriory coordinates from products
        void _removeApr(base_time &epo);

        xml_node line;
        bool _read;
        base_iof *_flt;
        base_iof *_enufile;
        base_iof* _obsqualityfile = nullptr;      // hlgou added in 20220512
        base_iof *_gpggafile = nullptr;           //lvhb added in 20210312
        std::map<OFMT, base_io_tcp*> _maptcp;            // zhshen
        base_iof *_smt;
        std::string _kml_name;
        bool _kml;
        bool _restarted;
        bool _beg_end;

        // Models
        gnss_model_random_walk *_grdStoModel;
        gnss_model_random_walk *_ambStoModel;

        // Update time for RDW processes
        virtual void _timeUpdate(const base_time &epo);

        // Data for smoothing (Rauch-Tung-Striebel algorithm)
        gnss_proc_fltmat _fltdata;
        gnss_all_fltmat _vfltdata;

        virtual void _backflt(bool prtOut);

        // Intervals for reseting parameters
        int _reset_amb;
        void _reset_ambig();
        void _check_ambig();
        int _reset_par;
        void _reset_param();
        Triple xyz_standard;          ///< enu coord reference
        std::map<std::string, int> _glofrq_num; ///< glofrq number, set GLO frequency number
        std::pair<int, Triple> neu_sum_nepo;

        //   std::ofstream      _iono_ofstr;
        //   std::ofstream      _ifcb_ofstr;

        // satellite pco correction
        //   void          _satPCO_corr(base_time& epo, gnss_data_sats& gsatdata);
    };

} // namespace

#endif
