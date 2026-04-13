#ifndef hwa_gnss_data_interpol_H
#define hwa_gnss_data_interpol_H

#include "hwa_set_base.h"
#include "hwa_base_log.h"
#include "hwa_base_file.h"
#include "hwa_base_allproc.h"
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_data_aug.h"
#include "hwa_gnss_data_augGRID.h"
#include "hwa_gnss_model_grid.h"
#include "hwa_gnss_all_obs.h"

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    /**
    * @brief virtual class for interpolation method
    */
    class gnss_data_interpol;
    typedef std::shared_ptr<gnss_data_interpol> hwa_SPT_interpol;
    class gnss_data_interpol
    {
    public:
        /**
        * @brief constructor.
        *
        * @param[in]  set          gsetbase
        * @param[in]  data          the data
        * @param[in]  log          the log
        *
        */
        explicit gnss_data_interpol(set_base *set,  base_all_proc *data, base_log spdlog);

        explicit gnss_data_interpol(set_base *set,  base_all_proc *data, std::string &site, base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_interpol();

        /** 
        * @brief  Get Aug-data for NRTK/URTK/PPP-RTK 
        * 
        * @param[in]  nowT            the current time
        * @param[in]  data_rover    the data
        * @param[in]  data_base        the data base
        * @return      bool          the interpol mode
        * 
        */
        bool interpolAug(const base_time &nowT, std::vector<gnss_data_sats> &data_rover, std::vector<gnss_data_sats> &data_base);

        /**
        * @brief  Get Aug-data for NRTK/URTK/PPP-RTK from grid aug
        *
        * @param[in]  nowT            the current time
        * @param[in]  data_rover    the data
        * @param[in]  data_base        the data base
        * @return      bool          the interpol mode
        *
        */
        bool Grid2Aug(const base_time& nowT, std::vector<gnss_data_sats>& data_rover, std::vector<gnss_data_sats>& data_base);

        /**
        * @brief  Get Aug-filename
        *
        * @param[in]  augfile        the aug filename
        * @return      void
        *
        */
        void getAugFileName(base_iof* augfile);

        /**
        * @brief  Get Aug-data
        *
        * @param[in]  nowT            the current time
        * @return      hwa_map_id_augtype_value
        */
        hwa_map_id_augtype_value &augOut(const base_time &nowT) { return *_allaug->getAug(_site, nowT); }

        Triple get_crd_rover() { return _crd_rover; }

    private:

        /**
        * @brief  set coordinate
        *
        * @param[in]  dx            x column
        * @param[in]  dy            y column
        * @param[in]  dz            z column
        * @return      void
        */
        void _setCrd(Vector &dx, Vector &dy, Vector &dz);

        /**
        * @brief  DIM
        * lvhb added for 2 augs
        * 
        * @param[in]  AugData        aug data
        * @return      double        result
        */
        double _DIM(Vector &AugData);

        /**
        * @brief  MLCM
        * for NRTK/URTK/PPP-RTK,lvhb
        * 
        * @param[in]  AugData        aug data
        * @return      double        result
        */
        double _MLCM(Vector &AugData);

        /**
        * @brief  for URTK interpol
        * for URTK: refer zou xuan paper, only support 3 sites,lvhb in 202010
        *
        * @param[in]  AugData        aug data
        * @return      double        result
        */
        double _URTKInterpol(Vector &dAugData);

        /**
        * @brief  for NRTK interpol
        * for NRTK:twm, only support 3 sites,lvhb test in 202010
        *
        * @param[in]  AugData        aug data
        * @return      double        result
        */
        double _NRTKInterpol(Vector &dAugData);

        Vector _crd_dx;            ///< dx coordinate
        Vector _crd_dy;            ///< dy coordinate
        Vector _crd_dz;            ///< dz coordinate
        Vector _coef_a;            ///< coef a

        //hwa_map_id_augtype_value    _aug_out;
    private: 
        
        ///< Lvhb: mainly for NRTK solution about following functions
        /* 
        * @brief Select major site according baseline length,Just for NRTK,lvhb 
        * 
        * @param[in]  nowT            current time
        * @return      bool            select mode
        */
        bool _selMajorSite(const base_time &nowT);

        /*
        * @brief select Delaunay Site
        *
        * @param[in]  nowT            current time
        * @param[in]  selsite        site list
        * @return      bool            select mode
        */
        bool _selDelaunaySite(const base_time &nowT, std::vector<std::string> &selsite);

        /* 
        * @brief Check whether current VRS data exist,Just for NRTK,lvhb 
        * 
        * @param[in]  nowT            current time
        * @param[in]  sat            satellite name
        * @return      hwa_spt_obsmanager
        */
        hwa_spt_obsmanager _check_vrs_prn_pt(const base_time &nowT, std::string &sat);

        /** 
        * @brief Get SPP approximately coordinates 
        * 
        * @param[in]  nowT            current time
        * @param[in]  site            site name
        * @param[in]  crd            coordinate
        * @param[in]  reclk            receiver clock
        * @param[in]  satdata        satellite data
        * @return      bool            spp process mode
        */
        virtual bool _SppProcOneEpoch(const base_time &nowT, const std::string &site, Triple *crd, double *reclk, std::map<std::string, gnss_data_sats> *satdata = NULL);

        /** 
        * @brief Calculate satellite position/clk/rho/elevation/azel,lvhb added for NRTK 
        * 
        * @param[in]  obs_sat        satellite observation data
        * @param[in]  xyz_r            xyz coordinate
        * @return      void
        */
        void _cal_sat_inf(gnss_data_sats &obs_sat, const Triple &xyz_r);

        /** 
        * @brief Get Tropospheric delay according SAAS or other model,lvhb added for NRTK(URTK) 
        * 
        * @param[in]  nowT            current time
        * @param[in]  obsdata        observation data
        * @return      void
        */
        double _getTropModValue(const base_time &nowT, gnss_data_sats &obsdata);

        /**
        * @brief judge whether avaliable
        *
        * @param[in]  nowT            current time
        * @param[in]  site            site name
        * @param[in]  site_base        base site name
        * @return      bool            whether available
        */
        virtual bool _avaliable(base_time nowT, std::string *site = NULL, std::string *site_base = NULL);

        /*  
        * @brief Not using now by as a result of using comprehensive augmentation in current NRTK
        * 
        * @param[in]  nowT            current time
        * @param[in]  sign            sign
        * @param[in]  bcrd            base coordinate
        * @param[in]  rcrd            reciver coordinate
        * @param[in]  crd            coordinate
        * @param[in]  bdata_sat        base satellite data
        * @param[in]  bdata_bsat    base bsatellite data
        * @param[in]  rdata_sat        rsatellite data
        * @param[in]  rdata_bsat    rdata bsat
        * @return      double        result
        */
        double _corrTropHeight(const base_time &nowT, int sign, const Triple &bcrd, Triple &rcrd, const Triple &crd,
                               gnss_data_sats bdata_sat, gnss_data_sats bdata_bsat, gnss_data_sats rdata_sat, gnss_data_sats rdata_bsat);

        /** 
        * @brief Calculate rho/ele according to positions of Sat/Site,lvhb added to for NRTK 
        * 
        * @param[in]  site_name        site name
        * @param[in]  xyz_s            satellite coordinate
        * @param[in]  xyz_r            receiver coordinate
        * @param[in]  obs_sat        observation data
        * @return      void
        */
        void cal_rho_azel(const std::string &site_name, Triple &xyz_s, const Triple &xyz_r, gnss_data_sats &obs_sat);

    private:
        hwa_set::set_base *            _set = nullptr;            ///< settings
         base_all_proc *            _alldata = nullptr;        ///< all data
        base_log _spdlog = nullptr; ///< spdlog
        gnss_all_obj *                _allobj = nullptr;        ///< all object
        gnss_all_nav *                _allnav = nullptr;        ///< all navigation data
        gnss_data_aug *                _allaug = nullptr;        ///< all Augmentation data
        gnss_data_gridaug*             _allauggrid = nullptr;      ///< all grid augmentation
        gnss_model_grid*             _grid_model = nullptr;      ///< grid model
        base_iof*                    _augfile = nullptr;        ///< aug file name
        bool                    _isBase;                ///< For DD/RTK model
        bool                    _isCompAug;                ///< whether compute aug, @lvhb, 202010
        bool                    _isGridAug;                ///< whether grid aug
        bool                    _isDelaunay;            ///< whether compute Delaunay
        bool                    _isCorObs;              ///< whether correct observation
        std::string                    _site;                    ///< site name
        std::string                    _strMajorSite;            ///< For NRTK
        std::string                    _site_vrs;                ///< For NRTK
        int                        _nipSite = 0;            ///< site number
        std::vector<std::string>            _list_ipsite;            ///< sites name lists

        //for one epoch augmentation information
        std::vector<gnss_data_sats>        _data_rover;            ///< rover data 
        std::vector<gnss_data_sats>        _data_base;                ///< base data
        Triple                _crd_rover;                ///< rover coordinate
        double                    _clk_rovor;                ///< rover clock

        double                    _comp_aug_limit = 0.0;    ///< compute aug limit
        double                    _ion_aug_limit = 0.0;    ///< ion aug limit
        double                    _trop_aug_limit = 0.0;    ///< trop aug limit
        bool                    _isSelfCorr = false;    ///< whether self correct
        int                        _obs_level = 3;            ///< observation level
        bool                    _simulation = false;    ///< whether the data is simulation

        /* For one epoch processing,lvhb created for URTK/PPP-RTK SSR */
        //map<int, map<string, map<GOBSBAND, pair<double, double>>>> _AllCorr; ///< for one epoch: first-range,second-phase //int site,string sat
        //map<int, map<string, map<GOBSBAND, double>>> _AllIonAug;             ///< int site,string sat
        //map<int, map<string, double>> _AlltropAug;                             ///< int site,string sat
    };

}
#endif