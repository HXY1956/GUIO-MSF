#ifndef hwa_gnss_model_grid_H
#define hwa_gnss_model_grid_H

#include "hwa_gnss_data_obj.h"
#include "hwa_base_data.h"
#include "hwa_base_globaltrans.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_data_aug.h"
#include "hwa_gnss_data_augGRID.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_set_ionest.h"

using namespace hwa_base;
using namespace hwa_set;

#define MAXGRIDNUM 200
#define MAXSAgnss_coder_aug 201

namespace hwa_gnss
{
    /**
    *@brief       Class for gnss_model_precise_biasgpp, derive from gnss_model_precise_bias
    */
    class gnss_model_grid
    {
    public:
        gnss_model_grid();
        gnss_model_grid(hwa_set::set_base* set, std::shared_ptr<spdlog::logger> spdlog);
        gnss_model_grid(gnss_data_gridaug* grid_data, std::shared_ptr<spdlog::logger> spdlog);
        virtual ~gnss_model_grid();

        void setcurtime(const base_time& cur_time);
        void setObsData(std::vector<gnss_data_sats>& obs_used);
        bool getGridData(const base_time& nowT, gnss_data_augion& ion_grid);
        bool getGridData(const base_time& nowT, gnss_data_augtrop& trop_grid);
        std::map<GSYS, std::map<std::string, double>> getBias(const base_time& cur_time);
        int getNumofGrid();
        bool Aug2Grid(std::map<std::string, std::map<std::string, double>>& STECin, const std::map<std::string, double>& Tropin, std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_index, const std::map<std::string, Triple>& siteell);
        bool Grid2Aug(const base_time& cur_time, Triple& crd_site, std::map<std::string, double>& stec, std::map<std::string, double>& trop);
        bool Grid2Aug(const base_time& cur_time, Triple& crd_site, gnss_data_sats& obs, hwa_map_id_augtype_value& aug, GOBSBAND crt_band);
    protected:
        bool _Initdata(const std::map<std::string, std::map<std::string, double>>& augin, bool isTrop = false);
        bool _InitGridUsed(Triple& ell_site, int(&index)[4], Triple(&used_crd)[4]);
        bool _CalBias_AllSite(std::map<std::string, std::map<std::string, double>>& augin, std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_index, const std::map<std::string, Triple>& siteell);
        bool _CalBias_MINSite(std::map<std::string, std::map<std::string, double>>& augin, std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_index, const std::map<std::string, Triple>& siteell);
        bool _CalBias_2Site(std::map<std::string, std::map<std::string, double>>& augin, std::map<GSYS, std::map<FREQ_SEQ, GOBSBAND>> band_index, const std::map<std::string, Triple>& siteell);
        bool _CalCoefRes(const std::map<std::string, Triple>& siteell, bool isTrop = false);
        bool _CalCoefRes(const std::map<std::string, Triple>& siteell, const std::map<std::string, double>& Tropin);
        bool _findSite(Triple& grid_ell, const std::string& sat, const std::map<std::string, Triple>& sitesell, std::map<std::string, double>& site_dis);
        void _pushOneGrid(gnss_data_augion& ion);
        void _pushOneGrid(gnss_data_augtrop& trop);
        double _getSatele(const std::string& sat, Triple site_crd);
    protected:
        gnss_data_gridaug* _grid_data = nullptr;

        std::string _mask;                       ///< Grid name
        std::string _ID;                         ///< Grid ID
        double _RefLon;                     ///< reference longitude
        double _RefLat;                     ///< reference latitude
        double _RefH;                       ///< reference height
        double _SpaceLon;                   ///< space of grid for longitude
        double _SpaceLat;                   ///< space of grid for latitude
        int _CountLon;                      ///< count of grid for longitude
        int _CountLat;                      ///< count of grid for latitude
        double _maxBias_sigma;              ///< Bias equ choose sigma
        double _maxBias_baseline;           ///< Bias equ choose max distance

        int _min_site;
        unsigned int  _grid_count[3] = { 0 }; // dimension 3 not use
        double _refLat;
        double _refLon;
        double _grid_space[2] = { 0 };
        double _center_BLH[2] = { 0.0 };
        Triple _grid_pos[MAXGRIDNUM];

        std::shared_ptr<spdlog::logger> _mylogger;
        std::vector<gnss_data_sats> _obs_used;
        std::map<std::string, std::map<std::string, double>> _STECValSite;
        std::map<std::string, std::map<std::string, double>> _TropValSite;
        std::string _RefSite;
        std::map<base_time, std::map<GSYS, std::map<std::string, double>>> _Bias;
        std::map<std::string, std::map<std::string, double>> _Site_Site_dis;
        base_time _cur_time;
        bool          _isZWD = false;
    };
}
#endif