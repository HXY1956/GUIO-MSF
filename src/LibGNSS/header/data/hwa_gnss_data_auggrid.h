#ifndef hwa_gnss_data_augGRID_H
#define hwa_gnss_data_augGRID_H

#include "hwa_gnss_data_obj.h"
#include "hwa_base_data.h"
#include "hwa_base_globaltrans.h"
#include "hwa_gnss_sys.h"
#include "hwa_gnss_data_aug.h"
#define MAXGRIDNUM 200
#define MAXSAgnss_coder_aug 201

namespace hwa_gnss
{
    class gnss_data_SATION
    {
    public:
        gnss_data_SATION() {};
        virtual ~gnss_data_SATION() {};

        unsigned int valid = 0;
        std::string prn;
        //unsigned int isys = 0;
        unsigned int equlity = 0;
        GSYS    csys;
        double  C[4] = { 0 };// C00/C01/C10/C11
        double  res[MAXGRIDNUM] = { 0 };
        double  bias = 0;
    };

    class gnss_data_SATTROP
    {
    public:
        gnss_data_SATTROP() {};
        virtual ~gnss_data_SATTROP() {};


        unsigned int valid = 0;
        std::string prn;
        //unsigned int isys = 0;
        unsigned int equlity = 0;
        GSYS    csys;
        double  T[4] = { 0 };// C00/C01/C10/C11
        double  res[MAXGRIDNUM] = { 0 };
        double  bias = 0;
    };

    class gnss_data_augion
    {
    public:
        gnss_data_augion() {};
        virtual ~gnss_data_augion() {};

        unsigned int equ_type = 0;
        unsigned int nsat = 0;
        unsigned int index = 0;//hjx broadcast onesat
        std::map<std::string,gnss_data_SATION> satSTEC;
        //gnss_data_SATION satRes[MAXSAgnss_coder_aug];
    };

    class gnss_data_augtrop
    {
    public:
        gnss_data_augtrop() {};
        virtual ~gnss_data_augtrop() {};

        unsigned int equ_type = 0;
        unsigned int nsat = 0;
        unsigned int index = 0;//hjx broadcast onesat
        std::map<std::string, gnss_data_SATTROP> satTrop;
        gnss_data_SATTROP siteZWD;
        //gnss_data_SATION satTrop[MAXSAgnss_coder_aug];
    };

    //class gnss_data_augtrop
    //{
    //public:
    //    unsigned int valid = 0;
    //    unsigned int equ_type = 0;
    //    unsigned int equlity = 0;
    //    double  zhd = 0;
    //    double  T[4] = { 0 };// T00/T01/T10/T11
    //    double  res[MAXGRIDNUM] = { 0 };
    //};

    class gnss_data_gridaug : public base_data
    {
    public:
        gnss_data_gridaug();
        gnss_data_gridaug(int row, int col, double lat, double lon, double dlat, double dlon);
        virtual ~gnss_data_gridaug();
        int gridNum();
        int getGridRow();
        int getGridCol();
        std::string getMarker();
        std::string getID();
        double getRefLon();
        double getRefLat();
        double getSpaceLon();
        double getSpaceLat();

        void addGridData(base_time& epoch, gnss_data_SATION& data, std::string& sat, int nsat);
        void addGridData(base_time& epoch, gnss_data_augtrop& data);
        void addGridData(base_time& epoch, gnss_data_augion& data);
        
        void setHeader(int row, int col, double lat, double lon, double dlat, double dlon, std::string id, std::string marker, std::map<GSYS,GOBSBAND> sys_band);
        void setRefSite(std::string& site);
        bool getGridData(const base_time& nowT,gnss_data_augion& ion_grid);
        bool getGridData(const base_time& nowT,gnss_data_augtrop& trop_grid);
        
        
    private:
        void _setGridNum(int row, int col);
        void _setRefBL(double lat, double lon);
        void _setGridSpace(double dlat, double dlon);
    
        unsigned int  area_version = 0;
        unsigned int  idc_no = 0;
        unsigned int  narea = 0;
        unsigned int  area_id = 0;
        unsigned int  valid = 0;
        unsigned int  endtag = 0;                //hjx broadcast onesat
        bool          _isZWD = false;
        base_time _Time;
        std::map<base_time, gnss_data_augion> _AllIonoGrid;
        std::map<base_time, gnss_data_augtrop> _AllTropGrid;

        double N1[MAXSAgnss_coder_aug] = { 0 }; // not broadcast
        double N2[MAXSAgnss_coder_aug] = { 0 };
    private:
        unsigned int  _grid_count[3] = { 0 }; // dimension 3 not use
        double _refLat;
        double _refLon;
        double _grid_space[2] = { 0 };
        double _center_BLH[2] = { 0.0 };
        int _min_site;
        std::string _ID;
        std::string _Marker;
        std::map<GSYS, GOBSBAND> _sys_band;
        std::vector<gnss_data_sats> _obs_used;
        Triple _grid_pos[MAXGRIDNUM];
        std::map<std::string, std::map<std::string,double>> _STECValSite;
        std::map<std::string, std::map<std::string, double>> _TropValSite;
        std::shared_ptr<spdlog::logger> _mylogger;
        std::string _RefSite;
    };

} // namespace

#endif