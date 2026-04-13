#include "hwa_gnss_data_ambflag.h"
#include "hwa_gnss_data_augGRID.h"
#include<iostream>

using namespace std;

namespace hwa_gnss {
    gnss_data_gridaug::gnss_data_gridaug():_isZWD(true)
    {
        id_type(base_data::AUGGRID);
    }
    gnss_data_gridaug::gnss_data_gridaug(int row, int col, double lat, double lon,double dlat,double dlon):_isZWD(true)
    {
        id_type(base_data::AUGGRID);
        _setGridNum(row, col);
        _setRefBL(lat, lon);
        _setGridSpace(dlat, dlon);
        _center_BLH[0] = _refLat - _grid_space[0] * (_grid_count[0] - 1) / 2.0;
        _center_BLH[1] = _refLon - _grid_space[1] * (_grid_count[1] - 1) / 2.0;
        _obs_used.clear();
        _min_site = 3;
    }
    void gnss_data_gridaug::_setGridNum(int row, int col)
    {
        _grid_count[0] = row;
        _grid_count[1] = col;
    }
    gnss_data_gridaug::~gnss_data_gridaug()
    {
    }
    int gnss_data_gridaug::gridNum()
    {
        return _grid_count[0] * _grid_count[1];
    }
    int gnss_data_gridaug::getGridRow()
    {
        return _grid_count[0];
    }
    int gnss_data_gridaug::getGridCol()
    {
        return _grid_count[1];
    }
    string gnss_data_gridaug::getMarker()
    {
        return _Marker;
    }
    string gnss_data_gridaug::getID()
    {
        return _ID;
    }
    double gnss_data_gridaug::getRefLon()
    {
        return _refLon;
    }
    double gnss_data_gridaug::getRefLat()
    {
        return _refLat;
    }
    double gnss_data_gridaug::getSpaceLon()
    {
        return _grid_space[1];
    }
    double gnss_data_gridaug::getSpaceLat()
    {
        return _grid_space[0];
    }
    void gnss_data_gridaug::addGridData(base_time& epoch, gnss_data_SATION& data, string& sat, int nsat)
    {
        _AllIonoGrid[epoch].nsat = nsat;
        _AllIonoGrid[epoch].satSTEC[sat] = data;
        _AllIonoGrid[epoch].satSTEC[sat].valid = 1;
    }
    void gnss_data_gridaug::addGridData(base_time& epoch, gnss_data_augtrop& data)
    {
        _AllTropGrid[epoch] = data;
    }
    void gnss_data_gridaug::addGridData(base_time& epoch, gnss_data_augion& data)
    {
        _AllIonoGrid[epoch] = data;
    }
    void gnss_data_gridaug::setHeader(int row, int col, double lat, double lon, double dlat, double dlon, string id, string marker, map<GSYS, GOBSBAND> sys_band)
    {
        _setGridNum(row, col);
        _setRefBL(lat, lon);
        _setGridSpace(dlat, dlon);
        _center_BLH[0] = _refLat - _grid_space[0] * (_grid_count[0] - 1) / 2.0;
        _center_BLH[1] = _refLon - _grid_space[1] * (_grid_count[1] - 1) / 2.0;
        _obs_used.clear();
        _ID = id;
        _Marker = marker;
        _sys_band = sys_band;
    }
    void gnss_data_gridaug::setRefSite(string& site)
    {
        _RefSite = site;
    }
    bool gnss_data_gridaug::getGridData(const base_time& nowT, gnss_data_augion& ion_grid)
    {
        if (_AllIonoGrid.find(nowT) != _AllIonoGrid.end())
        {
            ion_grid = _AllIonoGrid[nowT];
            return true;
        }
        else
        {
            auto latter = _AllIonoGrid.lower_bound(nowT);
            auto former = --latter;
            double dist_latter = fabs(nowT.diff(latter->first));
            double dist_former = fabs(nowT.diff(former->first));
            if (dist_latter <= dist_former)
            {
                if (dist_latter <= 300)
                {
                    ion_grid = latter->second;
                    return true;
                }
                else
                    return false;
            }
            else
            {
                if (dist_former <= 300)
                {
                    ion_grid = former->second;
                    return true;
                }
                else
                    return false;
            }
            
        }
        return false;
    }
    bool gnss_data_gridaug::getGridData(const base_time& nowT, gnss_data_augtrop& trop_grid)
    {
        if (_AllTropGrid.find(nowT) != _AllTropGrid.end())
        {
            trop_grid = _AllTropGrid[nowT];
            return true;
        }
        else
        {
            auto latter = _AllTropGrid.lower_bound(nowT);
            auto former = --latter;
            double dist_latter = fabs(nowT.diff(latter->first));
            double dist_former = fabs(nowT.diff(former->first));
            if (dist_latter <= dist_former)
            {
                if (dist_latter <= 300)
                {
                    trop_grid = latter->second;
                    return true;
                }
                else
                    return false;
            }
            else
            {
                if (dist_former <= 300)
                {
                    trop_grid = former->second;
                    return true;
                }
                else
                    return false;
            }
        }
        return false;
    }
    void gnss_data_gridaug::_setRefBL(double lat, double lon)
    {
        _refLat = lat;
        _refLon = lon;
    }
    void gnss_data_gridaug::_setGridSpace(double dlat, double dlon)
    {
        _grid_space[0] = dlat;
        _grid_space[1] = dlon;
    }
    

}//namespace