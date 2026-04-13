#include "hwa_gnss_data_ambflag.h"
#include "hwa_gnss_model_grid.h"
#define DEBUG 1

using namespace std;

namespace hwa_gnss
{
    gnss_model_grid::gnss_model_grid()
    {
    }
    gnss_model_grid::gnss_model_grid(hwa_set::set_base* set, shared_ptr<spdlog::logger> spdlog):_isZWD(true)
    {
        _mylogger = spdlog;
        _mask = dynamic_cast<set_ionest*>(set)->mask_grid();
        _ID = dynamic_cast<set_ionest*>(set)->ID_grid();
        _RefLon = dynamic_cast<set_ionest*>(set)->reflon_grid();
        _RefLat = dynamic_cast<set_ionest*>(set)->reflat_grid();
        _SpaceLon = dynamic_cast<set_ionest*>(set)->spacelon_grid();
        _SpaceLat = dynamic_cast<set_ionest*>(set)->spacelat_grid();
        _CountLon = dynamic_cast<set_ionest*>(set)->countlon_grid();
        _CountLat = dynamic_cast<set_ionest*>(set)->countlat_grid();
        _RefSite = dynamic_cast<set_ionest*>(set)->ref_Site();
        _min_site = dynamic_cast<set_ionest*>(set)->min_site();
        _maxBias_baseline = dynamic_cast<set_ionest*>(set)->max_Baseline();
        _maxBias_sigma = dynamic_cast<set_ionest*>(set)->max_Sigma();
        if (!_grid_data)
            _grid_data = new gnss_data_gridaug(_CountLat, _CountLon, _RefLat, _RefLon, _SpaceLat, _SpaceLon);
        _grid_count[0] = _CountLat;
        _grid_count[1] = _CountLon;
        _refLat = _RefLat;
        _refLon = _RefLon;
        _grid_space[0] = _SpaceLat;
        _grid_space[1] = _SpaceLon;
        _center_BLH[0] = _RefLat - _grid_space[0] * (_grid_count[0] - 1) / 2.0;
        _center_BLH[1] = _RefLon - _grid_space[1] * (_grid_count[1] - 1) / 2.0;
        _obs_used.clear();
        _Site_Site_dis.clear();

    }
    gnss_model_grid::gnss_model_grid(gnss_data_gridaug* grid_data, shared_ptr<spdlog::logger> spdlog) :_isZWD(true),_grid_data(grid_data),_mylogger(spdlog)
    {
        
        
        if (_grid_data != nullptr)
        {
            _mask = _grid_data->getMarker();
            _ID = _grid_data->getID();
            _RefLon = _grid_data->getRefLon();
            _RefLat = _grid_data->getRefLat();
            _SpaceLon = _grid_data->getSpaceLon();
            _SpaceLat = _grid_data->getSpaceLat();
            _CountLon = _grid_data->getGridCol();
            _CountLat = _grid_data->getGridRow();
        }
        else
        {

            _mask = "NONE";
            _ID = "NONE";
            _RefLon = 0.0;
            _RefLat = 0.0;
            _SpaceLon = 0.0;
            _SpaceLat = 0.0;
            _CountLon = 0;
            _CountLat = 0;
        }
        

        
        _grid_count[0] = _CountLat;
        _grid_count[1] = _CountLon;
        _refLat = _RefLat;
        _refLon = _RefLon;
        _grid_space[0] = _SpaceLat;
        _grid_space[1] = _SpaceLon;
        _center_BLH[0] = _RefLat - _grid_space[0] * (_grid_count[0] - 1) / 2.0;
        _center_BLH[1] = _RefLon - _grid_space[1] * (_grid_count[1] - 1) / 2.0;
        _obs_used.clear();

    }
    gnss_model_grid::~gnss_model_grid()
    {
    }
    void gnss_model_grid::setcurtime(const base_time& cur_time)
    {
        _cur_time = cur_time;
    }
    void gnss_model_grid::setObsData(vector<gnss_data_sats>& obs_used)
    {
        _obs_used = obs_used;
    }
    bool gnss_model_grid::getGridData(const base_time& nowT, gnss_data_augion& ion_grid)
    {
        return _grid_data->getGridData(nowT, ion_grid);
    }
    bool gnss_model_grid::getGridData(const base_time& nowT, gnss_data_augtrop& trop_grid)
    {
        return _grid_data->getGridData(nowT, trop_grid);
    }
    map<GSYS, map<string, double>> gnss_model_grid::getBias(const base_time& cur_time)
    {
        if (_Bias.find(cur_time) != _Bias.end())
            return _Bias[cur_time];
        return map<GSYS, map<string, double>>();
    }
    bool gnss_model_grid::Aug2Grid(map<string, map<string, double>>& STECin, const map<string, double>& Tropin, map<GSYS, map<FREQ_SEQ, GOBSBAND>> band_index, const map<string, Triple>& siteell)
    {
        if (STECin.size() < 3 || Tropin.size() < 3)
            return false;
        /*if (!_CalBias_2Site(STECin, band_index, siteell))
            return false;*/
        /*if (!_CalBias_AllSite(STECin, band_index, siteell))
            return false;*/
        if (!_CalBias_MINSite(STECin, band_index, siteell))
            return false;

        if (!_Initdata(STECin, false))
            return false;

        int gridnum = 0;
        for (int i = 0; i < _grid_count[0]; i++)
        {
            for (int j = 0; j < _grid_count[1]; j++)
            {
                gridnum++;
                Triple blh_temp = { _refLat - _grid_space[0] * i,_refLon + _grid_space[1] * j,0.0 };
                Triple blh;
                blh = blh_temp;
                ell2xyz(blh, _grid_pos[gridnum - 1], true);
            }
        }

        /*calculate ion coefficient and grid value*/
        if (!_CalCoefRes(siteell))
            return false;

        /*calculate ion coefficient and grid value*/
        if (!_CalCoefRes(siteell, Tropin))
            return false;



        return true;
    }
    /*test*/
    bool gnss_model_grid::Grid2Aug(const base_time& cur_time, Triple& crd_site, map<string, double>& stec, map<string, double>& trop)
    {
        stec.clear();
        trop.clear();
        Vector B(4), P(4), used_res(4);
        double w = 0;
        double I_G, I_P;
        int used_index[4];
        Triple crd_used[4];
        Triple ell_site(0.0, 0.0, 0.0);
        xyz2ell(crd_site, ell_site, true);
        if (!_InitGridUsed(ell_site, used_index, crd_used))
            return false;
        gnss_data_augtrop cur_trop;
        gnss_data_augion cur_ion;
        if (!_grid_data->getGridData(cur_time, cur_trop) || !_grid_data->getGridData(cur_time, cur_ion))
        {
            return false;
        }
        
        if (cur_ion.satSTEC.size() == 0 || (cur_trop.satTrop.size() == 0 && !_isZWD))
            return false;

        B(0) = 1;
        B(1) = ell_site[0] - _center_BLH[0];
        B(2) = ell_site[1] - _center_BLH[1];
        B(3) = B(1) * B(2);

        for (int i = 0; i < 4; i++)
        {
            Triple grid_ell;
            xyz2ell2(crd_used[i], grid_ell, true);
            P(i) = 1 / sqrt(pow(grid_ell[0] - ell_site[0], 2) + pow(grid_ell[1] - ell_site[1], 2));
            //P(i) = 1 / sqrt(pow(crd_used[i][0] - crd_site[0], 2) + pow(crd_used[i][1] - crd_site[1], 2));
            w += P(i);
            //used_res(i) = sat_ion_grid.res[used_index[k]];
        }
        P = P / w;

        for (auto iter : cur_ion.satSTEC)
        {
            if (iter.second.valid != 1)
                continue;
            for (int k = 0; k < 4; k++)
            {
                used_res(k) = iter.second.res[used_index[k]];
            }
            I_G = (P.transpose() * used_res).sum();
            I_P = 0.0;
            for (int k = 0; k < 4; k++)
            {
                I_P += iter.second.C[k] * B(k);
            }
            stec[iter.first] = I_G + I_P;
        }
        for (int i = 0; i < 4; i++)
        {
            P(i) = 1 / sqrt(pow(crd_used[i][0] - crd_site[0], 2) + pow(crd_used[i][1] - crd_site[1], 2));
            w += P(i);
        }

        if (!_isZWD)
        {
            for (auto iter : cur_trop.satTrop)
            {
                if (iter.second.valid != 1)
                    continue;
                for (int k = 0; k < 4; k++)
                {
                    used_res(k) = iter.second.res[used_index[k]];
                }
                I_G = (P.transpose() * used_res).sum();
                I_P = 0.0;
                for (int k = 0; k < 4; k++)
                {
                    I_P += iter.second.T[k] * B(k);
                }
                trop[iter.first] = I_G + I_P;
            }
        }
        else
        {
            if (cur_trop.siteZWD.valid != 1)
                return false;
            for (int k = 0; k < 4; k++)
            {
                used_res(k) = cur_trop.siteZWD.res[used_index[k]];
            }
            I_G = (P.transpose() * used_res).sum();
            I_P = 0.0;
            for (int k = 0; k < 4; k++)
            {
                I_P += cur_trop.siteZWD.T[k] * B(k);
            }
            trop["ZWD"] = I_G + I_P;
        }


        return true;
    }
    bool gnss_model_grid::Grid2Aug(const base_time& cur_time, Triple& crd_site, gnss_data_sats& obs, hwa_map_id_augtype_value& aug, GOBSBAND crt_band)
    {
        if (double_eq(crd_site.norm(), 0.0))
        {
            SPDLOG_LOGGER_ERROR(_mylogger, cur_time.str_ymdhms("No crd_site"));
            return false;
        }
        Vector B(4), P(4), used_res(4);
        double w = 0;
        double I_G, I_P;
        int used_index[4];
        Triple crd_used[4];
        Triple ell_site(0.0, 0.0, 0.0);
        xyz2ell(crd_site, ell_site, true);
        if (!_InitGridUsed(ell_site, used_index, crd_used))
            return false;
        string sat = obs.sat();
        gnss_data_augtrop cur_trop;
        gnss_data_augion cur_ion;
        gnss_data_SATION sat_ion_grid;
        if (!_grid_data->getGridData(cur_time, cur_trop) || !_grid_data->getGridData(cur_time, cur_ion))
        {
            return false;
        }
        else
        {
            if (cur_ion.satSTEC.find(sat) != cur_ion.satSTEC.end())
                sat_ion_grid = cur_ion.satSTEC[sat];
            else
            {
                SPDLOG_LOGGER_ERROR(_mylogger, cur_time.str_ymdhms(sat + "No Iono Grid data"));
                return false;
            }
            
        }

        if (cur_ion.satSTEC.size() == 0 || (cur_trop.satTrop.size() == 0 && !_isZWD))
            return false;
        B(0) = 1;
        B(1) = ell_site[0] - _center_BLH[0];
        B(2) = ell_site[1] - _center_BLH[1];
        B(3) = B(1) * B(2);
        _obs_used.push_back(obs);
        double STEC;
        double ion, trop;
        double fre1;
        fre1 = obs.frequency(crt_band);
        double fac = fre1 * fre1 / (40.3e16);
        // weight ell
        for (int i = 0; i < 4; i++)
        {
            Triple grid_ell;
            xyz2ell2(crd_used[i], grid_ell, true);
            P(i) = 1 / sqrt(pow(grid_ell[0] - ell_site[0], 2) + pow(grid_ell[1] - ell_site[1], 2));
            //P(i) = 1 / sqrt(pow(crd_used[i][0] - crd_site[0], 2) + pow(crd_used[i][1] - crd_site[1], 2));
            w += P(i);
            //used_res(i) = sat_ion_grid.res[used_index[k]];
        }
        // weight crd
        //for (int i = 0; i < 4; i++)
        //{
        //    Triple grid_ell;
        //    xyz2ell2(crd_used[i], grid_ell, true);
        //    //P(i) = 1 / sqrt(pow(grid_ell[0] - ell_site[0], 2) + pow(grid_ell[1] - ell_site[1], 2));
        //    P(i) = 1 / sqrt(pow(crd_used[i][0] - crd_site[0], 2) + pow(crd_used[i][1] - crd_site[1], 2));
        //    w += P(i);
        //    //used_res(i) = sat_ion_grid.res[used_index[k]];
        //}
        if (sat_ion_grid.valid == 1)
        {
            //double site_ele = _getSatele(sat, crd_site);
            for (int k = 0; k < 4; k++)
            {
                //double grid_ele = _getSatele(sat, crd_used[k]);
                //P(k) = 1 / fabs(grid_ele - site_ele);
                //P(k) = 1 / sqrt(pow(crd_used[k][0] - crd_site[0], 2) + pow(crd_used[k][1] - crd_site[1], 2));
                //w += P(k);
                used_res(k) = sat_ion_grid.res[used_index[k]];
            }
            P = P / w;
            I_G = (P.transpose() * used_res).sum();
            I_P = 0.0;
            for (int k = 0; k < 4; k++)
            {
                I_P += sat_ion_grid.C[k] * B(k);
            }
            STEC = I_G + I_P;
        }
        ion = STEC / fac;
        w = 0;

        //P = P / w;
        if (!_isZWD)
        {
            for (auto iter : cur_trop.satTrop)
            {
                if (iter.second.valid != 1)
                    continue;
                for (int k = 0; k < 4; k++)
                {
                    used_res(k) = iter.second.res[used_index[k]];
                }
                I_G = (P.transpose() * used_res).sum();
                I_P = 0.0;
                for (int k = 0; k < 4; k++)
                {
                    I_P += iter.second.T[k] * B(k);
                }
                trop = I_G + I_P;
            }
        }
        else
        {
            if (cur_trop.siteZWD.valid != 1)
                return false;
            for (int k = 0; k < 4; k++)
            {
                used_res(k) = cur_trop.siteZWD.res[used_index[k]];
            }
            I_G = (P.transpose() * used_res).sum();
            I_P = 0.0;
            for (int k = 0; k < 4; k++)
            {
                I_P += cur_trop.siteZWD.T[k] * B(k);
            }
            trop = I_G + I_P;
        }
        aug[sat][make_pair(AUGTYPE::TYPE_ION, crt_band)] = ion;
        aug[sat][make_pair(AUGTYPE::TYPE_TRP, crt_band)] = trop;

        return true;
    }
    int gnss_model_grid::getNumofGrid()
    {
        return _grid_count[0] * _grid_count[1];
    }
    bool gnss_model_grid::_Initdata(const map<string, map<string, double>>& augin, bool isTrop)
    {
        if (augin.size() < 3)
            return false;
        _STECValSite.clear();
        _TropValSite.clear();
        for (auto iter = augin.begin(); iter != augin.end(); iter++)
        {
            for (auto jter = iter->second.begin(); jter != iter->second.end(); jter++)
            {
                if (isTrop)
                    _TropValSite[jter->first][iter->first] = jter->second;
                else
                    _STECValSite[jter->first][iter->first] = jter->second;
            }
        }
        return true;
    }
    bool gnss_model_grid::_InitGridUsed(Triple& ell_site, int(&index)[4], Triple(&used_crd)[4])
    {
        int row = 0;
        int col = 0;
        row = ((-ell_site[0] + _refLat) / _grid_space[0]);
        col = ((ell_site[1] - _refLon) / _grid_space[1]);
        if (row > _grid_count[0] - 1 || col > _grid_count[1] - 1 || row < 0 || col < 0)
            return false;
        Triple ell_grid[4];

        row = floor(row);
        col = floor(col);
        index[0] = row * _grid_count[1] + col;
        index[1] = index[0] + 1;
        index[2] = (row + 1) * _grid_count[1] + col;
        index[3] = index[2] + 1;

        ell_grid[0][0] = _refLat - row * _grid_space[0];
        ell_grid[0][1] = _refLon + col * _grid_space[1];
        ell_grid[0][2] = 0.0;

        ell_grid[1][0] = ell_grid[0][0];
        ell_grid[1][1] = ell_grid[0][1] + _grid_space[1];
        ell_grid[1][2] = 0.0;

        ell_grid[2][0] = ell_grid[0][0] - _grid_space[0];
        ell_grid[2][1] = ell_grid[0][1];
        ell_grid[2][2] = 0.0;

        ell_grid[3][0] = ell_grid[0][0] - _grid_space[0];
        ell_grid[3][1] = ell_grid[0][1] + _grid_space[1];
        ell_grid[3][2] = 0.0;

        for (int i = 0; i < 4; i++)
        {
            ell2xyz(ell_grid[i], used_crd[i], true);
        }

        return true;
    }
    bool gnss_model_grid::_CalBias_AllSite(map<string, map<string, double>>& augin, map<GSYS, map<FREQ_SEQ, GOBSBAND>> band_index, const map<string, Triple>& siteell)
    {
        /*Step1 Find Ref Site*/
        int maxSat = 0;
        string refSite;
        for (auto iter : augin)
        {
            if (iter.second.size() >= maxSat)
            {
                maxSat = iter.second.size();
                refSite = iter.first;
            }
        }
        if (_RefSite != "NONE")
            refSite = _RefSite;
            
        if (augin.find(refSite) == augin.end())
            return false;
        map<string, double> ref = augin.at(refSite);
        map<string, map<string, double>> omc_all;
        map<GSYS, double> sys_freq;
        for (auto isys : band_index)
        {
            /*Step2 single difference Site with reference site*/
            Matrix B, x, l;
            map<string, map<string, double>> omc;
            int l_size = 0;
            int x_size = 0;
            x.resize(augin.size(), 1);
            for (auto iter : augin)
            {
                int curSite_lsize = 0;
                if (iter.first == refSite)
                    continue;
                for (auto jter : iter.second)
                {
                    if (gnss_sys::sat2gsys(jter.first) != isys.first)
                        continue;
                    if (ref.find(jter.first) == ref.end())
                        continue;
                    l_size++;
                    curSite_lsize++;
                    omc[iter.first][jter.first] = -(ref[jter.first] - jter.second);
                    omc_all[iter.first][jter.first] = -(ref[jter.first] - jter.second);
                }//end sat of one site
                if (curSite_lsize != 0)
                    x_size++;
            }//end site
            if (x_size <= 0)
                continue;
            B.resize(l_size + 1, x_size + 1);
            B.setZero();
            l.resize(l_size + 1, 1);
            x.resize(x_size + 1, 1);
            int i = -1;
            int j = -1;
            for (auto iter : omc)
            {
                i++;
                for (auto jter : iter.second)
                {
                    j++;
                    B(j, i) = 1;
                    B(j, x_size) = -1;
                    l(j, 0) = jter.second;
                }
            }
            /*Step3 add benchmark*/
            for (i = 0; i < x_size + 1; i++)
            {
                B(l_size, i) = 1;
            }
            //B(l_size, x_size) = 1;
            l(l_size, 0) = 0.0;
#ifdef DEBUG
            std::cout << "IonoGrid bias DEBUG:" << endl;
            std::cout << "B:" << endl;
            std::cout << B << endl;
            std::cout << "L:" << endl;
            std::cout << l << endl;
#endif // DEBUG
            /*Step4 lsq*/

            x = (B.transpose() * B).inverse() * (B.transpose() * l);
            /*Step5 save bias*/
            _Bias[_cur_time][isys.first][refSite] = x(x_size - 1, 0);
            i = 0;
            for (auto iter : augin)
            {

                if (iter.first == refSite || omc.find(iter.first) == omc.end())
                    continue;
                _Bias[_cur_time][isys.first][iter.first] = x(i, 0);
                i++;
            }
        }//end sys

        /*Step6 correct bias*/
        set<GSYS> ref_sys_flag;
        map<string, map<string, double>> augused;
        augused = augin;
        for (auto iter : augin)
        {
            if (omc_all.find(iter.first) == omc_all.end() && iter.first != refSite)
            {
                augused.erase(iter.first);
                continue;
            }
            for (auto jter : iter.second)
            {
                if (iter.first != refSite && omc_all[iter.first].find(jter.first) == omc_all[iter.first].end())
                {
                    augused[iter.first].erase(jter.first);
                    continue;
                }
                GSYS cur_sys = gnss_sys::sat2gsys(jter.first);
                if (_Bias[_cur_time].find(cur_sys) == _Bias[_cur_time].end())
                {
                    continue;
                }
                gnss_data_sats obs_sat;
                obs_sat.sat(jter.first);
                if (band_index.find(cur_sys) != band_index.end())
                {

                    double fre = obs_sat.frequency(band_index[cur_sys][FREQ_1]);
                    double fac = fre * fre / (40.3e16);
                    augused[iter.first][jter.first] = (augused[iter.first][jter.first] - _Bias[_cur_time][gnss_sys::sat2gsys(jter.first)][iter.first]) * fac;
                }
                else
                {
                    augused[iter.first].erase(jter.first);
                }
            }
        }
        augin = augused;
        map<GSYS, map<string, double>> cur_Bias;
        cur_Bias = _Bias[_cur_time];
        for (auto iter : cur_Bias)
        {
            for (auto jter : iter.second)
            {
                if (jter.first == refSite)
                {
                    double temp = _Bias[_cur_time][iter.first][jter.first];
                    _Bias[_cur_time][iter.first].erase(jter.first);
                    _Bias[_cur_time][iter.first][jter.first + "*"] = temp;
                    break;
                }
            }
        }

        return true;
    }
    bool gnss_model_grid::_CalBias_MINSite(map<string, map<string, double>>& augin, map<GSYS, map<FREQ_SEQ, GOBSBAND>> band_index, const map<string, Triple>& siteell)
    {
        /*Step1 Find Ref Site*/
        int maxSat = 0;
        string refSite;
        for (auto iter : augin)
        {
            if (iter.second.size() >= maxSat)
            {
                maxSat = iter.second.size();
                refSite = iter.first;
            }
        }
        if (_RefSite != "NONE")
            refSite = _RefSite;

        if (augin.find(refSite) == augin.end())
            return false;
        vector<map<string, double>> refdata;
        vector<string> refSites;
        map<string, map<string, double>> omc_all;
        map<GSYS, double> sys_freq;
        map<string, int> Site_index;
        map<string, double> SiteSite_dis;
        /*Sort the Site and calculate the distance*/
        int index = 1;
        for (auto iter = siteell.begin();iter != siteell.end();iter++)
        {
            if (Site_index.find(iter->first) != Site_index.end())
                continue;
            Site_index[iter->first] = index;
            index++;
            /*if (_Site_Site_dis.find(iter->first) != _Site_Site_dis.end())
                continue;*/
            for (auto jter = siteell.begin(); jter != siteell.end(); jter++)
            {
                if (iter->first == jter->first)
                    continue;
                if (_Site_Site_dis.find(jter->first) != _Site_Site_dis.end())
                {
                    if (_Site_Site_dis[jter->first].find(iter->first) != _Site_Site_dis[jter->first].end())
                        continue;
                }
                if (_Site_Site_dis[iter->first].find(jter->first) == _Site_Site_dis[iter->first].end())
                {
                    Triple Site1((iter->second)[0], (iter->second)[1], 0);
                    Triple Site2((jter->second)[0], (jter->second)[1], 0);
                    _Site_Site_dis[iter->first][jter->first] = (Site1 - Site2).norm();
                }
            }
        }

        for (auto isys : band_index)
        {
            
            if (isys.first == GSYS::BDS || isys.first == GSYS::GAL || isys.first == GSYS::GPS)
                cout << "current system: " << gnss_sys::gsys2str(isys.first) << endl;
            else
                continue;
            /*Step2 single difference Site with reference site*/
            Matrix B, x, l;
            map<string, map<string, double>> omc;
            map<string, map<string, double>> omc_sys;
            omc_sys.clear();
            omc.clear();
            map<double, string> dis_site;
            int l_size = 0;
            int x_size = 0;
            x.resize(augin.size(), 1);
            vector<string> Bias_Site;
            Bias_Site.clear();
            int Sys_rank_deficient = 0;
            map<string, int> equ_equnum;
            for (auto iter : augin)
            {
                int curSite_lsize = 0;
                dis_site.clear();
                refdata.clear();
                refSites.clear();
                string curSite = iter.first;
                for (auto jter : augin)
                {
                    if (iter.first == jter.first)
                        continue;
                    if (_Site_Site_dis.find(iter.first) != _Site_Site_dis.end())
                    {
                        if (_Site_Site_dis[iter.first].find(jter.first) != _Site_Site_dis[iter.first].end())
                        {
                            dis_site[_Site_Site_dis[iter.first][jter.first]] = jter.first;
                            continue;
                        }
                    }
                    else
                    {
                        if (_Site_Site_dis.find(jter.first) != _Site_Site_dis.end())
                        {
                            if (_Site_Site_dis[jter.first].find(iter.first) != _Site_Site_dis[jter.first].end())
                            {
                                dis_site[_Site_Site_dis[jter.first][iter.first]] = jter.first;
                                continue;
                            }
                        }
                    }
                }//find and sort distance

                for (auto onedis : dis_site)
                {
                    if (onedis.first >= _maxBias_baseline)
                        break;
                    if (augin.find(onedis.second) == augin.end())
                        continue;
                    refdata.push_back(augin[onedis.second]);
                    refSites.push_back(onedis.second);
                    //break;
                }//chose the ref based on the dis
                map<string, pair<double,int>> sum_num;
                sum_num.clear();
                map<string, int> num_equ;
                num_equ.clear();
                for (auto jter : iter.second)
                {         
                    if (refSites.size() == 0)
                        break;
                    if (gnss_sys::sat2gsys(jter.first) != isys.first)
                        continue;
                    map<string, double> one_ref;
                    bool ref_sat = true;
                    for (auto one : refdata)
                    {
                        if (one.find(jter.first) != one.end())
                        {
                            refSite = refSites[find(refdata.begin(), refdata.end(), one) - refdata.begin()];
                            one_ref = one;
                            ref_sat = true;
                            break;
                        }
                        ref_sat = false;
                    }
                    if (!ref_sat)
                        continue;
                    string equ1 = curSite + "-" + refSite;
                    string equ2 = refSite + "-" + curSite;
                    if (omc.find(equ1) != omc.end())
                    {
                        if (omc[equ1].find(jter.first) != omc[equ1].end())
                            continue;
                    }
                    if (omc.find(equ2) != omc.end())
                    {
                        if (omc[equ2].find(jter.first) != omc[equ2].end())
                            continue;
                    }
                    
                    /*if (find(Bias_Site.begin(),Bias_Site.end(),curSite)==Bias_Site.end())
                        Bias_Site.push_back(curSite);
                    if (find(Bias_Site.begin(), Bias_Site.end(), refSite) == Bias_Site.end())
                        Bias_Site.push_back(refSite);*/
                    l_size++;
                    curSite_lsize++;
                    omc[equ1][jter.first] = -(one_ref)[jter.first] + jter.second;
                    equ_equnum[equ1] = 0 ;
                    if (sum_num.find(equ1) == sum_num.end())
                    {
                        sum_num[equ1].first = 0.0;
                        sum_num[equ1].second = 0;
                    }
                    sum_num[equ1].first = sum_num[equ1].first -(one_ref)[jter.first] + jter.second;
                    sum_num[equ1].second = sum_num[equ1].second + 1;
                    
                    /*omc_sys[curSite][jter.first] = -(one_ref)[jter.first] + jter.second;
                    omc_sys[refSite][jter.first] = -(one_ref)[jter.first] + jter.second;*/
                }//end sat of one site
                /*Edit the omc*/
                map<string, map<string, double>> omc_temp = omc;
                if (curSite_lsize >= 1)
                {
                    for (auto one_equ : sum_num)
                    {                        
                        string equ = one_equ.first;
                        equ.replace(equ.find("-"), 1, " ");
                        string Sitei, Sitej;
                        istringstream temp(equ);
                        temp >> Sitei >> Sitej;
                        equ = one_equ.first;
                        double ave = one_equ.second.first / one_equ.second.second * 1.0;
                        bool isUsed = true;
                        bool isDelete = true;
                        int New_Site = 0;
                        if (find(Bias_Site.begin(), Bias_Site.end(), Sitei) == Bias_Site.end())
                            New_Site++;
                        if (find(Bias_Site.begin(), Bias_Site.end(), Sitej) == Bias_Site.end())
                            New_Site++;
                        /*if (New_Site == 2 && omc_sys.size() != 0)
                        {
                            Sys_rank_deficient++;
                        }
                        if (New_Site == 0 && Sys_rank_deficient >= 1)
                        {
                            Sys_rank_deficient--;
                            isDelete = false;
                        }*/
                        for (auto sat_omc : omc_temp[equ])
                        {
                            isUsed = true;
                            if (one_equ.second.second>1)
                            {
                                cout << "ave-omc:" << gnss_sys::gsys2str(isys.first) <<"   "
                                << _cur_time.str_ymdhms() << "    " << sat_omc.first << "   "
                                <<setw(10)<<setprecision(5)<< ave - sat_omc.second << endl;
                            }
                            if (fabs(ave - sat_omc.second) >= _maxBias_sigma || one_equ.second.second<=1)
                            {
                                
                                if (isDelete)
                                {
                                    omc[equ].erase(sat_omc.first);
                                    curSite_lsize--;
                                    l_size--;
                                    isUsed = false;
                                }
                                
                            }
                            if(isUsed)
                            {
                                omc_all[Sitei][sat_omc.first] = sat_omc.second;
                                omc_all[Sitej][sat_omc.first] = sat_omc.second;
                                omc_sys[Sitei][sat_omc.first] = sat_omc.second;
                                omc_sys[Sitej][sat_omc.first] = sat_omc.second;
                                if (find(Bias_Site.begin(), Bias_Site.end(), Sitei) == Bias_Site.end())
                                    Bias_Site.push_back(Sitei);
                                if (find(Bias_Site.begin(), Bias_Site.end(), Sitej) == Bias_Site.end())
                                    Bias_Site.push_back(Sitej);
                            }
                            
                        }
                        if (omc[equ].size() == 0)
                        {
                            omc.erase(equ);
                            equ_equnum.erase(equ);
                            if (New_Site == 2 && omc_sys.size() != 0)
                            {
                                Sys_rank_deficient--;
                            }
                        }
                        /*if (omc_sys[Sitei].size() == 0)
                        {
                            omc_sys.erase(Sitei);
                            Bias_Site.erase(find(Bias_Site.begin(), Bias_Site.end(), Sitei));
                        }
                        if (omc_sys[Sitej].size() == 0)
                        {
                            omc_sys.erase(Sitej);
                            Bias_Site.erase(find(Bias_Site.begin(), Bias_Site.end(), Sitej));
                        }*/
                    }
                    
                }
                if (curSite_lsize != 0)
                {
                        x_size++;
                }
                    
            }//end site

            /*Step n spetial rank deficient*/
            Sys_rank_deficient = 0;
            map<string, map<string, double>> omc_temp;
            int i = 0;
            int j = 0;
            int l_omc = 0;
            Matrix_T<float> B_equ(omc_temp.size(), Site_index.size());
            B_equ.fill(0);
            double rank = 0;
            if (Sys_rank_deficient < 1)
            {
                bool rank_sys = false;
                while (1)
                {
                    omc_temp = omc;
                    if (omc.size() <= 1)
                    {
                        rank_sys = false;
                        break;
                    }
                    B_equ.resize(omc_temp.size(), Site_index.size());
                    B_equ.fill(0);
                    l_omc = 0;
                    for (auto iter : omc_temp)
                    {
                        string equ = iter.first;
                        cout << "EQU: " << equ << endl;
                        equ.replace(equ.find("-"), 1, " ");
                        string Sitei, Sitej;
                        istringstream temp(equ);
                        temp >> Sitei >> Sitej;
                        i = Site_index[Sitei];
                        j = Site_index[Sitej];
                        for (auto site_num : equ_equnum)
                        {
                            if (site_num.first.find(Sitei) != string::npos)
                                equ_equnum[site_num.first] = equ_equnum[site_num.first] + 1;
                            if (site_num.first.find(Sitej) != string::npos)
                                equ_equnum[site_num.first] = equ_equnum[site_num.first] + 1;
                        }
                        cout << i << " " << j << endl;
                        if (i <= 0 || j <= 0)
                            return false;
                        l_omc++;
                        cout << l_omc << endl;
                        B_equ(l_omc - 1, i - 1) = 1;
                        B_equ(l_omc - 1, j - 1) = -1;
                    }
                    Vector_T<float> col_sum = B_equ.cwiseAbs().colwise().sum();

                    int num_of_0 = 0;
                    for (i = 0; i < Site_index.size(); i++)
                    {
                        if (double_eq(col_sum(i), 0))
                            num_of_0++;
                    }

                    Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(B_equ);
                    std::cout << gnss_sys::gsys2str(isys.first) << " rank B_equ:" << endl;
                    std::cout << B_equ << endl;
                    std::cout << B_equ.cwiseAbs() << endl;
                    std::cout << lu_decomp.rank() << endl;
                    if (lu_decomp.rank() + 1 + num_of_0 >= Site_index.size())
                    {
                        rank_sys = true;
                        break;
                    }
                    if (lu_decomp.rank() + 1 + num_of_0 < Site_index.size())
                    {
                        cout << gnss_sys::gsys2str(isys.first) << ":" << "RANK DEFICIENT 2!" << endl;
                        int min_num = 99999;
                        string min_equ = "";
                        for (auto iter : equ_equnum)
                        {
                            if (iter.second < min_num)
                            {
                                min_equ = iter.first;
                                min_num = iter.second;
                            }
                            equ_equnum[iter.first] = 0;
                        }
                        omc.erase(min_equ);
                        equ_equnum.erase(min_equ);
                    }
                }
                if (!rank_sys)
                {
                    cout << gnss_sys::gsys2str(isys.first) << ":" << "RANK DEFICIENT 3!" << endl;
                    continue;
                }
            }

            /*if (Sys_rank_deficient >= 1)
            {
                cout<< gnss_sys::gsys2str(isys.first)<<":" << "RANK DEFICIENT 1!" << endl;
                continue;
            }*/
                
            x.resize(Site_index.size(), 1);
            /**/
            Bias_Site.clear();
            l_size = 0;
            for (auto iter : omc)
            {
                string equ = iter.first;
                equ.replace(equ.find("-"), 1, " ");
                string Sitei, Sitej;
                istringstream temp(equ);
                temp >> Sitei >> Sitej;
                //cout << Sitei << ", " << Sitej << endl;
                if (find(Bias_Site.begin(), Bias_Site.end(), Sitei) == Bias_Site.end())
                    Bias_Site.push_back(Sitei);
                if (find(Bias_Site.begin(), Bias_Site.end(), Sitej) == Bias_Site.end())
                    Bias_Site.push_back(Sitej);
                l_size = l_size + iter.second.size();
            }
            
            
            
            if (Bias_Site.size() >= Site_index.size())
            {
                B.resize(l_size + 1, Site_index.size());
                B.setZero();
                l.resize(l_size + 1, 1);
                l.setZero();
                
            }
            else
            {
                B.resize(l_size + 1 + Site_index.size()-Bias_Site.size(), Site_index.size());
                B.setZero();
                l.resize(l_size + 1 + Site_index.size() - Bias_Site.size(), 1);
                l.setZero();
            }
            
            i = 0;
            j = 0;
            l_omc = 0;
            for (auto iter : omc)
            {                
                string equ = iter.first;
                cout <<"EQU: " << equ << endl;
                equ.replace(equ.find("-"), 1, " ");
                string Sitei, Sitej;
                istringstream temp(equ);
                temp >> Sitei >> Sitej;
                i = Site_index[Sitei];
                j = Site_index[Sitej];
                cout << i << " " << j << endl;
                if (i <= 0 || j <= 0)
                    return false;
                for (auto jter : iter.second)
                {
                    l_omc++;
                    cout << l_omc << endl;
                    B(l_omc - 1, i - 1) = 1;
                    B(l_omc - 1, j - 1) = -1;
                    l(l_omc - 1, 0) = jter.second;
                }
            }

            /*Constraining stations without observations to prevent rank deficit*/
            if (Bias_Site.size() < Site_index.size())
            {
                for (int B_col = 0; B_col < B.cols(); B_col++)
                {
                    Vector sum_col;
                    sum_col=B.col(B_col);
                    if (sum_col.isZero())
                    {
                        l_omc++;
                        B(l_omc - 1, B_col) = 1;
                        l(l_omc - 1, 0) = 0;
                    }
                }
            }
            /*Step3 add benchmark only to station with observation*/
            l_omc++;
            for (auto iter : omc)
            {
                string equ = iter.first;
                equ.replace(equ.find("-"), 1, " ");
                string Sitei, Sitej;
                istringstream temp(equ);
                temp >> Sitei >> Sitej;
                i = Site_index[Sitei];
                j = Site_index[Sitej];
                if (i <= 0 || j <= 0)
                    return false;
                for (auto jter : iter.second)
                {                    
                    B(l_omc - 1, i - 1) = 1;
                    B(l_omc - 1, j - 1) = 1;
                    l(l_omc - 1, 0) = 0;
                }
            }
            
            //B(l_size, x_size) = 1;
            l(l_size, 0) = 0.0;
            
#ifdef DEBUG
            std::cout << "IonoGrid bias DEBUG:" << endl;
            std::cout << "B:" << endl;
            std::cout << B << endl;
            std::cout << "L:" << endl;
            std::cout << l << endl;
#endif // DEBUG
            /*Step4 lsq*/
            
            if ((omc.size() + 1 - Bias_Site.size()) < 0)
            {
                cerr << "warning" << endl;
            }
            x = (B.transpose() * B).inverse() * (B.transpose() * l);
            /*Step5 save bias*/
            //_Bias[_cur_time][isys.first][refSite] = x(x_size, 1);
            i = 0;
            for (auto iter : Site_index)
            {
                i++;
                if (double_eq(x(i - 1, 0),0.0))
                    continue;
                _Bias[_cur_time][isys.first][iter.first] = x(i - 1, 0);
            }
            Bias_Site.clear();
        }//end sys

        /*Step6 correct bias*/
        set<GSYS> ref_sys_flag;
        map<string, map<string, double>> augused;
        augused = augin;
        for (auto iter : augin)
        {
            if (omc_all.find(iter.first) == omc_all.end())
            {
                augused.erase(iter.first);
                continue;
            }
            for (auto jter : iter.second)
            {
                if (omc_all[iter.first].find(jter.first) == omc_all[iter.first].end())
                {
                    augused[iter.first].erase(jter.first);
                    continue;
                }
                GSYS cur_sys = gnss_sys::sat2gsys(jter.first);
                if (_Bias[_cur_time].find(cur_sys) == _Bias[_cur_time].end())
                {
                    augused[iter.first].erase(jter.first);
                    if (augused[iter.first].size() == 0)
                    {
                        augused.erase(iter.first);
                        break;
                    }
                    continue;
                }
                if (_Bias[_cur_time][cur_sys].find(iter.first) == _Bias[_cur_time][cur_sys].end())
                {
                    augused[iter.first].erase(jter.first);
                    cout << "No Site: " << iter.first << endl;
                    if (augused[iter.first].size() == 0)
                    {
                        augused.erase(iter.first);
                        break;
                    }
                    continue;
                }
                gnss_data_sats obs_sat;
                obs_sat.sat(jter.first);
                if (band_index.find(cur_sys) != band_index.end())
                {
                    double fre = obs_sat.frequency(band_index[cur_sys][FREQ_1]);
                    double fac = fre * fre / (40.3e16);
                    augused[iter.first][jter.first] = (augused[iter.first][jter.first] - _Bias[_cur_time][gnss_sys::sat2gsys(jter.first)][iter.first]) * fac;

                }
                else
                {
                    augused[iter.first].erase(jter.first);
                }
            }
        }
        augin = augused;
        map<GSYS, map<string, double>> cur_Bias;
        cur_Bias = _Bias[_cur_time];
        /*for (auto iter : cur_Bias)
        {
            for (auto jter : iter.second)
            {
                if (jter.first == refSite)
                {
                    double temp = _Bias[_cur_time][iter.first][jter.first];
                    _Bias[_cur_time][iter.first].erase(jter.first);
                    _Bias[_cur_time][iter.first][jter.first + "*"] = temp;
                    break;
                }
            }
        }*/

        return true;
    }
    bool gnss_model_grid::_CalBias_2Site(map<string, map<string, double>>& augin, map<GSYS, map<FREQ_SEQ, GOBSBAND>> band_index, const map<string, Triple>& siteell)
    {
        /*Step1 Find Ref Site*/
        int maxSat = 0;
        string refSite;
        for (auto iter : augin)
        {
            if (iter.second.size() >= maxSat)
            {
                maxSat = iter.second.size();
                refSite = iter.first;
            }
        }
        if (_RefSite == "NONE")
            refSite = _RefSite;
        
        if (augin.find(refSite) == augin.end())
            return false;
        map<string, double> ref = augin.at(refSite);
        map<string, map<string, double>> omc_all;
        map<GSYS, double> sys_freq;
        set<string> sat_list;
        double ave = 0.0;
        double sum = 0.0;
        for (auto isys : band_index)
        {
            /*Step2 single difference Site with reference site*/
            Matrix B, x, l;
            map<string, map<string, double>> omc;
            int l_size = 0;
            int x_size = 0;
            x.resize(augin.size(), 1);
            for (auto iter : augin)
            {
                int curSite_lsize = 0;
                sum = 0.0;
                if (iter.first == refSite)
                    continue;
                for (auto jter : iter.second)
                {
                    if (gnss_sys::sat2gsys(jter.first) != isys.first)
                        continue;
                    if (ref.find(jter.first) == ref.end())
                    {
                        //augin.at(iter.first).erase(jter.first);
                        continue;
                    }
                    l_size++;
                    curSite_lsize++;
                    omc[iter.first][jter.first] = ref[jter.first] - jter.second;
                    sum = sum + omc[iter.first][jter.first];
                    omc_all[iter.first][jter.first] = ref[jter.first] - jter.second;
                }//end sat of one site
                if (curSite_lsize >= 2)
                {
                    x_size++;
                    ave = sum / curSite_lsize;
                    sum = 0.0;
                    curSite_lsize = 0;

                    sat_list.clear();
                    for (auto jter : omc[iter.first])
                    {
                        if (fabs(jter.second - ave) > 0.06)
                        {
                            omc_all[iter.first].erase(jter.first);
                            continue;
                        }
                        else
                        {
                            curSite_lsize++;
                            sat_list.insert(jter.first);
                            sum = sum + omc_all[iter.first][jter.first];
                        }
                    }
                }
                else
                    continue;

                if (curSite_lsize >= 2)
                {
                    ave = sum / curSite_lsize;
                    Vector P(curSite_lsize), X(curSite_lsize);
                    double W = 0.0;
                    int iPX = 0;
                    for (auto jter : sat_list)
                    {
                        P(iPX) = 1 / ((ave - omc_all[iter.first][jter]) * (ave - omc_all[iter.first][jter]));
                        W = W + P(iPX);
                        X(iPX) = omc_all[iter.first][jter];
                        iPX++;
                    }
                    P = P / W;
                    _Bias[_cur_time][isys.first][iter.first] = (P.transpose() * X).sum();
                }

            }//end site
#ifdef DEBUG
            /*std::cout << "IonoGrid bias DEBUG:" << endl;
            std::cout << "B:" << endl;
            std::cout << B << endl;
            std::cout << "L:" << endl;
            std::cout << l << endl;*/
#endif // DEBUG
            _Bias[_cur_time][isys.first][refSite] = 0.0;
        }//end sys

        /*Step6 correct bias*/
        set<GSYS> ref_sys_flag;
        map<string, map<string, double>> augused;
        augused = augin;
        for (auto iter : augin)
        {
            if (omc_all.find(iter.first) == omc_all.end() && iter.first != refSite)
            {
                augused.erase(iter.first);
                continue;
            }
            for (auto jter : iter.second)
            {
                if (iter.first != refSite && omc_all[iter.first].find(jter.first) == omc_all[iter.first].end())
                {
                    augused[iter.first].erase(jter.first);
                    continue;
                }
                GSYS cur_sys = gnss_sys::sat2gsys(jter.first);
                if (_Bias[_cur_time].find(cur_sys) == _Bias[_cur_time].end())
                {
                    continue;
                }
                if (_Bias[_cur_time][cur_sys].find(iter.first) == _Bias[_cur_time][cur_sys].end())
                {
                    augused[iter.first].erase(jter.first);
                    continue;
                }
                gnss_data_sats obs_sat;
                obs_sat.sat(jter.first);
                if (band_index.find(cur_sys) != band_index.end())
                {
                    double fre = obs_sat.frequency(band_index[cur_sys][FREQ_1]);
                    double fac = fre * fre / (40.3e16);
                    augused[iter.first][jter.first] = (augused[iter.first][jter.first] - _Bias[_cur_time][gnss_sys::sat2gsys(jter.first)][iter.first]) * fac;
                }
                else
                {
                    augused[iter.first].erase(jter.first);
                }
            }
        }
        augin = augused;
        map<GSYS, map<string, double>> cur_Bias;
        cur_Bias = _Bias[_cur_time];
        for (auto iter : cur_Bias)
        {
            for (auto jter : iter.second)
            {
                if (jter.first == refSite)
                {
                    double temp = _Bias[_cur_time][iter.first][jter.first];
                    _Bias[_cur_time][iter.first].erase(jter.first);
                    _Bias[_cur_time][iter.first][jter.first + "*"] = temp;
                    break;
                }
            }
        }

        return true;
    }
    bool gnss_model_grid::_CalCoefRes(const map<string, Triple>& siteell, bool isTrop)
    {
        if (!isTrop)
        {
            gnss_data_augion  ion;
            for (auto iter : _STECValSite)
            {
                map<string, int> site_index;
                site_index.clear();
                if (iter.second.size() < _min_site)
                    continue;
                int valSite = iter.second.size();
                /*confirm the equtation type*/
                Matrix B, L(valSite, 1), x;
                if (valSite > 3)
                {
                    ion.equ_type = 4;
                    B.resize(valSite, 4);
                    x.resize(4, 1);
                }
                else
                {
                    ion.equ_type = 3;
                    B.resize(valSite, 3);
                    x.resize(3, 1);
                }
                bool init = true;
                string crt_sat = iter.first;
                int k = 0;
                for (auto onesite : iter.second)
                {
                    double crt_Lat = 0.0;
                    double crt_Lon = 0.0;
                    if (siteell.find(onesite.first) == siteell.end())
                    {
                        continue;
                    }
                    else
                    {
                        crt_Lat = (siteell.find(onesite.first))->second[0];
                        crt_Lon = (siteell.find(onesite.first))->second[1];
                    }

                    if (init)
                    {
                        ion.satSTEC[crt_sat].valid = 1;
                        ion.satSTEC[crt_sat].csys = gnss_sys::str2gsys(crt_sat.substr(0, 1));
                        ion.satSTEC[crt_sat].prn = crt_sat;
                        ion.nsat++;
                        init = false;
                    }
                    B(k, 0) = 1;
                    B(k, 1) = crt_Lat - _center_BLH[0];
                    B(k, 2) = crt_Lon - _center_BLH[1];
                    if (B.rows() > 3)
                    {
                        B(k, 3) = (crt_Lat - _center_BLH[0]) * (crt_Lon - _center_BLH[1]);
                    }
                    L(k, 0) = onesite.second;
                    k++;
                    site_index[onesite.first] = k;
                }
                x = (B.transpose() * B).inverse() * (B.transpose() * L);
                for (int p = 0; p < B.cols(); p++)
                {
                    ion.satSTEC[crt_sat].C[p] = x(p + 1, 1);
                }
                /*calculate grid residuals*/
                Vector RES;
                RES = B * x - L;       /*calculate residuals in reference station*/
                for (int index = 0; index < _grid_count[0] * _grid_count[1]; index++)
                {
                    //Vector B1(valSite), X(valSite);
                    Vector B1(3), X(3);
                    Vector RES_3(3);
                    Triple site_crd(0, 0, 0);
                    Triple grid_ell(0, 0, 0);
                    Triple grid_crd(_grid_pos[index]);
                    xyz2ell2(grid_crd, grid_ell, true);
                    //double grid_ele = _getSatele(crt_sat, grid_crd);
                    grid_crd[2] = 0.0;
                    double P = 0;
                    int k = 0;
                    map<string, double> site_dis;
                    if (!_findSite(grid_ell, crt_sat, siteell, site_dis))
                    {
                        ion.satSTEC[crt_sat].valid = 0;
                        break;
                    }
                    for (auto onesite : site_dis)
                    {
                        //ell2xyz((siteell.find(onesite.first))->second, site_crd, true);
                        //double site_ele = _getSatele(crt_sat, site_crd);
                        //site_crd[2] = 0.0;
                        // weight by distance
                        //B1(k + 1) = 1 / fabs(site_ele - grid_ele);
                        int index_old = site_index.at(onesite.first);
                        B1(k) = 1 / onesite.second;
                        P += B1(k);
                        k++;
                        RES_3(k) = RES(index_old);
                    }
                    X = B1 / P;
                    ion.satSTEC[crt_sat].res[index] = (X.transpose() * RES_3).sum();
                }
            } // end all sat
            _pushOneGrid(ion);
            return true;
        }
        else
        {
            gnss_data_augtrop trop;
            for (auto iter : _TropValSite)
            {
                if (iter.second.size() < 3)
                    continue;
                int valSite = iter.second.size();
                /*confirm the equtation type*/
                Matrix B, L(valSite, 1), x;
                if (valSite > 3)
                {
                    trop.equ_type = 4;
                    B.resize(valSite, 4);
                    x.resize(4, 1);
                }
                else
                {
                    trop.equ_type = 3;
                    B.resize(valSite, 3);
                    x.resize(3, 1);
                }
                bool init = true;
                string crt_sat = iter.first;
                int k = 0;
                for (auto onesite : iter.second)
                {
                    double crt_Lat = 0.0;
                    double crt_Lon = 0.0;
                    if (siteell.find(onesite.first) == siteell.end())
                    {
                        continue;
                    }
                    else
                    {
                        crt_Lat = (siteell.find(onesite.first))->second[0];
                        crt_Lon = (siteell.find(onesite.first))->second[1];
                    }

                    if (init)
                    {
                        trop.satTrop[crt_sat].valid = 1;
                        trop.satTrop[crt_sat].csys = gnss_sys::str2gsys(crt_sat.substr(0, 1));
                        trop.satTrop[crt_sat].prn = crt_sat;
                        trop.nsat++;
                        init = false;
                    }
                    B(k, 0) = 1;
                    B(k, 1) = crt_Lat - _center_BLH[0];
                    B(k, 2) = crt_Lon - _center_BLH[1];
                    if (B.rows() > 3)
                    {
                        B(k, 3) = (crt_Lat - _center_BLH[0]) * (crt_Lon - _center_BLH[1]);
                    }
                    L(k, 0) = onesite.second;
                    k++;
                }
                x = (B.transpose() * B).inverse() * (B.transpose() * L);
                for (int p = 0; p < B.cols(); p++)
                {
                    trop.satTrop[crt_sat].T[p] = x(p, 0);
                }

                Vector RES;
                RES = B * x - L;       /*calculate residuals in reference station*/
                for (int index = 0; index < _grid_count[0] * _grid_count[1]; index++)
                {
                    Vector B1(valSite), X(valSite);
                    Triple site_crd(0, 0, 0);
                    Triple grid_crd(_grid_pos[index]);
                    grid_crd[2] = 0.0;
                    double P = 0;
                    int k = 0;
                    for (auto onesite : iter.second)
                    {
                        ell2xyz((siteell.find(onesite.first))->second, site_crd, true);
                        site_crd[2] = 0.0;
                        B1(k) = 1 / (grid_crd - site_crd).norm();
                        P += B1(k);
                        k++;
                    }
                    X = B1 / P;
                    trop.satTrop[crt_sat].res[index] = (X.transpose() * RES).sum();
                }
            }
            _pushOneGrid(trop);
            return true;
        }
    }
    bool gnss_model_grid::_CalCoefRes(const map<string, Triple>& siteell, const map<string, double>& Tropin)
    {
        gnss_data_augtrop trop;
        if (Tropin.size() < _min_site)
            return false;
        _isZWD = true;
        int valSite = Tropin.size();
        /*confirm the equtation type*/
        Matrix B, L(valSite, 1), x;
        if (valSite > 3)
        {
            trop.equ_type = 4;
            B.resize(valSite, 4);
            x.resize(4, 1);
        }
        else
        {
            trop.equ_type = 3;
            B.resize(valSite, 3);
            x.resize(3, 1);
        }
        bool init = true;
        int k = 0;
        for (auto onesite : Tropin)
        {
            double crt_Lat = 0.0;
            double crt_Lon = 0.0;
            if (siteell.find(onesite.first) == siteell.end())
            {
                continue;
            }
            else
            {
                crt_Lat = (siteell.find(onesite.first))->second[0];
                crt_Lon = (siteell.find(onesite.first))->second[1];
            }

            if (init)
            {
                trop.siteZWD.valid = 1;
                init = false;
            }
            B(k, 0) = 1;
            B(k, 1) = crt_Lat - _center_BLH[0];
            B(k, 2) = crt_Lon - _center_BLH[1];
            if (B.rows() > 3)
            {
                B(k, 3) = (crt_Lat - _center_BLH[0]) * (crt_Lon - _center_BLH[1]);
            }
            L(k, 0) = onesite.second;
            k++;
        }
        x = (B.transpose() * B).inverse() * (B.transpose() * L);
        for (int p = 0; p < B.cols(); p++)
        {
            trop.siteZWD.T[p] = x(p, 0);
        }

        Vector RES;
        RES = B * x - L;       /*calculate residuals in reference station*/
        for (int index = 0; index < _grid_count[0] * _grid_count[1]; index++)
        {
            Vector B1(valSite), X(valSite);
            Triple site_crd(0, 0, 0);
            Triple grid_crd(_grid_pos[index]);
            Triple grid_ell(0, 0, 0);
            xyz2ell2(_grid_pos[index], grid_ell, true);
            grid_crd[2] = 0.0;
            double P = 0;
            int k = 0;
            for (auto onesite : Tropin)
            {
                ell2xyz((siteell.find(onesite.first))->second, site_crd, true);
                Triple site_ell = siteell.at(onesite.first);
                site_ell[2] = 0.0;
                B1(k) = 1 / (grid_ell - site_ell).norm();
                P += B1(k);
                k++;
            }
            X = B1 / P;
            trop.siteZWD.res[index] = (X.transpose() * RES).sum();
        }
        _pushOneGrid(trop);
    }
    bool gnss_model_grid::_findSite(Triple& grid_ell, const string& sat, const map<string, Triple>& sitesell, map<string, double>& site_dis)
    {
        site_dis.clear();
        map <string, double> site_stec;
        site_stec = _STECValSite[sat];
        map<double, string> dis_site;
        set<string> temp_site;
        //temp_site.insert("HKST"); temp_site.insert("HKPC"); temp_site.insert("HKOH");
        for (auto iter : site_stec)
        {
            if (sitesell.find(iter.first) == sitesell.end())
                continue;
            /*if (sitesell.find(iter.first) == sitesell.end() || temp_site.find(iter.first) == temp_site.end())
                continue;*/
            Triple siteell = sitesell.at(iter.first);
            siteell[2] = 0.0;
            double dis = (grid_ell - siteell).norm();
            dis_site[dis] = iter.first;
        }
        int i = 0;
        if (dis_site.size() < 3)
            return false;
        else
        {
            for (auto jter : dis_site)
            {
                i++;
                site_dis[jter.second] = jter.first;
                if (i == 3)
                    return true;

            }
        }
    }
    void gnss_model_grid::_pushOneGrid(gnss_data_augion& ion)
    {
        _grid_data->addGridData(_cur_time, ion);
    }
    void gnss_model_grid::_pushOneGrid(gnss_data_augtrop& trop)
    {
        _grid_data->addGridData(_cur_time, trop);
    }
    double gnss_model_grid::_getSatele(const string& sat, Triple site_crd)
    {
        double ele = 0.0;
        for (auto iter : _obs_used)
        {
            if (iter.sat() == sat)
            {
                iter.cmpVal(site_crd);
                ele = iter.ele();
            }
            else
                continue;
        }
        return ele;
    }
}
