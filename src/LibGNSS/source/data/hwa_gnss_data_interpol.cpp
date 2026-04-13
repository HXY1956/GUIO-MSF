#include "hwa_gnss_data_interpol.h"
#include "hwa_gnss_all_obs.h"
#include "hwa_set_rec.h"
#include "hwa_set_gbase.h"
#include "hwa_gnss_Delaunay.h"
#include "hwa_gnss_proc_sppflt.h"
#include "hwa_gnss_prod_clk.h"
#include "hwa_gnss_model_precisebias.h"
#include "hwa_gnss_model_gmf.h"
#include "hwa_gnss_model_precisebiasgpp.h"
#include "hwa_gnss_data_rec.h"

using namespace std;

hwa_gnss::gnss_data_interpol::gnss_data_interpol(hwa_set::set_base *set,  base_all_proc *data, base_log spdlog) : _set(set),
                                                                                  _alldata(data),
                                                                                  _spdlog(spdlog),
                                                                                  _isBase(false),
                                                                                  _isCompAug(true),
                                                                                  _isDelaunay(false)
{
    if (dynamic_cast<set_gen *>(_set)->list_base().size())
        _isBase = true;
    _allaug = dynamic_cast<gnss_data_aug *>((*_alldata)[base_data::AUG]);
    _allobj = dynamic_cast<gnss_all_obj *>((*_alldata)[base_data::ALLOBJ]);
    _site = dynamic_cast<set_gen *>(_set)->list_rover()[0];
    _simulation = dynamic_cast<set_gproc *>(_set)->simulation();
    base_time begt = dynamic_cast<set_gen *>(_set)->beg();
    base_time endt = dynamic_cast<set_gen *>(_set)->end();

    _isDelaunay = dynamic_cast<set_npp *>(_set)->npp_delaunay();
    _isCompAug = dynamic_cast<set_npp *>(_set)->comp_aug();
    _isGridAug = dynamic_cast<set_npp*>(_set)->grid_aug();
    _isCorObs = dynamic_cast<set_npp*>(_set)->cor_obs();
    if (_isGridAug)
    {
        _allauggrid = dynamic_cast<gnss_data_gridaug*>((*_alldata)[base_data::AUGGRID]);
        _grid_model = new gnss_model_grid(_allauggrid, spdlog);
        
    }
    _isSelfCorr = dynamic_cast<set_npp *>(_set)->self_cor();
    _obs_level = dynamic_cast<set_npp *>(_set)->obs_level();
    dynamic_cast<set_npp *>(_set)->aug_limit(_comp_aug_limit, _ion_aug_limit, _trop_aug_limit);

    //Step1:set interpolation site list from gset
    _list_ipsite.clear();
    if (_isSelfCorr)
    {
        _list_ipsite.push_back(_site);
    }
    else
    {
        auto sites = dynamic_cast<set_gen *>(_set)->recs();
        for (auto it = sites.begin(); it != sites.end(); it++)
        {
            if (*it == _site)
                continue;
            _list_ipsite.push_back(*it);
            //auto crd = dynamic_cast<set_rec*>(_set)->get_crd_xyz(*it);
        }
    }
    _nipSite = _list_ipsite.size();

    //Setp2:initialize posproc class
    _site_vrs = "";
    if (_isBase)
    {
        _site_vrs = dynamic_cast<set_gen *>(_set)->list_base()[0];
        //add vrs-site obj
        if (_allobj->obj(_site_vrs) == nullptr)
        {
            std::shared_ptr<gnss_data_rec> rec = make_shared<gnss_data_rec>(_spdlog);
            rec->name(_site_vrs);
            rec->id(_site_vrs);
            _allobj->add(rec);
        }
    }
}

hwa_gnss::gnss_data_interpol::gnss_data_interpol(hwa_set::set_base * set,  base_all_proc * data, string &site, base_log spdlog) : _set(set),
                                                                                                  _alldata(data),
                                                                                                  _spdlog(spdlog),
                                                                                                  _isBase(false),
                                                                                                  _isCompAug(true),
                                                                                                  _isDelaunay(false)
{
    if (dynamic_cast<set_gen *>(_set)->list_base().size())
        _isBase = true;
    _allaug = dynamic_cast<gnss_data_aug *>((*_alldata)[base_data::AUG]);
    _allobj = dynamic_cast<gnss_all_obj *>((*_alldata)[base_data::ALLOBJ]);
    _site = site;
    _simulation = dynamic_cast<set_gproc *>(_set)->simulation();
    base_time begt = dynamic_cast<set_gen *>(_set)->beg();
    base_time endt = dynamic_cast<set_gen *>(_set)->end();

    _isDelaunay = dynamic_cast<set_npp *>(_set)->npp_delaunay();
    _isCompAug = dynamic_cast<set_npp *>(_set)->comp_aug();
    _isSelfCorr = dynamic_cast<set_npp *>(_set)->self_cor();
    _obs_level = dynamic_cast<set_npp *>(_set)->obs_level();
    dynamic_cast<set_npp *>(_set)->aug_limit(_comp_aug_limit, _ion_aug_limit, _trop_aug_limit);

    //Step1:set interpolation site list from gset
    _list_ipsite.clear();
    if (_isSelfCorr)
    {
        _list_ipsite.push_back(_site);
    }
    else
    {
        auto sites = dynamic_cast<set_gen *>(_set)->recs();
        for (auto it = sites.begin(); it != sites.end(); it++)
        {
            bool isrover = false;
            vector<string> list_rover = dynamic_cast<set_gen *>(_set)->list_rover();
            for (auto iter = list_rover.begin(); iter != list_rover.end(); iter++)
            {
                if (*iter == *it)
                {
                    isrover = true;
                    break;
                }
            }
            if (*it == _site || isrover)
                continue;
            _list_ipsite.push_back(*it);
            //auto crd = dynamic_cast<set_rec*>(_set)->get_crd_xyz(*it);
        }
    }

    _nipSite = _list_ipsite.size();

    //Setp2:initialize posproc class
    _site_vrs = "";
    if (_isBase)
    {
        _site_vrs = dynamic_cast<set_gen *>(_set)->list_base()[0];
        //add vrs-site obj
        if (_allobj->obj(_site_vrs) == nullptr)
        {
            shared_ptr<gnss_data_rec> rec = make_shared<gnss_data_rec>();
            rec->name(_site_vrs);
            rec->id(_site_vrs);
            _allobj->add(rec);
        }
    }
}

hwa_gnss::gnss_data_interpol::~gnss_data_interpol()
{
}

void hwa_gnss::gnss_data_interpol::_setCrd(Vector &dx, Vector &dy, Vector &dz)
{
    _crd_dx = dx;
    _crd_dy = dy;
    _crd_dz = dz;
}

double hwa_gnss::gnss_data_interpol::_DIM(Vector &AugData)
{
    int n = AugData.rows();

    _coef_a.resize(n);
    _coef_a.setConstant(1.0);

    double sum = 0;
    for (int i = 0; i < n; i++)
    {

        double dist = sqrt(pow(_crd_dx(i), 2) + pow(_crd_dy(i), 2) + pow(_crd_dz(i), 2));

        _coef_a(i) /= dist;
        sum += _coef_a(i);
    }

    _coef_a /= sum;
    Matrix dAug = _coef_a.transpose() * AugData;

    return dAug(0, 0);
}

double hwa_gnss::gnss_data_interpol::_MLCM(Vector &AugData)
{
    // double dAug = 0.0;
    int n = AugData.rows();

    _coef_a.resize(n);
    _coef_a.setZero();

    Vector unit(n);
    unit.setConstant(1.0);
    Matrix B;
    B.resize(3, n);
    Vector l(3);
    l.setZero();
    l(0) = 1.0;
    B.row(0) = unit.transpose();
    B.row(1) = _crd_dx.transpose();
    B.row(2) = _crd_dy.transpose();

    //X= (BtB).i() * (BtL)
    //_coef_a << (B.transpose() * B).i() * (B.transpose() * l);
    _coef_a << B.transpose() * (B * B.transpose()).inverse() * (l); //lvhb modifeid
    //cout << _coef_a << endl;
    Matrix dAug = _coef_a.transpose() * AugData;

    return dAug(0, 0);
}

double hwa_gnss::gnss_data_interpol::_URTKInterpol(Vector &dAugData)
{
    auto XY_12 = _crd_dx(1) * _crd_dy(0) - _crd_dx(0) * _crd_dy(1);
    auto XY_23 = _crd_dx(1) * _crd_dy(2) - _crd_dx(2) * _crd_dy(1);
    auto XY_13 = _crd_dx(2) * _crd_dy(0) - _crd_dx(0) * _crd_dy(2);

    auto a = XY_23 / XY_12;
    auto b = XY_13 / XY_12;

    auto daug = a * dAugData(0) + b * dAugData(1);

    return daug;
}
double hwa_gnss::gnss_data_interpol::_NRTKInterpol(Vector &dAugData)
{
    Vector v;
    v.resize(2);
    v.setZero();
    v(0) = _crd_dx(2);
    v(1) = _crd_dy(2);
    Matrix A;
    A.resize(2, 2);
    A.setZero();
    A(0, 0) = _crd_dx(0);
    A(0, 1) = _crd_dy(0);
    A(1, 0) = _crd_dx(1);
    A(1, 1) = _crd_dy(1);

    Vector X(1);
    X.resize(1);
    X.setZero();
    Vector l(2);
    l.resize(2);
    l.setZero();
    l(0) = dAugData(1);
    l(1) = dAugData(2);
    X << v.transpose() * (A.inverse() * l);

    cout << "V: " << endl;
    cout << fixed << setprecision(5) << setw(15) << v << endl;
    cout << "A: " << endl;
    cout << fixed << setprecision(5) << setw(15) << A << endl;
    cout << "X: " << endl;
    cout << fixed << setprecision(5) << setw(15) << X << endl;

    return X(0);
}

bool hwa_gnss::gnss_data_interpol::interpolAug(const base_time &nowT, vector<gnss_data_sats> &data_rover, vector<gnss_data_sats> &data_base)
{
    base_time curT = nowT;
    _data_rover.erase(_data_rover.begin(), _data_rover.end());

    auto gobs = dynamic_cast<gnss_all_obs *>((*_alldata)[base_data::ALLOBS]);
    _data_rover = gobs->obs(_site, nowT);
    if (_data_rover.size() == 0)
        return false;

    if (_simulation)
    {
        _crd_rover = dynamic_cast<set_rec *>(_set)->get_crd_xyz(_site);
        if (_crd_rover.isZero())
            return false;
    }
    else
    {
        if (!_SppProcOneEpoch(nowT, _site, &_crd_rover, NULL))
            return false;
    }
    _allobj->obj(_site)->crd(_crd_rover, Triple(1, 1, 1), nowT, LAST_TIME, true);
    cout << "sppflt 1  :" << fixed << fixed << _crd_rover[0] << "  " << _crd_rover[1] << "  " << _crd_rover[2] << "  " << _site << endl;
    _crd_rover += _allobj->obj(_site)->eccxyz(nowT);

    /* delaunay site setting for npp */
    vector<string> DelaunaySite(3);

    if (_isDelaunay)
    {
        if (!_selDelaunaySite(nowT, DelaunaySite))
            return false;
    }

    /* base site setting for rtk */
    map<string, gnss_data_sats> bdata;
    if (_isBase)
    {
        if (!_selMajorSite(nowT))
            return false;
        _data_base.erase(_data_base.begin(), _data_base.end());
        _data_base = gobs->obs(_strMajorSite, nowT);
        if (_data_base.size() == 0)
            return false;
        //if (!_Proc->SppProcOneEpoch(nowT, _strMajorSite, &crd_major, &clk_major, &bdata)) return false;
    }

    int nSat_cor = 0;

    Vector dx, dy, dz;
    vector<Triple> crd_base;
    Vector Aug_l, Aug_p, Aug_ion, Aug_trp;

    ostringstream os;
    os << "> " << left << setw(35) << nowT.str("%Y %m %d %H %M %p")
       << setw(5) << "" << endl;

    ostringstream aug_os;
    int interval = dynamic_cast<set_gen *>(_set)->sampling();
    map<string, hwa_map_id_augtype_value> augs_in, pre_augs;
    for (int j = 0; j < _nipSite; j++)
    {
        string site = _list_ipsite[j];
        auto augepo = _allaug->getAug(site, nowT);
        if (augepo)
            augs_in[site] = *augepo;
    }
    if (_isCompAug && !double_eq(_comp_aug_limit, 0.0))
    {
        auto augpre = _allaug->getAug(_site, curT - interval);
        if (!augpre)
            augpre = _allaug->getAug(_site, curT - 2.0 * interval);
        if (!augpre)
            augpre = _allaug->getAug(_site, curT - 3.0 * interval);
        if (augpre)
            pre_augs[_site] = *augpre;
    }

    //if (!_isCompAug) {
    //    Aug_ztrp.resize(augs_in.size());
    //    Aug_ztrp = 0; int order = 1;

    //    dx.resize(augs_in.size()); dy.resize(augs_in.size()); dz.resize(augs_in.size());
    //    dx = 0; dy = 0; dz = 0;

    //    for (auto iter : augs_in) {
    //        Aug_ztrp(order) = iter.second.ztd;
    //        auto crd = dynamic_cast<set_rec*>(_set)->get_crd_xyz(iter.first);
    //        crd += _allobj->obj(iter.first)->eccxyz(nowT);
    //        dx(order) = -_crd_rover[0] + crd[0];
    //        dy(order) = -_crd_rover[1] + crd[1];
    //        dz(order) = -_crd_rover[2] + crd[2];
    //        order++;
    //    }
    //    _setCrd(dx, dy, dz);
    //    _aug_out.ztd = _MLCM(Aug_ztrp);
    //}

    for (auto iter = _data_rover.begin(); iter != _data_rover.end();)
    {
        auto sat = iter->sat();
        auto gs = iter->gsys();

        gnss_data_sats obsdata_major(_spdlog);
        if (_isBase)
        {
            for (int j = 0; j < _data_base.size(); j++)
            {
                if (_data_base[j].sat() == sat)
                {
                    obsdata_major = (_data_base)[j];
                    break;
                }
            }
            if (obsdata_major.sat() == "")
            {
                iter = _data_rover.erase(iter);
                continue;
            }
        }
        os << right << setw(3) << iter->sat();
        aug_os << right << nowT.str_hms() + "   " << setw(15) << "AUG differ  " + iter->sat();

        bool bcalcor = false;
        bool cycleslip = false;
        gnss_data_obs_manager gnss(_spdlog, sat);
        //xjhan
        if (sat.substr(0, 1) == "2" || sat.substr(0, 1) == "3" || sat.substr(0, 1) == "4" || sat.substr(0, 1) == "5")
            gnss.sat("G");
        vector<GOBSBAND> band = dynamic_cast<set_gnss *>(_set)->band(gs);
        double ion1 = 0.0;
        for (int i = 0; i < band.size(); i++)
        {
            GOBSBAND b = band[i];
            auto gobsP = iter->select_range(b, true);
            auto gobsL = iter->select_phase(b, true);
            if (gobsP == X || gobsL == X)
            {
                iter->eraseband(b);
                continue;
            }
            int iobs = 0;
            dx.resize(_nipSite);
            dy.resize(_nipSite);
            dz.resize(_nipSite);
            Aug_l.resize(_nipSite);
            Aug_p.resize(_nipSite);
            Aug_ion.resize(_nipSite);
            Aug_trp.resize(_nipSite);
            dx.setZero();
            dy.setZero();
            dz.setZero();
            Aug_p.setZero();
            Aug_l.setZero();
            Aug_ion.setZero();
            Aug_trp.setZero();
            crd_base.clear();
            double Aug_p_rover = 0.0, Aug_l_rover = 0.0, Aug_ion_rovor = 0.0, Aug_trp_rovor = 0.0;
            cout << nowT.str_hms() << "   sat-band-site   " << iter->sat() << "  " << b;
            int sitenum = _nipSite;
            if (!_isSelfCorr && _isDelaunay)
                sitenum = 3;

            bool valid = false;
            for (int j = 0; j < sitenum; j++)
            {
                string site = _list_ipsite[j];
                if (!_isSelfCorr && _isDelaunay)
                    site = DelaunaySite[j];
                if (augs_in.find(site) == augs_in.end() || augs_in[site].find(sat) == augs_in[site].end())
                    continue;
                auto aug_in = augs_in[site][sat];
                auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(site);
                crd += _allobj->obj(site)->eccxyz(nowT);

                valid = false;
                if (aug_in.find(make_pair(AUGTYPE::TYPE_P, b)) != aug_in.end() &&
                    aug_in.find(make_pair(AUGTYPE::TYPE_L, b)) != aug_in.end())
                {
                    if (_isCompAug)
                        valid = true;
                    else if (aug_in.find(make_pair(AUGTYPE::TYPE_ION, b)) != aug_in.end() &&
                             aug_in.find(make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1)) != aug_in.end())
                    {
                        valid = true;
                    }
                }
                if (valid)
                {
                    dx(iobs) = -_crd_rover[0] + crd[0];
                    dy(iobs) = -_crd_rover[1] + crd[1];
                    dz(iobs) = -_crd_rover[2] + crd[2];
                    Aug_p(iobs) = aug_in[make_pair(AUGTYPE::TYPE_P, b)];
                    Aug_l(iobs) = aug_in[make_pair(AUGTYPE::TYPE_L, b)];
                    if (!_isCompAug)
                    {
                        Aug_ion(iobs) = aug_in[make_pair(AUGTYPE::TYPE_ION, b)];
                        Aug_trp(iobs) = aug_in[make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1)];
                    }
                    crd_base.push_back(crd);
                    //_isInterpolRange(&crd);
                    cout << "  " << site;
                    ++iobs;
                }
            } //end nipsite
            cout << endl;

            //interpolation
            if (iobs >= _obs_level && iobs != 0) // iobs >= 1
            {
                dx = dx.segment(0, iobs);
                dy = dy.segment(0, iobs);
                dz = dz.segment(0, iobs);
                Aug_p = Aug_p.segment(0, iobs);
                Aug_l = Aug_l.segment(0, iobs);
                Aug_ion = Aug_ion.segment(0, iobs);
                Aug_trp = Aug_trp.segment(0, iobs);
                /*hwa_SPT_interpol interpol = make_shared<gnss_data_interpol>(dx, dy, dz);*/
                _setCrd(dx, dy, dz);

                if (iobs == 1)
                {
                    Aug_p_rover = Aug_p(0);
                    Aug_l_rover = Aug_l(0);
                    if (!_isCompAug)
                    {
                        Aug_ion_rovor = Aug_ion(0);
                        Aug_trp_rovor = Aug_trp(0);
                    }
                }
                else if (iobs == 2)
                {
                    Aug_p_rover = _DIM(Aug_p);
                    Aug_l_rover = _DIM(Aug_l);
                    if (!_isCompAug)
                    {
                        Aug_ion_rovor = _DIM(Aug_ion);
                        Aug_trp_rovor = _DIM(Aug_trp);
                    }
                }
                else if (iobs >= 3)
                {
                    Aug_p_rover = _MLCM(Aug_p);
                    Aug_l_rover = _MLCM(Aug_l);
                    if (!_isCompAug)
                    {
                        Aug_ion_rovor = _MLCM(Aug_ion);
                        Aug_trp_rovor = _MLCM(Aug_trp);
                    }
                }

                os << right
                    << setw(12) << fixed << setprecision(4) << Aug_p_rover
                    << setw(12) << fixed << setprecision(4) << Aug_l_rover;

                iter->setobsLevelFlag(gobsP, _isSelfCorr ? 3 : iobs);
                iter->setobsLevelFlag(gobsL, _isSelfCorr ? 3 : iobs);
                _allaug->add_data(_site, curT, sat, make_pair(AUGTYPE::TYPE_P, b), Aug_p_rover);
                _allaug->add_data(_site, curT, sat, make_pair(AUGTYPE::TYPE_L, b), Aug_l_rover);

                //add obs
                if (_isBase) //vrs
                {
                    if (obsdata_major.select_phase(b) == X || obsdata_major.select_range(b) == X || double_eq(obsdata_major.obs_L(gobsL), 0.0) || double_eq(obsdata_major.obs_C(gobsP), 0.0))
                    {
                        continue;
                    }
                    //creating VRS observations- referenced satellite
                    if (!bcalcor)
                    {
                        //major site coordinate
                        auto xyz_r = dynamic_cast<set_rec *>(_set)->get_crd_xyz(_strMajorSite);
                        xyz_r += _allobj->obj(_strMajorSite)->eccxyz(nowT);
                        _cal_sat_inf(obsdata_major, xyz_r); //major site
                        _cal_sat_inf(*iter, _crd_rover);    //rover site
                    }

                    if (double_eq(obsdata_major.rho(), 0.0) || double_eq(iter->rho(), 0.0))
                        continue;
                    double rho_AV_sat = iter->rho() - obsdata_major.rho();

                    hwa_spt_obsmanager obs_vrs = _check_vrs_prn_pt(nowT, sat);

                    auto aug_major = _allaug->getAug(_strMajorSite, nowT, sat);
                    if (!aug_major)
                        continue;
                    if (aug_major->find(make_pair(AUGTYPE::TYPE_P, b)) == aug_major->end() &&
                        aug_major->find(make_pair(AUGTYPE::TYPE_L, b)) == aug_major->end())
                    {
                        continue;
                    }
                    auto Aug_p_major = (*aug_major)[make_pair(AUGTYPE::TYPE_P, b)];
                    auto Aug_l_major = (*aug_major)[make_pair(AUGTYPE::TYPE_L, b)];

                    //new method by lvhb,202010
                    double trop_satb = _getTropModValue(nowT, obsdata_major);
                    double trop_satr = _getTropModValue(nowT, *iter);
                    double tmpsdtrop = (trop_satr - trop_satb);
                    Aug_p_rover += tmpsdtrop;
                    Aug_l_rover += tmpsdtrop;

                    double obsP = obsdata_major.obs_C(gobsP) + rho_AV_sat + Aug_p_rover - Aug_p_major /*- clk_major*/;
                    double obsL = (obsdata_major.obs_L(gobsL) + rho_AV_sat + Aug_l_rover - Aug_l_major /*- clk_major - Amb_major*/) / obs_vrs->wavelength(b);

                    obs_vrs->addobs(gobsP, obsP);
                    obs_vrs->addobs(gobsL, obsL);

                    gobs->addobs(obs_vrs);
                }
                else // PPP-RTK
                {
                    if (!_isCompAug && iobs >= _obs_level)
                    {
                        _allaug->add_data(_site, curT, sat, make_pair(AUGTYPE::TYPE_ION, b), Aug_ion_rovor);
                        _allaug->add_data(_site, curT, sat, make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1), Aug_trp_rovor);
                        
                        if (i == 0)
                            ion1 = Aug_ion_rovor;
                        else if (i == 1)
                        {
                            os << right
                                << setw(12) << fixed << setprecision(4) << Aug_trp_rovor
                                << setw(12) << fixed << setprecision(4) << ion1
                                << setw(12) << fixed << setprecision(4) << Aug_ion_rovor;
                        }
                    }
                    if (!_isCompAug)
                    {
                        if (!_isCorObs)
                        {
                            Aug_p_rover = 0.0;
                            Aug_l_rover = 0.0;
                        }
                        else
                        {
                            Aug_p_rover = (Aug_ion_rovor + Aug_trp_rovor);
                            Aug_l_rover = (-Aug_ion_rovor + Aug_trp_rovor);
                        }
                    }
                    

                    double corrP = (iter->obs_C(gobsP) - Aug_p_rover);
                    double corrL = (iter->obs_L(gobsL) - Aug_l_rover) / gnss.wavelength(b);
                    iter->addobs(gobsP, corrP);
                    iter->addobs(gobsL, corrL);

                    // judge aug between adjacent epochs
                    if (_isCompAug && !double_eq(_comp_aug_limit, 0.0) && pre_augs[_site].find(sat) != pre_augs[_site].end())
                    {
                        auto pre_aug = pre_augs[_site][sat];
                        if (pre_aug.find(make_pair(AUGTYPE::TYPE_P, b)) != pre_aug.end() &&
                            pre_aug.find(make_pair(AUGTYPE::TYPE_L, b)) != pre_aug.end())
                        {
                            auto pre_aug_p_rover = pre_aug[make_pair(AUGTYPE::TYPE_P, b)];
                            auto pre_aug_l_rover = pre_aug[make_pair(AUGTYPE::TYPE_L, b)];

                            double diff_p = Aug_p_rover - pre_aug_p_rover;
                            double diff_l = Aug_l_rover - pre_aug_l_rover;
                            string flag = "     ";
                            if (abs(diff_l) > _comp_aug_limit)
                            {
                                flag = " run ";
                                cycleslip = true;
                            }

                            aug_os << fixed << setw(5) << "P" + gobsband2str(b) << setw(8) << setprecision(3) << diff_p
                                   << setw(5) << "L" + gobsband2str(b) << setw(8) << setprecision(3) << diff_l
                                   << flag;
                        }
                    }
                }
                bcalcor = true;
            }
            else // iobs == 0
            {
                if (!_isBase)
                {
                    if (gobsL != X)
                        iter->setobsLevelFlag(gobsL, 0);
                    if (gobsP != X)
                        iter->setobsLevelFlag(gobsP, 0);
                }
            }

            // delete obs according to obs_level
            if (iter->getobsLevelFlag(gobsL) < _obs_level)
                iter->eraseband(b);

        } //end band

        os << endl;
        aug_os << endl;
        if (iter->obs().size() == 0)
            bcalcor = false;
        else if (_obs_level == 0)
            bcalcor = true;

        if (bcalcor)
        {
            nSat_cor++;
            iter->exisgnss_coder_aug(true);
        }
        else
        {
            iter = _data_rover.erase(iter);
            continue;
        }

        if (cycleslip) // add slip flag
        {
            for (int i = 0; i < band.size(); i++)
            {
                auto gobsL = iter->select_phase(band[i]);
                if (gobsL == X)
                    continue;
                iter->addlli(gobsL, 1);
            } //end band
        }

        iter++;
    } //end data one epoch
    if (nSat_cor < 5)
        return false;
    else
    {
        if (_augfile)
        {
            _augfile->write(os.str().c_str(), os.str().size());
            _augfile->flush();
        }
        cout << aug_os.str();
    }
    // Todo reset AMB once ObsLevel Change ...
    //output lvhb
    data_rover.erase(data_rover.begin(), data_rover.end());
    data_rover = _data_rover;
    if (_isBase)
    {
        _data_base.erase(_data_base.begin(), _data_base.end());
        _data_base = gobs->obs(_site_vrs, nowT);
        if (_data_base.size() < 6)
            return false;
        //output lvhb
        data_base.erase(data_base.begin(), data_base.end());
        data_base = _data_base;
    }

    return true;
}

bool hwa_gnss::gnss_data_interpol::Grid2Aug(const base_time& nowT, vector<gnss_data_sats>& data_rover, vector<gnss_data_sats>& data_base)
{
    base_time curT = nowT;

    _data_rover.erase(_data_rover.begin(), _data_rover.end());

    auto gobs = dynamic_cast<gnss_all_obs*>((*_alldata)[base_data::ALLOBS]);
    auto gnav = dynamic_cast<gnss_all_nav*>((*_alldata)[base_data::GRP_EPHEM]);
    _data_rover = gobs->obs(_site, nowT);
    if (_data_rover.size() == 0)
        return false;

    if (_simulation)
    {
        _crd_rover = dynamic_cast<set_rec*>(_set)->get_crd_xyz(_site);
        if (_crd_rover.isZero())
            return false;
    }
    else
    {
        if (!_SppProcOneEpoch(nowT, _site, &_crd_rover, NULL))
            return false;
    }
    _allobj->obj(_site)->crd(_crd_rover, Triple(1, 1, 1), nowT, LAST_TIME, true);
    cout << "sppflt 1  :" << fixed << fixed << _crd_rover[0] << "  " << _crd_rover[1] << "  " << _crd_rover[2] << "  " << _site << endl;
    _crd_rover += _allobj->obj(_site)->eccxyz(nowT);
 
    /* base site setting for rtk */

    int nSat_cor = 0;

    ostringstream os;
    os << "> " << left << setw(35) << nowT.str("%Y %m %d %H %M %p")
        << setw(5) << "" << endl;

    ostringstream aug_os;
    int interval = dynamic_cast<set_gen*>(_set)->sampling();
    

    

    for (auto iter = _data_rover.begin(); iter != _data_rover.end();)
    {
        auto sat = iter->sat();
        auto gs = iter->gsys();
        iter->addprd(gnav);
        os << right << setw(3) << iter->sat();

        bool bcalcor = false;
        bool cycleslip = false;
        gnss_data_obs_manager gnss(_spdlog, sat);
        //xjhan
        if (sat.substr(0, 1) == "2" || sat.substr(0, 1) == "3" || sat.substr(0, 1) == "4" || sat.substr(0, 1) == "5")
            gnss.sat("G");
        vector<GOBSBAND> band = dynamic_cast<set_gnss*>(_set)->band(gs);
        for (int i = 0; i < band.size(); i++)
        {
            hwa_map_id_augtype_value crgnss_coder_aug;
            crgnss_coder_aug.clear();
            GOBSBAND b = band[i];
            auto gobsP = iter->select_range(b, true);
            auto gobsL = iter->select_phase(b, true);
            if (gobsP == X || gobsL == X)
            {
                iter->eraseband(b);
                continue;
            }
            if (!_grid_model->Grid2Aug(nowT, _crd_rover, *iter, crgnss_coder_aug, b))
            {
                iter->eraseband(b);
                continue;
            }
            if (i == 0)
            {
                os << right
                    << setw(12) << fixed << setprecision(4) << crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_TRP, b)];         
            }
            os << right
                << setw(12) << fixed << setprecision(4) << crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_ION, b)];
            _allaug->add_data(_site, curT, sat, make_pair(AUGTYPE::TYPE_ION, b), crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_ION, b)]);
            _allaug->add_data(_site, curT, sat, make_pair(AUGTYPE::TYPE_TRP, GOBSBAND::BAND_1), crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_TRP, b)]);
            double Aug_p_rover = crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_ION, b)] + crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_TRP, b)];
            double Aug_l_rover = -crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_ION, b)] + crgnss_coder_aug[sat][make_pair(AUGTYPE::TYPE_TRP, b)];
            if (_isCorObs)
            {
                double corrP = (iter->obs_C(gobsP) - Aug_p_rover);
                double corrL = (iter->obs_L(gobsL) - Aug_l_rover) / gnss.wavelength(b);
                iter->addobs(gobsP, corrP);
                iter->addobs(gobsL, corrL);
            }
        } //end band
        bcalcor = true;
        os << endl;
        if (iter->obs().size() == 0)
            bcalcor = false;

        if (bcalcor)
        {
            nSat_cor++;
            iter->exisgnss_coder_aug(true);
        }
        else
        {
            iter = _data_rover.erase(iter);
            continue;
        }

        if (cycleslip) // add slip flag
        {
            for (int i = 0; i < band.size(); i++)
            {
                auto gobsL = iter->select_phase(band[i]);
                if (gobsL == X)
                    continue;
                iter->addlli(gobsL, 1);
            } //end band
        }

        iter++;
    } //end data one epoch
    if (nSat_cor < 5)
        return false;
    else
    {
        if (_augfile)
        {
            _augfile->write(os.str().c_str(), os.str().size());
            _augfile->flush();
        }
        cout << aug_os.str();
    }
    // Todo reset AMB once ObsLevel Change ...
    //output lvhb
    data_rover.erase(data_rover.begin(), data_rover.end());
    data_rover = _data_rover;
    if (_isBase)
    {
        _data_base.erase(_data_base.begin(), _data_base.end());
        _data_base = gobs->obs(_site_vrs, nowT);
        if (_data_base.size() < 6)
            return false;
        //output lvhb
        data_base.erase(data_base.begin(), data_base.end());
        data_base = _data_base;
    }

    return true;
}

void hwa_gnss::gnss_data_interpol::getAugFileName(base_iof* augfile)
{
    if (augfile) 
        _augfile = augfile;
}

bool hwa_gnss::gnss_data_interpol::_selMajorSite(const base_time &nowT)
{
    //base_time beg = dynamic_cast<set_gen*>(_set)->beg();
    base_time end = dynamic_cast<set_gen *>(_set)->end();
    auto grec_vrs = _allobj->obj(_site_vrs);
    //auto xyz = _crd_rover + _allobj->obj(_site)->eccxyz(nowT);
    grec_vrs->crd(_crd_rover, Triple(1, 1, 1), nowT, end + 30, true);

    double preLengthBL = 0.0;
    _strMajorSite = "";
    for (int j = 0; j < _nipSite; j++)
    {
        string site = _list_ipsite[j];
        if (!_allaug->getAug(site, nowT))
            continue;

        auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(site);
        crd[0] -= _crd_rover[0];
        crd[1] -= _crd_rover[1];
        crd[2] -= _crd_rover[2];
        double lengthBL = crd.norm();
        if (_strMajorSite == "")
        {
            _strMajorSite = site;
            preLengthBL = lengthBL;
        }
        else if (lengthBL < preLengthBL)
        {
            preLengthBL = lengthBL;
            _strMajorSite = site;
        }
    }
    if (_strMajorSite == "")
        return false;

    string ant = _allobj->obj(_strMajorSite)->ant(nowT);
    grec_vrs->ant(ant, nowT);
    _allobj->sync_pcvs();

    if (dynamic_cast<set_gproc *>(_set)->realtime())
    {
        while (!_avaliable(nowT, &_site, &_strMajorSite))
            base_time::gmsleep(1);
    }

    return true;
}

bool hwa_gnss::gnss_data_interpol::_selDelaunaySite(const base_time &nowT, vector<string> &selsite)
{
    selsite.clear();
    vector<Triple> crd_base;
    vector<string> base;
    for (int j = 0; j < _nipSite; j++)
    {
        string site = _list_ipsite[j];
        if (!_allaug->getAug(site, nowT))
            continue;

        auto crd = dynamic_cast<set_rec *>(_set)->get_crd_xyz(site);
        crd_base.push_back(crd);
        base.push_back(site);
    }
    if (crd_base.size() >= 3)
    { //select 3 sites by Delaunay method
        gnss_base_celaunay delaunay;
        int index[3] = {0};
        if (!delaunay.getIntSite(_crd_rover, crd_base, index))
            return false;

        cout << endl
             << "DEL  ";
        for (int i = 0; i < 3; i++)
        {
            selsite.push_back(base[index[i]]);
            cout << base[index[i]] << " ";
        }
        cout << endl;
    }
    else
        return false;
    return true;
}

hwa_spt_obsmanager hwa_gnss::gnss_data_interpol::_check_vrs_prn_pt(const base_time &nowT, string &sat)
{
    auto gobs = dynamic_cast<gnss_all_obs *>((*_alldata)[base_data::ALLOBS]);
    auto obs_temp = gobs->obs_pt(_site_vrs, nowT);
    hwa_spt_obsmanager obs_ref(nullptr);
    for (auto iter = obs_temp.begin(); iter != obs_temp.end(); iter++)
    {
        if ((*iter)->sat() == sat)
        {
            obs_ref = *iter;
            break;
        }
    }
    if (!obs_ref)
    {
        obs_ref = make_shared<gnss_data_obs_manager>(_spdlog, _site_vrs, sat, nowT);
    }
    return obs_ref;
}

bool hwa_gnss::gnss_data_interpol::_SppProcOneEpoch(const base_time &nowT, const string &site, Triple *crd, double *reclk, map<string, gnss_data_sats> *satdata)
{
    auto gobs = dynamic_cast<gnss_all_obs *>((*_alldata)[base_data::ALLOBS]);
    auto gorb = dynamic_cast<gnss_all_nav *>((*_alldata)[base_data::GRP_EPHEM]);
    auto gobj = dynamic_cast<gnss_all_obj *>((*_alldata)[base_data::ALLOBJ]);
    auto gbias = dynamic_cast<gnss_all_bias *>((*_alldata)[base_data::ALLBIAS]);
    /*shared_ptr<t_gspplsq> spplsq = make_shared<t_gspplsq>(site, _set, gobj, _spdlog);  // there are bugs in spplsq 
    spplsq->setDAT(gobs, gorb);
    spplsq->setDCB(gbias);
    if (spplsq->processBatch(nowT, nowT) <= 0)
    {
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, site + "spplsq failed");
        return false;
    }
    if (crd) 
        *crd = spplsq->getCrd(nowT);
    if (reclk)
        *reclk = spplsq->getRecClk(nowT);
    if (satdata) 
        spplsq->outcomsat(*satdata);
    return true;*/

    shared_ptr<gnss_proc_sppflt> sppflt = make_shared<gnss_proc_sppflt>(site, _set, _spdlog,"interpol");
    //sppflt->reset_observ(IONO_FREE);
    //sppflt->phase(false);
    sppflt->minsat(5);
    sppflt->setDAT(gobs, gorb);
    sppflt->setOBJ(gobj);
    sppflt->setDCB(gbias);

    gnss_all_prod *prod = new gnss_all_prod();
    sppflt->setOUT(prod);
    sppflt->processBatch(nowT, nowT);
    auto prd_crd = static_pointer_cast<gnss_prod_crd>(prod->get(site, base_data::POS, nowT));
    auto prd_clk = static_pointer_cast<gnss_prod_clk>(prod->get(_site, base_data::CLK, nowT));
    if (prd_crd != nullptr /*&& prd_clk != nullptr*/)
    {
        if (crd)
            *crd = prd_crd->xyz();
        if (reclk)
            *reclk = prd_clk->clk();
        if (prod)
        {
            delete prod;
            prod = nullptr;
}
    }
    else
    {
        if (_spdlog)
        {
            SPDLOG_LOGGER_ERROR(_spdlog, "interpol-spp", site + " SPP failed! (epo: " + nowT.str_ymdhms() + ")");
        }
        if (prod)
        {
            delete prod;
            prod = nullptr;
        }
        return false;
    }
    return true;
}

void hwa_gnss::gnss_data_interpol::cal_rho_azel(const string &site_name, Triple &xyz_s, const Triple &xyz_r, gnss_data_sats &obs_sat)
{
    //hwa_spt_obsmanager pvtflt = make_shared<t_gpvtflt>(_site, site_base, set, data);
}

void hwa_gnss::gnss_data_interpol::_cal_sat_inf(gnss_data_sats &obs_sat, const Triple &xyz_r)
{
    auto precise_bias = make_shared<gnss_model_precise_biasgpp>(_alldata, _spdlog, _set);
    base_allpar parm;

    int ipar = 0;
    string tmpsite = obs_sat.site();
    //xyz
    base_par parx(tmpsite, par_type::CRD_X, ++ipar, "");
    parx.value(xyz_r[0]);
    base_par pary(tmpsite, par_type::CRD_Y, ++ipar, "");
    pary.value(xyz_r[1]);
    base_par parz(tmpsite, par_type::CRD_Z, ++ipar, "");
    parz.value(xyz_r[2]);
    parm.addParam(parx);
    parm.addParam(pary);
    parm.addParam(parz);
    //clk
    double clk = 0.0;
    //if (!SppProcOneEpoch(obs_sat.epoch(), tmpsite, NULL, &clk)) clk = 0.0;
    base_par parclk(tmpsite, par_type::CLK, ++ipar, "");
    parclk.value(clk);
    parm.addParam(parclk);

    precise_bias->_update_obs_info_GPP(obs_sat.epoch(), _allnav, _allobj, obs_sat, parm);
    precise_bias->_prepare_obs_GPP(obs_sat.epoch(), _allnav, _allobj, parm);
    precise_bias->_update_obs_info(obs_sat);
}

double hwa_gnss::gnss_data_interpol::_getTropModValue(const base_time &nowT, gnss_data_sats &obsdata)
{
    double delay = 0.0;
    auto strEst = dynamic_cast<set_gen *>(_set)->estimator();
    auto isFLT = (_isBase || strEst == "FLT");

    //saas
    auto trpModStr = dynamic_cast<set_gproc *>(_set)->tropo_model();
    shared_ptr<gnss_model_tropo> tropoModel;
    if (trpModStr == TROPMODEL::SAASTAMOINEN)
        tropoModel = make_shared<gnss_model_tropo_SAAST>();
    else if (trpModStr == TROPMODEL::DAVIS)
        tropoModel = make_shared<gnss_model_tropo_davis>();
    else if (trpModStr == TROPMODEL::HOPFIELD)
        tropoModel = make_shared<gnss_model_tropo_hopf>();
    //else if (trpModStr == MOPS)          tropoModel = make_shared<t_blindmops>();
    else
        return 0.0;

    string site = obsdata.site();
    auto grec = dynamic_cast<gnss_all_obj *>((*_alldata)[base_data::ALLOBJ])->obj(site);
    // Cartesian coordinates to ellipsodial coordinates
    Triple xyz, ell;
    xyz = dynamic_cast<set_rec *>(_set)->get_crd_xyz(site);
    if (xyz.isZero())
    {
        xyz = grec->crd_arp(nowT);
    }
    else
    {
        xyz = xyz + grec->eccxyz(nowT);
    }
    xyz2ell(xyz, ell, false);

    double zwd = 0.0, zhd = 0.0;
    zwd = tropoModel->getZWD(ell, nowT);
    zhd = tropoModel->getZHD(ell, nowT);

    string sat = obsdata.sat();
    double ele = obsdata.ele();
    if (double_eq(ele, 0.0))
    {
        if (!isFLT)
        {
            xyz = xyz - grec->eccxyz(nowT);
        }
        _cal_sat_inf(obsdata, xyz);
        ele = obsdata.ele();
    }

    auto tropo_mf = dynamic_cast<set_gproc *>(_set)->tropo_mf();

    double mfh, mfw, dmfh, dmfw;
    mfh = mfw = dmfh = dmfw = 0.0;
    if (tropo_mf == ZTDMPFUNC::GMF || tropo_mf == ZTDMPFUNC::NO_MF)
    {
        gnss_model_gmf mf;
        mf.gmf(nowT.mjd(), ell[0], ell[1], ell[2], hwa_pi / 2.0 - ele,
               mfh, mfw, dmfh, dmfw);
    }
    else if (tropo_mf == ZTDMPFUNC::COSZ)
    {
        mfh = mfw = 1 / sin(ele);
        dmfh = dmfw = -(cos(ele)) / (sin(ele) * sin(ele));
    }

    delay = mfh * zhd + mfw * zwd;

    return delay;
}

bool hwa_gnss::gnss_data_interpol::_avaliable(base_time nowT, string *site, string *site_base)
{
    auto gobs = dynamic_cast<gnss_all_obs *>((*_alldata)[base_data::ALLOBS]);
    // auto gorb = dynamic_cast<gnss_all_nav *>((*_alldata)[base_data::GRP_EPHEM]);
    //auto nppmodel = dynamic_cast<set_gproc*>(_set)->npp_model();
    // bool isClient = dynamic_cast<set_npp *>(_set)->isClient();

    bool b = nowT < gobs->end_obs(site ? (*site) : _site);
    if (_isBase && b)
    {
        if (site_base)
        {
            b = nowT < gobs->end_obs((*site_base));
        }
        else
            b = false;
    }

    return b;
}

double hwa_gnss::gnss_data_interpol::_corrTropHeight(const base_time &nowT, int sign, const Triple &bcrd, Triple &rcrd, const Triple &crd, gnss_data_sats bdata_sat, gnss_data_sats bdata_bsat, gnss_data_sats rdata_sat, gnss_data_sats rdata_bsat)
{
    double delay = 0.0;
    //saas
    auto trpModStr = dynamic_cast<set_gproc *>(_set)->tropo_model();
    shared_ptr<gnss_model_tropo> tropoModel;
    if (trpModStr == TROPMODEL::SAASTAMOINEN)
        tropoModel = make_shared<gnss_model_tropo_SAAST>();
    else if (trpModStr == TROPMODEL::DAVIS)
        tropoModel = make_shared<gnss_model_tropo_davis>();
    else if (trpModStr == TROPMODEL::HOPFIELD)
        tropoModel = make_shared<gnss_model_tropo_hopf>();
    //else if (trpModStr == MOPS)          tropoModel = make_shared<t_blindmops>();
    else
        return false;
    auto tropo_mf = dynamic_cast<set_gproc *>(_set)->tropo_mf();

    string bsat = bdata_bsat.sat();
    string sat = bdata_sat.sat();

    Triple bell, rell, ell_h0;
    xyz2ell(bcrd, bell, false);
    xyz2ell(rcrd, rell, false);
    xyz2ell(crd, ell_h0, false);

    double zwd = tropoModel->getZWD(rell, nowT);
    double zhd = tropoModel->getZHD(rell, nowT);

    bell[2] = ell_h0[2];
    rell[2] = ell_h0[2];
    double zwd_h0 = tropoModel->getZWD(rell, nowT) - zwd;
    double zhd_h0 = tropoModel->getZHD(rell, nowT) - zhd;

    Triple xyz_r, xyz_b;
    ell2xyz(rell, xyz_r, false);
    ell2xyz(bell, xyz_b, false);
    Triple rb_satcrd = rdata_bsat.satcrd(), r_satcrd = rdata_sat.satcrd(),
              bb_satcrd = bdata_bsat.satcrd(), b_satcrd = bdata_sat.satcrd();
    cal_rho_azel(rdata_bsat.site(), rb_satcrd, xyz_r, rdata_bsat);
    cal_rho_azel(rdata_sat.site(), r_satcrd, xyz_r, rdata_sat);
    cal_rho_azel(bdata_bsat.site(), bb_satcrd, xyz_b, bdata_bsat);
    cal_rho_azel(bdata_sat.site(), b_satcrd, xyz_b, bdata_sat);

    double bbsat_ele = bdata_bsat.ele();
    //double bsat_ele = bdata_sat.ele();
    //double rbsat_ele = rdata_bsat.ele();
    double rsat_ele = rdata_sat.ele();

    //auto tropo_mf = dynamic_cast<set_gproc*>(_set)->tropo_mf();

    for (int j = 0; j < 2; j++)
    {
        auto tmpell = bell;
        auto tmpele = bbsat_ele;
        if (j)
        {
            tmpell = rell;
            tmpele = rsat_ele;
        }
        double mfh, mfw, dmfh, dmfw;
        mfh = mfw = dmfh = dmfw = 0.0;
        if (tropo_mf == ZTDMPFUNC::GMF)
        {
            gnss_model_gmf mf;
            mf.gmf(nowT.mjd(), tmpell[0], tmpell[1], tmpell[2], hwa_pi / 2.0 - tmpele,
                   mfh, mfw, dmfh, dmfw);
        }
        else if (tropo_mf == ZTDMPFUNC::COSZ)
        {
            mfh = mfw = 1 / sin(tmpele);
            dmfh = dmfw = -(cos(tmpele)) / (sin(tmpele) * sin(tmpele));
        }

        delay = (mfh * zhd_h0 + mfw * zwd_h0) - delay;
    }

    return delay;
}
