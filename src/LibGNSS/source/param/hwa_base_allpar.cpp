#include <iostream>
#include <iomanip>
#include <cmath>
#include <assert.h>
#include "hwa_base_allpar.h"

namespace hwa_base
{
    void base_allpar::addParam(const base_par& newPar)
    {
        this->_vParam.push_back(newPar);
        this->_point_par.push_back(_max_point++);
        this->_index_par[newPar.get_head()][newPar.get_timearc()] = this->_point_par[this->_point_par.size() - 1];
    }

    void base_allpar::delParam(const int& i)
    {

        auto& all = this->_index_par.at(_vParam[i].get_head());
        for (auto iter = all.begin(); iter != all.end(); ++iter)
        {
            if (iter->second == _point_par[i])
            {
                all.erase(iter);
                break;
            }
        }
        if (this->_index_par[_vParam[i].get_head()].size() == 0)
        {
            this->_index_par.erase(_vParam[i].get_head());
        }
        _point_par.erase(_point_par.begin() + i);

        _vParam.erase(_vParam.begin() + i);
    }

    // get position of item according: station name, par type, PRN, begin time, end time
    // -----------------------------------------------------
    int base_allpar::getParam(const std::string& mark, const par_type& type, const std::string& prn, const base_time& beg, const base_time& end) const
    {
        if (this->_index_par.count(base_par_head(type, mark, prn)) == 0)
        {
            return -1;
        }
        else
        {
            const auto& all = this->_index_par.find(base_par_head(type, mark, prn))->second;
            base_time_arc dst_timearc(beg, end);

            auto all_end = all.end();
            auto par_beg = _point_par.begin();
            auto par_end = _point_par.end();
            for (auto iter = all.begin(); iter != all_end; ++iter)
            {
                if (iter->first.inside(dst_timearc))
                {
                    auto ans = lower_bound(par_beg, par_end, iter->second);
                    assert(ans != par_end && *ans == iter->second);
                    return ans - par_beg;
                }
            }
            return -1;
        }
        return -1;
    }

    int base_allpar::getParam(const int& index)
    {

        for (unsigned int i = 0; i <= _vParam.size() - 1; i++)
        {
            if (_vParam[i].index == index)
            {
                return i;
            }
        }
        return -1;
    }

    int base_allpar::getParIndex(const int& idx)
    {
        if (idx >= 0 && idx < _vParam.size())
        {
            return _vParam[idx].index;
        }
        else
        {
            return -1;
        }
    }

    double base_allpar::getParValue(const int& idx)
    {
        if (idx >= 0 && idx < _vParam.size())
        {
            return _vParam[idx].value();
        }
        else
        {
            return 0.0;
        }
    }

    void base_allpar::setParValue(const int& idx, const double& value)
    {
        if (idx >= 0 && idx < _vParam.size())
        {
            _vParam[idx].value(value);
        }
    }

    void base_allpar::setParRemove(const int& idx, const bool& judge)
    {
        if (idx >= 0 && idx < _vParam.size())
        {
            _vParam[idx].lremove = judge;
        }
    }

    // Reindexing parametres.
    // New indexes are reordered form 1 to n
    // -----------------------------------------------
    void base_allpar::reIndex()
    {
        int index_new = 0;
        for (unsigned int iPar = 0; iPar <= _vParam.size() - 1; iPar++)
        {
            _vParam[iPar].index = index_new;
            index_new++;
        }
    }

    // Reindexing parametres.
    // All indexes larger than "i" is decresed by 1
    // -----------------------------------------------
    void base_allpar::decIndex(const int& i)
    {

        for (unsigned int iPar = 0; iPar <= _vParam.size() - 1; iPar++)
        {
            if (_vParam[iPar].index > i)
                _vParam[iPar].index -= 1;
        }
    }

    // Reindexing parametres.
    // All indexes larger than "i" is incresed by 1
    // -----------------------------------------------
    void base_allpar::incIndex(const int& i)
    {

        for (unsigned int iPar = 0; iPar <= _vParam.size() - 1; iPar++)
        {
            if (_vParam[iPar].index >= i)
                _vParam[iPar].index += 1;
        }
    }

    // Get number of parametres
    // ---------------------------------------------
    unsigned int base_allpar::parNumber() const
    {

        return _vParam.size();
    }

    // Get number of orbit parametres
    // ---------------------------------------------
    unsigned int base_allpar::orbParNumber() const
    {

        unsigned int orbparnum = 0;
        for (auto sat : _vOrbParam)
        {
            orbparnum += sat.second.size();
        }

        return orbparnum;
    }

    // Strore Coordinates parametres to t_Triple crd
    // ------------------------------------------------

    int base_allpar::getCrdParam(const std::string& station, Triple& crd, const base_time& Tbeg, const base_time& Tend) const
    {

        int found = 0;

        int idx = this->getParam(station, par_type::CRD_X, "", Tbeg, Tend);
        int idy = this->getParam(station, par_type::CRD_Y, "", Tbeg, Tend);
        int idz = this->getParam(station, par_type::CRD_Z, "", Tbeg, Tend);

        if (idx != -1)
        {
            crd[0] = _vParam[idx].value();
            found++;
        }
        if (idy != -1)
        {
            crd[1] = _vParam[idy].value();
            found++;
        }
        if (idz != -1)
        {
            crd[2] = _vParam[idz].value();
            found++;
        }

        if (found == 3)
            return 1; // all three crd were found
        if (found == 1)
            return -1; // just one crd was found
        if (found == 2)
            return -2; // just two crd was found

        if (found == 0)
            return -3; // crd not found

        return -1;
    }

    int base_allpar::getVelParam(const std::string& station, Triple& vel, const base_time& Tbeg, const base_time& Tend) const
    {
        int found = 0;
        std::vector<base_par>::const_iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
        {
            if (iter->parType == par_type::VEL_X && iter->site.compare(station) == 0 &&
                iter->beg == Tbeg && iter->end == Tend)
            {
                vel[0] = iter->value();
                found++;
            }
            else if (iter->parType == par_type::VEL_Y && iter->site.compare(station) == 0 &&
                iter->beg == Tbeg && iter->end == Tend)
            {
                vel[1] = iter->value();
                found++;
            }
            else if (iter->parType == par_type::VEL_Z && iter->site.compare(station) == 0 &&
                iter->beg == Tbeg && iter->end == Tend)
            {
                vel[2] = iter->value();
                found++;
            }
        }
        if (found == 3)
            return 1; // all three crd were found
        if (found == 1)
            return -1; // just one crd was found
        if (found == 2)
            return -2; // just two crd was found

        if (found == 0)
            return -3; // crd not found

        return -1;
    }

    std::vector<int> base_allpar::getPartialIndex(const std::string& site, const std::string& sat)
    {
        _update_partial_index();

        std::vector<int> ans;
        std::pair<std::string, std::string> type_list[4] =
        {
            std::make_pair(site, sat),
            std::make_pair(site, ""),
            std::make_pair("", sat),
            std::make_pair("", "") };

        _allpar_mtx.lock();
        for (int i = 0; i < 4; i++)
        {
            ans.insert(ans.end(), _index_for_parital[type_list[i]].begin(), _index_for_parital[type_list[i]].end());
        }
        _allpar_mtx.unlock();

        return ans;
    }

    const base_par& base_allpar::getPar(const int& idx) const
    {
        return _vParam[idx];
    };

    // get single base_par element from container
    // ----------------------------------------------
    base_par& base_allpar::operator[](const size_t idx)
    {
        return _vParam[idx];
    }

    base_allpar base_allpar::operator-(const base_allpar& gallpar)
    {
        base_allpar diff;
        if (this->parNumber() != gallpar.parNumber())
        {
            std::cerr << "base_allpar::operator-: Incompatible dimension ("
                << this->parNumber() << ", " << gallpar.parNumber() << ")"
                << std::endl;
            return diff;
        }

        diff = (*this);
        std::vector<base_par>::const_iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
        {
            int i = gallpar.getParam(iter->site, iter->parType, iter->prn, iter->beg, iter->end);
            if (i >= 0)
            {
                diff[i] = (*iter) - gallpar.getPar(i);
            }
        }
        return diff;
    }

    // Operator +
    // ----------------------------------------
    base_allpar base_allpar::operator+(const base_allpar& gallpar)
    {
        base_allpar diff;

        if (this->parNumber() != gallpar.parNumber())
        {
            std::cerr << "base_allpar::operator+: Incopatible dimension ("
                << this->parNumber() << ", " << gallpar.parNumber() << ")"
                << std::endl;
            return diff;
        }

        diff = (*this);
        std::vector<base_par>::const_iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
        {
            int i = gallpar.getParam(iter->site, iter->parType, iter->prn, iter->beg, iter->end);
            if (i >= 0)
            {
                diff[i] = (*iter) + gallpar.getPar(i);
            }
        }
        return diff;
    }

    void base_allpar::delAllParam()
    {
        _vParam.clear();
        this->_index_par.clear();
        this->_point_par.clear();
        this->_max_point = 0;
        this->_last_point = std::make_pair(0, 0);
    }

    // Restd::set all parems value
    // -------------------------------
    void base_allpar::resetAllParam()
    {

        std::vector<base_par>::iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
            iter->value(0);
    }

    // std::set site name for all pars in gallpar
    // -----------------------------------------
    void base_allpar::setSite(const std::string& site)
    {

        std::vector<base_par>::iterator iter;
        for (iter = _vParam.begin(); iter != _vParam.end(); ++iter)
            iter->site = site;
    }

    int base_allpar::mult(const Matrix& K, base_allpar& mult, base_allpar& res)
    {

        int parN = mult.parNumber();
        if (parN != K.cols())
        {
            std::cerr << "base_allpar::mult - incorrect dimension " << parN << " " << K.cols() << std::endl;
            return -1;
        }

        res = mult;

        double c = 0;
        for (int i = 1; i <= K.rows(); i++)
        {
            for (int j = 1; j <= K.cols(); j++)
            {
                if (mult.getParam(j) < 0)
                {
                    return -1;
                }
                c += K(i, j) * mult[j - 1].value();
            }
            res[i - 1].value(c);
            c = 0;
        }

        return 1;
    }

    std::ostream& operator<<(std::ostream& os, base_allpar& x)
    {
        for (unsigned int i = 0; i < x.parNumber(); i++)
        {
            if (x[i].parType == par_type::AMB_IF ||
                x[i].parType == par_type::AMB_L1 ||
                x[i].parType == par_type::AMB_L2 ||
                x[i].parType == par_type::AMB_L3 ||
                x[i].parType == par_type::AMB_L4 ||
                x[i].parType == par_type::AMB_L5 ||
                x[i].parType == par_type::SION ||
                x[i].parType == par_type::VION ||
                x[i].parType == par_type::IFB_C3 ||
                x[i].parType == par_type::IFB_C4 ||
                x[i].parType == par_type::IFB_C5 ||
                x[i].parType == par_type::GPS_REC_IFB_C3 ||
                x[i].parType == par_type::GPS_REC_IFB_C4 ||
                x[i].parType == par_type::GPS_REC_IFB_C5 ||
                x[i].parType == par_type::SAT_IFB_C3 ||
                x[i].parType == par_type::SAT_IFB_C4 ||
                x[i].parType == par_type::SAT_IFB_C5 ||
                x[i].parType == par_type::IFCB_F3 ||
                x[i].parType == par_type::IFCB_F4 ||
                x[i].parType == par_type::IFCB_F5)
            {
                os << x[i].str_type() << "_" << x[i].prn << " ";
            }
            else
            {
                os << x[i].str_type() << " ";
            }

            if (x[i].parType == par_type::GRD_N || x[i].parType == par_type::GRD_E)
            {
                os << "value: " << x[i].value() * 1000 << " "
                    << "index:" << x[i].index;
            }
            else
            {
                os << "value: " << x[i].value() << " "
                    << "index:" << x[i].index;
            }
            os << std::endl;
        }
        return os;
    }

    // X1 + X2
    //---------------------------------
    int base_allpar::sum(base_allpar& X1, base_allpar& X2)
    {
        if (X1.parNumber() != X2.parNumber())
            return -1;

        for (unsigned int i = 0; i < _vParam.size(); i++)
        {
            int id1 = X1.getParam(_vParam[i].site, _vParam[i].parType, _vParam[i].prn, FIRST_TIME, LAST_TIME);
            int id2 = X2.getParam(_vParam[i].site, _vParam[i].parType, _vParam[i].prn, FIRST_TIME, LAST_TIME);
            if (id1 >= 0 && id2 >= 0)
                _vParam[i].value(X1[id1].value() + X2[id2].value());
            else
                return -1;
        }
        return 1;
    }

    // get Vector w.r.t par indexes
    // ---------------------------
    Vector base_allpar::get_cvect(const base_allpar& par)
    {
        int n = par.parNumber();
        Vector vec(n);

        for (int i = 0; i < n; i++)
        {
            int idx = this->getParam(i);
            vec[i] = _vParam[idx].value();
        }
        return vec;
    }

    void base_allpar::setOrbPar(const std::string& sat, const std::vector<base_par>& par)
    {
        this->_vOrbParam[sat] = par;
    }

    std::vector<base_par> base_allpar::getOrbPar(const std::string& sat)
    {
        return _vOrbParam[sat];
    }

    std::vector<base_par> base_allpar::getAllPar()
    {
        return _vParam;
    }

    std::map<std::string, std::vector<base_par>> base_allpar::getAllOrbPar()
    {
        return _vOrbParam;
    }
    void base_allpar::_update_partial_index()
    {
        _allpar_mtx.lock();
        auto point_now = std::make_pair(_point_par[_point_par.size() - 1], int(_point_par.size() - 1));

        if (point_now != _last_point)
        {
            _last_point = point_now;
            _index_for_parital.clear();
            for (unsigned int i = 0; i < _vParam.size(); i++)
            {
                _index_for_parital[std::make_pair(_vParam[i].site, _vParam[i].prn)].push_back(i);
            }
        }
        _allpar_mtx.unlock();
        return;
    }

    int base_allpar::get_last_idx() {
        return _vParam.size() - 1;
    }
} // namespace
