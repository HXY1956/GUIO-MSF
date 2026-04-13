#include <stdio.h>
#include <math.h>
#include "hwa_gnss_data_obj.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_obj::gnss_data_obj()
        : base_data(),
          _id(""),
          _name(""),
          _overwrite(false)
    {

        id_type(OBJ);
        _pcvnull = 0;
    }

    gnss_data_obj::gnss_data_obj(base_log spdlog)
        : base_data(spdlog),
          _id(""),
          _name(""),
          _overwrite(false)
    {

        id_type(OBJ);
        _pcvnull = 0;
    }

    gnss_data_obj::~gnss_data_obj()
    {
        //_mapantpcv.clear();
        _mappcv.clear();
        _mapeccxyz.clear();
        _mapeccneu.clear();
        _mapant.clear();
        _mapcrd.clear();
        //_params.clear();
    }

    // std::set id
    // ----------
    void gnss_data_obj::id(std::string str)
    {
        _id = str;
    }

    // get name
    // ----------
    std::string gnss_data_obj::id() const
    {
        std::string tmp = _id;
        return tmp;
    }

    // std::set overwrite
    // ----------
    void gnss_data_obj::overwrite(bool overwrite)
    {
        _overwrite = overwrite;
    }

    // get overwrite
    // ----------
    bool gnss_data_obj::overwrite()
    {
        return _overwrite;
    }

    // std::set name
    // ----------
    void gnss_data_obj::name(std::string str)
    {
        _name = str;
        return;
    }

    // get name
    // ----------
    std::string gnss_data_obj::name() const
    {
        std::string tmp = _name;
        return tmp;
    }

    // std::set domes
    // ----------
    void gnss_data_obj::domes(std::string str)
    {
        _domes = str;
        return;
    }

    // get DOMES
    // ----------
    std::string gnss_data_obj::domes() const
    {
        std::string tmp = _domes;
        return tmp;
    }

    // std::set description
    // ----------
    void gnss_data_obj::desc(std::string str)
    {
        _desc = str;
        return;
    }

    // get description
    // ----------
    std::string gnss_data_obj::desc() const
    {
        std::string tmp = _desc;
        return tmp;
    }

    // std::set ecc offsets w.r.t. center of mass/reference point
    // ----------
    void gnss_data_obj::eccxyz(const Triple &ecc, const base_time &beg, const base_time &end)
    {
        _eccxyz(ecc, beg, end);
        return;
    }

    // std::set ecc offsets w.r.t. center of mass/reference point
    // ----------
    void gnss_data_obj::_eccxyz(const Triple &ecc, const base_time &beg, const base_time &end)
    {
        Triple zero(0.0, 0.0, 0.0);
        hwa_map_eccxyz::iterator it = _mapeccxyz.find(beg);

        if (end < beg)
        {
            return;
        }

        // begin record
        if (it == _mapeccxyz.end())
        { // not exists
            _mapeccxyz[beg] = ecc;
        }
        else
        { // record exists
            if (it->first == LAST_TIME ||
                it->second == zero)
            {

                _mapeccxyz[beg] = ecc;
            }
            else
            {
                return;
            }
        }

        // control end of record (with new beg search)
        it = _mapeccxyz.find(beg);
        it++;

        // beg was last in std::map (add final empty record)
        if (it == _mapeccxyz.end())
        {
            _mapeccxyz[end] = zero;
        }
        else
        { // process end according to next value
            if (end < it->first)
            { // only if end is smaller then existing
                if (fabs(it->first - end) > 3600)
                { // significantly smaller!
                    if (it->second == zero)
                        _mapeccxyz.erase(it); // remove obsolete empty record
                    _mapeccxyz[end] = zero;
                }
                else
                { // too close to next record
                    std::string lg = "Warning: object " + _id + " 'eccxyz' end tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, lg);
                }
            }
            else if (end != it->first)
            {
                std::string lg = "Warning: object " + _id + " 'eccxyz' end cut and tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, lg);
            }
        }

        // remove duplicated empty records
        hwa_map_eccxyz::iterator itNEW = _mapeccxyz.begin();
        hwa_map_eccxyz::iterator itOLD = itNEW;
        while (itOLD != _mapeccxyz.end())
        {
            if (++itNEW != _mapeccxyz.end())
            {
                if ((itNEW->second == zero && itOLD->second == zero))
                //          ( itNEW->first == LAST_TIME ) )
                {
                    _mapeccxyz.erase(itNEW++);
                }
            }
            itOLD = itNEW;
        }
        return;
    }

    // get ecc offsets (>=t) w.r.t. center of mass/reference point
    // ----------
    Triple gnss_data_obj::eccxyz(const base_time &t) const
    {
        Triple eccxyz(0.0, 0.0, 0.0);
        eccxyz = this->_eccxyz(t);
        return eccxyz;
    }

    // get ecc offsets (>=t) w.r.t. center of mass/reference point
    // ----------
    Triple gnss_data_obj::_eccxyz(const base_time &t) const
    {
        Triple tmp(0.0, 0.0, 0.0);
        hwa_map_eccxyz::const_iterator it = _mapeccxyz.upper_bound(t);

        if (it != _mapeccxyz.begin())
        {
            it--;
            tmp = it->second;
            return tmp;
        }

        if ((it == _mapeccxyz.begin() || tmp.isZero()) && _mapeccneu.size() > 0)
        { // not found
            if (_mapeccneu.upper_bound(t) == _mapeccneu.begin())
                return tmp;

            Triple eccneu(0.0, 0.0, 0.0);
            eccneu = this->_eccneu(t);

            if (!eccneu.isZero())
            {
                Triple xyz = this->_crd(t);
                Triple ell;
                xyz2ell(xyz, ell, false);
                neu2xyz(ell, eccneu, tmp);
            }
        }

        return tmp;
    }

    // return validity for eccxyx at epoch t
    void gnss_data_obj::eccxyz_validity(const base_time &t, base_time &beg, base_time &end) const
    {

        hwa_map_eccxyz::const_iterator it = _mapeccxyz.upper_bound(t);
        if (it != _mapeccxyz.begin() && it != _mapeccxyz.end())
        {
            end = it->first;
            it--;
            beg = it->first;
        }
    }

    // std::set ecc offsets w.r.t. center of mass/reference point
    // ----------
    void gnss_data_obj::eccneu(const Triple &ecc, const base_time &beg, const base_time &end)
    {
        _eccneu(ecc, beg, end);
        return;
    }

    // std::set ecc offsets w.r.t. center of mass/reference point
    // ----------
    void gnss_data_obj::_eccneu(const Triple &ecc, const base_time &beg, const base_time &end)
    {
        Triple zero(0.0, 0.0, 0.0);
        hwa_map_eccneu::iterator it = _mapeccneu.find(beg);

        if (end < beg)
        {
            return;
        }

        // begin record
        if (it == _mapeccneu.end())
        { // not exists
            _mapeccneu[beg] = ecc;
        }
        else
        { // record exists
            if (it->first == LAST_TIME ||
                it->second == zero)
            {

                _mapeccneu[beg] = ecc;
            }
            else
            {
                return;
            }
        }

        // control end of record (with new beg search)
        it = _mapeccneu.find(beg);
        it++;

        // beg was last in std::map (add final empty record)
        if (it == _mapeccneu.end())
        {
            _mapeccneu[end] = zero;
        }
        else
        { // process end according to next value
            if (end < it->first)
            { // only if end is smaller then existing
                if (fabs(it->first - end) > 3600)
                { // significantly smaller!
                    if (it->second == zero)
                        _mapeccneu.erase(it); // remove obsolete empty record
                    _mapeccneu[end] = zero;
                }
                else
                { // too close to next record
                    std::string lg = "Warning: object " + _id + " 'eccneu' end tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, lg);
                }
            }
            else if (end != it->first)
            {
                std::string lg = "Warning: object " + _id + " 'eccneu' end cut and tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, lg);
            }
        }

        // remove duplicated empty records
        hwa_map_eccneu::iterator itNEW = _mapeccneu.begin();
        hwa_map_eccneu::iterator itOLD = itNEW;
        while (itOLD != _mapeccneu.end())
        {
            if (++itNEW != _mapeccneu.end())
            {
                if ((itNEW->second == zero && itOLD->second == zero))
                //          ( itNEW->first == LAST_TIME ) )
                {
                    _mapeccneu.erase(itNEW++);
                }
            }
            itOLD = itNEW;
        }
        return;
    }

    // get ecc offsets (>=t) w.r.t. center of mass/reference point (interface only)
    // ----------
    Triple gnss_data_obj::eccneu(const base_time &t) const
    {
        Triple eccneu(0.0, 0.0, 0.0);
        eccneu = this->_eccneu(t);
        return eccneu;
    }

    // get ecc offsets (>=t) w.r.t. center of mass/reference point
    // ---------
    Triple gnss_data_obj::_eccneu(const base_time &t) const
    {
        Triple tmp(0.0, 0.0, 0.0);
        hwa_map_eccneu::const_iterator it = _mapeccneu.upper_bound(t);

        if (it != _mapeccneu.begin())
        {
            it--;
            tmp = it->second;
            return tmp;
        }

        if ((it == _mapeccneu.begin() || tmp.isZero()) && _mapeccxyz.size() > 0)
        { // not found
            if (_mapeccxyz.upper_bound(t) == _mapeccxyz.begin())
                return tmp;

            // transformation from XYZ if not available NEU
            Triple eccxyz = this->_eccxyz(t);

            if (!eccxyz.isZero())
            {
                Triple xyz = this->_crd(t);
                xyz2neu(eccxyz, xyz, tmp);
            }
        }

        return tmp;
    }

    // return validity for eccneu at epoch t
    void gnss_data_obj::eccneu_validity(const base_time &t, base_time &beg, base_time &end) const
    {

        hwa_map_eccneu::const_iterator it = _mapeccneu.upper_bound(t);
        if (it != _mapeccneu.begin() && it != _mapeccneu.end())
        {
            end = it->first;
            it--;
            beg = it->first;
        }
    }

    // std::set object position
    // ----------
    void gnss_data_obj::crd(const Triple &crd, const Triple &std, const base_time &beg, const base_time &end, bool overwrite)
    {
        _crd(crd, std, beg, end, overwrite);
        return;
    }

    // std::set object position
    // ----------
    void gnss_data_obj::crd(const Triple &crd, const Triple &std)
    {
        _mapcrd.begin()->second.first = crd;
        _mapcrd.begin()->second.second = std;
        return;
    }

    void gnss_data_obj::clear_crd()
    {
        _mapcrd.clear();
    }

    // std::set object position
    // ----------
    void gnss_data_obj::_crd(const Triple &crd, const Triple &std, const base_time &beg, const base_time &end, bool overwrite)
    {
        std::pair<Triple, Triple> val;
        val = std::make_pair(crd, std);

        std::pair<Triple, Triple> zero;
        Triple nullCRD(0.0, 0.0, 0.0);
        Triple nullSTD(0.0, 0.0, 0.0);
        zero = std::make_pair(nullCRD, nullSTD);

        hwa_map_crd::iterator it = _mapcrd.find(beg);
        if (end < beg)
        {
            std::string lg = "Warning: " + _id + " not valid end time (end<beg) for coordinates (beg:" + beg.str_ymdhms() + " -> end:" + end.str_ymdhms() + ")";

            if (_spdlog)
                SPDLOG_LOGGER_WARN(_spdlog, lg);
            return;
        }
        // begin record
        if (it == _mapcrd.end())
        { // not exists
            _mapcrd[beg] = val;
        }
        else
        { // record exists
            if (it->first == LAST_TIME ||
                it->second == zero || overwrite)
            {
                _mapcrd[beg] = val;
            }
            else
            {
                return;
            }
        }
        // control end of record (with new beg search)
        it = _mapcrd.find(beg);
        it++;

        // beg was last in std::map (add final empty record)
        if (it == _mapcrd.end())
        {
            _mapcrd[end] = zero;
        }
        else
        { // process end according to next value
            if (end < it->first)
            { // only if end is smaller then existing
                if (fabs(it->first - end) > 3600)
                { // significantly smaller!
                    // change by ZHJ (Here not std::set the zero according to the before time��
                    auto beg_it = _mapcrd.find(beg);
                    if (beg_it == _mapcrd.begin())
                    {
                        if (it->second == zero)
                            _mapcrd.erase(it); // remove obsolete empty record
                        _mapcrd[end] = zero;
                    }
                    else
                    {
                        beg_it--;
                        _mapcrd[end] = beg_it->second;
                    }
                }
                else
                { // too close to next record
                    std::string lg = "Warning: object " + _id + " 'crd' end tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, lg);
                }
            }
            else if (end != it->first)
            {
                if (overwrite)
                {

                    bool insert_flag = false;
                    // if can't find end insert
                    if (_mapcrd.find(end) == _mapcrd.end())
                    {
                        _mapcrd.insert(std::make_pair(end, zero));
                        insert_flag = true;
                    }
                    auto end_pre = _mapcrd.find(end);
                    end_pre--;
                    auto beg_post = _mapcrd.find(beg);
                    beg_post++;
                    if (insert_flag)
                    {
                        // assign pre crd info to end
                        _mapcrd[end] = end_pre->second;
                    }

                    // erase [beg_post,end_pre+1)
                    _mapcrd.erase(beg_post, ++end_pre);
                }
                else
                {
                    std::string lg = "Warning: object " + _id + " 'crd' end cut and tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, lg);
                }
            }
        }

        // remove duplicated empty records
        hwa_map_crd::iterator itNEW = _mapcrd.begin();
        hwa_map_crd::iterator itOLD = itNEW;
        while (itOLD != _mapcrd.end())
        {
            if (++itNEW != _mapcrd.end())
            {
                if ((itNEW->second == zero && itOLD->second == zero))
                //          ( itNEW->first == LAST_TIME ) )
                {
                    _mapcrd.erase(itNEW++);
                }
            }
            itOLD = itNEW;
        }

        return;
    }

    // get object position (using eccentricity)
    // ----------
    Triple gnss_data_obj::crd_arp(const base_time &t) const
    {
        Triple marker(0.0, 0.0, 0.0);
        marker = this->_crd(t);

        // applying eccentricity
        Triple arp(0.0, 0.0, 0.0);
        arp = marker + this->_eccxyz(t);
        return arp;
    }

    // interface for _crd(const base_time& t) const
    // ----------
    Triple gnss_data_obj::crd(const base_time &t) const
    {
        Triple marker(0.0, 0.0, 0.0);
        marker = this->_crd(t);
        return marker;
    }

    bool gnss_data_obj::get_recent_crd(const base_time &t, const double &ref_std, Triple &crd, Triple &std)
    {

        if (_mapcrd.size() == 0)
        {
            return false;
        }
        auto find_iter = _mapcrd.upper_bound(t);

        if (find_iter != _mapcrd.begin())
        {
            find_iter--;
        }

        // find by time reverse traverse
        while (find_iter != _mapcrd.begin())
        {
            if (!find_iter->second.first.isZero() && find_iter->second.second.norm() < ref_std)
            {
                crd = find_iter->second.first;
                std = find_iter->second.second;
                return true;
            }
            find_iter--;
        }

        // find_iter is the begin of std::mapcrd
        if (find_iter->second.second.norm() < ref_std)
        {
            crd = find_iter->second.first;
            std = find_iter->second.second;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool gnss_data_obj::get_adjacent_crd(const base_time &t, const double &ref_std, Triple &crd, Triple &std)
    {

        if (_mapcrd.size() == 0)
        {
            return false;
        }
        auto find_iter = _mapcrd.lower_bound(t - 86400 * 7);

        if (find_iter != _mapcrd.begin())
        {
            find_iter--;
        }

        // find by time reverse traverse
        while (find_iter != _mapcrd.begin())
        {
            if (!find_iter->second.first.isZero() && find_iter->second.second.norm() < ref_std)
            {
                crd = find_iter->second.first;
                std = find_iter->second.second;
                return true;
            }
            find_iter--;
        }

        // find_iter is the begin of std::mapcrd
        if (find_iter->second.second.norm() < ref_std)
        {
            crd = find_iter->second.first;
            std = find_iter->second.second;
            return true;
        }
        else
        {
            return false;
        }
    }

    // interface for _std(const base_time& t) const
    // ----------
    Triple gnss_data_obj::std(const base_time &t) const
    {
        Triple marker(0.0, 0.0, 0.0);
        marker = this->_std(t);
        return marker;
    }

    // get object position (without eccentricity correction)
    // ----------
    Triple gnss_data_obj::_crd(const base_time &t) const
    {
#ifdef DEBUG
        for (hwa_map_crd::const_iterator itE = _mapcrd.begin(); itE != _mapcrd.end(); ++itE)
            std::cerr << "OBJ-CRD: searching EPOCH: " << t.str_ymdhms()
                 << "   found: " << itE->first.str_ymdhms() << "\n";
#endif

        Triple crd(0.0, 0.0, 0.0);
        hwa_map_crd::const_iterator it = _mapcrd.upper_bound(t);
        if (it == _mapcrd.begin())
        {
            return crd;
        } // not found
        it--;
        Triple tmp = it->second.first;

        return tmp;
    }

    // get object position std
    // ----------
    Triple gnss_data_obj::_std(const base_time &t) const
    {
#ifdef DEBUG
        for (hwa_map_crd::const_iterator itE = _mapcrd.begin(); itE != _mapcrd.end(); ++itE)
            std::cerr << "OBJ-CRD: searching EPOCH: " << t.str_ymdhms()
                 << "   found: " << itE->first.str_ymdhms() << "\n";
#endif

        Triple std(0.0, 0.0, 0.0);
        hwa_map_crd::const_iterator it = _mapcrd.upper_bound(t);
        if (it == _mapcrd.begin())
        {
            return std;
        } // not found
        it--;
        Triple tmp = it->second.second;

        return tmp;
    }

    // return validity for crd at epoch t
    void gnss_data_obj::crd_validity(const base_time &t, base_time &beg, base_time &end) const
    {

        hwa_map_crd::const_iterator it = _mapcrd.upper_bound(t);
        if (it != _mapcrd.begin())
        {
            end = it->first;
            it--;
            beg = it->first;
        }
    }

    // std::set pcv element
    // ----------
    void gnss_data_obj::pcv(std::shared_ptr<gnss_data_pcv> pcv, const base_time &beg, const base_time &end)
    {
        this->_pcv(pcv, beg, end);
    }

    // std::set pcv element (protected)
    // ----------
    void gnss_data_obj::_pcv(std::shared_ptr<gnss_data_pcv> pcv, const base_time &beg, const base_time &end)
    {

        hwa_map_pcv::iterator it = _mappcv.find(beg);

        if (end < beg)
        {
            return;
        }

        // begin record
        if (it == _mappcv.end())
        {                       // not exists
            _mappcv[beg] = pcv; // ?? POINTER ??

            // record exists
        }
        else if (it->first == LAST_TIME || !it->second)
        {
            _mappcv[beg] = pcv; // ?? POINTER ??
        }
        else
        {
            return;
        }

        // control end of record (with new beg search)
        it = _mappcv.find(beg);
        it++;

        // beg was last in std::map (add final empty record)
        if (it == _mappcv.end())
        {
            _mappcv[end] = _pcvnull;
        }
        else
        { // process end according to next value
            if (end < it->first)
            { // only if end is smaller then existing
                if (fabs(it->first - end) > 3600)
                { // significantly smaller!
                    if (it->second == 0)
                        _mappcv.erase(it); // remove obsolete empty record
                    _mappcv[end] = _pcvnull;
                }
                else
                {
                    // too close to next record
                    std::string lg = "Warning: object " + _id + " 'pcv' end tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, lg);
                }
            }
            else if (end != it->first)
            {
                std::string lg = "Warning: object " + _id + " 'pcv' end cut and tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, lg);
            }
        }

        // remove duplicated empty records
        hwa_map_pcv::iterator itNEW = _mappcv.begin();
        hwa_map_pcv::iterator itOLD = itNEW;
        while (itOLD != _mappcv.end())
        {
            if (++itNEW != _mappcv.end())
            {
                if ((!itNEW->second && !itOLD->second))
                //            ( itNEW->first == LAST_TIME ) )
                {
                    _mappcv.erase(itNEW++);
                }
            }
            itOLD = itNEW;
        }
        //  }
    }

    // get pcv element (>=t)
    // ----------
    std::shared_ptr<gnss_data_pcv> gnss_data_obj::pcv(const base_time &t) const
    {
        std::shared_ptr<gnss_data_pcv> tmp = _pcv(t);
        return tmp;
    }

    // get pcv element (>=t) (protected)
    // ----------
    std::shared_ptr<gnss_data_pcv> gnss_data_obj::_pcv(const base_time &t) const
    {

        std::shared_ptr<gnss_data_pcv> pcv;
        std::string ant;

        hwa_map_ant::const_iterator it = _mapant.upper_bound(t);
        if (it == _mapant.begin())
        {
            ant = ""; // antenna not found

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: unknown PCO (no antenna found in the object " + _id + " ) " + t.str_ymdhms());
            return _pcvnull;
        }
        else
        {
            ant = (--it)->second;
        }

        hwa_map_pcv::const_iterator it2 = _mappcv.upper_bound(t);
        if (it2 == _mappcv.begin())
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: unknown PCO ( antenna " + ant + " not found in ATX ) " + t.str_ymdhms());
            return _pcvnull;
        }
        else
        {
            pcv = (--it2)->second;
        }

        if (pcv->pcvkey().compare(ant) != 0)
        {
            //if (pcv->pcvkey().compare(0, 16, ant, 0, 16) == 0) commented by zhangwei
            if (base_type_conv::trim(pcv->pcvkey().substr(0, 16)) == base_type_conv::trim(ant.substr(0, 16)))
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: PCO Used without considering randome " + pcv->pcvkey() + " " + ant);
            }
            else
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: unknown PCO ( changed antenna " + ant + " not found in ATX ) " + t.str_ymdhms());
#ifdef DEBUG
                std::cout << "gobj RESET: ant [" << ant << "] [" << pcv->pcvkey() << " " << t.str_ymdhms(" epo:") << "]\n";
#endif
                return _pcvnull;
            }
        }

        return pcv;
    }

    // std::set antenna name
    // ----------
    void gnss_data_obj::ant(std::string ant, const base_time &beg, const base_time &end)
    {
        _ant(ant, beg, end);
        return;
    }

    // std::set antenna name
    // ----------
    void gnss_data_obj::_ant(std::string ant, const base_time &beg, const base_time &end)
    {
        hwa_map_ant::iterator it = _mapant.find(beg);

        if (end < beg)
        {
            return;
        }

        // begin record
        if (it == _mapant.end())
        { // not exists
            _mapant[beg] = ant;
        }
        else
        { // last value
            if (it->first == LAST_TIME ||
                it->second.empty())
            {

                _mapant[beg] = ant;
            }
            else
            { // record exists
                return;
            }
        }

        // control end of record (with new beg search)
        it = _mapant.find(beg);
        it++;

        // beg was last in std::map (add final empty record)
        if (it == _mapant.end())
        {
            _mapant[end] = "";
        }
        else
        { // process end according to next value
            if (end < it->first)
            { // only if end is smaller then existing
                if (fabs(it->first - end) > 3600)
                { // significantly smaller!
                    if (it->second.empty())
                    {
                        _mapant.erase(it); // remove obsolete empty record
                        _mapant[end] = "";
                    }
                    else
                    {
                        _mapant[end] = it->second;
                    }
                }
                else
                { // too close to next record
                    std::string lg = "Warning: object " + _id + " 'obj' end tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, lg);
                }
            }
            else if (end != it->first)
            {
                std::string lg = "Warning: object " + _id + " 'ant' " + ant + " end cut and tied to the existing value " + end.str("%Y-%m-%d %H:%M:%S -> ") + it->first.str("%Y-%m-%d %H:%M:%S");

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, lg);
            }
        }

        // remove duplicated empty records
        hwa_map_ant::iterator itNEW = _mapant.begin();
        hwa_map_ant::iterator itOLD = itNEW;
        while (itOLD != _mapant.end())
        {
            if (++itNEW != _mapant.end())
            {
                if ((itNEW->second.empty() && itOLD->second.empty()))
                //          ( itNEW->first == LAST_TIME ) )
                {
                    _mapant.erase(itNEW++);
                }
            }
            itOLD = itNEW;
        }

        return;
    }

    // get antenna name (>=t)
    // ----------
    std::string gnss_data_obj::ant(const base_time &t) const
    {
        std::string tmp = this->_ant(t);
        return tmp;
    }

    // get antenna name (>=t) (protected)
    // ----------
    std::string gnss_data_obj::_ant(const base_time &t) const
    {

        hwa_map_ant::const_iterator it = _mapant.upper_bound(t);
        if (it == _mapant.begin())
            return ""; // not found
        it--;
        return it->second;
    }

    // get time tags
    // ----------
    /*std::vector<base_time> gnss_data_obj::ecc_id() const
{
  std::vector<base_time> tmp;
  hwa_map_eccxyz::const_iterator itMAP = _mapeccxyz.begin();
  while( itMAP != _mapecc.end() ){
    tmp.push_back( itMAP->first );
    itMAP++;
  }
  return tmp;
}
*/

    // return validity for antenna at epoch t
    void gnss_data_obj::ant_validity(const base_time &t, base_time &beg, base_time &end) const
    {

        hwa_map_ant::const_iterator it = _mapant.upper_bound(t);
        if (it != _mapant.begin() && it != _mapant.end())
        {
            end = it->first;
            it--;
            beg = it->first;
        }
    }

    // get time tags
    // ----------
    std::vector<base_time> gnss_data_obj::pcv_id() const
    {
        std::vector<base_time> tmp;
        //  hwa_map_antpcv::const_iterator itant = _mapantpcv.find(ant);

        //  if (itant == _mapantpcv.end()){
        //     return tmp;
        //  }else{
        hwa_map_pcv::const_iterator itMAP = _mappcv.begin();
        while (itMAP != _mappcv.end())
        {
            tmp.push_back(itMAP->first);
            itMAP++;
        }
        return tmp;
        //  }
    }

    // get time tags
    // ----------
    std::vector<base_time> gnss_data_obj::ant_id() const
    {
        std::vector<base_time> tmp = this->_ant_id();
        return tmp;
    }

    // get time tags (protected)
    // ----------
    std::vector<base_time> gnss_data_obj::_ant_id() const
    {

        std::vector<base_time> tmp;
        hwa_map_ant::const_iterator itMAP = _mapant.begin();
        while (itMAP != _mapant.end())
        {
            tmp.push_back(itMAP->first);
            itMAP++;
        }
        return tmp;
    }

    // get time tags
    // ----------
    std::vector<base_time> gnss_data_obj::crd_id() const
    {
        std::vector<base_time> tmp;
        hwa_map_crd::const_iterator itMAP = _mapcrd.begin();
        while (itMAP != _mapcrd.end())
        {
            tmp.push_back(itMAP->first);
            itMAP++;
        }
        return tmp;
    }

    // std::set mappcv for all gnss_data_obj
    // -----------------------
    void gnss_data_obj::sync_pcv(gnss_all_pcv *pcvs)
    {

        if (pcvs == 0)
            return;
        std::vector<base_time> vant = this->_ant_id();

        if (vant.size() == 0)
        { // ant in obj is not std::set
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ant in obj is not std::set");
        }
        else
        {
            for (std::vector<base_time>::iterator it = vant.begin(); it != vant.end(); it++)
            {

                // std::set pcv for all antennas
                std::string ant = this->_ant(*it);
                base_time epo = *it;

                std::shared_ptr<gnss_data_pcv> gpcv = pcvs->gpcv(ant, "*", epo);
                if (gpcv != _pcvnull)
                {
                    this->_pcv(gpcv, epo);
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: unknown PCO ( antenna " + ant + " not found in ATX ) " + epo.str_ymdhms());
                }
            }
        }
        return;
    }

    // check consistency
    // ----------
    void gnss_data_obj::compare(std::shared_ptr<gnss_data_obj> gobj, const base_time &tt, std::string source)
    {
        std::string old, alt;
        Triple trip_old, trip_alt;
        Triple std_old, std_alt;

        // Name
        old = base_type_conv::trim(_name);
        alt = base_type_conv::trim(gobj->name());

        if (old != alt && !alt.empty())
        {
            if (old.empty())
            {
                _name = alt;

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " completed by " + source + " (Domes): " + alt + " (" + tt.str_ymdhms() + ")");
            }
            else if (_overwrite)
            {
                _name = alt;

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " modified  by " + source + " (Domes): " + old + " -> " + alt + " (" + tt.str_ymdhms() + ")");
            }
            else
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " does not match " + source + " (Domes): " + old + " -> " + alt + " (" + tt.str_ymdhms() + ")");
            }
        }

        // Domes
        old = base_type_conv::trim(_domes);
        alt = base_type_conv::trim(gobj->domes());

        if (old != alt && !alt.empty())
        {
            if (old.empty())
            {
                _domes = alt;

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " completed by " + source + " (Domes): " + alt + " (" + tt.str_ymdhms() + ")");
            }
            else if (_overwrite)
            {
                _domes = alt;

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " modified  by " + source + " (Domes): " + old + " -> " + alt + " (" + tt.str_ymdhms() + ")");
            }
            else
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " does not match " + source + " (Domes): " + old + " -> " + alt + " (" + tt.str_ymdhms() + ")");
            }
        }

        // Antenna
        old = base_type_conv::trim(_ant(tt));
        alt = base_type_conv::trim(gobj->ant(tt));
        base_time beg, end;
        gobj->ant_validity(tt, beg, end);

        if (old != alt && !alt.empty())
        {
            if (old.empty())
            {
                _ant(alt, beg, end);
                this->ant_validity(tt, beg, end);

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " completed by " + source + " (Antenna): " + alt + " (" + beg.str_ymdhms() + "->" + end.str_ymdhms() + ")");
            }
            else if (_overwrite)
            {
                _ant(alt, beg, end);

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " modified  by " + source + " (Antenna): " + old + " -> " + alt + " (" + beg.str_ymdhms() + "->" + end.str_ymdhms() + ")");
            }
            else
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " setting does not match " + source + " (Antenna): " + old + " -> " + alt + " (" + tt.str_ymdhms() + ")");
            }
        }

        // Coordinates
        trip_old = _crd(tt);
        trip_alt = gobj->crd(tt);
        std_old = _std(tt);
        std_alt = gobj->std(tt);
        gobj->crd_validity(tt, beg, end);

        Triple diff = trip_old - trip_alt;
        double dist = diff.norm();
        if ((dist > 10 || compareL(std_alt,std_old)) && !trip_alt.isZero())
        {
            if (trip_old.isZero() || compareL(std_alt,std_old))
            {
                _crd(trip_alt, std_alt, beg, end, true);
                this->crd_validity(tt, beg, end);

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " completed by " + source + " (Coordinates): " + base_type_conv::dbl2str(trip_alt[0]) + " " + base_type_conv::dbl2str(trip_alt[1]) + " " + base_type_conv::dbl2str(trip_alt[2]) + " (" + beg.str_ymdhms() + "->" + end.str_ymdhms() + ")");
            }
            else if (_overwrite)
            {
                _crd(trip_alt, std_alt, beg, end, true);

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " modified  by " + source + " (Coordinates): " + base_type_conv::dbl2str(trip_old[0]) + " " + base_type_conv::dbl2str(trip_old[1]) + " " + base_type_conv::dbl2str(trip_old[2]) + " -> " + base_type_conv::dbl2str(trip_alt[0]) + " " + base_type_conv::dbl2str(trip_alt[1]) + " " + base_type_conv::dbl2str(trip_alt[2]) + " (" + beg.str_ymdhms() + "->" + end.str_ymdhms() + ")");
            }
            else
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: object " + _id + " does not match " + source + " (Coordinates)" + " (" + tt.str_ymdhms() + ")");
            }
        }

        // NEU eccentricity
        trip_old = _eccneu(tt);
        trip_alt = gobj->eccneu(tt);
        gobj->eccneu_validity(tt, beg, end);

        if (trip_old != trip_alt && !trip_alt.isZero())
        {
            if (trip_old.isZero())
            {
                _eccneu(trip_alt, beg, end);
                this->eccneu_validity(tt, beg, end);
            }
            else if (_overwrite)
            {
                _eccneu(trip_alt, beg, end);
            }
            else
            {
            }
        }

        // xyz eccentricity
        trip_old = _eccxyz(tt);
        trip_alt = gobj->eccxyz(tt);
        gobj->eccxyz_validity(tt, beg, end);

        if (trip_old != trip_alt && !trip_alt.isZero())
        {
            if (trip_old.isZero())
            {
                _eccxyz(trip_alt, beg, end);
                this->eccxyz_validity(tt, beg, end);
            }
            else if (_overwrite)
            {
                _eccxyz(trip_alt, beg, end);
            }
            else
            {
            }
        }
        return;
    }

    // operator for sorting
    // ----------
    bool gnss_data_obj::operator<(const gnss_data_obj &t) const
    {
        return _name < t.name();
    }

    // operator for sorting
    // ----------
    bool gnss_data_obj::operator==(const gnss_data_obj &t) const
    {
        return _name == t.name();
    }

} // namespace
