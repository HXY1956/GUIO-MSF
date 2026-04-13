#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "hwa_gnss_all_bias.h"

namespace hwa_gnss
{
    gnss_all_bias::gnss_all_bias() : base_data(), _isOverWrite(false)
    {
        id_type(base_data::ALLBIAS);
        id_group(base_data::GRP_model);

        _acOrder["COD_A"] = 1;
        _acOrder["CAS_A"] = 2;
        _acOrder["WHU_A"] = 3;
        _acOrder["DLR_A"] = 4;
        _acOrder["CAS_R"] = 5;
        _acOrder["COD_R"] = 6;
        _acOrder["WHU_R"] = 7;
        _acOrder["DLR_R"] = 8;
        _acOrder["CNT_A"] = 9;
        _acOrder["RTB_A"] = 10;
    }
    // constructor
    // ----------
    gnss_all_bias::gnss_all_bias(base_log spdlog)
        : base_data(spdlog), _isOverWrite(false)
    {
        id_type(base_data::ALLBIAS);
        id_group(base_data::GRP_model);

        _acOrder["COD_A"] = 1;
        _acOrder["CAS_A"] = 2;
        _acOrder["WHU_A"] = 3;
        _acOrder["DLR_A"] = 4;
        _acOrder["CAS_R"] = 5;
        _acOrder["COD_R"] = 6;
        _acOrder["WHU_R"] = 7;
        _acOrder["DLR_R"] = 8;
        _acOrder["CNT_A"] = 9;
        _acOrder["RTB_A"] = 10;
    }

    // destructor
    // ----------
    gnss_all_bias::~gnss_all_bias()
    {
        _mapBias.clear();
    }

    // add satellite bias
    // ----------
    void gnss_all_bias::add(const std::string &ac, const base_time &epo, const std::string &obj, hwa_spt_bias pt_cb)
    {
        if (pt_cb == nullptr)
        {
            return;
        }

        if (pt_cb->ref() == X)
        { // When ref GOBS is X => bias is expressed in absolute sense
            _mapBias[ac][epo][obj][pt_cb->gobs()] = pt_cb;
        }
        else
        { // Differential biases need to be splitted
            if (_mapBias[ac][epo][obj].size() == 0)
            {
                std::shared_ptr<gnss_data_bias> pt_ref = std::make_shared<gnss_data_bias>(_spdlog);               // create first reference bias
                pt_ref->set(pt_cb->beg(), pt_cb->end(), 0.0, pt_cb->ref(), pt_cb->ref()); // reference bias is std::set up to zero
                _mapBias[ac][epo][obj][pt_ref->gobs()] = pt_ref;                          // store new bias (reference)
                _mapBias[ac][epo][obj][pt_cb->gobs()] = pt_cb;                            // store new bias
            }
            else
            {
                hwa_spt_bias pt_obs1 = _find(ac, epo, obj, pt_cb->gobs());
                hwa_spt_bias pt_obs2 = _find(ac, epo, obj, pt_cb->ref());
                if (pt_obs1 != nullptr && pt_obs2 == nullptr)
                { // connection with first signal
                    _connect_first(pt_obs1, pt_cb);
                    _mapBias[ac][epo][obj][pt_cb->gobs()] = pt_cb; // store modified bias
                }
                else if (pt_obs1 == nullptr && pt_obs2 != nullptr)
                { // connection with second signal
                    _connect_second(pt_obs2, pt_cb);
                    _mapBias[ac][epo][obj][pt_cb->gobs()] = pt_cb; // store modified bias
                }
                else if (pt_obs1 != nullptr && pt_obs2 != nullptr)
                {
                    // connectin two groups with different reference signal
                    // WARNING!!! - this case has not been tested (not happen in tested files)
                    // connection with first signal
                    _connect_first(pt_obs1, pt_cb);
                    // all biases connected with second signal need to be consolidated
                    _consolidate(ac, obj, pt_cb, pt_obs2);
                }
                else
                {
                    // glfeng add for GAL to store Q & X DCB bias
                    std::shared_ptr<gnss_data_bias> pt_ref = std::make_shared<gnss_data_bias>(_spdlog);               // create first reference bias
                    pt_ref->set(pt_cb->beg(), pt_cb->end(), 0.0, pt_cb->ref(), pt_cb->ref()); // reference bias is std::set up to zero
                    _mapBias[ac][epo][obj][pt_ref->gobs()] = pt_ref;                          // store new bias (reference)
                    _mapBias[ac][epo][obj][pt_cb->gobs()] = pt_cb;                            // store new bias
                }
            }
        }
        return;
    }

    // get undifferenced bias element
    // Note the variable 'meter' means in the original unit: meter for RTCM format, cycle for SGG format
    // ------------------------------
    double gnss_all_bias::get(const std::string &prd, const base_time &epo, const std::string &prn, const GOBS &gobs, const bool &meter)
    {
        double bias = 999.0;
        auto itAC = _mapBias.find(prd);
        if (itAC != _mapBias.end())
        {
            auto itEPO = itAC->second.upper_bound(epo);
            if (itEPO != itAC->second.begin() && itEPO != itAC->second.end())
                itEPO--; // between epochs
            if (itEPO == itAC->second.end() && itAC->second.size() != 0)
                itEPO--; // no epochs

            if (itEPO != itAC->second.end())
            {
                auto itSAT = itEPO->second.find(prn);
                if (itSAT != itEPO->second.end() && itSAT->second.find(gobs) != itSAT->second.end())
                {
                    hwa_spt_bias pobs1 = itSAT->second.find(gobs)->second;
                    bias = pobs1->bias();
                }
            }
        }
        return bias;
    }

    std::vector<std::string> gnss_all_bias::get_ac()
    {
        std::vector<std::string> ac_list;
        for (const auto &item : _mapBias)
        {
            ac_list.push_back(item.first);
        }
        return ac_list;
    }

    std::string gnss_all_bias::get_ac_priority()
    {
        std::string used_ac;
        int loc = 999;
        const std::map<std::string, int> ac_order{
            {"COD_A", 1}, {"CAS_A", 2}, {"WHU_A", 3}, {"DLR_A", 4}, {"CAS_R", 5}, {"COD_R", 6}, {"WHU_R", 7}, {"DLR_R", 8}, {"CNT_A", 9}, {"RTB_A", 10}, {"SGG_A", 11}};
        for (auto item : _mapBias)
        {
            std::string ac = (item.first == "WHU_A_PHASE") ? "WHU_A" : item.first;
            if (ac_order.at(ac) < loc)
            {
                used_ac = ac;
                loc = ac_order.at(item.first);
            }
        }
        return used_ac;
    }

    bool gnss_all_bias::is_osb()
    {
        std::string used_ac = get_used_ac();
        if (used_ac.find("_A") != used_ac.npos)
            return true;
        else
            return false;
    }

    void gnss_all_bias::set_used_ac(const std::string &ac)
    {
        _acUsed = ac;
    }

    std::string gnss_all_bias::get_used_ac()
    {
        if (_acUsed.empty())
            _acUsed = get_ac_priority();
        return _acUsed;
    }

    // get single code bias
    // -------------------------------
    double gnss_all_bias::get(const base_time &epo, const std::string &obj, const GOBS &gobs1, const GOBS &gobs2, const std::string &tmp)
    {
        double dcb = 0.0;
        std::string ac(tmp);
        // jdhuang fix
        if (ac == "" && _isOrdered == true)
            ac = _acPri;

        if (ac == "" && _isOrdered == false)
        {
            int loc = 999;
            for (const auto &item : _mapBias)
            {
                if (_acOrder.at(item.first) < loc)
                {
                    ac = item.first;
                    loc = _acOrder.at(item.first);
                }
            }

            _isOrdered = true;
            _acPri = ac;
        }

        if (gobs2 == gobs1)
        {
            GOBS gobs1_convert = gobs1;
            this->_convert_obstype(ac, obj, gobs1_convert);
            hwa_spt_bias pobs1 = _find(ac, epo, obj, gobs1_convert);
            if (pobs1 != nullptr)
            {
                dcb = pobs1->bias();
            }
        }
        else
        {
            GOBS gobs1_convert = gobs1;
            GOBS gobs2_convert = gobs2;

            // align obstype to ac(code or cas)
            this->_convert_obstype(ac, obj, gobs1_convert);
            this->_convert_obstype(ac, obj, gobs2_convert);

            hwa_spt_bias pobs1 = _find(ac, epo, obj, gobs1_convert);
            hwa_spt_bias pobs2 = _find(ac, epo, obj, gobs2_convert);

            if (pobs1 != nullptr && pobs2 != nullptr && pobs1->ref() == pobs2->ref())
            {
                dcb = pobs1->bias() - pobs2->bias();
            }
        }
        return dcb;
    }

    // get single code bias
    // -------------------------------
    hwa_spt_bias gnss_all_bias::_find(const std::string &ac, const base_time &epo, const std::string &obj, const GOBS &gobs)
    {
        hwa_spt_bias pt_bias = nullptr;

        auto itAC = _mapBias.find(ac);
        if (itAC != _mapBias.end())
        {
            auto itEPO = itAC->second.upper_bound(epo);
            if (itEPO != itAC->second.begin() && itEPO != itAC->second.end())
                itEPO--; // between epochs
            if (itEPO == itAC->second.end() && itAC->second.size() != 0)
                itEPO--; // no epochs

            if (itEPO != itAC->second.end())
            {
                auto itOBJ = itEPO->second.find(obj);
                if (itOBJ != itEPO->second.end())
                {
                    auto itGOBS = itOBJ->second.find(gobs);
                    if (itGOBS != itOBJ->second.end())
                    {
                        if (itGOBS->second->valid(epo))
                        {
                            pt_bias = itGOBS->second;
                        }
                    }
                }
            }
        }

        return pt_bias;
    }

    // get all biases with particular reference singal
    // -------------------------------
    std::vector<hwa_spt_bias> gnss_all_bias::_find_ref(const std::string &ac, const base_time &epo, const std::string &obj, const GOBS &ref)
    {
        std::vector<hwa_spt_bias> vec_bias;

        auto itAC = _mapBias.find(ac);
        if (itAC != _mapBias.end())
        {
            auto itEPO = itAC->second.find(epo);
            if (itEPO != itAC->second.end())
            {
                auto itOBJ = itEPO->second.find(obj);
                if (itOBJ != itEPO->second.end())
                {
                    for (auto itGOBS = itOBJ->second.begin(); itGOBS != itOBJ->second.end(); itGOBS++)
                    {
                        if (itGOBS->second->ref() == ref)
                            vec_bias.push_back(itGOBS->second);
                    }
                }
            }
        }

        return vec_bias;
    }

    void gnss_all_bias::_convert_obstype(const std::string &ac, const std::string &obj, GOBS &obstype)
    {
        if (ac == "COD_R")
        {
            // GPS GLO //xjhan
            if (obj[0] == 'G' || obj[0] == 'R' || obj[0] == '2' || obj[0] == '3' || obj[0] == '4' || obj[0] == '5')
            {
                switch (obstype)
                {
                case C1C:
                    obstype = C1;
                    break;
                case C1P:
                case C1Y:
                case C1W:
                    obstype = P1;
                    break;
                case C2C:
                    obstype = C2;
                    break;
                case C2P:
                case C2Y:
                case C2W:
                    obstype = P2;
                    break;
                default:
                    break;
                }
            }
        }
        else if (ac == "CAS_R")
        {
            // GPS //xjhan
            if (obj[0] == 'G' || obj[0] == '2' || obj[0] == '3' || obj[0] == '4' || obj[0] == '5')
            {
                switch (obstype)
                {
                case P1:
                    obstype = C1W;
                    break;
                case P2:
                    obstype = C2W;
                    break;
                case C1:
                    obstype = C1C;
                    break;
                case C2:
                    obstype = C2C;
                    break;
                default:
                    break;
                }
            }
            // GLO
            if (obj[0] == 'R')
            {
                switch (obstype)
                {
                case P1:
                    obstype = C1P;
                    break;
                case P2:
                    obstype = C2P;
                    break;
                case C1:
                    obstype = C1C;
                    break;
                case C2:
                    obstype = C2C;
                    break;
                default:
                    break;
                }
            }
        }
    }

    // Connect DCB pt_cb2 with first GOBS
    void gnss_all_bias::_connect_first(const hwa_spt_bias &pt_cb1, const hwa_spt_bias &pt_cb2)
    {
        double newval = pt_cb1->bias() - pt_cb2->bias();
        pt_cb2->set(newval, pt_cb2->ref(), pt_cb1->ref());
    }

    // Connect DCB pt_cb2 with second GOBS
    void gnss_all_bias::_connect_second(const hwa_spt_bias &pt_cb1, const hwa_spt_bias &pt_cb2)
    {
        double newval = pt_cb1->bias() + pt_cb2->bias();
        pt_cb2->set(newval, pt_cb2->gobs(), pt_cb1->ref());
    }

    // Consolidate all biases with reference signal of pt_cb2
    void gnss_all_bias::_consolidate(const std::string &ac, const std::string &obj, const hwa_spt_bias &pt_cb1, const hwa_spt_bias &pt_cb2)
    {
        //double diff = pt_cb2->ref() - pt_cb1->ref();
        double diff = pt_cb2->val() - pt_cb1->val();
        base_time epo = pt_cb1->beg();
        std::vector<hwa_spt_bias> vec = _find_ref(ac, epo, obj, pt_cb2->ref());

        for (auto itSPT = vec.begin(); itSPT != vec.end(); itSPT++)
        {
            double newval = (*itSPT)->bias() - diff;
            GOBS gobs = (*itSPT)->gobs();
            GOBS newref = pt_cb1->ref();
            (*itSPT)->set(newval, gobs, newref);
        }
    }

    // clean rtcm & gnav
    // ----------
    void gnss_all_bias::clean_outer(const base_time &beg, const base_time &end)
    {

        if (end < beg)
            return;
        // biases - loop over all satellites
        // -----------------------------------------
        auto itAC = _mapBias.begin();
        while (itAC != _mapBias.end())
        {
            std::string acs = itAC->first;

            // find and CLEAN all data (epochs) out of the specified period !
            auto itFirst = _mapBias[acs].begin();
            auto itLast = _mapBias[acs].end();
            auto itBeg = _mapBias[acs].lower_bound(beg); // greater only!
            auto itEnd = _mapBias[acs].upper_bound(end); // greater only!

            if (itBeg != itFirst)
            {

                // begin is last
                if (itBeg == itLast)
                {
                    itBeg--;
                    _mapBias[acs].erase(itFirst, itLast);

                    // begin is not last
                }
                else
                {
                    _mapBias[acs].erase(itFirst, itBeg);
                }
            }

            // remove after END request
            if (itEnd != itLast)
            {
                _mapBias[acs].erase(itEnd, itLast);
            }
            itAC++;
        }
#ifdef BMUTEX
        lock.unlock();
#endif
        return;
    }

    void gnss_all_bias::add_bia_intv(const int &intv)
    {
        _udbiaInt = intv;
    }

    bool gnss_all_bias::bias_avail(const base_time &now)
    {
        if (_udbiaInt == 99999 || _acUsed == "")
            return true;
        else
        {
            if (_mapBias[_acUsed].begin() != _mapBias[_acUsed].end())
            {
                auto itEPO = _mapBias[_acUsed].end();
                itEPO--;
                if (itEPO->first > now)
                    return true;
                else
                    return false;
            }
        }

        return true;
    }

} // namespace
