#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "hwa_gnss_data_navgps.h"
#include "hwa_gnss_data_navglo.h"
#include "hwa_gnss_data_navgal.h"
#include "hwa_gnss_data_navqzs.h"
#include "hwa_gnss_data_navsbs.h"
#include "hwa_gnss_data_navbds.h"
#include "hwa_gnss_data_navirn.h"
#include "hwa_gnss_all_Nav.h"
#include "hwa_gnss_sys.h"
#include "hwa_base_timeSYNC.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_fileconv.h"
#include "hwa_gnss_stat.h"

using namespace hwa_base;
using namespace std;

namespace hwa_gnss
{

    // constructor
    // ----------
    gnss_all_nav::gnss_all_nav()
        : base_data(),
          _com(false),
          _offset(0),
          _nepoch(base_time::GPS),
          _multimap(false),
          _overwrite(false),
          _chkHealth(true), // jdhuang : change true to false
          _chkNavig(true),
          _chkTot(false)
    {

        id_type(base_data::ALLNAV);
        id_group(base_data::GRP_EPHEM);
    }

    gnss_all_nav::gnss_all_nav(base_log spdlog)
        : base_data(spdlog),
          _com(false),
          _offset(0),
          _nepoch(base_time::GPS),
          _multimap(false),
          _overwrite(false),
          _chkHealth(true), // jdhuang : change true to false
          _chkNavig(true),
          _chkTot(false)
    {
        id_type(base_data::ALLNAV);
        id_group(base_data::GRP_EPHEM);
    }
    // destructor
    // ----------
    gnss_all_nav::~gnss_all_nav()
    {

        _mapsat.clear();
    }

    // return gnav element
    // ----------
    std::shared_ptr<gnss_data_eph> gnss_all_nav::find(const std::string &sat, const base_time &t, const bool &chk_mask)
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t, _chkHealth && chk_mask);
        return tmp;
    };

    // return gnav element
    // ----------
    std::shared_ptr<gnss_data_eph> gnss_all_nav::find(const std::string &sat, const int &iod, const base_time &t, const bool &chk_mask)
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, iod, t, _chkHealth && chk_mask);
        return tmp;
    };

    // return gnav elements
    // ----------
    std::vector<std::shared_ptr<gnss_data_eph>> gnss_all_nav::find_mult(const std::string &sat, const base_time &t) const
    {
        std::vector<std::shared_ptr<gnss_data_eph>> vec = this->_find_mult(sat, t);
        return vec;
    }

    // return position
    // ----------
    int gnss_all_nav::pos(const std::string &sat,
                       const base_time &t,
                       double xyz[3],
                       double var[3],
                       double vel[3],
                       const bool &chk_mask) // [m]
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t, _chkHealth && chk_mask);
        // added by yqyuan
        if (tmp == _null)
        {
            tmp = gnss_all_nav::_find(sat, t, false);
        }
        // end adding
        if (tmp == _null)
        {
            for (int i = 0; i < 3; i++)
            {
                xyz[i] = 0.0;
                if (var)
                    var[i] = 0.0;
                if (vel)
                    vel[i] = 0.0;
            }
            return -1;
        }

        int irc = tmp->pos(t, xyz, var, vel, _chkHealth && chk_mask);
        // added by yqyuan
        if (irc == -1)
        {
            irc = tmp->pos(t, xyz, var, vel, false);
        }
        // end adding
        return irc;

        //  return find( sat, t )->pos( t, xyz, var, vel, _chk_health && chk_mask );
    }

    // return position
    // ----------
    int gnss_all_nav::pos(const std::string &sat,
                       const int &iod,
                       const base_time &t,
                       double xyz[3],
                       double var[3],
                       double vel[3],
                       const bool &chk_mask) // [m]
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, iod, t, _chkHealth && chk_mask);
        // added by yqyuan
        if (tmp == _null)
        {
            tmp = gnss_all_nav::_find(sat, iod, t, false);
        }
        // end adding
        if (tmp == _null)
        {
            for (int i = 0; i < 3; i++)
            {
                xyz[i] = 0.0;
                if (var)
                    var[i] = 0.0;
                if (vel)
                    vel[i] = 0.0;
            }
            return -1;
        }

        int irc = tmp->pos(t, xyz, var, vel, _chkHealth && chk_mask);
        // added by yqyuan
        if (irc == -1)
        {
            irc = tmp->pos(t, xyz, var, vel, false);
        }
        // end adding
        return irc;

        //  return find( sat, t )->pos( t, xyz, var, vel, _chk_health && chk_mask );
    }

    // return satellite health
    // ----------
    bool gnss_all_nav::health(const std::string &sat, const base_time &t)
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t, false);

        if (tmp == _null)
        {
            return false;
        }

        bool status = tmp->healthy();
        return status;
    }

    // return position aka navigation
    // ----------
    int gnss_all_nav::nav(const std::string &sat,
                       const base_time &t,
                       double xyz[3],
                       double var[3],
                       double vel[3],
                       const bool &chk_mask) // [m]
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t, _chkHealth && chk_mask);

        if (tmp == _null)
        {
            for (int i = 0; i < 3; i++)
            {
                xyz[i] = 0.0;
                if (var)
                    var[i] = 0.0;
                if (vel)
                    vel[i] = 0.0;
            }
            return -1;
        }
        int irc = tmp->nav(t, xyz, var, vel, _chkHealth && chk_mask);
        return irc;

        //  return find( sat, t )->pos( t, xyz, var, vel, _chk_health && chk_mask );
    }

    // return clock corrections
    // ----------
    int gnss_all_nav::clk(const std::string &sat,
                       const base_time &t,
                       double *clk,
                       double *var,
                       double *dclk,
                       const bool &chk_mask) // [s]
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t, _chkHealth && chk_mask);

        if (tmp == _null)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            return -1;
        }

        int irc = tmp->clk(t, clk, var, dclk, _chkHealth && chk_mask);
        return irc;

        //  return this->find( sat, t )->clk( t, clk, var, dclk, _chk_health && chk_mask );
    }

    // return clock corrections
    // ----------
    int gnss_all_nav::clk(const std::string &sat, const int &iod, const base_time &t, double *clk,
                       double *var, double *dclk, const bool &chk_mask) // [s]
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, iod, t, _chkHealth && chk_mask);

        if (tmp == _null)
        {
            *clk = 0.0;
            if (var)
                *var = 0.0;
            if (dclk)
                *dclk = 0.0;
            return -1;
        }

        int irc = tmp->clk(t, clk, var, dclk, _chkHealth && chk_mask);
        return irc;

        //  return this->find( sat, t )->clk( t, clk, var, dclk, _chk_health && chk_mask );
    }

    int gnss_all_nav::clk_ifcb(const std::string &sat, const base_time &t, double *clk, double *var, double *dclk, const bool &chk_mask)
    {

        return 1;
    }

    int gnss_all_nav::get_pos_clk_correction(const std::string &sat, const base_time &t, const int &iod, double *xyz, double *vxyz, double &clk, double &dclk)
    {
        return -1;
    }

    //int gnss_all_nav::get_iod(const std::string & sat, const base_time & t)
    std::pair<int, int> gnss_all_nav::get_iod(const std::string &sat, const base_time &t)
    {
        std::pair<int, int> iod = std::make_pair(-1, -1);

        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t);

        double maxdiff = gnss_data_nav::nav_validity(gnss_sys::str2gsys(sat.substr(0, 1))) * 1.1;

        if (tmp == _null)
        {
            if (_ephLast.find(sat) != _ephLast.end() && t <= _ephLast[sat]->epoch() + maxdiff)
            {
                iod.first = std::dynamic_pointer_cast<gnss_data_nav>(_ephLast[sat])->iod(); // Last eph
            }
            if (_ephPrev.find(sat) != _ephPrev.end() && t <= _ephPrev[sat]->epoch() + maxdiff)
            {
                iod.second = std::dynamic_pointer_cast<gnss_data_nav>(_ephPrev[sat])->iod(); // Prev eph
            }
        }
        else
        {
            if (_ephLast.find(sat) == _ephLast.end()) // first time
            {
                _ephLast[sat] = tmp;
                _ephPrev[sat] = _ephLast[sat];
            }
            else if (_ephLast[sat]->epoch() < tmp->epoch()) // update eph
            {
                _ephPrev[sat] = _ephLast[sat];
                _ephLast[sat] = tmp;
            }

            if (t <= _ephLast[sat]->epoch() + maxdiff)
            {
                iod.first = std::dynamic_pointer_cast<gnss_data_nav>(_ephLast[sat])->iod(); // Last eph
            }
            if (t <= _ephPrev[sat]->epoch() + maxdiff)
            {
                iod.second = std::dynamic_pointer_cast<gnss_data_nav>(_ephPrev[sat])->iod(); // Prev eph
            }
        }
        return iod;
    }

    // print function
    // -------------------
    void gnss_all_nav::print(const std::string &sat, const base_time &t)
    {
        std::shared_ptr<gnss_data_eph> tmp = gnss_all_nav::_find(sat, t);
        tmp->print();
        return;
    }

    // return list of available constellation
    // ----------
    std::set<GSYS> gnss_all_nav::systems() const
    {
        std::set<GSYS> all_sys;
        auto itPRN = _mapsat.begin();

        while (itPRN != _mapsat.end())
        {
            GSYS gsys = gnss_sys::str2gsys(itPRN->first.substr(0, 1));
            all_sys.insert(gsys);
            itPRN++;
        }
        return all_sys;
    }

    // return list of available satellites
    // ----------
    std::set<std::string> gnss_all_nav::satellites() const
    {
        std::set<std::string> all_sat;
        auto itPRN = _mapsat.begin();

        while (itPRN != _mapsat.end())
        {
            all_sat.insert(itPRN->first);
            itPRN++;
        }
        return all_sat;
    }

    // return first position for satellite
    // ----------
    base_time gnss_all_nav::beg_gnav(const std::string &prn) const
    {
        base_time tmp = LAST_TIME;
        if (!prn.empty())
        {
            if (_mapsat.find(prn) != _mapsat.end())
                tmp = _mapsat.at(prn).begin()->first;
        }
        else
        {
            for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapsat.at(itSAT->first).begin()->first < tmp)
                        tmp = _mapsat.at(itSAT->first).begin()->first;
                }
            }
        }
        return tmp;
    }

    // return last position for satellite
    // ----------
    base_time gnss_all_nav::end_gnav(const std::string &prn) const
    {
        base_time tmp = FIRST_TIME;

        if (!prn.empty())
        {
            if (_mapsat.at(prn).size() > 0 && _mapsat.find(prn) != _mapsat.end())
            {
                tmp = _mapsat.at(prn).rbegin()->first;
            }
        }
        else
        {
            for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
            {
                for (auto it = itSAT->second.begin(); it != itSAT->second.end(); ++it)
                {
                    if (_mapsat.at(itSAT->first).rbegin()->first > tmp)
                    {
                        tmp = _mapsat.at(itSAT->first).rbegin()->first;
                    }
                }
            }
        }
        return tmp;
    }

    // add navigation message
    // ----------
    int gnss_all_nav::add(std::shared_ptr<gnss_data_nav> nav)
    {
        base_time ep(nav->epoch());
        std::string sat = nav->sat();

        if (_chkNavig)
        {
            std::set<std::string> msg;
            nav->chk(msg);
        }

        // test navigation type
        bool add = false;
        if (_multimap)
        {
            add = true;
        } // std::multimap
        else if (_mapsat[sat].find(ep) == _mapsat[sat].end())
        {
            add = true;
        } // non-existent
        else if (nav->id_type() == base_data::EPHGAL)
        { // check nav-type
            auto itB = _mapsat[sat].lower_bound(ep);
            auto itE = _mapsat[sat].upper_bound(ep);
            while (itB != itE)
            {
                if (std::dynamic_pointer_cast<gnss_data_nav>(itB->second)->gnavtype() == nav->gnavtype())
                {
                    add = false;
                    break;
                } // exclude the message and skip!
                else
                {
                    add = true;
                } // ok
                ++itB;
            }
        }
        if (!nav->valid())
            add = false; // validity test

        if (add)
        {
            _mapsat[sat].insert(std::make_pair(ep, nav));
            if (sat.substr(0, 1) == "R")
            {
                _gloFreqNum[sat] = nav->freq_num();
            }
        }
        return 0;
    }

    // return number of epochs
    // ----------
    unsigned int gnss_all_nav::nepochs(const std::string &prn)
    {
        unsigned int tmp = 0;
        if (_mapsat.find(prn) != _mapsat.end())
            tmp = _mapsat[prn].size();
        return tmp;
    }

    // list of epochs  ( "" = all, "G" = system, "G01" = satellite )
    // ----------
    std::set<base_time> gnss_all_nav::epochs(const std::string &prn)
    {
        std::set<base_time> epochs;

        auto itFIRST = _mapsat.begin();
        auto itLAST = _mapsat.end();

        if (!prn.empty() && prn.size() == 3)
            itLAST++ = itFIRST = _mapsat.find(prn);

#ifdef DEBUG
        if (itFIRST == _mapsat.begin())
            std::cout << prn << " FIRST = begin\n";
        if (itLAST == _mapsat.end())
            std::cout << prn << "  LAST = end  \n";
        if (itFIRST == itLAST)
            std::cout << prn << "  LAST = FIRST\n";
#endif

        for (auto itSAT = itFIRST; itSAT != itLAST; ++itSAT)
        {
            if (itSAT->first == prn ||                          // individual satellite
                (itSAT->first[0] == prn[0] && prn.size() == 1)) // all satellites of system
            {
                for (auto itEPO = itSAT->second.begin(); itEPO != itSAT->second.end(); ++itEPO)
                {
                    epochs.insert(itEPO->first);
                }
            }
        }
        return epochs;
    }

    // list of nav messages
    // ----------
    std::vector<std::shared_ptr<gnss_data_eph>> gnss_all_nav::vec_nav(const std::string &prn)
    {
        std::vector<std::shared_ptr<gnss_data_eph>> v_nav;

        auto itFIRST = _mapsat.begin();
        auto itLAST = _mapsat.end();

        if (!prn.empty())
            itLAST++ = itFIRST = _mapsat.find(prn);

        for (auto itSAT = itFIRST; itSAT != itLAST; ++itSAT)
        {
            for (auto itEPO = itSAT->second.begin(); itEPO != itSAT->second.end(); ++itEPO)
            {
                v_nav.push_back(itEPO->second);
            }
        }
        return v_nav;
    }

    // list of calculated crd
    // ----------
    std::map<std::string, Triple> gnss_all_nav::map_xyz(const std::set<std::string> &prns, const base_time &epo)
    {
        std::map<std::string, Triple> m_xyz;

        Triple xyz;

        for (auto itPRN = prns.begin(); itPRN != prns.end(); itPRN++)
        {
            std::string prn = *itPRN;

            double sat_xyz[3] = {0.0, 0.0, 0.0};
            double sat_var[3] = {0.0, 0.0, 0.0};
            double sat_vel[3] = {0.0, 0.0, 0.0};

            if (pos(prn, epo, sat_xyz, sat_var, sat_vel) >= 0)
            {
                xyz[0] = sat_xyz[0];
                xyz[1] = sat_xyz[1];
                xyz[2] = sat_xyz[2];

                m_xyz[prn] = xyz;
            }
            else
            {
                continue;
            }
        }

        return m_xyz;
    }

    // list of calculated crd and clk
    std::map<std::string, double> gnss_all_nav::map_clk(const std::set<std::string> &prns, const base_time &epo)
    {
        std::map<std::string, double> m_clk;

        double sat_clk = 0;
        double var = 0;
        double dclk = 0;

        for (auto itPRN = prns.begin(); itPRN != prns.end(); itPRN++)
        {
            if (clk(*itPRN, epo, &sat_clk, &var, &dclk) >= 0)
            {
                m_clk[*itPRN] = sat_clk;
            }
        }

        return m_clk;
    }

    // list of calculated pos from all redundant navig. messages
    std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, Triple>> gnss_all_nav::multi_xyz(const std::set<std::string> &prns, const base_time &epo)
    {
        std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, Triple>> m_xyz;

        double sat_xyz[3] = {0.0, 0.0, 0.0};
        double sat_var[3] = {0.0, 0.0, 0.0};
        double sat_vel[3] = {0.0, 0.0, 0.0};
        Triple xyz;

        for (auto itPRN = prns.begin(); itPRN != prns.end(); itPRN++)
        {
            std::vector<std::shared_ptr<gnss_data_eph>> vec_geph = this->_find_mult(*itPRN, epo);

            for (auto itEPH = vec_geph.begin(); itEPH != vec_geph.end(); itEPH++)
            {
                if (*itEPH && (*itEPH)->pos(epo, sat_xyz, sat_var, sat_vel) >= 0)
                {
                    xyz[0] = sat_xyz[0];
                    xyz[1] = sat_xyz[1];
                    xyz[2] = sat_xyz[2];
                    m_xyz[*itPRN][*itEPH] = xyz;
                }
                else
                    continue;
            }
        }

        return m_xyz;
    }

    // list of calculated clk from all redundant navig. messages
    std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, double>> gnss_all_nav::multi_clk(const std::set<std::string> &prns, const base_time &epo)
    {
        std::map<std::string, std::map<std::shared_ptr<gnss_data_eph>, double>> m_clk;

        double sat_clk = 0;
        double var = 0;
        double dclk = 0;

        for (auto itPRN = prns.begin(); itPRN != prns.end(); itPRN++)
        {
            std::vector<std::shared_ptr<gnss_data_eph>> vec_geph = this->_find_mult(*itPRN, epo);

            for (auto itEPH = vec_geph.begin(); itEPH != vec_geph.end(); itEPH++)
            {
                if (*itEPH && (*itEPH)->clk(epo, &sat_clk, &var, &dclk) >= 0)
                    m_clk[*itPRN][*itEPH] = sat_clk;
            }
        }

        return m_clk;
    }

    // clean invalid messages
    // ----------
    void gnss_all_nav::clean_invalid()
    {
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string prn = itSAT->first;

            auto itEPO = itSAT->second.begin();
            while (itEPO != itSAT->second.end())
            {
                if (!itEPO->second->valid())
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Del NAV invalid: " + prn + " " + itEPO->second->epoch().str_ymdhms() + " " + itEPO->second->gio()->path());
                    _mapsat[prn].erase(itEPO++);
                }
                else
                {
                    itEPO++;
                }
            }
        }
    }

    // clean redundant messages
    // ----------
    void gnss_all_nav::clean_duplicit()
    {
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string prn = itSAT->first;
            std::map<base_time, std::set<int>> list_epo;

            for (auto itEPO = itSAT->second.begin();
                 itEPO != itSAT->second.end();)
            {
                base_time epo = itEPO->first;
                int src = itEPO->second->src(false); // clean all duplicites (false: distinguish INAV/FNAV type only)

                if (list_epo.find(epo) == list_epo.end() ||
                    list_epo[epo].find(src) == list_epo[epo].end())
                {
                    list_epo[epo].insert(src);
                    // + " " + itEPO->second->gio()->path());  glfeng
                    ++itEPO;
                }
                else
                { // remove redundant
                    //+ " " + itEPO->second->gio()->path());  glfeng

                    _mapsat[prn].erase(itEPO++);
                }
            }
        }
    }

    // clean function
    // ----------
    void gnss_all_nav::clean_outer(const base_time &beg, const base_time &end)
    {

        if (end < beg)
            return;
        if (beg == FIRST_TIME)
            return;
        if (end == LAST_TIME)
            return;
        // loop over all satellites
        auto itPRN = _mapsat.begin();
        while (itPRN != _mapsat.end())
        {
            std::string prn = itPRN->first;

            // find and CLEAN all data (epochs) out of the specified period !
            auto itFirst = _mapsat[prn].begin();
            auto itLast = _mapsat[prn].end();
            auto itBeg = _mapsat[prn].lower_bound(beg); // greater only   old: // greater|equal
            auto itEnd = _mapsat[prn].upper_bound(end); // greater only!

            //  std::cout << "distance = " << distance(itBeg,itEnd) << " prn " << prn << std::endl;

            // remove before BEGIN request
            if (itBeg != itFirst)
            {
                auto it = itFirst;

                while (it != itBeg && it != itLast)
                {
                    if ((it->second).use_count() == 1)
                    {
                        _mapsat[prn].erase(it++);
                    }
                    else
                    {
                        it++;
                    }
                }
            }

            // remove after END request
            if (itEnd != itLast)
            {
                auto it = itEnd;

                while (it != itLast)
                {
                    if ((it->second).use_count() == 1)
                    {
                        _mapsat[prn].erase(it++);
                    }
                    else
                    {
                        it++;
                    }
                }
            }
            itPRN++;
        }
#ifdef BMUTEX
        lock.unlock();
#endif
    }

    // get number of satellites
    // ----------
    int gnss_all_nav::nsat(const GSYS &gs) const
    {

        int nsatell = 0;
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string sat = itSAT->first;
            if (gnss_sys::char2gsys(sat[0]) == gs || gs == GNS)
                nsatell++;
        }
        return nsatell;
    }

    // get interval between messages
    // ----------
    int gnss_all_nav::intv(const GSYS &gs) const
    {

        int interval = 0;
        if (gs == GNS)
            return interval;

        // collect sampling intervals
        std::map<int, int> m_intv;
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string sat = itSAT->first;
            if (gnss_sys::char2gsys(sat[0]) != gs)
                continue;

            base_time lstEPO;
            for (auto itEPO = _mapsat.at(sat).begin(); itEPO != _mapsat.at(sat).end(); ++itEPO)
            {
                if (itEPO != _mapsat.at(sat).begin())
                {
                    int diff = int(floor(itEPO->first - lstEPO));
                    m_intv[diff]++;
                }
                lstEPO = itEPO->first;
            }
        }

        // auto-detect sampling interval
        int count = 0;
        for (auto it = m_intv.begin(); it != m_intv.end(); ++it)
        {

            //    std::cout << " " << it->first << ":" << it->second;
            if (it->second > count)
            {
                interval = it->first;
                count = it->second;
            }
            //    std::cout << std::endl;
        }
        //  std::cout << gnss_sys::gsys2str(gs) << " : " << interval << std::endl;

        return interval;
    }

    // get existing number of messages
    // ----------
    int gnss_all_nav::have(const GSYS &gs, const base_time &beg, const base_time &end) const
    {

        int existing = 0;
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string sat = itSAT->first;
            if (gnss_sys::char2gsys(sat[0]) != gs)
                continue;

            for (auto itEPO = _mapsat.at(sat).begin(); itEPO != _mapsat.at(sat).end(); ++itEPO)
            {
                if (itEPO->first < beg - 900 || itEPO->first > end + 900)
                    continue;
                else
                    existing++;
            }
        }
        return existing;
    }

    // get expected number of messages
    // ----------
    int gnss_all_nav::expt(const GSYS &gs, const base_time &beg, const base_time &end) const
    {

        if (end < beg)
            return 0;
        int xint = intv(gs);
        int xsat = nsat(gs);
        int diff = int(floor(end - beg));
        if (xint == 0 || xsat == 0 || diff == 0.0)
            return 0;
        return int(floor(diff / xint * xsat));
    }

    // get excluded number of messages
    // ----------
    int gnss_all_nav::excl(const GSYS &gs, const base_time &beg, const base_time &end) const
    {

        int exclude = 0;
        return exclude;
    }

    // consolidate NAV messsage
    // ----------
    int gnss_all_nav::consolidate(const double &incfdi)
    {
        double cfdi = incfdi;
        if (cfdi <= 0.0)
            cfdi = 10.0; // DEFAULT

        std::map<std::shared_ptr<gnss_data_eph>, double> m_penalty;

        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string prn = itSAT->first;

            NAVDATA str_KPL[] = {NAV_A, NAV_E, NAV_M, NAV_I, NAV_IDOT, NAV_OMEGA, NAV_OMG, NAV_OMGDOT, NAV_DN, NAV_F0};
            NAVDATA str_XYZ[] = {NAV_X, NAV_XD, NAV_XDD, NAV_Y, NAV_YD, NAV_YDD, NAV_Z, NAV_ZD, NAV_ZDD};

            std::vector<NAVDATA> param;
            switch (prn[0])
            {
            case 'R':
            case 'S':
                param.assign(str_XYZ, str_XYZ + 9);
                break;
            default:
                param.assign(str_KPL, str_KPL + 10);
                break;
            }

            for (auto itPAR = param.begin(); itPAR != param.end(); ++itPAR)
            {
                std::vector<std::shared_ptr<gnss_data_eph>> v_ptr;
                std::vector<std::shared_ptr<gnss_data_eph>>::iterator itPTR;

                std::vector<base_time> v_tim;
                std::vector<base_time>::const_iterator itTIM;

                std::vector<double> v_val, v_timdif, v_valdif;
                std::vector<double>::const_iterator itVAL, itDIF;

                // prepare values & differences std::vectors for statistics
                for (auto itEPH = _mapsat[prn].begin(); itEPH != _mapsat[prn].end(); ++itEPH)
                {
                    hwa_map_time_value tmp = itEPH->second->param(*itPAR);
                    double mindif = 60.0; // minimum timedif applied for differences

                    if (!v_tim.empty())
                    { // itEPH != _mapsat[prn].begin() ){
                        // check selected parameters (differences) for periodic switch
                        if (itEPH->second->param_cyclic(*itPAR))
                        {
                            if (tmp.second - v_val.back() > hwa_pi)
                                tmp.second -= 2 * hwa_pi;
                            if (tmp.second - v_val.back() < -hwa_pi)
                                tmp.second += 2 * hwa_pi;
                            mindif = 1.0; // periodic
                        }
                        v_timdif.push_back(tmp.first - v_tim.back());
                        v_valdif.push_back(fabs(v_timdif.back()) > mindif // relax a short time difference to a minute
                                               ? (tmp.second - v_val.back()) / (tmp.first - v_tim.back())
                                               : (tmp.second - v_val.back()) / mindif);
                    }

                    v_ptr.push_back(itEPH->second); // save pointer
                    v_tim.push_back(tmp.first);
                    v_val.push_back(tmp.second);
                }

                if (v_val.size() <= 1)
                {
                    continue;
                } // no values for v_val or v_valdif

                // calculate values & differences statistics
                gnss_base_stat stt_val(_spdlog, v_val);
                gnss_base_stat stt_dif(_spdlog, v_valdif);

                if (!stt_val.valid() && _spdlog)
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Invalid statistics for prn[" + prn + "] values");

                if (!stt_dif.valid() && _spdlog)
                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Invalid statistics for prn[" + prn + "] differences");

                std::ostringstream oSTT;
                oSTT << std::fixed << std::setprecision(5)
                     << prn << " stat: " << std::setw(12) << "parameter" << std::setw(12) << "difference" << std::endl
                     << prn << " medi: " << std::setw(12) << stt_val.get_median() << std::setw(12) << stt_dif.get_median() << std::endl
                     << prn << " mean: " << std::setw(12) << stt_val.get_mean() << std::setw(12) << stt_dif.get_mean() << std::endl
                     << prn << " sdev: " << std::setw(12) << stt_val.get_sdev() << std::setw(12) << stt_dif.get_sdev() << std::endl
                     << prn << " rms : " << std::setw(12) << stt_val.get_rms() << std::setw(12) << stt_dif.get_rms() << std::endl
                     << prn << " min : " << std::setw(12) << stt_val.get_min() << std::setw(12) << stt_dif.get_min() << std::endl
                     << prn << " max : " << std::setw(12) << stt_val.get_max() << std::setw(12) << stt_dif.get_max() << std::endl;

                for (itPTR = v_ptr.begin(), itVAL = v_val.begin(), itTIM = v_tim.begin(), itDIF = v_valdif.begin();
                     itPTR != v_ptr.end(), itVAL != v_val.end();
                     ++itPTR, ++itVAL, ++itTIM, ++itDIF)
                {
                    bool badV = (fabs(*itVAL - stt_val.get_median()) > fabs(cfdi * stt_val.get_sdev()) && stt_val.get_sdev() > 0);
                    bool badD = false;
                    if (itDIF != v_valdif.end())
                    {
                        badD = (fabs(*itDIF - stt_dif.get_median()) > fabs(cfdi * stt_dif.get_sdev()) && stt_dif.get_sdev() > 0);
                    }
                    else
                    {
                        itDIF--;
                    }
                    if (itPTR == v_ptr.begin())
                    {

                        std::ostringstream os;
                        os << std::fixed << std::setprecision(9) << prn << itTIM->str_ymdhms(" ")
                           << "   rmsV: " << std::setw(20) << cfdi * stt_val.get_sdev()
                           << "   rmsD: " << std::setw(20) << cfdi * stt_dif.get_sdev();
                    }

                    std::ostringstream os;
                    os << std::fixed << std::setprecision(6)
                       << prn << itTIM->str_ymdhms(" ")
                       << " " << std::setw(2) << *itPAR
                       << " val: " << std::setw(18) << *itVAL
                       << " " << std::setw(18) << *itVAL - stt_val.get_median()
                       << " badO: " << std::setw(1) << badV;

                    if (itDIF != v_valdif.end())
                    {
                        os << std::setprecision(12)
                           << " dif: " << std::setw(20) << *itDIF
                           << " " << std::setw(20) << *itDIF - stt_dif.get_median()
                           << " badD: " << std::setw(1) << badD;
                    }
                    else
                    {
                        os << std::endl;
                    }

                    if (badV)
                    {
                        m_penalty[*itPTR] += 1.00;
                    }
                    if (badD)
                    {
                        m_penalty[*itPTR] += 0.25;
                        if ((itPTR + 1) != v_ptr.end())
                            m_penalty[*(itPTR + 1)] += 0.25;
                    }
                }
            }
        }

        for (auto it = m_penalty.begin(); it != m_penalty.end(); ++it)
        {
            if (it->second > 3)
            { // criteria
                std::shared_ptr<gnss_data_eph> itP = it->first;
                itP->valid(false); // SET INVALID

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Set NAV invalid: " + itP->sat() + " " + itP->epoch().str_ymdhms() + " [penalty" + base_type_conv::dbl2str(it->second, 1) + "] " + itP->gio()->path());
            }
        }

        return 0;
    }

    // consolidate healthy status & biases
    // ----------
    int gnss_all_nav::consolidate_others() const
    {

        // identify special issue
        std::map<std::string, std::set<NAVDATA>> excl;
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string prn = itSAT->first;
            GSYS gs = gnss_sys::char2gsys(itSAT->first[0]);

            for (auto itEPH = _mapsat.at(prn).begin(); itEPH != _mapsat.at(prn).end(); ++itEPH)
            {
                std::shared_ptr<gnss_data_eph> gnav = itEPH->second;

                GNAVTYPE nav = gnav->gnavtype(false); // distinguish INAV/FNAV only (no source)
                GNAVTYPE ful = gnav->gnavtype(true);  // distinguish all
                std::string hlt = gnav->health_str();
                std::string pth = gnav->gio()->path();

                if (gs == GAL)
                { // identify TRIMBLE NETR9 issue (GALILEO)
                    if ((nav == INAV && hlt.compare("110000000") == 0) ||
                        (ful == INAV_E01 && hlt.compare("000000111") == 0) ||
                        (ful == INAV_E01 && hlt.compare("000111111") == 0))
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "TRIMBLE NETR9 identified [" + prn + "] " + gnavtype2str(ful) + " " + hlt + " " + pth);
                        excl[pth].insert(NAV_HEALTH); // mark for exclusion
                        excl[pth].insert(NAV_TGD0);   // mark for exclusion
                        excl[pth].insert(NAV_TGD1);   // mark for exclusion
                    }
                }
            }
        }

        // consolidate
        for (auto itSAT = _mapsat.begin(); itSAT != _mapsat.end(); ++itSAT)
        {
            std::string prn = itSAT->first;
            GSYS gs = gnss_sys::char2gsys(itSAT->first[0]);

            std::vector<NAVDATA> param;
            //                  param.push_back(NAV_IOD);    // don't do this
            param.push_back(NAV_HEALTH); // should be before TGD to identify exclusions

            switch (prn[0])
            {
            case 'R':
                break;
            case 'S':
                break;
            case 'E':
                param.push_back(NAV_TGD0);
                param.push_back(NAV_TGD1);
                break;
            case 'C':
                param.push_back(NAV_TGD0);
                param.push_back(NAV_TGD1);
                break;
            case 'G':
                param.push_back(NAV_TGD0);
                param.push_back(NAV_TGD1);
                param.push_back(NAV_TGD2);
                param.push_back(NAV_TGD3);
                break;
            case 'Q':
                param.push_back(NAV_TGD0);
                param.push_back(NAV_TGD1);
                param.push_back(NAV_TGD2);
                param.push_back(NAV_TGD3);
                break;
            case 'I':
                param.push_back(NAV_TGD0);
                param.push_back(NAV_TGD1);
                param.push_back(NAV_TGD2);
                param.push_back(NAV_TGD3);
                break;
            }

            for (auto itPAR = param.begin(); itPAR != param.end(); ++itPAR)
            {
                NAVDATA navd = *itPAR;
                bool tgd_param = (navd == NAV_TGD0 || navd == NAV_TGD1 || navd == NAV_TGD2 || navd == NAV_TGD3);
                bool tgd_combine = true; // use all NAV types for TGDs!!! (and see below)

                std::map<GNAVTYPE, std::map<double, int>> day_sort;
                std::map<base_time, std::map<GNAVTYPE, std::map<double, int>>> data_sort;
                std::map<GNAVTYPE, std::map<double, int>::const_iterator> itDAY;
                std::map<GNAVTYPE, std::map<double, int>::const_iterator> itMAX;

                for (auto itEPH = _mapsat.at(prn).begin(); itEPH != _mapsat.at(prn).end(); ++itEPH)
                {
                    std::shared_ptr<gnss_data_eph> gnav = itEPH->second;

                    base_time epo = itEPH->first;
                    hwa_map_time_value tmp = gnav->param(*itPAR);
                    GNAVTYPE nav = gnav->gnavtype(false); // distinguish INAV/FNAV only (no source)
                    GNAVTYPE ful = gnav->gnavtype(true);  // full
                    std::string hlt = gnav->health_str();
                    std::string pth = gnav->gio()->path();
                    double val = tmp.second;

                    if (gs == GAL)
                    { // skip issue

                        bool xxx = (excl.find(pth) != excl.end() && excl[pth].find(navd) != excl[pth].end());

                        if (ful == INAV && (hlt.compare("111000000") == 0 || hlt.compare("000000111") == 0))
                        { // INAV (E1B+E5b with different HEALTH)
                            if (_spdlog)
                                SPDLOG_LOGGER_DEBUG(_spdlog, "INAV - inconsistent E1B/E5b HEALTHY flags [" + prn + "] " + gnavtype2str(nav) + " " + hlt + " " + pth);
                            continue;
                        } // use excl for HEALTHY INAV
                        if (xxx && nav == FNAV && hlt.compare("000000000") == 0)
                        {
                            continue;
                        } // use excl for HEALTHY FNAV
                        if (xxx && nav == FNAV && tgd_param)
                        {
                            continue;
                        } // use excl for GALILEO TGD
                          //        if(        val == 0.0  && tgd_param                     ){ continue; } // use excl zero values
                        if (xxx && navd == NAV_HEALTH)
                        {
                            continue;
                        }
                    }

                    if (tgd_param && tgd_combine)
                    {
                        nav = NAV;
                    } // use all NAV types for TGDs!!! (and see below)

                    data_sort[epo][nav][val]++;
                    day_sort[nav][val]++;
                }

                // find DAY maximum
                for (auto itNAV = day_sort.begin(); itNAV != day_sort.end(); ++itNAV)
                {
                    GNAVTYPE xnav = itNAV->first;
                    if (tgd_param && tgd_combine)
                    {
                        xnav = NAV;
                    }
                    for (auto it = itNAV->second.begin(); it != itNAV->second.end(); ++it)
                    {
                        if (itDAY.find(xnav) == itDAY.end() || double_eq(itDAY[xnav]->first, 0.0) ||
                            (it->second > itDAY[xnav]->second && !double_eq(it->first, 0.0)))
                        {
                            itDAY[xnav] = it;
                        }
                    }
                }

                // loop over nav records
                for (auto itEPH = _mapsat.at(prn).begin(); itEPH != _mapsat.at(prn).end(); ++itEPH)
                {
                    std::shared_ptr<gnss_data_eph> gnav = itEPH->second;

                    base_time epo = itEPH->first;
                    hwa_map_time_value tmp = gnav->param(*itPAR);
                    GNAVTYPE nav = gnav->gnavtype(false); // distinguish INAV/FNAV only (no source)
                    std::string hlt = gnav->health_str();
                    std::string pth = gnav->gio()->path();
                    std::string name = base_name(pth);

                    auto itLOW = _mapsat.at(prn).lower_bound(epo);
                    // auto itUPP = _mapsat.at(prn).upper_bound(epo);

                    if (itEPH == itLOW) // once find epoch-wise frequency maximum
                    {
                        if (data_sort.find(epo) != data_sort.end())
                        {
                            for (auto itNAV = data_sort[epo].begin(); itNAV != data_sort[epo].end(); ++itNAV)
                            {
                                GNAVTYPE xnav = itNAV->first;
                                if (tgd_param && tgd_combine)
                                {
                                    xnav = NAV;
                                }
                                for (auto it = itNAV->second.begin(); it != itNAV->second.end(); ++it)
                                {
                                    if (itMAX.find(xnav) == itMAX.end() || double_eq(itMAX[xnav]->first, 0.0) ||
                                        (it->second > itMAX[xnav]->second && !double_eq(it->first, 0.0)))
                                    {
                                        itMAX[xnav] = it;
                                    }
                                }
                            }
                        }
                        else
                        {
                            for (auto itNAV = day_sort.begin(); itNAV != day_sort.end(); ++itNAV)
                            {
                                GNAVTYPE xnav = itNAV->first;
                                if (tgd_param && tgd_combine)
                                {
                                    xnav = NAV;
                                }
                                for (auto it = itNAV->second.begin(); it != itNAV->second.end(); ++it)
                                {
                                    itMAX[xnav] = it;
                                }
                            }
                        }
                    }

                    if (tgd_param) // consolidate TGDs
                    {
                        GNAVTYPE navfix = nav;
                        if (tgd_combine)
                        {
                            navfix = NAV;
                        } // fix all nav to TGD

                        if ((int((tmp.second - itMAX[navfix]->first) * 1e12) != 0) &&
                            (int((itMAX[navfix]->first) * 1e12) != 0) &&
                            ((itMAX[navfix]->second)) > 0)
                        {
                            gnav->param(navd, itMAX[navfix]->first); // change data to EPOCH MAX
                        }
                        else if ((int(tmp.second * 1e12) == 0) &&
                                 (int(itDAY[navfix]->first * 1e12) != 0))
                        {
                            gnav->param(navd, itDAY[navfix]->first); // change data to DAILY MAX
                        }
                    }
                    else
                    { // consolidate HEALTHY+IOD

                        if ((int(tmp.second - itMAX[nav]->first) != 0) &&
                            (int(itMAX[nav]->first) != 0))
                        {
                            gnav->param(navd, itMAX[nav]->first); // change data
                        }
                    }
                }
            }
        }

        return 0;
    }

    // get IONO CORR
    // -----------------
    gnss_data_iono_corr gnss_all_nav::gegnss_data_iono_corr(const IONO_CORR &c) const
    {
        if (_brdc_iono_cor.find(c) != _brdc_iono_cor.end())
        {
            return _brdc_iono_cor.at(c);
        }
        gnss_data_iono_corr io;
        // default settings , according rtklib->rtkcmn.c->ionmodel
        if (c == IO_GPSA)
        {
            io.x0 = 0.1118e-07;
            io.x1 = -0.7451e-08;
            io.x2 = -0.5961e-07;
            io.x3 = 0.1192e-06;
        }
        else if (c == IO_GPSB)
        {
            io.x0 = 0.1167e+06;
            io.x1 = -0.2294e+06;
            io.x2 = -0.1311e+06;
            io.x3 = 0.1049e+07;
        }

        return io;
    }

    // std::set IONO CORR
    // -----------------
    void gnss_all_nav::add_iono_corr(const IONO_CORR &c, const gnss_data_iono_corr &io)
    {
        if (_brdc_iono_cor.find(c) == _brdc_iono_cor.end())
        {
            _brdc_iono_cor[c] = io;
        }
    }

    // return list of available satellites
    // ----------
    std::shared_ptr<gnss_data_eph> gnss_all_nav::_find(const std::string &sat, const base_time &t, const bool &chk_mask)
    {

        if (_mapsat.find(sat) == _mapsat.end())
            return _null; // std::make_shared<gnss_data_eph>();

        GSYS gnss = gnss_sys::str2gsys(sat.substr(0, 1));

        if (_mapsat[sat].size() == 0)
        {
            return _null;
        }

        auto it = _mapsat[sat].lower_bound(t); // greater|equal  (can be still end())
        if (it == _mapsat[sat].end())
            it--; // size() > 0 already checked above

        double maxdiff = gnss_data_nav::nav_validity(gnss) * 1.1;

        if (gnss == GAL)
        {
            // Galileo ephemerides are valid for toc+10min -> toc+20...180min
            for (int bck = 1; bck <= 5; bck++)
            { // max eph for going back is 5
                base_time toc = it->second->epoch();
                if ((t < toc + 10 * 60 || t > toc + maxdiff) ||
                    (_chkTot && !it->second->chktot(t)))
                { // check ToT for using past messages only
                    if (_mapsat[sat].size() > 0 && it != _mapsat[sat].begin())
                        it--; // one more step back
                    else
                        break;
                }
                if ((t >= toc + 10 * 60 && t <= toc + maxdiff) &&
                    (_chkTot && it->second->chktot(t)))
                    break;
            }
        }
        else if (gnss == BDS)
        {
            // BeiDou ephemerides are valid for toc -> toc+60min
            for (int bck = 1; bck <= 5; bck++)
            { // max eph for going back is 5
                base_time toc = it->second->epoch();
                if ((t < toc || t > toc + maxdiff) ||
                    (_chkTot && !it->second->chktot(t)))
                { // check ToT for using past messages only
                    if (_mapsat[sat].size() > 0 && it != _mapsat[sat].begin())
                        it--; // one more step back
                    else
                        break;
                }
                if ((t >= toc && t <= toc + maxdiff) &&
                    (_chkTot && it->second->chktot(t)))
                    break;
            }
        }
        else if (gnss == GLO)
        {
            base_time tt = t - maxdiff; // time span of Glonass ephemerides is 15 min

            auto itt = _mapsat[sat].lower_bound(tt); // greater|equal
            if (itt == _mapsat[sat].end())
            { // size() > 0 already checked above
                return _null;
            }

            double dt1 = itt->first - t;
            double dt2 = it->first - t;
            if (fabs(dt1) < fabs(dt2))
                it = itt;
        }
        else
        {
            auto itLAST = _mapsat[sat].end();
            itLAST--;
            auto itSAVE = it;
            bool found = false;
            while (_chkTot && !itSAVE->second->chktot(t) && itSAVE->second->epoch().sod() % 3600 != 0 && itSAVE != itLAST)
            {
                itSAVE++; // one more step forward (special case for non-regular ephemerides (e.g. XX:59:44))
            }
            if (_chkTot && itSAVE->second->chktot(t))
            { // check found non-regular ephemeris
                found = true;
                it = itSAVE;
            }

            // 2 conditions for all cases when NAV is in fugure and the iterator should be moved back (it--)
            if (fabs(t - it->second->epoch()) > maxdiff ||    // too far navigation message in future!
                (_chkTot && !it->second->chktot(t) && !found) // check ToT for using past messages only
            )
            {
                if (_mapsat[sat].size() > 0 && it != _mapsat[sat].begin())
                    it--; // one more step back
            }
        }

        // tested found ephemeris
        if (fabs(t - it->second->epoch()) > maxdiff ||
            (_chkTot && !it->second->chktot(t)))
        { // simulation real-time epoch search
            // simulation real-time epoch search
            return _null;
        }

        if (_chkHealth && chk_mask)
        {
            if (it->second->healthy())
                return it->second;
            else
                return _null;
        }
        return it->second;
    }

    // return list of available satellites
    // ----------
    std::vector<std::shared_ptr<gnss_data_eph>> gnss_all_nav::_find_mult(const std::string &sat, const base_time &t) const
    {

        std::vector<std::shared_ptr<gnss_data_eph>> vec_geph;

        if (_mapsat.find(sat) == _mapsat.end())
        {
            vec_geph.push_back(_null);
            return vec_geph;
        }

        GSYS gnss = gnss_sys::str2gsys(sat.substr(0, 1));

        auto it = _mapsat.at(sat).lower_bound(t); // greater|equal

        double maxdiff = gnss_data_nav::nav_validity(gnss) * 1.1;

        if (gnss != GLO)
        {
            if (it == _mapsat.at(sat).end())
            {
                base_time tt = t - maxdiff;
                auto itt = _mapsat.at(sat).lower_bound(tt); // greater|equal
                if (itt == _mapsat.at(sat).end())
                {
                    vec_geph.push_back(_null);
                    return vec_geph;
                }
                else
                {
                    it = itt;
                }
            }
        }
        else
        {
            base_time tt = t - maxdiff;                   // time span of Glonass ephemerides is 15 min
            auto itt = _mapsat.at(sat).lower_bound(tt); // greater|equal
            if (it == _mapsat.at(sat).end())
            {
                if (itt == _mapsat.at(sat).end())
                {
                    vec_geph.push_back(_null);
                    return vec_geph;
                }
                else
                {
                    it = itt;
                }
            }
            else
            {
                if (itt == _mapsat.at(sat).end())
                {
                    vec_geph.push_back(_null);
                    return vec_geph;
                }
                double dt1 = itt->first - t;
                double dt2 = it->first - t;
                if (fabs(dt1) < fabs(dt2))
                    it = itt;
            }
        }

        base_time epo = it->first;
        unsigned int cnt = _mapsat.at(sat).count(epo);

        if (cnt >= 1)
        {
            for (auto itMULT = _mapsat.at(sat).equal_range(epo).first; itMULT != _mapsat.at(sat).equal_range(epo).second; ++itMULT)
            {
                vec_geph.push_back(itMULT->second);
            }
        }

        return vec_geph;
    }

    // return list of available satellites
    // ----------
    std::shared_ptr<gnss_data_eph> gnss_all_nav::_find(const std::string &sat, const int &iod, const base_time &t, const bool &chk_mask)
    {
        if (_mapsat.find(sat) == _mapsat.end())
            return _null; // std::make_shared<gnss_data_eph>();

        GSYS gnss = gnss_sys::str2gsys(sat.substr(0, 1));
        double maxdiff = gnss_data_nav::nav_validity(gnss);

        auto it = _mapsat.at(sat).rbegin();
        while (it != _mapsat.at(sat).rend())
        {
            std::shared_ptr<gnss_data_nav> pt_nav;
            if ((pt_nav = std::dynamic_pointer_cast<gnss_data_nav>(it->second)) != nullptr)
            {
                if (pt_nav->iod() == iod && abs(pt_nav->epoch().diff(t)) <= maxdiff)
                {
                    break; // found
                }
            }
            it++;
        }

        // not found !
        if (it == _mapsat.at(sat).rend())
        {
            return _null; // std::make_shared<gnss_data_eph>();
        }

        if (_chkHealth && chk_mask)
        {
            if (it->second->healthy())
                return it->second;
            else
                return _null;
        }

        return it->second;
    }

} // namespace
