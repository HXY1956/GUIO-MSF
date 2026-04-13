#include "hwa_gnss_all_prod.h"

namespace hwa_gnss
{
    // constructor
    // ----------
    gnss_all_prod::gnss_all_prod()
        : base_data()
    {
        id_type(base_data::ALLPROD);
        id_group(base_data::GRP_prodUCT);
    }
    gnss_all_prod::gnss_all_prod(base_log spdlog)
        : base_data(spdlog)
    {
        id_type(base_data::ALLPROD);
        id_group(base_data::GRP_prodUCT);
    }

    // destructor
    // ----------
    gnss_all_prod::~gnss_all_prod()
    {

        this->clear();
    }

    // clean
    // --------------------
    void gnss_all_prod::clear()
    {
        _map_prod.clear();
    }

    // add product
    // ---------------------
    int gnss_all_prod::add(std::shared_ptr<gnss_prod> prod, std::string site) // str only if prod->obj_pt = 0
    {

        ID_TYPE type = prod->id_type();

        std::string id = prod->obj_id(); // if( id.empty() ){ return -1; } // no, use id instead!

        if (id.empty())
            id = site;
        if (id.empty() && !site.empty() && id != site)
            std::cerr << "warning [gallprod] - adding product with both [and different] non-zero std::string and object id!\n";
        _map_prod[id][type][prod->epoch()] = prod; // allocate in heap

#ifdef DEBUG
        std::cout << "adding pt: " << prod->epoch().str_ymdhms(" ") << ":" << prod->epoch().dsec() << " type: " << type << std::endl;
#endif
        return 0;
    }

    // get product
    // --------------------
    std::shared_ptr<gnss_prod> gnss_all_prod::get(const std::string &site, ID_TYPE type, const base_time &t)
    {
        std::shared_ptr<gnss_prod> obj_pt = _find(site, type, t);

#ifdef DEBUG
        std::cout << "geting pt: " << obj_pt << " " << obj_pt.use_count() << " " << _map_prod.size() << " ... "
             << t.str_ymdhms(" ") << ":" << t.dsec() << std::endl;
#endif
        return obj_pt;
    }

    // list of sites
    // --------------------
    std::set<std::string> gnss_all_prod::prod_sites()
    {

        std::set<std::string> site_list;
        hwa_map_iit_prod::const_iterator it;
        for (it = _map_prod.begin(); it != _map_prod.end(); ++it)
        {
            site_list.insert(it->first);
        }
        return site_list;
    }

    // list of product types
    // --------------------
    std::set<base_data::ID_TYPE> gnss_all_prod::prod_types(const std::string &site)
    {

        std::set<base_data::ID_TYPE> type_list;
        hwa_map_it_prod::const_iterator itTYPE;
        hwa_map_iit_prod::const_iterator it = _map_prod.find(site);

        if (it == _map_prod.end())
        {
            return type_list;
        }

        for (itTYPE = _map_prod[site].begin(); itTYPE != _map_prod[site].end(); ++itTYPE)
        {
            type_list.insert(itTYPE->first);
        }
        return type_list;
    }

    // list of product types
    // --------------------
    std::set<base_time> gnss_all_prod::prod_epochs(const std::string &site, ID_TYPE type)
    {

        std::set<base_time> epo_list;
        hwa_map_t_prod::const_iterator itEPO;
        hwa_map_it_prod::const_iterator itTYPE;
        hwa_map_iit_prod::const_iterator it = _map_prod.find(site);

        if (it == _map_prod.end())
        {
            return epo_list;
        }
        else
        {
            itTYPE = it->second.find(type);
            if (itTYPE == it->second.end())
            {
                return epo_list;
            }
            else
            {
                for (itEPO = itTYPE->second.begin(); itEPO != itTYPE->second.end(); ++itEPO)
                {
                    epo_list.insert(itEPO->first);
                }
            }
        }
        return epo_list;
    }

    // clean data out of the interval
    // ----------------------------
    void gnss_all_prod::clean_outer(const base_time &beg, const base_time &end)
    {

        if (end < beg)
            return;
        // loop over all sites
        hwa_map_iit_prod::iterator itKEY = _map_prod.begin();
        while (itKEY != _map_prod.end())
        {
            std::string key = itKEY->first;

            hwa_map_it_prod::iterator itID = itKEY->second.begin();
            while (itID != itKEY->second.end())
            {
                std::string id = base_data::type2str(itID->first);

                hwa_map_t_prod::iterator it;
                hwa_map_t_prod::iterator itFirst = itID->second.begin();
                hwa_map_t_prod::iterator itLast = itID->second.end();
                hwa_map_t_prod::iterator itBeg = itID->second.lower_bound(beg);
                hwa_map_t_prod::iterator itEnd = itID->second.upper_bound(end);

                // remove before BEGIN request
                if (itBeg != itFirst)
                {

                    it = itFirst;

                    // begin is last
                    if (itBeg == itLast)
                    {
                        itBeg--;

                        while (it != itLast) // itID->second.erase(it++);
                            if ((it->second).use_count() == 1)
                            {
                                itID->second.erase(it++);
                            }
                            else
                            {
                                it++;
                            }

                        // begin is not last
                    }
                    else
                    {

                        while (it != itBeg) // itID->second.erase(it++);
                            if ((it->second).use_count() == 1)
                            {
                                itID->second.erase(it++);
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
                    it = itEnd;

                    while (it != itLast) // itID->second.erase(it++);
                        if ((it->second).use_count() == 1)
                        {
                            itID->second.erase(it++);
                        }
                        else
                        {
                            it++;
                        }
                }
                itID++;
            }
            itKEY++;
        }

#ifdef BMUTEX
        lock.unlock();
#endif
        return;
    }

    // remove appropriate element gnss_prod*
    // ---------------------
    void gnss_all_prod::rem(const std::string &site, ID_TYPE type, const base_time &t)
    {
        hwa_map_t_prod::iterator itEP;
        hwa_map_it_prod::iterator itID;
        hwa_map_iit_prod::iterator it = _map_prod.find(site);
        if (it != _map_prod.end())
        {
            itID = it->second.find(type);
            if (itID != it->second.end())
            {
                itEP = itID->second.find(t);
                if (itEP != itID->second.end())
                    _map_prod[site][type].erase(itEP);
            }
        }
        return;
    }

    // find appropriate element gnss_prod*
    // ---------------------
    std::shared_ptr<gnss_prod> gnss_all_prod::_find(const std::string &site, ID_TYPE type, const base_time &t)
    {

        hwa_map_t_prod::iterator itEP;
        hwa_map_it_prod::iterator itID;
        hwa_map_iit_prod::iterator it = _map_prod.find(site);
        if (it != _map_prod.end())
        {
            itID = it->second.find(type);
            if (itID != it->second.end())
            {
                itEP = itID->second.find(t);
                if (itEP != itID->second.end())
                    return itEP->second;
            }
        }

        std::shared_ptr<gnss_prod> tmp;
        return tmp;
    }

} // namespace
