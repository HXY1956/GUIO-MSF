#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "hwa_gnss_all_obj.h"
#include "hwa_gnss_data_trn.h"

namespace hwa_gnss
{
    gnss_all_obj::gnss_all_obj()
        : base_data(),
          _gpcv(0),
          _gotl(0)
    {
        id_type(base_data::ALLOBJ);
        id_group(base_data::GRP_objECT);
        _aloctrn();
    }
    gnss_all_obj::gnss_all_obj(base_log spdlog)
        : base_data(spdlog),
          _gpcv(0),
          _gotl(0)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
        id_type(base_data::ALLOBJ);
        id_group(base_data::GRP_objECT);
        _aloctrn();
    }

    // constructor
    // ----------
    gnss_all_obj::gnss_all_obj(gnss_all_pcv *pcv, gnss_all_otl *otl) : base_data(),
                                                           _gpcv(pcv),
                                                           _gotl(otl)
    {

        id_type(base_data::ALLOBJ);
        id_group(base_data::GRP_objECT);
        _aloctrn();
    }

    gnss_all_obj::gnss_all_obj(base_log spdlog, gnss_all_pcv *pcv, gnss_all_otl *otl) : base_data(spdlog),
                                                                            _gpcv(pcv),
                                                                            _gotl(otl)
    {
        id_type(base_data::ALLOBJ);
        id_group(base_data::GRP_objECT);
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
        _aloctrn();
    }
    // destructor
    // ----------
    gnss_all_obj::~gnss_all_obj()
    {
        _mapobj.clear();
        return;
    }

    // add object
    // ----------
    int gnss_all_obj::add(std::shared_ptr<gnss_data_obj> obj)
    {
        std::string s = obj->id();

        // object empty
        if (obj->id().empty())
        {
            return -1;
        }

        // object does not exist
        // or _overwrite (--> remove & addnew)
        if (_mapobj.find(s) == _mapobj.end())
        {

            _mapobj[s] = obj;

            _mapobj[s]->spdlog(_spdlog);
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "add new obj " + s);
        }
        else
        {
            // FINALLY COULD ADD RECORDS FROM OBJ to existing OBJ (merge/overwrite mode)
            // jdhuang : change 0 ==> 1
            // if (_log && _log->verb() >= 0) _log->comment(0, "gallobj", "warning - cannot overwrite object: " + s);
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "warning - cannot overwrite object: " + s);
            return -1;
        }

        // individual get/sync PCV for new object
        // --------------------------------------------------------------------------
        // HOWEVER, IT MUST BE GUARANTED THAT WHENEVER GOBJ IS MODIFIED PCVs are SYNC AGAIN!
        // --------------------------------------------------------------------------

        _mapobj[s]->sync_pcv(_gpcv);
        return 0;
    }

    // get single object
    // -------------------------------
    std::shared_ptr<gnss_data_obj> gnss_all_obj::obj(const std::string &s)
    {
        std::shared_ptr<gnss_data_obj> p_obj;

        hwa_map_id_obj::iterator it = _mapobj.find(s);
        if (it == _mapobj.end())
        {
            //    std::cout << "object " << _mapobj.size() <<  " " << s << " not found !\n";
            return p_obj;
        }
        else
        {
            p_obj = it->second;
        }
        return p_obj;
    }

    std::set<std::string> gnss_all_obj::objects_set(const base_data::ID_TYPE &id)
    {
        std::set<std::string> all_obj;
        hwa_map_id_obj::const_iterator itOBJ = _mapobj.begin();
        hwa_map_id_obj::const_iterator itEND = _mapobj.end();
        while (itOBJ != itEND)
        {
            if (id != base_data::NONE && id != itOBJ->second->id_type())
            {
                ++itOBJ;
                continue;
            } // filter objects
            all_obj.insert(itOBJ->first);
            ++itOBJ;
        }
        return all_obj;
    }

    // get object number
    // -----------
    int gnss_all_obj::obj_num()
    {
        return _mapobj.size();
    }


    // get all object elements
    // ----------
    std::map<std::string, std::shared_ptr<gnss_data_obj>> gnss_all_obj::objects(const base_data::ID_TYPE &id)
    {
        std::map<std::string, std::shared_ptr<gnss_data_obj>> all_obj;
        hwa_map_id_obj::const_iterator itOBJ = _mapobj.begin();

        while (itOBJ != _mapobj.end())
        {
            if (id != NONE && id != itOBJ->second->id_type())
            {
                ++itOBJ;
                continue;
            } // filter objects
            std::string site = itOBJ->second->id();
            all_obj[site] = itOBJ->second;
            ++itOBJ;
        }
        return all_obj;
    }

    // std::set PCV
    // ------------------------
    void gnss_all_obj::setPCV(gnss_all_pcv *pcv)
    {
        _gpcv = pcv;
        return;
    }

    // std::set OTL
    // ------------------------
    void gnss_all_obj::setOTL(gnss_all_otl *otl)
    {
        _gotl = otl;
        return;
    }

    // get all object elements
    // ----------
    void gnss_all_obj::sync_pcvs()
    {
        hwa_map_id_obj::const_iterator itOBJ = _mapobj.begin();

        while (itOBJ != _mapobj.end())
        {
#ifdef DEBUG
            std::cout << "GALLOBJ syncing: " << itOBJ->second->id() << std::endl;
#endif
            itOBJ->second->sync_pcv(_gpcv);
            ++itOBJ;
        }

#ifdef DEBUG
        std::cout << " PCVs synchronized for all objects !\n";
#endif
        return;
    }

    // alocate all gnss_data_trn objects for all GNSS
    // ------------------
    void gnss_all_obj::_aloctrn()
    {

        for (int i = 0; i != GNS; ++i)
        {
            GSYS sys = static_cast<GSYS>(i);
            std::set<std::string> sats = gnss_sats()[sys];
            for (auto iter = sats.begin(); iter != sats.end(); ++iter)
            {
                std::string satID = *iter;
                std::shared_ptr<gnss_data_trn> trn_new = std::make_shared<gnss_data_trn>(_spdlog);
                trn_new->id(satID);
                //         trn_new->ant( satID, FIRST_TIME );    // add antena name
                _mapobj[satID] = trn_new;
            }
        }
    }

    // Read Satelite info file
    // ------------------
    // !!!!!!! MUST be enhanced for reading whole satelite info file !!!!!!!!!!!!
    void gnss_all_obj::read_satinfo(base_time &epo)
    {

        std::cout << _mapobj["G01"]->name();
        hwa_map_id_obj::iterator it;
        for (it = _mapobj.begin(); it != _mapobj.end(); ++it)
        {
            if (it->second->id_type() == TRN)
            {
                std::string ID = it->first;
                it->second->ant(ID, epo);
            }
        }
    }

} // namespace
