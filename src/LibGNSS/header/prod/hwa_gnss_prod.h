#ifndef hwa_prod_H
#define hwa_prod_H

#include <set>
#include <iostream>
#include <map>
#include <string>
#include <memory>
#include <string>
#include "hwa_base_data.h"
#include "hwa_base_common.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_Obj.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    static std::shared_ptr<gnss_data_obj> nullobj;

    /** @brief class for gnss_prod derive from base_data. */
    class gnss_prod : public base_data
    {

    public:
        /** @brief constructor 1. */
        explicit gnss_prod(const base_time &t, std::shared_ptr<gnss_data_obj> pt = nullobj);

        gnss_prod(base_log spdlog, const base_time &t, std::shared_ptr<gnss_data_obj> pt = nullobj);
        /** @brief default destructor. */
        virtual ~gnss_prod();

        typedef std::map<std::string, std::pair<double, double>> hwa_map_prod; ///< std::map of prod

        /** @brief get/std::set epo. */
        base_time epoch() const { return _epo; }
        void epoch(const base_time &epo) { _epo = epo; }

        /** @brief get/std::set obj. */
        std::shared_ptr<gnss_data_obj> obj() const { return _obj; }
        std::string obj_id() const
        {
            if (_obj)
                return _obj->id();
            else
                return "";
        }

        /** @brief get/std::set val. */
        int set_val(const std::string &str, const double &val, const double &rms = 0.0);
        int get_val(const std::string &str, double &val, double &rms);
        int get_val(const std::string &str, double &val);

        /** @brief get list. */
        std::set<std::string> list_id();

        /** @brief get/std::set nSat. */
        int nSat() { return _nSat; }
        void nSat(const int &n) { _nSat = n; }

        /** @brief get/std::set nSat_excl. */
        int nSat_excl() { return _nSat_excl; }
        void nSat_excl(const int &n) { _nSat_excl = n; }

    protected:
        base_time _epo;                      ///< epo
        std::shared_ptr<gnss_data_obj> _obj;           ///< object
        hwa_map_prod _prod;                  ///< prod
        hwa_map_prod::const_iterator itPROD; ///< it_prod

        int _nSat;      ///< number of sat
        int _nSat_excl; ///< number of sat excl

    private:
    };

} // namespace

#endif
