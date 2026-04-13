#ifndef hwa_gnss_all_prod_H
#define hwa_gnss_all_prod_H

#include "hwa_gnss_prod.h"
#include "hwa_base_time.h"

namespace hwa_gnss
{
    class gnss_all_prod : public base_data
    {

    public:
        /** @brief default constructor. */
        gnss_all_prod();
        /**
         * @brief Construct a new t gallprod object
         * 
         * @param spdlog 
         */
        gnss_all_prod(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_all_prod();

        typedef std::map<base_time, std::shared_ptr<gnss_prod>> hwa_map_t_prod; ///< time + gnss_prod
        typedef std::map<ID_TYPE, hwa_map_t_prod> hwa_map_it_prod;            ///< ID_TYPE + hwa_map_id
        typedef std::map<std::string, hwa_map_it_prod> hwa_map_iit_prod;             ///< std::string + hwa_map_iit_prod

        /** @brief add product. */
        int add(std::shared_ptr<gnss_prod> prod, std::string site = "");

        /** @brief get product. */
        std::shared_ptr<gnss_prod> get(const std::string &site, ID_TYPE type, const base_time &t);

        /** @brief remove appropriate element gnss_prod*. */
        void rem(const std::string &site, ID_TYPE type, const base_time &t);

        /** @brief get the list of sites*. */
        std::set<std::string> prod_sites();

        /** @brief get the list of product types. */
        std::set<ID_TYPE> prod_types(const std::string &site);

        /** @brief get the list of product types. */
        std::set<base_time> prod_epochs(const std::string &site, ID_TYPE type);

        /** @brief clean. */
        void clear();

        /** @brief clean data out of the interval. */
        void clean_outer(const base_time &beg = FIRST_TIME, const base_time &end = LAST_TIME);

    protected:
        /** @brief find appropriate element gnss_prod*. */
        std::shared_ptr<gnss_prod> _find(const std::string &site, ID_TYPE type, const base_time &t); // find element

        hwa_map_iit_prod _map_prod; ///< hwa_map_iit_prod
    };

} // namespace

#endif
