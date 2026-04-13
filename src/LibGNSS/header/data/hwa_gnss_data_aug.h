#ifndef hwa_gnss_data_aug_H
#define hwa_gnss_data_aug_H

#include "hwa_base_data.h"
#include "hwa_set_gtype.h"
#include "hwa_base_time.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{
    /** @brief aug type. */
    enum class AUGTYPE
    {
        TYPE_P = 1,    ///< code
        TYPE_L = 2,    ///< phase
        TYPE_TRP = 3,  ///< trop
        TYPE_ION = 4,  ///< ion
        TYPE_NON = 999 ///< none
    };

    typedef std::pair<AUGTYPE, GOBSBAND> hwa_pair_augtype; ///< aug type
    typedef std::map<hwa_pair_augtype, double> hwa_map_augtype_value; ///< aug_type--value
    typedef std::map<std::string, hwa_map_augtype_value> hwa_map_id_augtype_value; ///< satellite--aug_type
    typedef std::map<base_time, hwa_map_id_augtype_value> hwa_map_ti_augtype_value; ///< epoch--satellite
    typedef std::map<std::string, hwa_map_ti_augtype_value> hwa_map_iti_augtype_value;  ///< site--epoch

    /**
    *@brief       Class for aug data storaging, derive from t_gcoder
    */
    class gnss_data_aug : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_aug();

        /** @brief default destructor. */
        virtual ~gnss_data_aug();

        /** 
        * @brief get the aug end_time. 
        * 
        * @param[in]  site        site name
        * @return      base_time      end time
        */
        base_time endTime_aug(std::string &site);

        /**
        * @brief add data.
        *
        * @param[in]  site        site name
        * @param[in]  epo         the current time
        * @param[in]  sat         satellite name
        * @param[in]  type        aug type
        * @param[in]  value       the value
        * 
        * @return      void
        * 
        */
        void add_data(std::string site, const base_time &epo, std::string sat, hwa_pair_augtype type, double value);

        /**
        * @brief get the aug.
        * 
        * @return      hwa_map_iti_augtype_value      Aug
        */
        hwa_map_iti_augtype_value *getAug();

        /**
        * @brief get the aug.
        * 
        * @param[in]  site        site name
        * @return      hwa_map_iti_augtype_value      Aug
        */
        hwa_map_ti_augtype_value *getAug(std::string site);

        /**
        * @brief get the aug.
        *
        * @param[in]  site        site name
        * @param[in]  epo          the current time
        * @return      hwa_map_iti_augtype_value      Aug
        */
        hwa_map_id_augtype_value *getAug(std::string site, const base_time &epo);

        /**
        * @brief get the aug.
        *
        * @param[in]  site        site name
        * @param[in]  epo          the current time
        * @param[in]  sat          the satellite name
        * @return      hwa_map_iti_augtype_value      Aug
        */
        hwa_map_augtype_value *getAug(std::string site, const base_time &epo, std::string sat);

        /**
        * @brief clean the outer.
        * lvhb added in 20210431
        *
        * @param[in]  site        site name
        * @param[in]  beg          the begin time
        * @param[in]  end          the end name
        * @return      void
        */
        virtual void clean_outer(const std::string &site = "",
                                 const base_time &beg = FIRST_TIME,
                                 const base_time &end = LAST_TIME);

    protected:
        hwa_map_iti_augtype_value _gaug; ///< aug data
    };

}
#endif