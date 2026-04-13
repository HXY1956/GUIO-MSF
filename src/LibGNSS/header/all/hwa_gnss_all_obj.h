
#ifndef hwa_gnss_all_obj_H
#define hwa_gnss_all_obj_H

#include <vector>
#include <memory>
#include "hwa_base_data.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_log.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_Obj.h"
#include "hwa_gnss_all_pcv.h"
#include "hwa_gnss_all_Otl.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
     *@brief Class for t_allobj derive from base_data
     */
    class gnss_all_obj : public base_data
    {
    public:
        /** @brief default constructor. */
        explicit gnss_all_obj();

        /** @brief constructor pcv_olt. */
        explicit gnss_all_obj(gnss_all_pcv *pcv, gnss_all_otl *otl);

        /**
         * @brief Construct a new t gallobj object
         * 
         * @param spdlog 
         */
        explicit gnss_all_obj(base_log spdlog);

        /**
         * @brief Construct a new t gallobj object
         * 
         * @param spdlog 
         * @param pcv 
         * @param otl 
         */
        explicit gnss_all_obj(base_log spdlog, gnss_all_pcv *pcv, gnss_all_otl *otl);

        /** @brief default destructor. */
        virtual ~gnss_all_obj();

        /// @relates gnss_all_obj
        ///< first : , second :
        typedef std::map<std::string, std::shared_ptr<gnss_data_obj>> hwa_map_id_obj; ///< allocated data of a single object

        /**
         * @brief 
         * 
         * @param pcv 
         */
        void setPCV(gnss_all_pcv *pcv); ///< std::set/get PCV

        /**
         * @brief 
         * 
         * @param otl 
         */
        void setOTL(gnss_all_otl *otl); ///< std::set/get OTL

        /**
         * @brief 
         * 
         * @param objs 
         * @return int 
         */
        int add(const std::set<std::string> &objs); ///< std::set/get objects

        /**
         * @brief 
         * 
         * @param obj 
         * @return int 
         */
        int add(std::shared_ptr<gnss_data_obj> obj); ///< std::set/get single obj element

        /**
         * @brief 
         * 
         * @param s 
         * @return std::shared_ptr<gnss_data_obj> 
         */
        std::shared_ptr<gnss_data_obj> obj(const std::string &s);

        /**
        * @brief synchronize PCVs.
        *
        * @return    void
        */
        void sync_pcvs();

        /**
        * @brief read satellite information.
        *
        * @param[in]  epo    epoch
        * @return    void
        */
        void read_satinfo(base_time &epo);

        /**
        * @brief get all object elements.
        *
        * @param[in]  id
        * @return    all object
        */
        virtual std::map<std::string, std::shared_ptr<gnss_data_obj>> objects(const base_data::ID_TYPE &id = NONE);

        /**
        * @brief std::set all object elements.
        *
        * @param[in]  id        id type
        * @return    all object
        */
        virtual std::set<std::string> objects_set(const base_data::ID_TYPE &id = base_data::NONE);

        /**
        * @brief get object number.
        *
        * @return    size of object
        */
        virtual int obj_num(); ///< get object number

    private:
        hwa_map_id_obj _mapobj; ///< std::map of all objects
        gnss_all_pcv *_gpcv;  ///< std::map of all PCV
        gnss_all_otl *_gotl;  ///< std::map of all otl
        void _aloctrn();   ///< alocate all gnss_data_trn objects for all GNSS
    };

} // namespace

#endif
