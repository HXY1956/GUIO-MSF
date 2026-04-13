/**
* @file        gallcoder.h
* @brief    main decode function for all files
*/
#ifndef hwa_gnss_all_coder_H
#define hwa_gnss_all_coder_H

#include "hwa_base_data.h"
#include "hwa_base_coder.h"
#include "hwa_set_inp.h"
#include "hwa_gnss_all_obj.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief Class for gnss_all_coder
    */
    class gnss_all_coder
    {
    public:
        /** @brief default constructor. */
        gnss_all_coder();

        /** @brief default constructor. */
        gnss_all_coder(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_all_coder();

        /** @brief add spdlog. */
        virtual void spdlog(base_log spdlog);

        /** @brief add setting. */
        virtual void set(set_base *set);

        /** @brief convert data to ID_TYPE. */
        IFMT data2ifmt(base_data::ID_TYPE type);

        /** @brief convert ID_TYPE to data. */
        base_data::ID_TYPE ifmt2data(IFMT ifmt);

        /**
        * @brief judge whether support the format.
        * @param[in]  ifmt        data format
        * @return 
        *        true: support
        *        false: not support
        */
        virtual bool support(const IFMT &ifmt) = 0;

        /**
        * @brief coder.
        * @param[in]  ifmt        data format
        * @param[in]  path        file path
        * @param[in]  data        data pointer
        * @param[in]  obj        object pointer
        * @return    void
        */
        virtual void coder(const IFMT &ifmt, std::string path, base_data *data, gnss_all_obj *obj) = 0;

        /**
        * @brief creat.
        * @param[in]  ifmt        data format
        * @param[in]  map_gdata    map of data
        * @return    void
        */
        virtual void creat(const IFMT &ifmt, std::map<base_data::ID_TYPE, base_data *> &map_gdata) = 0;

        /**
        * @brief destroy.
        * @param[in]  map_gdata    map of data
        * @return    void
        */
        void destroy(std::map<base_data::ID_TYPE, base_data *> &map_gdata);

    protected:
        set_base *_gset = nullptr; ///< gset
        base_log _spdlog;            ///< spdlog pointer
    };
}

#endif