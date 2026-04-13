#ifndef hwa_base_all_proc_h
#define hwa_base_all_proc_h

#include "hwa_base_data.h"

using namespace hwa_base;

namespace hwa_base
{
    /**
    *@brief       Class for storaging all process data
    */
    class  base_all_proc : public base_data
    {
    public:
        /** @brief default constructor. */
         base_all_proc();

        /**
         * @brief Construct a new t gallproc object
         * 
         * @param spdlog 
         */
         base_all_proc(base_log spdlog);
        /** @brief copy constructor. */
         base_all_proc(const  base_all_proc &Other);

        /** @brief default destructor. */
        virtual ~ base_all_proc();

        /** @brief assignment operator . */
        void operator=(const  base_all_proc &Other);

        /** @brief add one process data. */
        void Add_Data(const std::string &type, base_data *data);

        /** 
        * @brief get the data by type of base_data.
        * @param[in] id_type specified type of base_data 
        * @return specified data
        */
        base_data *operator[](base_data::ID_TYPE id_type);

        /** 
        * @brief get the data by group type of base_data.
        * @param[in] id_type specified group type of base_data 
        * @return specified data
        */
        base_data *operator[](base_data::ID_GROUP);


    protected:
        std::map<base_data::ID_TYPE, base_data *> _mapData; ///< std::map of type and data pointer  for all process data
    };
}

#endif