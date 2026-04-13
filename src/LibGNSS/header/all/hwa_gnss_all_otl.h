/**
* @file     gallobs.h
* @brief    Purpose: container along sites for BLQ data
*/

#ifndef hwa_gnss_all_otl_H
#define hwa_gnss_all_otl_H

#include <string>
#include <map>

#include "hwa_base_eigendef.h"
#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_gnss_data_otl.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
     *@brief Class for gnss_all_otl derive from base_data
     */
    class gnss_all_otl : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_all_otl();

        /**
         * @brief Construct a new t gallotl object
         * 
         * @param spdlog 
         */
        gnss_all_otl(base_log spdlog);

        /** @brief default destructor. */
        ~gnss_all_otl();

        /** @brief get data. */
        int data(Matrix &data, const std::string &site);

        /**
         * @brief 
         * 
         * @param data 
         * @param lon 
         * @param lat 
         * @return int 
         */
        int data(Matrix &data, double lon, double lat);

        /** @brief get lat/lon. */
        double lat(const std::string &site);

        /**
         * @brief 
         * 
         * @param site 
         * @return double 
         */
        double lon(const std::string &site);

        /**
         * @brief 
         * 
         * @param otl 
         */
        void add(gnss_data_otl &otl);

        /**
         * @brief 
         * 
         */
        void print();

    private:
        std::map<std::string, gnss_data_otl> _mapotl;
    };

} // namespace

#endif
