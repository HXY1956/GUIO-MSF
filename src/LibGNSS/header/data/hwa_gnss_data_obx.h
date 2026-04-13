/**
* @file        gobx.h
* @brief       Storage the obx files' data
*/

#ifndef hwa_gnss_data_obx_H
#define hwa_gnss_data_obx_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief     Class for storaging obx file record data
    */
    class gnss_data_obx_record
    {
    public:
        /** @brief default constructor. */
        gnss_data_obx_record();
        gnss_data_obx_record(const base_time& epo, const std::string& id, const Matrix& rotLeft);
        gnss_data_obx_record(const base_time& epo, const std::string& rec, const std::string& id, const int& num, 
            const double& q0, const double& q1, const double& q2, const double& q3);

        /** @brief default constructor. */
        virtual ~gnss_data_obx_record();

        std::string rec;      ///< type
        std::string id;       ///< sat prn
        int num = 0;         ///< number
        double  q0 = 0.0; ///< scalar
        double  q1 = 0.0; ///< qx
        double  q2 = 0.0; ///< qy
        double  q3 = 0.0; ///< qz
        bool    isValid = false;
        base_time epo;
    private:
        bool    isRotMat = false;
        Matrix  rotMat;
    };

    /**
    *@brief     Class for storaging obx file epoch data
    */
    class gnss_data_obx_epoch
    {
    public:
        /** @brief default constructor. */
        gnss_data_obx_epoch();
        /** @brief default constructor. */
        virtual ~gnss_data_obx_epoch();

        // get obx time 
        bool addTime(const base_time& epoch);
        // add obx time
        const base_time& getTime() const;
        // add obx record
        bool addObxRecord(const gnss_data_obx_record& obx_record);
        // get obx record
        const gnss_data_obx_record& getObxRecord(const std::string& prn) const;
        // get sat_num;
        const int& getSatNum() const;
    public:
        std::map<std::string, gnss_data_obx_record> obx_data; ///< prn, obx_data


    private:
        base_time _epoch;                       ///< epoch data
        int     _sat_num = 0;                 ///< sat number
    };


}

#endif // !GOBX_H
