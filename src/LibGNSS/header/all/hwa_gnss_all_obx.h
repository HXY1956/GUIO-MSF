/**
* @file       gallobx.h
* @brief      Storage the ambiguity obx files' data(more than one epoch)
*/
#ifndef hwa_gnss_all_obx_H
#define hwa_gnss_all_obx_H

#include <vector>
#include "hwa_gnss_data_obx.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for all ambupd file data of all stations storaging
    */
    class gnss_all_obx : public base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_all_obx();

        /** @brief default constructor. */
        explicit gnss_all_obx(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_obx();

        /** @brief std::map for storaging obx of all epoch. */
        typedef std::map<base_time, gnss_data_obx_epoch> hwa_map_time_obx;

        /**
        * @brief add one epochs obx data.
        * @param[in]  epoch        epoch time.
        * @param[in]  obx_epoch    obx data of one station.
        * @return      void
        */
        void addObx(const base_time &epoch, const gnss_data_obx_epoch &obx_epoch);

        // get Begin time
        const base_time& getBegEpo() const;
        // get End   time
        const base_time& getEndEpo() const;

        // get rot mat from trs to sat fix
        Matrix getRotMat(const std::string& prn, const base_time& epoch);

        const hwa_map_time_obx& getObxData() const;
    protected:
        // get epoch obx data
        const  gnss_data_obx_epoch& getObxEpoch(const base_time& epoch) const;
        // get upper obx data
        const  gnss_data_obx_record& getObxUpperRecord(const std::string& prn, const base_time& epoch) const;
        // get lower obx data
        const  gnss_data_obx_record& getObxLowerRecord(const std::string& prn, const base_time& epoch) const;

        // q to matrix
        Matrix Quat2RotMat(const  gnss_data_obx_record& obx_record);

        // slerp
        bool slerp(const base_time& epo, const base_time& beg, const  gnss_data_obx_record&beg_obx, const base_time& end, const  gnss_data_obx_record& end_obx, gnss_data_obx_record& now_obw);

    protected:
        hwa_map_time_obx _allobx; ///< std::map of station name and ambupd data for all epochs and all satellites

        base_time _beg_epo = LAST_TIME;
        base_time _end_epo = FIRST_TIME;

        base_log _spdlog;

        gnss_data_obx_record _empty_record;
        gnss_data_obx_epoch  _empty_epoch;
    };

} // namespace

#endif
