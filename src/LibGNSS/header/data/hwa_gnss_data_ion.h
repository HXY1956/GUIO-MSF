/**
* @file        gion.h
* @brief    Storage the ion files' data
*/

#ifndef hwa_ION_H
#define hwa_ION_H

#include "hwa_base_data.h"
#include "hwa_base_time.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief     Class for storaging ambinp file data
    */
    class gnss_data_ion_record
    {
    public:
        /** @brief default constructor. */
        gnss_data_ion_record();
        gnss_data_ion_record(const std::string &type, const std::string &rec, const std::string &sat,
                      const double &value, const double &sigma,
                      const base_time &beg, const base_time &end);
        /** @brief default constructor. */
        virtual ~gnss_data_ion_record();

        std::string ion_type; ///< ionosphere type SION or VION
        double value;    ///< value
        double sigma;    ///< mean square error
        std::string sat;      ///< satellite list
        std::string rec;      ///< receiver list
        base_time beg;     ///< begin time
        base_time end;     ///< end time
    };

    /**
    *@brief     Class for storaging ion file data
    */
    class gnss_data_ion : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ion();

        /** @brief default constructor. */
        gnss_data_ion(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ion();
        /**
        * @brief std::set header info
        * @param[in]  beg          begin time
        * @param[in]  end          end time
        * @param[in]  intv          time interval
        * @param[in]  num_amb      ambiguity number
        * @param[in]  sats          satellite list
        * @param[in]  recs          receiver list
        */
        void set_header(const base_time &beg, const base_time &end, const double &intv, const int &num_amb, const std::set<std::string> &sats, const std::set<std::string> &recs);
        /**
        * @brief get header info
        * @param[in]  beg          begin time
        * @param[in]  end          end time
        * @param[in]  intv          time interval
        * @param[in]  sats          satellite list
        * @param[in]  recs          receiver list
        */
        void get_header(base_time &beg, base_time &end, double &intv, std::set<std::string> &sats, std::set<std::string> &recs);
        /**
        * @brief get value
        * @param[in]  sats          satellite list
        * @param[in]  recs          receiver list
        * @param[in]  beg          begin time
        * @param[in]  end          end time
        */
        double get_value(const std::string &rec, const std::string &sat, const base_time &beg, const base_time &end);
        /**
        * @brief add record info
        * @param[in]  record        record info
        */
        void add_record(const gnss_data_ion_record &record);
        /**
        * @brief get record info
        * @param[in]  rec          receiver list
        * @param[in]  sat          satellite list
        * @return
            @retval std::map of ionosphere data records    
        */
        std::map<base_time, gnss_data_ion_record> &get_records(std::string rec, std::string sat);

    protected:
        std::map<std::pair<std::string, std::string>, std::map<base_time, gnss_data_ion_record>> _ions; ///< rec_sat_beg
    private:
        base_time _beg;         ///< begin time
        base_time _end;         ///< end time
        double _intv = 300.0; ///< time interval
        int _numb = 0;        ///< number
        std::set<std::string> _sats;    ///< satellite list
        std::set<std::string> _recs;    ///< receiver list
    };
}

#endif // !GION_H
