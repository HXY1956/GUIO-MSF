/**
* @file            gclkdif.h
* @brief        clock differ processing for GNSS satellite
*/
#ifndef hwa_gnss_proc_clkdif_H
#define hwa_gnss_proc_clkdif_H

#include "hwa_base_iof.h"
#include "hwa_gnss_all_prec.h"
#include "hwa_gnss_proc_lsq.h"

namespace hwa_gnss
{
    /**
    *@brief       Class for GNSS clock differ processing
    */
    class gnss_proc_clkdif
    {
    public:
        /** @brief constructor. */
        gnss_proc_clkdif();

        /**
        * @brief  constructor.
        * @param[in]    std::set    setting information
        * @param[in]    spdlog    spdlog file
        */
        gnss_proc_clkdif(set_base *set, base_log spdlog);

        /** @brief destructor. */
        virtual ~gnss_proc_clkdif();

    public:
        /**
        * @brief  batch processing for GNSS clock differ
        * @param[in]  beg         begin time from XML
        * @param[in]  end         end time from XML
        * @return
            @retval true    clock differ correctly
            @retval false   clock differ failed
        */
        bool processBatch();

        /**
        * @brief  std::set rinexc data
        * @param[in]  gorb rinexc data
        */
        void setCLKPRD(gnss_all_nav *gclkprd);
        void setCLKREF(gnss_all_nav *gclkref);

    protected:
        // For spdlog
        base_log _spdlog; ///< Genereal   spdlog output.

        base_iof *_clkdif; ///< write clock differ result
        // For std::set
        set_base *_set; ///< Base setting.

        // data for calulate
        gnss_all_nav *_gclkprd; ///< rinexc product data.
        gnss_all_nav *_gclkref; ///< rinexc reference data.

        base_time _beg;     ///< begin time from xml
        base_time _end;     ///< end time from xml
        double _sampling; ///< sampling
        std::set<std::string> _sat; ///< satellite prn.
        std::string _refsat;   ///<reference satallite

        int _nepoch;                     ///< number of epochs
        int _numsat;                     ///< number of satallites
        std::vector<std::vector<double>> _res;     ///< clock differ result
        std::vector<std::vector<double>> _res_rms; ///< rms statistic result
        std::vector<std::vector<int>> _flag;       ///< whether the data is valid
        std::vector<int> _res_del;            ///< number of delete epochs
        double _mean_rms;
        double _mean_std;
        int _ncount; ///< ncount for the position of refsat

#ifdef BMUTEX
        boost::mutex _mutex;
#endif
        base_mutex _gmutex;

    protected:
        /**
        * @brief   check the data needed.
        * @return
            @retval true    data is completed
            @retval false   some data is lack
        */
        bool _get_setting();

        bool _set_out();

        bool _check_time(std::string sat);

        bool _get_sat();

        bool _init_par();

        bool _rms_statistics();

        bool _generate_result();

        void _int2sstream(std::ostringstream &os, int num, int width)
        {
            if ((num > 0.99999 * pow(10, width)) || (num < -0.99999 * pow(10, width - 1)))
                os << std::setw(width) << std::fixed << std::setfill('*') << std::right << '*';
            else
                os << std::setw(width) << std::fixed << std::setfill(' ') << std::right << num;
        }

        void _double2sstream(std::ostringstream &os, double value, int width, int prec)
        {
            int wd = value > 0 ? width - prec - 1 : width - prec - 2;
            if (wd < 0 || round(abs(value)) > 0.99999 * pow(10, wd))
                os << std::setw(width) << std::fixed << std::setfill('*') << std::right << '*';
            else
                os << std::setw(width) << std::fixed << std::setfill(' ') << std::setprecision(prec) << std::right << value;
        }
    };
}
#endif // !GCLKDIF_H#pragma once