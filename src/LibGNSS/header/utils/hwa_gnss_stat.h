#ifndef hwa_gnss_base_stat_H
#define hwa_gnss_base_stat_H

#include "hwa_base_data.h"
#include "hwa_base_log.h"
#include "hwa_base_pair.h"
#define CONF_INT 3 ///< confident interval factor

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for gnss_base_stat derive from base_data. */
    class gnss_base_stat : public base_data
    {
    public:
        typedef std::map<base_pair, int> gnss_base_stat_HIST;

        /**
        *@brief just construct.
        */
        gnss_base_stat(double cint = CONF_INT);

        /**
        *@brief construct + calculate statistics.
        */
        explicit gnss_base_stat(std::vector<double> &data, double cint = CONF_INT);
        gnss_base_stat(base_log spdlog, double cint = CONF_INT);
        gnss_base_stat(base_log spdlog, std::vector<double> &data, double cint = CONF_INT);

        /** @brief default destructor. */
        virtual ~gnss_base_stat();

        /** @brief clear + calculate statistics. */
        void add_data(std::vector<double> &data);

        /** @brief get status. */
        virtual bool valid() { return _valid; }

        /** @brief calculate statistics. */
        virtual int calc_stat(double sig = 0.0);

        /** @brief calculate quartiles (upper/lower). */
        virtual int calc_quartiles(double &low, double &upp);

        /** @brief interquartile limits. */
        virtual int calc_iqrlimits(double &low, double &upp);

        // virtual double calc_mad(){    _calc_mad();    return _mad;    }

        // virtual int robust_mean();

        /** @brief get min. */
        virtual double get_min() { return _min; }

        /** @brief get max. */
        virtual double get_max() { return _max; }

        /** @brief get var. */
        virtual double get_var() { return _var; }

        /** @brief get rms. */
        virtual double get_rms() { return _rms; }

        /** @brief get sdev. */
        virtual double get_sdev() { return _sdev; }

        /** @brief get mean. */
        virtual double get_mean() { return _mean; }

        /** @brief get median. */
        virtual double get_median() { return _medi; }
        // virtual double get_mad(){                     return _mad;    }

        /** @brief get data size. */
        virtual int get_size() { return _data.size(); }

        /** @brief get outl. */
        virtual int get_outl() { return _outl.size(); }

        gnss_base_stat_HIST histogram(std::vector<double> &data, std::set<double> &bound);

    protected:
        /** @brief add data. */
        void _add_data(std::vector<double> &data);

        /** @brief clear. */
        virtual void _clear();

        /** @brief calculate statistics with outlier detection and set validity status. */
        virtual int _statistics(double sig = 0.0);

        /** @brief internal purposes only (no validity status changed). */
        virtual int _calc_minmax();
        virtual int _calc_median();
        virtual int _calc_mean();
        virtual int _calc_sdev();
        virtual int _calc_rms();
        // virtual int  _calc_mad();

        /** @brief check residuals (optional use of external sigma). */
        virtual int _chk_resid(double sig = 0.0);

        // virtual double _p(double v);

        bool _valid;

        std::vector<double> _data; ///< data
        std::vector<double> _outl; ///< outl

        double _cint; ///< cint
        double _sdev; ///< sdev
        double _var;  ///< var
        double _rms;  ///< RMS
        double _mean; ///< mean
        double _medi; ///< medi
        // double _mad;
        double _min; ///< min
        double _max; ///< max
    };

} // namespace

#endif
