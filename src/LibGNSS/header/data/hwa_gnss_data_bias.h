#ifndef hwa_gnss_data_bias_H
#define hwa_gnss_data_bias_H

#include "hwa_base_eigendef.h"
#include "hwa_base_data.h"
#include "hwa_base_const.h"
#include "hwa_base_Time.h"
#include "hwa_gnss_data_obsmanager.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_data_bias. */
    class gnss_data_bias : public base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_data_bias();

        explicit gnss_data_bias(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_bias();

        /** @brief add single differential bias in meters. */
        void set(const base_time &beg, const base_time &end, double d, GOBS obs1, GOBS obs2 = X);
        void set(double d, GOBS obs1, GOBS obs2 = X);
        void ref(GOBS ref);

        /** @brief get signgle differential bias. */
        double bias(bool meter = true);

        GOBS gobs() const { return _gobs; }
        double val() const { return _val; }
        GOBS ref() const { return _ref; }

        /** @brief set/get valid from. */
        void beg(const base_time &t) { _beg = t; }
        const base_time &beg() const { return _beg; }

        /** @brief set/get valid until. */
        void end(const base_time &t) { _end = t; }
        const base_time &end() const { return _end; }

        /** @brief valid. */
        bool valid(const base_time &epo);

    private:
        base_time _beg; ///< valid from
        base_time _end; ///< valid until

        GOBS _gobs;  ///< observation
        GOBS _ref;   ///< reference
        double _val; ///< code biases are stored in meters
    };

} // namespace

#endif
