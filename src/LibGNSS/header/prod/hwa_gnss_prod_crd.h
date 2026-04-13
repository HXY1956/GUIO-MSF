/**
*
* @file        gprodcrd.h
* @brief    crd class
*.
*/

#ifndef hwa_prodCRD_H
#define hwa_prodCRD_H

#include <iostream>
#include "hwa_gnss_Prod.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief cov type. */
    enum COV_TYPE
    {
        COV_XY,
        COV_XZ,
        COV_YZ
    };

    /** @brief class for gnss_prod_crd derive from gnss_prod. */
    class gnss_prod_crd : public gnss_prod
    {

    public:
        /** @brief constructor 1. */
        gnss_prod_crd(const base_time &t, std::shared_ptr<gnss_data_obj> pt = nullobj);

        gnss_prod_crd(base_log spdlog, const base_time &t, std::shared_ptr<gnss_data_obj> pt = nullobj);
        /** @brief default destructor. */
        virtual ~gnss_prod_crd();

        /** @brief std::set/get xyz. */
        void xyz(const Triple &xyz);
        Triple xyz() const;

        /** @brief std::set/get xyz rms. */
        void xyz_rms(const Triple &xyz_rms);
        Triple xyz_rms() const;

        /** @brief get xyz var. */
        Triple xyz_var() const;

        /** @brief std::set/get apr. */
        void apr(const Triple &apr);
        Triple apr() const;

        /** @brief std::set/get apr rms. */
        void apr_rms(const Triple &apr_rms);
        Triple apr_rms() const;

        /** @brief get apr var. */
        Triple apr_var() const;

        /** @brief add cov. */
        void cov(COV_TYPE type, double &cov);

        /** @brief get xyz cov. */
        double cov(COV_TYPE type) const;

    protected:
        Triple _xyz;     ///< xyz
        Triple _xyz_rms; ///< xyz rms

        Triple _apr;     ///< apr
        Triple _apr_rms; ///< apr rms

        double _xy_cov; ///< xy cov
        double _xz_cov; ///< xz cov
        double _yz_cov; ///< yz cov

    private:
    };

} // namespace

#endif
