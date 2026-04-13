#include "hwa_gnss_prod_crd.h"

namespace hwa_gnss
{
    gnss_prod_crd::gnss_prod_crd(const base_time &t, std::shared_ptr<gnss_data_obj> pt)
        : gnss_prod(t, pt)
    {
        id_type(POS);
        id_group(GRP_prodUCT);
    }
    gnss_prod_crd::gnss_prod_crd(base_log spdlog, const base_time &t, std::shared_ptr<gnss_data_obj> pt)
        : gnss_prod(spdlog, t, pt)
    {

        id_type(POS);
        id_group(GRP_prodUCT);
    }

    // destructor
    // --------------
    gnss_prod_crd::~gnss_prod_crd()
    {
    }

    // add xyz
    // --------------------------
    void gnss_prod_crd::xyz(const Triple &xyz)
    {
        _xyz = xyz;
    }

    // get xyz
    // --------------------------
    Triple gnss_prod_crd::xyz() const
    {
        return _xyz;
    }

    // add xyz rms
    // -------------------------
    void gnss_prod_crd::xyz_rms(const Triple &xyz_rms)
    {
        _xyz_rms = xyz_rms;
    }

    // get xyz rms
    // -------------------------
    Triple gnss_prod_crd::xyz_rms() const
    {
        return _xyz_rms;
    }

    // get xyz var
    // -------------------------
    Triple gnss_prod_crd::xyz_var() const
    {
        Triple var(_xyz_rms[0] * _xyz_rms[0],
                      _xyz_rms[1] * _xyz_rms[1],
                      _xyz_rms[2] * _xyz_rms[2]);

        return var;
    }

    // add apr
    // --------------------------
    void gnss_prod_crd::apr(const Triple &apr)
    {
        _apr = apr;
    }

    // get apr
    // --------------------------
    Triple gnss_prod_crd::apr() const
    {
        return _apr;
    }

    // add apr rms
    // -------------------------
    void gnss_prod_crd::apr_rms(const Triple &apr_rms)
    {
        _apr_rms = apr_rms;
    }

    // get apr rms
    // -------------------------
    Triple gnss_prod_crd::apr_rms() const
    {
        return _apr_rms;
    }

    // get apr var
    // -------------------------
    Triple gnss_prod_crd::apr_var() const
    {
        Triple var(_apr_rms[0] * _apr_rms[0],
                      _apr_rms[1] * _apr_rms[1],
                      _apr_rms[2] * _apr_rms[2]);

        return var;
    }

    // add cov
    // -------------------------
    void gnss_prod_crd::cov(COV_TYPE type, double &cov)
    {
        switch (type)
        {
        case COV_XY:
            _xy_cov = cov;
            break;
        case COV_XZ:
            _xz_cov = cov;
            break;
        case COV_YZ:
            _yz_cov = cov;
            break;
        default:
            _xy_cov = _xz_cov = _yz_cov = 0.0;
        }
    }

    // get xyz rms
    // -------------------------
    double gnss_prod_crd::cov(COV_TYPE type) const
    {
        switch (type)
        {
        case COV_XY:
            return _xy_cov;
            break;
        case COV_XZ:
            return _xz_cov;
            break;
        case COV_YZ:
            return _yz_cov;
            break;
        default:
            return 0.0;
        }
    }

} // namespace
