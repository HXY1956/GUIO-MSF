#include <stdio.h>
#include <math.h>
#include "hwa_gnss_proc_fltmat.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    gnss_proc_fltmat::gnss_proc_fltmat(Symmetric Qp,
                         Symmetric Qu,
                         base_allpar xp,
                         base_allpar xu)
    {
        this->_Qp = Qp;
        this->_Qu = Qu;
        this->_xp = xp;
        this->_xu = xu;
    }

    gnss_proc_fltmat::~gnss_proc_fltmat()
    {
        _slips.clear();
        _data.clear();
    }

    Symmetric gnss_proc_fltmat::Qp()
    {
        return _Qp;
    }

    void gnss_proc_fltmat::Qp(const Symmetric &Qp)
    {
        this->_Qp = Qp;
    }

    Symmetric gnss_proc_fltmat::Qu()
    {
        return _Qu;
    }

    void gnss_proc_fltmat::Qu(const Symmetric &Qu)
    {
        this->_Qu = Qu;
    }

    Diag gnss_proc_fltmat::Noise()
    {
        return _Noise;
    }

    void gnss_proc_fltmat::Noise(const Diag &Noise)
    {
        this->_Noise = Noise;
    }

    base_allpar gnss_proc_fltmat::xp()
    {
        return _xp;
    }

    void gnss_proc_fltmat::xp(const base_allpar &xp)
    {
        this->_xp = xp;
    }

    base_allpar gnss_proc_fltmat::xu()
    {
        return _xu;
    }

    void gnss_proc_fltmat::xu(const base_allpar &xu)
    {
        this->_xu = xu;
    }

    base_time gnss_proc_fltmat::epo()
    {
        return _epo;
    }

    void gnss_proc_fltmat::epo(const base_time &epo)
    {
        this->_epo = epo;
    }

    std::set<std::string> gnss_proc_fltmat::slips()
    {
        return _slips;
    }

    void gnss_proc_fltmat::slips(const std::set<std::string> &cs)
    {
        this->_slips = cs;
    }

    std::vector<gnss_data_sats> gnss_proc_fltmat::data()
    {
        return _data;
    }

    void gnss_proc_fltmat::data(const std::vector<gnss_data_sats> &data)
    {
        this->_data = data;
    }

    void gnss_proc_fltmat::delParam(int i, int index)
    {
        if (i < 0 || index <= 0)
            return;
        index--;
        _Qu.Matrix_remRC(index, index);
        _Qp.Matrix_remRC(index, index);

        _xu.delParam(i);
        _xu.reIndex();
        _xp.delParam(i);
        _xu.reIndex();
    }

} // namespace
