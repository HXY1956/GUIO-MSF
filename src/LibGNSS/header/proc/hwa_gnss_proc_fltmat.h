
/**
* @file        gfltmat.h
* @brief    Purpose: implements gneq stacking class
*.
*/

#ifndef hwa_gnss_procFLTMAT_H
#define hwa_gnss_proc_fltmat_H

#include <map>
#include <vector>
#include <iostream>
#include "hwa_base_eigendef.h"

#include "hwa_base_par.h"
#include "hwa_base_allpar.h"
#include "hwa_gnss_all_rslt.h"
#include "hwa_gnss_data_SATDATA.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for gnss_proc_fltmat. */
    class gnss_proc_fltmat
    {
    public:
        /** @brief default constructor. */
        gnss_proc_fltmat(){};

        /** @brief constructor 1. */
        gnss_proc_fltmat(Symmetric Qp,
                  Symmetric Qu,
                  base_allpar xp,
                  base_allpar xu);

        /** @brief default destructor. */
        ~gnss_proc_fltmat();

        /** @brief get/set Qp. */
        Symmetric Qp();
        void Qp(const Symmetric &Qp);

        /** @brief get/set Qu. */
        Symmetric Qu();
        void Qu(const Symmetric &Qu);

        /** @brief get/set Noise. */
        Diag Noise();
        void Noise(const Diag &Noise);

        /** @brief get/set xp. */
        base_allpar xp();
        void xp(const base_allpar &xp);

        /** @brief get/set xu. */
        base_allpar xu();
        void xu(const base_allpar &xu);

        /** @brief get/set epo. */
        base_time epo();
        void epo(const base_time &t);

        /** @brief get/set slips. */
        std::set<std::string> slips();
        void slips(const std::set<std::string> &cs);

        /** @brief get/set data. */
        std::vector<gnss_data_sats> data();
        void data(const std::vector<gnss_data_sats> &data);

        /** @brief delet parameter. */
        void delParam(int i, int index);

    protected:
        Symmetric _Qp;      ///< Qp
        Symmetric _Qu;      ///< Qu
        Diag _Noise;    ///< Noise
        base_allpar _xp;            ///< xp
        base_allpar _xu;            ///< xu
        base_time _epo;             ///< epoch
        std::set<std::string> _slips;       ///< slips
        std::vector<gnss_data_sats> _data; ///< data
    };

} // namespace

#endif // GFLTMAT_H
