/**
 * @file        gallopl.h
 * @brief        Storage the oceanpoleload data. 
 */

#ifndef hwa_gnss_all_opl_H
#define hwa_gnss_all_opl_H

#include <string>
#include <map>
#include "hwa_base_data.h"
#include "hwa_base_time.h"
#include "hwa_base_pair.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_data_opl.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class gnss_all_opl : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_all_opl();
        /** @brief default destructor. */
        ~gnss_all_opl();

        /**
        * @brief get opl data from _ren_r and _ren_l.
        * @return  is successful or not.
        * @param [in]  ell    antenna coordinates.
        * @param [in]  rne_r    ocean pole load tide coefficients from Desai (2002): u_r^R, u_n^R, u_e^R.
        * @param [in]  rne_i    ocean pole load tide coefficients from Desai (2002): u_r^I, u_n^I, u_e^I.
        */
        int data(const Triple &ell, Triple &rne_r, Triple &rne_i);

        /**
        * @brief push opl data into _ren_r and _ren_l.
        * @return  is successful or not.
        * @param [in]  lon    antenna longitude.
        * @param [in]  lat    antenna latitude.
        * @param [in]  rne_r    ocean pole load tide coefficients from Desai (2002): u_r^R, u_n^R, u_e^R.
        * @param [in]  rne_i    ocean pole load tide coefficients from Desai (2002): u_r^I, u_n^I, u_e^I.
        */
        void add(const double &lon, const double &lat, const Triple &rne_r, const Triple &rne_i);

        Triple kart2ell(const Triple &p, const double &a, const double &b);

    protected:
        std::map<base_pair, Triple> _rne_r;
        std::map<base_pair, Triple> _rne_i;
    };
}

#endif