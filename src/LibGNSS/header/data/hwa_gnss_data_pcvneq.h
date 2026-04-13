/**
 * @file            gpcvneq.h
 * @brief            The class for storaging PCV neq information for accumulating NEQ.
 */

#ifndef GPCVNEQ_H
#define GPCVNEQ_H

#include <vector>
#include <map>
#include <string>
#include <stdio.h>
#include <iostream>
#include "hwa_base_data.h"

using namespace hwa_base;

namespace hwa_gnss
{
    class  gnss_data_pcvneq : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_pcvneq();

        gnss_data_pcvneq(base_log spdlog);
        /** @brief default destructor. */
        virtual ~gnss_data_pcvneq(){};
        /**
         * @brief set zen beg
         * @param[in]  z         zen beg
         */
        void zen_beg(const double &z) { _zen_beg = z; }
        /**
         * @brief get zen beg
         * @return double zen beg
         */
        const double &zen_beg() const { return _zen_beg; }
        /**
         * @brief set zen end
         * @param[in]  z         zen end
         */
        void zen_end(const double &z) { _zen_end = z; }
        /**
         * @brief get zen end
         * @return double zen end
         */
        const double &zen_end() const { return _zen_end; }
        /**
         * @brief set dzen
         * @param[in]  d         dzen
         */
        void dzen(const double &d) { _dzen = d; }
        /**
         * @brief get dzen
         * @return double dzen
         */
        const double &dzen() const { return _dzen; }
        /**
         * @brief set azi beg
         * @param[in]  a         azi beg
         */
        void azi_beg(const double &a) { _azi_beg = a; }
        /**
         * @brief get azi beg
         * @return double azi beg
         */
        const double &azi_beg() const { return _azi_beg; }
        /**
         * @brief set azi end
         * @param[in]  a         azi end
         */
        void azi_end(const double &a) { _azi_end = a; }
        /**
         * @brief get azi end
         * @return double azi end
         */
        const double &azi_end() const { return _azi_end; }
        /**
         * @brief set azi end
         * @param[in]  d         azi end
         */
        void dazi(const double &d) { _dazi = d; }
        /**
         * @brief get dazi
         * @return double dazi
         */
        const double &dazi() const { return _dazi; }
        /**
         * @brief set pcv val
         * @param[in]  i         index of pcv
         * @param[in]  val       value of pcv
         */
        void pcv_val(const int &i, const double &val);
        /**
         * @brief set pcv value
         * @param[in]  vals      index and pcv value map
         */
        void pcv_val(const std::map<int, double> &vals);
        /**
         * @brief get one pcv value
         * @param[in]  i         index of pcv
         * @return double pcv value
         */
        double pcv_val(const int &i);
        /**
         * @brief get all pcv data
         * @return map<int, double> index and pcv vallue 
         */
        std::map<int, double> pcv_val();
        /**
         * @brief set pcv neq
         * @param[in]  i         index of neq
         * @param[in]  j         jndex if neq
         * @param[in]  val       value of neq
         */
        void pcv_neq(const int &i, const int &j, const double &val);
        /**
         * @brief set pcv neq
         * @param[in]  i         index of neq
         * @param[in]  vals      index and value neq
         */
        void pcv_neq(const int &i, const std::map<int, double> &vals);
        /**
         * @brief set pcv neq
         * @param[in]  vals      index, jndex, value
         */
        void pcv_neq(const std::map<int, std::map<int, double>> &vals);
        /**
         * @brief get pcv neq value
         * @param[in]  i         row
         * @param[in]  j         line
         * @return double pcv neq value
         */
        double pcv_neq(const int &i, const int &j);
        /**
         * @brief get pcv neq
         * @return map<int, map<int, double>> map pcv neq value
         */
        std::map<int, std::map<int, double>> pcv_neq();

    protected:
        std::map<int, double> _pcv_val;           ///< TODO
        std::map<int, std::map<int, double>> _pcv_neq; ///< TODO
        std::string _par_name;                    ///< parameter name
        double _zen_beg;                     ///< TODO
        double _zen_end;                     ///< TODO
        double _dzen;                        ///< TODO
        double _azi_beg;                     ///< TODO
        double _azi_end;                     ///< TODO
        double _dazi;                        ///< TODO
        int _npar;                           ///< TODO
    };
}
#endif