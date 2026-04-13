#include <iostream>
#include <iomanip>
#include "hwa_base_typeconv.h"
#include "hwa_gnss_model_stochastic.h"

using namespace std;

namespace hwa_gnss
{

    // Constant stochastic model

    gnss_model_stochastic::gnss_model_stochastic()
    {
    }

    // Random walk stochastic model

    gnss_model_random_walk::gnss_model_random_walk() : gnss_model_stochastic()
    {
        _dSig = 9999.9;

        base_time Tfirst(1980, 01, 01, 00, 00, 00, base_time::GPS);
        setTprev(Tfirst);
        setTcurr(Tfirst);
    }

    void gnss_model_random_walk::setq(double q)
    {
        this->_dSig = q;
    }

    void gnss_model_random_walk::setTprev(const base_time &Tprev)
    {
        this->_Tprev = Tprev;
    }

    void gnss_model_random_walk::setTcurr(const base_time &Tcurr)
    {
        this->_Tcurr = Tcurr;
    }

    double gnss_model_random_walk::getQ()
    {
        double Q;
        double q = (_dSig * 1e-3 * _dSig * 1e-3) / 3600;
        //   double q = (_dSig*1.e-3)/3600;

        Q = q * std::abs(_Tcurr - _Tprev);

#ifdef DEBUG
        std::cout << "Current time: " << _Tcurr.str("%Y:%m:%d  %H:%M:%S") << std::endl;
        std::cout << "Previous time: " << _Tprev.str("%Y:%m:%d  %H:%M:%S") << std::endl;
        std::cout << "dT [s]: " << _Tcurr - _Tprev << std::endl;
        std::cout << scientific << setprecision(3) << "_dSig: " << _dSig << std::endl;
        std::cout << scientific << setprecision(3) << "q: " << q << std::endl;
        std::cout << scientific << setprecision(3) << "Q: " << Q << std::endl;
#endif

        return Q;
    }

    double gnss_model_random_walk::get_dt()
    {
        return (_Tcurr - _Tprev);
    }

    void gnss_model_random_walk::updateTime(const base_time &Tnew)
    {
        setTprev(_Tcurr);
        setTcurr(Tnew);
    }

    // White noise stochastic model

    gnss_model_white_noise::gnss_model_white_noise(double var) : gnss_model_stochastic()
    {
        this->_var = var;
    }

    void gnss_model_white_noise::setVar(double var)
    {
        this->_var = var;
    }

    double gnss_model_white_noise::getQ()
    {
        return this->_var;
    }

    const double gnss_model_state_mode::_coeff[6] = {1.0, -sqrt(3), 2 * sqrt(3), sqrt(5), -6 * sqrt(5), 12 * sqrt(5)};

    gnss_model_state_mode::gnss_model_state_mode() : order(0)
    {
    }

    gnss_model_state_mode::gnss_model_state_mode(int order, double dt, double noise) : order(order)
    {
        // set the Matrix M P
        M = Matrix(order, order);

        // set Matrix M
        for (int Row = order - 1; Row >= 0; Row--)
        {
            M(Row, Row) = 1.0;
            for (int Col = Row + 1; Col < order; Col++)
            {
                M(Row, Col) = M(Row, Col - 1) * dt / ((double)(Col - Row));
            }
        }

        // set Matrix P
        Matrix P_temp(order, order);
        int count = 0;
        for (int Row = order - 1; Row >= 0; Row--)
        {
            for (int Col = order - 1; Col >= Row; Col--)
            {
                P_temp(Row, Col) = _coeff[count] * pow(dt, Col - order + 1);
                count++;
            }
        }

        P_temp = P_temp / (noise * sqrt(dt));

        P.matrixW() = P_temp.transpose() * P_temp;
    }

    gnss_model_state_mode::~gnss_model_state_mode()
    {
    }

} // namespace
