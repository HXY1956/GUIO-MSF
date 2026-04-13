
/**
*
* @file        gstochasticmodel.h
* @brief    Purpose: stochastic models
*/

#ifndef hwa_gnss_model_stochastic_H
#define hwa_gnss_model_stochastic_H

#include "hwa_base_time.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{

    /** @brief class for gnss_model_stochastic. */
    class gnss_model_stochastic
    {
    public:
        /** @brief default constructor. */
        gnss_model_stochastic();

        /** @brief default destructor. */
        virtual ~gnss_model_stochastic(){};

        /** @brief get Q. */
        virtual double getQ()
        {
            return 0.0;
        }

    protected:
    private:
    };

    /** @brief class for gnss_model_random_walk derive from gnss_model_stochastic. */
    class gnss_model_random_walk : public gnss_model_stochastic
    {
    public:
        /** @brief default constructor. */
        gnss_model_random_walk();

        /** @brief default destructor. */
        virtual ~gnss_model_random_walk(){};

        /** @brief get Q. */
        virtual double getQ();

        /** @brief set Tprev. */
        void setTprev(const base_time &);

        /** @brief set Tcurr. */
        void setTcurr(const base_time &);

        /** @brief update time. */
        void updateTime(const base_time &);

        /** @brief set Q. */
        void setq(double q);

        /** @brief get dt. */
        double get_dt();

    protected:
    private:
        base_time _Tprev; ///< time of prev
        base_time _Tcurr; ///< time of current
        double _dSig;   ///< dsigma
    };

    /** @brief class for gnss_model_white_noise derive from gnss_model_stochastic. */
    class gnss_model_white_noise : public gnss_model_stochastic
    {
    public:
        /** @brief constructor 1. */
        gnss_model_white_noise(double);

        /** @brief default destructor. */
        virtual ~gnss_model_white_noise(){};

        /** @brief get Q. */
        virtual double getQ();

        /** @brief set Var */
        void setVar(double);

    private:
        double _var; ///< var
    };

    /** @brief class for gnss_model_state_mode. */
    class gnss_model_state_mode
    {
    public:
        /** @brief default constructor. */
        gnss_model_state_mode();

        /** @brief constructor 1. */
        gnss_model_state_mode(int order, double dt, double noise);

        /** @brief default destructor. */
        virtual ~gnss_model_state_mode();

        int order; ///< order
        // x(t+1) = M*x(t)  P
        Matrix M;          ///< M
        Symmetric P; ///< P

    private:
        static const double _coeff[6]; ///coff
    };

} // namespace

#endif // STOCHASTIC_H
