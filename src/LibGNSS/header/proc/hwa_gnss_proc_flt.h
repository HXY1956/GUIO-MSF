#ifndef hwa_gnss_proc_flt_h
#define hwa_gnss_proc_flt_h

#include "hwa_base_eigendef.h"
#include "hwa_base_allpar.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief Base class for all filtering technique. */
    class gnss_proc_flt
    {
    public:
        /** @brief default constructor. */
        gnss_proc_flt();

        /** @brief default destructor. */
        virtual ~gnss_proc_flt();

        /** @brief update parameter. */
        virtual void update() {}

        /**
        * @brief update parametere.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P        P matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        virtual void update(const Matrix &A, const Diag &P, const Vector &l, Vector &dx, Symmetric &Q){};

        /**
        * @brief update parametere.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P        P matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        virtual void update(const Matrix &A, const Symmetric &P, const Vector &l, Vector &dx, Symmetric &Q){};

        /**
        * @brief update parametere.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  R        R matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        virtual void update(const Matrix& A, const Matrix& R, const Vector& l, Vector& dx, Symmetric& Q) {};

        /**
        * @brief update parametere for EL amb.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P        P matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        virtual void update_AmbEl(const Matrix &A, const Diag &P, const Vector &l, Vector &dx, const Symmetric &Q, Symmetric &Q_el){};

        /**
        * @brief smooth estimate.
        *
        * @param[in]  Xp           Xp Pars in flt
        * @param[in]  Xu              Xu Pars in flt
        * @param[in]  Qp           Qp matrix in flt
        * @param[in]  Qu              Qu matrix in flt
        * @param[in]  Xsm              Xsm Pars in flt
        * @param[in]  Qsm              Qsm matrix in flt
        * @param[in]  Noise              Noise in flt
        * @return void
        */
        virtual void smooth(base_allpar &Xp, base_allpar &Xu, Symmetric &Qp, Symmetric &Qu, base_allpar &Xsm, Symmetric &Qsm, Diag &Noise){};

        /**
        * @brief add data.
        *
        * @param[in]  param        Pars in flt
        * @param[in]  dx              dx matrix in flt
        * @param[in]  Qx           Qx matrix in flt
        * @param[in]  sigma0       sigma0 in flt
        * @param[in]  Qx0              Qx0 matrix in flt
        * @return void
        */
        virtual void add_data(const base_allpar &param, const Vector &dx, const Symmetric &Qx, const double &sigma0, const Symmetric &Qx0);

        /**
        * @brief add data.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P           P matrix in flt
        * @param[in]  l              l matrix in flt
        * @return void
        */
        virtual void add_data(const Matrix &A, const Symmetric &P, const Vector &l);

        /**
        * @brief add data.
        *
        * @param[in]  vtpv                 v*p*v in flt
        * @param[in]  nobs_total           number of obs in flt
        * @param[in]  npar_number              number of pars in flt
        * @return void
        */
        virtual void add_data(const double &vtpv, const int &nobs_total, const int &npar_number);

        /**
        * @brief add virtual obs to flt.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P           P matrix in flt
        * @param[in]  l              l matrix in flt
        * @return void
        */
        virtual void add_virtual_obs(const Matrix &A, const Symmetric &P, const Vector &l);

        /**
        * @brief get virtual obs to flt.
        *
        * @param[out]  A        A matrix in flt
        * @param[out]  P        P matrix in flt
        * @param[out]  l           l matrix in flt
        * @return void
        */
        virtual void get_virtual_obs(Matrix &A, Symmetric &P, Vector &l);

        /**
        * @brief detect outlier.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  Q        Q matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  l        l matrix in flt
        * @param[in]  P           P matrix in flt
        * @return void
        */
        virtual bool outlierDetect(const Matrix &A, const Symmetric &Q, const Vector &dx, const Vector &l, Symmetric &P);

        /** @brief reset Qx. */
        virtual void resetQ();

        /** @brief std::set Qx Matrix specified location 
        * @param[in]  row        row of Qx matrix in flt
        * @param[in]  col        col of Qx matrix in flt
        * @param[in]  xx         value for (row,col) of Qx matrix in flt
        * 
        */
        void change_Qx(int row, int col, double xx);

        /** @brief std::set dx  specified location 
        * @param[in]  n          n of dx matrix in flt
        * @param[in]  xx         value for (n) of dx matrix in flt
        * 
        */
        void change_dx(int n, double xx);

        /** @brief std::set stdx specified location 
        * @param[in]  n          n of dx matrix in flt
        * @param[in]  xx         value for (n) of stdx matrix in flt
        * 
        */
        void change_stdx(int n, double xx);

        /** @brief std::set sigma0 by  specified sigma 
        * @param[in]  n          n of dx matrix in flt
        * @param[in]  xx         value for (n) of sigma0 matrix in flt
        * 
        */
        void change_sigma0(double sigma);

        /** @brief std::set vtpv by  specified value 
        * @param[in]  xx         value for vtpv  in flt
        * 
        */
        void change_vtpv(double xx);

        /** @brief get stdx */
        Vector stdx();

        /** @brief get dx */
        Vector dx() { return _dx; }

        /** @brief get Qx */
        Symmetric Qx() { return _Qx; }

        /** @brief get param */
        base_allpar param() { return _param; }

        /** @brief get sigma0 */
        double sigma0() { return _sigma0; }

        /** @brief get vtpv */
        double vtpv() { return _vtpv; }

        /** @brief get nobs total */
        int nobs_total() { return _nobs_total; }

        /** @brief get npar number */
        int npar_number() { return _npar_number; }

        /** @brief std::set/get amb 
        * @param[in]  state         amb state  in flt
        */
        void amb(bool state) { _amb = state; }

        /** @brief std::set/get amb */
        bool amb() { return _amb; }

    protected:
        Vector _dx;    ///< dx
        Vector _stdx;  ///< stdx
        Symmetric _Qx; ///< Qx
        //LX added for WL constraint
        Symmetric _Qx_tmp;    ///< tmp Qx matrix
        Symmetric _Qx0;       ///< Qx0 matrix
        Matrix _A;                  ///< A matrix
        Matrix _A_virtual;          ///< A matrix for virtual using
        Symmetric _P;         ///< P matrix
        Symmetric _P_virtual; ///< P matrix for virtual using
        Vector _l;            ///< l matrix
        Vector _l_virtual;    ///< l matrix for virtual using
        base_allpar _param;           ///< all pars in flt
        double _sigma0;             ///< sigma0
        double _vtpv;               ///< value of v*p*v
        int _nobs_total;            ///< number of obs
        int _npar_number;           ///< number of par
        bool _amb = false;          ///< fix amb
    };

    /** @brief class for Classical formule for Kalman filter. */
    class gnss_proc_kalman : public gnss_proc_flt
    {

    public:
        /** @brief default constructor. */
        gnss_proc_kalman();

        /** @brief default destructor. */
        ~gnss_proc_kalman();

        /** @brief update parameter. */
        virtual void update();
        void update(const Matrix &A, const Diag &P, const Vector &l, Vector &dx, Symmetric &Q);
        void update(const Matrix &A, const Symmetric &P, const Vector &l, Vector &dx, Symmetric &Q);
        void update(const Matrix& A, const Matrix& R, const Vector& l, Vector& dx, Symmetric& Q);
    };

    /** @brief class for Square root covariance filter derive from gnss_proc_flt. */
    class gnss_proc_srf : public gnss_proc_flt
    {
    public:
        /** @brief default constructor. */
        gnss_proc_srf();

        /** @brief default destructor. */
        ~gnss_proc_srf();

        /** @brief update parameter. */
        virtual void update();

        /**
        * @brief update parametere.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P        P matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        void update(const Matrix &A, const Diag &P, const Vector &l, Vector &dx, Symmetric &Q);

        /**
        * @brief update parametere.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P        P matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        void update(const Matrix &A, const Symmetric &P, const Vector &l, Vector &dx, Symmetric &Q);
    };

    /** @brief class for Square root information filter derive from gnss_proc_flt. */
    class gnss_proc_srif : public gnss_proc_flt
    {
    public:
        /** @brief default constructor. */
        gnss_proc_srif();

        /** @brief default destructor. */
        ~gnss_proc_srif();

        /**
        * @brief update parametere.
        *
        * @param[in]  A        A matrix in flt
        * @param[in]  P        P matrix in flt
        * @param[in]  l           l matrix in flt
        * @param[in]  dx       dx matrix in flt
        * @param[in]  Q              Q matrix in flt
        * @return void
        */
        void update(const Matrix &A, const Diag &P, const Vector &l, Vector &dx, Symmetric &Q);
    };

} // namespace

#endif
