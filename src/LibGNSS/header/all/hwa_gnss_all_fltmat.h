
#ifndef hwa_gnss_all_fltmat_H
#define hwa_gnss_all_fltmat_H

#include "hwa_base_data.h"
#include "hwa_gnss_proc_fltmat.h"
#include "hwa_base_allpar.h"
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
     *@brief Class for filter matrix setting derive from base_data
     */
    class gnss_all_fltmat : public base_data
    {

    public:
        /** @brief default constructor. */
        explicit gnss_all_fltmat();

        /**
         * @brief Construct a new t gallfltmat object
         * 
         * @param spdlog 
         */
        explicit gnss_all_fltmat(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_all_fltmat();

        /**
        * @brief add filter mat.
        *
        * @param[in]  mat    filter mat
        * @return    void
        */
        void add(const gnss_proc_fltmat &mat);

        /**
        * @brief synchronize par1 and par2.
        *
        * @param[in]  par1    param 1
        * @param[in]  Q1        cofactor mat 1
        * @param[in]  par2    param 2
        * @param[in]  Q2        cofactor mat 2
        * @param[in]  Noise    noise mat
        * @return    1
        */
        int common(base_allpar &par1, Symmetric &Q1, base_allpar &par2, Symmetric &Q2, Diag *Noise = NULL);

        /**
        * @brief synchronize smooth vector according to par1.
        *
        * @param[in]  Xu        
        * @param[in]  Qu     
        * @param[in]  Xsm    
        * @param[in]  Qsm    
        * @param[in]  Noise    noise mat
        * @return    1
        */
        int syncSMT(base_allpar &Xu, Symmetric &Qu, base_allpar &Xsm, Symmetric &Qsm, Diag *Noise = NULL);

        /**
        * @brief checking reset ambiguities.
        *
        * @param[in]  Xu
        * @param[in]  Qu
        * @param[in]  Xp
        * @param[in]  Qp
        * @param[in]  Xsm
        * @param[in]  Qsm
        * @param[in]  slips    
        * @return    1
        */
        int checkSlp(base_allpar &Xu, Symmetric &Qu, base_allpar &Xp, Symmetric &Qp, base_allpar &Xsm, Symmetric &Qsm, std::set<std::string> &slips);

        /**
        * @brief checking outliers.
        *
        * @param[in]  Xp
        * @param[in]  Qp
        * @param[in]  Xsm
        * @param[in]  Qu
        * @param[in]  slips
        * @return    1
        */
        int checkOutl(base_allpar &Xp, Symmetric &Qp, base_allpar &Xsm, Symmetric &Qu);

        /**
        * @brief clear data.
        *
        * @return    void
        */
        void clear();

        /**
        * @brief clear added ambiguities.
        *
        * @return    void
        */
        void clear_addAmb();

        /// Not necessary

        /**
        * @brief return both X1 and X2 have the sama position in cov matrix.
        *
        * @param[in]  par1    param 1
        * @param[in]  Q1        cofactor mat 1
        * @param[in]  par2    param 2
        * @param[in]  Q2        cofactor mat 2
        * @return    1
        */
        int reorder(base_allpar &X1, Symmetric &Q1, base_allpar &X2, Symmetric &Q2);

        /**
        * @brief reorder all synchronized filter mat.
        *
        * @param[in]  par1    param 1
        * @param[in]  Q1        cofactor mat 1
        * @param[in]  par2    param 2
        * @param[in]  Q2        cofactor mat 2
        * @return    1
        */
        int reorder2(base_allpar &X1, Symmetric &Q1, base_allpar &X2, Symmetric &Q2);

        /** ================================================ ******  =========================================================== */
    public:
        std::vector<gnss_proc_fltmat> data; ///<
        std::set<int> addAmb;        ///< add ambiguities

    protected:
    };

} // namespace

#endif
