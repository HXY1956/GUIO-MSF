/**
* @file        glambda.h
* @brief    integer estimation with the LAMBDA method.
*/

#ifndef hwa_gnss_amb_lambda_H
#define hwa_gnss_amb_lambda_H

#include <cstddef>

namespace hwa_gnss
{
    /**
    *@ brief class for LAMADA search finding fixing ambiguity.
    */
    class gnss_amb_lambda
    {
    public:
        /** @brief default constructor. */
        gnss_amb_lambda(){};

        /** @brief default destructor. */
        virtual ~gnss_amb_lambda()
        {
            if (pDia)
                delete[] pDia;
        };

        /**
        * @brief integer estimation with the LAMBDA method
        * @param[in] iMaxCan
        * @param[in] iN        dimension of matrix
        * @param[in] pdQ
        * @param[in] pdA
        * @param[in] piNcan
        * @param[in] piPos
        * @param[in] pdCands   2-dimensional array to store the candidates
        * @param[in] boot   bootstrapping rate in amb fix
        * @param[in] pdDisall  according squared norms \hat{a}-\check{a} dA, piNcan, piPos, pdCands, pdDisal
        * @return void
        */
        void LAMBDA4(int iMaxCan, int iN, double *pdQ, double *pdA, int *piNcan, int *piPos, double *pdCands, double *pdDisall, double *boot);

        /*static void LAMBDA4(int iMaxcan, int n, double *arrdQ, double* arrdA, int* iNcan, int* iPos, double* arrdCands, double* arrdIsall);
        static void STOREs(int iCan, int iPos, int *piMax, double t, double *pdTmax, double *arrpdDist, double *arrpdDisall,
        double *arrpdCands, int iMaxCan, int n);*/

        /**
        * @brief backtrack in the search tree, internal subroutine for FI71.
        * @param[in] iN      dimension of matrix
        * @param[in] piI     level in the tree
        * @param[in] pdEnd   dp work std::vector
        * @param[in] pdDist  difference between the integer tried and \hat{a}_i
        * @param[in] pdLef   dp work std::vector
        * @param[in] pdLeft  dp work std::vector
        * @param[in] pdEnde  if .true., then search is done
        * @param[out] piI pdDist pdLeft pdEnde
        * return void
        */
        void BACKTs(int iN, int *piI, double *pdEnd, double *pdDist, double *pdLef, double *pdLeft, bool *pdEnde);

        /**
        * @brief Updates integral Z (transposed inverse); only column `first' until `last'.
        *    see: the LAMBDA method for integer ambiguity estimation: implementation aspects (section 3.5 & 3.9).
        * @param[in] iFirst   first column to be updated
        * @param[in] iLast      last column to be updated
        * @param[in] iN          dimension of the system
        * @param[in] pdL      lower triangular matrix L
        * @param[in] pdA      Z (transposed) a, with a the std::vector of unknowns
        * @param[in] pdZti      Z (transposed inverse) matrix
        * @param[out] pdL pdA pdZti
        * @return void
        */
        void ZTRANi(int iFirst, int iLast, int iN, double *pdL, double *pdA, double *pdZti);

        /**
        * @brief stores integer std::vectors and corresponding distances.
        * @param[in] iCan       Min (number of std::vectors found until now, MaxCan)
        * @param[in] iPos        position in disall/cands to put the new found vec
        * @param[in] piMax        position in disall/cands of the std::vector with the largest distance of the ican std::vectors with minimum distance found until now
        * @param[in] dT         distance of the new found std::vector
        * @param[in] pdTmax        the largest distance of the ican std::vectors with minimum distance found until now
        * @param[in] pdDist        difference between the integer tried and \hat{a}_n
        * @param[in] pdDisall    2d-array to store the integer std::vectors
        * @param[in] pdCands    distance of the MaxCan integer std::vectors
        * @param[in] iMaxCan    number of integer std::vectors required
        * @param[in] iN            dimension of the system (number of DD ambiguities)
        * @param[out] piMax  pdTmax  pdDist  pdDisall
        * @return void
        */
        void STOREs(int iCan, int iPos, int *piMax, double dT, double *pdTmax, double *pdDist, double *pdDisall, double *pdCands, int iMaxCan, int iN);

        /**
        * @brief collects integer std::vectors and corresponding squared distances.
        * @param[in] iN         dimension of the system
        * @param[in] iMaxCan    number of minimum integer std::vectors requiered
        * @param[in] dD_1        first element of diagonal matrix D
        * @param[in] dLef_1     first element of dp work std::vector lef
        * @param[in] dLeft_1    first element of dp work std::vector left
        * @param[in] dRight_1   first element of dp work std::vector right
        * @param[in] dChic        Chi squared
        * @param[in] pdDist        difference between the integer tried and \hat{a}_n
        * @param[in] pdEnd_1    first element of dp work std::vector end
        * @param[in] piNcan        number of integer std::vectors found
        * @param[in] pdDisall | 2-dimensional array to store the candidates
        * @param[in] pdCands  | according squared norms \hat{a}-\check{a}
        * @param[in] pdTmax     the largest distance of the Min (ncan,MaxCan) std::vectors with minimum distance found until now
        * @param[in] piMax  position in disall/cands of the std::vector with the largest distance of the Min (ncan,MaxCan) std::vectors
        *                    with minimum distance found until now
        * @param[out] pdDist pdEnd_1 piNcan pdDisall  pdCands pdTmax  piMax
        * @return void
        */
        void COLLECTs(int iN, int iMaxCan, double dD_1, double dLef_1, double dLeft_1, double dRight_1, double dChic, double *pdDist,
                      double *pdEnd_1, int *piNcan, double *pdDisall, double *pdCands, double *pdTmax, int *piMax);

        /**
        * @brief transpose(L) D L factorization of Q, L over-writes Q bordering method, computation of Googe number (section 3.3)s.
        * @param[in] pdL   symmetric lower triangular matrix Q to be factored
        * @param[in] pdD   diagonal
        * @param[in] iN    dimension of the matrix
        * @param[in] dEps  if the Googe number is smaller than eps the matrix is considered to be singular
        * @param[out] pdL pdD
        * @return void
        */
        int FMFAC6(double *pdL, double *pdD, int iN, double dEps);

        /**
        * @brief compute the inverse of a lower triangular matrix Lm may over write L.
        * @param[in] iN    dimension of the matrix
        * @param[in] pdL   lower triangular matrix
        * @param[in] pdLm  inverse of L (also lower triangular)
        * @param[in] pdVec double precision work array with lenght n
        * @param[out] pdL, pdLm, pdVec
        * @return void
        */
        void INVLT2d(int iN, double *pdL, double *pdLm, double *pdVec);

        /**
        * @brief Computation of the Z matrix .
        * @param[in] iN      dimension of the system
        * @param[in] pdL     lower triangular matrix L
        * @param[in] pdD     diagonal matrix D stored as a one-dimensional array
        * @param[in] pdA     Z (transposed) a, with a the original std::vector of unknowns
        * @param[in] pdZti   Z (transposed inverse) transformation matrix
        * @param[out] pdL, pdD, pdA, pdZti
        * @return void
        */
        void SRC1i(int iN, double *pdL, double *pdD, double *pdA, double *pdZti);

        /**
        * @brief computes squared distance of partially rounded float std::vectors to
        *               the float std::vector in the metric of the variance - covariance
        *               matrix(see `The LAMBDA method for integer ambiguity estimation :
        *               implementation aspects', 5.8: The volume of the ellipsoidal region.
        * @param[in] iN      number of ambiguities
        * @param[in] pdL     lower triangular matrix: Q   = L D L, although L is lower triangular, in this example,
        *                    program L is stored column-wise in a 2-dimensional array, to avoid the necessity of a dedicated storage ins_scheme.
        * @param[in] pdD     diagonal matrix
        * @param[in] pdDist  double precision std::vector of length n
        * @param[in] pdE     double precision std::vector of length n
        * @param[in] pdA     float solution
        * @param[in] dTm     tm(1) is smallest norm, tm(2) one-but-smallest
        * @param[in] dT         double precision std::vector of length n
        * @param[out] pdL, pdD, pdA, pdDist,pdE,pdT
        * @return void
        */
        void CHIstrt4(int iN, double *pdD, double *pdL, double *pdDist, double *pdE, double *pdA, double dTm[2], double *pdT);

        /**
        * @brief takes inproduct of column 'icol' of triangular unit matrix L and std::vector a (exclusive the diagonal element).
        * @param[in] iN      length of a
        * @param[in] pdL     triangular matrix L
        * @param[in] pdA     std::vector a
        * @param[in] iCol    see rol
        * @param[out] pdL, pdA
        * @return dDINKi
        */
        double DINKi(int iN, double *pdL, double *pdA, int iCol);

        /**
        * @brief 'ceiling' (rounding towards +infinity) of a double precision number.
        * @param[in]  dD     double precision number
        * @param[out] iJNT2  ceiled number
        * @return iJNT2
        */
        int JNT2(double dD);

        /**
        * @brief finds 'MaxCan' integer std::vectors whose distance to the the real std::vector 'a' is minimal in the metric of Q=transpose(L) D L. Only
        *        integer std::vectors with a distance less than sqrt(Chic) are regarded.
        * @param[in] dChic      Chi squared
        * @param[in] iMaxCan    number of minimum integer std::vectors requiered
        * @param[in] iN            dimension of matrix
        * @param[in] pdA        the std::vector with real valued estimates \hat{a} (float solution)
        * @param[in] pdD        | diagonal matrix           -1        *
        * @param[in] pdL        | lower triangular matrix: Q   = L D L
        *                        | although L is lower triangular, in this example
        *                        | program L is stored column-wise in a 2-dimensional
        *                        | array, to avoid the necessity of a dedicated
        *                        | storage ins_scheme.
        * @param[in] pdLef        dp work std::vector with length = n
        * @param[in] pdLeft     dp work std::vector with length = n+1
        * @param[in] pdRight    dp work std::vector with length = n+1
        * @param[in] pdDis        difference between the integer tried and \hat{a}_i,length = n
        * @param[in] pdEnd        dp work std::vector with length = n
        * @param[in] pdDq        dp work std::vector with length = n
        * @param[in] piNcan        number of integer std::vectors found
        * @param[in] pdDisall    | 2-dimensional array to store the candidate
        * @param[in] pdCands    | according squared norms \hat{a}-\check{a}s
        * @param[in] piPos        column number in 'cands' where the candidate belonging to the minimum distance is stored
        * @param[out] pdA, pdD, dpdL, pdLef, pdLeft, pdRight,pdDist, pdEnd, pdDq, ipiNcan, pdDisall, pdCands, piPos
        * @return void
        */
        void FI71(double dChic, int iMaxCan, int iN, double *pdA, double *pdD, double *pdL, double *pdLef, double *pdLeft, double *pdRight,
                  double *pdDist, double *pdEnd, double *pdDq, int *piNcan, double *pdDisall, double *pdCands, int *piPos);

        double pBootStrapping(const double &sig);

    public:
        double *pDia = NULL;
    };

}

#endif