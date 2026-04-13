#ifndef hwa_gnss_amb_bdeci_H
#define hwa_gnss_amb_bdeci_H

namespace hwa_gnss
{
    /**
     * @brief class for Judge whether ambiguities meet the requirement of fixing. 
     */
    class gnss_amb_bdeci
    {
    public:
        /**
         * @brief Construct a new t gbdeci object
         */
        gnss_amb_bdeci();
        /**
         * @brief Construct a new t gbdeci object
         * @param[in]  cutdev    current deviation
         * @param[in]  cutsig    current sigma
         * @param[in]  cutprob   current prob
         * @param[in]  cutdeci   current deci
         */
        gnss_amb_bdeci(const double &cutdev, const double &cutsig,
                 const double &cutprob, const double &cutdeci);
        /**
         * @brief Destroy the t gbdeci object
         */
        virtual ~gnss_amb_bdeci(){};
        /**
         * @brief TODO
         * @param[in]  dEst      doc
         * @param[in]  dSigma    doc
         * @param[in]  iIh       doc
         * @return true 
         * @return false 
         */
        bool bdeci(double dEst, double dSigma, int iIh);
        /**
         * @brief TODO
         * @param[in]  dEst      doc
         * @param[in]  dSigma    doc
         * @param[in]  iIh       doc
         * @param[in]  dCutdev   doc
         * @param[in]  dCutsig   doc
         * @param[in]  dProb     doc
         * @param[in]  dDeci     doc
         */
        void bdeci(double dEst, double dSigma, int iIh, double dCutdev, double dCutsig, double &dProb, double &dDeci);

    private:
        /**
         * @brief get erf
         * @param[in]  dX        dx
         * @return double 
         */
        double _erf(double dX);
        /**
         * @brief get erfc
         * @param[in]  dX        dx
         * @return double 
         */
        double _erfc(double dX);
        double _gammp(double dA, double dX);
        double _gammq(double dA, double dX);
        double _gammln(double dXx);
        void _gcf(double &dGammcf, double dA, double dX, double &dGln);
        void _gser(double &dGamser, double dA, double dX, double &dGln);

    private:
        double _cutdev;  ///< TODO
        double _cutsig;  ///< TODO
        double _cutprob; ///< TODO
        double _cutdeci; ///< TODO
    };
}

#endif
