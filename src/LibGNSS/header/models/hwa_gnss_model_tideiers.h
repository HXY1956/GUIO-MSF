/**
* @file            gtideIERS.h
* @brief        precise model for computing  correction
*/

#ifndef hwa_gnss_model_tideiers_H
#define hwa_gnss_model_tideiers_H

#include "hwa_set_base.h"
#include "hwa_gnss_model_tide.h"
#include "hwa_gnss_data_navde.h"
#include "hwa_gnss_all_atmloading.h"
#include "hwa_gnss_all_opl.h"

using namespace hwa_set;

namespace hwa_gnss
{
    /** @brief The class for gtide of IERS model.*/
    class gnss_model_tideiers : public gnss_model_tide
    {
    public:
        /** @brief constructor.
        *
        *param[in] l                set spdlog control
        */
        gnss_model_tideiers();

        /** @brief constructor.
        *
        *param[in] l                set spdlog control
        */
        gnss_model_tideiers(base_log spdlog);

        /** @brief destructor.
        *
        *param[in] l                set spdlog control
        *param[in] otl                TODO
        */
        gnss_model_tideiers(gnss_all_otl *otl);

        /** @brief destructor.
        *
        *param[in] l                set spdlog control
        *param[in] otl                TODO
        */
        gnss_model_tideiers(base_log spdlog, gnss_all_otl *otl);

        /** @brief destructor.*/
        virtual ~gnss_model_tideiers(){};

        /** @brief solid base_earth tides(Mark sure the pos in J2000).*/
        Triple tide_solid(const base_time &epo, Triple &xyz, Matrix &rot_trs2crs, gnss_data_navde *nav_planet);
        Triple tide_solid(const base_time &epo, Triple &xyz, Matrix &rot_trs2crs, Vector &sun_pos, Vector &moon_pos);

        /** @brief solid base_earth tide corrections to be computed in the frequency domain from the diurnal band*/
        /** added by kkzhang 2021/11/25 */
        /** Reference: IERS Conventions 2010 v1.0, Eq (7.12); STEP2DIU.F from IERS 2010 Conventions software collection */
        Triple tide_solid_frequency_diurnal(const base_time &epo, Triple &xyz);

        /** @brief solid base_earth tide corrections to be computed in the frequency domain from the long-period band*/
        /** added by kkzhang 2021/11/25 */
        /** Reference: IERS Conventions 2010 v1.0, Eq (7.13); STEP2LON.F from IERS 2010 Conventions software collection */
        Triple tide_solid_frequency_longperiod(const base_time &epo, Triple &xyz);

        /** @brief the out-of-phase corrections induced by mantle anelasticity in the diurnal band.*/
        /** added by yqyuan 2021/11/25 */
        /** Reference: IERS Conventions 2010 v1.0, Eq (7.10) */
        /** Remeber: SIN(PHIj)=Zj/Rj; SIN(LAMBDAj)=Yj/COS(PHIj)/Rj; COS(LAMBDAj)=Xj/COS(PHIj)/Rj */
        Triple _st1idiu(Triple &xyz, Vector &sun, Vector &moon, double &FAC2SUN, double &FAC2MON);

        /** @brief the out-of-phase corrections induced by mantle anelasticity in the semi-diurnal band.*/
        /** added by yqyuan 2021/11/25 */
        /** Reference: IERS Conventions 2010 v1.0, Eq (7.11) */
        /** Remeber: SIN(PHIj)=Zj/Rj; SIN(LAMBDAj)=Yj/COS(PHIj)/Rj; COS(LAMBDAj)=Xj/COS(PHIj)/Rj */
        Triple _st1isem(Triple &xyz, Vector &sun, Vector &moon, double &FAC2SUN, double &FAC2MON);

        /** @brief the corrections induced by the latitude dependence given by L^1 in Mathews et al. 1991.*/
        /** added by yqyuan 2021/11/25.*/
        /** Reference: IERS Conventions 2010 v1.0, Eq (7.7-7.9).*/
        Triple _st1l1(Triple &xyz, Vector &sun, Vector &moon, double &FAC2SUN, double &FAC2MON);

        /** @brief atmospheric tide loading.*/
        /** added by yqyuan for atmospheric pressure loading; 2021/11/30 */
        /** References: [1] IERS Conventions 2010, Sect. 7.1.3           */
        /**             [2] https://geophy.uni.lu/displacementgrids/     */
        /**             [3] /gamit/model/etide.f                         */
        Triple atmospheric_loading(const base_time &epoch, const Triple &xyz);

        void set_atm_grid(gnss_all_atmloading *atm_grid);

        gnss_all_atmloading *get_atm_grid();

        void set_opl_grid(gnss_all_opl *opl_grid);

        Triple load_oceanpole(const base_time &epo, const Triple &xRec, const double xp, const double yp);

        void set_mean_pole_model(modeofmeanpole mean_pole_model);

        /** @brief pole tides.*/
        Triple tide_pole();

        /** @brief pole tides.*/
        Triple tide_pole_pod(const base_time &epo, double xpole, double ypole, Triple &xyz);

        /** @brief ocean tide loading.*/
        Triple load_ocean(const base_time &epoch, const std::string &site, const Triple &xRec) override;

        /** @brief atmospheric tide loading.*/
        Triple load_atmosph() override;

        /** @brief get frequency of tide.*/
        Triple tide_freq(const std::string &site, const Triple &xRec, double gast);

        /** @brief get mean pole.*/
        void getMeanPole(double mjd, double &xpm, double &ypm);

    protected:
        double _EARTH_R = 6378.1366; // in km
        double _MASS_RATIO_MOON = 0.0123000371;
        double _MASS_RATIO_SUN = 332946.0482;
        // Solid tide corrections due to the frequency dependence of Love and Shida numbers in the diurnal band
        // References: IERS 2010 p107 Table 7.3a and STEP2DIU.F function from IERS 2010 software collection
        const double _solid_diurnal_tide[31][9] = {
            {-3.0, 0.0, 2.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0},
            {-3.0, 2.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0},
            {-2.0, 0.0, 1.0, -1.0, 0.0, -0.02, 0.0, 0.0, 0.0},
            {-2.0, 0.0, 1.0, 0.0, 0.0, -0.08, 0.0, -0.01, 0.01},
            {-2.0, 2.0, -1.0, 0.0, 0.0, -0.02, 0.0, 0.0, 0.0},
            {-1.0, 0.0, 0.0, -1.0, 0.0, -0.10, 0.0, 0.0, 0.0},
            {-1.0, 0.0, 0.0, 0.0, 0.0, -0.51, 0.0, -0.02, 0.03},
            {-1.0, 2.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0},
            {0.0, -2.0, 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0},
            {0.0, 0.0, -1.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 1.0, 0.0, 0.01, 0.0, 0.0, 0.0},
            {0.0, 2.0, -1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0},
            {1.0, -3.0, 0.0, 0.0, 1.0, -0.06, 0.0, 0.0, 0.0},
            {1.0, -2.0, 0.0, -1.0, 0.0, 0.01, 0.0, 0.0, 0.0},
            {1.0, -2.0, 0.0, 0.0, 0.0, -1.23, -0.07, 0.06, 0.01},
            {1.0, -1.0, 0.0, 0.0, -1.0, 0.02, 0.0, 0.0, 0.0},
            {1.0, -1.0, 0.0, 0.0, 1.0, 0.04, 0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0, -1.0, 0.0, -0.22, 0.01, 0.01, 0.0},
            {1.0, 0.0, 0.0, 0.0, 0.0, 12.00, -0.80, -0.67, -0.03},
            {1.0, 0.0, 0.0, 1.0, 0.0, 1.73, -0.12, -0.10, 0.0},
            {1.0, 0.0, 0.0, 2.0, 0.0, -0.04, 0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0, 0.0, -1.0, -0.50, -0.01, 0.03, 0.0},
            {1.0, 1.0, 0.0, 0.0, 1.0, 0.01, 0.0, 0.0, 0.0},
            {0.0, 1.0, 0.0, 1.0, -1.0, -0.01, 0.0, 0.0, 0.0},
            {1.0, 2.0, -2.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0},
            {1.0, 2.0, 0.0, 0.0, 0.0, -0.11, 0.01, 0.01, 0.0},
            {2.0, -2.0, 1.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0},
            {2.0, 0.0, -1.0, 0.0, 0.0, -0.02, 0.0, 0.0, 0.0},
            {3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

        // Solid tide corrections due to the frequency dependence of Love and Shida numbers in the long period band
        // References: IERS 2010 p108 Table 7.3b and STEP2LON.F function from IERS 2010 software collection
        const double _solid_long_tide[5][9] = {
            {0, 0, 0, 1, 0, 0.47, 0.23, 0.16, 0.07},
            {0, 2, 0, 0, 0, -0.20, -0.12, -0.11, -0.05},
            {1, 0, -1, 0, 0, -0.11, -0.08, -0.09, -0.04},
            {2, 0, 0, 0, 0, -0.13, -0.11, -0.15, -0.07},
            {2, 0, 0, 1, 0, -0.05, -0.05, -0.06, -0.03}};

        // atmospheric loading frequencies S1 S2
        double _atm_freq[2] = {7.27220521664304e-05, 0.000145444104332861};
        gnss_all_atmloading *_atm_grid = nullptr;
        gnss_all_opl *_opl = nullptr;
        // mean pole model
        modeofmeanpole _mean_pole_model;
    };
} // namespace

#endif // !GTIDEPOD_H
