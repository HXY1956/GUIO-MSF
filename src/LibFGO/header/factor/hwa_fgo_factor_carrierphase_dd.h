#ifndef HWA_FGO_L_DD_FACTOR
#define HWA_FGO_L_DD_FACTOR

#include"hwa_base_posetrans.h"
#include"hwa_gnss_model_bias.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace hwa_base;

namespace hwa_fgo
{
	class CarrierphaseDDFactor : public ceres::SizedCostFunction<1, 3, 1, 1>
	{
	public:
		
		/**
		 * @brief Carrier phase DD factor constructor
		 * Initializes with time, site std::pairs, parameters, satellite data and frequency band
		 */
		CarrierphaseDDFactor(const base_time &cur_time, const std::pair<std::string, std::string> &base_rover_site, const base_allpar  &params, const std::vector<std::pair<gnss_data_sats, gnss_data_sats>> &DD_sat_data, gnss_model_bias *bias_model, const std::pair<FREQ_SEQ, GOBSBAND> &freq_band);
		
		/**
		 * @brief Update parameters for carrier phase DD factor
		 * Updates rover coordinates, reference and non-reference satellite ambiguities in parameter set for linearization point adjustment
		 */
		void updatePara(base_allpar & params_tmp, const double &ref_sd_amb, const double &nonref_sd_amb, const Eigen::Vector3d &Pi, const Eigen::Vector3d &Vi = Eigen::Vector3d::Identity()) const;
		
		/**
		 * @brief Transform design matrices to Eigen format
		 * Converts sparse design matrix, weight matrix and residuals to dense Eigen matrices for efficient linear algebra operations in optimization
		 */
		void trans2Eigen(const std::vector<std::vector<std::pair<int, double>>> &B, const std::vector<double> &P, const std::vector<double> &l, Eigen::Matrix<double, 2, 5> &B_new, Eigen::Matrix<double, 2, 2> &P_new, Eigen::Matrix<double, 2, 1> &l_new) const;
		
		/**
		 * @brief Evaluate carrier phase double-difference factor for optimization
		 *
		 * Computes residuals and Jacobians for carrier phase DD observations in factor graph.
		 * Handles coordinate updates, ambiguity parameters, and measurement weighting.
		 * Constructs DD equations from single-difference observations and transforms to optimization-friendly format with proper covariance scaling.
		 */
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		/**
		 * @brief Debug function for factor verification
		 * Tests residual and Jacobian computation with current parameters
		 * Outputs numerical results for validation and debugging purposes
		 */
		void check(double **parameters);		
	protected:
		base_time _cur_time;
		std::pair<std::string, std::string> _base_rover_site;
		base_allpar _params;
		//shared_ptr<gnss_model_bias> _gprecise_bias_model = nullptr;		
		gnss_model_bias *_gprecise_bias_model = nullptr;
		std::pair<FREQ_SEQ, GOBSBAND> _freq_band;
		std::vector<std::pair<gnss_data_sats, gnss_data_sats>> _DD_sat_data;

	};
}
#endif