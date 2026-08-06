#ifndef HWA_FGO_P_DD_ING_FACTOR
#define HWA_FGO_P_DD_ING_FACTOR

#include"hwa_base_posetrans.h"
#include"hwa_gnss_model_bias.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace hwa_base;

namespace hwa_fgo
{
	class PseudorangeDDINGFactor : public ceres::SizedCostFunction<1, 7>
	{
	public:

		/**
		 * @brief Pseudorange DD with INS/GNSS lever arm factor constructor
		 * Initializes DD factor including IMU-GNSS antenna lever arm offset
		 */
		PseudorangeDDINGFactor(const base_time &cur_time, const std::pair<std::string, std::string> &base_rover_site, const base_allpar  &params, const std::vector<std::pair<gnss_data_sats, gnss_data_sats>> &DD_sat_data, gnss_model_bias *bias_model, const std::pair<FREQ_SEQ, GOBSBAND> &freq_band, const Eigen::Vector3d &lever_arm);
		
		/**
		 * @brief Update rover coordinates in parameter set for INS/GNSS
		 * Replaces rover position parameters considering lever arm transformation
		 */
		void updatePara(base_allpar &params_tmp, const Eigen::Vector3d &Pi, const Eigen::Vector3d &Vi = Eigen::Vector3d::Identity()) const;
		
		/**
		 * @brief Transform design matrices to Eigen format for INS/GNSS pseudorange
		 * Converts sparse design matrix, weight matrix and residuals to dense Eigen matrices
		 * Handles coordinate parameters considering lever arm effects
		 */
		void trans2Eigen(const std::vector<std::vector<std::pair<int, double>>> &B, const std::vector<double> &P, const std::vector<double> &l, Eigen::Matrix<double, 2, 3> &B_new, Eigen::Matrix<double, 2, 2> &P_new, Eigen::Matrix<double, 2, 1> &l_new) const;
		
		/**
		 * @brief Evaluate INS/GNSS pseudorange DD factor with lever arm compensation
		 *
		 * Computes residuals and Jacobians considering IMU-GNSS antenna offset.
		 * Handles pose parameters and lever arm transformation for pseudorange observations.
		 * Transforms INS pose to GNSS antenna frame and constructs DD observations with proper covariance scaling for tight coupling integration.
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
		Eigen::Vector3d _lever_arm;
	};
}
#endif