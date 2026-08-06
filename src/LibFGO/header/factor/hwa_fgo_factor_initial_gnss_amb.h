#ifndef HWA_FGO_INITIAL_AMB_FACTOR_H
#define HWA_FGO_INITIAL_AMB_FACTOR_H

#include"hwa_base_posetrans.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace hwa_base;

namespace hwa_fgo
{
	/**
	*@brief InitialPoseFactor Class for initializing pose factors
	*/
	class InitialGnssAMB : public ceres::SizedCostFunction<1, 1>
	{
	public:

		/**
		 * @brief GNSS ambiguity prior factor constructor
		 * Sets initial ambiguity constraint with uncertainty weighting
		 */
		InitialGnssAMB(const double & gnss_ambiguity)
		{
			initial_amb = gnss_ambiguity;
			sqrt_info = 0.033;
		}

		/**
		 * @brief Evaluate GNSS ambiguity prior factor
		 *
		 * Computes residual and Jacobian for ambiguity parameter constraint.
		 * Simple linear factor for float ambiguity initialization.
		 */
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
		{
			double c_amb = parameters[0][0];
			
			double res = c_amb - initial_amb;
			
			residuals[0] = sqrt_info * res;
			if (jacobians)
			{
				if (jacobians[0])
				{
					Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_ambiguity1(jacobians[0]);
					jacobian_ambiguity1 = sqrt_info* Eigen::Matrix<double, 1, 1>::Identity();
				}

			}
			return true;

		}				
		double initial_amb;
		double  sqrt_info;
	};

}
#endif