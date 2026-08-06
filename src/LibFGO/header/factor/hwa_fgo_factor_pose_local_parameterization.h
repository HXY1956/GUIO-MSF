#ifndef HWA_FGO_POSE_LOCAL_PARAMETERIZATION_H
#define HWA_FGO_POSE_LOCAL_PARAMETERIZATION_H

#include"hwa_base_posetrans.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace hwa_base;

namespace hwa_fgo
{
	/**
	*@brief  Class for pose parameterization
	*/
	class  PoseLocalParameterization : public ceres::LocalParameterization
	{
		/**
		 * @brief Apply pose perturbation on manifold
		 * Updates position and quaternion using tangent space increments
		 * Maintains quaternion normalization for SO(3) manifold
		 */
		virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

		/**
		 * @brief Compute pose parameterization Jacobian
		 * Returns identity mapping for position and quaternion tangent space
		 */
		virtual bool ComputeJacobian(const double *x, double *jacobian) const;


		virtual int GlobalSize() const { return 7; };


		virtual int LocalSize() const { return 6; };
	};

}
#endif