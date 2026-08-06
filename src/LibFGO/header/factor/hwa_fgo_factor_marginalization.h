#ifndef HWA_FGO_MARGINALIZATION_FACTOR_H
#define HWA_FGO_MARGINALIZATION_FACTOR_H

#include"hwa_base_posetrans.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace hwa_base;

namespace hwa_fgo
{
	const int NUM_THREADS = 4;

	/**
	*@brief class for construting residual blocks
	*/
	struct ResidualBlockInfo
	{
		ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, const std::vector<double *> &_parameter_blocks, const std::vector<int> &_drop_set)
			: cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

		/**
		 * @brief Evaluate residual block with robust loss function
		 *
		 * Computes residuals and Jacobians for general factors, applies robust loss functions,
		 * and scales residuals/Jacobians for outlier rejection. 
		 * Handles complex parameter blocks including pose manifolds with proper tangent space operations.
		 */
		void Evaluate();

		ceres::CostFunction *cost_function;
		ceres::LossFunction *loss_function;
		std::vector<double *> parameter_blocks;
		std::vector<int> drop_set;

		double **raw_jacobians = nullptr;
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
		Vector residuals;

		int localSize(int size)
		{
			return size == 7 ? 6 : size;
		}
	};

	/**
	*@brief ThreadsStruct for constructing H matrix in parallel
	*/
	struct ThreadsStruct
	{
		std::vector<ResidualBlockInfo *> sub_factors;
		Matrix A;
		Vector b;
		std::map<long, int> parameter_block_size;		/// global size
		std::map<long, int> parameter_block_idx;		/// local size
	};

	/**
	*@brief  class for storing the prior information for Marginalization
	*/
	class MarginalizationInfo
	{
	public:
		MarginalizationInfo() { valid = true; };
		~MarginalizationInfo();

		int localSize(int size) const;
		int globalSize(int size) const;

		/**
		 * @brief Add residual block to marginalization information
		 *
		 * Stores residual factors and manages parameter block indexing for marginalization.
		 * Tracks parameter sizes and identifies blocks to be marginalized (dropped).
		 */
		void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);

		/**
		 * @brief Precompute residuals and cache parameter data for marginalization
		 *
		 * Evaluates all residual factors and stores current parameter values.
		 * Prepares data structures for Schur complement computation.
		 */
		void preMarginalize();

		/**
		 * @brief Perform Schur complement marginalization
		 *
		 * Executes multi-threaded marginalization using Schur complement method.
		 * Partitions parameters into marginalized (m) and retained (n) sets, computes information matrices, and performs eigenvalue decomposition to obtain linearized Jacobians and residuals for prior factors.
		 */
		void marginalize();

		/**
		 * @brief Get retained parameter blocks after marginalization
		 *
		 * Extracts parameter blocks to be kept (not marginalized) and updates their addresses using the provided shift mapping. Prepares data structures for prior factor construction.
		 */
		std::vector<double *> getParameterBlocks(std::map<long, double *> &addr_shift);

		std::vector<ResidualBlockInfo *> factors;
		int m=0, n=0;
		std::map<long, int> parameter_block_size;		/// global size
		int sum_block_size=0;
		std::map<long, int> parameter_block_idx;		/// local size
		std::map<long, double *> parameter_block_data;

		std::vector<int> keep_block_size;						/// global size
		std::vector<int> keep_block_idx;						/// local size
		std::vector<double *> keep_block_data;

		Matrix linearized_jacobians;
		Vector linearized_residuals;
		const double eps = 1e-8;
		bool valid;

	};

	/**
	*@brief the prior factor for optimization
	*/
	class MarginalizationFactor : public ceres::CostFunction
	{
	public:

		/**
		 * @brief Construct marginalization prior factor
		 * Initializes parameter block sizes from marginalization info
		 * Sets up residual dimension for prior constraint
		 */
		explicit MarginalizationFactor(MarginalizationInfo* _marginalization_info);

		/**
		 * @brief Evaluate marginalization prior factor
		 *
		 * Computes residuals and Jacobians for marginalized prior constraint.
		 * Handles quaternion manifold for pose parameters and linear error propagation from Schur complement marginalization results.
		 */
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		MarginalizationInfo* marginalization_info;
	};
}
#endif