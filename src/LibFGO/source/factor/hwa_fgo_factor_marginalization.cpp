#include "hwa_fgo_factor_marginalization.h"
#include "hwa_base_timecost.h"
#include "iomanip"

namespace hwa_fgo
{
	void ResidualBlockInfo::Evaluate()
	{
		residuals.resize(cost_function->num_residuals());

		std::vector<int> block_sizes = cost_function->parameter_block_sizes();
		raw_jacobians = new double *[block_sizes.size()];
		jacobians.resize(block_sizes.size());

		for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
		{
			jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
			raw_jacobians[i] = jacobians[i].data();
		}
		cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

		if (loss_function)
		{
			double residual_scaling_, alpha_sq_norm_;

			double sq_norm, rho[3];

			sq_norm = residuals.squaredNorm();
			loss_function->Evaluate(sq_norm, rho);

			double sqrt_rho1_ = sqrt(rho[1]);

			if ((sq_norm == 0.0) || (rho[2] <= 0.0))
			{
				residual_scaling_ = sqrt_rho1_;
				alpha_sq_norm_ = 0.0;
			}
			else
			{
				const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
				const double alpha = 1.0 - sqrt(D);
				residual_scaling_ = sqrt_rho1_ / (1 - alpha);
				alpha_sq_norm_ = alpha / sq_norm;
			}

			for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
			{
				jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
			}

			residuals *= residual_scaling_;
		}
	}

	MarginalizationInfo::~MarginalizationInfo()
	{
		for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
			delete it->second;

		for (int i = 0; i < (int)factors.size(); i++)
		{

			delete[] factors[i]->raw_jacobians;

			if(factors[i]->cost_function)
				delete factors[i]->cost_function;

			delete factors[i];
		}
	}

	void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
	{
		factors.emplace_back(residual_block_info);

		std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
		std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

		for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
		{
			double *addr = parameter_blocks[i];
			int size = parameter_block_sizes[i];
			parameter_block_size[reinterpret_cast<long>(addr)] = size;
		}

		for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
		{
			double *addr = parameter_blocks[residual_block_info->drop_set[i]];
			parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
		}
	}

	void MarginalizationInfo::preMarginalize()
	{
		for (auto it : factors)
		{
			it->Evaluate();

			std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
			for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
			{
				long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
				int size = block_sizes[i];
				if (parameter_block_data.find(addr) == parameter_block_data.end())
				{
					double *data = new double[size];
					memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
					parameter_block_data[addr] = data;
				}
			}
		}
	}

	int MarginalizationInfo::localSize(int size) const
	{
		return size == 7 ? 6 : size;
	}

	int MarginalizationInfo::globalSize(int size) const
	{
		return size == 6 ? 7 : size;
	}

	void* ThreadsConstructA(void* threadsstruct)
	{
		ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);

		//Eigen::IOFormat fmt(
		//	6,                  // precision 小数位数
		//	0,                  // 不对齐
		//	", ",               // 列之间分隔
		//	"\n",               // 行之间换行
		//	"[",                // 开始符号
		//	"]"                 // 结束符号
		//);

		for (auto it : p->sub_factors)
		{
			for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
			{
				int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
				int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
				if (size_i == 7)
					size_i = 6;
				Matrix jacobian_i = it->jacobians[i].leftCols(size_i);
				for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
				{
					int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
					int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
					if (size_j == 7)
						size_j = 6;
					Matrix jacobian_j = it->jacobians[j].leftCols(size_j);
					if (i == j)
						p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
					else
					{
						p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
						p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
					}


					//std::cout << "========== Margin Details ==========" << std::endl;
					//std::cout << "jacobian_i : "<< idx_i << "," << size_i << "\n" << jacobian_i.format(fmt) << std::endl;
					//std::cout << "\njacobian_j : " << idx_j << "," << size_j << "\n" << jacobian_j.format(fmt) << std::endl;
				}
				p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
			}
		}
		return threadsstruct;
	}

	void MarginalizationInfo::marginalize()
	{
		int pos = 0;
		for (auto &it : parameter_block_idx)
		{
			it.second = pos;
			pos += localSize(parameter_block_size[it.first]);
		}

		m = pos;

		for (const auto &it : parameter_block_size)
		{
			if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
			{
				parameter_block_idx[it.first] = pos;
				pos += localSize(it.second);
			}
		}

		n = pos - m;

		if (m == 0)
		{
			valid = false;
			printf("unstable tracking...\n");
			return;
		}

		Matrix A(pos, pos);
		Vector b(pos);
		A.setZero();
		b.setZero();


		TicToc t_thread_summing;
		std::thread tids[NUM_THREADS];
		ThreadsStruct threadsstruct[NUM_THREADS];
		int i = 0;
		for (auto it : factors)
		{
			threadsstruct[i].sub_factors.push_back(it);
			i++;
			i = i % NUM_THREADS;
		}
		for (int i = 0; i < NUM_THREADS; i++)
		{
			threadsstruct[i].A = Matrix::Zero(pos, pos);
			threadsstruct[i].b = Vector::Zero(pos);
			threadsstruct[i].parameter_block_size = parameter_block_size;
			threadsstruct[i].parameter_block_idx = parameter_block_idx;
			tids[i] = std::thread(ThreadsConstructA, (void*)&(threadsstruct[i]));
		}
		for (int i = NUM_THREADS - 1; i >= 0; i--)
		{
			tids[i].join();
			A += threadsstruct[i].A;
			b += threadsstruct[i].b;
		}
		//std::cout << "thread summing up costs " << t_thread_summing.toc() << " ms" << std::endl;

		//Eigen::IOFormat fmt(
		//	6,                  // precision 小数位数
		//	0,                  // 不对齐
		//	", ",               // 列之间分隔
		//	"\n",               // 行之间换行
		//	"[",                // 开始符号
		//	"]"                 // 结束符号
		//);


		//std::cout << "========== Margin A ==========" << std::endl;
		//std::cout << A.format(fmt) << std::endl;


		//std::cout << "========== Margin b ==========" << std::endl;
		//std::cout << b.format(fmt) << std::endl;

		//std::cout<<"A Size: ["<<A.rows()<<","<<A.cols()<<"]"<<std::endl;

		TicToc t_compute;
		TicToc t_step;

		t_step.tic();
		Matrix Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
		//std::cout << "  Amm computation: " << t_step.toc() << " ms" << std::endl;

		//t_step.tic();
		//Eigen::SelfAdjointEigenSolver<Matrix> saes(Amm);
		//std::cout << "  Eigen decomposition: " << t_step.toc() << " ms" << std::endl;

		//t_step.tic();
		//Matrix Amm_inv = saes.eigenvectors() * Vector((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
		//std::cout << "  Amm_inv computation: " << t_step.toc() << " ms" << std::endl;

		//Eigen::LDLT<Matrix> ldlt(Amm);
		//Matrix Amm_inv;

		//if (ldlt.info() == Eigen::Success) {
		//	Amm_inv = ldlt.solve(Matrix::Identity(m, m));
		//}
		//else {
		//	throw std::runtime_error("Matrix Amm is not positive definite or is singular.");
		//}

		double eps = 1e-6;
		Matrix Amm_damped = Amm;
		Amm_damped.diagonal().array() += eps;

		Eigen::LLT<Matrix> llt(Amm_damped);

		Matrix Amm_inv = llt.solve(Matrix::Identity(m, m));

		t_step.tic();
		Vector bmm = b.segment(0, m);
		Matrix Amr = A.block(0, m, m, n);
		Matrix Arm = A.block(m, 0, n, m);
		Matrix Arr = A.block(m, m, n, n);
		Vector brr = b.segment(m, n);
		//std::cout << "  Matrix/vector extraction: " << t_step.toc() << " ms" << std::endl;

		t_step.tic();
		A = Arr - Arm * Amm_inv * Amr;
		//std::cout << "  A matrix update: " << t_step.toc() << " ms" << std::endl;

		t_step.tic();
		b = brr - Arm * Amm_inv * bmm;
		//std::cout << "  b vector update: " << t_step.toc() << " ms" << std::endl;

		//std::cout << "Total A,b computation: " << t_compute.toc() << " ms" << std::endl;

		TicToc t_compute1;

		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
		Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
		Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));
	    
		//================ condition number ================

		//double max_eigenvalue = 0.0;
		//double min_eigenvalue = DBL_MAX;
		//
		//Eigen::VectorXd eigenvalues = saes2.eigenvalues();

		//for (int i = 0; i < eigenvalues.size(); i++)
		//{
		//	if (eigenvalues(i) > eps)
		//	{
		//		max_eigenvalue = std::max(max_eigenvalue, eigenvalues(i));
		//		min_eigenvalue = std::min(min_eigenvalue, eigenvalues(i));
		//	}
		//}

		//double condition_number = max_eigenvalue / min_eigenvalue;


		//std::cout << "========== Margin Condition ==========" << std::endl;

		//std::cout << "max eigenvalue = "
		//	<< std::setprecision(15)
		//	<< max_eigenvalue
		//	<< std::endl;

		//std::cout << "min eigenvalue = "
		//	<< std::setprecision(15)
		//	<< min_eigenvalue
		//	<< std::endl;

		//std::cout << "condition number = "
		//	<< std::scientific
		//	<< condition_number
		//	<< std::endl;

		//std::cout << "log10(condition number) = "
		//	<< std::log10(condition_number)
		//	<< std::endl;


		//// 输出特征值
		//std::cout << "Eigenvalues:\n";
		//std::cout << saes2.eigenvalues().format(fmt) << std::endl;

		//// 输出特征向量
		//std::cout << "Eigenvectors:\n";
		//std::cout << saes2.eigenvectors().format(fmt) << std::endl;

		Vector S_sqrt = S.cwiseSqrt();
		Vector S_inv_sqrt = S_inv.cwiseSqrt();

		linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
		linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;

		//std::cout << "Compute B " << t_compute1.toc() << " ms" << std::endl;
		//std::cout << "========== Margin Amm  ==========" << std::endl;
		//std::cout << Amm.format(fmt) << std::endl;

		//std::cout << "========== Margin Amm_inv ==========" << std::endl;
		//std::cout << Amm_inv.format(fmt) << std::endl;

		//std::cout << "========== Margin A_after  ==========" << std::endl;
		//std::cout << Amm.format(fmt) << std::endl;

		//std::cout << "========== Margin S  ==========" << std::endl;
		//std::cout << S.format(fmt) << std::endl;

		//std::cout << "========== Margin S_inv ==========" << std::endl;
		//std::cout << S_inv.format(fmt) << std::endl;

		//std::cout << "========== Margin S_sqrt  ==========" << std::endl;
		//std::cout << S_sqrt.format(fmt) << std::endl;

		//std::cout << "========== Margin S_inv_sqrt ==========" << std::endl;
		//std::cout << S_inv_sqrt.format(fmt) << std::endl;

		//std::cout << "========== Margin jaco ==========" << std::endl;
		//std::cout << linearized_jacobians.format(fmt) << std::endl;


		//std::cout << "========== Margin resi ==========" << std::endl;
		//std::cout << linearized_residuals.format(fmt) << std::endl;
	}

	std::vector<double *> MarginalizationInfo::getParameterBlocks(std::map<long, double *> &addr_shift)
	{
		std::vector<double *> keep_block_addr;
		keep_block_size.clear();
		keep_block_idx.clear();
		keep_block_data.clear();

		for (const auto &it : parameter_block_idx)
		{
			if (it.second >= m)
			{
				keep_block_size.push_back(parameter_block_size[it.first]);
				keep_block_idx.push_back(parameter_block_idx[it.first]);
				keep_block_data.push_back(parameter_block_data[it.first]);
				keep_block_addr.push_back(addr_shift[it.first]);                                                                                                                                                         
			}
		}                         
		sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

		return keep_block_addr;                    
	}

	MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info) :marginalization_info(_marginalization_info)
	{
		//int cnt = 0;
		for (auto it : marginalization_info->keep_block_size)
		{
			mutable_parameter_block_sizes()->push_back(it);
			//cnt += it;
		}
		//printf("residual size: %d, %d\n", cnt, n);
		set_num_residuals(marginalization_info->n);
	};

	bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
	{
		//printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
		//for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
		//{
		//    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
		//    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
		//printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
		//printf("residual %x\n", reinterpret_cast<long>(residuals));
		//}
		int n = marginalization_info->n;
		int m = marginalization_info->m;
		Vector dx(n);
		for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
		{
			int size = marginalization_info->keep_block_size[i];
			int idx = marginalization_info->keep_block_idx[i] - m;
			Vector x = Eigen::Map<const Vector>(parameters[i], size);
			Vector x0 = Eigen::Map<const Vector>(marginalization_info->keep_block_data[i], size);
			if (size != 7)
				dx.segment(idx, size) = x - x0;
			else
			{
				dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
				dx.segment<3>(idx + 3) = 2.0 * base_att_trans::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
				if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))
				{
					dx.segment<3>(idx + 3) = 2.0 * -base_att_trans::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
				}
			}
		}
		Eigen::Map<Vector>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
		if (jacobians)
		{

			for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
			{
				if (jacobians[i])
				{
					int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
					int idx = marginalization_info->keep_block_idx[i] - m;
					Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
					jacobian.setZero();
					jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
				}
			}
		}

		//std::cout << "\n================ Marginalization Factor Debug ================\n";


		//// 输出 dx
		//std::cout << "\n---------- dx (" << dx.size() << ") ----------\n";
		//for (int i = 0; i < dx.size(); i++)
		//{
		//	std::cout << std::setw(4) << i
		//		<< " : "
		//		<< std::fixed
		//		<< std::setprecision(6)
		//		<< dx(i)
		//		<< std::endl;
		//}


		//// 输出 residual
		//Eigen::Map<Eigen::VectorXd> res(residuals, n);

		//std::cout << "\n---------- Residual (" << n << ") ----------\n";
		//for (int i = 0; i < n; i++)
		//{
		//	std::cout << std::setw(4) << i
		//		<< " : "
		//		<< std::fixed
		//		<< std::setprecision(8)
		//		<< res(i)
		//		<< std::endl;
		//}


		//// 输出线性化jacobian
		//std::cout
		//	<< "\n---------- Linearized Jacobian "
		//	<< marginalization_info->linearized_jacobians.rows()
		//	<< " x "
		//	<< marginalization_info->linearized_jacobians.cols()
		//	<< " ----------\n";

		//Eigen::IOFormat fmt(
		//	Eigen::StreamPrecision,
		//	Eigen::DontAlignCols,
		//	", ",
		//	"\n"
		//);

		//std::cout
		//	<< marginalization_info->linearized_jacobians.format(fmt)
		//	<< std::endl;

		return true;
	}

}