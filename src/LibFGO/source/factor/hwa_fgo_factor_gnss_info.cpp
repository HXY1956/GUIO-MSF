#include "hwa_fgo_factor_gnss_info.h"
#include "hwa_base_timecost.h"
#include <iostream>
#include <iomanip>
using namespace std;

void hwa_fgo::GNSSResidualBlockInfo::Evaluate()
{
	residuals.resize(cost_function->num_residuals());
	std::vector<int> block_sizes = cost_function->parameter_block_sizes();
	raw_jacobians = new double *[block_sizes.size()];
	jacobians.resize(block_sizes.size());
	for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
	{
		jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
		raw_jacobians[i] = jacobians[i].data();
		//dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
	}
	cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

	if (loss_function)
	{
		double residual_scaling_, alpha_sq_norm_;

		double sq_norm, rho[3];

		sq_norm = residuals.squaredNorm();
		loss_function->Evaluate(sq_norm, rho);
		//printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

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

hwa_fgo::GNSSInfo::~GNSSInfo()
{
	//for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
	//	delete it->second;

	for (int i = 0; i < (int)factors.size(); i++)
	{

		delete[] factors[i]->raw_jacobians;

		delete factors[i]->cost_function;

		delete factors[i];
	}

}

void hwa_fgo::GNSSInfo::addResidualBlockInfo(GNSSResidualBlockInfo * residual_block_info, const map<long, std::vector<int>> &para_ids)
{
	factors.emplace_back(residual_block_info);
	para_index.emplace_back(para_ids);
	/// parameter blocks
	std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
	/// parameter block size
	std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();
	/// get address and size of parameter blocks
	for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
	{
		double *addr = parameter_blocks[i];
		int size = parameter_block_sizes[i];
		parameter_block_size[reinterpret_cast<long>(addr)] = size;
	}
}
void hwa_fgo::GNSSInfo::addResidualBlockInfo(GNSSResidualBlockInfo * residual_block_info)
{
	factors.emplace_back(residual_block_info);	
	/// parameter blocks
	std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
	/// parameter block size
	std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();
	/// get address and size of parameter blocks
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

int hwa_fgo::GNSSInfo::localSize(int size) const
{
	return size == 7 ? 6 : size;
}

void hwa_fgo::GNSSInfo::constructEqu(Matrix variance)
{
	/// get index of parameter block
	Qx0 = variance;
	int pos = 0;
	for (const auto &it : parameter_block_size)
	{
		if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
		{
			//parameter_block_idx[it.first] = localSize(pos);
			pos += localSize(parameter_block_size[it.first]);
			//pos += ;

			//int size_i = localSize(it.second);
			//parameter_block_idx[it.first] = pos;
			//pos += size_i;
		}
	}
	para_size = pos;
	n_size = factors.size();
	Matrix J(n_size, pos);
	Vector r(n_size);
	J.setZero();

	r.setZero();	
	int j_row_count = 0;
	int obs_count = 0;
	for (auto it : factors)
	{
		it->Evaluate(); /// get residuals and jacobians
		int r_size = it->cost_function->num_residuals();
		for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
		{
			int j_col_count = 0; /// colunm index of jacobian matrix
			//int size_i = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];			
			int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
			assert(size_i != 7);
			Matrix jacobian_i = it->jacobians[i].block(0, j_col_count, r_size, size_i);			
			assert(r_size == jacobian_i.rows());
			long str = reinterpret_cast<long>(it->parameter_blocks[i]);
			auto it_find = para_index[obs_count].find(str);
			if (it_find != para_index[obs_count].end())
			{
				std::vector<int> jacobian_col_id = para_index[obs_count][reinterpret_cast<long>(it->parameter_blocks[i])];
				J.block(j_row_count, jacobian_col_id[0], r_size, size_i) += -jacobian_i;
				j_col_count = j_col_count + size_i;
			}
		}		
		r.segment(j_row_count, r_size) += it->residuals;
		j_row_count = j_row_count + r_size;
		obs_count++;
	}
	linearized_jacobians = J;
	linearized_residuals = r;
	weight.resize(linearized_residuals.rows(), linearized_residuals.rows());
	weight.setIdentity();	
	//posterior 
	Matrix Qx_inverse =Qx0.inverse()+ linearized_jacobians.transpose()*linearized_jacobians;		
	////Qx = (linearized_jacobians.transpose()*linearized_jacobians).inverse();		
	Qx = Qx_inverse.inverse();
	// normalized post-fit residuals
	Matrix Qv = linearized_jacobians * Qx * linearized_jacobians.transpose() + weight;	
	v_norm.resize(linearized_residuals.rows());
	for (int i = 0; i < v_norm.rows(); i++)
	{
		v_norm(i)= sqrt(1.0 / Qv(i, i)) * linearized_residuals(i);
	}	
	int freedom = linearized_jacobians.rows() - linearized_jacobians.cols();
	if (freedom < 1)
	{
		cout << "No redundant observations!" << endl;		
		freedom = 1;
	}
	Vector vtPv = linearized_residuals.transpose()*linearized_residuals;
	//freedom += linearized_jacobians.cols();
	sig_unit = vtPv(0) / freedom;
	vtpv = vtPv(0);	
	valid = true;
	Qx = sig_unit * sig_unit * Qx;

	
}

void hwa_fgo::GNSSInfo::constructEqu(Matrix variance, int var_type)
{
	assert(var_type == 0 || var_type == 1);
	/// get index of parameter block
	Qx0 = variance;
	int pos = 0;
	for (const auto &it : parameter_block_size)
	{
		if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
		{
			int size_i = localSize(it.second);
			parameter_block_idx[it.first] = pos;
			pos += size_i;
		}
	}
	para_size = pos;
	n_size = factors.size();
	Matrix J(n_size, pos);
	Vector r(n_size);
	J.setZero();

	r.setZero();
	int j_row_count = 0;
	int obs_count = 0;
	for (auto it : factors)
	{
		it->Evaluate(); /// get residuals and jacobians
		int r_size = it->cost_function->num_residuals();
		for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
		{
			int j_col_count = 0; /// colunm index of jacobian matrix
			int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
			assert(size_i != 7);
			Matrix jacobian_i = it->jacobians[i].block(0, j_col_count, r_size, size_i);
			assert(r_size == jacobian_i.rows());
			long str = reinterpret_cast<long>(it->parameter_blocks[i]);
			auto it_find = para_index[obs_count].find(str);
			if (it_find != para_index[obs_count].end())
			{
				std::vector<int> jacobian_col_id = para_index[obs_count][reinterpret_cast<long>(it->parameter_blocks[i])];
				J.block(j_row_count, jacobian_col_id[0], r_size, size_i) += -jacobian_i;
				j_col_count = j_col_count + size_i;
			}
		}
		r.segment(j_row_count, r_size) += it->residuals;
		j_row_count = j_row_count + r_size;
		obs_count++;
	}
	linearized_jacobians = J;
	linearized_residuals = r;
	weight.resize(linearized_residuals.rows(), linearized_residuals.rows());
	weight.setIdentity();
	//cout << "linearized_jacobians: " << endl;
	//cout<<linearized_jacobians << endl;
	//cout << "Qx0: " << endl;
	//cout<< Qx0 << endl;
	//posterior 
	if (var_type == 0)
	{
		Matrix Qx_inverse = Qx0.inverse() + linearized_jacobians.transpose()*linearized_jacobians;
		////Qx = (linearized_jacobians.transpose()*linearized_jacobians).inverse();		
		Qx = Qx_inverse.inverse();
	}
	else if (var_type == 1)
	{
		Qx = Qx0;
	}
	// normalized post-fit residuals
	Matrix Qv = linearized_jacobians * Qx * linearized_jacobians.transpose() + weight;
	v_norm.resize(linearized_residuals.rows());
	for (int i = 0; i < v_norm.rows(); i++)
	{
		v_norm(i) = sqrt(1.0 / Qv(i, i)) * linearized_residuals(i);
	}
	int freedom = linearized_jacobians.rows() - linearized_jacobians.cols();
	if (freedom < 1)
	{
		cout << "No redundant observations!" << endl;
		freedom = 1;
	}
	Vector vtPv = linearized_residuals.transpose()*linearized_residuals;
	sig_unit = vtPv(0) / freedom;
	vtpv = vtPv(0);
	valid = true;
}

void hwa_fgo::GNSSInfo::constructEqu_fromCeres(Matrix variance)
{
	int pos = 0;
	for (const auto& it : parameter_block_size)
	{
		if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
		{
			//parameter_block_idx[it.first] = localSize(pos);
			pos += localSize(parameter_block_size[it.first]);
			//pos += ;
		}
	}
	para_size = pos;
	n_size = factors.size();
	Matrix J(n_size, pos);
	Vector r(n_size);
	J.setZero();

	r.setZero();
	int j_row_count = 0;
	int obs_count = 0;
	for (auto it : factors)
	{
		it->Evaluate(); /// get residuals and jacobians
		int r_size = it->cost_function->num_residuals();
		for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
		{
			int j_col_count = 0; /// colunm index of jacobian matrix
			int size_i = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
			Matrix jacobian_i = it->jacobians[i].block(0, j_col_count, r_size, size_i);
			assert(r_size == jacobian_i.rows());
			std::vector<int> jacobian_col_id = para_index[obs_count][reinterpret_cast<long>(it->parameter_blocks[i])];
			J.block(j_row_count, jacobian_col_id[0], r_size, size_i) += -jacobian_i;
			j_col_count = j_col_count + size_i;
		}
		r.segment(j_row_count, r_size) += it->residuals;
		j_row_count = j_row_count + r_size;
		obs_count++;
	}
	linearized_jacobians = J;
	linearized_residuals = r;
	weight.resize(linearized_residuals.rows(), linearized_residuals.rows());
	weight.setIdentity();
	//posterior 
	Qx0 = Matrix::Identity(linearized_jacobians.cols(), linearized_jacobians.cols()) * 1;
	Matrix Qx_inverse = Qx0.inverse() + linearized_jacobians.transpose() * linearized_jacobians;
	Qx = variance;
	if (Qx(0, 0) == 0)
		Qx = Qx_inverse.inverse();
	//std::cout.precision(15);
	//cout << "Qx: " << endl;
	//std::cout << Qx << std::endl;
	// normalized post-fit residuals
	Matrix Qv = linearized_jacobians * Qx * linearized_jacobians.transpose() + weight;
	v_norm.resize(linearized_residuals.rows());
	for (int i = 0; i < v_norm.rows(); i++)
	{
		v_norm(i) = sqrt(1.0 / Qv(i, i)) * linearized_residuals(i);
	}
	int freedom = linearized_jacobians.rows() - linearized_jacobians.cols();
	if (freedom < 1)
	{
		cout << "No redundant observations!" << endl;
		freedom = 1;
	}
	Vector vtPv = linearized_residuals.transpose() * linearized_residuals;
	//freedom += linearized_jacobians.cols() - 3;
	sig_unit = vtPv(0) / freedom;
	vtpv = vtPv(0);
	valid = true;
}


void hwa_fgo::GNSSInfo::preMarginalize()
{
	for (auto it : factors)
	{
		it->Evaluate();        

		// Iterate through optimized variables and copy them to parameter_block_data
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


void hwa_fgo::GNSSInfo::marginalize()
{  
	int pos = 0;    
	
	for (auto &it : parameter_block_idx)
	{
		it.second = pos;
		pos += parameter_block_size[it.first];
	}
	m = pos;    // Total number of variables to be discarded


	for (const auto &it : parameter_block_size)
	{
		if (parameter_block_idx.find(it.first) == parameter_block_idx.end())
		{
			parameter_block_idx[it.first] = pos;
			pos += it.second;
		}
	}
	n = pos - m;    // Total number of variables retained
	if (m == 0)
	{
		valid = false;
		printf("unstable tracking...\n");
		return;
	}
	if (n == 0)
	{
		valid = false;
		return;
	}
	Matrix A(pos, pos);
	Vector b(pos);
	A.setZero();
	b.setZero();	

	for (auto it : factors)
	{
		for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
		{
			int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
			int size_i = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
			Matrix jacobian_i = it->jacobians[i].leftCols(size_i);
			for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
			{
				int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
				int size_j = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
				Matrix jacobian_j = it->jacobians[j].leftCols(size_j);
				if (i == j)
					A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
				else
				{
					A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
					A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
				}
			}
			b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
		}
	}


	// Shu'er Bu marginalizes, gaining new A and b.
	Matrix Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
	Eigen::SelfAdjointEigenSolver<Matrix> saes(Amm);
	//ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());
	Matrix Amm_inv = saes.eigenvectors() * Vector((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
	//printf("error1: %f\n", (Amm * Amm_inv - Matrix::Identity(m, m)).sum());
	Vector bmm = b.segment(0, m);
	Matrix Amr = A.block(0, m, m, n);
	Matrix Arm = A.block(m, 0, n, m);
	Matrix Arr = A.block(m, m, n, n);
	Vector brr = b.segment(m, n);
	A = Arr - Arm * Amm_inv * Amr;
	b = brr - Arm * Amm_inv * bmm;

	// Recover the Jacobian and residual from the new A and b for subsequent inclusion of epoch marginalization factors.
	Eigen::SelfAdjointEigenSolver<Matrix> saes2(A);
	Vector S = Vector((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
	Vector S_inv = Vector((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));
	Vector S_sqrt = S.cwiseSqrt();
	Vector S_inv_sqrt = S_inv.cwiseSqrt();
	linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
	linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;

	//printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
	//     (linearized_jacobians.transpose() * linearized_residuals - b).sum());
	valid = true;

}




std::vector<double*> hwa_fgo::GNSSInfo::getParameterBlocks(std::unordered_map<long, double*>& addr_shift)
{

	// Some data is retained in the Marg variable after the iteration. This data is used solely for adding prior residuals during the next optimization step and is not utilized for the subsequent Marg calculation.
	std::vector<double *> keep_block_addr;
	keep_block_size.clear();
	keep_block_idx.clear();
	keep_block_data.clear();

	for (const auto &it : parameter_block_idx)
	{
		if (it.second >= m)
		{
				/* auto it_find = addr_shift.find(it.first);
			if (it_find != addr_shift.end())
			{
				
			}	*/		
			keep_block_size.push_back(parameter_block_size[it.first]);
			
			keep_block_idx.push_back(parameter_block_idx[it.first]);
		
			keep_block_data.push_back(parameter_block_data[it.first]);
			keep_block_addr.push_back(addr_shift[it.first]);
			
		}
	}
	// The total size of all parameters in the variable block, the sum of all fuzziness values, i.e., the number of fuzziness values.
	sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);
	return keep_block_addr;
}

hwa_fgo::MarginalizationGNSSFactor::MarginalizationGNSSFactor(GNSSInfo * _gnss_info): gnss_info(_gnss_info)
{
	//int cnt = 0;
	for (auto it : gnss_info->keep_block_size)
	{
		mutable_parameter_block_sizes()->push_back(it);
		//cnt += it;
	}
	//printf("residual size: %d, %d\n", cnt, n);
	set_num_residuals(gnss_info->n);
}

bool hwa_fgo::MarginalizationGNSSFactor::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
	//printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
			//for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
			//{
			//    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
			//    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
			//printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
			//printf("residual %x\n", reinterpret_cast<long>(residuals));
			//}
	int n = gnss_info->n;
	int m = gnss_info->m;
	Vector dx(n);
	for (int i = 0; i < static_cast<int>(gnss_info->keep_block_size.size()); i++)
	{
		int size = gnss_info->keep_block_size[i];
		int idx = gnss_info->keep_block_idx[i] - m;
		Vector x = Eigen::Map<const Vector>(parameters[i], size);
		Vector x0 = Eigen::Map<const Vector>(gnss_info->keep_block_data[i], size);
		dx.segment(idx, size) = x - x0;		
	}
	Eigen::Map<Vector>(residuals, n) = gnss_info->linearized_residuals + gnss_info->linearized_jacobians * dx;
	if (jacobians)
	{
		for (int i = 0; i < static_cast<int>(gnss_info->keep_block_size.size()); i++)
		{
			if (jacobians[i])
			{
				int size = gnss_info->keep_block_size[i];
				int idx = gnss_info->keep_block_idx[i] - m;
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
				jacobian.setZero();
				jacobian.leftCols(size) = gnss_info->linearized_jacobians.middleCols(idx, size);
			}
		}
	}
	return true;
}
