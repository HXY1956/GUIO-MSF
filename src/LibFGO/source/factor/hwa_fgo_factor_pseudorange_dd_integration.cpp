#include "hwa_fgo_factor_pseudorange_dd_integration.h"
#include "hwa_base_timecost.h"

hwa_fgo::PseudorangeDDINGFactor::PseudorangeDDINGFactor(const base_time & cur_time, const std::pair<std::string, std::string>& base_rover_site, const base_allpar & params, const std::vector<std::pair<gnss_data_sats, gnss_data_sats>>& DD_sat_data, gnss_model_bias * bias_model, const std::pair<FREQ_SEQ, GOBSBAND>& freq_band, const Eigen::Vector3d &lever_arm):
	_cur_time(cur_time), _base_rover_site(base_rover_site), _params(params), _DD_sat_data(DD_sat_data), _gprecise_bias_model(bias_model), _freq_band(freq_band),  _lever_arm(lever_arm)
{
}

void hwa_fgo::PseudorangeDDINGFactor::updatePara(base_allpar & params_tmp, const Eigen::Vector3d & Pi, const Eigen::Vector3d & Vi) const
{
	params_tmp = _params;
	int i = 0;
	i = params_tmp.getParam(_base_rover_site.second, par_type::CRD_X, "");
	if (i >= 0)
	{
		params_tmp[i].value(Pi.x());
	}
	i = params_tmp.getParam(_base_rover_site.second, par_type::CRD_Y, "");
	if (i >= 0)
	{
		params_tmp[i].value(Pi.y());
	}

	i = params_tmp.getParam(_base_rover_site.second, par_type::CRD_Z, "");
	if (i >= 0)
	{
		params_tmp[i].value(Pi.z());
	}
}
void hwa_fgo::PseudorangeDDINGFactor::trans2Eigen(const std::vector<std::vector<std::pair<int, double>>>& B, const std::vector<double>& P, const std::vector<double>& l, Eigen::Matrix<double, 2, 3>& B_new, Eigen::Matrix<double, 2, 2>& P_new, Eigen::Matrix<double, 2, 1>& l_new) const
{
	B_new.setZero();
	P_new.setZero();
	l_new.setZero();
	for (int i = 0; i < B.size(); i++)
	{
		for (int j = 0; j < B[i].size(); j++)
		{
			B_new(i, j) = B[i][j].second;
		}
	}
	for (int i = 0; i < 2; i++)
	{
		P_new(i, i) = P[i];
	}
	for (int i = 0; i < 2; i++)
	{
		l_new(i) = l[i];
	}
}
bool hwa_fgo::PseudorangeDDINGFactor::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{
	/*if (_cur_time.sow() == 200340 && _DD_sat_data.at(0).first.sat() == "G10" && _DD_sat_data.at(1).first.sat() == "G25")
	{
		cout << endl;
	}*/

	Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
	Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
	//Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2])
	Eigen::Matrix3d Reb = Qi.toRotationMatrix();
	Eigen::Vector3d P_INS = Pi + Reb * _lever_arm;
	double sqrt_info;
	Eigen::Matrix<double, 1, 2> DD_operator(1, 2);
	//construct DD equ	
	unsigned npar_orig = _params.parNumber() - 5;
	base_allpar params_temp;
	updatePara(params_temp, P_INS);
	std::vector<std::vector<std::pair<int, double>>> B;        ///< coeff of equations
	std::vector<double> P;                           ///< weight of equations
	std::vector<double> l;                           ///< res of equations
	Matrix B_DD;
	double l_DD, P_DD;
	for (auto it : _DD_sat_data)
	{
		gnss_model_base_equation tempP;
		std::pair<gnss_data_sats, gnss_data_sats> rec_pair = it;
		for (int isite = 0; isite < 2; isite++)
		{
			gnss_data_sats *satdata_ptr;
			if (isite == 0) satdata_ptr = &rec_pair.first;
			else satdata_ptr = &rec_pair.second;
			gnss_data_obs  obsP = gnss_data_obs(satdata_ptr->select_range(_freq_band.second));
			base_time crt = satdata_ptr->epoch();
			if (!_gprecise_bias_model->cmb_equ(crt, params_temp, *satdata_ptr, obsP, tempP))
			{
				std::cout << "sat " << rec_pair.second.sat() << "  construct pseudorange DD factor error" << std::endl;
				return false;

			}
		}
		std::vector<std::pair<int, double>> B_P;
		double P_P, l_P;
		int ibase = 0;
		int irover = 1;
		for (const auto& b : tempP.B[irover]) {
			if (b.first >= npar_orig) continue;
			B_P.push_back(b);
		}
		for (const auto& b : tempP.B[ibase]) {
			if (b.first >= npar_orig) continue;
			B_P.emplace_back(b.first, -b.second);
		}
		P_P = 1 / (1 / tempP.P[irover] + 1 / tempP.P[ibase]); l_P = tempP.l[irover] - tempP.l[ibase];

		B.push_back(B_P);
		P.push_back(P_P);
		l.push_back(l_P);

#ifndef DEBUG_FACTOR
		//cout << "sat: " << it.first.sat() << " " << "band: " << _freq_band.first << " " << endl;
		//cout << "code: " << "weight: " << P_P << " " << "residual: " << l_P << " " << "Jacbian: ";
		/*for (int i = 0; i < B_P.size(); i++)
		{
			cout << B_P[i].second << " ";
		}
		cout << endl;*/
#endif // !1
	}
	int iobs = 1;
	int index_ref = 0;
	int index_sat = 1;
	DD_operator(iobs - 1, index_ref) = -1;
	DD_operator(iobs - 1, index_sat) = 1;
	Eigen::Matrix<double, 2, 3> B_new;
	Eigen::Matrix<double, 2, 2> P_new;
	Eigen::Matrix<double, 2, 1> l_new;
	trans2Eigen(B, P, l, B_new, P_new, l_new);
	B_DD = DD_operator * B_new;
	l_DD = DD_operator * l_new;
	P_DD = DD_operator * P_new.inverse()*DD_operator.transpose();
	P_DD = 1.0 / P_DD;
	//auto crt = _DD_sat_data.at(0).first.epoch();
	//double time = crt.sow() + crt.dsec();

	//std::string ref_sat = _DD_sat_data.at(0).first.sat();
	//std::string nonref_sat = _DD_sat_data.at(1).first.sat();

	//std::cout << "time: " << time << std::endl;
	//std::cout << "ref_sat: " << ref_sat << std::endl;
	//std::cout << "nonref_sat: " << nonref_sat << std::endl;
	//std::cout << "residual: " << l_DD << std::endl;
	//std::cout << "Jacbian: " << B_DD << std::endl;
	//std::cout << "weight: " << P_DD << std::endl;
	//std::cout << std::endl;
	//set ceres value
	sqrt_info = sqrt(P_DD);
	residuals[0] = sqrt_info * l_DD;

	if (jacobians)
	{

		if (jacobians[0])
		{
			Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
			Eigen::Matrix<double, 1, 6> jaco_pose;
			jaco_pose.leftCols<3>() = B_DD.leftCols<3>();
			jaco_pose.rightCols<3>() = B_DD.leftCols<3>() * -Reb * base_att_trans::skewSymmetric(_lever_arm);
			jacobian_pose.leftCols<6>() = -sqrt_info * jaco_pose;
			jacobian_pose.rightCols<1>().setZero();

			/*if (_cur_time.sow() == 200340 && _DD_sat_data.at(0).first.sat() == "G10" && _DD_sat_data.at(1).first.sat() == "G25")
				cout << jacobian_pose << endl;*/

			//std::cout << "range Jacbian: " << jacobian_pose.transpose() << endl;
		}
		/*if (jacobians[1])
		{   TODO£ºfor velocity parameter block;


		}*/

	}
	return true;
}

void hwa_fgo::PseudorangeDDINGFactor::check(double ** parameters)
{
	double *res = new double[1];
	double **jaco = new double *[2];
	jaco[0] = new double[1 * 7];
	Evaluate(parameters, res, jaco);
	/*puts("check begins");

	puts("my");*/

	/*std::cout << Eigen::Map<Eigen::Matrix<double, 1, 1>>(res).transpose() << std::endl
		<< std::endl;
	std::cout << Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
		<< std::endl;*/
		
}
