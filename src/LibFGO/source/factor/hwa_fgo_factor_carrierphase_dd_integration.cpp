#include "hwa_fgo_factor_carrierphase_dd_integration.h"
#include "hwa_base_timecost.h"

hwa_fgo::CarrierphaseDDINGFactor::CarrierphaseDDINGFactor(const base_time & cur_time, const std::pair<std::string, std::string>& base_rover_site, const base_allpar & params, const std::vector<std::pair<gnss_data_sats, gnss_data_sats>>& DD_sat_data, gnss_model_bias * bias_model, const std::pair<FREQ_SEQ, GOBSBAND>& freq_band, const Eigen::Vector3d &lever_arm):
	_cur_time(cur_time), _base_rover_site(base_rover_site), _params(params), _DD_sat_data(DD_sat_data), _gprecise_bias_model(bias_model), _freq_band(freq_band),_lever_arm(lever_arm)
{

}

void hwa_fgo::CarrierphaseDDINGFactor::updatePara(base_allpar & params_tmp, const double & ref_sd_amb, const double & nonref_sd_amb, const Eigen::Vector3d & Pi, const Eigen::Vector3d & Vi) const
{
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
	std::map<FREQ_SEQ, par_type> ambtype_list = {
				{FREQ_1, par_type::AMB_L1},
				{FREQ_2, par_type::AMB_L2},
				{FREQ_3, par_type::AMB_L3},
				{FREQ_4, par_type::AMB_L4},
				{FREQ_5, par_type::AMB_L5} };
	//for ref sat
	std::string ref_sat_name = _DD_sat_data[0].first.sat();
	std::string site = _DD_sat_data[0].second.site(); //for rover

	i = params_tmp.getParam(site, ambtype_list[_freq_band.first], ref_sat_name);
	if (i >= 0)
	{
		params_tmp[i].value(ref_sd_amb);
	}
	//for nonref sat
	std::string nonref_sat_name = _DD_sat_data[1].first.sat();
	i = params_tmp.getParam(site, ambtype_list[_freq_band.first], nonref_sat_name);
	if (i >= 0)
	{
		params_tmp[i].value(nonref_sd_amb);
	}

}

void hwa_fgo::CarrierphaseDDINGFactor::trans2Eigen(const std::vector<std::vector<std::pair<int, double>>>& B, const std::vector<double>& P, const std::vector<double>& l, Eigen::Matrix<double, 2, 5>& B_new, Eigen::Matrix<double, 2, 2>& P_new, Eigen::Matrix<double, 2, 1>& l_new) const
{
	B_new.setZero();
	P_new.setZero();
	l_new.setZero();
	for (int i = 0; i < B.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			B_new(i, j) = B[i][j].second;
		}

		if (i == 0)  B_new(i, 3) = B[i][3].second;
		if (i == 1)  B_new(i, 4) = B[i][3].second;
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

bool hwa_fgo::CarrierphaseDDINGFactor::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const
{

	Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
	Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
	Eigen::Matrix3d Reb = Qi.toRotationMatrix();
	Eigen::Vector3d P_INS = Pi + Reb * _lever_arm;

	//Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]); 
	double ref_SD_ambiguity = parameters[1][0];
	double nonref_SD_ambiguity = parameters[2][0];
	double sqrt_info;
	Eigen::Matrix<double, 1, 2> DD_operator(1, 2);
	//construct DD equ	
	unsigned npar_orig = _params.parNumber() - 5;
	base_allpar params_temp = _params;
	Triple xyz;


	updatePara(params_temp, ref_SD_ambiguity, nonref_SD_ambiguity, P_INS);

	//params_temp.getCrdParam(_base_rover_site.second, xyz);
	//Eigen::Vector3d xyz_before = Eigen::Vector3d(xyz.crd(0), xyz.crd(1), xyz.crd(2));
	//cout << " phase YXZ update: " << xyz_before.transpose() << endl;
	std::vector<std::vector<std::pair<int, double>>> B;        ///< coeff of equations
	std::vector<double> P;                           ///< weight of equations
	std::vector<double> l;                           ///< res of equations
	Matrix B_DD;
	double l_DD, P_DD;
	std::map<FREQ_SEQ, par_type> ambtype_list = {
				{FREQ_1, par_type::AMB_L1},
				{FREQ_2, par_type::AMB_L2},
				{FREQ_3, par_type::AMB_L3},
				{FREQ_4, par_type::AMB_L4},
				{FREQ_5, par_type::AMB_L5} };

	for (auto it : _DD_sat_data)
	{
		gnss_model_base_equation tempL;
		std::pair<gnss_data_sats, gnss_data_sats> rec_pair = it;
		for (int isite = 0; isite < 2; isite++)
		{
			gnss_data_sats *satdata_ptr;
			if (isite == 0) satdata_ptr = &rec_pair.first;
			else satdata_ptr = &rec_pair.second;
			gnss_data_obs  obsL = gnss_data_obs(satdata_ptr->select_phase(_freq_band.second));
			base_time crt = satdata_ptr->epoch();
			if (!_gprecise_bias_model->cmb_equ(crt, params_temp, *satdata_ptr, obsL, tempL))
			{
				std::cout << "sat " << rec_pair.second.sat() << "  construct carrierphase DD factor error" << std::endl;
				return false;
			}
			if (satdata_ptr->site() == _base_rover_site.second)
			{
				int idx = params_temp.getParam(satdata_ptr->site(), ambtype_list[_freq_band.first], satdata_ptr->sat());

				if (idx < 0)
				{
					std::cout << "sat " << rec_pair.second.sat() << "  construct carrierphase DD factor error" << std::endl;
					return false;
				}

				tempL.B.back().push_back(std::make_pair(idx, 1.0));
				tempL.l.back() -= params_temp[idx].value();
			}
		}
		std::vector<std::pair<int, double>> B_L;
		double P_L, l_L;
		int ibase = 0;
		int irover = 1;
		for (const auto& b : tempL.B[irover]) {
			if (b.first >= npar_orig) continue;
			B_L.push_back(b);
		}
		for (const auto& b : tempL.B[ibase]) {
			if (b.first >= npar_orig) continue;
			B_L.emplace_back(b.first, -b.second);
		}
		P_L = 1 / (1 / tempL.P[irover] + 1 / tempL.P[ibase]); l_L = tempL.l[irover] - tempL.l[ibase];

		B.push_back(B_L);
		P.push_back(P_L);
		l.push_back(l_L);


		//cout << "sat: " << it.first.sat() << " " << "band: " << _freq_band.first << " " << endl;
		//cout << "phase: " << "weight: " << P_L << " " << "residual: " << l_L << " " << "Jacbian: ";

		//for (int i = 0; i < B_L.size(); i++)
		//{
		//	cout << B_L[i].second << " ";
		//}
		//cout << endl;


	}
	int iobs = 1;
	int index_ref = 0;
	int index_sat = 1;
	DD_operator(iobs - 1, index_ref) = -1;
	DD_operator(iobs - 1, index_sat) = 1;
	Eigen::Matrix<double, 2, 5> B_new;
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

			//jacobian_XYZ = B_DD.leftCols<3>();
			//std::cout << "carrier Jacbian: " << jacobian_pose.transpose() << endl;
		}
		/*if (jacobians[1])
		{   TODO£ºfor velocity parameter block;


		}*/
		if (jacobians[1])
		{
			Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_ambiguity1(jacobians[1]);
			jacobian_ambiguity1 = -sqrt_info * B_DD.middleCols<1>(3);
			//jacobian_ambiguity1 = B_DD.middleCols<1>(3);
			//cout << jacobian_ambiguity1.transpose() << " ";
		}

		if (jacobians[2])
		{
			Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_ambiguity2(jacobians[2]);
			jacobian_ambiguity2 = -sqrt_info * B_DD.rightCols<1>();
			//jacobian_ambiguity2 = B_DD.rightCols<1>();
			//cout << jacobian_ambiguity2.transpose() << endl;
		}
	}
	return true;
}

void hwa_fgo::CarrierphaseDDINGFactor::check(double ** parameters)
{
	double *res = new double[1];
	double **jaco = new double *[3];
	jaco[0] = new double[1 * 7];
	jaco[1] = new double[1 * 1];
	jaco[2] = new double[1 * 1];
	Evaluate(parameters, res, jaco);
	/*puts("CarrierphaseDDFactor check begins");
	puts("my: ");
	std::cout << Eigen::Map<Eigen::Matrix<double, 1, 1>>(res).transpose() << std::endl
		<< std::endl;
	std::cout << Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
		<< std::endl;
	std::cout << Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>>(jaco[1]) << std::endl
		<< std::endl;
	std::cout << Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>>(jaco[2]) << std::endl
		<< std::endl;*/		
}
