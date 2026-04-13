#include <Eigen/Dense>
#include <Eigen/SVD>
#include <stdlib.h>
#include <sstream>
#include <stack>
#include <cstdio>
#include <algorithm>
#include <iomanip>

#include "hwa_base_log.h"
#include "hwa_base_file.h"
#include "hwa_base_eigendef.h"
#include "hwa_gnss_coder_recover.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_gnss_proc_lsq.h"
#ifdef USE_CLAPACK
#include "gproc/ginverse_lapack.h"
#endif
#ifdef USE_OPENBLAS
#include "gproc/ginverse_openblas.h"
#endif
#define SIZE_INT sizeof(int)
#define SIZE_DBL sizeof(double)


using namespace std; // NOLINT
using namespace hwa_base;

namespace hwa_gnss
{ // NOLINT
    gnss_proc_lsqbase::gnss_proc_lsqbase() : _mode(LSQ_MODE::EPO),
                       _gset(nullptr),
                       _spdlog(nullptr),
                       _tempfile(nullptr),
                       _obs_total_num(0),
                       _npar_tot_num(0),
                       _sigma0(0.0),
                       _res_obs(0.0)
    { // NOLINT
        // NOLINT
    }

    // Constructors
    gnss_proc_lsqbase::gnss_proc_lsqbase(set_base *set) : _mode(LSQ_MODE::EPO),
                                      _spdlog(nullptr),
                                      _gset(set),
                                      _tempfile(nullptr),
                                      _obs_total_num(0),
                                      _npar_tot_num(0), // ztd clk change by BoWong  // NOLINT
                                      _sigma0(0.0),
                                      _res_obs(0.0)
    { // NOLINT
        if (nullptr == set)
        {
            spdlog::critical("your set pointer is nullptr !");
        }
        else
        {
            _gset = set;
        }

        _dx.resize(0);
        stringstream this_addr;
        this_addr << this;
        reset_tempfile("tempfile_" + this_addr.str());
    }
    gnss_proc_lsqbase::gnss_proc_lsqbase(base_log spdlog, set_base *set) : _mode(LSQ_MODE::EPO),
                                                       _tempfile(nullptr),
                                                       _obs_total_num(0),
                                                       _npar_tot_num(0), // ztd clk change by BoWong
                                                       _sigma0(0.0),
                                                       _res_obs(0.0),
                                                       _gset(set)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
        if (nullptr == set)
        {
            spdlog::critical("your set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = set;
        }
        //_NEQ.resize(0);

        _dx.resize(0);

        if (set != nullptr)
        {
            _beg = dynamic_cast<set_gen *>(set)->beg();
            _end = dynamic_cast<set_gen *>(set)->end();
            _interval = dynamic_cast<set_gen *>(set)->sampling(); //glfeng 2019.5.1
            _mode = dynamic_cast<set_gproc *>(set)->lsq_mode();
            _buffer_size = dynamic_cast<set_gproc *>(set)->lsq_buffer_size() * 1024 * 1000;
        }

        // get random_tempfile
        stringstream this_addr;
        this_addr << this;
        reset_tempfile("tempfile_" + this_addr.str());
    }

    gnss_proc_lsqbase::gnss_proc_lsqbase(const gnss_proc_lsqbase &Other)
        : _epo(Other._epo),
          _beg(Other._beg),
          _end(Other._end),
          _tempfile(nullptr),
          _spdlog(Other._spdlog),
          _NEQ(Other._NEQ),
          _dx(Other.dx()),
          _dx_final(Other.dx()), //glfeng
          _W(Other._W),
          _Qx(Other._Qx),
          _mode(Other._mode),
          _update_lsqpar(Other._update_lsqpar),
          _x_solve(Other._x_solve),
          _stdx(Other._stdx),
          _res_obs(Other._res_obs),
          _npar_tot_num(Other._npar_tot_num), // ztd clk change by BoWong
          _interval(Other._interval),         //glfeng
          _vtpv(Other._vtpv), _sigma0(Other._sigma0), _obs_total_num(Other._obs_total_num),
          _obs_total_num_epo(Other._obs_total_num_epo),
          _buffer_size(Other._buffer_size)
    {
        //TODO:TOO BAD solve way NOW..
        stringstream this_addr;
        this_addr << this;
        reset_tempfile("tempfile_" + this_addr.str());
    }

    gnss_proc_lsqbase::~gnss_proc_lsqbase()
    {
        if (_tempfile != nullptr)
        {
            if (_tempfile->is_open())
            {
                _tempfile->close();
            }

            std::remove(_tempfile->name().c_str());

            delete _tempfile;
            _tempfile = nullptr;
        }
    }

    void gnss_proc_lsqbase::spdlog(base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        if (nullptr != spdlog)
        {
            _spdlog = spdlog;
        }
    }

    //HwangShih added
    bool gnss_proc_lsqbase::lsq_minimum_constraint(string minimum_constraint, set<string> core_station, double sig)
    {
        set<string>::iterator ret;
        vector<double> x0, y0, z0;
        vector<int> icrdx, icrdy, icrdz;
        vector<string> siteall;
        base_allpar pars = _x_solve;
        int parNum = pars.parNumber();
        gnss_data_obscombtype type;
        double P = 1 / sig;

        for (int ipar = 0; ipar < parNum; ipar++)
        {
            ret = find(core_station.begin(), core_station.end(), pars.getPar(ipar).site);
            if (ret != core_station.end())
            {
                string site = pars.getPar(ipar).site;
                siteall.push_back(site);
                icrdx.push_back(pars.getParam(site, par_type::CRD_X, ""));
                icrdy.push_back(pars.getParam(site, par_type::CRD_Y, ""));
                icrdz.push_back(pars.getParam(site, par_type::CRD_Z, ""));
                x0.push_back(pars.getParValue(*(icrdx.rbegin())));
                y0.push_back(pars.getParValue(*(icrdy.rbegin())));
                z0.push_back(pars.getParValue(*(icrdz.rbegin())));

                core_station.erase(ret);
            }
        }

        if (siteall.size() < 3)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "The number of stations is not enough to add NNT or NNR constraint!");
            return false;
        }

        Matrix A(3 * siteall.size(), 3);
        Matrix B_temp(3, 3 * siteall.size());

        //NNR
        if (minimum_constraint == "NNR")
        {
            for (unsigned int i = 0; i < siteall.size(); i++)
            {
                A.row(i * 3) << 0 , z0[i] , -y0[i];
                A.row(i * 3 + 1) << -z0[i] , 0, x0[i];
                A.row(i * 3 + 2) << y0[i] , -x0[i], 0;
            }
        }

        //NNT
        if (minimum_constraint == "NNT")
        {
            for (unsigned int i = 0; i < siteall.size(); i++)
            {
                A.row(i * 3) << 1 ,0 , 0;
                A.row(i * 3 + 1) << 0 , 1 , 0;
                A.row(i * 3 + 2) << 0 , 0 ,1;
            }
        }

        B_temp = ((A.transpose() * A).inverse()) * (A.transpose());
        vector<pair<int, double>> B;
        string sitename = "coresite";
        for (int i = 0; i < B_temp.rows(); i++)
        {
            for (unsigned int j = 0; j < siteall.size(); j++)
            {
                B.push_back(std::make_pair(icrdx[j] + 1, B_temp(i, 3 * j)));
                B.push_back(std::make_pair(icrdy[j] + 1, B_temp(i, 3 * j + 1)));
                B.push_back(std::make_pair(icrdz[j] + 1, B_temp(i, 3 * j + 2)));
            }

            gnss_proc_lsq_equationmatrix virtual_equ;
            virtual_equ.add_equ(B, P, 0, sitename, "", type, false);
            this->add_equation(virtual_equ, _beg, false);

            B.clear(); //clear B matrix
        }

        return true;
    }

    void gnss_proc_lsqbase::clear_tempfile()
    {
        if (_tempfile != nullptr)
        {
            //cerr << _tempfile->name();
            //remove(_tempfile->name().c_str());
            //
            //string name = _tempfile->name().c_str();
            
            _tempfile = nullptr;
            delete _tempfile;
            //remove(name.c_str());
        }
    }

    int gnss_proc_lsqbase::add_parameter(const base_par &par)
    {
        // add the par
        _x_solve.addParam(par);
        _NEQ.addBackZero();
        _W.addBackZero();
        _npar_tot_num++;
        return 1;
    }

    void gnss_proc_lsqbase::add_par_state_equ(par_type par_type, int order, double dt, double noise)
    {
        _update_lsqpar->set_par_state_mode(par_type, order, dt, noise);
    }

    void gnss_proc_lsqbase::set_update_par(shared_ptr<gnss_proc_update_par> updatepar)
    {
        _update_lsqpar = updatepar;
    }

    bool gnss_proc_lsqbase::update_parameter(const base_time &epoch, vector<gnss_data_sats> &obsdata, bool matrix_remove, bool write_temp)
    {
        if (!_update_lsqpar)
        {
            throw std::runtime_error("_update_lsqpar is empty!!!");
        }

        gnss_proc_update_par_info remove_info = _update_lsqpar->get_all_update_parameters(epoch, _x_solve, obsdata);

        vector<int> remove_id;
        vector<base_par> new_par_list;
        vector<base_par> equ_par_list;
        gnss_proc_lsq_equationmatrix virtual_equ;
        remove_info.get(remove_id, new_par_list, equ_par_list, virtual_equ);

        // First. Add virtual equ
        // jdhuang : chang auto to const auto&
        for (const auto &par : equ_par_list)
        {
            add_parameter(par);
        }
        add_equation(virtual_equ, epoch, false);

        // add ISB constraint if ISB par in remove par list
        //for (auto item : remove_id)
        //{
        //    if (_x_solve[item - 1].str_type().find("ISB") != -1)
        //    {
        //        lsq_sysbias_constraint();
        //        break;
        //    }
        //}
        // Second. Remove old pars
        remove_info.get(remove_id);
        if (matrix_remove)
        {
            // use matrix remove way
            remove_parameter(remove_id, write_temp);
        }
        else
        {
            // use one by one remove way
            //rearrange before remove
            sort(remove_id.begin(), remove_id.end());

            //write_change
            rearrange_lsqmatrix(remove_id, _NEQ, _W, _x_solve);
            _write_parchage(remove_id);

            int max_i = _NEQ.num();
            int beg_i = _NEQ.num() - remove_id.size();
            for (int i = max_i; i > beg_i; i--)
            {
                remove_parameter(i, write_temp);
            }
        }

        // Third. Add new pars
        remove_info.get(new_par_list);
        for (const auto &par : new_par_list)
        {
            add_parameter(par);
        }

        return true;
    }

    bool gnss_proc_lsqbase::update_parameter(bool phase,
                                  const base_time &epoch,
                                  vector<gnss_data_sats> &obsdata,
                                  bool matrix_remove,
                                  bool write_temp)
    {
        if (!phase)
        {
            return this->update_parameter(epoch, obsdata, matrix_remove, write_temp);
        }

        if (!_update_lsqpar)
        {
            throw std::runtime_error("_update_lsqpar is empty!!!");
        }

        gnss_proc_update_par_info remove_info = _update_lsqpar->get_all_update_parameters(epoch, _x_solve, obsdata);

        vector<int> remove_id;
        vector<base_par> new_par_list;
        vector<base_par> equ_par_list;
        gnss_proc_lsq_equationmatrix virtual_equ;
        remove_info.get(remove_id, new_par_list, equ_par_list, virtual_equ);

        // First. Add virtual equ
        for (const auto &par : equ_par_list)
        {
            add_parameter(par);
        }
        add_equation(phase, virtual_equ);

        // Second. Remove old pars
        remove_info.get(remove_id);
        if (matrix_remove)
        {
            // use matrix remove way
            remove_parameter(remove_id, write_temp);
        }
        else
        {
            // use one by one remove way
            sort(remove_id.begin(), remove_id.end());
            for (unsigned int i = 0; i < remove_id.size(); i++)
            {
                remove_parameter(remove_id[i] - i, write_temp);
            }
        }

        // Third. Add new pars
        remove_info.get(new_par_list);
        for (const auto &par : new_par_list)
        {
            add_parameter(par);
        }

        return true;
    }

    int gnss_proc_lsqbase::set_parameter(const base_allpar &parameters)
    {
        _x_solve = parameters;

        int numpar = _x_solve.parNumber();

        _W.resize(numpar);
        _NEQ.resize(numpar);

        return 1;
    }
    int gnss_proc_lsqbase::add_equation(const Matrix &B, const Diag &P, const Vector &l, const base_time &epoch)
    {
        int numpar = _x_solve.parNumber();

        if (B.cols() != numpar ||
            B.rows() != P.rows() ||
            B.rows() != l.rows() ||
            P.rows() != l.rows())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "input matrix is wrong row/col!");
            throw exception();
        }

        _epo = epoch;

        gnss_proc_lsq_equationmatrix equ;
        vector<string> empty_site(B.rows(), "");
        vector<string> empty_sat(B.rows(), "");
        vector<gnss_data_obscombtype> empty_obstype(B.rows(), gnss_data_obscombtype());
        equ.add_equ(B, P, l, empty_site, empty_sat, empty_obstype);

        _NEQ.add(equ);
        _W.add(equ);

        _obs_total_num += equ.num_equ();
        _res_obs += equ.res_equ();

        return 1;
    }
    int gnss_proc_lsqbase::add_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch, bool write_temp)
    {
        if (write_temp)
        {
            this->write_equation(equ, epoch);
        }

        _epo = epoch;

        _NEQ.add(equ);
        _W.add(equ);
        //jqlou debug
        //print_matrx();
        // jdhuang debug
        //_W.add_related(equ);

        _obs_total_num += equ.num_equ();
        _res_obs += equ.res_equ();

        return 1;
    }

    int gnss_proc_lsqbase::add_equation(const bool phase, const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch, bool write_temp)
    {

        if (!phase)
        {
            return this->add_equation(equ, epoch, write_temp);
        }
        this->write_equation(equ, epoch, phase);

        _epo = epoch;

        _NEQ.add(equ, phase);
        _W.add(equ, phase);

        _obs_total_num += equ.num_equ() / 2;
        _res_obs += equ.res_equ(phase);

        return 1;
    }

    int gnss_proc_lsqbase::get_equ_obs_total_num()
    {
        if (_obs_total_num_epo == 0)
        {
            _obs_total_num_epo = _obs_total_num;
            return _obs_total_num_epo;
        }
        else
        {
            int result = _obs_total_num - _obs_total_num_epo;
            _obs_total_num_epo = _obs_total_num;
            return result;
        }
        return -1;
    }

    int gnss_proc_lsqbase::del_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch)
    {
        _epo = epoch;

        _NEQ.del(equ);
        _W.del(equ);

        _obs_total_num -= equ.num_equ();
        _res_obs -= equ.res_equ();

        return 1;
    }

    bool gnss_proc_lsqbase::write_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch)
    {
        if (_tempfile == nullptr)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "the tempfile don't exist!");
            throw exception();
        }

        int len_size = 0;

        // write coefficinet Matrix
        for (int Row = 1; Row <= equ.num_equ(); Row++)
        {

            // length of record
            len_size = 0;

            // write identifier
            _tempfile->write("obs", 3);
            len_size += 3;

            // write parameter number
            int par_num = equ.B[Row - 1].size();
            _tempfile->write((char *)&par_num, SIZE_INT);
            len_size += SIZE_INT;

            // write time
            _tempfile->write((char *)&epoch, sizeof(base_time));
            len_size += sizeof(base_time);

            // write ambflag
            int is_newamb = equ.is_newamb(Row - 1) ? 1 : 0;
            _tempfile->write((char *)&is_newamb, SIZE_INT);
            len_size += SIZE_INT;

            // write info
            string out_info;
            out_info = equ.get_sitename(Row - 1) + "    " + equ.get_satname(Row - 1) + "     " + equ.get_obscombtype2str(Row - 1) + "     ";
            int len_out_info = out_info.size();
            _tempfile->write((char *)&len_out_info, SIZE_INT);
            len_size += SIZE_INT;
            _tempfile->write(out_info.c_str(), len_out_info);
            len_size += len_out_info;

            // write coefficient
            for (int Col = 1; Col <= par_num; Col++)
            {
                double coeff = equ.B[Row - 1][Col - 1].second;
                int loc = equ.B[Row - 1][Col - 1].first;
                //if (!double_eq(coeff, 0.0))
                ////if ( coeff == 0.0)

                _tempfile->write((char *)&loc, SIZE_INT);
                len_size += SIZE_INT;
                _tempfile->write((char *)&coeff, SIZE_DBL);
                len_size += SIZE_DBL;
            }

            // Write Max loc
            int Max_loc = _x_solve.parNumber();
            _tempfile->write((char *)&Max_loc, SIZE_INT);
            len_size += SIZE_INT;

            // write P
            double p = equ.P[Row - 1];
            _tempfile->write((char *)&p, SIZE_DBL);
            len_size += SIZE_DBL;

            // write res
            double res = equ.l[Row - 1];
            _tempfile->write((char *)&res, SIZE_DBL);
            len_size += SIZE_DBL;

            _tempfile->write((char *)&len_size, SIZE_INT);
        }

        return true;
    }

    bool gnss_proc_lsqbase::write_equation(const gnss_proc_lsq_equationmatrix &equ, const base_time &epoch, bool phase)
    {
        if (!phase)
        {
            return this->write_equation(equ, epoch);
        }

        if (_tempfile == nullptr)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "the tempfile don't exist!");
            throw exception();
        }

        int len_size = 0;

        // write coefficinet Matrix
        for (int Row = 1; Row <= equ.num_equ(); Row++)
        {
            if (equ.get_obscombtype(Row - 1).is_code())
            {
                continue;
            }
            // length of record
            len_size = 0;

            // write identifier
            _tempfile->write("obs", 3);
            len_size += 3;

            // write parameter number
            int par_num = equ.B[Row - 1].size();
            _tempfile->write((char *)&par_num, SIZE_INT);
            len_size += SIZE_INT;

            // write time
            _tempfile->write((char *)&epoch, sizeof(base_time));
            len_size += sizeof(base_time);

            // write ambflag
            int is_newamb = equ.is_newamb(Row - 1) ? 1 : 0;
            _tempfile->write((char *)&is_newamb, SIZE_INT);
            len_size += SIZE_INT;

            // write info
            string out_info;
            out_info = equ.get_sitename(Row - 1) + "    " + equ.get_satname(Row - 1) + "     " + equ.get_obscombtype2str(Row - 1) + "     ";
            int len_out_info = out_info.size();
            _tempfile->write((char *)&len_out_info, SIZE_INT);
            len_size += SIZE_INT;
            _tempfile->write(out_info.c_str(), len_out_info);
            len_size += len_out_info;

            // write coefficient
            for (int Col = 1; Col <= par_num; Col++)
            {
                double coeff = equ.B[Row - 1][Col - 1].second;
                int loc = equ.B[Row - 1][Col - 1].first;
                //if (!double_eq(coeff, 0.0))
                ////if ( coeff == 0.0)

                _tempfile->write((char *)&loc, SIZE_INT);
                len_size += SIZE_INT;
                _tempfile->write((char *)&coeff, SIZE_DBL);
                len_size += SIZE_DBL;
            }

            // Write Max loc
            int Max_loc = _x_solve.parNumber();
            _tempfile->write((char *)&Max_loc, SIZE_INT);
            len_size += SIZE_INT;

            // write P
            double p = equ.P[Row - 1];
            _tempfile->write((char *)&p, SIZE_DBL);
            len_size += SIZE_DBL;

            // write res
            double res = equ.l[Row - 1];
            _tempfile->write((char *)&res, SIZE_DBL);
            len_size += SIZE_DBL;

            _tempfile->write((char *)&len_size, SIZE_INT);
        }

        return true;
    }

    int gnss_proc_lsqbase::solve_NEQ()
    {
        if (_NEQ.num() != _W.num())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "input Matrix NEQ or W is Wrong!");
            return 0;
        }

        // add apriori
        add_apriori_weight();

        Symmetric NEQ_Matrix = _NEQ.changeNewMat();
        Vector W_Matrix = _W.changeNewMat();

        if (double_eq(NEQ_Matrix.matrixR().maxCoeff(), 0.0))
        {
            _dx.resize(_x_solve.parNumber());
            _dx.setZero();
        }
        else
        {
            vector<int> zero_idx;

            for (int Row = 0; Row < NEQ_Matrix.rows(); Row++)
            {
                if (double_eq(NEQ_Matrix(Row, Row), 0.0))
                {
                    zero_idx.push_back(Row);
                }
            }

            if (zero_idx.size() == 0)
            {
                _solve_equation(NEQ_Matrix, W_Matrix, _dx, _stdx);
            }
            else
            {

                V_Symmetric remove_NEQ = _NEQ;
                V_Vector remove_W = _W;
                Vector temp_dx, temp_dq;
                for (auto iter = zero_idx.rbegin(); iter != zero_idx.rend(); iter++)
                {
                    remove_NEQ.remove(*iter);
                    remove_W.remove(*iter);
                }

                Symmetric temp_NEQ = remove_NEQ.changeNewMat();
                Vector temp_W = remove_W.changeNewMat();
                _solve_equation(temp_NEQ, temp_W, temp_dx, temp_dq);

                _dx.resize(NEQ_Matrix.rows());
                _dx.setZero();
                _stdx.resize(NEQ_Matrix.rows());
                _stdx.setZero();
                Symmetric temp_Qx = _Qx;
                _Qx.resize(NEQ_Matrix.rows());
                _Qx.setZero();

                int idx = 0;
                int idy = 0;
                for (int Row = 0; Row < NEQ_Matrix.rows(); Row++)
                {
                    // not zero
                    if (find(zero_idx.begin(), zero_idx.end(), Row) == zero_idx.end())
                    {
                        idy = 0;
                        for (int Col = 0; Col < NEQ_Matrix.cols(); Col++)
                        {
                            // not zero
                            if (find(zero_idx.begin(), zero_idx.end(), Col) == zero_idx.end())
                            {
                                _Qx.set(temp_Qx(idx, idy), Row, Col);
                                // std::cout << setw(12) << setprecision(8) << temp_Qx(idx, idy);
                                idy++;
                            }
                        }
                        _dx(Row) = temp_dx(idx);
                        _stdx(Row) = temp_dq(idx);
                        idx++;
                        // std::cout << endl;
                    }
                }

                if (idx != temp_dx.rows())
                    throw exception();
            }

            // slove sigama0
            _vtpv = _res_obs;
            for (int i = 0; i <= _dx.rows(); i++)
            {
                _vtpv -= W_Matrix(i) * (_dx(i));
            }
            //_sigma0 = sqrt(abs(_vtpv) / (_obs_total_num - _x_solve.parNumber()));

#ifdef DEBUG_pppRTK
            std::cout << "vtpv = " << abs(_vtpv) << " ntot = " << _obs_total_num << " npar = " << _npar_tot_num << endl;
#endif                                                                     // DEBUG_pppRTK
            _sigma0 = sqrt(abs(_vtpv) / (_obs_total_num - _npar_tot_num)); // glfeng same with panda
            if (_obs_total_num - _npar_tot_num < 0)
                _sigma0 = -1.0;

            for (int i = 0; i < _stdx.rows(); i++)
            {
                _stdx(i) = sqrt(_stdx(i)) * _sigma0;
            }
        }

        _dx_final << _dx;
        return 1;
    }

    void gnss_proc_lsqbase::solve_x()
    {
        if (_NEQ.num() != _W.num())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "input Matrix NEQ or W is Wrong!");
            throw runtime_error("gnss_proc_lsqbase::solve_NEQ:input Matrix NEQ or W is Wrong!");
        }

        // add apriori
        add_apriori_weight();

        Symmetric NEQ_Matrix = _NEQ.changeNewMat();
        Vector W_Matrix = _W.changeNewMat();

        if (double_eq(NEQ_Matrix.matrixR().maxCoeff(), 0.0))
        {
            _dx.resize(_x_solve.parNumber());
            _dx.setZero();
        }
        else
        {
            vector<int> zero_idx;

            for (int Row = 0; Row < NEQ_Matrix.rows(); Row++)
            {
                if (double_eq(NEQ_Matrix(Row, Row), 0.0))
                {
                    zero_idx.push_back(Row);
                }
            }

            if (zero_idx.size() == 0)
            {
                _solve_x(NEQ_Matrix, W_Matrix, _dx);
            }
            else
            {

                V_Symmetric remove_NEQ = _NEQ;
                V_Vector remove_W = _W;
                Vector temp_dx, temp_dq;
                for (auto iter = zero_idx.rbegin(); iter != zero_idx.rend(); iter++)
                {
                    //_remove_zero_element(temp_NEQ, temp_W, *iter);
                    remove_NEQ.remove(*iter);
                    remove_W.remove(*iter);
                }

                Symmetric temp_NEQ = remove_NEQ.changeNewMat();
                Vector temp_W = remove_W.changeNewMat();
                _solve_x(temp_NEQ, temp_W, temp_dx);

                _dx.resize(NEQ_Matrix.rows());
                _dx.setZero();

                //std::cout << zero_idx[0] << endl;
                int idx = 0;
                // int idy = 1;
                for (int Row = 0; Row < NEQ_Matrix.rows(); Row++)
                {
                    // not zero
                    if (find(zero_idx.begin(), zero_idx.end(), Row) == zero_idx.end())
                    {
                        _dx(Row) = temp_dx(idx);
                        idx++;
                    }
                }

                if (idx != temp_dx.rows())
                    throw exception();
            }
        }

        _dx_final << _dx;
        return;
    }

    // remove parameter ! idx from 1
    int gnss_proc_lsqbase::remove_parameter(const int &idx, bool write_temp)
    {
        add_apriori_weight(idx);
        double center_value = _NEQ.center_value(idx);
        double w_remove = _W.num(idx);
        if (center_value != 0.0)
        {
            // compute the ltpl
            _res_obs -= w_remove * w_remove / center_value;
        }
        // write the coeff to tempfile
        if (write_temp)
            _write_coefficient(idx);
        // remove NEQ W Matrix
        if (!remove_lsqmatrix(idx, _NEQ, _W))
        {
            throw "remove lsq matrix error!";
        }

        // remove the par
        _x_solve.delParam(idx - 1);
        //changed by LX, _npar_tot_num need to plus 1, otherwise the lsq->sigma may be wrong.
        _npar_tot_num--;

        return 1;
    }

    int gnss_proc_lsqbase::remove_parameter(vector<int> &idx, bool write_temp)
    {
        if (idx.empty())
        {
            return 1;
        }

        chrono::high_resolution_clock::time_point beg_t;
        chrono::high_resolution_clock::time_point end_t;

        beg_t = chrono::high_resolution_clock::now();

        //rearrange before remove
        sort(idx.begin(), idx.end());
        int zero_size = rearrange_lsqmatrix(idx, _NEQ, _W, _x_solve);
        _write_parchage(idx);
        end_t = chrono::high_resolution_clock::now();
        std::cout << "Rearrange Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

        beg_t = chrono::high_resolution_clock::now();
        //process zero
        int rows_new = _NEQ.num() - idx.size();
        int rows_remove = idx.size() - zero_size;
        for (int zero_idx = _NEQ.num(); zero_idx > rows_new + rows_remove; zero_idx--)
        {
            remove_parameter(zero_idx, write_temp);
        }
        end_t = chrono::high_resolution_clock::now();
        std::cout << "Process zero Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

        int rows = _NEQ.num();
        // add apriori
        for (int i = rows; i > rows_new; i--)
        {
            add_apriori_weight(i);
        }

        beg_t = chrono::high_resolution_clock::now();
        vector<double> temp_NEQ(rows * rows, 0.0);

        for (int i = 0; i < rows; i++)
        {
            memcpy(&temp_NEQ[i * rows], &_NEQ._element[i][0], sizeof(double) * (i + 1));
        }

#ifndef USE_OPENBLAS
        vector<double> temp_Eigen_NEQ(rows * rows, 0.0);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                temp_Eigen_NEQ[i * rows + j] = _NEQ._element[i][j];
                temp_Eigen_NEQ[j * rows + i] = _NEQ._element[i][j];
            }
        }

        Vector W1(Eigen::Map<Vector>(&_W._element[0], rows_new));
        Vector W2(Eigen::Map<Vector>(&_W._element[rows_new], rows_remove));

        Matrix A(Eigen::Map<Matrix, 0, Eigen::OuterStride<>>(&temp_Eigen_NEQ[0], rows_new, rows_new, Eigen::OuterStride<>(rows)));
        Matrix B(Eigen::Map<Matrix, 0, Eigen::OuterStride<>>(&temp_Eigen_NEQ[rows_new], rows_remove, rows_new, Eigen::OuterStride<>(rows)));
        Matrix C(Eigen::Map<Matrix, 0, Eigen::OuterStride<>>(&temp_Eigen_NEQ[rows_new * rows + rows_new], rows_remove, rows_remove, Eigen::OuterStride<>(rows)));
#endif

        end_t = chrono::high_resolution_clock::now();
        std::cout << "Init Matrix time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

        Matrix CL, CLi, Ci, Bt, CiB, CiW2, NEQ_new, W_new;
        vector<double> CiB_vec(rows_remove * rows_new, 0.0);
        vector<double> CiW2_vec(rows_remove, 0.0);
        if (rows_remove != 0)
        {
            /***************SVD Debug remove matrix*************************************
            for (int i =rows_new+1;i<=rows_new+rows_remove;i++){
                printf("par_type is %s apriori is %lf NEQ Value is %e\n",_x_solve[i-1].str_type().c_str(),_x_solve[i-1].apriori(),_NEQ.center_value(i));
            }
            Eigen::JacobiSVD<Matrix> svd(C, Eigen::ComputeThinU | Eigen::ComputeThinV);
            auto singlev = svd.singularValues();
            for (int i =0;i<singlev.size();i++){
                printf(" %d SVD value is: %e\n",i+1,singlev[i]);
            }
            printf("Max SVD %e  Min SVD %e  Compare %e\n",singlev[0],singlev[singlev.size()-1],singlev[singlev.size()-1]/singlev[0]);
            std::cout << "Frist Max SVD " << singlev[0] << endl;
            std::cout << "Second Max SVD " << singlev[1] << endl;
            std::cout << "Second Min SVD " << singlev[singlev.size()-2] << endl;
            std::cout << "Second Max SVD " << singlev[singlev.size()-1] << endl;
            **************************************************************/
#ifdef USE_OPENBLAS
            beg_t = chrono::high_resolution_clock::now();
            //BASE_INVERSE_OPENBLAS<Matrix> openblas_tool(true);
            //openblas_tool.inverse_NEQ(rows_remove,C_temp);
            char U = 'U';
            int error_info = 0;
            // Cholosky
            LAPACK_dpotrf(&U, &rows_remove, &temp_NEQ[rows_new * rows + rows_new], &rows, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Remove par Lapack_dpotrf(cholosky) Solve Wrong!");
            }
            // Inverse NEQ
            LAPACK_dpotri(&U, &rows_remove, &temp_NEQ[rows_new * rows + rows_new], &rows, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Remove par Lapack_dpotri() Solve Wrong!");
            }
            // assign another triangle
            for (int i = 0; i < rows_remove; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    temp_NEQ[rows_new * rows + rows_new + j * rows + i] = temp_NEQ[rows_new * rows + rows_new + i * rows + j];
                }
            }
            end_t = chrono::high_resolution_clock::now();
            std::cout << "Openblas Cholosky Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

#ifdef DEBUG_COMPARE
            //  compute difference
            printf("Compare Ci matrix\n");
            CL = Eigen::LLT<Matrix>(C).matrixL();
            CLi.noalias() = CL.inverse();
            Ci.noalias() = CLi.transpose() * CLi;
            for (int i = 0; i < rows_remove; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double openblas_v = temp_NEQ[rows_new * rows + rows_new + i * rows + j];
                    double eigen_v = Ci(i, j);
                    if (abs(openblas_v - eigen_v) > 1E-6)
                    {
                        printf("Row %d Col %d Eigen: %lf OpenBlas %lf diff %lf\n", i, j, eigen_v, openblas_v, eigen_v - openblas_v);
                    }
                }
            }
#endif

            beg_t = chrono::high_resolution_clock::now();

#ifdef DEBUG_COMPARE
            Bt.noalias() = B.transpose();
            CiB.noalias() = Ci * B;
            CiW2.noalias() = Ci * W2;
            NEQ_new.noalias() = A - Bt * CiB;
            W_new.noalias() = W1 - Bt * CiW2;
            printf("Compare B matrix\n");
            for (int i = 0; i < rows_remove; i++)
            {
                for (int j = 0; j < rows_new; j++)
                {
                    double openblas_v = temp_NEQ[rows_new * rows + i * rows + j];
                    double eigen_v = B(i, j);
                    if (abs(openblas_v - eigen_v) > 1E-9)
                    {
                        printf("Row %d Col %d Eigen: %lf OpenBlas %lf diff %lf \n", i, j, eigen_v, openblas_v, eigen_v - openblas_v);
                    }
                }
            }

#endif
            // CiB =  Ci * B
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, rows_remove, rows_new, rows_remove, 1, &temp_NEQ[rows_new * rows + rows_new], rows, &temp_NEQ[rows_new * rows], rows, 0, &CiB_vec[0], rows_new);

#ifdef DEBUG_COMPARE
            printf("Compare CiB\n");
            for (int i = 0; i < rows_remove; i++)
            {
                for (int j = 0; j < rows_new; j++)
                {
                    double openblas_v = CiB_vec[i * rows_new + j];
                    double eigen_v = CiB(i, j);
                    if (abs(openblas_v - eigen_v) > 1E-6)
                    {
                        printf("Row %d Col %d Eigen: %lf OpenBlas %lf diff %lf\n", i, j, eigen_v, openblas_v, eigen_v - openblas_v);
                    }
                }
            }
#endif
            // CiW2 = Ci * W2
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, rows_remove, 1, rows_remove, 1, &temp_NEQ[rows_new * rows + rows_new], rows, &_W._element[rows_new], 1, 1, &CiW2_vec[0], 1);
#ifdef DEBUG_COMPARE
            printf("Compare CiW2\n");
            for (int i = 0; i < rows_remove; i++)
            {
                double openblas_v = CiW2_vec[i];
                double eigen_v = CiW2(i);
                if (abs(openblas_v - eigen_v) > 1E-6)
                {
                    printf("Row %d Col %d Eigen: %lf OpenBlas %lf diff %lf\n", i, 0, eigen_v, openblas_v, eigen_v - openblas_v);
                }
            }
#endif
            // A = -1*Bt*CiB + A
            cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, rows_new, rows_new, rows_remove, -1, &temp_NEQ[rows_new * rows], rows, &CiB_vec[0], rows_new, 1, &temp_NEQ[0], rows);
#ifdef DEBUG_COMPARE
            printf("Compare NEQ_new\n");
            for (int i = 0; i < rows_new; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double openblas_v = temp_NEQ[i * rows + j];
                    double eigen_v = NEQ_new(i, j);
                    if (abs(openblas_v - eigen_v) > 1E-6)
                    {
                        printf("Row %d Col %d Eigen: %lf OpenBlas %lf diff %lf\n", i, j, eigen_v, openblas_v, eigen_v - openblas_v);
                    }
                }
            }
#endif
            // W1 = -1*Bt*CiB + W1
            cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, rows_new, 1, rows_remove, -1, &temp_NEQ[rows_new * rows], rows, &CiW2_vec[0], 1, 1, &_W._element[0], 1);
#ifdef DEBUG_COMPARE
            printf("Compare W_new\n");
            for (int i = 0; i < rows_new; i++)
            {
                double openblas_v = _W._element[i];
                double eigen_v = W_new(i);
                if (abs(openblas_v - eigen_v) > 1E-6)
                {
                    printf("Row %d Col %d Eigen: %lf OpenBlas %lf diff %lf\n", i, 0, eigen_v, openblas_v, eigen_v - openblas_v);
                }
            }
#endif
            for (int i = 0; i < rows_new; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    _NEQ._element[i][j] = temp_NEQ[i * rows + j];
                }
            }
            //openblas_tool.inverse_NEQ(rows_remove,C);
            end_t = chrono::high_resolution_clock::now();
            std::cout << "Openblas Multiply Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;
#else
            beg_t = chrono::high_resolution_clock::now();
            //Colosky
            CL = Eigen::LLT<Matrix>(C).matrixL();
            CLi.noalias() = CL.inverse();
            Ci.noalias() = CLi.transpose() * CLi;

            end_t = chrono::high_resolution_clock::now();
            std::cout << "Eigen Cholosky Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

            beg_t = chrono::high_resolution_clock::now();
            // transpose
            Bt.noalias() = B.transpose();

            // multiply
            CiB.noalias() = Ci * B;
            CiW2.noalias() = Ci * W2;

            // multiply decmial
            NEQ_new.noalias() = A - Bt * CiB;
            W_new.noalias() = W1 - Bt * CiW2;

            //get new res_obs
            _res_obs = _res_obs - (W2.transpose() * CiW2)(0, 0);

            // ouptut matrix
            for (int i = 0; i < rows_new; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    _NEQ._element[i][j] = NEQ_new(i, j);
                }
                _W._element[i] = W_new(i);
            }
            end_t = chrono::high_resolution_clock::now();
            std::cout << "Eigen Multiply and Resize Matrix Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;
#endif
        }

        // Resize
        for (int i = rows - 1; i >= rows_new; i--)
        {
            _NEQ.remove(i + 1);
            _W.remove(i + 1);
        }

        beg_t = chrono::high_resolution_clock::now();
        // _write_coefficient
        if (_tempfile != nullptr)
        {
            for (int i = rows_remove - 1; i >= 0; i--)
            {
                int len_size = 0;
                // write identifier
                _tempfile->write("prt", 3);
                len_size += 3;

                // write parameter number
                vector<pair<int, double>> par_record;
                double w_record;

#ifdef USE_OPENBLAS
                w_record = CiW2_vec[i];
                for (int j = 0; j < rows_new; j++)
                {
                    if (CiB_vec[i * rows_new + j] != 0.0)
                    {
                        par_record.push_back(std::make_pair(j + 1, CiB_vec[i * rows_new + j]));
                    }
                }
#else
                w_record = CiW2(i);
                for (int j = 0; j < rows_new; j++)
                {
                    if (CiB(i, j) != 0.0)
                    {
                        par_record.push_back(std::make_pair(j + 1, CiB(i, j)));
                    }
                }
#endif

                // write par num
                int par_num = par_record.size();
                _tempfile->write((char *)&par_num, SIZE_INT);
                len_size += SIZE_INT;

                // write parameter index
                int par_idx = rows_new + i + 1;
                _tempfile->write((char *)&(par_idx), SIZE_INT);
                len_size += SIZE_INT;
                par_idx--;

                // write par name station satlltie
                string name = _x_solve[par_idx].str_type();
                name = _x_solve[par_idx].site + "_" + name;
                if (_x_solve[par_idx].prn == "")
                    name = name + "_";
                _tempfile->write(name.c_str(), 20);
                len_size += 20;

                // write par time
                // change by glfeng write start and end time
                _tempfile->write((char *)&_x_solve[par_idx].beg, sizeof(base_time));
                len_size += sizeof(base_time);
                _tempfile->write((char *)&_x_solve[par_idx].end, sizeof(base_time));
                len_size += sizeof(base_time);

                // write parameter value
                double value = _x_solve[par_idx].value();
                _tempfile->write((char *)&value, SIZE_DBL);
                len_size += SIZE_DBL;

                // write zhd for ztd retrieval glfeng
                double zhd = _x_solve[par_idx].zhd;
                _tempfile->write((char *)&zhd, SIZE_DBL);
                len_size += SIZE_DBL;

                // write coefficient BTPB
                for (int ipar = 1; ipar <= par_num; ipar++)
                {
                    _tempfile->write((char *)&(par_record[ipar - 1].first), SIZE_INT);
                    len_size += SIZE_INT;
                    _tempfile->write((char *)&(par_record[ipar - 1].second), SIZE_DBL);
                    len_size += SIZE_DBL;
                }

                // write BTPL
                _tempfile->write((char *)&w_record, SIZE_DBL);
                len_size += SIZE_DBL;

                _tempfile->write((char *)&len_size, SIZE_INT);

                // remove par
                _x_solve.delParam(par_idx);
                _npar_tot_num--;
            }
        }
        else
        {
            for (int i = rows_remove - 1; i >= 0; i--)
            {
                // remove par
                _x_solve.delParam(rows_new + i);
                _npar_tot_num--;
            }
        }

        end_t = chrono::high_resolution_clock::now();
        std::cout << "Write tempfile is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

        return 1;
    }
    bool gnss_proc_lsqbase::recover_parameters(gnss_all_recover &allrecover)
    {
        // make sure the latest obs information output to the file

        if (!_tempfile)
        {
            throw_logical_error(_spdlog, "have no tempfile Can't recover!");
        }

        //_log->comment(t_glog::LOG_LV::LOG_INFO, "gnss_proc_lsqbase", "reocver_parameters", "recovering...");

        // reopen tempfile
        _tempfile->flush();
        _tempfile->close();

        //_tempfile->open(_tempfile->mask(), ios::ate | ios::in | ios::binary);
        base_io_READTEMP tempfile_in(_tempfile->mask());

        int len_size = size_int;
        tempfile_in.seekg_from_cur(-len_size);
        tempfile_in.read((char *)&len_size, size_int);

        // loop for tempfile
        while (len_size != -1)
        {
            len_size += size_int;
            //_tempfile->seekg(-len_size, ios::cur);
            tempfile_in.seekg_from_cur(-len_size);

            char identifiy[3];
            //_tempfile->read(identifiy, 3);
            tempfile_in.read(identifiy, 3);

            string temp(identifiy, 3);

            if (temp == "obs")
            {
                _recover_obs(tempfile_in, allrecover);
            }
            else if (temp == "par")
            {
                _recover_par(tempfile_in, allrecover);
            }
            else if (temp == "prt")
            {
                _recover_par_part(tempfile_in, allrecover);
            }
            else if (temp == "swp")
            {
                _recover_par_swap(tempfile_in);
            }
            else
            {
                break;
            }

            // back
            len_size -= size_int;
            tempfile_in.seekg_from_cur(-len_size);

            // read length
            len_size = size_int;
            tempfile_in.seekg_from_cur(-len_size);
            tempfile_in.read((char *)&len_size, size_int);
        }

        allrecover.set_interval(_interval);
        allrecover.set_sigma0(_sigma0);
        return true;
    }
    void gnss_proc_lsqbase::get_result_parameter(gnss_all_recover &allrecover)
    {

        string par_name;
        for (unsigned int i = 0; i < _x_solve.parNumber(); i++)
        {
            base_par par = _x_solve[i];
            gnss_data_recover_par recover_par(_spdlog, _x_solve[i], _dx(i + 1));
            allrecover.add_recover_par(recover_par);
        }
    }
    void gnss_proc_lsqbase::add_apriori_weight()
    {
        // add apriori weight
        if (_x_solve.parNumber() != _NEQ.num())
        {
            throw exception();
        }

        if (_x_solve.parNumber() == 0)
        {
            return;
        }

        //auto row_iter = _NEQ._element.begin();
        int neq_num = _NEQ.num();
        for (int Row = 1; Row <= neq_num; Row++)
        {
            double apriori = _x_solve[Row - 1].apriori();

            // jdhuang
            if (_NEQ.num(Row, Row) == 0.0)
                continue;
            if (apriori == 0.0)
                continue;

            _NEQ.num(Row, Row) = _NEQ.num(Row, Row) + 1 / (apriori * apriori);
        }
    }
    void gnss_proc_lsqbase::add_apriori_weight(const int &idx)
    {
        // add apriori weight
        double apriori = _x_solve[idx - 1].apriori();
        if (!double_eq(apriori, 0.0))
        {
            _NEQ.num(idx, idx) = _NEQ.num(idx, idx) + 1 / (apriori * apriori);
        }
        return;
    }
    void gnss_proc_lsqbase::add_apriori_weight(const int &idx, const double &value, const double &weight)
    {
        // add apriori weight

        // add in NEQ
        double P = weight;
        _NEQ.num(idx, idx) = _NEQ.num(idx, idx) + P;

        // add in W
        double l = value - _x_solve[idx - 1].value();
        _W.num(idx) = _W.num(idx) + P * l;

        return;
    }
    int gnss_proc_lsqbase::lsq_sysbias_constraint()
    {
        // impose constraint on IFB/ISB parameters, the sum of all IFB / ISB to a satellite is zero
        // TODO(jdhuang) ifb constraint....
        // only for ISB
        int idx0 = -1;
        int idx1 = -1;
        int nx;
        map<int, bool> isused;
        map<int, int> iptx;
        for (unsigned int ipar = 0; ipar < _x_solve.parNumber(); ipar++)
        {
            isused[ipar] = false;
            if (_x_solve[ipar].str_type() == "GAL_ISB" ||
                _x_solve[ipar].str_type() == "GLO_ISB" ||
                _x_solve[ipar].str_type() == "BDS_ISB" ||
                _x_solve[ipar].str_type() == "QZS_ISB")
            {
                if (idx0 == -1)
                    idx0 = ipar;
                idx1 = ipar;
            }
        }
        if (idx0 == -1 && idx1 == -1)
            return 0;
        for (int ipar = idx0; ipar <= idx1 - 1; ipar++)
        {
            if (_x_solve[ipar].str_type() == "GAL_ISB" ||
                _x_solve[ipar].str_type() == "GLO_ISB" ||
                _x_solve[ipar].str_type() == "BDS_ISB" ||
                _x_solve[ipar].str_type() == "QZS_ISB")
            {
                if (isused[ipar])
                    continue;
                nx = 0;
                iptx[nx] = ipar;
                isused[ipar] = true;
                for (int jpar = ipar + 1; jpar <= idx1; jpar++)
                {
                    if (isused[jpar])
                        continue;
                    if (_x_solve[ipar].parType != _x_solve[jpar].parType)
                        continue;
                    nx++;
                    iptx[nx] = jpar;
                    isused[jpar] = true;
                }

                for (int i = 0; i <= nx; i++)
                {
                    int ip1 = iptx[i];
                    for (int j = 0; j <= i; j++)
                    {
                        int jp1 = iptx[j];
                        if (_NEQ.num(ip1 + 1, jp1 + 1) != 0.0)
                        {
                            _NEQ.num(ip1 + 1, jp1 + 1) = _NEQ.num(ip1 + 1, jp1 + 1) + 1e7;
                        }
                    }
                }
            }
        }
        // jdhuang
        return 1;
    }

    int gnss_proc_lsqbase::lsq_clk_constraint()
    {
        vector<pair<int, double>> B;
        for (size_t ipar = 0; ipar < _x_solve.parNumber(); ++ipar)
        {
            if (_x_solve[ipar].str_type().substr(0, 7) != "CLK_SAT")
                continue;
            if (double_eq(_x_solve.getAmbParam(ipar).value(), 0.0))
                continue;
            B.push_back(std::make_pair(ipar + 1, 1));
        }
        gnss_proc_lsq_equationmatrix virtual_equ;
        gnss_data_obscombtype type;
        virtual_equ.add_equ(B, 1e7, 0.0, "", "", type, false);
        this->add_equation(virtual_equ, _epo, false);
        return 1;
    }

    //HwangShih added for geocenter estimating
    bool gnss_proc_lsqbase::lsq_helmert_constraint(HMTCONSTR hmtcst)
    {
        vector<string> core_station = {
            "AB09", "AB21", "ABMF", "ABPO", "ACOR", "ADIS", "AGGO", "AIRA", "AIS5", "AJAC",
            "ALAC", "ALBH", "ALBY", "ALGO", "ALIC", "ALME", "ALON", "AMC4", "AMU2", "ANA1",
            "ANKR", "ANMG", "AQUI", "AREG", "AREQ", "ARHT", "ARTU", "ARUC", "ASCG", "ASPA",
            "ATW2", "AUCK", "AUKT", "AUT1", "AV09", "AZGB", "AZRY", "BADG", "BAKE", "BAKO",
            "BAKU", "BAMF", "BAR1", "BARH", "BAYR", "BBRY", "BELL", "BHR3", "BHR4", "BIK0",
            "BILB", "BISK", "BJFS", "BJNM", "BLUF", "BNDY", "BOAV", "BOGI", "BOGO", "BOGT",
            "BOR1", "BRAZ", "BREW", "BRFT", "BRMU", "BRO1", "BRST", "BRUN", "BRUX", "BSHM",
            "BUCU", "BUDP", "BUEN", "BUR2", "BZRG", "CABL", "CAGS", "CANT", "CAS1", "CASC",
            "CAT2", "CATA", "CCJ2", "CEBR", "CEDU", "CEFE", "CHAC", "CHAN", "CHPG", "CHPI",
            "CHTI", "CHUM", "CHUR", "CKIS", "CKSV", "CLGO", "CMUM", "CNCL", "CNMR", "COEC",
            "CORD", "CORX", "COSO", "COT1", "CPVG", "CPXF", "CRAR", "CREU", "CRO1", "CRU1",
            "CUCU", "CUIB", "CUSV", "CZTG", "DAEJ", "DAKR", "DARW", "DAV1", "DEAR", "DENT",
            "DGAR", "DHLG", "DJIG", "DLF1", "DOUR", "DRAG", "DRAO", "DUBO", "DUND", "DVAO",
            "EBRE", "EIL3", "EIL4", "ELAT", "ESCO", "ESMR", "EUR2", "EUSK", "EYAC", "FAA1",
            "FAIR", "FALK", "FARB", "FFMJ", "FLF1", "FLIN", "FLM5", "FLRS", "FRDN", "FTNA",
            "FUNC", "GAMB", "GANP", "GENO", "GISB", "GLPS", "GLSV", "GMSD", "GODE", "GODN",
            "GODS", "GODZ", "GOL2", "GOLD", "GOPE", "GRAS", "GRAZ", "GUAM", "GUAT", "GUAX",
            "GUUG", "HAL1", "HAMM", "HARB", "HARV", "HELG", "HERS", "HERT", "HIL1", "HKSL",
            "HKWS", "HLFX", "HNPT", "HNUS", "HOB2", "HOBU", "HOFN", "HOKI", "HOLB", "HRAG",
            "HRAO", "HUEG", "HYDE", "IBEC", "IDDR", "IENG", "IISC", "IMPZ", "INEG", "INVK",
            "IPAZ", "IQAL", "IQQE", "IRKJ", "IRKM", "ISHI", "ISTA", "IZMI", "JCTW", "JFNG",
            "JOEN", "JOZ2", "JOZE", "JPLM", "JSLM", "KABR", "KARA", "KARL", "KARR", "KAT1",
            "KATO", "KATZ", "KERG", "KGNI", "KIR0", "KIR8", "KIRI", "KIRU", "KIT3", "KITG",
            "KLOP", "KMNM", "KMOR", "KOKB", "KOKV", "KOST", "KOUC", "KOUG", "KOUR", "KRGG",
            "KRS1", "KRTV", "KSNB", "LAGO", "LAMA", "LAMP", "LAUT", "LCK3", "LCK4", "LEIJ",
            "LEPA", "LHAZ", "LHCL", "LINZ", "LKHU", "LLIV", "LMMF", "LORD", "LPAL", "LPGS",
            "LPIL", "LROC", "LYNS", "MAC1", "MAD2", "MADR", "MAG0", "MAJU", "MAL2", "MALL",
            "MANA", "MAO0", "MAR6", "MAS1", "MAT1", "MATE", "MATG", "MAUI", "MAW1", "MAYG",
            "MBAR", "MCHL", "MCM4", "MDVJ", "MEDI", "MELI", "MERI", "MERS", "METG", "METS",
            "MFKG", "MGUE", "MIG1", "MIKL", "MIZU", "MKEA", "MOBJ", "MOBK", "MOBN", "MOBS",
            "MONP", "MOPI", "MORP", "MPL2", "MQZG", "MRL1", "MRL2", "MRO1", "MTJO", "MTKA",
            "MTV1", "MTV2", "MTY2", "NANO", "NAS0", "NAUS", "NEAH", "NEIA", "NETP", "NEWL",
            "NICO", "NIST", "NIUM", "NKLG", "NLIB", "NMEA", "NNOR", "NNVN", "NORF", "NOVM",
            "NPLY", "NPRI", "NRC1", "NRIF", "NRIL", "NRMD", "NSSS", "NTUS", "NVSK", "NYA1",
            "NYA2", "NYAL", "OAK1", "OAK2", "OAX2", "OHI2", "OHI3", "ONS1", "ONSA", "OPMT",
            "ORID", "OSN3", "OSN4", "OUS2", "OWMG", "PADO", "PALM", "PARC", "PARK", "PARL",
            "PBRI", "PDEL", "PENC", "PERT", "PETS", "PFA2", "PFRR", "PGC5", "PICL", "PIE1",
            "PIMO", "PLTK", "PNGM", "POAL", "POHN", "POL2", "POLV", "POTR", "POTS", "POVE",
            "PPPC", "PPTE", "PRDS", "PRE3", "PRE4", "PTAG", "PTBB", "PTGG", "PTVL", "PUMO",
            "QAQ1", "QUAR", "QUEM", "QUI3", "QUI4", "QUIN", "RAEG", "RAMO", "RBAY", "RDSD",
            "REDU", "REUN", "REYK", "RGDG", "RIGA", "RIO2", "RIOB", "ROB4", "ROTH", "RWSN",
            "SAGA", "SALU", "SAMA", "SAMO", "SANT", "SASK", "SAVO", "SBOK", "SCH2", "SCOA",
            "SCOR", "SCRZ", "SCTB", "SELD", "SEY2", "SEYG", "SFER", "SGOC", "SGPO", "SIDN",
            "SIO5", "SJDV", "SKE0", "SMST", "SNLR", "SOD3", "SOFI", "SOLO", "SPT0", "SQUO",
            "SRJV", "SRMP", "SRS1", "SSIA", "STAS", "STFU", "STHL", "STJ3", "STJO", "STK2",
            "STR1", "STR2", "STVI", "SULP", "SUMK", "SUTB", "SUTH", "SUTM", "SUWN", "SVTL",
            "SYDN", "SYOG", "TABL", "TABV", "TALA", "TASH", "TCMS", "TDOU", "TELA", "TERS",
            "TERU", "TETN", "THIO", "THTG", "THTI", "THU2", "TID1", "TIDB", "TIXG", "TIXI",
            "TJRN", "TLSE", "TMGO", "TN22", "TOPL", "TORI", "TORP", "TOW2", "TRDS", "TRNT",
            "TRO1", "TSEA", "TSK2", "TSKB", "TUBI", "TUKT", "TWTF", "TXES", "TXLU", "UCAL",
            "UCLU", "UFPR", "ULAB", "ULDI", "UNB3", "UNBJ", "UNIV", "UNPG", "UNPM", "UNSA",
            "UNTR", "URAL", "URUM", "USMX", "USN7", "USUD", "UTQI", "VAAS", "VACO", "VACS",
            "VALD", "VALE", "VARS", "VBCA", "VIL0", "VILL", "VIS0", "VNDP", "VOIM", "VTIS",
            "VTSP", "WAB2", "WARK", "WARN", "WDC5", "WDC6", "WES2", "WEST", "WGTN", "WGTT",
            "WHIT", "WIDC", "WILL", "WIND", "WROC", "WSRT", "WTZA", "WTZL", "WTZR", "WTZZ",
            "WUH2", "XMIS", "YAKT", "YAR2", "YAR3", "YARR", "YEBE", "YEL2", "YELL", "YESX",
            "YRCM", "YSSK", "ZAMB", "ZECK", "ZIM2", "ZIM3", "ZIMM", "ZOUF"};
        vector<string>::iterator ret;
        vector<double> x0, y0, z0;
        vector<int> icrdx, icrdy, icrdz;
        vector<string> siteall;
        base_allpar pars = _x_solve;
        int parNum = pars.parNumber();
        gnss_data_obscombtype type;

        for (int ipar = 0; ipar < parNum; ipar++)
        {
            ret = find(core_station.begin(), core_station.end(), pars.getPar(ipar).site);
            if (ret != core_station.end())
            {
                string site = pars.getPar(ipar).site;
                siteall.push_back(site);
                icrdx.push_back(pars.getParam(site, par_type::CRD_X, ""));
                icrdy.push_back(pars.getParam(site, par_type::CRD_Y, ""));
                icrdz.push_back(pars.getParam(site, par_type::CRD_Z, ""));
                x0.push_back(pars.getParValue(*(icrdx.rbegin())));
                y0.push_back(pars.getParValue(*(icrdy.rbegin())));
                z0.push_back(pars.getParValue(*(icrdz.rbegin())));

                core_station.erase(ret);
            }
        }

        if (siteall.size() < 3)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "The number of stations is not enough to add NNT or NNR constraint!");
            return false;
        }

        Matrix A(3 * siteall.size(), 3);
        Matrix B_temp(3, 3 * siteall.size());

        //NNT+NNR
        if (hmtcst == HMTCONSTR::NNTNNR)
        {
            A.resize(3 * siteall.size(), 6);
            B_temp.resize(6, 3 * siteall.size());
            for (unsigned int i = 0; i < siteall.size(); i++)
            {
                A.row(i * 3) << 1 , 0 , 0 , 0 , z0[i] , -y0[i];
                A.row(i * 3 + 1) << 0 , 1 , 0 , -z0[i] , 0 , x0[i];
                A.row(i * 3 + 2) << 0 , 0 , 1 , y0[i] , -x0[i] , 0;
            }
        }

        //NNR
        if (hmtcst == HMTCONSTR::NNR)
        {
            for (unsigned int i = 0; i < siteall.size(); i++)
            {
                A.row(i * 3) << 0 , z0[i] , -y0[i];
                A.row(i * 3 + 1) << -z0[i] , 0 , x0[i];
                A.row(i * 3 + 2) << y0[i] , -x0[i] , 0;
            }
        }

        //NNT
        if (hmtcst == HMTCONSTR::NNT)
        {
            for (unsigned int i = 0; i < siteall.size(); i++)
            {
                A.row(i * 3) << 1 , 0 , 0;
                A.row(i * 3 + 1) << 0 , 1 , 0;
                A.row(i * 3 + 3) << 0 , 0 , 1;
            }
        }

        B_temp = ((A.transpose() * A).inverse()) * (A.transpose());
        vector<pair<int, double>> B;
        string sitename = "coresite";
        for (int i = 0; i < B_temp.rows(); i++)
        {
            for (unsigned int j = 0; j < siteall.size(); j++)
            {
                B.push_back(std::make_pair(icrdx[j] + 1, B_temp(i, 3 * j)));
                B.push_back(std::make_pair(icrdy[j] + 1, B_temp(i, 3 * j + 1)));
                B.push_back(std::make_pair(icrdz[j] + 1, B_temp(i, 3 * j + 2)));
            }

            gnss_proc_lsq_equationmatrix virtual_equ;
            virtual_equ.add_equ(B, 1e8, 0, sitename, "", type, false);
            this->add_equation(virtual_equ, _beg, false);

            B.clear(); //clear B matrix
        }

        return true;
    }

    void gnss_proc_lsqbase::print_matrx()
    {
        /*_NEQ.print();
        _W.print();*/
        std::cout << "neq center is" << endl;
        for (unsigned int i = 0; i < _x_solve.parNumber(); i++)
        {
            if (_NEQ.center_value(i + 1) == 0.0)
                continue;
            std::cout << _x_solve[i].site + "_" + _x_solve[i].str_type()
                 << " "
                 << setw(20) << setprecision(15) << scientific << _NEQ.center_value(i + 1)
                 << endl;
        }
    }

    int gnss_proc_lsqbase::lsq_antpcv_constraint()
    {
        map<string, map<GFRQ, vector<int>>> indx_sat;
        map<string, map<GFRQ, vector<int>>> indx_rec;
        map<string, map<GFRQ, map<double, vector<int>>>> indx_sat_azi;
        map<string, map<GFRQ, map<int, vector<int>>>> indx_rec_azi;
        // GOBS_LC lc;
        bool lazi_sat = false, lazi_rec = false;
        bool lfirst = true;
        int ipar_first, ipair = 1;
        for (unsigned int ipar = 0; ipar < _x_solve.parNumber(); ipar++)
        {
            base_par par = _x_solve[ipar];
            if (par.str_type() == "PCV_SAT")
            {
                if (par.sat_zero_con)
                {
                    indx_sat[par.prn][*(par.fq.begin())].push_back(ipar);
                }
                if (!double_eq(par.dazi, 0.0)) // azi
                {
                    lazi_sat = true;
                    if (par.nazi == ceil(360.0 / par.dazi) + 1) // 0-360
                    {
                        if (double_eq(par.azi1, 0.0) || double_eq(par.azi1, 360.0))
                        {
                            indx_sat_azi[par.prn][*(par.fq.begin())][par.zen1].push_back(ipar);
                        }
                    }
                }
            }
            if (par.str_type() == "PCV_REC")
            {
                if (par.rec_zero_con)
                {
                    indx_rec[par.site][*(par.fq.begin())].push_back(ipar);
                }
                if (!double_eq(par.dazi, 0.0)) // azi
                {
                    if (lfirst)
                    {
                        ipar_first = ipar;
                        lfirst = false;
                    }
                    lazi_rec = true;
                    if (double_eq(par.zen1, 0.0))
                    {
                        if (ipar != ipar_first)
                        {
                            indx_rec_azi[par.site][*(par.fq.begin())][ipair * 100].push_back(ipar_first);
                            indx_rec_azi[par.site][*(par.fq.begin())][ipair * 100].push_back(ipar);
                            ipair++;
                        }
                    }
                    else
                    {
                        if (par.nazi == ceil(360.0 / par.dazi) + 1) // 0-360
                        {
                            if ((double_eq(par.azi1, 0.0) || double_eq(par.azi1, 360.0)))
                            {
                                indx_rec_azi[par.site][*(par.fq.begin())][par.zen1].push_back(ipar);
                            }
                        }
                    }
                }
            }
        }

        if (lazi_sat)
        {
            for (auto sat = indx_sat_azi.begin(); sat != indx_sat_azi.end(); sat++)
            {
                for (auto ind = sat->second.begin(); ind != sat->second.end(); ind++)
                {
                    map<double, vector<int>> inds = ind->second;
                    for (auto iter = inds.begin(); iter != inds.end(); iter++)
                    {
                        vector<int> pair = iter->second;
                        _NEQ.num(pair[0] + 1, pair[0] + 1) += 1e12;
                        _NEQ.num(pair[1] + 1, pair[1] + 1) += 1e12;
                        _NEQ.num(pair[1] + 1, pair[0] + 1) -= 1e12;
                    }
                }
            }
        }

        if (lazi_rec)
        {
            for (auto rec = indx_rec_azi.begin(); rec != indx_rec_azi.end(); rec++)
            {
                for (auto ind = rec->second.begin(); ind != rec->second.end(); ind++)
                {
                    map<int, vector<int>> inds = ind->second;
                    for (auto iter = inds.begin(); iter != inds.end(); iter++)
                    {
                        vector<int> pair = iter->second;
                        _NEQ.num(pair[0] + 1, pair[0] + 1) += 1e12;
                        _NEQ.num(pair[1] + 1, pair[1] + 1) += 1e12;
                        _NEQ.num(pair[1] + 1, pair[0] + 1) -= 1e12;
                    }
                }
            }
        }

        if (indx_sat.size() > 0)
        {
            for (auto sat = indx_sat.begin(); sat != indx_sat.end(); sat++)
            {
                for (auto ind = sat->second.begin(); ind != sat->second.end(); ind++)
                {
                    vector<int> inds = ind->second;
                    int num = inds.size();
                    for (int i = 0; i < num; i++)
                    {
                        int ip = inds[i];
                        for (int j = 0; j <= i; j++)
                        {
                            int jp = inds[j];
                            _NEQ.num(ip + 1, jp + 1) += 1e12;
                        }
                    }
                }
            }
        }
        if (indx_rec.size() > 0)
        {
            for (auto rec = indx_rec.begin(); rec != indx_rec.end(); rec++)
            {
                for (auto ind = rec->second.begin(); ind != rec->second.end(); ind++)
                {
                    vector<int> inds = ind->second;
                    int num = inds.size();
                    for (int i = 0; i < num; i++)
                    {
                        int ip = inds[i];
                        for (int j = 0; j <= i; j++)
                        {
                            int jp = inds[j];
                            _NEQ.num(ip + 1, jp + 1) += 1e12;
                        }
                    }
                }
            }
        }
        return 1;
    }

    int gnss_proc_lsqbase::lsq_antpcv_addneq(gnss_all_pcvneq *pcvneq)
    {
        map<string, shared_ptr<gnss_data_pcvneq>> neqs = pcvneq->allpcvneq();
        for (auto iter = neqs.begin(); iter != neqs.end(); iter++)
        {
            string obj = iter->first;
            shared_ptr<gnss_data_pcvneq> neq = iter->second;
            map<int, double> w_inf = neq->pcv_val();
            map<int, map<int, double>> neq_inf = neq->pcv_neq();
            int npar = w_inf.size();
            int ipar = 0;
            for (; ipar < _x_solve.parNumber(); ipar++)
            {
                base_par par = _x_solve[ipar];
                if ((obj == par.site && par.str_type() == "PCV_REC") || (obj == par.prn && par.str_type() == "PCV_SAT"))
                {
                    break;
                }
            }
            for (int i = 1; i <= npar; i++)
            {
                _W.num(ipar + i) += w_inf[i];
                for (int j = 1; j <= i; j++)
                {
                    _NEQ.num(ipar + i, ipar + j) += neq_inf[i][j];
                }
            }
        }
        return 1;
    }

    void gnss_proc_lsqbase::setlog(base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }
    const Symmetric &gnss_proc_lsqbase::Qx() const
    {
        return _Qx;
    }
    Symmetric gnss_proc_lsqbase::NEQ() const
    {
        return _NEQ.changeNewMat();
    }
    Vector gnss_proc_lsqbase::W()
    {
        return _W.changeNewMat();
    }
    V_Symmetric gnss_proc_lsqbase::v_NEQ() const
    {
        return _NEQ;
    }

    V_Vector gnss_proc_lsqbase::v_W() const
    {
        return _W;
    }
    Vector gnss_proc_lsqbase::dx() const
    {
        return _dx_final;
    }
    Vector gnss_proc_lsqbase::stdx() const
    {
        return _stdx;
    }
    double gnss_proc_lsqbase::Qx(const int &col, const int &row) const
    {
        return _Qx(col, row);
    }
    double gnss_proc_lsqbase::dx(int idx) const
    {
        return _dx_final(idx);
    }
    double gnss_proc_lsqbase::sigma0() const
    {
        return _sigma0;
    }
    double gnss_proc_lsqbase::stdx(int idx) const
    {
        return _stdx(idx);
    }
    double gnss_proc_lsqbase::vtpv() const
    {
        return _vtpv;
    }
    int gnss_proc_lsqbase::nobs_total() const
    {
        return _obs_total_num;
    }
    base_time gnss_proc_lsqbase::epo() const
    {
        return _epo;
    }
    void gnss_proc_lsqbase::epo(const base_time &t)
    {
        _epo = t;
    }
    LSQ_MODE gnss_proc_lsqbase::mode()
    {
        return _mode;
    }
    void gnss_proc_lsqbase::reset_tempfile(string filename)
    {
        if (_tempfile != nullptr)
        {
            if (_tempfile->is_open())
            {
                _tempfile->close();
            }
            remove(_tempfile->name().c_str());
            delete _tempfile;
            _tempfile = nullptr;
        }

        //_tempfile = new base_iof;
        _tempfile = new base_io_bigf(filename, _buffer_size);
        _tempfile->tsys(base_time::GPS);
        _tempfile->mask(filename);
        _tempfile->append(false);
        _tempfile->open(filename, ios::out | ios::trunc | ios::binary);

        // for begin
        int identify = -1;
        _tempfile->write((char *)&identify, SIZE_INT);
    }

    int gnss_proc_lsqbase::_write_parchage(const vector<int> &remove_id)
    {
        if (!_tempfile)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "don't init the tempfile");
            return -1;
        }

        int len_size = 0;
        _tempfile->write("swp", 3);
        len_size += 3;

        int total_size = _x_solve.parNumber();
        _tempfile->write((char *)&total_size, sizeof(int));
        len_size += sizeof(int);

        int remove_size = remove_id.size();
        _tempfile->write((char *)&remove_size, sizeof(int));
        len_size += sizeof(int);

        for (int i = 0; i < remove_size; i++)
        {
            _tempfile->write((char *)&remove_id[i], sizeof(int));
            len_size += sizeof(int);
        }

        _tempfile->write((char *)&len_size, sizeof(int));
        return 0;
    }
    int gnss_proc_lsqbase::_write_coefficient(int idx)
    {
        std::ostringstream os;
        if (idx < 1 || idx > _NEQ.num())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "the input idx is Wrong!");
            return -1;
        }
        // changge by BoWong
        if (!_tempfile)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "don't init the tempfile");
            return -1;
        }
        int len_size = 0;
        // write identifier
        _tempfile->write("par", 3);
        len_size += 3;

        // write parameter number
        vector<pair<int, double>> par_record;

        // jdhuang
        double w_record = _W.num(idx);
        int NEQ_num = _NEQ.num();
        for (int col = 1; col <= NEQ_num; col++)
        {
            double value = _NEQ.num(idx, col);
            if (!double_eq(value, 0.0))
            {
                par_record.push_back(std::make_pair(col, value));
            }
        }

        // write par num
        int par_num = par_record.size();
        _tempfile->write((char *)&par_num, SIZE_INT);
        len_size += SIZE_INT;

        // write parameter index
        _tempfile->write((char *)&idx, SIZE_INT);
        len_size += SIZE_INT;

        // write par name station satlltie
        // jdhuang
        const base_par &par_tmp = _x_solve[idx - 1];
        string name = par_tmp.site + "_" + par_tmp.str_type();
        if (par_tmp.prn == "")
            name = name + "_";
        _tempfile->write(name.c_str(), 20);
        len_size += 20;

        // write par time
        // change by glfeng write start and end time
        _tempfile->write((char *)&par_tmp.beg, sizeof(base_time));
        len_size += sizeof(base_time);
        _tempfile->write((char *)&par_tmp.end, sizeof(base_time));
        len_size += sizeof(base_time);

        // write parameter value
        double value = par_tmp.value();
        _tempfile->write((char *)&value, SIZE_DBL);
        len_size += SIZE_DBL;

        // write zhd for ztd retrieval glfeng
        double zhd = par_tmp.zhd;
        _tempfile->write((char *)&zhd, SIZE_DBL);
        len_size += SIZE_DBL;

        // write coefficient BTPB
        for (int ipar = 1; ipar <= par_num; ipar++)
        {
            _tempfile->write((char *)&(par_record[ipar - 1].first), SIZE_INT);
            len_size += SIZE_INT;
            _tempfile->write((char *)&(par_record[ipar - 1].second), SIZE_DBL);
            len_size += SIZE_DBL;
        }

        // write BTPL
        _tempfile->write((char *)&w_record, SIZE_DBL);
        len_size += SIZE_DBL;

        _tempfile->write((char *)&len_size, SIZE_INT);

        return 1;
    }

    void gnss_proc_lsqbase::_recover_par(base_io_READTEMP &tempfile_in, gnss_all_recover &_allrecover)
    {

        // read par number
        int par_num;
        tempfile_in.read((char *)&par_num, SIZE_INT);

        if (par_num > _dx.rows() + 1)
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "par recover more than par now Recover Fail!");
            throw exception();
        }

        // read idx
        int idx;
        tempfile_in.read((char *)&idx, SIZE_INT);
        idx--;

        // read par name sat
        char par_info[20];
        tempfile_in.read(par_info, 20);

        // read par time  change by glfeng
        base_time par_begtime;
        tempfile_in.read((char *)&par_begtime, sizeof(base_time));
        base_time par_endtime;
        tempfile_in.read((char *)&par_endtime, sizeof(base_time));

        // read par value
        double par_value;
        tempfile_in.read((char *)&par_value, SIZE_DBL);

        // write zhd for ztd retrieval glfeng
        double par_zhd;
        tempfile_in.read((char *)&par_zhd, SIZE_DBL);

        // N for recording coefficient data
        vector<int> loc;
        vector<double> N;
        for (int ipar = 0; ipar < par_num; ipar++)
        {
            int i;
            double j;
            tempfile_in.read((char *)&i, SIZE_INT);
            i--;
            loc.push_back(i);
            tempfile_in.read((char *)&j, SIZE_DBL);
            j--;
            N.push_back(j);
        }

        // read omc
        double omc = 0.0;
        tempfile_in.read((char *)&omc, SIZE_DBL);

        // Resize for dx !! Size is par_num+1
        Vector dx_temp = _dx;
        _dx.resize(_dx.rows() + 1);
        _dx.setZero();

        if (idx != 0)
        {
            _dx.segment(0, idx - 1) << dx_temp.segment(0, idx - 1);
        }
        if (idx != _dx.rows() - 1)
        {
            _dx.segment(idx, _dx.rows()- 1 - idx) = dx_temp.segment(idx - 1, _dx.rows()- 1 - idx);
        }

        dx_temp = _dx;

        // compute the dx
        if (find(loc.begin(), loc.end(), idx) == loc.end())
        {
            _dx(idx) = 0;
        }
        else
        {
            _dx(idx) = omc;
            int flag_idx = 0;
            for (int ipar = 0; ipar < par_num; ipar++)
            {
                if (loc[ipar] != idx)
                {
                    _dx(idx) = _dx(idx) - (N[ipar] * dx_temp(loc[ipar]));
                }
                else
                {
                    flag_idx = ipar;
                }
            }
            _dx(idx) = _dx(idx) / N[flag_idx];
        }

        base_par par = str2gpar(par_info);
        par.beg = par_begtime;
        par.end = par_endtime;
        par.value(par_value);
        par.zhd = par_zhd;
        gnss_data_recover_par recover_par(_spdlog, par, _dx(idx));
        _allrecover.add_recover_par(recover_par);
    }
    void gnss_proc_lsqbase::_recover_par_part(base_io_READTEMP &tempfile_in, gnss_all_recover &_allrecover)
    {
        // read par number
        int par_num;
        tempfile_in.read((char *)&par_num, SIZE_INT);

        // read idx
        int idx;
        tempfile_in.read((char *)&idx, SIZE_INT);
        idx--;

        // read par name sat
        char par_info[20];
        tempfile_in.read(par_info, 20);

        // read par time  change by glfeng
        base_time par_begtime;
        tempfile_in.read((char *)&par_begtime, sizeof(base_time));
        base_time par_endtime;
        tempfile_in.read((char *)&par_endtime, sizeof(base_time));

        // read par value
        double par_value;
        tempfile_in.read((char *)&par_value, SIZE_DBL);

        // write zhd for ztd retrieval glfeng
        double par_zhd;
        tempfile_in.read((char *)&par_zhd, SIZE_DBL);

        // N for recording coefficient data
        vector<int> loc;
        vector<double> N;
        for (int ipar = 0; ipar < par_num; ipar++)
        {
            int i;
            double j;
            tempfile_in.read((char *)&i, SIZE_INT);
            i--;
            loc.push_back(i);
            tempfile_in.read((char *)&j, SIZE_DBL);
            j--;
            N.push_back(j);
        }

        // read omc
        double omc = 0.0;
        tempfile_in.read((char *)&omc, SIZE_DBL);

        // Resize for dx !! Size is par_num+1
        Vector dx_temp = _dx;
        _dx.resize(_dx.rows() + 1);
        _dx.setZero();

        if (idx != 0)
        {
            _dx.segment(0, idx - 1) = dx_temp.segment(0, idx - 1);
        }
        if (idx != _dx.rows() - 1)
        {
            _dx.segment(idx, _dx.rows() - idx - 1) = dx_temp.segment(idx - 1, _dx.rows()- 1 - idx);
        }

        dx_temp = _dx;

        // compute the dx
        _dx(idx) = omc;
        for (int ipar = 0; ipar < par_num; ipar++)
        {
            _dx(idx) -= (N[ipar] * dx_temp(loc[ipar]));
        }

        for (unsigned int i = 0; i < loc.size(); i++)
        {
            assert(loc[i] != idx);
        }

        base_par par = str2gpar(par_info);
        par.beg = par_begtime;
        par.end = par_endtime;
        par.value(par_value);
        par.zhd = par_zhd;
        gnss_data_recover_par recover_par(_spdlog, par, _dx(idx));
        _allrecover.add_recover_par(recover_par);
    }
    void gnss_proc_lsqbase::_recover_par_swap(base_io_READTEMP &tempfile_in)
    {
        // read total size
        int total_size = 0;
        tempfile_in.read((char *)&total_size, sizeof(int));

        // read remove size
        int remove_size = 0;
        tempfile_in.read((char *)&remove_size, sizeof(int));

        // read remove_id;
        vector<int> remove_id(remove_size, 0);
        for (int i = 0; i < remove_size; i++)
        {
            tempfile_in.read((char *)&remove_id[i], sizeof(int));
            remove_id[i]--;
        }

        vector<int> remove_id_sort(remove_id.begin(), remove_id.end());
        sort(remove_id_sort.begin(), remove_id_sort.end());

        Vector dx_temp = _dx;
        int idx_old = 0, idx_new = 0;
        for (int i = 0; i < total_size; i++)
        {
            if (idx_new < remove_size && i == remove_id_sort[idx_new])
            {
                //_dx(i) = dx_temp(total_size - remove_size + 1 + idx_new);
                idx_new++;
            }
            else
            {
                _dx(i) = dx_temp(idx_old);
                idx_old++;
            }
        }

        for (int i = 0; i < remove_size; i++)
        {
            _dx(remove_id[i]) = dx_temp(total_size - remove_size + i);
        }
    }
    void gnss_proc_lsqbase::_recover_obs(base_io_READTEMP &tempfile_in, gnss_all_recover &_allrecover)
    {
        // read parnumber
        int par_num;
        tempfile_in.read((char *)&par_num, SIZE_INT);

        // read time
        base_time time;
        tempfile_in.read((char *)&time, sizeof(base_time));

        // read ambflag;
        int is_newamb;
        tempfile_in.read((char *)&is_newamb, SIZE_INT);

        // read info
        int len_info;
        tempfile_in.read((char *)&len_info, SIZE_INT);
        char *info = new char[len_info];
        tempfile_in.read(info, len_info);

        // B for recording the coefficient of obs equation
        if (par_num > _dx.rows())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "par recover more than par now Recover Fail!");
            throw exception();
        }

        //shared_ptr<double> B(new double[par_num]);
        vector<int> loc;
        vector<double> B;

        for (int ipar = 1; ipar <= par_num; ipar++)
        {
            int i;
            double j;
            tempfile_in.read((char *)&i, SIZE_INT);
            i--;
            loc.push_back(i);
            tempfile_in.read((char *)&j, SIZE_DBL);
            B.push_back(j);
        }

        int Max_loc;
        tempfile_in.read((char *)&Max_loc, SIZE_INT);
        if (_dx.rows() > Max_loc)
        {
            Vector _dx_temp = _dx;
            _dx.resize(Max_loc);
            _dx << _dx_temp.segment(0, Max_loc);
        }

        // read p
        double p;
        tempfile_in.read((char *)&p, SIZE_DBL);

        // read old res
        double res;
        tempfile_in.read((char *)&res, SIZE_DBL);

        // compute the new res
        for (int ipar = 0; ipar < par_num; ipar++)
        {
            res -= B[ipar] * _dx(loc[ipar]);
        }

        stringstream obsinfo(string(info, len_info));
        delete[] info;
        info = nullptr;
        string site, sat, str_obstype;
        obsinfo >> site >> sat >> str_obstype;
        if (str_obstype == "KBRRATR")
        {
            str_obstype = "KBRRATE";
        }
        gnss_data_recover_equation recover_equ(_spdlog, time, site, sat);
        //std::cout << obsinfo.str() << setw(10) << p << "           " << setw(10) << res << endl; // jdhuang for debug
        recover_equ.set_recover_equation(gnss_data_obscombtype(str_obstype), std::make_pair(p, res), is_newamb);
        _allrecover.add_recover_equation(recover_equ);
    }
    void gnss_proc_lsqbase::_remove_zero_element(Symmetric &B, Vector &l, int idx)
    {
        idx--;
        if (idx < 0 || idx >= B.rows() || B.rows() != l.rows())
        {
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "input is Wrong!");
            throw exception();
        }

        Symmetric temp_B = B;
        Vector temp_l = l;
        B.resize(temp_B.rows() - 1);
        B.setZero();
        l.resize(temp_l.rows() - 1);
        l.setZero();

        if (idx == 0)
        {
            B.matrixW() = temp_B.matrixR().block(1, 1, temp_B.rows() - 1, temp_B.rows() - 1);
            l << temp_l.segment(1, temp_l.rows() - 1);
        }
        else if (idx == temp_B.rows() - 1)
        {
            B.matrixW() = temp_B.matrixR().block(0, 0, temp_B.rows() - 1, temp_B.rows() - 1);
            l << temp_l.segment(0, temp_l.rows() - 1);
        }
        else
        {
            B.matrixW().block(0, 0, idx - 1, idx - 1) << temp_B.matrixR().block(1, 1, idx - 1, idx - 1);
            B.matrixW().block(idx, 0, B.rows() - idx + 1, idx - 1) << temp_B.matrixR().block(idx + 1, 0, temp_B.rows() - idx, idx - 1);
            B.matrixW().block(idx, idx, B.rows() - idx + 1, B.cols() - idx + 1) << temp_B.matrixR().block(idx + 1, idx + 1, temp_B.rows() - idx, temp_B.cols() - idx);

            l.segment(0, idx - 1) << temp_l.segment(0, idx - 1);
            l.segment(idx, l.rows() - idx + 1) << temp_l.segment(idx + 1, temp_l.rows() - idx);
        }
    }
    void gnss_proc_lsqbase::_solve_equation(const Symmetric &NEQ, const Vector &W, Vector &ans, Vector &Q)
    {

        Matrix Inverse_N = NEQ.matrixR();

        ans = Inverse_N.llt().solve(W);

        _Qx.matrixW() = Inverse_N;
        Q.resize(Inverse_N.rows());
        for (int i = 0; i < Inverse_N.rows(); i++)
        {
            Q(i) = Inverse_N(i, i);
        }
    }
    void gnss_proc_lsqbase::_solve_x(const Symmetric &NEQ, const Vector &W, Vector &ans)
    {
        ans = NEQ.matrixR().llt().solve(W);
    }
    void gnss_proc_lsqbase::change_NEQ(int row, int col, double xx)
    {
        _NEQ.num(row, col) = xx;
    }

    void gnss_proc_lsqbase::change_Qx(int row, int col, double xx)
    {
        _Qx.set(xx, row, col);
    }

    void gnss_proc_lsqbase::change_Qx(Symmetric Qx)
    {
        _Qx = Qx;
    }

    void gnss_proc_lsqbase::change_dx(int n, double xx)
    {
        if (n < 0)
        {
            _dx_final.setConstant(xx);
        }
        else
        {
            _dx_final(n) = xx;
        }
    }

    void gnss_proc_lsqbase::change_dx(Vector dx)
    {
        _dx_final << dx;
    }

    void gnss_proc_lsqbase::change_stdx(int n, double xx)
    {

        _stdx(n) = xx;
    }

    void gnss_proc_lsqbase::change_stdx(Vector stdx)
    {

        _stdx << stdx;
    }

    void gnss_proc_lsqbase::change_sigma0(double sigma)
    {
        _sigma0 = sigma;
    }

    void gnss_proc_lsqbase::change_vtpv(double xx)
    {
        _vtpv = xx;
    }

    void gnss_proc_lsqbase::set_new_NEQ(const V_Vector &W, const V_Symmetric &NEQ)
    {
        _W = W;
        _NEQ = NEQ;
    }

    int gnss_proc_lsqbase::reset_npar_total(const base_par &par)
    {
        // Ionosphere/Clk_Sat/IFCB  estimate per epoch
        if (par.parType == par_type::SION || par.parType == par_type::VION || par.parType == par_type::CLK_SAT || par.str_type().find("IFCB") != string::npos)
        {
            return 1;
        }
        // Ambiguity per Arc
        if (par.str_type().find("AMB") != string::npos && _epo == par.beg)
        {
            return 1;
        }

        // IFB per Arc
        if (par.str_type().find("IFB") != string::npos && _epo == par.beg)
        {
            return 1;
        }

        return 0;
    }

    // changge by BoWong
    int gnss_proc_lsqbase::npar_number() const
    {
        return _npar_tot_num;
    }
    base_time str2gtime(string str_time)
    {

        replace(str_time.begin(), str_time.end(), '-', ' ');
        replace(str_time.begin(), str_time.end(), ':', ' ');
        stringstream time(str_time);
        int year, month, day, hour, minute;
        double sec;
        time >> year >> month >> day >> hour >> minute >> sec;
        base_time time_t;
        time_t.from_ymdhms(year, month, day, hour, minute, sec);
        return time_t;
    }
}
