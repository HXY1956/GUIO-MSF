#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <emmintrin.h>
#include <assert.h>
#ifdef USE_OPENBLAS
#include "cblas.h"
#endif
#include "hwa_base_eigendef.h"
#include "hwa_gnss_proc_lsqmatrix.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{

    gnss_proc_lsq_equationmatrix::gnss_proc_lsq_equationmatrix()
    {
    }

    gnss_proc_lsq_equationmatrix::~gnss_proc_lsq_equationmatrix()
    {
    }

    //void gnss_proc_lsq_equationmatrix::add_equ(const vector<pair<int, double> >& B_value, const vector<pair<int, double> >& P_raw_value, const double& P_value, const double& l_value, const string& stie_name, const string& sat_name, const gnss_data_obscombtype& obscombtype, const bool& is_newamb)
    //{
    //    B.push_back(B_value);
    //    P.push_back(P_value);
    //    P_raw.push_back(P_raw_value);
    //    l.push_back(l_value);
    //    this->_site_sat_pairlist.push_back(std::make_pair(stie_name, sat_name));
    //    this->_obstypelist.push_back(obscombtype);
    //    this->_newamb_list.push_back(is_newamb);
    //}

    void gnss_proc_lsq_equationmatrix::add_equ(const vector<pair<int, double>> &B_value, const double &P_value, const double &l_value, const string &stie_name, const string &sat_name, const gnss_data_obscombtype &obscombtype, const bool &is_newamb)
    {
        B.push_back(B_value);
        P.push_back(P_value);
        l.push_back(l_value);
        this->_site_sat_pairlist.push_back(std::make_pair(stie_name, sat_name));
        this->_obstypelist.push_back(obscombtype);
        this->_newamb_list.push_back(is_newamb);
    }

    void gnss_proc_lsq_equationmatrix::add_equ(const gnss_proc_lsq_equationmatrix &Other)
    {
        for (int i = 0; i < Other.num_equ(); i++)
        {
            this->add_equ(Other.B[i], Other.P[i], Other.l[i], Other._site_sat_pairlist[i].first, Other._site_sat_pairlist[i].second, Other._obstypelist[i], false);
        }
    }

    void gnss_proc_lsq_equationmatrix::add_equ(const Matrix &B_value, const Diag &P_value, const Vector &l_value, const vector<string> &site_name, const vector<string> &sat_name, const vector<gnss_data_obscombtype> &obstype)
    {

        if (B_value.rows() != P_value.rows() || B_value.rows() != l_value.rows() ||
            B_value.rows() != site_name.size() || B_value.rows() != sat_name.size() || B_value.rows() != obstype.size())
        {
            throw exception();
        }

        // int num_equ = B_value.rows();
        //for B
        vector<pair<int, double>> temp;
        for (int row = 0; row < B_value.rows(); row++)
        {
            temp.clear();
            for (int col = 0; col < B_value.cols(); col++)
            {
                if (B_value(row, col) == 0.0)
                {
                    continue;
                }
                temp.push_back(std::make_pair(col, B_value(row, col)));
            }

            B.push_back(temp);

            P.push_back(P_value(row));
            l.push_back(l_value(row));

            // Note: row from 1
            _site_sat_pairlist.push_back(std::make_pair(site_name[row], sat_name[row]));
            _obstypelist.push_back(obstype[row]);
            _newamb_list.push_back(false);
        }
    }

    void gnss_proc_lsq_equationmatrix::set_newamb(const int &idx, const bool &is_newamb)
    {
        if (idx >= this->num_equ())
        {
            throw exception(std::logic_error("input idx is more than number of equ"));
        }

        _newamb_list[idx] = is_newamb;
    }

    void gnss_proc_lsq_equationmatrix::set_newamb(gnss_data_turboedit *tb_slip)
    {
        if (!tb_slip)
        {
            return;
        }

        set<pair<string, string>> site_sat_list;
        for (unsigned int i = 0; i < _site_sat_pairlist.size(); i++)
        {
            if (!this->get_obscombtype(i).is_phase())
            {
                continue;
            }
            string rec = this->get_sitename(i);
            string sat = this->get_satname(i);

            if (tb_slip->new_amb(rec, sat))
            {
                _newamb_list[i] = true;
                site_sat_list.insert(std::make_pair(rec, sat));
            }
        }
        for (auto iter : site_sat_list)
        {
            tb_slip->set_new_amb(iter.first, iter.second, false);
        }
    }

    void gnss_proc_lsq_equationmatrix::set_newamb(const FREQ_SEQ &freq_1, const FREQ_SEQ &freq_2, gnss_data_turboedit *tb_slip)
    {
        if (!tb_slip)
        {
            return;
        }

        set<pair<string, string>> site_sat_list;
        for (unsigned int i = 0; i < _site_sat_pairlist.size(); i++)
        {
            auto obscombtype = this->get_obscombtype(i);
            if (!obscombtype.is_phase() || !obscombtype.is_freq(freq_1, freq_2))
            {
                continue;
            }
            string rec = this->get_sitename(i);
            string sat = this->get_satname(i);

            if (tb_slip->new_amb(rec, sat))
            {
                _newamb_list[i] = true;
                site_sat_list.insert(std::make_pair(rec, sat));
            }
        }
        for (auto iter : site_sat_list)
        {
            tb_slip->set_new_amb(iter.first, iter.second, false);
        }
    }

    void gnss_proc_lsq_equationmatrix::remove(const int &idx)
    {
        B.erase(B.begin() + idx - 1);
        P.erase(P.begin() + idx - 1);
        l.erase(l.begin() + idx - 1);

        _site_sat_pairlist.erase(_site_sat_pairlist.begin() + idx - 1);
        _obstypelist.erase(_obstypelist.begin() + idx - 1);
        _newamb_list.erase(_newamb_list.begin() + idx - 1);
    }

    void gnss_proc_lsq_equationmatrix::remove_last_equation()
    {
        B.pop_back();
        P.pop_back();
        l.pop_back();
        _site_sat_pairlist.pop_back();
        _obstypelist.pop_back();
        _newamb_list.pop_back();
    }

    void gnss_proc_lsq_equationmatrix::chageNewMat(Matrix &B_value, Diag &P_value, Vector &l_value, const int &par_num)
    {
        B_value.resize(B.size(), par_num);
        B_value.setZero();
        for (unsigned int row = 0; row < B.size(); row++)
        {
            for (unsigned int col = 0; col < B[row].size(); col++)
            {
                B_value(row, B[row][col].first) = B[row][col].second;
            }
        }
        P_value.resize(P.size());
        P_value.setZero();
        for (unsigned int row = 0; row < P.size(); row++)
        {
            P_value(row) = P[row];
        }
        l_value.resize(l.size());
        l_value.setZero();
        for (unsigned int row = 0; row < l.size(); row++)
        {
            l_value(row) = l[row];
        }
    }

    void gnss_proc_lsq_equationmatrix::chageNewMat(Matrix &B_value, Symmetric &P_value, Vector &l_value, const int &par_num)
    {
        B_value.resize(B.size(), par_num);
        B_value.setZero();
        for (unsigned int row = 0; row < B.size(); row++)
        {
            for (unsigned int col = 0; col < B[row].size(); col++)
            {
                B_value(row, B[row][col].first) = B[row][col].second;
            }
        }
        P_value.resize(P.size());
        P_value.setZero();
        for (unsigned int row = 0; row < P.size(); row++)
        {
            P_value.set(P[row], row, row);
        }
        l_value.resize(l.size());
        l_value.setZero();
        for (unsigned int row = 0; row < l.size(); row++)
        {
            l_value(row) = l[row];
        }
    }

    void gnss_proc_lsq_equationmatrix::print()
    {
        int count = 0;
        std::cout << "B is" << endl;
        for (int i = 0; i < B.size();)
        {
            count = 0;
            for (int j = 0; j < B[i].size(); j++)
            {
                count++;
                std::cout << scientific << setprecision(15) << setw(30) << B[i][j].second << "   ";
                if (count % 3 == 0)
                    std::cout << endl;
            }
            std::cout << endl;
            i += 2;
        }
        std::cout << "P is" << endl;
        for (int i = 0; i < B.size(); i++)
        {
            std::cout << scientific << setprecision(15) << setw(30) << P[i] << endl;
        }
        std::cout << "l is" << endl;
        for (int i = 0; i < B.size();)
        {
            std::cout << scientific << setprecision(15) << setw(30) << l[i] << endl;
            i += 2;
        }
    }

    int gnss_proc_lsq_equationmatrix::num_equ() const
    {
        return B.size();
    }

    double gnss_proc_lsq_equationmatrix::res_equ() const
    {
        double ans = 0;
        for (int i = 0; i < num_equ(); i++)
        {
            ans += P[i] * l[i] * l[i];
        }
        return ans;
    }

    double gnss_proc_lsq_equationmatrix::res_equ(bool phase) const
    {
        if (!phase)
        {
            return this->res_equ();
        }
        double ans = 0;
        for (int i = 0; i < num_equ(); i++)
        {
            if (get_obscombtype(i).is_code())
            {
                continue;
            }
            ans += P[i] * l[i] * l[i];
        }
        return ans;
    }

    string gnss_proc_lsq_equationmatrix::get_sitename(int equ_idx) const
    {
        return _site_sat_pairlist[equ_idx].first;
    }

    set<string> gnss_proc_lsq_equationmatrix::get_satlist(string rec) const
    {
        set<string> sat_list;
        int num_equ = this->num_equ();
        for (int i = 0; i < num_equ; i++)
        {
            if (_site_sat_pairlist[i].first == rec)
            {
                sat_list.insert(_site_sat_pairlist[i].second);
            }
        }

        return sat_list;
    }

    string gnss_proc_lsq_equationmatrix::get_satname(int equ_idx) const
    {
        return _site_sat_pairlist[equ_idx].second;
    }

    vector<pair<string, string>> gnss_proc_lsq_equationmatrix::get_site_sat_pair() const
    {
        return _site_sat_pairlist;
    }

    string gnss_proc_lsq_equationmatrix::get_obscombtype2str(int equ_idx) const
    {
        return _obstypelist[equ_idx].convert2str();
    }

    gnss_data_obscombtype gnss_proc_lsq_equationmatrix::get_obscombtype(int equ_idx) const
    {
        return _obstypelist[equ_idx];
    }

    vector<double> gnss_proc_lsq_equationmatrix::get_codeomc() const
    {
        int equ_num = this->num_equ();
        vector<double> codeomc;
        for (int i = 0; i < equ_num; i++)
        {
            if (_obstypelist[i].is_code() && (_obstypelist[i].is_freq12() || _obstypelist[i].is_freq_raw1()))
            //if (_obstypelist[i].is_code() && _obstypelist[i].is_freq12())
            {
                codeomc.push_back(l[i]);
            }
        }
        return codeomc;
    }
    vector<double> gnss_proc_lsq_equationmatrix::get_phaseomc() const
    {
        int equ_num = this->num_equ();
        vector<double> phaseomc;
        for (int i = 0; i < equ_num; i++)
        {
            if (_obstypelist[i].is_phase())
            {
                phaseomc.push_back(l[i]);
            }
        }
        return phaseomc;
    }

    bool gnss_proc_lsq_equationmatrix::is_newamb(int equ_idx) const
    {
        return _newamb_list[equ_idx];
    }
    void print_equ_debinfo(const gnss_proc_lsq_equationmatrix &equ)
    {
        for (int i = 0; i < equ.num_equ() / 2.0; i++)
        {
            std::cout << setw(5) << equ._site_sat_pairlist[2 * i].first
                 << setw(5) << equ._site_sat_pairlist[2 * i].second
                 << setiosflags(ios::fixed) << setprecision(4)
                 << setw(13) << equ.l[2 * i]
                 << setw(13) << equ.l[2 * i + 1]
                 << endl;
        }
    }

    vector<int> gnss_proc_lsq_equationmatrix::find_equ(const string &site)
    {
        vector<int> ans;
        int num_equ = this->num_equ();
        for (int i = 0; i < num_equ; i++)
        {
            if (_site_sat_pairlist[i].first == site)
            {
                ans.push_back(i);
            }
        }
        return ans;
    }

    vector<int> gnss_proc_lsq_equationmatrix::find_equ(const string &site, const string &sat)
    {
        vector<int> ans;
        int num_equ = this->num_equ();
        auto site_sat = std::make_pair(site, sat);
        for (int i = 0; i < num_equ; i++)
        {
            if (_site_sat_pairlist[i] == site_sat)
            {
                ans.push_back(i);
            }
        }
        return ans;
    }

    int gnss_proc_lsq_equationmatrix::find_equ(const string &site, const string &sat, const gnss_data_obscombtype &obscomtype)
    {
        int ans = -1;
        int num_equ = this->num_equ();
        auto site_sat = std::make_pair(site, sat);
        for (int i = 0; i < num_equ; i++)
        {
            if (_site_sat_pairlist[i] == site_sat && _obstypelist[i] == obscomtype)
            {
                ans = i;
                break;
            }
        }
        return ans;
    }

    void gnss_proc_lsq_equationmatrix::clear_allequ()
    {
        B.clear();
        P.clear();
        l.clear();
        _site_sat_pairlist.clear();
        _obstypelist.clear();
        _newamb_list.clear();
    }

    L_Symmetric::L_Symmetric() : row_record(0),
                                             col_record(0)
    {
    }

    L_Symmetric::~L_Symmetric()
    {
    }

    int L_Symmetric::num() const
    {
        return _element.size();
    }

    double &L_Symmetric::num(int a, int b)
    {
        int row, col;
        if (a < b)
        {
            row = b;
            col = a;
        }
        else
        {
            row = a;
            col = b;
        }

        if (row >= row_record)
        {
            advance(row_it, row - row_record);
        }
        else
        {
            row_it = _element.begin();
            advance(row_it, row - 1);
        }

        if (row == row_record && col >= col_record)
        {
            advance(col_it, col - col_record);
        }
        else
        {
            col_it = row_it->begin();
            advance(col_it, col - 1);
        }
        row_record = row;
        col_record = col;

        return *col_it;
    }

    //double L_SymmericMatrix::num(int a, int b) const
    //{
    //    int row, col;
    //    if (a < b) {
    //        row = b;
    //        col = a;
    //    }
    //    else
    //    {
    //        row = a;
    //        col = b;
    //    }
    //    if (row >= row_record) {
    //        advance(row_iter, row - row_record);
    //    }
    //    else{
    //        advance(_element.begin(), row - 1);
    //    }
    //    row_record = row;

    //    if (col >= col_record) {
    //        advance(col_iter,col-b)
    //    }

    //    auto row_iter = _element.begin();
    //    advance(row_iter, row - 1);
    //    auto col_iter = row_iter->begin();
    //    advance(col_iter, col - 1);
    //    return *col_iter;
    //}

    void L_Symmetric::resize(int size)
    {
        for (int i = 1; i <= size; i++)
        {
            _element.push_back(list<double>(i, 0.0));
        }
        row_it = _element.begin();
        row_record = 1;
        col_it = row_it->begin();
        col_record = 1;
    }

    void L_Symmetric::add(const gnss_proc_lsq_equationmatrix &equ)
    {
        // TODO
        for (int num = 0; num < equ.num_equ(); num++)
        {
            auto row_iter = _element.begin();
            advance(row_iter, equ.B[num][0].first);
            for (unsigned int num_row = 0; num_row < equ.B[num].size(); num_row++)
            {
                if (num_row > 0)
                {
                    advance(row_iter, equ.B[num][num_row].first - equ.B[num][num_row - 1].first);
                }
                auto col_iter = row_iter->begin();
                advance(col_iter, equ.B[num][0].first);
                for (unsigned int num_col = 0; num_col < num_row + 1; num_col++)
                {
                    if (num_col > 0)
                    {
                        advance(col_iter, equ.B[num][num_col].first - equ.B[num][num_col - 1].first);
                    }
                    *col_iter = *col_iter + equ.B[num][num_row].second * equ.P[num] * equ.B[num][num_col].second;
                }
            }
        }
        row_record = 1;
        row_it = _element.begin();
        col_record = 1;
        col_it = row_it->begin();
    }

    void L_Symmetric::addBackZero()
    {
        _element.push_back(list<double>(_element.size() + 1, 0.0));
        row_record = 1;
        row_it = _element.begin();
        col_record = 1;
        col_it = row_it->begin();
    }

    void L_Symmetric::remove(int idx)
    {
        if (idx < 1 || idx > _element.size())
        {
            throw exception();
        }

        if (idx == 1)
        {
            _element.pop_front();
            for (auto iter = _element.begin(); iter != _element.end(); iter++)
            {
                iter->pop_front();
            }
        }
        else if (idx == _element.size())
        {
            _element.pop_back();
        }
        else
        {
            auto iterRow = _element.begin();
            advance(iterRow, idx - 1);
            iterRow = _element.erase(iterRow);
            while (iterRow != _element.end())
            {
                auto iterCol = iterRow->begin();
                advance(iterCol, idx - 1);
                iterRow->erase(iterCol);
                iterRow++;
            }
        }

        row_record = 1;
        row_it = _element.begin();
        col_record = 1;
        col_it = row_it->begin();
    }

    void L_Symmetric::print()
    {

        std::cout << setw(20) << setprecision(5);
        for (auto Row = _element.begin(); Row != _element.end(); Row++)
        {
            for (auto Col = Row->begin(); Col != Row->end(); Col++)
            {
                std::cout << setw(20) << *Col;
            }
            std::cout << endl;
        }
    }

    double L_Symmetric::center_value(int idx) const
    {
        if (idx < 1 || idx > _element.size())
        {
            throw exception();
        }

        auto iter = _element.begin();
        advance(iter, idx - 1);
        return *(iter->rbegin());
    }

    Symmetric L_Symmetric::changeNewMat() const
    {
        Symmetric temp(_element.size());
        int row = 0, col = 0;
        for (auto Row = _element.begin(); Row != _element.end(); Row++)
        {
            col = 0;
            for (auto Col = Row->begin(); Col != Row->end(); Col++)
            {
                temp.set(*Col, row, col);
                col++;
            }
            row++;
        }
        return temp;
    }

    L_Vector::L_Vector() : row_it(_element.begin()),
                                       row_record(1)
    {
    }

    L_Vector::~L_Vector()
    {
    }

    int L_Vector::num() const
    {
        return _element.size();
    }

    double &L_Vector::num(int a)
    {
        //auto row_iter = _element.begin();
        //advance(row_iter, a - 1);
        //return *row_iter;

        if (a >= row_record)
        {
            advance(row_it, a - row_record);
        }
        else
        {
            row_it = _element.begin();
            advance(row_it, a - 1);
        }
        row_record = a;

        return *row_it;
    }

    //double L_Vector::num(int a) const
    //{
    //    auto row_iter = _element.begin();
    //    advance(row_iter, a - 1);
    //    return *row_iter;
    //}

    void L_Vector::add(const gnss_proc_lsq_equationmatrix &equ)
    {
        for (int num = 0; num < equ.num_equ(); num++)
        {
            auto iter_element = _element.begin();
            advance(iter_element, equ.B[num][0].first);
            for (int num_par = 0; num_par < equ.B[num].size(); num_par++)
            {
                if (num_par > 0)
                {
                    advance(iter_element, equ.B[num][num_par].first - equ.B[num][num_par - 1].first);
                }
                *iter_element = *iter_element + equ.B[num][num_par].second * equ.P[num] * equ.l[num];
            }
        }

        row_it = _element.begin();
        row_record = 1;
    }

    void L_Vector::addBackZero()
    {
        _element.push_back(0.0);

        row_it = _element.begin();
        row_record = 1;
    }

    void L_Vector::resize(int size)
    {
        for (int i = 1; i <= size; i++)
        {
            _element.push_back(0.0);
        }

        row_it = _element.begin();
        row_record = 1;
    }

    void L_Vector::remove(int idx)
    {
        auto row = _element.begin();
        advance(row, idx - 1);
        _element.erase(row);

        row_it = _element.begin();
        row_record = 1;
    }

    void L_Vector::print()
    {
        std::cout << setprecision(5);
        for (auto row = _element.begin(); row != _element.end(); row++)
        {
            std::cout << setw(20) << *row << endl;
        }
    }

    Vector L_Vector::changeNewMat()
    {
        Vector temp(_element.size());
        int row = 0;
        for (auto Row = _element.begin(); Row != _element.end(); Row++)
        {
            temp(row) = *Row;
            row++;
        }
        return temp;
    }

    int V_Symmetric::num() const
    {
        return _element.size();
    }

    //double& V_Symmetric::num(int a, int b)
    //{
    //    int row = a, col = b;
    //    if (a < b) {
    //        row = b;
    //        col = a;
    //    }
    //    return _element[row - 1][col - 1];
    //}

    void V_Symmetric::resize(int num)
    {
        _element.clear();
        _element.reserve(num);
        for (int i = 0; i < num; i++)
        {
            _element.push_back(vector<double>(i + 1, 0.0));
        }
    }

    void V_Symmetric::add(const gnss_proc_lsq_equationmatrix &equ)
    {
        // cycle equ
        for (int num = 0; num < equ.num_equ(); num++)
        {
            // cycle par
            auto B_temp = equ.B[num];
            sort(B_temp.begin(), B_temp.end());
            for (int ipar = 0; ipar < B_temp.size(); ipar++)
            {
                auto row = B_temp[ipar];
                for (int jpar = 0; jpar <= ipar; jpar++)
                {
                    auto col = B_temp[jpar];
                    _element[row.first][col.first] += row.second * equ.P[num] * col.second;
                }
            }
        }
    }

    void V_Symmetric::del(const gnss_proc_lsq_equationmatrix &equ)
    {
        // cycle equ
        for (int num = 0; num < equ.num_equ(); num++)
        {
            // cycle par
            auto B_temp = equ.B[num];
            sort(B_temp.begin(), B_temp.end());
            for (unsigned int ipar = 0; ipar < B_temp.size(); ipar++)
            {
                auto row = B_temp[ipar];
                for (int jpar = 0; jpar <= ipar; jpar++)
                {
                    auto col = B_temp[jpar];
                    _element[row.first][col.first] -= row.second * equ.P[num] * col.second;
                }
            }
        }
    }

    void V_Symmetric::add_related(const gnss_proc_lsq_equationmatrix &equ)
    {
        // cycle equ
        for (int num = 0; num < equ.num_equ(); num++)
        {
            // cycle par
            auto B_temp = equ.B[num];
            sort(B_temp.begin(), B_temp.end());
            for (int ipar = 0; ipar < B_temp.size(); ipar++)
            {
                auto row = B_temp[ipar];
                for (int jpar = 0; jpar <= ipar; jpar++)
                {
                    auto col = B_temp[jpar];
                    _element[row.first][col.first] += row.second * equ.P[num] * col.second;
                }
            }
        }
    }

    void V_Symmetric::add(const gnss_proc_lsq_equationmatrix &equ, bool phase)
    {
        if (!phase)
        {
            this->add(equ);
            return;
        }
        // cycle equ
        for (int num = 0; num < equ.num_equ(); num++)
        {
            if (equ.get_obscombtype(num).is_code())
            {
                continue;
            }
            // cycle par
            auto B_temp = equ.B[num];
            sort(B_temp.begin(), B_temp.end());
            for (int ipar = 0; ipar < B_temp.size(); ipar++)
            {
                auto row = B_temp[ipar];
                for (int jpar = 0; jpar <= ipar; jpar++)
                {
                    auto col = B_temp[jpar];
                    _element[row.first][col.first] += row.second * equ.P[num] * col.second;
                }
            }
        }
    }

    void V_Symmetric::addBackZero()
    {
        _element.push_back(vector<double>(_element.size() + 1, 0.0));
    }

    void V_Symmetric::remove(int idx)
    {
        for (int row = _element.size(); row > idx; row--)
        {
            _element[row - 1].erase(_element[row - 1].begin() + idx - 1);
        }
        _element.erase(_element.begin() + idx - 1);
    }

    void V_Symmetric::print()
    {
        std::cout << setw(20) << setprecision(5);
        for (auto Row = _element.begin(); Row != _element.end(); Row++)
        {
            for (auto Col = Row->begin(); Col != Row->end(); Col++)
            {
                std::cout << setw(20) << *Col;
            }
            std::cout << endl;
        }
    }

    Symmetric V_Symmetric::changeNewMat() const
    {
        Symmetric temp(_element.size());
        int row = 0, col = 0;
        for (auto Row = _element.begin(); Row != _element.end(); Row++)
        {
            col = 0;
            for (auto Col = Row->begin(); Col != Row->end(); Col++)
            {
                temp.set(*Col, row, col);
                col++;
            }
            row++;
        }
        return temp;
    }

    double V_Symmetric::center_value(int idx) const
    {
        return _element[idx - 1][idx - 1];
    }

    void V_Symmetric::swap(int a, int b)
    {
        double temp_a = num(a, a);
        double temp_b = num(b, b);
        double temp_ab = num(a, b);
        for (int i = 0; i < num(); i++)
        {
            double temp = num(a, i + 1);
            num(a, i + 1) = num(b, i + 1);
            num(b, i + 1) = temp;
        }
        num(a, a) = temp_b;
        num(b, b) = temp_a;
        num(a, b) = temp_ab;
    }

    int V_Vector::num() const
    {
        return _element.size();
    }

    double &V_Vector::num(int a)
    {
        return _element[a - 1];
    }

    void V_Vector::resize(int num)
    {
        _element.resize(num, 0.0);
    }

    void V_Vector::add(const gnss_proc_lsq_equationmatrix &equ)
    {
        for (int num = 0; num < equ.num_equ(); num++)
        {
            for (unsigned int ipar = 0; ipar < equ.B[num].size(); ipar++)
            {
                _element[equ.B[num][ipar].first] += equ.B[num][ipar].second * equ.P[num] * equ.l[num];
            }
        }
    }

    //void V_Vector::add_related(const gnss_proc_lsq_equationmatrix& equ)
    //{
    //    map< pair<string, string>, set<pair<FREQ_SEQ, FREQ_SEQ>>> freq_pair_list;
    //    map< pair<string, string>, set<pair<GOBSBAND, GOBSBAND>>> band_pair_list;
    //    map< pair<string, string>, map<FREQ_SEQ, GOBSBAND>>       band_list;
    //    map< pair<string, string>, map<GOBSBAND, FREQ_SEQ>>       freq_list;
    //    map<pair<string, string>, map<pair<FREQ_SEQ, FREQ_SEQ>, int>> site_sat_index_P;
    //    map<pair<string, string>, map<pair<FREQ_SEQ, FREQ_SEQ>, int>> site_sat_index_L;

    //    map<pair<string, string>, map<FREQ_SEQ, double>>              site_sat_sigma;

    //    const auto& site_sat_pair = equ.get_site_sat_pair();
    //    for (int i = 0; i < site_sat_pair.size();i++)
    //    {
    //        auto obs_type = equ.get_obscombtype(i);
    //        auto band_pair = obs_type.getBand_pair();
    //        auto freq_pair = obs_type.getFreq_pair();
    //        if (obs_type.is_phase()) site_sat_index_L[site_sat_pair[i]][freq_pair] = i;
    //        if (obs_type.is_code())  site_sat_index_P[site_sat_pair[i]][freq_pair] = i;
    //        band_pair_list[site_sat_pair[i]].insert(band_pair);
    //        freq_pair_list[site_sat_pair[i]].insert(freq_pair);
    //        freq_list[site_sat_pair[i]][band_pair.first]  = freq_pair.first;
    //        band_list[site_sat_pair[i]][freq_pair.first]  = band_pair.first;
    //        freq_list[site_sat_pair[i]][band_pair.second] = freq_pair.second;
    //        band_list[site_sat_pair[i]][freq_pair.second] = band_pair.second;

    //        if (equ.P_raw[i * 2 + 0][0].second != 0.0) site_sat_sigma[site_sat_pair[i]][freq_pair.first]  = 1.0 / equ.P_raw[i * 2 + 0][0].second;
    //        if (equ.P_raw[i * 2 + 1][1].second != 0.0) site_sat_sigma[site_sat_pair[i]][freq_pair.second] = 1.0 / equ.P_raw[i * 2 + 1][1].second;

    //    }

    //    // jdhuang :
    //    gnss_data_obs_manager gnss;
    //    for (const auto& site_sat : site_sat_pair)
    //    {
    //        gnss.sat(site_sat.second);
    //        double alpha_12 = 0.0; double beta_12 = 0.0;
    //        double alpha_13 = 0.0; double beta_13 = 0.0;
    //        double alpha_14 = 0.0; double beta_14 = 0.0;
    //        double alpha_15 = 0.0; double beta_15 = 0.0;
    //
    //        if (freq_pair_list[site_sat].find(std::make_pair(FREQ_1, FREQ_2)) != freq_pair_list[site_sat].end()) gnss.coef_ionofree(band_list[site_sat][FREQ_1], alpha_12, band_list[site_sat][FREQ_2], beta_12);
    //        if (freq_pair_list[site_sat].find(std::make_pair(FREQ_1, FREQ_3)) != freq_pair_list[site_sat].end()) gnss.coef_ionofree(band_list[site_sat][FREQ_1], alpha_13, band_list[site_sat][FREQ_3], beta_13);
    //        if (freq_pair_list[site_sat].find(std::make_pair(FREQ_1, FREQ_4)) != freq_pair_list[site_sat].end()) gnss.coef_ionofree(band_list[site_sat][FREQ_1], alpha_14, band_list[site_sat][FREQ_4], beta_14);
    //        if (freq_pair_list[site_sat].find(std::make_pair(FREQ_1, FREQ_5)) != freq_pair_list[site_sat].end()) gnss.coef_ionofree(band_list[site_sat][FREQ_1], alpha_15, band_list[site_sat][FREQ_5], beta_15);

    //        Matrix Sigma;
    //        if (freq_pair_list[site_sat].size() == 1)
    //        {
    //            Sigma.resize(1, 1);

    //            Sigma(1, 1) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_12 * beta_12* site_sat_sigma[site_sat][FREQ_2];
    //        }

    //        if (freq_pair_list[site_sat].size() == 2)
    //        {
    //            Sigma.resize(2, 2);

    //            Sigma(1, 1) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_12 * beta_12 * site_sat_sigma[site_sat][FREQ_2];
    //            Sigma(1, 2) = alpha_12 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(2, 1) = alpha_12 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(2, 2) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_13 * beta_13 * site_sat_sigma[site_sat][FREQ_3];
    //        }

    //        if (freq_pair_list[site_sat].size() == 3)
    //        {
    //            Sigma.resize(3, 3);

    //            Sigma(1, 1) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_12 * beta_12 * site_sat_sigma[site_sat][FREQ_2];
    //            Sigma(1, 2) = alpha_12 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(1, 3) = alpha_12 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];

    //            Sigma(2, 1) = alpha_13 * alpha_12 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(2, 2) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_13 * beta_13 * site_sat_sigma[site_sat][FREQ_3];
    //            Sigma(2, 3) = alpha_13 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];

    //            Sigma(3, 1) = alpha_14 * alpha_12 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(3, 2) = alpha_14 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(3, 3) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_14 * beta_14 * site_sat_sigma[site_sat][FREQ_4];

    //        }

    //        if (freq_pair_list[site_sat].size() == 4)
    //        {
    //            Sigma.resize(4, 4);

    //            Sigma(1, 1) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_12 * beta_12 * site_sat_sigma[site_sat][FREQ_2];
    //            Sigma(1, 2) = alpha_12 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(1, 3) = alpha_12 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(1, 4) = alpha_12 * alpha_15 * site_sat_sigma[site_sat][FREQ_1];

    //            Sigma(2, 1) = alpha_12 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(2, 2) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_13 * beta_13 * site_sat_sigma[site_sat][FREQ_3];
    //            Sigma(2, 3) = alpha_13 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(2, 4) = alpha_13 * alpha_15 * site_sat_sigma[site_sat][FREQ_1];

    //            Sigma(3, 1) = alpha_12 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(3, 2) = alpha_13 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(3, 3) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_14 * beta_14 * site_sat_sigma[site_sat][FREQ_4];
    //            Sigma(3, 4) = alpha_14 * alpha_15 * site_sat_sigma[site_sat][FREQ_1];

    //            Sigma(4, 1) = alpha_15 * alpha_12 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(4, 2) = alpha_15 * alpha_13 * site_sat_sigma[site_sat][FREQ_1];
    //            Sigma(4, 3) = alpha_12 * alpha_12 * site_sat_sigma[site_sat][FREQ_1] + beta_15 * beta_15 * site_sat_sigma[site_sat][FREQ_5];
    //            Sigma(4, 4) = alpha_15 * alpha_14 * site_sat_sigma[site_sat][FREQ_1];
    //        }

    //        Matrix SigmaP = Sigma.i();

    //        if (site_sat_index_P[site_sat].size() == 1)
    //        {
    //        }
    //
    //    }
    //    //if(freq_list.size() == 2)

    //    for (int num = 0; num < equ.num_equ(); num++)
    //    {
    //        for (int ipar = 0; ipar < equ.B[num].size(); ipar++)
    //        {
    //            _element[equ.B[num][ipar].first - 1] += equ.B[num][ipar].second * equ.P[num] * equ.l[num];
    //        }
    //    }
    //}

    void V_Vector::del(const gnss_proc_lsq_equationmatrix &equ)
    {
        for (int num = 0; num < equ.num_equ(); num++)
        {
            for (unsigned int ipar = 0; ipar < equ.B[num].size(); ipar++)
            {
                _element[equ.B[num][ipar].first] -= equ.B[num][ipar].second * equ.P[num] * equ.l[num];
            }
        }
    }

    void V_Vector::add(const gnss_proc_lsq_equationmatrix &equ, bool phase)
    {
        if (!phase)
        {
            this->add(equ);
            return;
        }
        for (unsigned int num = 0; num < equ.num_equ(); num++)
        {
            if (equ.get_obscombtype(num).is_code())
            {
                continue;
            }
            for (unsigned int ipar = 0; ipar < equ.B[num].size(); ipar++)
            {
                _element[equ.B[num][ipar].first] += equ.B[num][ipar].second * equ.P[num] * equ.l[num];
            }
        }
    }

    void V_Vector::addBackZero()
    {
        _element.push_back(0.0);
    }

    void V_Vector::remove(int idx)
    {
        _element.erase(_element.begin() + idx - 1);
    }

    void V_Vector::print()
    {
        std::cout << setprecision(5);
        for (auto row = _element.begin(); row != _element.end(); row++)
        {
            std::cout << setw(20) << *row << endl;
        }
    }

    Vector V_Vector::changeNewMat()
    {
        Vector temp(_element.size());
        int row = 1;
        for (auto Row = _element.begin(); Row != _element.end(); Row++)
        {
            temp(row) = *Row;
            row++;
        }
        return temp;
    }

    void V_Vector::swap(int a, int b)
    {
        double temp;
        temp = _element[a - 1];
        _element[a - 1] = _element[b - 1];
        _element[b - 1] = temp;
    }

    bool remove_lsqmatrix(int idx, V_Symmetric &NEQ, V_Vector &W)
    {
        if (NEQ.num() != W.num())
            return false;
        if (idx < 1 || idx > NEQ.num())
            return false;

        double center_value = NEQ.center_value(idx);

        if (!double_eq(center_value, 0.0))
        {
            int NEQ_num = NEQ.num();

            // record need remove element
            vector<double> remove_element;
            remove_element.reserve(NEQ_num);

            double w_remove = 0.0;
            vector<int> remove_rows;
            for (int col = 1; col <= NEQ_num; col++)
            {
                remove_element.push_back(NEQ.num(idx, col));
                if (col == idx || double_eq(NEQ.num(idx, col), 0.0))
                {
                    continue;
                }
                remove_rows.push_back(col - 1);
            }

            w_remove = W._element[idx - 1];

            // remove the par in NEQ and W by for cycle
            int remove_rows_size = remove_rows.size();

#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < remove_rows_size; ++i)
            {
                int row = remove_rows[i];
                double temp_coeff = -remove_element[row] / center_value;
                auto &NEQ_row = NEQ._element[row];
                for (int col = 0; col <= row; col++)
                {
                    NEQ_row[col] += remove_element[col] * temp_coeff;
                }

                W._element[row] += w_remove * temp_coeff;
            }
#elif USE_OPENBLAS
            for (int i = 0; i < remove_rows_size; i++)
            {
                int row = remove_rows[i];
                double temp_coeff = -remove_element[row] / center_value;
                cblas_daxpy(row + 1, temp_coeff, &remove_element[0], 1, &NEQ._element[row][0], 1);
            }
            cblas_daxpy(NEQ_num, -w_remove / center_value, &remove_element[0], 1, &W._element[0], 1);
#else
            for (int i = 0; i < remove_rows_size; ++i)
            {
                int row = remove_rows[i];
                double temp_coeff = -remove_element[row] / center_value;
                for (int col = 0; col <= row; col++)
                {
                    NEQ._element[row][col] += remove_element[col] * temp_coeff;
                }

                W._element[row] += w_remove * temp_coeff;
            }
#endif
        }

        NEQ.remove(idx);
        W.remove(idx);

        return true;
    }
    bool rearrange_lsqmatrix(const vector<int> &remove_idx, base_allpar &allpar, V_Symmetric &NEQ, V_Vector &W, Matrix &N11, Matrix &N21, Matrix &N22, Vector &W1, Vector &W2, bool idx_from_zero)
    {

        auto beg_t = chrono::high_resolution_clock::now();
        if (remove_idx.size() == 0)
        {
            return true;
        }
        int rows = NEQ.num();
        int rows_new = rows - remove_idx.size();
        vector<bool> idx_map(rows, false);
        set<int> zero_idx;
        for (int i = 0; i < remove_idx.size(); i++)
        {
            idx_map[remove_idx[i] - 1] = true;
            if (NEQ.center_value(remove_idx[i]) == 0.0)
            {
                zero_idx.insert(remove_idx[i] - 1);
            }
        }

        int n22 = remove_idx.size() - zero_idx.size();

        // init size
        N11 = Matrix::Zero(rows_new, rows_new);
        N21 = Matrix::Zero(n22, rows_new);
        N22 = Matrix::Zero(n22, n22);

        W1 = Vector::Zero(rows_new);
        W2 = Vector::Zero(n22);

        auto end_t = chrono::high_resolution_clock::now();
        std::cout << "Init Matrix Spend time is " << chrono::duration_cast<chrono::milliseconds>(end_t - beg_t).count() / 1000.0 << " sec " << endl;

        // rearrange
        int i_skip = 0;
        int i_not_skip = 0;
        for (int i = 0; i < rows; i++)
        {
            if (zero_idx.count(i) != 0)
            {
                continue;
            }
            int j_skip = 0;
            int j_not_skip = 0;
            if (idx_map[i] == true)
            {
                i_skip++;
                W2(i_skip - 1) = W._element[i];
                for (int j = 0; j <= i; j++)
                {
                    if (zero_idx.count(j) != 0)
                    {
                        continue;
                    }
                    if (idx_map[j] == true)
                    {
                        j_skip++;
                        N22(i_skip - 1, j_skip - 1) = NEQ._element[i][j];
                        N22(j_skip - 1, i_skip - 1) = NEQ._element[i][j];
                    }
                    else
                    {
                        j_not_skip++;
                        N21(i_skip - 1, j_not_skip - 1) = NEQ._element[i][j];
                    }
                }
            }
            else
            {
                i_not_skip++;
                W1(i_not_skip - 1) = W._element[i];
                for (int j = 0; j <= i; j++)
                {
                    if (zero_idx.count(j) != 0)
                    {
                        continue;
                    }

                    if (idx_map[j] == true)
                    {
                        j_skip++;
                        N21(j_skip - 1, i_not_skip - 1) = NEQ._element[i][j];
                    }
                    else
                    {
                        j_not_skip++;
                        N11(i_not_skip - 1, j_not_skip - 1) = NEQ._element[i][j];
                        N11(j_not_skip - 1, i_not_skip - 1) = NEQ._element[i][j];
                    }
                }
            }
        }
        return true;
    }
    int rearrange_lsqmatrix(vector<int> &remove_idx, V_Symmetric &NEQ, V_Vector &W, base_allpar &allpar)
    {
        if (remove_idx.size() == 0)
        {
            return true;
        }

        int rows = NEQ.num();
        // int rows_new = rows - remove_idx.size();

        vector<bool> idx_map(rows, false);
        set<int> zero_idx;
        int remove_size = remove_idx.size();
        for (int i = 0; i < remove_size; i++)
        {
            idx_map[remove_idx[i] - 1] = true;
            if (NEQ.center_value(remove_idx[i]) == 0.0)
            {
                zero_idx.insert(remove_idx[i] - 1);
            }
        }

        // rearrange paramter and init map_idx
        base_allpar allpar_orig = allpar;
        allpar.delAllParam();
        vector<int> map_idx(rows);  //map from old -> new
        vector<int> remove_idx_new; // from new->old (size is remove size)
        // add not remove parameter
        int count = 0;
        for (int i = 0; i < rows; i++)
        {
            if (idx_map[i])
            {
                continue;
            }
            allpar.addParam(allpar_orig[i]);
            map_idx[i] = count++;
        }
        // add remove paramter (but center value not zero)
        for (int i = 0; i < remove_size; i++)
        {
            if (zero_idx.count(remove_idx[i] - 1) != 0)
            {
                continue;
            }
            allpar.addParam(allpar_orig[remove_idx[i] - 1]);
            map_idx[remove_idx[i] - 1] = count++;
            remove_idx_new.push_back(remove_idx[i]);
        }
        // add remove parameter (but center value is zero)
        for (auto i : zero_idx)
        {
            allpar.addParam(allpar_orig[i]);
            map_idx[i] = count++;
            remove_idx_new.push_back(i + 1);
        }

        assert(rows == count);

        // rearrange remove_idx
        assert(remove_idx.size() == remove_idx_new.size());
        remove_idx = remove_idx_new;

        V_Symmetric NEQ_orig = NEQ;
        V_Vector W_orig = W;

        // rearrange

#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < rows; i++)
        {
            int x = map_idx[i];
            for (int j = 0; j <= i; j++)
            {
                int y = map_idx[j];
                (x > y ? NEQ._element[x][y] : NEQ._element[y][x]) = NEQ_orig._element[i][j];
                //NEQ.num(map_idx[i]+1,map_idx[j]+1) = NEQ_orig._element[i][j];
            }
            W._element[x] = W_orig._element[i];
        }
        return zero_idx.size();
    }

}