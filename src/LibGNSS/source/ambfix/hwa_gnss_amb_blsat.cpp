#include "hwa_set_amb.h"
#include "hwa_gnss_amb_BLSAT.h"
#include "hwa_set_rec.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_amb_baseline_sat::gnss_amb_baseline_sat()
    {
    }

    hwa_gnss::gnss_amb_baseline_sat::gnss_amb_baseline_sat(std::set<std::string> &sats)
    {
        for (auto iter : sats)
        {
            auto for_iter = ++sats.find(iter);
            while (for_iter != sats.end())
            {
                gnss_amb_baseline tmp(iter, *for_iter, 1, 0.0);
                _baselines.push_back(tmp);
                for_iter++;
            }
        }

#ifdef DEBUG_AMBFIX
        int base_num = 0;
        std::cout << " ============> The baselines of sats ================" << std::endl;
        for (auto iter : _baselines)
        {
            base_num++;
            std::cout << std::setw(8) << base_num
                 << std::setw(4) << iter.nsys
                 << std::setw(5) << iter.obj1
                 << std::setw(5) << iter.obj2
                 << std::setw(16) << std::setprecision(8) << iter.distance
                 << std::endl;
        }
#endif
    }

    hwa_gnss::gnss_amb_baseline_sat::~gnss_amb_baseline_sat()
    {
    }

    void gnss_amb_baseline_sat::add_sat(const std::string &sat)
    {
        if (std::find(_sats.begin(), _sats.end(), sat) == std::end(_sats))
        {
            _sats.push_back(sat);
            std::sort(_sats.begin(), _sats.end());
        }
    }

    int hwa_gnss::gnss_amb_baseline_sat::baseline_num()
    {
        return _baselines.size();
    }

    std::vector<std::pair<std::string, std::string>> hwa_gnss::gnss_amb_baseline_sat::baselines()
    {
        std::vector<std::pair<std::string, std::string>> tmp;
        if (_baselines.empty())
        {
            if (_sats.empty())
                return tmp;
            for (int i = 0; i < _sats.size(); i++)
            {
                for (int j = i + 1; j < _sats.size(); j++)
                {
                    if (_sats[i].substr(0, 1) != _sats[j].substr(0, 1))
                        continue;
                    gnss_amb_baseline line(_sats[i], _sats[j], 0, 1);
                    _baselines.push_back(line);
                }
            }
        }

        for (const auto &iter : _baselines)
        {
            tmp.push_back(iter.bl_pair());
        }

        // std::cout the base lines
        //for (const auto& iter : tmp)
        //{
        //    std::cout << "The baseline is " + iter.first + "   " + iter.second << std::endl;
        //}
        return tmp;
    }

    std::vector<std::tuple<std::string, std::string, std::string, std::string>> gnss_amb_baseline_sat::get_depend_Dd_baseline(std::vector<std::tuple<std::string, std::string, std::string, std::string>> &baselines)
    {

        std::vector<std::tuple<std::string, std::string, std::string, std::string>> result;

        // get all the objs
        std::map<std::string, int> index;
        int count = 0;
        for (auto &it : baselines)
        {
            std::string tmp_1st = std::get<0>(it);
            std::string tmp_2nd = std::get<1>(it);
            std::string tmp_3rd = std::get<2>(it);
            std::string tmp_4th = std::get<3>(it);

            if (index.count(tmp_1st) <= 0)
            {
                count++;
                index[tmp_1st] = count;
            }

            if (index.count(tmp_2nd) <= 0)
            {
                count++;
                index[tmp_2nd] = count;
            }

            if (index.count(tmp_3rd) <= 0)
            {
                count++;
                index[tmp_3rd] = count;
            }

            if (index.count(tmp_4th) <= 0)
            {
                count++;
                index[tmp_4th] = count;
            }
        }

        // check the depend
        Matrix vector_space;
        int ncol = index.size();
        for (auto &it : baselines)
        {
            std::string tmp_1st = std::get<0>(it);
            std::string tmp_2nd = std::get<1>(it);
            std::string tmp_3rd = std::get<2>(it);
            std::string tmp_4th = std::get<3>(it);

            auto tmp1 = index.count(tmp_1st);
            auto tmp2 = index.count(tmp_2nd);
            auto tmp3 = index.count(tmp_3rd);
            auto tmp4 = index.count(tmp_4th);
            int index1 = index[tmp_1st];
            int index2 = index[tmp_2nd];
            int index3 = index[tmp_3rd];
            int index4 = index[tmp_4th];

            if (tmp1 <= 0 || tmp2 <= 0 || tmp3 <= 0 || tmp4 <= 0)
                continue;

            if (vector_space.isZero())
            {
                vector_space.resize(1, ncol);
                vector_space.setZero();
                vector_space(0, index1 - 1) = 1;
                vector_space(0, index2 - 1) = -1;
                vector_space(0, index3 - 1) = -1;
                vector_space(0, index4 - 1) = 1;
                result.push_back(it);
            }

            RowVector new_vector(ncol);
            new_vector.setZero();
            vector_space(0, index1 - 1) = 1;
            vector_space(0, index2 - 1) = -1;
            vector_space(0, index3 - 1) = -1;
            vector_space(0, index4 - 1) = 1;
            new_vector(index1 - 1) = 1;
            new_vector(index2 - 1) = -1;
            new_vector(index3 - 1) = -1;
            new_vector(index4 - 1) = 1;

            //TODO COMMENT
            Matrix A = Vstack(vector_space,new_vector);
            Diag D;
            if (A.rows() < A.cols())
            {
                SVD(A.transpose(), D.matrixW());
            }
            else
            {
                SVD(A, D.matrixW());
            }
            bool isDepend = (fabs(D(D.rows() - 1, D.cols() - 1)) > 1.0e-2);

            if (A.rows() != (vector_space.rows() + new_vector.rows()))
                continue;
            if (isDepend)
            {
                //std::cout << D << std::endl;
                vector_space = A;
                result.push_back(it);
            }
        }

        return result;
    }
}
