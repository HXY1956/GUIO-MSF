#include "hwa_base_string.h"
#include "hwa_gnss_amb_blrec.h"
#include "hwa_gnss_amb_bl.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_set_amb.h"
#include "hwa_set_leo.h"
#include "hwa_set_rec.h"
#include "Eigen/Dense"

namespace hwa_gnss
{
    hwa_gnss::gnss_amb_baseline_rec::gnss_amb_baseline_rec()
    {
    }

    gnss_amb_baseline_rec::gnss_amb_baseline_rec(const base_time &cut_time, double cut_length)
    {
        _cut_length = cut_length;
        _cut_time = cut_time;
    }

    hwa_gnss::gnss_amb_baseline_rec::~gnss_amb_baseline_rec()
    {
    }

    void gnss_amb_baseline_rec::add_cut(const base_time &cut_time, double cut_length)
    {
        _cut_length = cut_length;
        _cut_time = cut_time;
    }

    void gnss_amb_baseline_rec::add_gobj(gnss_all_obj *obj)
    {
        this->_gobj = obj;
    }

    void gnss_amb_baseline_rec::add_gset(set_base *set)
    {
        // std::set the setting pointer
        if (nullptr == set)
        {
            spdlog::critical("your std::set pointer is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _gset = set;
        }

        _beg_time = dynamic_cast<set_gen *>(set)->beg();
        _end_time = dynamic_cast<set_gen *>(set)->end();
    }

    void gnss_amb_baseline_rec::add_glog(base_log spdlog)
    {
        // std::set spdlog
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

    void gnss_amb_baseline_rec::add_rec(const std::string &rec, const std::string &sat)
    {
        if (std::find(_recs.begin(), _recs.end(), rec) == std::end(_recs))
        {
            _recs.push_back(rec);
            std::sort(_recs.begin(), _recs.end());
        }

        if (_sys[rec].count(sat.substr(0, 1)) < 1)
            _sys[rec].insert(sat.substr(0, 1));
    }

    void gnss_amb_baseline_rec::add_rec(const std::string &rec, const std::string &sat, Triple crd)
    {
        add_rec(rec, sat);
        if (!crd.isZero() && _xyz.find(rec) == _xyz.end())
            _xyz.emplace(rec, crd);
    }

    std::vector<std::pair<std::string, std::string>> gnss_amb_baseline_rec::baselines()
    {
        if (_spdlog)
            SPDLOG_LOGGER_INFO(_spdlog, "get the baselines for recs ");

        std::vector<std::pair<std::string, std::string>> tmp;

        if (_baselines.empty())
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "get rec xyz  ");
            this->_get_rec_xyz();

            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "create_baseline  ");

            this->_create_baseline();
        }

        for (const auto &iter : _baselines)
        {
            tmp.push_back(iter.bl_pair());
        }

        // std::cout the base line
        for (const auto &iter : tmp)
        {
            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "The baseline is " + iter.first + "   " + iter.second);
        }

        return tmp;
    }

    bool hwa_gnss::gnss_amb_baseline_rec::_check_amb_depend()
    {
        // rec std::map
        std::map<std::string, int> _obj_index;
        for (int i = 0; i < _recs.size(); i++)
        {
            _obj_index[_recs[i]] = i + 1;
        }

        // judge
        std::vector<gnss_amb_baseline> Independent_baseline;
        Matrix vector_space;
        for (const auto &iter : _baselines)
        {
            const auto &line = iter.bl_pair();
            const auto &tmp1 = _obj_index.count(line.first);
            const auto &tmp2 = _obj_index.count(line.second);
            if (tmp1 <= 0 || tmp2 <= 0)
                continue;

            int ncol = _obj_index.size();
            if (vector_space.isZero())
            {
                vector_space.resize(1, ncol);
                vector_space.setZero();
                int index1 = _obj_index[line.first];
                int index2 = _obj_index[line.second];
                vector_space(0, index1 - 1) = 1;
                vector_space(0, index2 - 1) = -1;

                Independent_baseline.push_back(iter);
                continue;
            }

            RowVector new_vector(ncol);
            new_vector.setZero();
            int index1 = _obj_index[line.first];
            int index2 = _obj_index[line.second];
            new_vector(index1 - 1) = 1;
            new_vector(index2 - 1) = -1;

            Matrix A = Vstack(vector_space, new_vector);
            if (A.rows() != (vector_space.rows() + new_vector.rows()))
                return false;
            Diag D;
            if (A.rows() < A.cols())
            {
                SVD(A.transpose(), D.matrixW());
            }
            else
            {
                SVD(A, D.matrixW());
            }

            bool isDepend = (fabs(D(D.rows() - 1, D.cols() - 1)) > 0.01);
            if (isDepend)
            {
                vector_space = A;
                Independent_baseline.push_back(iter);
            }
        }

        _baselines = Independent_baseline;

        return true;
    }

    bool hwa_gnss::gnss_amb_baseline_rec::_get_rec_xyz()
    {
        if (!_gobj && !_gset)
            return false;
        for (const auto &crt : _recs)
        {
            bool isfind = false;
            const std::string &rec = str2upper(crt);

            if (_xyz.find(crt) != _xyz.end())
                continue;
            if (this->_gset)
            {
                Triple tmp = dynamic_cast<set_rec *>(_gset)->get_crd_xyz(rec);
                if (!tmp.isZero())
                {
                    std::cout << rec << "  find xyz in xml file  " << std::endl;
                    this->_xyz[rec] = tmp;
                    isfind = true;
                }
            }

            if (isfind == false && this->_gobj)
            {
                std::shared_ptr<gnss_data_obj> grec = this->_gobj->obj(rec);
                if (grec)
                {
                    base_time tmp = _cut_time;
                    while (tmp <= _end_time)
                    {
                        //Triple rec_crd = grec->crd(_cut_time);
                        Triple rec_crd;
                        Triple rec_std;
                        grec->get_recent_crd(_cut_time, 100, rec_crd, rec_std);
                        if (!rec_crd.isZero())
                        {
                            std::cout << rec << "  find xyz in snx file  " << std::endl;
                            this->_xyz[rec] = rec_crd;
                            isfind = true;
                        }

                        if (isfind)
                            break;
                        tmp = tmp + 300;
                    }
                }
            }
        }
        return true;
    }

    bool gnss_amb_baseline_rec::_create_baseline()
    {
        if (_recs.empty())
        {
            std::cout << "  rec list is empty, return false  " << std::endl;
            return false;
        }

        // const auto &loop_1st = _recs;
        // const auto &loop_2nd = _recs;

        // int index = 0;
        std::vector<std::string> tmp;
        for (const auto &iter : _recs)
        {
            if (!IS_SUPPORTED_LEO(strs2strl(iter)) && this->_xyz.count(iter) == 0)
                continue;
            tmp.push_back(iter);
        }
        _recs = tmp;

#ifdef DEBUG_AMBFIX
        std::cout << std::endl;
        std::cout << " ============> The baselines of recs(before sort by distance and check dependence) ================" << std::endl;
        for (auto crt_obj = objs.begin(); crt_obj != objs.end(); crt_obj++)
        {
            std::cout << std::setw(8) << crt_obj->first
                 << std::setw(8) << crt_obj->second
                 << std::endl;
        }
        std::cout << std::endl;
#endif

        for (int i = 0; i < _recs.size(); i++)
        {
            const std::string &rec_1st = _recs[i];
            if (_sys[rec_1st].empty())
            {
                std::cout << "  no system for " + rec_1st + " skiped" << std::endl;
                continue;
            }
            for (int j = i + 1; j < _recs.size(); j++)
            {
                const std::string &rec_2nd = _recs[j];
                if (_sys[rec_2nd].empty())
                    continue;
                Triple first = this->_xyz[rec_1st];
                Triple second = this->_xyz[rec_2nd];
                Triple D_value = first - second;
                double distance = D_value.norm() / 1000.0;
                if (IS_SUPPORTED_LEO(strs2strl(rec_1st)) || IS_SUPPORTED_LEO(strs2strl(rec_2nd)))
                    distance = 1;
                if (distance > fabs(_cut_length))
                    continue;
                int nsys = 0;
                for (const auto &iter : _sys[rec_1st])
                {
                    if (_sys[rec_2nd].find(iter) != std::end(_sys[rec_2nd]))
                        nsys++;
                }
                if (nsys == 0)
                    continue;

                gnss_amb_baseline bl(rec_1st, rec_2nd, distance, nsys);
                _baselines.push_back(bl);
            }
        }

        if (_baselines.empty())
            return false;

#ifdef DEBUG_AMBFIX
        int tmp1 = 0;
        std::cout << std::endl;
        std::cout << " ============> The baselines of recs(before sort by distance and check dependence) ================" << std::endl;
        for (auto iter : sort_bl)
        {
            tmp1++;
            std::cout << std::setw(8) << tmp1
                 << std::setw(4) << iter.nsys
                 << std::setw(5) << iter.obj1
                 << std::setw(5) << iter.obj2
                 << std::setw(16) << std::setprecision(8) << iter.distance
                 << std::endl;
        }
        std::cout << std::endl;
#endif

        std::sort(_baselines.begin(), _baselines.end(), compare);
        _check_amb_depend();
#ifdef DEBUG_AMBFIX
        int tmp2 = 0;
        std::cout << std::endl;
        std::cout << " ============> The baselines of recs(after sort by distance) ================" << std::endl;
        for (auto iter : sort_bl)
        {
            tmp2++;
            std::cout << " Rec BaseLines : "
                 << std::setw(8) << tmp2
                 << std::setw(4) << iter.nsys
                 << std::setw(5) << iter.obj1
                 << std::setw(5) << iter.obj2
                 << std::setw(16) << std::setprecision(8) << iter.distance
                 << std::endl;
        }
        std::cout << std::endl;
#endif

#ifdef DEBUG_AMBFIX
        int tmp3 = 0;
        std::cout << std::endl;
        std::cout << " ============> The baselines of recs(after _check_amb_depend) ================" << std::endl;
        for (auto iter : this->_baselines)
        {
            tmp3++;
            std::cout << " Rec BaseLines : "
                 << std::setw(8) << tmp3
                 << std::setw(4) << iter.nsys
                 << std::setw(5) << iter.obj1
                 << std::setw(5) << iter.obj2
                 << std::setw(16) << std::setprecision(8) << iter.distance
                 << std::endl;
        }
        std::cout << std::endl;
#endif
        return true;
    }

    bool compare(const gnss_amb_baseline &bl1, const gnss_amb_baseline &bl2)
    {
        if (bl1.nsys < bl2.nsys)
            return false;
        return (bl1.nsys > bl2.nsys || bl1.distance < bl2.distance);
    }

}
