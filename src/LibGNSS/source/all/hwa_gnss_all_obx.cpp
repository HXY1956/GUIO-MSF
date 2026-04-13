#include "hwa_gnss_all_obx.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_all_obx::gnss_all_obx()
    {
    }

    gnss_all_obx::gnss_all_obx(base_log spdlog)
    {
        this->_spdlog = spdlog;
    }

    hwa_gnss::gnss_all_obx::~gnss_all_obx()
    {
    }

    void gnss_all_obx::addObx(const base_time & epoch, const gnss_data_obx_epoch & obx_epoch)
    {
        if (epoch < this->_beg_epo)
        {
            this->_beg_epo = epoch;
        }

        if (epoch > this->_end_epo)
        {
            this->_end_epo = epoch;
        }


        this->_allobx[epoch] = obx_epoch;
    }

    const base_time & gnss_all_obx::getBegEpo() const
    {
        return this->_beg_epo;
        // TODO: 在此处插入 return 语句
    }

    const base_time & gnss_all_obx::getEndEpo() const
    {
        return this->_end_epo;
        // TODO: 在此处插入 return 语句
    }


    Matrix gnss_all_obx::getRotMat(const std::string & prn, const base_time & epoch)
    {
        Matrix rotMat(3, 3);
        auto obx_beg = this->getObxLowerRecord(prn, epoch);
        auto obx_end = this->getObxUpperRecord(prn, epoch);

        gnss_data_obx_record obx_now;
        slerp(epoch, obx_beg.epo, obx_beg, obx_end.epo, obx_end, obx_now);

        if (obx_now.isValid)
        {
            rotMat = Quat2RotMat(obx_now);
        }

        return rotMat;
    }

    const gnss_all_obx::hwa_map_time_obx & gnss_all_obx::getObxData() const
    {
        return _allobx;
        // TODO: 在此处插入 return 语句
    }

    const gnss_data_obx_epoch & gnss_all_obx::getObxEpoch(const base_time & epoch) const
    {
        if (_allobx.find(epoch) != _allobx.end())
        {
            return _allobx.at(epoch);
        }
        else
        {
            return _empty_epoch;
        }
        // TODO: 在此处插入 return 语句
    }

    const gnss_data_obx_record & gnss_all_obx::getObxUpperRecord(const std::string & prn, const base_time & epoch) const
    {
        auto upper_obx = _allobx.lower_bound(epoch);

        if (upper_obx != _allobx.end() && upper_obx->second.obx_data.find(prn) != upper_obx->second.obx_data.end())
        {
            return upper_obx->second.getObxRecord(prn);
        }
        else
        {
            return _empty_record;
        }
        // TODO: 在此处插入 return 语句
    }

    const gnss_data_obx_record & gnss_all_obx::getObxLowerRecord(const std::string & prn, const base_time & epoch) const
    {
        auto upper_obx = _allobx.lower_bound(epoch);
        if ((upper_obx--) == _allobx.begin())
        {
            upper_obx = _allobx.begin();
        }

        if (upper_obx != _allobx.end() && upper_obx->second.obx_data.find(prn) != upper_obx->second.obx_data.end())
        {
            return upper_obx->second.getObxRecord(prn);
        }
        else
        {
            return _empty_record;
        }
        // TODO: 在此处插入 return 语句    }
    }

    Matrix gnss_all_obx::Quat2RotMat(const gnss_data_obx_record & obx_record)
    {
        Matrix xmat(3, 3);
        double q00, q11, q22, q33, q01, q02, q03, q12, q13, q23 = 0.0;
        double q0 = obx_record.q0;
        double q1 = obx_record.q1;
        double q2 = obx_record.q2;
        double q3 = obx_record.q3;


        q00 = q0 * q0; q11 = q1 * q1; q22 = q2 * q2; q33 = q3 * q3;
        q01 = q0 * q1; q02 = q0 * q2; q03 = q0 * q3;
        q12 = q1 * q2; q13 = q1 * q3;
        q23 = q2 * q3;
        xmat(0, 0) = q00 + q11 - q22 - q33;
        xmat(1, 0) = 2.0*(q12 - q03);
        xmat(2, 0) = 2.0*(q13 + q02);
        xmat(0, 1) = 2.0*(q12 + q03);
        xmat(1, 1) = q00 - q11 + q22 - q33;
        xmat(2, 1) = 2.0*(q23 - q01);
        xmat(0, 2) = 2.0*(q13 - q02);
        xmat(1, 2) = 2.0*(q23 + q01);
        xmat(2, 2) = q00 - q11 - q22 + q33;

        return xmat;
    }

    bool gnss_all_obx::slerp(const base_time & epo, 
        const base_time & beg, const gnss_data_obx_record & beg_obx, 
        const base_time & end, const gnss_data_obx_record & end_obx, 
        gnss_data_obx_record& now_obx)
    {
        if (epo < beg || epo > end || !beg_obx.isValid || !end_obx.isValid || (end - beg) > 60.0)
        {
            now_obx.isValid = false;
            return false;
        }

        double qn[4] = { 0.0,0.0,0.0,0.0 };
        double qs[4] = { beg_obx.q0, beg_obx.q1, beg_obx.q2, beg_obx.q3 };
        double qe[4] = { end_obx.q0, end_obx.q1, end_obx.q2, end_obx.q3 };
        double t = (epo - beg) / (end - beg);
        double k0 = 0.0, k1 = 0.0;
        double cosa = qs[0]*qe[0] + qs[1]*qe[1] + qs[2]*qe[2] + qs[3]*qe[3];
        if (cosa < 0.0)
        {
            cosa = -cosa;
            for (int i = 0; i < 4; i++)
            {
                qe[i] = -qe[i];
            }
        }

        if (cosa > 0.9995)
        {
            k0 = 1.0 - t;
            k1 = t;
        }
        else
        {
            double sina = sqrt(1.0 - cosa * cosa);
            double a = atan2(sina, cosa);
            k0 = sin((1.0 - t)*a) / sina;
            k1 = sin(t*a) / sina;
        }

        for (int i = 0; i < 4; i++)
        {
            qn[i] = qs[i] * k0 + qe[i] * k1;
        }

        now_obx = beg_obx;
        now_obx.q0 = qn[0];
        now_obx.q1 = qn[1];
        now_obx.q2 = qn[2];
        now_obx.q3 = qn[3];
        now_obx.epo = epo;
        now_obx.isValid = true;

        return true;
    }

}
