#include "hwa_gnss_data_obx.h"

using namespace std;

namespace hwa_gnss
{
    hwa_gnss::gnss_data_obx_record::gnss_data_obx_record()
    {
    }

    gnss_data_obx_record::gnss_data_obx_record(const base_time & epo, const std::string& id, const Matrix & rotLeft)
    {
        // get q
        double q0 = 0.5 * sqrt(1 + rotLeft(0, 0) + rotLeft(1, 1) + rotLeft(2, 2));
        double q1 = 0.5 * sqrt(1 + rotLeft(0, 0) - rotLeft(1, 1) - rotLeft(2, 2));
        double q2 = 0.5 * sqrt(1 + rotLeft(0, 0) + rotLeft(1, 1) - rotLeft(2, 2));
        double q3 = 0.5 * sqrt(1 - rotLeft(0, 0) - rotLeft(1, 1) + rotLeft(2, 2));
        
        this->epo = epo;
        //q0
        this->q0 = q0 * (+1.0);
        //q1
        if ((rotLeft(1, 2) - rotLeft(2, 1) < 0))
        {
            this->q1 = q1 * (-1.0);
        }
        else
        {
            this->q1 = q1 * (+1.0);
        }
        //q2
        if ((rotLeft(2, 0) - rotLeft(0, 2) < 0))
        {
            this->q2 = q2 * (-1.0);
        }
        else
        {
            this->q2 = q2 * (+1.0);
        }
        //q3
        if ((rotLeft(0, 1) - rotLeft(1, 0) < 0))
        {
            this->q3 = q3 * (-1.0);
        }
        else
        {
            this->q3 = q3 * (+1.0);
        }

        this->isValid = true;
        this->num = 4;
    }

    hwa_gnss::gnss_data_obx_record::gnss_data_obx_record(const base_time& epo, const std::string & rec, const std::string & id, const int & num, const double & q0, const double & q1, const double & q2, const double & q3)
    {
        this->epo = epo;
        this->rec = rec;
        this->id = id;
        this->num = num;
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;

        isValid = true;
    }

    hwa_gnss::gnss_data_obx_record::~gnss_data_obx_record()
    {
    }

    hwa_gnss::gnss_data_obx_epoch::gnss_data_obx_epoch()
    {
    }

    hwa_gnss::gnss_data_obx_epoch::~gnss_data_obx_epoch()
    {
    }

    bool hwa_gnss::gnss_data_obx_epoch::addTime(const base_time & epoch)
    {
        this->_epoch = epoch;
        return true;
    }

    const base_time & hwa_gnss::gnss_data_obx_epoch::getTime() const
    {
        return this->_epoch;
    }

    bool hwa_gnss::gnss_data_obx_epoch::addObxRecord(const gnss_data_obx_record & obx_record)
    {
        if (this->obx_data.find(obx_record.id) != this->obx_data.end())
        {
            this->obx_data[obx_record.id] = obx_record;
            _sat_num = this->obx_data.size();
            return true;
        }
        else
        {
            return false;
        }

    }

    const gnss_data_obx_record & hwa_gnss::gnss_data_obx_epoch::getObxRecord(const std::string & prn) const
    {
        return obx_data.at(prn);
    }

    const int & gnss_data_obx_epoch::getSatNum() const
    {
        return _sat_num;
    }

}
