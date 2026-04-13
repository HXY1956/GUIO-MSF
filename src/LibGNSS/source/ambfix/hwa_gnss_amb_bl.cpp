#include "hwa_set_amb.h"
#include "hwa_gnss_amb_blrec.h"
#include "hwa_set_rec.h"
#include "hwa_gnss_amb_bl.h"

namespace hwa_gnss
{
    gnss_amb_baseline::gnss_amb_baseline()
    {
    }

    gnss_amb_baseline::gnss_amb_baseline(const std::string &obj1, const std::string &obj2, const double &distance = 0.0, const int &nsys = 1)
    {
        this->obj1 = obj1;
        this->obj2 = obj2;
        this->nsys = nsys;
        this->distance = distance;
    }

    gnss_amb_baseline::~gnss_amb_baseline()
    {
    }

    std::pair<std::string, std::string> gnss_amb_baseline::bl_pair() const
    {
        return std::make_pair(this->obj1, this->obj2);
    }

    bool gnss_amb_baseline::isSame(const std::string &obj_1st, const std::string &obj_2nd)
    {
        if (obj_1st == obj1 && obj_2nd == obj2)
            return true;
        return false;
    }

}
