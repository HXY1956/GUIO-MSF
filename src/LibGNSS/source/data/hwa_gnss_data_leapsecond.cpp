#include "hwa_gnss_data_leapsecond.h"

namespace hwa_gnss
{

    /** @brief constructor. */
    gnss_data_leapsecond::gnss_data_leapsecond() : base_data()
    {

        id_type(base_data::LEAPSECOND);
    }

    gnss_data_leapsecond::gnss_data_leapsecond(base_log spdlog) : base_data(spdlog)
    {

        id_type(base_data::LEAPSECOND);
    }

    /** @brief destructor. */
    gnss_data_leapsecond::~gnss_data_leapsecond() {}

    /**
    * @brief add leapsecond data to std::map.
    * @param[in]   mjd   mjd of the leapsecon data
    * @param[in]   leap  leapsecon data
    */
    void gnss_data_leapsecond::add_data(int mjd, int leap)
    {
        _leapseconds[mjd] = leap;
    }

    /** @brief whether the leapsecond data is empty.
    * @return  bool
    *    @retval   true     leapsecond data is empty
    *   @retval   false    leapsecond data is existent
    */
    bool gnss_data_leapsecond::is_empty()
    {
        if (_leapseconds.size() == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
} //namespace