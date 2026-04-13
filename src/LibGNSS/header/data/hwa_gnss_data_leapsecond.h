/**
 * @file        gleapsecond.h
 * @brief        The class for storaging leapsecond data.
 */

#ifndef hwa_leapsecond_H
#define hwa_leapsecond_H

#include "hwa_base_data.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief       Class for save leapsecond data
    *
    * This class save leapsecond data and provide the interface to get data.
    */
    class gnss_data_leapsecond : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_leapsecond();

        /** @brief default constructor. */
        gnss_data_leapsecond(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_leapsecond();

        /**
        * @brief add leapsecond data to std::map.
        * @param[in]   mjd   mjd of the leapsecon data
        * @param[in]   leap  leapsecon data
        */
        void add_data(int mjd, int leap);

        /** @brief whether the leapsecond data is empty.
        * @return  bool
        *    @retval   true     leapsecond data is empty
        *   @retval   false    leapsecond data is existent
        */
        bool is_empty();

        /**
        * @brief get hole leapsecond data
        * @return leapsecond data include time and seconds
        */
        std::map<int, int> &get_leap() { return _leapseconds; };

    protected:
        std::map<int, int> _leapseconds; ///< leapsecond data(mjd,value)

    private:
    };

}
#endif // !GALLPLANETEPH_H
