#ifndef hwa_gnss_amb_BL_H
#define hwa_gnss_amb_BL_H

#include "hwa_set_gtype.h"

using namespace hwa_base;

namespace hwa_gnss
{

    class gnss_amb_baseline
    {
    public:
        /**
         * @brief Construct a new t gamb baseline object
         */
        explicit gnss_amb_baseline();
        /**
         * @brief Construct a new t gamb baseline object
         * @param[in]  obj1      object one
         * @param[in]  obj2      object two
         * @param[in]  distance  distance
         * @param[in]  nsys      system
         */
        explicit gnss_amb_baseline(const std::string &obj1, const std::string &obj2, const double &distance, const int &nsys);
        /**
         * @brief Destroy the t gamb baseline object
         */
        virtual ~gnss_amb_baseline();
        /**
         * @brief make obj1 and obj2 std::pair
         * @return std::pair<std::string, std::string> <obj1,obj2>
         */
        std::pair<std::string, std::string> bl_pair() const;
        /**
         * @brief whether obj1==obj_1st  obj2==obj_2nd
         * @param[in]  obj_1st   obj first
         * @param[in]  obj_2nd   obj second
         * @return true 
         *         @retval true same
         *         @retval false different
         */
        bool isSame(const std::string &obj_1st, const std::string &obj_2nd);

        std::string obj1;     ///< TODO
        std::string obj2;     ///< TODO
        int nsys;        ///< system
        double distance; ///< distance
    };
}

#endif