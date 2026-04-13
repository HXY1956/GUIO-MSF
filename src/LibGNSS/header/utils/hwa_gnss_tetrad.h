
/**
* @file        gtetrad.h
* @brief    Purpose: implements 4D representation (e.g. coordinates + time)
*/

#ifndef hwa_gnss_util_tetrad_h
#define hwa_gnss_util_tetrad_h

#include <iostream>
#include <string>
#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief class for gnss_util_tetrad. */
    class gnss_util_tetrad
    {

    public:
        /** @brief default constructor. */
        gnss_util_tetrad();

        /** @brief constructor 1. */
        gnss_util_tetrad(double x, double y, double z, double t);

        /** @brief constructor 2. */
        explicit gnss_util_tetrad(double crd[4]);

        /** @brief constructor 3. */
        explicit gnss_util_tetrad(const Vector &crd);

        /** @brief default destructor. */
        virtual ~gnss_util_tetrad();

        /** @brief override operator. */
        gnss_util_tetrad &operator=(const gnss_util_tetrad &other);      // assignment operator
        gnss_util_tetrad operator+(const gnss_util_tetrad &other) const; //
        bool operator==(const gnss_util_tetrad &tr) const;        // equal operator
        bool operator<(const gnss_util_tetrad &tr) const;         // equal for sorting
        double &operator[](const size_t idx);              // get a reference of element
        double operator[](const size_t idx) const;         // get value of element
        friend std::ostream &operator<<(std::ostream &os, const gnss_util_tetrad &x);

        /** @brief get single element. */
        double crd(const int &idx) const;

        /** @brief set single element. */
        void set(const int &idx, const double &newValue);

        /** @brief set array by Vector. */
        void set(const Vector &);

        /** @brief set array by array. */
        void set(double crd[4]);

        /** @brief get array. */
        double *crd_array();

        /** @brief get Vector. */
        Vector crd_cvect();

        /** @brief get tetrad. */
        gnss_util_tetrad &crd_tetrad();

        /** @brief get unit Vector. */
        Vector unitary();

        /** @brief true: zero elements, false: not zero elements. */
        bool zero();

    protected:
    private:
        double _crd[4];
    };

} // namespace

#endif