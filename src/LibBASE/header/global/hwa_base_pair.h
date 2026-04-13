#ifndef hwa_base_pair_h
#define hwa_base_pair_h

#include <iostream>
#include <string>
#include "hwa_base_eigendef.h"

namespace hwa_base
{

    /** @brief class for base_pair. */
    class base_pair
    {

    public:
        /** @brief default constructor. */
        base_pair();

        /** @brief constructor 1. */
        base_pair(double x, double y);

        /** @brief constructor 2. */
        explicit base_pair(double crd[2]);

        /** @brief constructor 3. */
        explicit base_pair(const Vector &crd);

        /** @brief default destructor. */
        virtual ~base_pair();

        /** @brief override operator. */
        base_pair &operator=(const base_pair &other);      // assignment operator
        base_pair operator+(const base_pair &other) const; //
        bool operator==(const base_pair &tr) const;      // equal operator
        bool operator<(const base_pair &tr) const;
        double &operator[](const size_t idx);      // get a reference of element
        double operator[](const size_t idx) const; // get value of element
        friend std::ostream &operator<<(std::ostream &os, const base_pair &x);

        double crd(int idx) const;          ///< get single element
        void set(int idx, double newValue); ///< set single element
        void set(const Vector &);     ///< set array by Vector
        void set(double crd[2]);            ///< set array by array
        double *crd_array();                ///< get array
        Vector crd_cvect();           ///< get Vector
        base_pair &crd_pair();                ///< get pair
        Vector unitary();             ///< get unit Vector
        bool zero();                        ///< true: zero elements, false: not zero elements

    protected:
    private:
        double _crd[2];
    };

} // namespace

#endif
