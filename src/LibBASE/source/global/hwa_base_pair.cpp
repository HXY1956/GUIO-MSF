#include <cmath>
#include <iomanip>
#include "hwa_base_pair.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_base
{

    // null constructor
    // ----------
    base_pair::base_pair()
    {
        _crd[0] = _crd[1] = 0.0;
    }

    // constructor
    // ----------
    base_pair::base_pair(double x, double y)
    {
        _crd[0] = x;
        _crd[1] = y;
    }

    // constructor
    // ----------
    base_pair::base_pair(double crd[2])
    {
        _crd[0] = crd[0];
        _crd[1] = crd[1];
    }

    // constructor
    // ----------
    base_pair::base_pair(const Vector &crd)
    {
        _crd[0] = crd(0);
        _crd[1] = crd(1);
    }

    // destructor
    // ----------
    base_pair::~base_pair() {}

    // get a reference of element
    // ----------
    double &base_pair::operator[](const size_t idx)
    {
        //  boost::mutex::scoped_lock lock(_mutex_pair);
        if (idx > 1)
        {
            cerr << "Not valid pair index [used 0]\n";
            return _crd[0];
        }

        return _crd[idx];
    }

    // get a value of element
    // ----------
    double base_pair::operator[](const size_t idx) const
    {
        //  boost::mutex::scoped_lock lock(
        //  _mutex_pair);
        if (idx < 2)
            return _crd[idx];

        return 0.0;
    }

    // operator +
    // ------------------------
    base_pair base_pair::operator+(const base_pair &other) const
    {
        base_pair tmp(*this);
        tmp[0] += other[0];
        tmp[1] += other[1];

        return tmp;
    }

    // get single element
    // ----------
    double base_pair::crd(int idx) const
    {
        //  boost::mutex::scoped_lock lock(_mutex_pair);
        if (idx >= 0 && idx < 2)
            return _crd[static_cast<unsigned int>(idx)];

        return 0.0;
    }

    // set single element
    // ------------------------
    void base_pair::set(int idx, double newValue)
    {
        //  boost::mutex::scoped_lock lock(_mutex_pair);
        if (idx >= 0 && idx < 2)
            _crd[static_cast<unsigned int>(idx)] = newValue;
    }

    // copy operator
    // ----------
    base_pair &base_pair::operator=(const base_pair &other)
    {
        //  boost::mutex::scoped_lock lock(_mutex_pair);
        if (this != &other)
        {
            _crd[0] = other[0];
            _crd[1] = other[1];
        }
        return *this;
    }

    // equal operator
    // ----------
    bool base_pair::operator==(const base_pair &tr) const
    {
        //  boost::mutex::scoped_lock lock(_mutex_pair);
        return (_crd[0] == tr[0] &&
                _crd[1] == tr[1]);
    }

    // operator for sorting
    // ----------
    bool base_pair::operator<(const base_pair &tr) const
    {
        return ((_crd[0] < tr[0]) ||
                (_crd[0] == tr[0] && _crd[1] < tr[1]));
    }

    // get array
    // ----------
    double *base_pair::crd_array()
    {
        //  boost::mutex::scoped_lock lock(_mutex_pair);
        return _crd;
    }

    // get Vector[3]
    // ----------
    Vector base_pair::crd_cvect()
    {
        Vector tmp(2);
        tmp(0) = _crd[0];
        tmp(1) = _crd[1];
        return tmp;
    }

    // get pair
    // ----------
    base_pair &base_pair::crd_pair()
    {
        return *this;
    }

    // set array by Vector
    // ------------------------------
    void base_pair::set(const Vector &crd)
    {
        _crd[0] = crd(0);
        _crd[1] = crd(1);
    }

    // set array by array
    // ------------------------------
    void base_pair::set(double crd[2])
    {
        _crd[0] = crd[0];
        _crd[1] = crd[1];
    }

    // get unit Vector
    // -----------------------------
    Vector base_pair::unitary()
    {
        Vector tmp(2);
        tmp = this->crd_cvect();
        double s = tmp.norm();
        tmp /= s;

        return tmp;
    }

    // overloading << operator
    // -----------------------------
    std::ostream &operator<<(std::ostream &os, const base_pair &x)
    {
        os << std::fixed << setprecision(3)
           << base_type_conv::dbl2str(x[0]) + " " + base_type_conv::dbl2str(x[1]);
        return os;
    }

    // test if all elements are zero
    // -----------------------------
    bool base_pair::zero()
    {
        if (double_eq(_crd[0], 0.0) &&
            double_eq(_crd[1], 0.0))
            return true;
        else
            return false;
    }

} // namespace
