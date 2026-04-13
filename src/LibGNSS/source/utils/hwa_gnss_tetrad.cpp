#include <cmath>
#include <iomanip>
#include "hwa_gnss_tetrad.h"
#include "hwa_base_const.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{

    // null constructor
    // ----------
    gnss_util_tetrad::gnss_util_tetrad()
    {
        _crd[0] = _crd[1] = _crd[2] = _crd[3] = 0.0;
    }

    // constructor
    // ----------
    gnss_util_tetrad::gnss_util_tetrad(double x, double y, double z, double t)
    {
        _crd[0] = x;
        _crd[1] = y;
        _crd[2] = z;
        _crd[3] = t;
    }

    // constructor
    // ----------
    gnss_util_tetrad::gnss_util_tetrad(double crd[])
    {
        _crd[0] = crd[0];
        _crd[1] = crd[1];
        _crd[2] = crd[2];
        _crd[3] = crd[3];
    }

    // constructor
    // ----------
    gnss_util_tetrad::gnss_util_tetrad(const Vector &crd)
    {
        _crd[0] = crd(1);
        _crd[1] = crd(2);
        _crd[2] = crd(3);
        _crd[3] = crd(4);
    }

    // destructor
    // ----------
    gnss_util_tetrad::~gnss_util_tetrad() {}

    // get a reference of element
    // ----------
    double &gnss_util_tetrad::operator[](const size_t idx)
    {
        //  boost::mutex::scoped_lock lock(_mutex_tetrad);
        if (idx > 3)
        {
            cerr << "Not valid tetrad index [used 0]\n";
            return _crd[0];
        }
        return _crd[idx];
    }

    // get a value of element
    // ----------
    double gnss_util_tetrad::operator[](const size_t idx) const
    {
        //  boost::mutex::scoped_lock lock(
        //  _mutex_tetrad);
        if (idx < 4)
            return _crd[idx];

        return 0.0;
    }

    // operator +
    // ------------------------
    gnss_util_tetrad gnss_util_tetrad::operator+(const gnss_util_tetrad &other) const
    {
        gnss_util_tetrad tmp(*this);
        tmp[0] += other[0];
        tmp[1] += other[1];
        tmp[2] += other[2];
        tmp[3] += other[3];

        return tmp;
    }

    // operator for sorting
    // ----------
    bool gnss_util_tetrad::operator<(const gnss_util_tetrad &tr) const
    {
        return ((_crd[0] < tr[0]) ||
                (_crd[0] == tr[0] && _crd[1] < tr[1]) ||
                (_crd[0] == tr[0] && _crd[1] == tr[1] && _crd[2] < tr[2]) ||
                (_crd[0] == tr[0] && _crd[1] == tr[1] && _crd[2] < tr[2] && _crd[3] < tr.crd(3)));
    }

    // get single element
    // ----------
    double gnss_util_tetrad::crd(const int &idx) const
    {
        //  boost::mutex::scoped_lock lock(_mutex_tetrad);
        if (idx >= 0 && idx < 4)
            return _crd[static_cast<unsigned int>(idx)];

        return 0.0;
    }

    // set single element
    // ------------------------
    void gnss_util_tetrad::set(const int &idx, const double &newValue)
    {
        //  boost::mutex::scoped_lock lock(_mutex_tetrad);
        if (idx >= 0 && idx < 4)
            _crd[static_cast<unsigned int>(idx)] = newValue;
    }

    // copy operator
    // ----------
    gnss_util_tetrad &gnss_util_tetrad::operator=(const gnss_util_tetrad &other)
    {
        //  boost::mutex::scoped_lock lock(_mutex_tetrad);
        if (this != &other)
        {
            _crd[0] = other[0];
            _crd[1] = other[1];
            _crd[2] = other[2];
            _crd[3] = other.crd(3);
        }
        return *this;
    }

    // equal operator
    // ----------
    bool gnss_util_tetrad::operator==(const gnss_util_tetrad &tr) const
    {
        //  boost::mutex::scoped_lock lock(_mutex_tetrad);
        return (_crd[0] == tr[0] &&
                _crd[1] == tr[1] &&
                _crd[2] == tr[2] &&
                _crd[3] == tr.crd(3));
    }

    // get array
    // ----------
    double *gnss_util_tetrad::crd_array()
    {
        //  boost::mutex::scoped_lock lock(_mutex_tetrad);
        return _crd;
    }

    // get Vector[3]
    // ----------
    Vector gnss_util_tetrad::crd_cvect()
    {
        Vector tmp(4);
        tmp(0) = _crd[0];
        tmp(1) = _crd[1];
        tmp(2) = _crd[2];
        tmp(3) = _crd[3];
        return tmp;
    }

    // get tetrad
    // ----------
    gnss_util_tetrad &gnss_util_tetrad::crd_tetrad()
    {
        return *this;
    }

    // set array by Vector
    // ------------------------------
    void gnss_util_tetrad::set(const Vector &crd)
    {
        _crd[0] = crd(1);
        _crd[1] = crd(2);
        _crd[2] = crd(3);
        _crd[3] = crd(4);
    }

    // set array by array
    // ------------------------------
    void gnss_util_tetrad::set(double crd[4])
    {
        _crd[0] = crd[0];
        _crd[1] = crd[1];
        _crd[2] = crd[2];
        _crd[3] = crd[3];
    }

    // get unit Vector
    // -----------------------------
    Vector gnss_util_tetrad::unitary()
    {
        Vector tmp(4);
        tmp = this->crd_cvect();
        double s = tmp.norm();
        tmp /= s;

        return tmp;
    }

    // overloading << operator
    // -----------------------------
    std::ostream &operator<<(std::ostream &os, const gnss_util_tetrad &x)
    {
        os << std::fixed << setprecision(3)
           << hwa_base::base_type_conv::dbl2str(x[0]) + " " + hwa_base::base_type_conv::dbl2str(x[1]) + " " + hwa_base::base_type_conv::dbl2str(x[2]) + " " + hwa_base::base_type_conv::dbl2str(x[3]);
        return os;
    }

    // test if all elements are zero
    // -----------------------------
    bool gnss_util_tetrad::zero()
    {
        if (hwa_base::double_eq(_crd[0], 0.0) &&
            hwa_base::double_eq(_crd[1], 0.0) &&
            hwa_base::double_eq(_crd[2], 0.0) &&
            hwa_base::double_eq(_crd[3], 0.0))
            return true;
        else
            return false;
    }

} // namespace
