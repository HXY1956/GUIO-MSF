#include "hwa_gnss_all_atmloading.h"
#include "hwa_base_const.h"
#include "hwa_base_time.h"

namespace hwa_gnss
{
    gnss_all_atmloading::gnss_all_atmloading()
    {
        //   std::cout << "vytvarim gnss_all_otl" << endl;
        id_type(base_data::ALLATL);
        _grid_cs1du.resize(_nlon, _nlat);
        _grid_cs1dui.resize(_nlon, _nlat);
        _grid_ss1du.resize(_nlon, _nlat);
        _grid_ss1dui.resize(_nlon, _nlat);
        _grid_cs2du.resize(_nlon, _nlat);
        _grid_cs2dui.resize(_nlon, _nlat);
        _grid_ss2du.resize(_nlon, _nlat);
        _grid_ss2dui.resize(_nlon, _nlat);
        _grid_cs1dn.resize(_nlon, _nlat);
        _grid_cs1dni.resize(_nlon, _nlat);
        _grid_ss1dn.resize(_nlon, _nlat);
        _grid_ss1dni.resize(_nlon, _nlat);
        _grid_cs2dn.resize(_nlon, _nlat);
        _grid_cs2dni.resize(_nlon, _nlat);
        _grid_ss2dn.resize(_nlon, _nlat);
        _grid_ss2dni.resize(_nlon, _nlat);
        _grid_cs1de.resize(_nlon, _nlat);
        _grid_cs1dei.resize(_nlon, _nlat);
        _grid_ss1de.resize(_nlon, _nlat);
        _grid_ss1dei.resize(_nlon, _nlat);
        _grid_cs2de.resize(_nlon, _nlat);
        _grid_cs2dei.resize(_nlon, _nlat);
        _grid_ss2de.resize(_nlon, _nlat);
        _grid_ss2dei.resize(_nlon, _nlat);

        _grid_cs1du.setConstant(0);
        _grid_cs1dui.setConstant(0);
        _grid_ss1du.setConstant(0);
        _grid_ss1dui.setConstant(0);
        _grid_cs2du.setConstant(0);
        _grid_cs2dui.setConstant(0);
        _grid_ss2du.setConstant(0);
        _grid_ss2dui.setConstant(0);
        _grid_cs1dn.setConstant(0);
        _grid_cs1dni.setConstant(0);
        _grid_ss1dn.setConstant(0);
        _grid_ss1dni.setConstant(0);
        _grid_cs2dn.setConstant(0);
        _grid_cs2dni.setConstant(0);
        _grid_ss2dn.setConstant(0);
        _grid_ss2dni.setConstant(0);
        _grid_cs1de.setConstant(0);
        _grid_cs1dei.setConstant(0);
        _grid_ss1de.setConstant(0);
        _grid_ss1dei.setConstant(0);
        _grid_cs2de.setConstant(0);
        _grid_cs2dei.setConstant(0);
        _grid_ss2de.setConstant(0);
        _grid_ss2dei.setConstant(0);
    }

    gnss_all_atmloading::~gnss_all_atmloading()
    {
        _gridatmld.clear();
    }

    /**
     * @brief 
     * 
     * @param _lon 
     * @param _lonmode 
     * @param _lat 
     * @param _latmode 
     * @param coeffs 
     * @return true 
     * @return false 
     */
    bool gnss_all_atmloading::data(const double &_lon, const std::string &lonmode, const double &_lat, const std::string &latmode, Matrix &coeffs)
    {

        const double deps = 1e-5;
        double lon = _lon;
        double lat = _lat;
        if (lon < 0)
            lon += 360;
        lat += 90;
        if (lon < 0 || lon > 360 || lat < 0 || lat > 180)
        {
            return false;
        }

        int ilon = 0, ilat = 0;
        if (lonmode.find("floor") != std::string::npos || lonmode.find("FLOOR") != std::string::npos)
        {
            ilon = floor(lon);
        }
        else
        {
            ilon = ceil(lon);
        }
        if (latmode.find("floor") != std::string::npos || latmode.find("FLOOR") != std::string::npos)
        {
            ilat = floor(lat);
        }
        else
        {
            ilat = ceil(lat);
        }
        for (auto atmld_iter = _gridatmld.begin(); atmld_iter != _gridatmld.end(); atmld_iter++)
        {
            double ddif_lon = fabs(atmld_iter->lon() - ilon);
            double ddif_lat = fabs(atmld_iter->lat() - ilat);
            if (ddif_lon < deps && ddif_lat < deps)
            {
                coeffs = atmld_iter->data();
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 
     * 
     * @param _lon 
     * @param _lat 
     * @param coeff 
     * @return true 
     * @return false 
     */
    bool gnss_all_atmloading::spine(const double &_lon, const double &_lat, Matrix &coeff)
    {
        double lon = _lon;
        double lat = _lat;
        if (lon < 0)
            lon += 360;
        if (lon < 0 || lon > 360 || lat < -90 || lat > 90)
        {
            return false;
        }

        return true;
    }

    // Add element to vector
    // -----------------------------
    void gnss_all_atmloading::add(gnss_data_atmloading &atmld)
    {
        _gridatmld.push_back(atmld);

        double lon = atmld.lon();
        double lat = atmld.lat();
        Matrix tgrid = atmld.data();

        int index_lon = int(lon + 1e-4) + 1;
        lat = 90.0 - lat; // from (90 -> -90) to (0 -> 180);
        int index_lat = int(lat + 1e-4) + 1;
        if (index_lon <= 360)
        {
            _grid_cs1du(index_lon, index_lat) = tgrid(0, 0);
            _grid_ss1du(index_lon, index_lat) = tgrid(0, 1);
            _grid_cs2du(index_lon, index_lat) = tgrid(0, 2);
            _grid_ss2du(index_lon, index_lat) = tgrid(0, 3);
            _grid_cs1dn(index_lon, index_lat) = tgrid(1, 0);
            _grid_ss1dn(index_lon, index_lat) = tgrid(1, 1);
            _grid_cs2dn(index_lon, index_lat) = tgrid(1, 2);
            _grid_ss2dn(index_lon, index_lat) = tgrid(1, 3);
            _grid_cs1de(index_lon, index_lat) = tgrid(2, 0);
            _grid_ss1de(index_lon, index_lat) = tgrid(2, 1);
            _grid_cs2de(index_lon, index_lat) = tgrid(2, 2);
            _grid_ss2de(index_lon, index_lat) = tgrid(2, 3);
        }
        return;
    }

    // Print all data
    // --------------------
    void gnss_all_atmloading::print()
    {
        std::vector<gnss_data_atmloading>::iterator it;
        for (it = _gridatmld.begin(); it != _gridatmld.end(); ++it)
        {
            std::cout << " Lon: " << it->lon() << " Lat: " << it->lat() << std::endl;
            std::cout << "Data: \n"
                 << it->data() << std::endl;
        }
    }

    void gnss_all_atmloading::splie2_coeffs()
    {
        _lon.resize(_nlon, 1);
        _lat.resize(_nlat, 1);
        for (int i = 1; i <= _nlon; ++i)
        {
            if (i <= _nlat)
            {
                _lat(i, 1) = i - 1.0;
            }
            _lon(i, 1) = i - 1.0;
        }
        _lat(_nlat, 1) = 179.99;

        splie2(_grid_cs1du, _grid_cs1dui);
        splie2(_grid_ss1du, _grid_ss1dui);
        splie2(_grid_cs2du, _grid_cs2dui);
        splie2(_grid_ss2du, _grid_ss2dui);
        splie2(_grid_cs1dn, _grid_cs1dni);
        splie2(_grid_ss1dn, _grid_ss1dni);
        splie2(_grid_cs2dn, _grid_cs2dni);
        splie2(_grid_ss2dn, _grid_ss2dni);
        splie2(_grid_cs1de, _grid_cs1dei);
        splie2(_grid_ss1de, _grid_ss1dei);
        splie2(_grid_cs2de, _grid_cs2dei);
        splie2(_grid_ss2de, _grid_ss2dei);
    }

    void gnss_all_atmloading::splie2(Matrix &ya, Matrix &y2a)
    {
        y2a.resize(_nlon, _nlat);
        y2a.setConstant(0);
        for (int j = 0; j < _nlon; ++j)
        {
            Matrix ytmp, y2tmp;
            ytmp.resize(_nlat, 1);
            y2tmp.resize(_nlat, 1);
            ytmp.setConstant(0);
            y2tmp.setConstant(0);
            for (int k = 0; k < _nlat; ++k)
            {
                ytmp(k, 0) = ya(j, k);
            }
            spline(_lat, ytmp, _nlat, 1e30, 1e30, y2tmp);
            for (int k = 0; k < _nlat; ++k)
            {
                y2a(j, k) = y2tmp(k, 0);
            }
        }
    }

    void gnss_all_atmloading::spline(Matrix &x, Matrix &y, const int &n, const double &yp1, const double &ypn, Matrix &y2)
    {
        Matrix u;
        u.resize(500, 1);
        u.setConstant(0);
        if (yp1 > 0.99e30)
        {
            y2(0, 0) = 0;
            u(0, 0) = 0;
        }
        else
        {
            y2(0, 0) = -0.5;
            u(0, 0) = (3. / (x(1, 0) - x(0, 0))) * ((y(1, 0) - y(0, 0)) / (x(1, 0) - x(0, 0)) - yp1);
        }
        for (int i = 1; i < n - 1; i++)
        {
            double sig = (x(i, 0) - x(i - 1, 0)) / (x(i + 1, 0) - x(i - 1, 0));
            double p = sig * y2(i - 1, 0) + 2.0;
            y2(i, 0) = (sig - 1.0) / p;
            u(i, 0) = (6.0 * ((y(i + 1, 0) - y(i, 0)) / (x(i + 1, 0) - x(i, 0)) - (y(i, 0) - y(i - 1, 0)) / (x(i, 0) - x(i - 1, 0))) / (x(i + 1, 0) - x(i - 1, 0)) - sig * u(i - 1, 0)) / p;
        }
        double qn;
        double un;
        if (ypn > 0.99e30)
        {
            qn = 0;
            un = 0;
        }
        else
        {
            qn = 0.5;
            un = (3.0 / (x(n - 1, 0) - x(n - 2, 0))) * (ypn - (y(n - 1, 0) - y(n - 2, 0)) / (x(n - 1, 0) - x(n - 2, 0)));
        }
        y2(n - 1, 0) = (un - qn * u(n - 2, 0)) / (qn * y2(n - 2, 0) + 1.0);
        for (int k = n - 2; k >= 0; k--)
        {
            y2(k, 0) = y2(k, 0) * y2(k + 1, 0) + u(k, 0);
        }
    }

    bool gnss_all_atmloading::get_coef(const double &_lon, const double &_lat, Matrix &coeff)
    {
        double lon = _lon;
        double lat = _lat;
        if (_lon < 0)
            lon += 360;
        lat = 90 - lat;
        if (lon < 0 || lon > 360 || lat < 0 || lat > 180)
        {
            return false;
        }

        // use the Linear Interpolation method
        int lat_lower = int(lat);      // 0-179 deg
        int lat_upper = lat_lower + 1; // 1-180 deg
        double dlat = lat - lat_lower;
        int lon_lower = int(lon);      // 0-359 deg
        int lon_upper = lon_lower + 1; // 1-360 deg
        if (lon_upper >= 360)
            lon_upper = 0;
        double dlon = lon - lon_lower;
        //std::cout << lat_lower << lat_upper << endl;
        //std::cout << lon_lower << lon_upper << endl;

        coeff(0, 0) = linear_intep(_grid_cs1du(lon_lower , lat_lower), _grid_cs1du(lon_upper, lat_lower),
                                   _grid_cs1du(lon_lower , lat_upper), _grid_cs1du(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(0, 1) = linear_intep(_grid_ss1du(lon_lower , lat_lower), _grid_ss1du(lon_upper, lat_lower),
                                   _grid_ss1du(lon_lower , lat_upper), _grid_ss1du(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(0, 2) = linear_intep(_grid_cs2du(lon_lower , lat_lower), _grid_cs2du(lon_upper, lat_lower),
                                   _grid_cs2du(lon_lower , lat_upper), _grid_cs2du(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(0, 3) = linear_intep(_grid_ss2du(lon_lower , lat_lower), _grid_ss2du(lon_upper, lat_lower),
                                   _grid_ss2du(lon_lower , lat_upper), _grid_ss2du(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(1, 0) = linear_intep(_grid_cs1dn(lon_lower , lat_lower), _grid_cs1dn(lon_upper, lat_lower),
                                   _grid_cs1dn(lon_lower , lat_upper), _grid_cs1dn(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(1, 1) = linear_intep(_grid_ss1dn(lon_lower , lat_lower), _grid_ss1dn(lon_upper, lat_lower),
                                   _grid_ss1dn(lon_lower , lat_upper), _grid_ss1dn(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(1, 2) = linear_intep(_grid_cs2dn(lon_lower , lat_lower), _grid_cs2dn(lon_upper, lat_lower),
                                   _grid_cs2dn(lon_lower , lat_upper), _grid_cs2dn(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(1, 3) = linear_intep(_grid_ss2dn(lon_lower , lat_lower), _grid_ss2dn(lon_upper, lat_lower),
                                   _grid_ss2dn(lon_lower , lat_upper), _grid_ss2dn(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(2, 0) = linear_intep(_grid_cs1de(lon_lower , lat_lower), _grid_cs1de(lon_upper, lat_lower),
                                   _grid_cs1de(lon_lower , lat_upper), _grid_cs1de(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(2, 1) = linear_intep(_grid_ss1de(lon_lower , lat_lower), _grid_ss1de(lon_upper, lat_lower),
                                   _grid_ss1de(lon_lower , lat_upper), _grid_ss1de(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(2, 2) = linear_intep(_grid_cs2de(lon_lower , lat_lower), _grid_cs2de(lon_upper, lat_lower),
                                   _grid_cs2de(lon_lower , lat_upper), _grid_cs2de(lon_upper, lat_upper),
                                   dlon, dlat);
        coeff(2, 3) = linear_intep(_grid_ss2de(lon_lower , lat_lower), _grid_ss2de(lon_upper, lat_lower),
                                   _grid_ss2de(lon_lower , lat_upper), _grid_ss2de(lon_upper, lat_upper),
                                   dlon, dlat);
        return true;
    }

    double gnss_all_atmloading::splin2(Matrix &grid_coef, Matrix &grid_coef_spined, const double &x1, const double &x2)
    {
        int m = _nlon;
        int n = _nlat;
        Matrix x1a = _lon;
        Matrix x2a = _lat;
        Matrix ya = grid_coef;
        Matrix y2a = grid_coef_spined;

        Matrix ytmp, y2tmp, yytmp;
        ytmp.resize(_nlon, 1);
        y2tmp.resize(_nlon, 1);
        yytmp.resize(_nlon, 1);
        ytmp.setConstant(0);
        y2tmp.setConstant(0);
        yytmp.setConstant(0);
        for (int j = 0; j < _nlon; ++j)
        {
            for (int k = 0; k < _nlat; ++k)
            {
                ytmp(k - 1, 0) = ya(j, k);
                y2tmp(k - 1, 0) = y2a(j, k);
            }
            yytmp(j, 0) = splint(x2a, ytmp, y2tmp, n, x2);
            //std::cout << "The " << j << "value is" << yytmp(j, 1) << endl;
            ytmp.setConstant(0);
            y2tmp.setConstant(0);
        }
        spline(x1a, yytmp, m, 1.e30, 1.e30, y2tmp);
        double y = splint(x1a, yytmp, y2tmp, m, x1);
        return y;
    }

    double gnss_all_atmloading::splint(Matrix &xa, Matrix &ya, Matrix &y2a, const int &n, const double &x)
    {
        int klo = 0;
        int khi = n - 1;
        for (; khi - klo > 1;)
        {
            int k = (khi + klo) / 2;
            if (xa(k, 0) > x)
            {
                khi = k;
            }
            else
            {
                klo = k;
            }
        }
        double h = xa(khi, 0) - xa(klo, 0);
        if (h != 0)
        {
            double a = (xa(khi, 0) - x) / h;
            double b = (x - xa(klo, 0)) / h;
            double y = a * ya(klo, 0) + b * ya(khi, 0) + ((pow(a, 3) - a) * y2a(klo, 0) + (pow(b, 3) - b) * y2a(khi, 0)) * pow(h, 2) / 6.0;
            return y;
        }
        else
        {
            return 0.0;
        }
    }

    double gnss_all_atmloading::linear_intep(const double &x1y1, const double &x2y1, const double &x1y2, const double &x2y2, const double &dx, const double &dy)
    {
        double dxy1 = (1.0 - dx) * x1y1 + dx * x2y1;
        double dxy2 = (1.0 - dx) * x1y2 + dx * x2y2;
        double dxdy = (1.0 - dy) * dxy1 + dy * dxy2;
        return dxdy;
    }

} // namespace
