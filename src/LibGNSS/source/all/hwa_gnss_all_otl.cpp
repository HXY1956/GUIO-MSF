#include "hwa_gnss_all_otl.h"
#include "hwa_base_const.h"
#include <math.h>

namespace hwa_gnss
{

    //Constructor
    gnss_all_otl::gnss_all_otl() : base_data()
    {
        id_type(base_data::ALLOTL);
    }
    gnss_all_otl::gnss_all_otl(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::ALLOTL);
    }

    //Destructor
    gnss_all_otl::~gnss_all_otl()
    {
        _mapotl.clear();
    }

    int gnss_all_otl::data(Matrix &otldata, const std::string &site)
    {
        if (_mapotl.find(site) == _mapotl.end() || _mapotl.size() == 0)
        {
            std::string site_short = site.substr(0, 4);
            if (_mapotl.find(site_short) == _mapotl.end() || _mapotl.size() == 0)
            {
                return -1;
            }
            else
                otldata = _mapotl[site_short].data();
        }
        else
            otldata = _mapotl[site].data();
        return 1;
    }

    int gnss_all_otl::data(Matrix &otldata, double lon, double lat)
    {

        const double dlon_eps = 1E4 / 6378235.0 * R2D, dlat_eps = dlon_eps / cos(lat * D2R);

        if (lon > 180)
            lon -= 360.0;

        for (auto otl_iter = _mapotl.begin(); otl_iter != _mapotl.end(); otl_iter++)
        {
            double dlon, dlat;
            dlon = fabs(otl_iter->second.lon() - lon);
            dlat = fabs(otl_iter->second.lat() - lat);
            if (dlon < dlon_eps && dlat < dlat_eps)
            {
                otldata = otl_iter->second.data();
                return 1;
            }
        }

        return -1;
    }

    double gnss_all_otl::lat(const std::string &site)
    {
        double tmp = 0.0;
        if (_mapotl.find(site) != _mapotl.end())
            tmp = _mapotl[site].lat();
        return tmp;
    }

    double gnss_all_otl::lon(const std::string &site)
    {
        double tmp = 0.0;
        if (_mapotl.find(site) != _mapotl.end())
            tmp = _mapotl[site].lon();
        return tmp;
    }

    // Add element to std::map container
    // -----------------------------
    void gnss_all_otl::add(gnss_data_otl &otl)
    {
        std::string site = otl.site();
        _mapotl[site] = otl;
        return;
    }

    // Print all data
    // --------------------
    void gnss_all_otl::print()
    {
        std::map<std::string, gnss_data_otl>::iterator it;
        for (it = _mapotl.begin(); it != _mapotl.end(); it++)
        {
            gnss_data_otl otl = it->second;
            std::cout << "Site: " << otl.site() << " Lon: " << otl.lon() << " Lat: " << otl.lat() << std::endl;
            std::cout << "Data: \n"
                 << otl.data() << std::endl;
        }
    }

} // namespace
