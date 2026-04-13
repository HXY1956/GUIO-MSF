#include <hwa_gnss_data_recover.h>

using namespace std;

namespace hwa_gnss
{
    gnss_data_recover_head::gnss_data_recover_head() : interval(0.0)
    {
    }

    gnss_data_recover_head::~gnss_data_recover_head()
    {
    }

    base_time gnss_data_recover_head::get_beg_time() const
    {
        return *(time_list.begin());
    }

    base_time gnss_data_recover_head::get_end_time() const
    {
        return *(time_list.end());
    }

    gnss_data_recover::gnss_data_recover() : base_data()
    {
    }

    gnss_data_recover::gnss_data_recover(base_log spdlog) : base_data(spdlog)
    {
    }
    gnss_data_recover::~gnss_data_recover()
    {
    }

    gnss_data_recover_equation::gnss_data_recover_equation(const base_time &time,
                                             const std::string &site,
                                             const std::string &sat) : gnss_data_recover(),
                                                                  time(time),
                                                                  sat_name(sat),
                                                                  site_name(site),
                                                                  weight(0.0),
                                                                  resuidal(0.0),
                                                                  is_newamb(0)
    {
        _type = base_data::RESOBS;
    }

    gnss_data_recover_equation::gnss_data_recover_equation(base_log spdlog,
                                             const base_time &time,
                                             const std::string &site,
                                             const std::string &sat) : gnss_data_recover(spdlog),
                                                                  time(time),
                                                                  sat_name(sat),
                                                                  site_name(site),
                                                                  weight(0.0),
                                                                  resuidal(0.0),
                                                                  is_newamb(0)
    {
        _type = base_data::RESOBS;
    }

    gnss_data_recover_equation::gnss_data_recover_equation(const gnss_data_recover_equation &other) : gnss_data_recover(other.spdlog()),
                                                                                 site_name(other.site_name),
                                                                                 sat_name(other.sat_name),
                                                                                 time(other.time),
                                                                                 obstype(other.obstype),
                                                                                 weight(other.weight),
                                                                                 resuidal(other.resuidal),
                                                                                 is_newamb(other.is_newamb)
    {
        time = other.time;
        _spdlog = (other.spdlog());
    }

    gnss_data_recover_equation::~gnss_data_recover_equation()
    {
    }

    void gnss_data_recover_equation::operator=(const gnss_data_recover_equation &other)
    {
        site_name = other.site_name;
        sat_name = other.sat_name;
        time = other.time;
        obstype = other.obstype;
        weight = other.weight;
        resuidal = other.resuidal;
        is_newamb = other.is_newamb;
    }

    base_time gnss_data_recover_equation::get_recover_time() const
    {
        return time;
    }

    std::string gnss_data_recover_equation::convert2strline() const
    {
        std::stringstream strline("");

        // format control
        strline << std::setiosflags(std::ios::right) << std::setprecision(4) << std::setiosflags(std::ios::fixed)
                << std::setw(5) << "RES:="
                << std::setw(25) << time.str()
                << std::setw(5) << is_newamb
                << std::setw(8) << site_name
                << std::setw(10) << sat_name
                << std::setw(8) << obstype.convert2str()
                << std::setw(15) << weight
                << std::setw(15) << resuidal << std::endl;

        return strline.str();
    }

    void gnss_data_recover_equation::set_recover_equation(const gnss_data_obscombtype &obstype, const std::pair<double, double> &resinfo, int is_newamb)
    {
        this->obstype = obstype;
        this->weight = resinfo.first;
        this->resuidal = resinfo.second;
        this->is_newamb = is_newamb;
    }

    gnss_data_recover_par::gnss_data_recover_par(const gnss_data_recover_par &other) : gnss_data_recover(other.spdlog()),
                                                                  par(other.par),
                                                                  correct_value(other.correct_value)
    {
    }

    gnss_data_recover_par::gnss_data_recover_par(base_log spdlog, const base_par &par, double correct_value) : gnss_data_recover(spdlog),
                                                                                               par(par),
                                                                                               correct_value(correct_value)
    {
        _type = base_data::RESPAR;
    }

    gnss_data_recover_par::~gnss_data_recover_par()
    {
    }

    base_time gnss_data_recover_par::get_recover_time() const
    {
        return par.end;
    }

    std::string gnss_data_recover_par::convert2strline() const
    {
        std::stringstream strline("");

        // jdhuang : no output for not estimated pars
        if (double_eq(correct_value, 0.0))
            return strline.str();

        // jdhuang : change some formate
        strline << std::setiosflags(std::ios::right) << std::setiosflags(std::ios::fixed)
                << std::setw(5) << "PAR:="
                << std::setw(25) << gpar2str(par)
                << std::setw(25) << par.beg.str()
                << std::setw(25) << par.end.str()
                << std::setw(25) << std::right << std::setprecision(7) << std::fixed << par.value()
                << std::setw(25) << std::right << std::setprecision(7) << std::scientific << std::uppercase << correct_value
                << std::setw(25) << std::right << std::setprecision(7) << std::fixed << par.value() + correct_value
                << std::endl;
        return strline.str();
    }

    bool gnss_data_recover_par::operator<(const gnss_data_recover_par &data) const
    {
        if (this->par > data.par)
            return true;
        return false;
    }

}
