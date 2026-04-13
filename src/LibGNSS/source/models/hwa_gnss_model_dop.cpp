#include <iostream>
#include "hwa_gnss_model_dop.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_gnss
{
    gnss_model_dop::gnss_model_dop()
    {
        _gnav = 0;
        _gobs = 0;
        _site = "";
        _Qx.resize(4);
    }
    gnss_model_dop::gnss_model_dop(base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }

        _gnav = 0;
        _gobs = 0;
        _site = "";
        _Qx.resize(4);
    }

    gnss_model_dop::gnss_model_dop(base_log spdlog, gnss_all_nav *gnav, gnss_all_obs *gobs, std::string site)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
        _gnav = gnav;
        _gobs = gobs;
        _site = site;
        _Qx.resize(4);
    }
    gnss_model_dop::gnss_model_dop(gnss_all_nav *gnav, gnss_all_obs *gobs, std::string site)
    {

        _gnav = gnav;
        _gobs = gobs;
        _site = site;
        _Qx.resize(4);
    }

    gnss_model_dop::gnss_model_dop(gnss_all_nav *gnav, std::set<std::string> sats)
    {
        _gnav = gnav;
        _sats = sats;
        _Qx.resize(4);
        _site = "";
        _gobs = 0;
    }
    gnss_model_dop::gnss_model_dop(base_log spdlog, gnss_all_nav *gnav, std::set<std::string> sats)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }

        _gnav = gnav;
        _sats = sats;
        _Qx.resize(4);
        _site = "";
        _gobs = 0;
    }

    // Destructor
    // ----------
    gnss_model_dop::~gnss_model_dop()
    {
    }

    // std::set nav, obs, site
    void gnss_model_dop::set_data(gnss_all_nav *gnav, gnss_all_obs *gobs, std::string site)
    {

        _gnav = gnav;
        _gobs = gobs;
        _site = site;
    }

    // std::set log
    void gnss_model_dop::set_log(base_log spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
    }

    // std::set satellite list for calculation
    void gnss_model_dop::set_sats(std::set<std::string> &sats)
    {

        _sats = sats;
    }

    // Calculate dop - _Qx
    int gnss_model_dop::calculate(const base_time &epoch, Triple &rec, GSYS gnss)
    {

        _Qx.setZero();

        _rec = rec;

        if (_sats.size() == 0)
        {
            if (_gobs)
                _sats = _gobs->sats(_site, epoch, gnss);
            else if (_sats.size() == 0)
            {
                std::string msg = "WARNING - not selected satellites for DOP calculation!";
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, msg);
                return -1;
            }
        }

        if (_sats.size() == 0)
            return -1;
        unsigned int Nsat = _sats.size();

        Matrix A(Nsat, 4);
        A.setZero();
        int i = 0;
        for (std::set<std::string>::iterator it = _sats.begin(); it != _sats.end(); it++)
        {

            double xyz[3] = {0.0, 0.0, 0.0};
            double vel[3] = {0.0, 0.0, 0.0};
            double var[3] = {0.0, 0.0, 0.0};

            int irc = _gnav->pos(*it, epoch, xyz, var, vel);
            if (irc < 0)
                continue;

            Triple satpos(xyz);
            Triple xyz_rho = satpos - _rec;
            Triple ell_site;
            xyz2ell(_rec, ell_site, false);

            // select only visible satellites
            Triple neu_sat;
            xyz2neu(ell_site, xyz_rho, neu_sat);
            if (neu_sat[2] < 0)
                continue;

            double rho = (_rec - satpos).norm();

            A(i, 0) = (_rec[0] - satpos[0]) / rho;
            A(i, 1) = (_rec[1] - satpos[1]) / rho;
            A(i, 2) = (_rec[2] - satpos[2]) / rho;
            A(i, 3) = 1.0;
            i++;
        }
        A = A.block(0, 0, i, A.cols()); // delete zero rows
        if (A.rows() < 4)
            return -1;

        Matrix NN = A.transpose() * A;

        _Qx.matrixW() = NN.inverse();

        return 1;
    }

    // Position dilution of precision
    double gnss_model_dop::pdop()
    {

        if (_Qx.cols() != _Qx.rows())
            return -1.0;
        if (_Qx.cols() != 4)
            return -1.0;

        return sqrt(_Qx(0, 0) + _Qx(1, 1) + _Qx(2, 2));
    }

    // Geom dilution of precision
    double gnss_model_dop::gdop()
    {
        if (_Qx.cols() != _Qx.rows())
            return -1.0;
        if (_Qx.cols() != 4)
            return -1.0;

        return sqrt(_Qx(0, 0) + _Qx(1, 1) + _Qx(2, 2) + _Qx(3, 3));
    }

    // Time dilution of precision
    double gnss_model_dop::tdop()
    {

        if (_Qx.cols() != _Qx.rows())
            return -1.0;
        if (_Qx.cols() != 4)
            return -1.0;

        return sqrt(_Qx(3, 3));
    }

    // Horizontal dilution of precision
    double gnss_model_dop::hdop()
    {

        if (_Qx.cols() != _Qx.rows())
            return -1.0;
        if (_Qx.cols() != 4)
            return -1.0;

        Symmetric Qp = _Qx.SymSubMatrix(0, 2);
        Symmetric Qneu;
        Triple neu;

        xyz2neu(_rec, _Qx, Qneu);

        return sqrt(Qneu(0, 0));
    }

    // Vertical dilution of precision
    double gnss_model_dop::vdop()
    {

        if (_Qx.cols() != _Qx.rows())
            return -1.0;
        if (_Qx.cols() != 4)
            return -1.0;

        Symmetric Qp = _Qx.SymSubMatrix(0, 2);
        Symmetric Qneu;
        Triple neu;

        xyz2neu(_rec, _Qx, Qneu);

        return sqrt(Qneu(2, 2));
    }

} // namespace
