#include "hwa_gnss_all_opl.h"

namespace hwa_gnss
{
    gnss_all_opl::gnss_all_opl()
    {
        id_type(base_data::ALLOPL);
    }

    //Destructor
    gnss_all_opl::~gnss_all_opl()
    {
    }

    Triple gnss_all_opl::kart2ell(const Triple &p, const double &a, const double &b)
    {
        double ee = (a * a - b * b) / (a * a);
        double r1 = sqrt(p[0] * p[0] + p[1] * p[1]);
        double beta = atan(p[2] / r1);
        double m = b * p[2] / (a * r1);
        double n = (a * a - b * b) / (a * r1);
        for (int i = 1; i <= 5; i++)
            beta = atan(m + n * sin(beta));
        double phi = atan(a / b * tan(beta));
        double lam = atan2(p[1], p[0]);
        double N = a / sqrt(1 - ee * sin(phi) * sin(phi));
        double h = p[2] / sin(phi) - (1 - ee) * N;
        Triple ret(phi, lam, h);
        return ret;
    }

    int gnss_all_opl::data(const Triple &xyz, Triple &rne_r, Triple &rne_i)
    {
        Triple ell;
        base_pair lu, ld, ru, rd;
        Triple lu_r, ld_r, ru_r, rd_r;
        Triple lu_i, ld_i, ru_i, rd_i;
        ell = kart2ell(xyz, 6378137.0, 6356752.3141);
        if (ell[1] < 0)
            ell[1] = ell[1] + 2 * hwa_pi;
        ell[0] = ell[0] * R2D;
        ell[1] = ell[1] * R2D;
        ld[0] = int((ell[1] * 100 - 25) / 50) * 0.5 + 0.25;

        lu[0] = int((ell[1] * 100 - 25) / 50) * 0.5 + 0.25;
        if (ell[0] >= 0.0)
        {
            ld[1] = int((ell[0] * 100 - 25) / 50) * 0.5 + 0.25;
            lu[1] = (int((ell[0] * 100 - 25) / 50) + 1) * 0.5 + 0.25;
        }
        else
        {
            ld[1] = ceil((ell[0] * 100 - 25) / 50 - 1) * 0.5 + 0.25;
            lu[1] = int((ell[0] * 100 - 25) / 50) * 0.5 + 0.25;
        }

        ru[0] = (int((ell[1] * 100 - 25) / 50) + 1) * 0.5 + 0.25;
        if (ell[0] >= 0.0)
        {
            ru[1] = (int((ell[0] * 100 - 25) / 50) + 1) * 0.5 + 0.25;
            rd[1] = int((ell[0] * 100 - 25) / 50) * 0.5 + 0.25;
        }
        else
        {
            ru[1] = int((ell[0] * 100 - 25) / 50) * 0.5 + 0.25;
            rd[1] = (ceil((ell[0] * 100 - 25) / 50) - 1) * 0.5 + 0.25;
        }

        rd[0] = (int((ell[1] * 100 - 25) / 50) + 1) * 0.5 + 0.25;

        if (ld[0] < 0.0)
            ld[0] = ld[0] + 360.0;
        if (rd[0] < 0.0)
            rd[0] = rd[0] + 360.0;
        if (lu[0] < 0.0)
            lu[0] = lu[0] + 360.0;
        if (ru[0] < 0.0)
            ru[0] = ru[0] + 360.0;
        if (ld[0] > 360.0)
            ld[0] = ld[0] - 360.0;
        if (rd[0] > 360.0)
            rd[0] = rd[0] - 360.0;
        if (lu[0] > 360.0)
            lu[0] = lu[0] - 360.0;
        if (ru[0] > 360.0)
            ru[0] = ru[0] - 360.0;

        if (ld[1] < -90.0)
            ld[1] = -ld[1] + ld[1] + 89.75;
        if (ld[1] > 90.0)
            ld[1] = -ld[1] + ld[1] - 89.75;
        if (rd[1] < -90.0)
            rd[1] = -rd[1] + rd[1] + 89.75;
        if (rd[1] > 90.0)
            rd[1] = -rd[1] + rd[1] - 89.75;
        if (lu[1] < -90.0)
            lu[1] = -lu[1] + lu[1] + 89.75;
        if (lu[1] > 90.0)
            lu[1] = -lu[1] + lu[1] - 89.75;
        if (ru[1] < -90.0)
            ru[1] = -ru[1] + ru[1] + 89.75;
        if (ru[1] > 90.0)
            ru[1] = -ru[1] + ru[1] - 89.75;
        if (_rne_r.find(lu) == _rne_r.end() || _rne_r.find(ld) == _rne_r.end() || _rne_r.find(ru) == _rne_r.end() || _rne_r.find(rd) == _rne_r.end())
        {
            return -1;
        }
        if (_rne_i.find(lu) == _rne_i.end() || _rne_i.find(ld) == _rne_i.end() || _rne_i.find(ru) == _rne_i.end() || _rne_i.find(rd) == _rne_i.end())
        {
            return -1;
        }
        lu_r = _rne_r[lu];
        ld_r = _rne_r[ld];
        ru_r = _rne_r[ru];
        rd_r = _rne_r[rd];
        lu_i = _rne_i[lu];
        ld_i = _rne_i[ld];
        ru_i = _rne_i[ru];
        rd_i = _rne_i[rd];
        double lr = (ell[1] - ld[0]) / 0.5;
        double ud = (ell[0] - ld[1]) / 0.5;
        double tmp1, tmp2;
        for (int i = 0; i < 3; i++)
        {
            tmp1 = ld_r[i] + (lu_r[i] - ld_r[i]) * ud;
            tmp2 = rd_r[i] + (ru_r[i] - rd_r[i]) * ud;
            rne_r[i] = tmp1 + (tmp2 - tmp1) * lr;
            tmp1 = ld_i[i] + (lu_i[i] - ld_i[i]) * ud;
            tmp2 = rd_i[i] + (ru_i[i] - rd_i[i]) * ud;
            rne_i[i] = tmp1 + (tmp2 - tmp1) * lr;
        }
        return 1;
    }

    void gnss_all_opl::add(const double &lon, const double &lat, const Triple &rne_r, const Triple &rne_i)
    {
        base_pair ell;
        ell[0] = lon;
        ell[1] = lat;
        _rne_r[ell] = rne_r;
        _rne_i[ell] = rne_i;
    }
}