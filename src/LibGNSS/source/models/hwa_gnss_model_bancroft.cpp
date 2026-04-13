#include <cmath>
#include <iomanip>
#include "hwa_base_const.h"
#include "hwa_base_eigendef.h"
#include "hwa_base_globaltrans.h"
#include "hwa_gnss_model_bancroft.h"
#include "hwa_set_gen.h"

using namespace hwa_base;

namespace hwa_gnss
{
    int gbancroft(const Matrix &BBpass, Vector &pos)
    {

        if (pos.rows() != 4)
        {
            pos.resize(4);
        }
        pos.setZero();

        for (int iter = 1; iter <= 2; iter++)
        {
            Matrix BB = BBpass;
            int mm = BB.rows();
            for (int ii = 0; ii < mm; ii++)
            {
                double xx = BB(ii, 0);
                double yy = BB(ii, 1);
                double traveltime = 0.072;
                if (iter > 1)
                {
                    double zz = BB(ii, 2);
                    double rho2 = (xx - pos(0)) * (xx - pos(0)) +
                                  (yy - pos(1)) * (yy - pos(1)) +
                                  (zz - pos(2)) * (zz - pos(2));
                    if (rho2 < 0)
                        return -1;

                    double rho = sqrt(rho2);

                    traveltime = rho / CLIGHT;
                }
                double angle = traveltime * OMEGA;
                double cosa = cos(angle);
                double sina = sin(angle);
                BB(ii, 0) = cosa * xx + sina * yy;
                BB(ii, 1) = -sina * xx + cosa * yy;
            }

            Matrix BBB;
            if (mm > 4)
            {
                Symmetric hlp;
                hlp.matrixW() = BB.transpose() * BB;
                BBB = hlp.matrixR().inverse() * BB.transpose();
            }
            else
            {
                BBB = BB.inverse();
            }
            Vector ee(mm);
            ee.setConstant(1.0);
            Vector alpha(mm);
            alpha.setConstant(1.0);
            for (int ii = 0; ii < mm; ii++)
            {
                alpha(ii) = lorentz(BB.row(ii).transpose(), BB.row(ii).transpose()) / 2.0;
            }
            Vector BBBe = BBB * ee;
            Vector BBBalpha = BBB * alpha;
            double aa = lorentz(BBBe, BBBe);
            double bb = lorentz(BBBe, BBBalpha) - 1;
            double cc = lorentz(BBBalpha, BBBalpha);

            double root2 = bb * bb - aa * cc;
            if (root2 < 0)
                return -1;
            double root = sqrt(root2);

            Matrix hlpPos(4, 2);
            hlpPos.col(0) = (-bb - root) / aa * BBBe + BBBalpha;
            hlpPos.col(1) = (-bb + root) / aa * BBBe + BBBalpha;

            Vector omc(2);
            for (int pp = 0; pp < 2; pp++)
            {
                hlpPos(3, pp) = -hlpPos(3, pp);

                double cost = 0.0;

                for (int i = 0; i < mm; i++)
                {
                    double dx = BB(i, 0) - hlpPos(0, pp);
                    double dy = BB(i, 1) - hlpPos(1, pp);
                    double dz = BB(i, 2) - hlpPos(2, pp);

                    double rho = sqrt(dx * dx + dy * dy + dz * dz);
                    double res = BB(i, 3) - rho - hlpPos(3, pp);
                    cost += res * res;

                }
                omc(pp) = cost;
            }

            if (fabs(omc(0) - omc(1)) < 1e-2) {
                double norm0 = hlpPos.col(0).head(3).norm();
                double norm1 = hlpPos.col(1).head(3).norm();

                double err0 = fabs(norm0 - R_SPHERE);
                double err1 = fabs(norm1 - R_SPHERE);

                pos = (err0 < err1) ? hlpPos.col(0) : hlpPos.col(1);
            }
            else {
                if (fabs(omc(0)) > fabs(omc(1)))
                {
                    pos = hlpPos.col(1);
                }
                else
                {
                    pos = hlpPos.col(0);
                }
            }
        }
        return 0;
    }

    double lorentz(const Vector &aa, const Vector &bb)
    {
        return aa(0) * bb(0) + aa(1) * bb(1) + aa(2) * bb(2) - aa(3) * bb(3);
    }

} // namespace
