#include "hwa_ins_base_utility.h"
#include "hwa_set_ins.h"
using namespace hwa_ins;
using namespace hwa_set;

double hwa_ins::Lagrange(double t[], double x[], int n, double t1)
{
    double res = 0.0;
    double* L = new double[n];

    for (int k = 0; k < n; k++)
    {
        double up = 1.0, down = 1.0;
        for (int m = 0; m < n; m++)
        {
            if (m == k)
                continue;
            up *= (t1 - t[m]);
            down *= (t[k] - t[m]);
        }
        L[k] = up / down;
    }

    for (int i = 0; i < n; i++)
    {
        res += x[i] * L[i];
    }
    delete[] L;
    return res;
}

int hwa_ins::sign(double d)
{
    if (d > glv.EPS)
        return 1;
    else if (d < -glv.EPS)
        return -1;
    else
        return 0;
}

void hwa_ins::removeRow(Matrix& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if (rowToRemove < numRows)
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

    matrix.conservativeResize(numRows, numCols);
}

void hwa_ins::removeRow(Vector& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if (rowToRemove < numRows)
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

    matrix.conservativeResize(numRows, numCols);
}

void hwa_ins::removeColumn(Matrix& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols() - 1;

    if (colToRemove < numCols)
        matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

    matrix.conservativeResize(numRows, numCols);
}

hwa_ins::ins_scheme::ins_scheme()
{
}

hwa_ins::ins_scheme::ins_scheme(set_base* set) {
    t = 0.0;
    ts = dynamic_cast<set_ins*>(set)->ts();
    freq = dynamic_cast<set_ins*>(set)->freq();
    nSamples = dynamic_cast<set_ins*>(set)->subsample();
    Cps = dynamic_cast<set_ins*>(set)->cps();
    start = dynamic_cast<set_ins*>(set)->start();
    end = dynamic_cast<set_ins*>(set)->end();
    _imu_inst_rot = dynamic_cast<set_ign*>(set)->imu_inst_rot();
    _imu_inst_trans = dynamic_cast<set_ign*>(set)->imu_inst_trans();
    _imu_scale = dynamic_cast<set_ign*>(set)->imu_scale();
    delay = dynamic_cast<set_ign*>(set)->delay_t();
    delay_odo = dynamic_cast<set_ign*>(set)->delay_odo();
    max_pdop = dynamic_cast<set_ign*>(set)->max_pdop();
    min_sat = dynamic_cast<set_ign*>(set)->min_sat();
}