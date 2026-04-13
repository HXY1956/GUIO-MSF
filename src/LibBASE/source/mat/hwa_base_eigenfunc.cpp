#include "HWA_BASE_EigenFunc.h"

using namespace std;

namespace HWA_BASE
{
    void Matrix::Matrix_remRC(int row, int col)
    {
        Matrix Qt = *this;
        this->resize(this->rows() - (row >= 0), this->cols() - (col >= 0));
        int rr = 0;
        for (int r = 0; r < Qt.rows(); r++)
        {
            if (r == row)
                continue;
            int cc = 0;

            for (int c = 0; c < Qt.cols(); c++)
            {
                if (c == col)
                    continue;
                this->operator()(rr, cc) = Qt(r, c);
                cc++;
            }
            rr++;
        }
    }

    void Matrix::Matrix_rem(vector<int>& ind)
    {
        vector<int>::iterator it;
        vector<int>::iterator it2;
        for (it = ind.begin(); it != ind.end(); it++)
        {
            Matrix_remRC(*it, *it);
            for (it2 = it; it2 != ind.end(); it2++)
                (*it2)--;
        }
    }

    void Matrix::Matrix_addRC(int row, int col)
    {
        Matrix Qt = *this;
        this->resize(this->rows() + (row>=0), this->cols() + (col >= 0));
        this->setZero();
        int rr = 0;
        for (int r = 0; r < this->rows(); r++)
        {
            if (r == row)
                continue;
            int cc = 0;

            for (int c = 0; c < this->cols(); c++)
            {
                if (c == col)
                    continue;
                this->operator()(r, c) = Qt(rr, cc);
                cc++;
            }
            rr++;
        }
    }

    void Matrix::Matrix_remR(int row)
    {
        int nrow = this->rows(), ncols = this->cols();
        if (row < 0) return;
        const Matrix At = *this;
        this->resize(nrow - 1, ncols);
        this->setZero();
        this->block(0, 0, row, ncols) = At.block(0, 0, row, ncols);
        if (row < nrow - 1) {
            this->block(row, 0, nrow - row - 1, ncols) = At.block(row + 1, 0, nrow - row - 1, ncols);
        }
    }

    void Matrix::Matrix_remR(vector<int>& ind)
    {
        vector<int>::iterator it;
        vector<int>::iterator it2;
        for (it = ind.begin(); it != ind.end(); it++)
        {
            Matrix_remR(*it);
            for (it2 = it; it2 != ind.end(); it2++)
                (*it2)--;
        }
    }

    Matrix Matrix::rotX(double Angle)
    {
        const double C = cos(Angle);
        const double S = sin(Angle);
        Matrix UU(3, 3);
        UU[0][0] = 1.0;
        UU[0][1] = 0.0;
        UU[0][2] = 0.0;
        UU[1][0] = 0.0;
        UU[1][1] = +C;
        UU[1][2] = +S;
        UU[2][0] = 0.0;
        UU[2][1] = -S;
        UU[2][2] = +C;
        return UU;
    }

    Matrix Matrix::rotY(double Angle)
    {
        const double C = cos(Angle);
        const double S = sin(Angle);
        Matrix UU(3, 3);
        UU[0][0] = +C;
        UU[0][1] = 0.0;
        UU[0][2] = -S;
        UU[1][0] = 0.0;
        UU[1][1] = 1.0;
        UU[1][2] = 0.0;
        UU[2][0] = +S;
        UU[2][1] = 0.0;
        UU[2][2] = +C;
        return UU;
    }

    Matrix Matrix::rotZ(double Angle)
    {
        const double C = cos(Angle);
        const double S = sin(Angle);
        Matrix UU(3, 3);
        UU[0][0] = +C;
        UU[0][1] = +S;
        UU[0][2] = 0.0;
        UU[1][0] = -S;
        UU[1][1] = +C;
        UU[1][2] = 0.0;
        UU[2][0] = 0.0;
        UU[2][1] = 0.0;
        UU[2][2] = 1.0;
        return UU;
    }

    void Symmetric::Matrix_remRC(int row, int col)
    {
        assert(row == col);
        if (row < 0) return;
        Symmetric Qt = *this;
        this->resize(this->rows() - 1);
        int rr = 0;
        for (int r = 0; r < Qt.rows(); r++)
        {
            if (r == row)
                continue;
            int cc = 0;

            for (int c = 0; c < Qt.cols(); c++)
            {
                if (c == col)
                    continue;
                this->set(Qt(r, c), rr, cc);
                cc++;
            }
            rr++;
        }
    }

    void Symmetric::Matrix_rem(vector<int>& ind)
    {
        vector<int>::iterator it;
        vector<int>::iterator it2;
        for (it = ind.begin(); it != ind.end(); it++)
        {
            Matrix_remRC(*it, *it);
            for (it2 = it; it2 != ind.end(); it2++)
                (*it2)--;
        }
    }

    void Symmetric::Matrix_addRC(int row, int col)
    {
        assert(row == col);
        if (row < 0) return;
        Symmetric Qt = *this;
        this->resize(this->rows() + 1);
        this->setZero();
        for (int i = 0, rr = 0; i < this->rows(); ++i) {
            if (i == row) continue;
            for (int j = 0, cc = 0; j <= i; ++j) {
                if (j == col) continue;
                this->set(Qt(rr, cc), i, j);
                ++cc;
            }
            ++rr;
        }
    }

    void Symmetric::Matrix_swap(int a, int b)
    {
        Symmetric T = *this;
        for (int i = 0; i < cols(); i++)
        {
            double temp = T(a, i);
            T(a, i) = T(b, i);
            T(b, i) = temp;
        }
        for (int i = 0; i < rows(); i++)
        {
            double temp = T(i, a);
            T(i, a) = T(i, b);
            T(i, b) = temp;
        }
        *this = T;
    }

    void Symmetric::Matrix_cpRC(const Symmetric& Q, int r, int c)
    {
        assert(Q.rows() >= rows() && Q.cols() >= cols());
        if (r < rows()) {
            for (int i = 0; i < cols(); i++) this->set(Q(r, i), r, i);
        }
        if (c < cols()) {
            for (int i = 0; i < rows(); i++) this->set(Q(i, c), i, c);
        }
    }

    void Diagonal::Matrix_remRC(int row)
    {
        const int n = this->rows();
        if (row < 0 || row >= n) return;
        const Vector d = this->diagonal();
        Vector newD(n - 1);
        newD.head(row) = d.head(row);
        newD.tail(n - 1 - row) = d.tail(n - row - 1);
        this->resize(n - 1);
        this->set(newD);
    }

    void Diagonal::Matrix_addRC(int row)
    {
        const int n = rows();
        if (row < 0 || row >= n) return;
        const Vector d = this->diagonal();
        Vector newD(n + 1);
        newD.setZero();
        newD.head(row) = d.head(row);
        newD.tail(n - row) = d.tail(n - row);
        this->resize(n + 1);
        this->set(newD);
    }

    void Vector::addR(int row)
    {
        if (row < 0 || row > this->rows()) return;
        const Vector Vt = *this;
        this->resize(this->rows() + 1);
        this->setZero();

        int rr = 0;
        for (int r = 0; r < this->rows(); r++)
        {
            if (r == row)
                continue;

            this->operator()(r) = Vt(rr);
            rr++;
        }
    }
} // namespace