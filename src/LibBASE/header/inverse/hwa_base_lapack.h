/**
*
*
* @file            ginverse.h
* @brief        header files of matrix inverse
*
*/

#ifdef USE_CLAPACK

#ifndef HWA_BASE_INVERSE_LAPACK_H
#define HWA_BASE_INVERSE_LAPACK_H

#include "f2c.h"
#include "clapack.h"
#include "blaswrap.h"
#include <stdexcept>
#include "HWA_BASE_Inverse.h"

namespace HWA_BASE
{
    template <class MyMatrix>
    class BASE_INVERSE_LAPACK : public BASE_INVERSE<MyMatrix>
    {
    public:
        BASE_INVERSE_LAPACK(bool is_from_zero) : BASE_INVERSE<MyMatrix>(is_from_zero)
        {
        }

        /**
        * @brief solve the NEQ * X = W
        * @param[in] num the number of dimension
        * @param[in] NEQ dimension:num*num
        * @note result save into the dx
        */
        virtual void sovle_NEQ(int num, MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X)
        {
            char U = 'U';
            integer Num = num;
            integer One = 1;
            integer error_info = 0;

            double *neq = new double[num * num];
            double *w = new double[num];

            // input
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq[i * num + j] = NEQ(i + this->_beg, j + this->_beg);
                }
                w[i] = W(i + this->_beg, this->_beg);
            }

            // Cholosky
            dpotrf_(&U, &Num, neq, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // Solve X
            dpotrs_(&U, &Num, &One, neq, &Num, w, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // Inverse NEQ
            dpotri_(&U, &Num, neq, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // output
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = neq[i * num + j];
                }
                X(i + this->_beg, this->_beg) = w[i];
            }

            delete[] neq;
            delete[] w;
        }

        virtual void solve_x(int num, const MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X)
        {
            char U = 'U';
            integer Num = num;
            integer One = 1;
            integer error_info = 0;

            double *neq = new double[num * num];
            double *w = new double[num];

            // input
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq[i * num + j] = NEQ(i + this->_beg, j + this->_beg);
                }
                w[i] = W(i + this->_beg, this->_beg);
            }

            // Cholosky
            dpotrf_(&U, &Num, neq, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // Solve X
            dpotrs_(&U, &Num, &One, neq, &Num, w, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // output
            for (int i = 0; i < num; i++)
            {
                X(i + this->_beg, this->_beg) = w[i];
            }

            delete[] neq;
            delete[] w;
        }

        virtual void inverse_NEQ(int num, MyMatrix &NEQ)
        {
            char U = 'U';
            integer Num = num;
            integer One = 1;
            integer error_info = 0;

            double *neq = new double[num * num];

            // input
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq[i * num + j] = NEQ(i + this->_beg, j + this->_beg);
                }
            }

            // Cholosky
            dpotrf_(&U, &Num, neq, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // Inverse NEQ
            dpotri_(&U, &Num, neq, &Num, &error_info);
            if (error_info != 0)
            {
                throw std::runtime_error("Lapack Solve Wrong!");
            }

            // output
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = neq[i * num + j];
                }
            }

            delete[] neq;
        }
    };
}

#endif // !GINVERSE_LAPACK_H

#endif // USE_CLPACK