/**
*
*
* @file            ginverse.h
* @brief        header files of matrix inverse
*
*/

#ifndef HWA_BASE_INVERSE_NEWMAT_H
#define HWA_BASE_INVERSE_NEWMAT_H

#include "HWA_BASE_Inverse.h"
#include "HWA_BASE_Newmat.h"

namespace HWA_BASE_
{
    template <class MyMatrix>
    class BASE_INVERSE_NEWMAT : public BASE_INVERSE<MyMatrix>
    {
    public:
        BASE_INVERSE_NEWMAT(bool is_from_zero) : BASE_INVERSE<MyMatrix>(is_from_zero)
        {
        }

        virtual void sovle_NEQ(int num, MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X) override
        {
            Matrix neq(num, num);
            Matrix w(num, 1);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i + 1, j + 1) = NEQ(i + this->_beg, j + this->_beg);
                }
                w(i + 1, 1) = W(i + this->_beg, this->_beg);
            }

            // using Newmat Matrix Lib
            LowerTriangularMatrix L = Cholesky(neq);
            neq << L.i();
            neq << neq.t() * neq;
            w << neq * w;

            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = neq(i + 1, j + 1);
                }
                X(i + this->_beg, this->_beg) = w(i + 1, 1);
            }
        }

        virtual void inverse_NEQ(int num, MyMatrix &NEQ) override
        {
            Matrix neq(num, num);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i + 1, j + 1) = NEQ(i + this->_beg, j + this->_beg);
                }
            }

            LowerTriangularMatrix L = Cholesky(neq);
            neq << L.i();
            neq << neq.t() * neq;

            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = neq(i + 1, j + 1);
                }
            }
        }
    };

}
#endif // !GINVERSE_NEWMAT_H
