/**
*
* @file            ginverse.h
* @brief        header files of matrix inverse
*/

#ifndef HWA_BASE_INVERSE_EIGEN_H
#define HWA_BASE_INVERSE_EIGEN_H

#include "HWA_BASE_Inverse.h"
#include <Eigen/Dense>

namespace HWA_GNSS
{
    template <class MyMatrix>
    class BASE_INVERSE_Eigen_SVD : public BASE_INVERSE<MyMatrix>
    {
    public:
        BASE_INVERSE_Eigen_SVD(bool is_from_zero) : BASE_INVERSE<MyMatrix>(is_from_zero)
        {
        }

        virtual void sovle_NEQ(int num, MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X) override
        {
            Eigen::MatrixXd neq(num, num);
            Eigen::VectorXd w(num, 1);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i, j) = NEQ(i + this->_beg, j + this->_beg);
                }
                w(i, 0) = W(i + this->_beg, this->_beg);
            }
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(neq, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXd x = svd.solve(w);
            for (int i = 0; i < num; i++)
            {
                X(i + this->_beg, this->_beg) = x(i, 0);
            }
        }

        virtual void inverse_NEQ(int num, MyMatrix &NEQ) override
        {
            Eigen::MatrixXd neq(num, num);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i, j) = NEQ(i + this->_beg, j + this->_beg);
                }
            }
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(neq, Eigen::ComputeThinU | Eigen::ComputeThinV);

            Eigen::MatrixXd U = svd.matrixU();
            Eigen::MatrixXd V = svd.matrixV();
            Eigen::MatrixXd D = svd.singularValues();

            Eigen::MatrixXd S(V.cols(), U.cols());
            S.setZero();

            for (unsigned int i = 0; i < D.size(); ++i)
            {

                if (D(i, 0) > 0.0)
                {
                    S(i, i) = 1 / D(i, 0);
                }
                else
                {
                    S(i, i) = 0;
                }
            }

            Eigen::MatrixXd ans = V * S * U.transpose();
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = ans(i, j);
                }
            }
        }
    };

    template <class MyMatrix>
    class BASE_INVERSE_Eigen_Cholosky : public BASE_INVERSE<MyMatrix>
    {
    public:
        BASE_INVERSE_Eigen_Cholosky(bool is_from_zero) : BASE_INVERSE<MyMatrix>(is_from_zero)
        {
        }

        void solve_x(int num, const MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X) override
        {
            Eigen::MatrixXd neq(num, num);
            Eigen::VectorXd w(num, 1);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i, j) = NEQ(i + this->_beg, j + this->_beg);
                }
                w(i, 0) = W(i + this->_beg, this->_beg);
            }

            //Eigen::LLT<Eigen::MatrixXd> lltofNEQ(neq);
            Eigen::MatrixXd x = neq.llt().solve(w);

            for (int i = 0; i < num; i++)
            {
                X(i + this->_beg, this->_beg) = x(i, 0);
            }
        }

        virtual void sovle_NEQ(int num, MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X) override
        {
            Eigen::MatrixXd neq(num, num);
            Eigen::VectorXd w(num, 1);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i, j) = NEQ(i + this->_beg, j + this->_beg);
                }
                w(i, 0) = W(i + this->_beg, this->_beg);
            }

            Eigen::LLT<Eigen::MatrixXd> lltofNEQ(neq);
            Eigen::MatrixXd x = lltofNEQ.solve(w);
            Eigen::MatrixXd Li = lltofNEQ.matrixL();
            Li = Li.inverse();
            neq = Li.transpose() * Li;

            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = neq(i, j);
                }
                X(i + this->_beg, this->_beg) = x(i, 0);
            }
        }

        virtual void inverse_NEQ(int num, MyMatrix &NEQ) override
        {
            Eigen::MatrixXd neq(num, num);
            Eigen::VectorXd w(num, 1);
            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    neq(i, j) = NEQ(i + this->_beg, j + this->_beg);
                }
            }

            Eigen::LLT<Eigen::MatrixXd> lltofNEQ(neq);
            Eigen::MatrixXd Li = lltofNEQ.matrixL();
            Li = Li.inverse();
            neq = Li.transpose() * Li;

            for (int i = 0; i < num; i++)
            {
                for (int j = 0; j < num; j++)
                {
                    NEQ(i + this->_beg, j + this->_beg) = neq(i, j);
                }
            }
        }
    };

}

#endif // !GINVERSE_H
