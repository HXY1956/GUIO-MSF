/**
*
* @file            ginverse.h
* @brief        header files of matrix inverse
*/

#ifndef HWA_BASE_INVERSE_H
#define HWA_BASE_INVERSE_H

namespace HWA_BASE
{
    template <class MyMatrix>
    class BASE_INVERSE
    {
    public:
        BASE_INVERSE(bool is_from_zero)
        {
            _beg = is_from_zero ? 0 : 1;
        }

        virtual void solve_x(int num, const MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X) = 0;
        virtual void sovle_NEQ(int num, MyMatrix &NEQ, const MyMatrix &W, MyMatrix &X) = 0;
        virtual void inverse_NEQ(int num, MyMatrix &NEQ) = 0;

    protected:
        int _beg;
    };

}
#endif // !GINVERSE_H
