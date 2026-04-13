#include "hwa_gnss_model_base.h"

namespace hwa_gnss
{
    gnss_model_base_equation::gnss_model_base_equation()
    {
    }

    gnss_model_base_equation::~gnss_model_base_equation()
    {
    }

    gnss_model_base_equation &gnss_model_base_equation::operator+(gnss_model_base_equation &Other)
    {
        this->B.insert(this->B.begin(), Other.B.begin(), Other.B.end());
        this->P.insert(this->P.begin(), Other.P.begin(), Other.P.end());
        this->l.insert(this->l.begin(), Other.l.begin(), Other.l.end());
        return *this;
    }

    gnss_model_base::gnss_model_base()
    {
    }

    gnss_model_base::~gnss_model_base()
    {
    }
}