
/**
*
* @file        gtropo.h
* @brief    Purpose: implements troposphere model class
*/

#ifndef hwa_gnss_model_tropo_H
#define hwa_gnss_model_tropo_H

#include "hwa_base_time.h"
#include "hwa_gnss_prod_crd.h"
#include "hwa_gnss_model_gpt.h"

namespace hwa_gnss
{

    /** @brief class for gnss_model_tropo. */
    class gnss_model_tropo
    {
    public:
        //gnss_model_tropo(string site);

        /** @brief default constructor. */
        gnss_model_tropo();

        /** @brief default destructor. */
        virtual ~gnss_model_tropo();

        /** @brief get ZHD. */
        virtual double getZHD(const Triple &ell, const base_time &epo); // ! Radians: Ell[0] and Ell[1]

        /** @brief get ZWD. */
        virtual double getZWD(const Triple &ell, const base_time &epo); // ! Radians: Ell[0] and Ell[1]

    protected:
        gnss_model_gpt _gpt; ///< gpt
    };

    /** @brief class for gnss_model_tropo_SAAST derive from gnss_model_tropo. */
    class gnss_model_tropo_SAAST : public gnss_model_tropo
    {
    public:
        /** @brief default constructor. */
        gnss_model_tropo_SAAST() {}

        /** @brief default destructor. */
        ~gnss_model_tropo_SAAST() {}

        /** @brief get STD/ZHD/ZWD. */
        virtual double getSTD(const double &ele, const double &hel);     // ! Radians: elevation
        virtual double getZHD(const Triple &ell, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const Triple &ell, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for gnss_model_tropo_davis derive from gnss_model_tropo. */
    class gnss_model_tropo_davis : public gnss_model_tropo
    {
    public:
        /** @brief default constructor. */
        gnss_model_tropo_davis() {}

        /** @brief default destructor. */
        ~gnss_model_tropo_davis() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for gnss_model_tropo_hopf derive from gnss_model_tropo. */
    class gnss_model_tropo_hopf : public gnss_model_tropo
    {
    public:
        /** @brief default constructor. */
        gnss_model_tropo_hopf() {}

        /** @brief default destructor. */
        ~gnss_model_tropo_hopf() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for gnss_model_tropo_baby derive from gnss_model_tropo. */
    class gnss_model_tropo_baby : public gnss_model_tropo
    {
    public:
        /** @brief default constructor. */
        gnss_model_tropo_baby() {}

        /** @brief default destructor. */
        ~gnss_model_tropo_baby() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for gnss_model_tropo_chao derive from gnss_model_tropo. */
    class gnss_model_tropo_chao : public gnss_model_tropo
    {
    public:
        /** @brief default constructor. */
        gnss_model_tropo_chao() {}

        /** @brief default destructor. */
        ~gnss_model_tropo_chao() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for gnss_model_tropo_ifad derive from gnss_model_tropo. */
    class gnss_model_tropo_ifad : public gnss_model_tropo
    {
    public:
        /** @brief default constructor. */
        gnss_model_tropo_ifad() {}

        /** @brief default destructor. */
        ~gnss_model_tropo_ifad() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const Triple &ele, const base_time &epo); // ! Radians: Ell[0] and Ell[1]
    };

} // namespace

#endif
