#ifndef hwa_set_param_h
#define hwa_set_param_h
#define XMLKEY_PARS "parameters"
#define XMLKEY_PARS_ERP "ERP"
#define XMLKEY_PARS_GEO "GEO"
#define XMLKEY_PARS_SAT "SAT"
#define XMLKEY_PARS_STA "STA"
#define XMLKEY_PARS_AMB "AMB"
#define XMLKEY_PARS_SRP "SRP"
#include "hwa_set_base.h"

namespace hwa_set
{
    class set_par : public virtual set_base
    {
    public:
        /** @brief default constructor. */
        set_par();

        /** @brief default destructor. */
        virtual ~set_par();

        /**
        * @brief  get priori sigma in X direction of GEO Cencter parameter.
        * @return    double     priori sigma in X direction of GEO Cencter parameter
        */
        double sigCX();

        /**
        * @brief  get priori sigma in Y direction of GEO Cencter parameter.
        * @return    double     priori sigma in Y direction of GEO Cencter parameter
        */
        double sigCY();

        /**
        * @brief  get priori sigma in Z direction of GEO Cencter parameter.
        * @return    double     priori sigma in Z direction of GEO Cencter parameter
        */
        double sigCZ();

        /**
        * @brief  get priori sigma in X direction of pole parameter.
        * @return    double     priori sigma in X direction of pole parameter
        */
        double sigXpole();

        /**
        * @brief  get priori sigma in Y direction of pole parameter.
        * @return    double     priori sigma in Y direction of pole parameter
        */
        double sigYpole();

        /**
        * @brief  get priori sigma of dX of pole parameter.
        * @return    double     priori sigma of dX of pole parameter
        */
        double sigDxpole();

        /**
        * @brief  get priori sigma of dY of pole parameter.
        * @return    double     priori sigma of dY of pole parameter
        */
        double sigDypole();

        /**
        * @brief  get priori sigma of UT1.
        * @return    double     priori sigma of UT1
        */
        double sigUt1();

        /**
        * @brief  get priori sigma of UT1(for VLBI).
        * @return    double     priori sigma of UT1(for VLBI)
        */
        double sigUt1_vlbi();

        /**
        * @brief  get priori sigma of dX of pole parameter(for VLBI).
        * @return    double     priori sigma of dX of pole parameter(for VLBI)
        */
        double sigDX_vlbi();

        /**
        * @brief  get priori sigma of dY of pole parameter(for VLBI).
        * @return    double     priori sigma of dY of pole parameter(for VLBI)
        */
        double sigDY_vlbi();

        /**
        * @brief  get priori sigma of dUT1.
        * @return    double     priori sigma of dUT1
        */
        double sigDut1();

        /**
        * @brief  get priori sigma of ambiguity.
        * @return    double     priori sigma of ambiguity
        */
        double sigAmb();

        /**
        * @brief  get priori sigma of ztd.
        * @return    double     priori sigma of ztd
        */
        double sigZtd(std::string rec);

        /**
        * @brief  get priori sigma of sion.
        * @return    double     priori sigma of sion
        */
        double sigSion(std::string rec);

        /**
        * @brief  get priori sigma of vion.
        * @return    double     priori sigma of vion
        */
        double sigVion(std::string rec);

        /**
        * @brief  get priori sigma of TropPd.
        * @return    double     priori sigma of TropPd
        */
        double sigTropPd(std::string rec);

        /**
        * @brief  get priori sigma of IonoPd.
        * @return    double     priori sigma of IonoPd
        */
        double sigIonoPd(std::string rec);

        /**
        * @brief  get priori sigma of gradient.
        * @return    double     priori sigma of gradient
        */
        double sigGRD(std::string rec); // add by glfeng

        /**
        * @brief  get priori sigma of GrdPd.
        * @return    double     priori sigma of GrdPd
        */
        double sigGrdPd(std::string rec); // add by glfeng

        /**
        * @brief  get priori sigma of receiver clock offset.
        * @return    double     priori sigma of receiver clock offset
        */
        double sigRecCLK(std::string rec);

        /**
        * @brief  get priori sigma of satellite clock offset.
        * @return    double     priori sigma of satellite clock offset
        */
        double sigSatCLK(std::string sat);

        /**
        * @brief  get priori sigma of range bias(for SLR).
        * @return    double     priori sigma of range bias(for SLR)
        */
        double sigRB();

        /**
        * @brief  get priori sigma of receiver position.
        * @return    std::map<std::string, double>     priori sigma of receiver position
        */
        std::map<std::string, double> sigRecPos(std::string sta_name);

        /**
        * @brief  get priori sigma of satellite position.
        * @return    std::map<std::string, double>     priori sigma of satellite position
        */
        std::map<std::string, double> sigSatPos(std::string sat_name);

        /**
        * @brief  get priori sigma of receiver velocity.
        * @return    std::map<std::string, double>     priori sigma of receiver velocity
        */
        std::map<std::string, double> sigSatVel(std::string sat_name);

        /**
        * @brief  get priori sigma of solar radiation pressure parameters(ECOM).
        * @return    std::map<std::string, double>     priori sigma of solar radiation pressure parameters(ECOM)
        */
        std::map<std::string, double> sigSatEcom(std::string sat_name);

        /**
        * @brief  get priori sigma of solar radiation pressure parameters(ABW, adjustable box-wing).
        * @return    std::map<std::string, double>     priori sigma of solar radiation pressure parameters(ABW, adjustable box-wing)
        */
        std::map<std::string, double> sigSatAbw(std::string sat_name);

        // add for LEO processing by zhangwei
        /**
        * @brief  get priori sigma of LEO satellite position.
        * @return    std::map<std::string, double>     priori sigma of LEO satellite position
        */
        std::map<std::string, double> sigLeoPos(std::string sta_name);

        /**
        * @brief  get priori sigma of LEO satellite velocity.
        * @return    std::map<std::string, double>     priori sigma of LEO satellite velocity
        */
        std::map<std::string, double> sigLeoVel(std::string sat_name);

        /**
        * @brief  get priori sigma of LEO satellite dynamic parameters.
        * @return    double     priori sigma of LEO satellite dynamic parameters
        */
        double sigLeoDyn(std::string sat_name);

        void help();

        /**
        * @brief settings check.
        */
        void check(){};

    private:
        /**
        * @brief get child value.
        * @param[in]    child    child node
        * @return        std::string    value of child node
        */
        std::string _child_value(const std::string &child);

        /**
        * @brief get child node's attribute value.
        * @param[in]    child    child node
        * @param[in]    child    child node's attribute
        * @return        std::string    value of child node's attribute
        */
        std::string _child_attribute_value(const std::string &child, const std::string &attribute);

        /**
        * @brief get attribute value.
        * @param[in]    index_name    TODO
        * @param[in]    index_value    TODO
        * @param[in]    attribute    TODO
        * @return        std::string        attribute value
        */
        std::string _attribute_value(const std::string &index_name, const std::string &index_value, const std::string &attribute);

        /**
        * @brief split std::string.
        * @param[in]    s        std::string
        * @param[in]    delim    delimiter
        * @param[out]    ret        results
        */
        void split(const std::string &s, std::string delim, std::vector<std::string> &ret);
        // default settings

        // for GEO
        const double _default_sigCX = 0.001; ///< default priori sigma in X direction of GEO Cencter parameter
        const double _default_sigCY = 0.001; ///< default priori sigma in Y direction of GEO Cencter parameter
        const double _default_sigCZ = 0.001; ///< default priori sigma in Z direction of GEO Cencter parameter

        // for ERP
        const double _default_sigXPOLE = 0.300;    ///< default priori sigma in X direction of pole parameter
        const double _default_sigYPOLE = 0.300;    ///< default priori sigma in Y direction of pole parameter
        const double _default_sigDXPOLE = 0.030;   ///< default priori sigma of dX of pole parameter
        const double _default_sigDYPOLE = 0.030;   ///< default priori sigma of dY of pole parameter
        const double _default_sigUT1 = 1E-4;       ///< default priori sigma of UT1
        const double _default_sigUT1_vlbi = 0.300; ///< default priori sigma of UT1(for VLBI)
        const double _default_sigDUT1 = 2E-3;      ///< default priori sigma of dUT1
        const double _default_sigDX = 0.300;       ///< default priori sigma of dX of pole parameter
        const double _default_sigDY = 0.300;       ///< default priori sigma of dY of pole parameter

        // for ZTD/ION
        const double _default_sigZTD = 0.201;    ///< default priori sigma of ztd
        const double _default_sigSION = 9000;    ///< default priori sigma of sion, 1.000 to 9000
        const double _default_sigGpsIfb = 9000;  ///< default priori sigma of GPS IFB
        const double _default_sigVION = 9000;    ///< default priori sigma of vion, 1.000 to 9000
        const double _default_sigTropPd = 0.015; ///< default priori sigma of TropPd
        const double _default_sigIonoPd = 0.15;  ///< default priori sigma of IonoPd
        const double _default_sigGRD = 0.001;    ///< default priori sigma of gradient, add by glfeng, unit m, temp
        const double _default_sigGrdPd = 0.0003; ///< default priori sigma of GrdPd, add by glfeng, unit m, temp

        // for AMB
        const double _default_sigAMB = 10000; ///< default priori sigma of ambiguity

        // for CLK
        const double _default_sta_sigCLK = 9000; ///< default priori sigma of receiver clock offset
        const double _default_sat_sigCLK = 5000; ///< default priori sigma of satellite clock offset
        const double _default_sta_sigRB = 0.1;   ///< default priori sigma of range bias(for SLR)

        // for POS
        const double _default_sta_sigPOS = 0.1; ///< default priori sigma of receiver position
        const double _default_sat_sigPOS = 10;  ///< default priori sigma of satellite position
        const double _default_leo_sigPOS = 100; ///< default priori sigma of LEO satellite position, for LEO processing: unit: m by zhangwei

        // for VEL
        const double _default_sat_sigVEL = 0.10; ///< default priori sigma of satellite velocity
        const double _default_leo_sigVEL = 1;    ///< default priori sigma of LEO satellite velocity, for LEO processing: unit: m/s by zhangwei

        // for solar pars
        const double _default_sat_sigECOM = 0.10;   ///< default priori sigma of solar radiation pressure parameters(ECOM)
        const double _default_sat_sigABW_SY = 1E20; ///< default priori sigma of solar radiation pressure parameters(ABW, adjustable box-wing)
        const double _default_sat_sigABW_AD = 0.01; ///< default priori sigma of solar radiation pressure parameters(ABW, adjustable box-wing)
        const double _default_sat_sigABW_R = 0.1;   ///< default priori sigma of solar radiation pressure parameters(ABW, adjustable box-wing)
        const double _default_leo_sigDYN = 0.10;    ///< default priori sigma of LEO satellite dynamic parameters, for LEO processing
    };
}

#endif