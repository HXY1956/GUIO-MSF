#ifndef hwa_set_leo_h
#define hwa_set_leo_h
#define XMLKEY_LEO "leo"
#include "hwa_set_base.h"
using namespace pugi;
namespace hwa_gnss
{
    /** @brief full name for all supported LEO satellites. */
    enum GLEOLONG
    {
        CHAMP,
        //GNOS,
        HY_2A,
        TIANGONG_2,
        ZIYUAN_3,
        KOMPSAT5,
        LINGQIAO,

        FY_3C,
        FY_3D,

        JASON_2,
        JASON_3,

        LAGEOS_1,
        LAGEOS_2,

        GRACE_A,
        GRACE_B,
        GRACE_C,
        GRACE_D,

        METOP_A,
        METOP_B,
        METOP_C,

        SWARM_A,
        SWARM_B,
        SWARM_C,

        COSMIC_1,
        COSMIC_2,
        COSMIC_3,
        COSMIC_4,
        COSMIC_5,
        COSMIC_6,

        SENTINEL_1A,
        SENTINEL_1B,
        SENTINEL_2A,
        SENTINEL_2B,
        SENTINEL_3A,
        SENTINEL_3B,

        TERRASAR_X,
        TANDEM_X,

        PAZSAT,
        UNDEF
    };

    /** @brief abbreviation for all supported LEO satellites(four character, just like a station name) */
    enum GLEOSHORT
    {
        CHAM,
        HY2A,
        TG02,
        ZY03,
        KOM5,
        LING,
        TERX,

        FY3C,
        FY3D,

        JAS2,
        JAS3,

        GRAA,
        GRAB,
        GRAC,
        GRAD,

        META,
        METB,
        METC,

        SWAA,
        SWAB,
        SWAC,

        COS1,
        COS2,
        COS3,
        COS4,
        COS5,
        COS6,

        SE1A,
        SE1B,
        SE2A,
        SE2B,
        SE3A,
        SE3B,

        TESX,
        TADX,

        PAZS,
        UNDF
    };
};

namespace hwa_set{
    /**
    *@brief       Class for LEO settings
    */
    class gnss_set_leo : public virtual set_base
    {
    public:
        /** @brief constructor. */
        gnss_set_leo();

        /** @brief destructor. */
        virtual ~gnss_set_leo();

        /** @brief settings check. */
        void check();

        /** @brief settings help. */
        void help();

        /**
        * @brief return all LEO satellites in config file(string)
        * @return  all LEO satellites in config file(string)
        */
        std::set<std::string> sat();

        /**
        * @brief return all LEO satellites in config file(GLEOLONG enum)
        * @return  all LEO satellites in config file(GLEOLONG enum)
        */
        std::vector<hwa_gnss::GLEOLONG> esat();

    protected:
    };

    /**
    * @brief convert lower string to upper string of LEO satellite name
    * @param[in]  s  lower string name of satellite
    * @return     the upper string name of satellite
    */
    std::string low2upp(std::string s);

    /**
    * @brief convert lower string to upper string of LEO satellite name
    * @param[in]  s  upper string name of satellite
    * @return     the lower string name of satellite
    */
    std::string upp2low(std::string s);

    /**
    * @brief convert long string to GLEOLONG enum
    * @parma[in]  s   long string name of satellite
    * @return    the GLEOLONG name of satellite
    */
    hwa_gnss::GLEOLONG str2long(std::string s);

    /**
    * @brief convert GLEOSHORT enum to GLEOLONG
    * @param[in]  leo   GLEOSHORT name of satellite
    * @return      GLEOLONG name of satellite
    */
    hwa_gnss::GLEOLONG short2long(hwa_gnss::GLEOSHORT leo);

    /**
    * @brief convert short string to GLEOSHORT
    * @param[in]  s  short string name of satellite
    * @return     GLEOSHORT name of satellite
    */
    hwa_gnss::GLEOSHORT str2short(std::string s);

    /**
    * @brief convert GLEOLONG enum to GLEOSHORT
    * @param[in]  leo  GLEOLONG name of satellite
    * @return    GLEOSHORT name of satellite
    */
    hwa_gnss::GLEOSHORT long2short(hwa_gnss::GLEOLONG leo);

    /**
    * @brief convert GLEOLONG enum to long string
    * @param[in]  leo  GLEOLONG name of satellite
    * @return   long string name of satellite
    */
    std::string long2str(hwa_gnss::GLEOLONG leo);

    /**
    * @brief convert GLEOSHORT enum to short string
    * @param[in]  leo  GLEOSHORT name of satellite
    * @return      short string name of satellite
    */
    std::string short2str(hwa_gnss::GLEOSHORT leo);

    /**
    * @brief convert short string to long string
    * @param[in]  s  short string name of satellite
    * @return   long string name of satellite
    */
    std::string strs2strl(std::string s);

    /**
    * @brief convert long string to short string
    * param[in]  s  long string name of satellite
    * @return  short string name of satellite
    */
    std::string strl2strs(std::string s);

    /**
    * @brief get all supported LEO satellites
    * @return  all supported LEO satellites(GLEOLONG name)
    */
    std::set<hwa_gnss::GLEOLONG> LEO_SUPPORTED();

    /**
    * @brief judge the LEO satellite is supported LEO or not
    * @param[in]  s  long string name of satellite
    * @return   bool
    *    @retval   true   the satellite is supported LEO
    *   @retval   false   the satellite isnot supported LEO
    */
    bool IS_SUPPORTED_LEO(std::string s);
} // namespace

#endif