#include "hwa_set_leo.h"
#include "hwa_set_gbase.h"
#include "hwa_base_mutex.h"
#include "hwa_base_string.h"
#include <algorithm>

using namespace std;
using namespace hwa_gnss;
using namespace hwa_base;

namespace hwa_set
{
    /** @brief constructor. */
    gnss_set_leo::gnss_set_leo()
        : set_base()
    {
        _set.insert(XMLKEY_LEO);
    }

    /** @brief destructor. */
    gnss_set_leo::~gnss_set_leo()
    {
    }

    /** @brief settings check. */
    void gnss_set_leo::check()
    {
    }

    /** @brief settings help. */
    void gnss_set_leo::help()
    {
    }

    /**
    * @brief return all LEO satellites in config file(string)
    * @return  all LEO satellites in config file(string)
    */
    set<string> gnss_set_leo::sat()
    {
        try
        {
            set<string> allsat;
            set<string>::const_iterator it;
            set<string> sats = set_base::_setval("LEO", "sat");
            for (it = sats.begin(); it != sats.end(); ++it)
            {
                if (IS_SUPPORTED_LEO(*it))
                {
                    allsat.insert(*it);
                }
            }
            return allsat;
        }
        catch (...)
        {
            cout << "ERROR: gnss_set_leo::sat  unknown mistake" << endl;
            return set<string>();
        }
    }

    /**
    * @brief return all LEO satellites in config file(GLEOLONG enum)
    * @return  all LEO satellites in config file(GLEOLONG enum)
    */
    vector<GLEOLONG> gnss_set_leo::esat()
    {
        try
        {
            vector<GLEOLONG> allsat;
            set<string>::const_iterator it;
            set<string> sats = set_base::_setval("LEO", "sat");
            for (it = sats.begin(); it != sats.end(); ++it)
            {
                if (IS_SUPPORTED_LEO(*it))
                {
                    allsat.push_back(str2long(*it));
                }
            }
            return allsat;
        }
        catch (...)
        {
            cout << "ERROR: gnss_set_leo::esat  unknown mistake" << endl;
            return vector<GLEOLONG>();
        }
    }

    /**
    * @brief convert lower string to upper string of LEO satellite name
    * @param[in]  s  lower string name of satellite
    * @return     the upper string name of satellite
    */
    string low2upp(string s)
    {
        transform(s.begin(), s.end(), s.begin(), ::toupper);
        return s;
    }

    /**
    * @brief convert lower string to upper string of LEO satellite name
    * @param[in]  s  upper string name of satellite
    * @return     the lower string name of satellite
    */
    string upp2low(string s)
    {
        transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s;
    }

    /**
    * @brief convert long string to GLEOLONG enum
    * @parma[in]  s   long string name of satellite
    * @return    the GLEOLONG name of satellite
    */
    GLEOLONG str2long(string s)
    {
        string name = low2upp(s);
        int len = trim(name).length();
        if (len < 4)
        {
            // �������ǲ�վҲ�ᱨ�������û������
            //cerr << "Wrong input LEO satellite name: "
            //    << name
            //    << endl;
            return UNDEF;
        }
        string strleo = name.substr(0, 4);
        string type1 = name.substr(len - 1, 1);
        string type2 = name.substr(len - 2, 2);
        if (strleo == "CHAM")
            return CHAMP;
        //if(strleo == "GNOS") return GNOS;
        if (strleo == "HY-2")
        {
            if (type1 == "A")
                return HY_2A;
        }
        if (strleo == "TIAN")
            return TIANGONG_2;
        if (strleo == "ZIYU")
        {
            if (type1 == "3")
                return ZIYUAN_3;
        }
        if (strleo == "KOMP")
        {
            if (type1 == "5")
                return KOMPSAT5;
        }
        if (strleo == "LING")
            return LINGQIAO;
        if (strleo == "TERR")
            return TERRASAR_X;
        if (strleo == "TAND")
            return TANDEM_X;
        if (strleo == "FY-3")
        {
            if (type1 == "C")
                return FY_3C;
            if (type1 == "D")
                return FY_3D;
        }
        if (strleo == "JASO")
        {
            if (type1 == "2")
                return JASON_2;
            if (type1 == "3")
                return JASON_3;
        }
        if (strleo == "LAGE")
        {
            if (type1 == "1")
                return LAGEOS_1;
            if (type1 == "2")
                return LAGEOS_2;
        }
        if (strleo == "GRAC")
        {
            if (type1 == "A")
                return GRACE_A;
            if (type1 == "B")
                return GRACE_B;
            if (type1 == "C")
                return GRACE_C;
            if (type1 == "D")
                return GRACE_D;
        }
        if (strleo == "METO")
        {
            if (type1 == "A")
                return METOP_A;
            if (type2 == "B")
                return METOP_B;
            if (type2 == "C")
                return METOP_C;
        }
        if (strleo == "SWAR")
        {
            if (type1 == "A")
                return SWARM_A;
            if (type1 == "B")
                return SWARM_B;
            if (type1 == "C")
                return SWARM_C;
        }
        if (strleo == "COSM")
        {
            if (type1 == "1")
                return COSMIC_1;
            if (type1 == "2")
                return COSMIC_2;
            if (type1 == "3")
                return COSMIC_3;
            if (type1 == "4")
                return COSMIC_4;
            if (type1 == "5")
                return COSMIC_5;
            if (type1 == "6")
                return COSMIC_6;
        }
        if (strleo == "SENT")
        {
            if (type2 == "1A")
                return SENTINEL_1A;
            if (type2 == "1B")
                return SENTINEL_1B;
            if (type2 == "2A")
                return SENTINEL_2A;
            if (type2 == "2B")
                return SENTINEL_2B;
            if (type2 == "3A")
                return SENTINEL_3A;
            if (type2 == "3B")
                return SENTINEL_3B;
        }
        if (strleo == "PAZS")
            return PAZSAT;

        return UNDEF;
    }

    /**
    * @brief convert GLEOSHORT enum to GLEOLONG
    * @param[in]  leo   GLEOSHORT name of satellite
    * @return      GLEOLONG name of satellite
    */
    GLEOLONG short2long(GLEOSHORT leo)
    {
        switch (leo)
        {
        case CHAM:
            return CHAMP;
        case TG02:
            return TIANGONG_2;
        case HY2A:
            return HY_2A;
        case ZY03:
            return ZIYUAN_3;
        case KOM5:
            return KOMPSAT5;
        case LING:
            return LINGQIAO;
        case TESX:
            return TERRASAR_X;
        case FY3C:
            return FY_3C;
        case FY3D:
            return FY_3D;
        case JAS2:
            return JASON_2;
        case JAS3:
            return JASON_3;
        case GRAA:
            return GRACE_A;
        case GRAB:
            return GRACE_B;
        case GRAC:
            return GRACE_C;
        case GRAD:
            return GRACE_D;
        case META:
            return METOP_A;
        case METB:
            return METOP_B;
        case METC:
            return METOP_C;
        case SWAA:
            return SWARM_A;
        case SWAB:
            return SWARM_B;
        case SWAC:
            return SWARM_C;
        case COS1:
            return COSMIC_1;
        case COS2:
            return COSMIC_2;
        case COS3:
            return COSMIC_3;
        case COS4:
            return COSMIC_4;
        case COS5:
            return COSMIC_5;
        case COS6:
            return COSMIC_6;
        case SE1A:
            return SENTINEL_1A;
        case SE1B:
            return SENTINEL_1B;
        case SE2A:
            return SENTINEL_2A;
        case SE2B:
            return SENTINEL_2B;
        case SE3A:
            return SENTINEL_3A;
        case SE3B:
            return SENTINEL_3B;
        case TADX:
            return TANDEM_X;
        case PAZS:
            return PAZSAT;
        default:
            return UNDEF;
        }
        return UNDEF;
    }

    /**
    * @brief convert short string to GLEOSHORT
    * @param[in]  s  short string name of satellite
    * @return     GLEOSHORT name of satellite
    */
    GLEOSHORT str2short(string s)
    {
        string name = low2upp(s);
        int len = trim(name).length();
        if (len < 4)
        {
            cerr << "Wrong input LEO satellite name: "
                 << name
                 << endl;
            return UNDF;
        }
        string strleo = name.substr(0, 4);
        string type1 = name.substr(len - 2, 1);
        string type2 = name.substr(len - 3, 2);
        if (strleo == "CHAM")
            return CHAM;
        if (strleo == "HY-2")
        {
            if (type1 == "A")
                return HY2A;
        }
        if (strleo == "TIAN")
            return TG02;
        if (strleo == "ZIYU")
        {
            if (type1 == "3")
                return ZY03;
        }
        if (strleo == "KOMP")
        {
            if (type1 == "5")
                return KOM5;
        }
        if (strleo == "LING")
            return LING;
        if (strleo == "TERR")
            return TESX;
        if (strleo == "TAND")
            return TADX;
        if (strleo == "PAZS")
            return PAZS;
        if (strleo == "FY-3")
        {
            if (type1 == "C")
                return FY3C;
            if (type1 == "D")
                return FY3D;
        }
        if (strleo == "JASO")
        {
            if (type1 == "2")
                return JAS2;
            if (type2 == "3")
                return JAS3;
        }
        if (strleo == "GRAC")
        {
            if (type1 == "A")
                return GRAA;
            if (type2 == "B")
                return GRAB;
            if (type1 == "C")
                return GRAC;
            if (type2 == "D")
                return GRAD;
        }
        if (strleo == "METO")
        {
            if (type1 == "A")
                return META;
            if (type2 == "B")
                return METB;
            if (type2 == "C")
                return METC;
        }
        if (strleo == "SWAR")
        {
            if (type1 == "A")
                return SWAA;
            if (type1 == "B")
                return SWAB;
            if (type1 == "C")
                return SWAC;
        }
        if (strleo == "COSM")
        {
            if (type1 == "1")
                return COS1;
            if (type1 == "2")
                return COS2;
            if (type1 == "3")
                return COS3;
            if (type1 == "4")
                return COS4;
            if (type1 == "5")
                return COS5;
            if (type1 == "6")
                return COS6;
        }
        if (strleo == "SENT")
        {
            if (type2 == "1A")
                return SE1A;
            if (type2 == "1B")
                return SE1B;
            if (type2 == "2A")
                return SE2A;
            if (type2 == "2B")
                return SE2B;
            if (type2 == "3A")
                return SE3A;
            if (type2 == "3B")
                return SE3B;
        }
        return UNDF;
    }

    /**
    * @brief convert GLEOLONG enum to GLEOSHORT
    * @param[in]  leo  GLEOLONG name of satellite
    * @return    GLEOSHORT name of satellite
    */
    GLEOSHORT long2short(GLEOLONG leo)
    {
        switch (leo)
        {
        case CHAMP:
            return CHAM;
        case TIANGONG_2:
            return TG02;
        case HY_2A:
            return HY2A;
        case ZIYUAN_3:
            return ZY03;
        case KOMPSAT5:
            return KOM5;
        case LINGQIAO:
            return LING;
        case TERRASAR_X:
            return TESX;
        case FY_3C:
            return FY3C;
        case FY_3D:
            return FY3D;
        case JASON_2:
            return JAS2;
        case JASON_3:
            return JAS3;
        case GRACE_A:
            return GRAA;
        case GRACE_B:
            return GRAB;
        case GRACE_C:
            return GRAC;
        case GRACE_D:
            return GRAD;
        case METOP_A:
            return META;
        case METOP_B:
            return METB;
        case METOP_C:
            return METC;
        case SWARM_A:
            return SWAA;
        case SWARM_B:
            return SWAB;
        case SWARM_C:
            return SWAC;
        case COSMIC_1:
            return COS1;
        case COSMIC_2:
            return COS2;
        case COSMIC_3:
            return COS3;
        case COSMIC_4:
            return COS4;
        case COSMIC_5:
            return COS5;
        case COSMIC_6:
            return COS6;
        case SENTINEL_1A:
            return SE1A;
        case SENTINEL_1B:
            return SE1B;
        case SENTINEL_2A:
            return SE2A;
        case SENTINEL_2B:
            return SE2B;
        case SENTINEL_3A:
            return SE3A;
        case SENTINEL_3B:
            return SE3B;
        case TANDEM_X:
            return TADX;
        case PAZSAT:
            return PAZS;
        default:
            return UNDF;
        }
        return UNDF;
    }

    /**
    * @brief convert GLEOLONG enum to long string
    * @param[in]  leo  GLEOLONG name of satellite
    * @return   long string name of satellite
    */
    string long2str(GLEOLONG leo)
    {
        switch (leo)
        {
        case CHAMP:
            return "CHAMP";
            //case GNOS        : return "GNOS";
        case TIANGONG_2:
            return "TIANGONG-2";
        case HY_2A:
            return "HY-2A";
        case ZIYUAN_3:
            return "ZIYUAN-3";
        case KOMPSAT5:
            return "KOMPSAT5";
        case LINGQIAO:
            return "LINGQIAO";
        case TERRASAR_X:
            return "TERRASAR-X";
        case FY_3C:
            return "FY-3C";
        case FY_3D:
            return "FY-3D";
        case JASON_2:
            return "JASON-2";
        case JASON_3:
            return "JASON-3";
        case GRACE_A:
            return "GRACE-A";
        case GRACE_B:
            return "GRACE-B";
        case GRACE_C:
            return "GRACE-C";
        case GRACE_D:
            return "GRACE-D";
        case METOP_A:
            return "METOP-A";
        case METOP_B:
            return "METOP-B";
        case METOP_C:
            return "METOP-C";
        case SWARM_A:
            return "SWARM-A";
        case SWARM_B:
            return "SWARM-B";
        case SWARM_C:
            return "SWARM-C";
        case COSMIC_1:
            return "COSMIC-1";
        case COSMIC_2:
            return "COSMIC-2";
        case COSMIC_3:
            return "COSMIC-3";
        case COSMIC_4:
            return "COSMIC-4";
        case COSMIC_5:
            return "COSMIC-5";
        case COSMIC_6:
            return "COSMIC-6";
        case SENTINEL_1A:
            return "SENTINEL-1A";
        case SENTINEL_1B:
            return "SENTINEL-1B";
        case SENTINEL_2A:
            return "SENTINEL-2A";
        case SENTINEL_2B:
            return "SENTINEL-2B";
        case SENTINEL_3A:
            return "SENTINEL-3A";
        case SENTINEL_3B:
            return "SENTINEL-3B";
        case TANDEM_X:
            return "TANDEM-X";
        case PAZSAT:
            return "PAZSAT";
        default:
            return "";
        }
        return "";
    }

    /**
    * @brief convert GLEOSHORT enum to short string
    * @param[in]  leo  GLEOSHORT name of satellite
    * @return      short string name of satellite
    */
    string short2str(GLEOSHORT leo)
    {
        switch (leo)
        {
        case CHAM:
            return "CHAM";
        case TG02:
            return "TG02";
        case HY2A:
            return "HY2A";
        case ZY03:
            return "ZY03";
        case KOM5:
            return "KOM5";
        case LING:
            return "LING";
        case TESX:
            return "TESX";
        case FY3C:
            return "FY3C";
        case FY3D:
            return "FY3D";
        case JAS2:
            return "JAS2";
        case JAS3:
            return "JAS3";
        case GRAA:
            return "GRAA";
        case GRAB:
            return "GRAB";
        case GRAC:
            return "GRAC";
        case GRAD:
            return "GRAD";
        case META:
            return "META";
        case METB:
            return "METB";
        case METC:
            return "METC";
        case SWAA:
            return "SWAA";
        case SWAB:
            return "SWAB";
        case SWAC:
            return "SWAC";
        case COS1:
            return "COS1";
        case COS2:
            return "COS2";
        case COS3:
            return "COS3";
        case COS4:
            return "COS4";
        case COS5:
            return "COS5";
        case COS6:
            return "COS6";
        case SE1A:
            return "SE1A";
        case SE1B:
            return "SE1B";
        case SE2A:
            return "SE2A";
        case SE2B:
            return "SE2B";
        case SE3A:
            return "SE3A";
        case SE3B:
            return "SE3B";
        case TADX:
            return "TADX";
        case PAZS:
            return "PAZS";
        default:
            return "";
        }
        return "";
    }

    /**
    * @brief convert short string to long string
    * @param[in]  s  short string name of satellite
    * @return   long string name of satellite
    */
    string strs2strl(string s)
    {
        string name = low2upp(s);
        int len = trim(name).length();
        if (len < 4)
        {
            cerr << "Wrong input LEO satellite name: "
                 << name
                 << endl;
            return "";
        }
        if (name == "CHAM")
            return "CHAMP";
        if (name == "HY2A")
            return "HY-2A";
        if (name == "TG02")
            return "TIANGONG-2";
        if (name == "ZY03")
            return "ZIYUAN-3";
        if (name == "KOM5")
            return "KOMPSAT5";
        if (name == "LING")
            return "LINGQIAO";
        if (name == "TESX")
            return "TERRASAR-X";
        if (name == "FY3C")
            return "FY-3C";
        if (name == "FY3D")
            return "FY-3D";
        if (name == "JAS2")
            return "JASON-2";
        if (name == "JAS3")
            return "JASON-3";
        if (name == "GRAA")
            return "GRACE-A";
        if (name == "GRAB")
            return "GRACE-B";
        if (name == "GRAC")
            return "GRACE-C";
        if (name == "GRAD")
            return "GRACE-D";
        if (name == "META")
            return "METOP-A";
        if (name == "METB")
            return "METOP-B";
        if (name == "METC")
            return "METOP-C";
        if (name == "SWAA")
            return "SWARM-A";
        if (name == "SWAB")
            return "SWARM-B";
        if (name == "SWAC")
            return "SWARM-C";
        if (name == "COS1")
            return "COSMIC-1";
        if (name == "COS2")
            return "COSMIC-2";
        if (name == "COS3")
            return "COSMIC-3";
        if (name == "COS4")
            return "COSMIC-4";
        if (name == "COS5")
            return "COSMIC-5";
        if (name == "COS6")
            return "COSMIC-6";
        if (name == "SE1A")
            return "SENTINEL-1A";
        if (name == "SE1B")
            return "SENTINEL-1B";
        if (name == "SE2A")
            return "SENTINEL-2A";
        if (name == "SE2B")
            return "SENTINEL-2B";
        if (name == "SE3A")
            return "SENTINEL-3A";
        if (name == "SE3B")
            return "SENTINEL-3B";
        if (name == "TADX")
            return "TANDEM-X";
        if (name == "LAGEOS-1")
            return "LAGEOS-1";
        if (name == "LAGEOS-2")
            return "LAGEOS-2";
        if (name == "PAZS")
            return "PAZSAT";
        return "";
    }

    /**
    * @brief convert long string to short string
    * param[in]  s  long string name of satellite
    * @return  short string name of satellite
    */
    string strl2strs(string s)
    {
        string name = low2upp(s);
        int len = trim(name).length();
        if (len < 4)
        {
            cerr << "Wrong input LEO satellite name: "
                 << name
                 << endl;
            return "";
        }
        if (name == "CHAMP")
            return "CHAM";
        if (name == "HY-2A")
            return "HY2A";
        if (name == "TIANGONG-2")
            return "TG02";
        if (name == "ZIYUAN-3")
            return "ZY03";
        if (name == "KOMPSAT5")
            return "KOM5";
        if (name == "LINGQIAO")
            return "LING";
        if (name == "TERRASAR-X")
            return "TESX";
        if (name == "FY-3C")
            return "FY3C";
        if (name == "FY-3D")
            return "FY3D";
        if (name == "JASON-2")
            return "JAS2";
        if (name == "JASON-3")
            return "JAS3";
        if (name == "GRACE-A")
            return "GRAA";
        if (name == "GRACE-B")
            return "GRAB";
        if (name == "GRACE-C")
            return "GRAC";
        if (name == "GRACE-D")
            return "GRAD";
        if (name == "METOP-A")
            return "META";
        if (name == "METOP-B")
            return "METB";
        if (name == "METOP-C")
            return "METC";
        if (name == "SWARM-A")
            return "SWAA";
        if (name == "SWARM-B")
            return "SWAB";
        if (name == "SWARM-C")
            return "SWAC";
        if (name == "COSMIC-1")
            return "COS1";
        if (name == "COSMIC-2")
            return "COS2";
        if (name == "COSMIC-3")
            return "COS3";
        if (name == "COSMIC-4")
            return "COS4";
        if (name == "COSMIC-5")
            return "COS5";
        if (name == "COSMIC-6")
            return "COS6";
        if (name == "SENTINEL-1A")
            return "SE1A";
        if (name == "SENTINEL-1B")
            return "SE1B";
        if (name == "SENTINEL-2A")
            return "SE2A";
        if (name == "SENTINEL-2B")
            return "SE2B";
        if (name == "SENTINEL-3A")
            return "SE3A";
        if (name == "SENTINEL-3B")
            return "SE3B";
        if (name == "TANDEM-X")
            return "TADX";
        if (name == "LAGEOS-1")
            return "LAGEOS-1";
        if (name == "LAGEOS-2")
            return "LAGEOS-2";
        if (name == "PAZSAT")
            return "PAZS";
        return "";
    }

    /**
    * @brief get all supported LEO satellites
    * @return  all supported LEO satellites(GLEOLONG name)
    */
    set<GLEOLONG> LEO_SUPPORTED()
    {
        set<GLEOLONG> leo;

        leo.insert(CHAMP);
        leo.insert(HY_2A);
        leo.insert(TIANGONG_2);
        leo.insert(ZIYUAN_3);
        leo.insert(KOMPSAT5);
        leo.insert(LINGQIAO);
        leo.insert(TERRASAR_X);
        leo.insert(FY_3C);
        leo.insert(FY_3D);
        leo.insert(JASON_2);
        leo.insert(JASON_3);
        leo.insert(GRACE_A);
        leo.insert(GRACE_B);
        leo.insert(GRACE_C);
        leo.insert(GRACE_D);
        leo.insert(METOP_A);
        leo.insert(METOP_B);
        leo.insert(METOP_C);
        leo.insert(SWARM_A);
        leo.insert(SWARM_B);
        leo.insert(SWARM_C);
        leo.insert(COSMIC_1);
        leo.insert(COSMIC_2);
        leo.insert(COSMIC_3);
        leo.insert(COSMIC_4);
        leo.insert(COSMIC_5);
        leo.insert(COSMIC_6);
        leo.insert(SENTINEL_1A);
        leo.insert(SENTINEL_1B);
        leo.insert(SENTINEL_2A);
        leo.insert(SENTINEL_2B);
        leo.insert(SENTINEL_3A);
        leo.insert(SENTINEL_3B);
        leo.insert(TANDEM_X);
        leo.insert(LAGEOS_1);
        leo.insert(LAGEOS_2);
        leo.insert(PAZSAT);

        return leo;
    }

    /**
    * @brief judge the LEO satellite is supported LEO or not
    * @param[in]  s  long string name of satellite
    * @return    the satellite is supported LEO or not
    */
    bool IS_SUPPORTED_LEO(string s)
    {
        try
        {
            string name = low2upp(s);
            GLEOLONG leo = str2long(name);
            set<GLEOLONG>::iterator it;
            set<GLEOLONG> leos = LEO_SUPPORTED();
            for (it = leos.begin(); it != leos.end(); it++)
            {
                if (leo == *it)
                {
                    return true;
                }
            }
            return false;
        }
        catch (...)
        {
            cout << "ERROR: function: IS_SUPPORTED_LEO  unknown mistake" << endl;
            return false;
        }
    }
} // namespace