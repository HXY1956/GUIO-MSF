#ifndef hwa_set_rec_h
#define hwa_set_rec_h
#define XMLKEY_REC "receiver"
#include <map>
#include <string>
#include <iostream>
#include "hwa_set_base.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_time.h"
#include "hwa_base_log.h"
#include "hwa_gnss_data_obj.h"
#include "hwa_gnss_data_rec.h"
#include "hwa_gnss_model_gpt.h"
#include "hwa_gnss_data_recleo.h"

#define HSL_UNKNOWN -9999 // unknown height ebove mean see level

using namespace hwa_base;

namespace hwa_set
{
    class set_rec : public virtual set_base
    {
    public:
        set_rec();
        ~set_rec();

        void check(); // settings check
        void help();  // settings help

        int get_crd_xyz(Triple &xyz, std::string s);
        Triple get_crd_xyz(std::string s);
        Triple get_std_xyz(std::string s);
        std::set<std::string> objects();                                  // get all objects IDs
        virtual std::shared_ptr<hwa_gnss::gnss_data_rec> grec(std::string s, base_log spdlog = 0); // ID: return grec object for selected ID

        // all below can be removed in future
        hwa_gnss::gnss_data_obj *obj(std::string s);
        //base_time beg(std::string s);          // ID: start time
        //base_time end(std::string s);          // ID: end time
        std::string rec(std::string s); // ID: receiver name
        std::string ant(std::string s); // ID: antenna name

        /**
         * @brief get the List of recevier names
         * @return set<std::string> : List of recevier names
         */
        virtual std::set<std::string> recs();
        std::set<std::string> all_rec();
        virtual hwa_gnss::gnss_data_obj *grec_all(std::string name, base_log spdlog); // Obj of recevier(including LEO and site)

    protected:
        Triple _get_crd_xyz(std::string s);
        Triple _get_std_xyz(std::string s);
        Triple _get_ecc_neu(std::string s);
        Triple _get_ecc_xyz(std::string s);
        Triple _get_crd_blh(std::string s); // lat, lon, hmsl(above see level)

        double _aprX(std::string s);
        double _aprY();
        double _aprZ();
        double _aprDX();
        double _aprDY();
        double _aprDZ();
        double _aprDE();
        double _aprDN();
        double _aprDU();
        double _aprZTD();
        double _sigZTD();
        std::string _sigCRD();
        std::set<std::string> _objects(); // get all objects names

        std::string _rec;      // default receiver name
        std::string _ant;      // default antenna name
        std::string _id;       // receiver id
        std::string _name_rec; // receiver name
        std::string _desc;     // receiver description
        std::string _domes;    // receiver monumentation domes
        base_time _beg;     // default begin time
        base_time _end;     // default end time
        double _X;        // receiver X-coordinate [m]
        double _Y;        // receiver Y-coordinate [m]
        double _Z;        // receiver Z-coordinate [m]
        double _dX;       // receiver X-eccentricity [m]
        double _dY;       // receiver Y-eccentricity [m]
        double _dZ;       // receiver Z-eccentricity [m]
        double _dE;       // receiver E-eccentricity [m]
        double _dN;       // receiver N-eccentricity [m]
        double _dU;       // receiver U-eccentricity [m]
        double _ZTD;
        double _ZTD_sig;
        std::string _crd_sig;
        bool _overwrite;

    private:
    };

} // namespace

#endif
