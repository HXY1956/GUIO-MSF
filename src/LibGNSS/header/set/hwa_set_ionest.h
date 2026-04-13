#ifndef hwa_set_ionest_h
#define hwa_set_ionest_h
#define XMLKEY_IONEST "ionest"
#define XMLKEY_IONOGRID  "ionogrid"
#define XMLKEY_ROM "read_ofile_mode"
#include "hwa_set_base.h"
#include "hwa_base_log.h"
#include "hwa_base_typeconv.h"
using namespace pugi;
namespace hwa_set
{
    enum class PRETYPE : int
    {
        IONO_P, IONO_PL, IONO_PPP, IONO_SION, IONO_PPPAR
    };
    /// The class for setting of process modelue in XML file
    class set_ionest : public virtual set_base
    {
    public:
        /// constructor
        set_ionest();
        /// destructor
        ~set_ionest() override;

        /// settings check
        void check() override;
        /// settings help
        void help() override;

        // for global iono model
        int n_ses();
        int order();
        double minlat();
        double maxlat();
        double minlon();
        double maxlon();
        double step();
        double sessLength();
        double maglat();
        double maglon();
        PRETYPE PreType();
        std::string IONdir();

        // for iono grid
        std::string mask_grid();
        std::string ID_grid();
        std::string ref_Site();
        double reflon_grid();
        double reflat_grid();
        double spacelon_grid();
        double spacelat_grid();
        int countlon_grid();
        int countlat_grid();
        int min_site();
        double max_Sigma();
        double max_Baseline();
        std::set<std::string> recs_rm();
    protected:
        // for global iono model
        int _n_ses;
        int _order;
        double _minlat;
        double _maxlat;
        double _minlon;
        double _maxlon;
        double _step;
        double _sessLength;
        double _maglat;
        double _maglon;
        PRETYPE _Pretype;
        std::string _IONdir;

        /// for iono grid
        std::string _mask;                       ///< Grid name
        std::string _ID;                         ///< Grid ID
        double _RefLon;                     ///< reference longitude
        double _RefLat;                     ///< reference latitude
        double _SpaceLon;                   ///< space of grid for longitude
        double _SpaceLat;                   ///< space of grid for latitude
        int _CountLon;                      ///< count of grid for longitude
        int _CountLat;                      ///< count of grid for latitude
    };

} // namespace

#endif
