#ifndef hwa_set_gbase_h
#define hwa_set_gbase_h

#define XMLKEY_GNSS "gnss" ///< The defination of gnss node

#include "hwa_set_base.h"
#include "hwa_set_gproc_type.h"
#include "hwa_base_typeconv.h"
#include "hwa_base_pair.h"

using namespace hwa_base;
using namespace hwa_gnss;
using namespace pugi;

namespace hwa_set
{
    /// The class for gnss module in xml file
    class set_gnss : public virtual set_base
    {
    public:
        /// constructor
        set_gnss();
        /// destructor
        ~set_gnss();

        /// settings check
        void check() override;
        /// settings help
        void help() override;

        /**
         * @brief get sats of all systems 
         * @return set<string> : sats of all systems
         */
        std::set<std::string> sat();

        /**
         * @brief get sats of single system def=true (give default values at least)
         * @param[in] gsys system
         * @param[in] def  if empty, return default setting or not. default is true
         * @return set<string> : sats of single system def=true (give default values at least)
         */
        std::set<std::string> sat(GSYS gsys, bool def = true);

        /**
         * @brief get obs(eg.C1C ..) of single system def=true (give default values at least)
         * @param[in] gsys system
         * @param[in] def  if empty, return default setting or not. default is true
         * @return set<string> : obs of single system def=true (give default values at least)
         */
        std::set<std::string> obs(GSYS gsys, bool def = true);

        /**
         * @brief get nav of single system def=true (give default values at least)
         * @param[in] gsys system
         * @param[in] def  if empty, return default setting or not. default is true
         * @return set<string> : nav of single system def=true (give default values at least)
         */
        std::set<std::string> nav(GSYS gsys, bool def = true);

        /**
         * @brief get extending gobs list with complete singals
         * @param[in] gsys system
         * @return set<string> : extending gobs list with complete singals
         */
        std::set<std::string> gobs(GSYS gsys);

        /**
         * @brief get nav of system:gsys
         * @param[in] gsys system
         * @return vector<GNAVTYPE> : nav of system:gsys
         */
        std::vector<GNAVTYPE> gnav(GSYS gsys);

        /**
         * @brief get obs type of system:gsys
         * @param[in] gsys system
         * @return vector<GOBSTYPE> : obs type of system:gsys
         */
        std::vector<GOBSTYPE> type(GSYS gsys);

        /**
         * @brief get obs band of system:gsys
         * @param[in] gsys system
         * @return vector<GOBSBAND> : obs band of system:gsys
         */
        std::vector<GOBSBAND> band(GSYS gsys);

        // jdhuang
        // add to get the freq used in proc
        std::vector<FREQ_SEQ> freqs(GSYS gsys);

        // jdhuang
        // add to get the freq and band order
        std::map<FREQ_SEQ, GOBSBAND> band_index(GSYS gsys);
        std::map<GOBSBAND, FREQ_SEQ> freq_index(GSYS gsys);

        /**
         * @brief get obs ATTR of system:gsys
         * @param[in] gsys system
         * @return vector<GOBSATTR> : obs ATTR of system:gsys
         */
        std::vector<GOBSATTR> attr(GSYS gsys);

        std::set<GSYS> SYS_SUPPORTED();

        /**
         * @brief get sigma value of L obs for gsys
         * @param[in] gsys system
         * @return double : sigma value of L obs for gsys
         */
        double sigma_L(GSYS gsys);

        double sigma_L_LEO(GSYS gsys); // add for LEO by zhangwei

        /**
         * @brief get sigma value of C obs for gsys
         * @param[in] gsys system
         * @return double : sigma value of C obs for gsys
         */
        double sigma_C(GSYS gsys);

        double sigma_C_LEO(GSYS gsys); // add for LEO by zhangwei

        /**
         * @brief get sigma value of D obs for gsys
         * @param[in] gsys system
         * @return double : sigma value of D obs for gsys
         */
        double sigma_D(GSYS gsys);

        /**
         * @brief get max res of L obs for gsys
         * @param[in] gsys system
         * @return double : max res of L obs for gsys
         */
        double maxres_L(GSYS gsys);

        /**
         * @brief get max res of C obs for gsys
         * @param[in] gsys system
         * @return double : max res of C obs for gsys
         */
        double maxres_C(GSYS gsys);

        /**
         * @brief get max res of D obs for gsys
         * @param[in] gsys system
         * @return double : max res of D obs for gsys
         */
        double maxres_D(GSYS gsys);

    protected:
        /**
         * @brief get obs type of system:gsys form XML
         * @param[in] gsys system
         * @return vector<GOBSTYPE> : obs type of system:gsys form XML
         */
        std::vector<GOBSTYPE> _type(GSYS gsys);

        /**
         * @brief get obs band of system:gsys form XML
         * @param[in] gsys system
         * @return vector<GOBSBAND> : obs band of system:gsys form XML
         */
        std::vector<GOBSBAND> _band(GSYS gsys);

        /**
         * @brief get obs band of system:gsys form XML
         * @param[in] gsys system
         * @return vector<GOBSBAND> : obs band of system:gsys form XML
         */
        std::vector<FREQ_SEQ> _sysfreq(GSYS gsys); //add by xiongyun

        // jdhuang
        // add to get the freq used in proc
        std::vector<FREQ_SEQ> _freqs(GSYS gsys);

        /**
         * @brief get obs attributes of system:gsys form XML
         * @param[in] gsys system
         * @return vector<GOBSATTR> : obs attributes of system:gsys form XML
         */
        std::vector<GOBSATTR> _attr(GSYS gsys);

        /**
         * @brief change GSYS to string::gsys
         * @param[in] gsys system
         * @return string : gsys
         */
        std::string _gsys(GSYS gsys);

        /**
         * @brief get the double value of sys.attribute(sigma_L)
         * @param[in] gsys system
         * @return string : the double value of sys.attribute(sigma_L)
         */
        double _sigma_L(GSYS gsys);

        double _sigma_L_LEO(GSYS gsys); //add for LEO by zhangwei

        /**
         * @brief get the double value of sys.attribute(sigma_C)
         * @param[in] gsys system
         * @return string : the double value of sys.attribute(sigma_C)
         */
        double _sigma_C(GSYS gsys);

        double _sigma_C_LEO(GSYS gsys); //add for LEO by zhangwei

        /**
         * @brief get the double value of sys.attribute(sigma_D)
         * @param[in] gsys system
         * @return string : the double value of sys.attribute(sigma_D)
         */
        double _sigma_D(GSYS gsys);

        /**
         * @brief get the double value of sys.attribute(maxres_L)
         * @param[in] gsys system
         * @return string : the double value of sys.attribute(maxres_L)
         */
        double _maxres_L(GSYS gsys);

        /**
         * @brief get the double value of sys.attribute(maxres_C)
         * @param[in] gsys system
         * @return string : the double value of sys.attribute(maxres_C)
         */
        double _maxres_C(GSYS gsys);

        /**
         * @brief get the double value of sys.attribute(maxres_D)
         * @param[in] gsys system
         * @return string : the double value of sys.attribute(maxres_D)
         */
        double _maxres_D(GSYS gsys);

    protected:
        std::map<GSYS, std::vector<std::string>> _band_str; ///< default set
        std::map<GSYS, std::vector<std::string>> _type_str; ///< default set
        std::map<GSYS, std::vector<std::string>> _attr_str; ///< default set

        std::map<GSYS, base_pair> _sigma_def;         ///< default set
        std::map<GSYS, base_pair> _sigma_def_LEO;     ///< default set for LEO
        std::map<GSYS, base_pair> _maxres_def;        ///< default set
        std::map<GSYS, double> _sigma_def_doppler;  ///< default set for doppler (by zhshen)
        std::map<GSYS, double> _maxres_def_doppler; ///< default set for doppler (by zhshen)

        // map<GSYS,vector<GOBSBAND> > _band_obs;         // default set
        // map<GSYS,vector<GOBSTYPE> > _type_obs;         // default set
        // map<GSYS,vector<GOBSATTR> > _attr_obs;         // default set

    private:
    };

} // namespace

#endif
