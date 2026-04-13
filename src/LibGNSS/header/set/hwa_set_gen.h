#ifndef hwa_set_gen_h
#define hwa_set_gen_h

#include "hwa_base_log.h"
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"
#include "hwa_set_base.h"
#include "hwa_set_gtype.h"
#include "hwa_gnss_data_obj.h"
#include "hwa_gnss_data_rec.h"

#define XMLKEY_GEN "gen"   ///< The defination of gen node

namespace hwa_set
{
    /// The class for setting node : gen
    class set_gen : public virtual set_base
    {
    public:
        /**
         * @brief default constructor, distinguish GNSS/nonGNSS app
         * @param[in] gnss : gnss process or not, default is true
         */
        set_gen(bool gnss = true);

        /// defalut desconstructor
        ~set_gen();

        /// settings check
        void check() override;

        /// settings help
        void help() override;

        /**
         * @brief get the beg time of process
         * @return base_time : the beg time of process
         */
        hwa_base::base_time beg(bool conv = true);

        /**
         * @brief get the end time of process
         * @return base_time : the end time of process
         */
        hwa_base::base_time end(bool conv = true);

        /**
         * @brief get the sampling time of process
         * @return double : the sampling time of process
         */
        double sampling();

        /**
         * @brief get the default sampling time of process
         * @return double : the default sampling time of process
         *  @retval DEF_SAMPLING default sampling 
         */
        double sampling_default() const;

        /**
         * @brief get the decimals scale of sampling time in process
         * @return int : the decimals scale of sampling time in process
         */
        int sampling_scalefc() { return (int)pow(10, _dec); }

        /**
         * @brief get the decimals for sampling interval (for high-rate) in process
         * @return int : decimals for sampling interval (for high-rate) in process
         */
        const int &sampling_decimal() { return _dec; }

        /**
         * @brief get the List of system names
         * @return std::set<std::string> : List of system names
         */
        virtual std::set<std::string> sys();
        virtual void sys(std::string str);
        /**
        * @brief use SLR observations or not
        * @returnstd::set<std::string>: GNS or LEO
        */
        virtual std::set<std::string> slr();

        /**
        * @brief use KBR observations or not
        * @return bool: true or false
        */
        virtual bool kbr();

        /**
        * @brief use LRI observations or not
        * @return bool: true or false
        */
        virtual bool lri();

        /**
        * @brief use VLBI observations or not
        * @return bool: true or false
        */
        virtual bool vlbi();

        virtual bool gnss();

        /**
         * @brief get the List of POD mode
         * @return std::set<std::string> : D(LEO DYN POD)/K(LEO KIN POD)/S(GPS POD)/SD(GPS+LEO POD)
         */
        virtual std::set<std::string> mode();
        std::map<std::string, std::set<std::string>> mode_map();

        std::set<std::string> recs();
        std::set<std::string> rec_all();
        std::set<std::string> rec_leo();

        virtual std::vector<std::string> list_base();
        virtual std::vector<std::string> list_rover();

        virtual hwa_gnss::gnss_data_obj *grec_obj(const std::string &name, hwa_base::base_log spdlog); // Obj of recevier

        // add for clk est.
        virtual std::string refsat();     // ref sat clk
        virtual std::string refsite();    // ref site clk
        virtual double sig_refclk(); // sig_refclk;

        /**
         * @brief get name of estimator
         * @return std::string : estimator name
         */
        virtual std::string estimator();

        /**
        * @brief add for remove unused satellites
        * @return std::set<std::string> : satellites which will be removed
        */
        virtual std::set<std::string> sat_rm();
        virtual void sat_rm(std::string str);

        /**
        * @brief add for PPP-RTK identify Server or Client
        * @return bool : YES is Client, NO is Server
        */
        virtual bool isClient();

    protected:
        bool _gnss;  ///< gnss or not
        std::string _sys; ///< sys name
        int _dec;    ///< sampling

    private:
    };

} // namespace

#endif
