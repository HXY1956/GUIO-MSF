/**
*
* @file          gallrecover.h
* @brief      Storage all recover info
*/

#ifndef hwa_gnss_all_recover_H
#define hwa_gnss_all_recover_H

#include "hwa_gnss_data_recover.h"
#include "hwa_gnss_data_poleut.h"
#include "hwa_base_allpar.h"
#include "hwa_gnss_all_pcv.h"
#include "hwa_gnss_all_Obj.h"
#include "hwa_gnss_all_prec.h"

namespace hwa_gnss
{
    ///< base_time : time, set_recover_equttion ： grecover equation data storaging
    typedef std::map<base_time, std::vector<gnss_data_recover_equation *>> hwa_map_TIME_EQU;
    ///< string : sat name
    typedef std::map<std::string, hwa_map_TIME_EQU> hwa_map_SAT_EQU;
    ///< string : station name
    typedef std::map<std::string, hwa_map_SAT_EQU> hwa_map_SITE_EQU;
    ///< base_time : time
    typedef std::map<base_time, std::vector<gnss_data_recover_par *>> hwa_map_gnss_TIME_PAR;
    ///< par_type : parameter's type
    typedef std::map<par_type, std::vector<gnss_data_recover_par *>> hwa_map_gnss_TYPE_PAR;

    typedef std::tuple<std::string, std::string, std::string, double, double, base_time, base_time> hwa_TUPLE_gnss_ION; // type, rec, sat, value, sigma,beg,end
    /**
     * @brief Class for parameter recover data
     */
    class gnss_all_recover : public base_data
    {
    public:
        explicit gnss_all_recover();

        /**
         * @brief Construct a new t gallrecover object
         */
        explicit gnss_all_recover(base_log spdlog);
        /**
         * @brief Destroy the t gallrecover object
         */
        virtual ~gnss_all_recover();
        /**
         * @brief add a recover equ
         * @param[in]  recover_equ recover equ
         */
        void add_recover_equation(const gnss_data_recover_equation &recover_equ);
        /**
         * @brief add a recovered parameter
         * @param[in]  recover_par  recovered parameter
         */
        void add_recover_par(const gnss_data_recover_par &recover_par);

        //// get recover equation

        /**
         * @brief Get the clkdata object
         * @param[in]  clkdata clk data
         * @param[in]  type    clk type
         */
        void get_clkdata(gnss_all_prec &clkdata, gnss_all_prec::clk_type type = gnss_all_prec::UNDEF);
        /**
         * @brief Get the clk13data object
         * @param[in]  clkdata clk data
         * @param[in]  type    clk type
         */
        void get_clk13data(gnss_all_prec &clkdata, gnss_all_prec::clk_type type = gnss_all_prec::UNDEF);
        /**
         * @brief Get the iondata object
         * @param[in]  ion_data Ionosphere's type, rec, sat, value, sigma,beg,end
         */
        void get_iondata(std::vector<hwa_TUPLE_gnss_ION> &ion_data);
        /**
         * @brief Get the poledata object
         * @param[in]  poledata Earth orientation parameters data
         */
        void get_poledata(gnss_data_poleut &poledata);
        /**
         * @brief Get the pcvdata object
         * @param[in]  pcvdata         pcv data
         * @param[in]  allobj          object data
         */
        void get_pcvdata(gnss_all_pcv &pcvdata, gnss_all_obj &allobj);

        /**
         * @brief Get the first phase recover equation
         * @param[in]  site            station's name
         * @param[in]  sat             sat's name
         * @param[in]  freq            frequency type
         * @return vector<gnss_data_recover_equation> recoverd equation
         */
        std::vector<gnss_data_recover_equation> get_first_phase_recover_equation(const std::string &site, const std::string &sat, const std::string &freq = "LC");
        /**
         * @brief Get the first phase recover equation
         * @param[in]  site            station's name
         * @param[in]  sat             sat's name
         * @param[in]  equ             recoverd equation
         * @param[in]  freq            frequency type
         * @return 
         *         @retval false can not get the equation 
         *         @retval true can get the equation
         */
        bool get_first_phase_recover_equation(const std::string &site, const std::string &sat, std::vector<gnss_data_recover_equation> &equ, const std::string &freq = "LC");

        /**
         * @brief Get the first pseudorange recover equation
         * @param[in]  site            station's name
         * @param[in]  sat             sat's name
         * @param[in]  equ             recoverd equation
         * @param[in]  freq            frequency type
         * @return
         *         @retval false can not get the equation
         *         @retval true can get the equation
         */
        bool get_first_pseudorange_recover_equation(std::string site, std::string sat, std::vector<gnss_data_recover_equation> &equ, std::string freq = "PC");

        /**
         * @brief Get the recover par object
         * @param[in]  parType         parameter's type
         * @return vector<gnss_data_recover_par> recover parameter data
         */
        std::vector<gnss_data_recover_par> get_recover_par(const par_type &parType);
        /**
         * @brief Get the beg time
         * @return base_time beg time
         */
        base_time get_beg_time() const;
        /**
         * @brief Get the end time
         * @return base_time end time
         */
        base_time get_end_time() const;
        /**
         * @brief Get the equ beg time object
         * @return base_time equ beg time
         */
        base_time get_equ_beg_time() const;
        /**
         * @brief Get the interval
         * @return double recover data's interval
         */
        double get_interval() const;
        /**
         * @brief Get the sigma0
         * @return double recover data's sigma0
         */
        double get_sigma0() const;
        /**
         * @brief Set the interval
         * @param[in]  intv            recover data's interval
         */
        void set_interval(double intv);
        /**
         * @brief Set the sigma
         * @param[in]  sigma0    recover data's sigma0
         */
        void set_sigma0(double sigma0);

        // get all record
        /**
         * @brief Get all of recover data
         * @return vector<gnss_data_recover*>  all of the recover data
         */
        const std::vector<gnss_data_recover *> &get_all_recover_data() const;
        /**
         * @brief Get the recover data value
         * @param[in]  parType   parameter's type
         * @return double the parType's parameter recover value
         */
        double get_recover_data_value(par_type parType);

        // get map_par
        /**
         * @brief Get the map time par
         * @return hwa_map_gnss_TIME_PAR the time map of recover parameter
         */
        const hwa_map_gnss_TIME_PAR &get_map_time_par() const;
        /**
         * @brief Get the map time equ
         * @return hwa_map_TIME_EQU the time map of recover equation
         */
        const hwa_map_TIME_EQU &get_map_time_equ() const;
        /**
         * @brief Get the map site equ
         * @return hwa_map_SITE_EQU the station map of recover equation
         */
        const hwa_map_SITE_EQU &get_map_site_equ() const;
        /**
         * @brief Get the sat list
         * @return set<string> the list of satellite's name
         */
        std::set<std::string> get_sat_list() const;
        /**
         * @brief Get the site list
         * @return set<string> the list of station's name
         */
        std::set<std::string> get_site_list() const;
        /**
         * @brief Get the time list
         * @return set<base_time> the list of recover time
         */
        std::set<base_time> get_time_list() const;
        /**
         * @brief Get the all obs time list 
         * @return vector<base_time> the list of observation time
         */
        std::vector<base_time> get_all_obs_time_list() const;

        /**
         * @brief Get the all parameters
         * @return base_allpar all of the parameters
         */
        base_allpar get_all_pars();

        // get nsat per epoch
        /**
         * @brief Get the sat number per epoch
         * @return map<base_time, int> sat number of the base_time
         */
        std::map<base_time, int> get_sat_number_per_epo() const;

        // SLR part
        /**
         * @brief Get the stadata
         * @param[in]  x         CRD X correct value
         * @param[in]  y         CRD Y correct value
         * @param[in]  z         CRD Z correct value
         */
        void get_stadata(std::map<std::string, Triple> &xyz, std::map<std::string, Triple> &neu);
        /**
         * @brief Get the range bias for slr
         * @param[in]  objs      station
         * @param[in]  obj_sats  satellite
         * @param[in]  rbs       range bias of slr
         */
        void get_rbdata(std::vector<std::string> &objs, std::vector<std::string> &obj_sats, std::vector<double> &rbs);
        /**
         * @brief Get the eopdata
         * @param[in]  epoches   time
         * @param[in]  xpoles    xpole values
         * @param[in]  ypoles    ypole values
         * @param[in]  dxpoles   xpole correct values
         * @param[in]  dypoles   ypole correct values
         * @param[in]  ut1s      ut1 values
         * @param[in]  dut1s     ut1 correct values
         */
        void get_eopdata(std::vector<base_time> &epoches,
                         std::vector<double> &xpoles,
                         std::vector<double> &ypoles,
                         std::vector<double> &dxpoles,
                         std::vector<double> &dypoles,
                         std::vector<double> &ut1s,
                         std::vector<double> &dut1s);

    protected:
        gnss_data_recover_head _recover_head;            ///< grcover head storaging
        std::vector<gnss_data_recover *> _recover_data; ///< grcover data storaging

    private:
        /**
         * @brief add a recover data
         * @param[in]  data      one recover data
         */
        void _add_common_data(gnss_data_recover *data);

        // Mainly for index for time for and sat but no for storaging
        hwa_map_TIME_EQU _time_equmap;                           ///< equations time map
        hwa_map_SITE_EQU _site_sat_time_equmap;                  ///< equations station & satellite map
        hwa_map_gnss_TIME_PAR _time_parmap;                           ///< time parameters map
        std::map<base_time, std::vector<gnss_data_recover_par>> _time_parmap_pce; ///< time recover parameters data map
        hwa_map_gnss_TYPE_PAR _type_parmap; ///< par_type : parameter's type
    };

}

#endif // !GALLRECOVER_H