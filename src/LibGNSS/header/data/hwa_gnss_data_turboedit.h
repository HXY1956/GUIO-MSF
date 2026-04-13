/**
*
* @brief          CRS coordinate transform to ACR coordinate
* @author         GREAT, Wuhan University
*/

#ifndef hwa_gnss_data_turboedit_H
#define hwa_gnss_data_turboedit_H

#include "hwa_gnss_data_cycleslip.h"
#include "hwa_gnss_data_SATDATA.h"
#include "hwa_gnss_all_ambflag.h"
#include "hwa_set_turboedit.h"

using namespace hwa_base;
using namespace hwa_set;

namespace hwa_gnss
{

    class gnss_data_turboedit : public gnss_data_cycleslip
    {
    public:
        /** @brief default constructor. */
        gnss_data_turboedit();

        /**
        * @brief constructor 1.
        * @param[in]  gset        settings.
        * @param[in]  glog        log file.
        * @param[in]  index        default value is 2.
        * @return      gnss_data_turboedit
        */
        gnss_data_turboedit(set_base *gset, base_log spdlog, int index = 2);

        /** @brief default destructor. */
        virtual ~gnss_data_turboedit();

        /**
        * @brief set ambiguity flag.
        * @param[in]  rec        the name of receiver.
        * @param[in]  sat        the name of satellite.
        * @param[in]  flag        the flag of ambiguity.
        * @return      void
        */
        void set_amb_flag(const std::string &rec, const std::string &sat, const int &flag) override;

        /**
        * @brief get ambiguity flag.
        * @param[in]  rec        the name of receiver.
        * @param[in]  sat        the name of satellite.
        * @return      the flag of ambiguity.
        */
        int get_amb_flag(const std::string &rec, const std::string &sat) override;

        /**
        * @brief get active ambiguity.
        * @param[in]  site        the name of site.
        * @return      the flag of ambiguity.
        */
        int get_active_amb(const std::string &site) override;

        /**
        * @brief set active ambiguity. override
        * @param[in]  site          the name of site.
        * @param[in]  active_num  active ambiguity value.
        * @return      void.
        */
        void set_active_amb(const std::string &site, const int &active_num) override; // glfeng

        /**
        * @brief get the number of ambiguity arc.
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current time.
        * @return      the number of ambiguity arc.
        */
        int num_of_amb_arc(const std::string &site, const std::string &prn, const base_time &time) override;

        /**
        * @brief whether using the arc observation.
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current epoch.
        * @return      whether using the arc observation.
        */
        bool use_of_obs(const std::string &site, const std::string &prn, const base_time &time) override;

        /**
        * @brief whether is the carrier observation.
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current epoch.
        * @return      whether is the carrier observation.
        */
        bool is_carrier_range(const std::string &site, const std::string &prn, const base_time &time) override;

        /**
        * @brief whether is the carrier observation. override
        * @param[in]  site          the name of site.
        * @param[in]  prn          the name of satellite.
        * @param[in]  time          the time of current epoch.
        * @param[in]  c1          
        * @param[in]  c2
        * @return      whether is the carrier observation.
        */
        bool is_carrier_range(const std::string &site, const std::string &prn, const base_time &time, double &c1, double &c2) override;

        /**
        * @brief whether cycle slipped?
        * @param[in]  obsdata      observation data.
        * @param[in]  time          the time of current epoch.
        * @return      whether cycle slipped?
        */
        bool cycle_slip(const gnss_data_sats &obsdata, const base_time &time) override;

        /**
        * @brief whether cycle slipped(123 ferq)?
        * @param[in]  obsdata      observation data.
        * @param[in]  time          the time of current epoch.
        * @return      whether cycle slipped?
        */
        bool cycle_slip123(gnss_data_sats &obsdata, const base_time &time);

        /**
        * @brief whether need add new ambiguity?
        * @param[in]  rec          the name of receiver.
        * @param[in]  sat          the name of satellite.
        * @return      whether need add new ambiguity?
        */
        bool new_amb(const std::string &rec, const std::string &sat);

        /**
        * @brief add new ambiguity
        * @param[in]  rec          the name of receiver.
        * @param[in]  sat          the name of satellite.
        * @param[in]  isNew          whether need add new ambiguity?
        * @return      void
        */
        void set_new_amb(const std::string &rec, const std::string &sat, const bool &isNew);

        /**
        * @brief get the crt ambiguity of the end epoch
        * @param[in]  rec          the name of receiver.
        * @param[in]  sat          the name of satellite.
        * @return      the end epoch
        */
        base_time get_crt_amb_end(const std::string &rec, const std::string &sat);

        /**
        * @brief add the ambiguity flag
        * @param[in]  site          the name of site.
        * @param[in]  sat          the name of satellite.
        * @param[in]  description the flag of ambiguity.
        * @param[in]  beg          the begin time.
        * @param[in]  end          the end time.
        * @return      void
        */
        void add_ambflag(const std::string &site, const std::string &sat, const std::string &description, const base_time &beg,
                         const base_time &end) override; // glfeng

        /** @brief get the active ambiguity. */
        virtual std::map<std::string, int> get_active_amb() { return _active_amb; }

        /** @brief set the value of _amb_info_file_exist */
        void merge_logfile_exist(const std::map<std::string, bool> &logfile);

        /** @brief get the value of _amb_info_file_exist */
        std::map<std::string, bool> &logfile_exist() { return _amb_info_file_exist; }

        /** @brief get the sitelist of logfile */
        std::set<std::string> get_sitelist_of_logfile() const;

        hwa_map_all_ambflag get_all_logfile(); //hanjj

    protected:
        /** @brief read log file and record cycle slip info [panda file format]*/
        void _read_logfie(const std::set<std::string> &rec, const base_time &epoch, int index);
        void _read_logfile(const std::set<std::string> &rec, const base_time &epoch, int index);

        std::map<std::string, int> _active_amb;                                                                    ///< Max ambc in one epoch  glfeng
        std::map<std::string, bool> _amb_info_file_exist;                                                          ///< recording exitence of panda logfile
        std::unordered_map<std::string, std::unordered_map<std::string, int>> _amb_flag;                                     ///< record the site-sat amb arc
        std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::pair<base_time, base_time>>>> _cycle_flag;        ///< recording cycle slip info in panda log file
        std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::pair<base_time, base_time>>>> _cycle_flag_unused; ///< recording unused cycle slip info in panda log file
        std::unordered_map<std::string, std::unordered_map<std::string, bool>> _new_amb;

        set_base *_gset = nullptr;
        base_log spdlog;
        std::shared_ptr<gnss_all_ambflag> _gambflag = nullptr;

        int _index = -1;
        bool _apply_carrier_range = false; ///< whether apply carrier observation
    };
}

#endif // !GTURBOEDIT_H
