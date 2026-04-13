/**
* @file        gambcon.h
* @brief    Storage the ambigulity constraint files data
*
*/

#ifndef hwa_gnss_data_ambcon_H
#define hwa_gnss_data_ambcon_H

#include "hwa_base_data.h"
#include "hwa_gnss_amb_ow.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /**
    *@brief     Class for storaging ambinp file data
    */
    class gnss_data_ambcon : public base_data
    {
    public:
        /** @brief default constructor. */
        gnss_data_ambcon();

        /** @brief default constructor. */
        gnss_data_ambcon(base_log spdlog);

        /** @brief default destructor. */
        virtual ~gnss_data_ambcon() = default;
        /**
        * @brief add double difference between satellites
        * @param[in]  dds        double difference between satellites
        */
        void add_amb(const std::vector<std::shared_ptr<gnss_amb_DD>> dds)
        {
            for (const auto &amb : dds)
            {
                _vamb.push_back(amb);
                _all_amb[amb->t_beg()].push_back(amb);
            }
        }
        /**
        * @brief add single difference between satellites
        * @param[in]  sds        single difference between satellites
        */
        void add_amb(const std::vector<std::shared_ptr<gnss_amb_SD>> sds)
        {
            for (const auto &amb : sds)
            {
                _vamb.push_back(amb);
                _all_amb[amb->t_beg()].push_back(amb);
            }
        }
        /**
        * @brief add ambiguity data
        * @param[in]  amb        ambiguity
        */
        void add_amb(const gnss_amb &amb)
        {
            _vamb.push_back(std::make_shared<gnss_amb>(amb));
            _all_amb[amb.t_beg()].push_back(std::make_shared<gnss_amb>(amb));
        }
        /**
        * @brief add double differences
        * @param[in]  dd        double differences
        */
        void add_amb(const gnss_amb_DD &dd)
        {
            _vamb.push_back(std::make_shared<gnss_amb>(dd));
            _all_amb[dd.t_beg()].push_back(std::make_shared<gnss_amb>(dd));
        }
        /**
        * @brief add single differences
        * @param[in]  sd        single differences
        */
        void add_amb(const gnss_amb_SD &sd)
        {
            _vamb.push_back(std::make_shared<gnss_amb>(sd));
            _all_amb[sd.t_beg()].push_back(std::make_shared<gnss_amb>(sd));
        }
        /**
        * @brief add one tpye ambiguity
        * @param[in]  id        id of ambiguity
        * @param[in]  beg        begin time
        * @param[in]  val        value
        * @param[in]  sig        single
        * @param[in]  upd        upd value
        * @param[in]  std::fixed        std::fixed or not
        */
        void add_ambe_onetype(const AMB_ID &id, const base_time &beg, double val, double sig, double upd, bool fixed)
        {
            (*_vamb.rbegin())->add_amb(id, val, sig, upd, std::fixed);
            (*_all_amb[beg].rbegin())->add_amb(id, val, sig, upd, std::fixed);
        }
        /**
        * @brief erase old ambiguity
        * @param[in]  t        time
        */
        void erase_old_amb(const base_time &t)
        {
            auto iter = _all_amb.upper_bound(t);
            if (iter == _all_amb.end())
                return;
            _all_amb.erase(_all_amb.begin(), iter);
        }
        /** @brief get all std::fixed ambiguity */
        std::vector<std::shared_ptr<gnss_amb>> get_amb_all() const { return _vamb; }
        /** @brief get std::fixed ambiguity */
        std::vector<std::shared_ptr<gnss_amb>> get_amb(const base_time &t) const { return _all_amb.find(t) != _all_amb.end() ? _all_amb.at(t) : std::vector<std::shared_ptr<gnss_amb>>(); }

        // get properties
        /**
         * @brief 
         * 
         * @return base_time 
         */
        const base_time &t_beg() const { return _beg; } ///< begin time

        /**
         * @brief 
         * 
         * @return base_time 
         */
        const base_time &t_end() const { return _end; } ///< end time

        /**
         * @brief 
         * 
         * @return set<std::string> 
         */
        const std::set<std::string> &sat_list() const { return _sat_list; } ///< satellite list

        /**
         * @brief 
         * 
         * @return set<std::string> 
         */
        const std::set<std::string> &rec_list() const { return _rec_list; } ///< receiver list

        /**
         * @brief 
         * 
         * @return set<std::string> 
         */
        const std::set<std::string> &gsys() const { return _gsys; } ///< gnss system

        /**
         * @brief 
         * 
         * @return AMB_TYPE 
         */
        const AMB_TYPE &mode() const { return _mode; } ///< ambiguity std::fixed mode

        /**
         * @brief 
         * 
         * @return std::map<AMB_ID, std::map<GSYS, int>> 
         */
        const std::map<AMB_ID, std::map<GSYS, int>> &num_fixed() const { return _num_fixed; } ///< number of fix

        /**
         * @brief 
         * 
         * @return std::map<AMB_ID, std::map<GSYS, int>> 
         */
        const std::map<AMB_ID, std::map<GSYS, int>> &num_all() const { return _num_all; } ///< number of all ambiguity

        /**
         * @brief 
         * 
         * @return set<AMB_ID> 
         */
        std::set<AMB_ID> amb_ids() const ///< id of ambiguity
        {
            std::set<AMB_ID> ids;
            for (const auto &iter : _num_fixed)
                if (_num_all.find(iter.first) != _num_all.end())
                    ids.insert(iter.first);
            return ids;
        }

        // set properties
        /**
        * @brief set number of ambiguity
        * @param[in]  num_fixed        number of fix
        * @param[in]  num_all        number of all ambiguity
        */
        void set_amb_num(const std::map<AMB_ID, std::map<GSYS, int>> &num_fixed, const std::map<AMB_ID, std::map<GSYS, int>> &num_all)
        {
            _num_fixed = num_fixed;
            _num_all = num_all;
        }
        /**
        * @brief set list of satellite and receiver
        * @param[in]  sats        satellite list
        * @param[in]  recs        receiver list
        */
        void set_sat_rec(const std::set<std::string> &sats, const std::set<std::string> &recs)
        {
            _sat_list = sats;
            _rec_list = recs;
        }
        /**
        * @brief set begin time and end time
        * @param[in]  beg        begin time
        * @param[in]  end        end time
        */
        void set_time(const base_time &beg, const base_time &end)
        {
            _beg = beg;
            _end = end;
        }
        /** @brief set GNSS system */
        void segnss_sys(const std::set<std::string> &s) { _gsys = s; }
        /** @brief set type of ambiguity */
        void set_type(const AMB_TYPE &tp) { _mode = tp; }

    protected:
        std::vector<std::shared_ptr<gnss_amb>> _vamb;                  ///< std::vector of ambiguity
        std::map<base_time, std::vector<std::shared_ptr<gnss_amb>>> _all_amb; ///< all ambiguity

        base_time _beg;                  ///< begin time
        base_time _end;                  ///< end time
        std::set<std::string> _sat_list;         ///< satellite list
        std::set<std::string> _rec_list;         ///< receiver list
        std::set<std::string> _gsys;             ///< gnss system
        AMB_TYPE _mode = AMB_TYPE::DD; ///< ambiguity type

        std::map<AMB_ID, std::map<GSYS, int>> _num_fixed; ///< number of fix
        std::map<AMB_ID, std::map<GSYS, int>> _num_all;   ///< number of all ambiguity
    };
}
#endif // !GAMBCON_H