#ifndef hwa_gnss_amb_ow_H
#define hwa_gnss_amb_ow_H

#include "hwa_set_gtype.h"
#include "hwa_set_amb.h"
#include "hwa_base_par.h"
#include "hwa_gnss_amb_bdeci.h"

using namespace hwa_set;
using namespace hwa_base;

namespace hwa_gnss
{
    enum class AMB_ID
    {
        IF12,
        IF13,
        IF14,
        IF15,
        RAW1,
        RAW2,
        RAW3,
        RAW4,
        RAW5,
        WL,
        EWL,
        EWL24,
        EWL25,
        UNDEF
    };
    /**
     * @brief Class for ambiguity
     */
    class gnss_amb
    {
    public:
        /**
         * @brief parameter to ambiguity id
         * @param[in]  ambtype   parameter type
         * @return AMB_ID ID of ambiguity
         */
        static AMB_ID par2amb_id(const par_type &ambtype);
        /**
         * @brief ambiguity id to parameter
         * @param[in]  amb_id    ambiguity id
         * @return par_type parameter type
         */
        static par_type amb_id2par(const AMB_ID &amb_id);
        /**
         * @brief ambiguity id 2 std::string
         * @param[in]  ID        ambguity
         * @return std::string std::string id
         */
        static std::string ambId2str(AMB_ID ID);
        /**
         * @brief std::string id ro enum
         * @param[in]  s         std::string id
         * @return AMB_ID enum id
         */
        static AMB_ID str2ambid(const std::string &s);
        /**
         * @brief Construct a new t gamb object
         */
        gnss_amb();

        /**
         * @brief Construct a new t gamb object
         * @param[in]  sat1      satellite1
         * @param[in]  sat2      satellite2
         * @param[in]  rec1      station1
         * @param[in]  rec2      station2
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        explicit gnss_amb(std::string sat1, std::string sat2, std::string rec1, std::string rec2, base_time beg, base_time end);

        /**
         * @brief Construct a new t gamb object
         * @param[in]  amb       ambiguity data
         */
        gnss_amb(const gnss_amb &amb) = default;

        /**
         * @brief Destroy the t gamb object
         */
        virtual ~gnss_amb() = default;
        /**
         * @brief get begin time
         * @return base_time begin time
         */
        base_time t_beg() const { return _beg; }
        /**
         * @brief get end time
         * @return base_time end time
         */
        base_time t_end() const { return _end; }
        /**
         * @brief get length of time
         * @return double length of time
         */
        double seslen() const { return _end - _beg; }
        /**
         * @brief Set the time
         * @param[in]  t1        begin time
         * @param[in]  t2        end time
         */
        void set_time(const base_time &t1, const base_time &t2)
        {
            _beg = t1;
            _end = t2;
        }
        /**
         * @brief get ambiguity type
         * @return AMB_TYPE ambiguity type
         */
        AMB_TYPE amb_type() const { return _amb_type; }
        /**
         * @brief get satellite std::pair
         * @return std::pair<std::string, std::string> satellite name std::pair 
         */
        std::pair<std::string, std::string> sats() const { return {_sat1, _sat2}; }
        /**
         * @brief get station std::pair
         * @return std::pair<std::string, std::string> station name std::pair
         */
        std::pair<std::string, std::string> recs() const { return {_rec1, _rec2}; }
        /**
         * @brief get all value
         * @return std::map<AMB_ID, double> type/value
         */
        std::map<AMB_ID, double> all_value() const { return _value; }
        /**
         * @brief get all sigma
         * @return std::map<AMB_ID, double> type/sigma
         */
        std::map<AMB_ID, double> all_sigma() const { return _sigma; }
        /**
         * @brief get all whether std::fixed
         * @return std::map<AMB_ID, double> type/std::fixed
         */
        std::map<AMB_ID, bool> all_fixed() const { return _fixed; }
        /**
         * @brief get all upd correction
         * @return std::map<AMB_ID, double> type/correction
         */
        std::map<AMB_ID, double> all_upd_cor() const { return _upd_cor; }
        /**
         * @brief get value of certain ambiguity type
         * @param[in]  amb_id    ambiguity type
         * @return double value of ambiguity
         */
        double value(const AMB_ID &amb_id) const { return (_value.find(amb_id) != _value.end()) ? _value.at(amb_id) : 0.0; }
        /**
         * @brief get sigma of certain ambiguity type
         * @param[in]  amb_id    ambiguity type
         * @return double sigma of ambiguity
         */
        double sigma(const AMB_ID &amb_id) const { return (_sigma.find(amb_id) != _sigma.end()) ? _sigma.at(amb_id) : 0.0; }
        /**
         * @brief get std::fixed of certain ambiguity type
         * @param[in]  amb_id    ambiguity type
         * @return double whether std::fixed of ambiguity
         */
        double fixed(const AMB_ID &amb_id) const { return (_fixed.find(amb_id) != _fixed.end()) ? _fixed.at(amb_id) : false; }
        /**
         * @brief get upd correction of certain ambiguity type
         * @param[in]  amb_id    ambiguity type
         * @return double upd correction of ambiguity
         */
        double upd_cor(const AMB_ID &amb_id) const { return (_upd_cor.find(amb_id) != _upd_cor.end() ? _upd_cor.at(amb_id) : 0.0); }
        /**
         * @brief Set the type
         * @param[in]  tp        type of ambiguity
         */
        void set_type(const AMB_TYPE &tp) { _amb_type = tp; }
        /**
         * @brief Set the value
         * @param[in]  amb_id    type of ambiguity
         * @param[in]  val       value
         */
        void set_value(const AMB_ID &amb_id, double val) { _value[amb_id] = val; }
        /**
         * @brief Set the sigma
         * @param[in]  amb_id    type of ambiguity
         * @param[in]  sig       sigma
         */
        void set_sigma(const AMB_ID &amb_id, double sig) { _sigma[amb_id] = sig; }
        /**
         * @brief Set the std::fixed
         * @param[in]  amb_id    type of ambiguity
         * @param[in]  b         whether std::fixed
         */
        void set_fixed(const AMB_ID &amb_id, bool b) { _fixed[amb_id] = b; }
        /**
         * @brief Set the upd correction
         * @param[in]  amb_id    type of ambiguity
         * @param[in]  upd       upd correction
         */
        void set_upd_cor(const AMB_ID &amb_id, double upd) { _upd_cor[amb_id] = upd; }
        /**
         * @brief add ambiguity
         * @param[in]  amb_id    type of ambiguity
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         * @param[in]  val       value
         * @param[in]  sig       sigma
         */
        void add_amb(const AMB_ID &amb_id, const base_time &beg, const base_time &end, double val, double sig);
        /**
         * @brief add ambiguity
         * @param[in]  amb_id    type of ambiguity
         * @param[in]  val       value
         * @param[in]  sig       sigma
         * @param[in]  cor       correction
         * @param[in]  std::fixed     whether std::fixed
         */
        void add_amb(const AMB_ID &amb_id, double val, double sig, double cor, bool fixed);
        /**
         * @brief delete certain ambiguity
         * @param[in]  amb_id    type of ambiguity
         */
        void del_amb(const AMB_ID &amb_id);
        /**
         * @brief check ambiguity
         * @param[in]  ids       type of ambiguity
         */
        bool check_amb(const std::set<AMB_ID> &ids) const;
        /**
         * @brief get system
         * @return GSYS system
         */
        GSYS gsys() const { return _gsys; }
        /**
         * @brief get type of ambiguity
         * @return std::set<AMB_ID> type of ambiguity list
         */
        std::set<AMB_ID> amb_ids() const
        {
            std::set<AMB_ID> ids;
            for (const auto &iter : _value)
                ids.insert(iter.first);
            return ids;
        }
        /**
         * @brief get frequency
         * @return int frequency
         */
        int frequency() const;
        /**
         * @brief get number of std::fixed
         * @return int number
         */
        int freq_fixed() const;

        bool is_used = false;      ///< whether used
        bool is_dependent = false; ///< whether dependent

    protected:
        AMB_TYPE _amb_type = AMB_TYPE::UD; ///< type of ambiguity
        base_time _beg;                      ///< begin time
        base_time _end;                      ///< end time
        std::string _sat1;                      ///< satellite1 name
        std::string _rec1;                      ///< station1 name
        std::string _sat2;                      ///< satellite2 name
        std::string _rec2;                      ///< station2 name
        GSYS _gsys = GSYS::GPS;            ///< system

        std::map<AMB_ID, double> _value;       ///< unit: cycle
        std::map<AMB_ID, double> _sigma;       ///< sigma
        std::map<AMB_ID, bool> _fixed;         ///< whether std::fixed
        std::map<AMB_ID, double> _upd_cor;     ///< upd correction
        std::vector<int> _idx{-1, -1, -1, -1}; ///< TODO
    };
    /**
     * @brief Class for ambiguity ow
     */
    class gnss_amb_ow : public gnss_amb
    {
    public:
        /**
         * @brief Construct a new t gamb Ow object
         * @param[in]  sat       satellite name
         * @param[in]  rec       statione name
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        gnss_amb_ow(const std::string &sat, const std::string &rec, const base_time &beg, const base_time &end) : gnss_amb(sat, "", rec, "", beg, end) { _amb_type = AMB_TYPE::UD; }
        /**
         * @brief Construct a new t gamb Ow object
         * @param[in]  ow        ambiguity ow
         */
        gnss_amb_ow(const gnss_amb_ow &ow) = default;
        /**
         * @brief Destroy the t gamb Ow object
         */
        virtual ~gnss_amb_ow() = default;
        /**
         * @brief check ow time
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         * @param[in]  min_common_time minimum conmmon time
         */
        bool check_ow_time(const base_time &beg, const base_time &end, double min_common_time);
        /**
         * @brief get satellite name
         * @return std::string satellite name
         */
        std::string sat() const { return _sat1; }
        /**
         * @brief get station name
         * @return std::string station name
         */
        std::string rec() const { return _rec1; }
        /**
         * @brief TODO
         * @return int index
         */
        int idx() const { return _idx[0]; }
        /**
         * @brief Set the index
         * @param[in]  i         index
         */
        void set_idx(int i) { _idx[0] = i; }
    };

    /**
     * @brief Class for single-difference ambiguity (between satellites)
     */
    class gnss_amb_SD : public gnss_amb
    {
    public:
        /**
         * @brief Construct a new t gamb Sd object
         * @param[in]  sat1      satellite1 name
         * @param[in]  sat2      satellite2 name
         * @param[in]  rec       station name
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        gnss_amb_SD(std::string sat1, std::string sat2, std::string rec, base_time beg, base_time end) : gnss_amb(sat1, sat2, rec, "", beg, end) { _amb_type = AMB_TYPE::SD; }
        /**
         * @brief Construct a new t gamb Sd object
         * @param[in]  ow1       TODO
         * @param[in]  ow2       TODO
         */
        gnss_amb_SD(const gnss_amb_ow &ow1, const gnss_amb_ow &ow2);
        /**
         * @brief Construct a new t gamb Sd object
         * @param[in]  Sd        single difference
         */
        gnss_amb_SD(const gnss_amb_SD &Sd) = default;
        /**
         * @brief Destroy the t gamb Sd object
         */
        virtual ~gnss_amb_SD() = default;
        /**
         * @brief get station name
         * @return std::string 
         */
        std::string rec() const { return _rec1; }
        /**
         * @brief get index
         * @return std::vector<int> index
         */
        std::vector<int> idx() const { return {_idx[0], _idx[1]}; }
        /**
         * @brief Set the index
         * @param[in]  i1        index1
         * @param[in]  i2        index2
         */
        void set_idx(int i1, int i2)
        {
            _idx[0] = i1;
            _idx[1] = i2;
        }
        /**
         * @brief exchange satellite1 and satellite2
         */
        void exchange_sat();

    private:
        /**
         * @brief std::set from ow
         * @param[in]  ow1       ow1
         * @param[in]  ow2       ow1
         */
        void _form_sd(const gnss_amb_ow &ow1, const gnss_amb_ow &ow2);
    };

    /**
     * @brief class for double difference ambiguity
     */
    class gnss_amb_DD : public gnss_amb
    {
    public:
        /**
         * @brief Construct a new t gamb Dd object
         * @param[in]  sat1      satellite1
         * @param[in]  sat2      satellite2
         * @param[in]  rec1      station1
         * @param[in]  rec2      station2
         * @param[in]  beg       begin time
         * @param[in]  end       end time
         */
        gnss_amb_DD(std::string sat1, std::string sat2, std::string rec1, std::string rec2, base_time beg, base_time end) : gnss_amb(sat1, sat2, rec1, rec2, beg, end){};
        /**
         * @brief Construct a new t gamb Dd object
         * @param[in]  sd1       single difference1
         * @param[in]  sd2       single difference2
         */
        gnss_amb_DD(const gnss_amb_SD &sd1, const gnss_amb_SD &sd2);
        // gnss_amb_DD(const gnss_amb_ow& ow1, const gnss_amb_ow& ow2, const gnss_amb_ow& ow3, const gnss_amb_ow& ow4);
        /**
         * @brief Construct a new t gamb Dd object
         * @param[in]  Dd        double difference
         */
        gnss_amb_DD(const gnss_amb_DD &Dd) = default;
        /**
         * @brief Destroy the t gamb Dd object
         */
        virtual ~gnss_amb_DD() = default;
        /**
         * @brief get index
         * @return std::vector<int> index 
         */
        std::vector<int> idx() const { return {_idx[0], _idx[1], _idx[2], _idx[3]}; }
        /**
         * @brief Set the index
         * @param[in]  i1        index1
         * @param[in]  i2        index2
         * @param[in]  i3        index3
         * @param[in]  i4        index4
         */
        void set_idx(int i1, int i2, int i3, int i4)
        {
            _idx[0] = i1;
            _idx[1] = i2;
            _idx[2] = i3;
            _idx[3] = i4;
        }
    };
    /**
     * @brief check single difference
     * @param[in]  ow1       ow1
     * @param[in]  ow2       ow2
     * @param[in]  min_common_time minimum commont time
     */
    bool check_sd_amb(const gnss_amb_ow &ow1, const gnss_amb_ow &ow2, double min_common_time);
    /**
     * @brief check double difference
     * @param[in]  ow1       single difference1
     * @param[in]  ow2       single difference2
     * @param[in]  min_common_time minimum commont time
     */
    bool check_dd_amb(const gnss_amb_SD &sd1, const gnss_amb_SD &sd2, double min_common_time);
}
#endif