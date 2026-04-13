#include "hwa_gnss_amb_ow.h"
#include "hwa_gnss_sys.h"
#include "hwa_base_time.h"

namespace hwa_gnss
{
    AMB_ID gnss_amb::par2amb_id(const par_type &ambtype)
    {
        switch (ambtype)
        {
        case par_type::AMB_IF:
            return AMB_ID::IF12;
        case par_type::AMB13_IF:
            return AMB_ID::IF13;
        case par_type::AMB14_IF:
            return AMB_ID::IF14;
        case par_type::AMB15_IF:
            return AMB_ID::IF15;
        case par_type::AMB_L1:
            return AMB_ID::RAW1;
        case par_type::AMB_L2:
            return AMB_ID::RAW2;
        case par_type::AMB_L3:
            return AMB_ID::RAW3;
        case par_type::AMB_L4:
            return AMB_ID::RAW4;
        case par_type::AMB_L5:
            return AMB_ID::RAW5;
        default:
            return AMB_ID::UNDEF;
        }
    }

    par_type gnss_amb::amb_id2par(const AMB_ID &amb_id)
    {
        switch (amb_id)
        {
        case AMB_ID::IF12:
            return par_type::AMB_IF;
        case AMB_ID::IF13:
            return par_type::AMB13_IF;
        case AMB_ID::IF14:
            return par_type::AMB14_IF;
        case AMB_ID::IF15:
            return par_type::AMB15_IF;
        case AMB_ID::RAW1:
            return par_type::AMB_L1;
        case AMB_ID::RAW2:
            return par_type::AMB_L2;
        case AMB_ID::RAW3:
            return par_type::AMB_L3;
        case AMB_ID::RAW4:
            return par_type::AMB_L4;
        case AMB_ID::RAW5:
            return par_type::AMB_L5;
        default:
            return par_type::NO_DEF;
        }
    }

    std::string gnss_amb::ambId2str(AMB_ID ID)
    {
        switch (ID)
        {
        case AMB_ID::IF12:
            return "IF12";
        case AMB_ID::IF13:
            return "IF13";
        case AMB_ID::IF14:
            return "IF14";
        case AMB_ID::IF15:
            return "IF15";
        case AMB_ID::RAW1:
            return "RAW1";
        case AMB_ID::RAW2:
            return "RAW2";
        case AMB_ID::RAW3:
            return "RAW3";
        case AMB_ID::RAW4:
            return "RAW4";
        case AMB_ID::RAW5:
            return "RAW5";
        case AMB_ID::WL:
            return "WL";
        case AMB_ID::EWL:
            return "EWL";
        case AMB_ID::EWL24:
            return "EWL24";
        case AMB_ID::EWL25:
            return "EWL25";
        default:
            return "UNDEF";
        }
    }
    AMB_ID gnss_amb::str2ambid(const std::string &s)
    {
        std::string tmp = base_type_conv::trim(s);
        if (tmp == "IF12")
            return AMB_ID::IF12;
        else if (tmp == "IF13")
            return AMB_ID::IF13;
        else if (tmp == "IF14")
            return AMB_ID::IF14;
        else if (tmp == "IF15")
            return AMB_ID::IF15;
        else if (tmp == "RAW1")
            return AMB_ID::RAW1;
        else if (tmp == "RAW2")
            return AMB_ID::RAW2;
        else if (tmp == "RAW3")
            return AMB_ID::RAW3;
        else if (tmp == "RAW4")
            return AMB_ID::RAW4;
        else if (tmp == "RAW5")
            return AMB_ID::RAW5;
        else if (tmp == "WL")
            return AMB_ID::WL;
        else if (tmp == "EWL")
            return AMB_ID::EWL;
        else if (tmp == "EWL24")
            return AMB_ID::EWL24;
        else if (tmp == "EWL25")
            return AMB_ID::EWL25;
        else
            return AMB_ID::UNDEF;
    }

    gnss_amb::gnss_amb()
    {
    }

    gnss_amb::gnss_amb(std::string sat1, std::string sat2, std::string rec1, std::string rec2, base_time beg, base_time end) : _beg(beg), _end(end), _sat1(sat1), _sat2(sat2), _rec1(rec1), _rec2(rec2),
                                                                                                   _gsys(gnss_sys::str2gsys(sat1.substr(0, 1)))
    {
        _amb_type = AMB_TYPE::DD;
    };

    void gnss_amb::add_amb(const AMB_ID &amb_id, const base_time &beg, const base_time &end, double val, double sig)
    {
        _beg = MAX_TIME(_beg, beg);
        _end = MIN_TIME(_end, end);
        _value[amb_id] = val;
        _sigma[amb_id] = sig;
        _upd_cor[amb_id] = 0.0;
        _fixed[amb_id] = false;
    }

    void gnss_amb::add_amb(const AMB_ID &amb_id, double val, double sig, double cor, bool fixed)
    {
        _value[amb_id] = val;
        _sigma[amb_id] = sig;
        _upd_cor[amb_id] = cor;
        _fixed[amb_id] = std::fixed;
    }

    void gnss_amb::del_amb(const AMB_ID &amb_id)
    {
        if (_value.find(amb_id) != _value.end())
            _value.erase(amb_id);
        if (_sigma.find(amb_id) != _sigma.end())
            _sigma.erase(amb_id);
        if (_fixed.find(amb_id) != _fixed.end())
            _fixed.erase(amb_id);
    }

    bool gnss_amb::check_amb(const std::set<AMB_ID> &ids) const
    {
        std::set<AMB_ID> ids_loc = amb_ids();
        for (const auto &id : ids)
        {
            if (ids_loc.find(id) == ids_loc.end())
                return false;
            if (double_eq(_value.at(id), 0) || _sigma.at(id) > 10)
                return false;
        }
        return true;
    }

    int gnss_amb::frequency() const
    {
        std::set<AMB_ID> ids = amb_ids();
        if (ids.find(AMB_ID::IF15) != ids.end() || ids.find(AMB_ID::RAW5) != ids.end() || ids.find(AMB_ID::EWL25) != ids.end())
            return 5;
        if (ids.find(AMB_ID::IF14) != ids.end() || ids.find(AMB_ID::RAW4) != ids.end() || ids.find(AMB_ID::EWL24) != ids.end())
            return 4;
        if (ids.find(AMB_ID::IF13) != ids.end() || ids.find(AMB_ID::RAW3) != ids.end() || ids.find(AMB_ID::EWL) != ids.end())
            return 3;
        return 2;
    }

    int gnss_amb::freq_fixed() const
    {
        std::set<AMB_ID> ids = amb_ids();
        int num = 0;
        if (ids.find(AMB_ID::EWL25) != ids.end() && _fixed.at(AMB_ID::EWL25))
            ++num;
        if (ids.find(AMB_ID::EWL24) != ids.end() && _fixed.at(AMB_ID::EWL24))
            ++num;
        if (ids.find(AMB_ID::EWL) != ids.end() && _fixed.at(AMB_ID::EWL))
            ++num;
        if (ids.find(AMB_ID::WL) != ids.end() && _fixed.at(AMB_ID::WL))
            ++num;
        if (ids.find(AMB_ID::RAW1) != ids.end() && _fixed.at(AMB_ID::RAW1))
            ++num;
        return num;
    }

    bool gnss_amb_ow::check_ow_time(const base_time &beg, const base_time &end, double min_common_time)
    {
        base_time tb = MAX_TIME(beg, _beg);
        base_time te = MIN_TIME(end, _end);
        double mct = std::max(min_common_time, this->seslen() / 2);
        return (te - tb > mct);
    }

    gnss_amb_SD::gnss_amb_SD(const gnss_amb_ow &ow1, const gnss_amb_ow &ow2)
    {
        if (ow1.sat() < ow2.sat())
            _form_sd(ow1, ow2);
        else
            _form_sd(ow2, ow1);
    }

    void gnss_amb_SD::exchange_sat()
    {
        std::string sat = _sat1;
        _sat1 = _sat2;
        _sat2 = sat;

        for (auto &val : _value)
        {
            val.second = -1 * val.second;
        }

        int idx = _idx[0];
        _idx[0] = _idx[1];
        _idx[1] = idx;
    }

    void gnss_amb_SD::_form_sd(const gnss_amb_ow &ow1, const gnss_amb_ow &ow2)
    {
        _amb_type = AMB_TYPE::SD;
        _sat1 = ow1.sat();
        _sat2 = ow2.sat();
        _rec1 = ow1.rec();
        _gsys = gnss_sys::str2gsys(ow1.sat().substr(0, 1));
        _beg = MAX_TIME(ow1.t_beg(), ow2.t_beg());
        _end = MIN_TIME(ow1.t_end(), ow2.t_end());
        for (const auto &iter : ow1.all_value())
        {
            AMB_ID amb_id = iter.first;
            if (double_eq(ow2.value(amb_id), 0) || double_eq(ow1.sigma(amb_id) * ow2.sigma(amb_id), 0))
                continue;

            _value[amb_id] = iter.second - ow2.value(amb_id);
            _sigma[amb_id] = sqrt(ow1.sigma(amb_id) * ow1.sigma(amb_id) + ow2.sigma(amb_id) * ow2.sigma(amb_id));
            _fixed[amb_id] = false;
            _upd_cor[amb_id] = 0.0;
        }
        _idx[0] = ow1.idx();
        _idx[1] = ow2.idx();
    }

    gnss_amb_DD::gnss_amb_DD(const gnss_amb_SD &sd1, const gnss_amb_SD &sd2)
    {
        gnss_amb_SD sd1_tmp = sd1;
        gnss_amb_SD sd2_tmp = sd2;

        // ensure the sequence of two satellites is the same
        if (sd1_tmp.sats().first > sd1_tmp.sats().second)
            sd1_tmp.exchange_sat();
        if (sd2_tmp.sats().first > sd2_tmp.sats().second)
            sd2_tmp.exchange_sat();

        _amb_type = AMB_TYPE::DD;
        _sat1 = sd1_tmp.sats().first;
        _sat2 = sd1_tmp.sats().second;
        _rec1 = sd1_tmp.rec();
        _rec2 = sd2_tmp.rec();

        _gsys = gnss_sys::str2gsys(_sat1.substr(0, 1));
        _beg = MAX_TIME(sd1_tmp.t_beg(), sd2_tmp.t_beg());
        _end = MIN_TIME(sd1_tmp.t_end(), sd2_tmp.t_end());
        for (const auto &amb_id : sd1_tmp.amb_ids())
        {
            if (double_eq(sd1_tmp.value(amb_id) * sd2_tmp.value(amb_id) * sd1_tmp.sigma(amb_id) * sd2_tmp.sigma(amb_id), 0))
                continue;

            _value[amb_id] = sd1_tmp.value(amb_id) - sd2_tmp.value(amb_id);
            _sigma[amb_id] = sqrt(sd1_tmp.sigma(amb_id) * sd1_tmp.sigma(amb_id) + sd2_tmp.sigma(amb_id) * sd2_tmp.sigma(amb_id));
            _fixed[amb_id] = false;
            _upd_cor[amb_id] = 0.0;
        }
        // get DD index
        _idx = {sd1_tmp.idx()[0], sd1_tmp.idx()[1], sd2_tmp.idx()[0], sd2_tmp.idx()[1]};
    }

    /*gnss_amb_DD::gnss_amb_DD(const gnss_amb_ow& ow1, const gnss_amb_ow& ow2, const gnss_amb_ow& ow3, const gnss_amb_ow& ow4) {
        _amb_type = AMB_TYPE::DD;
        _sat1 = ow1.sat();
        _sat2 = ow2.sat();
        _rec1 = ow1.rec();
        _rec2 = ow3.rec();
        _gsys = gnss_sys::str2gsys(_sat1.substr(0, 1));
        _beg = MAX_TIME({ ow1.t_beg(), ow2.t_beg(), ow3.t_beg(), ow4.t_beg() });
        _end = MIN_TIME({ ow1.t_end(), ow2.t_end(), ow3.t_end(), ow4.t_end() });
        for (const auto& iter : ow1.all_value()) {
            AMB_ID amb_id = iter.first;
            if (!double_eq(ow2.value(amb_id)* ow3.value(amb_id)* ow4.value(amb_id), 0))
                _value[amb_id] = iter.second - ow2.value(amb_id) - ow3.value(amb_id) + ow4.value(amb_id);
            if (!double_eq(ow1.sigma(amb_id) * ow2.sigma(amb_id)* ow3.sigma(amb_id)* ow4.sigma(amb_id), 0))
                _sigma[amb_id] = sqrt(ow1.sigma(amb_id) * ow1.sigma(amb_id) + ow2.sigma(amb_id) * ow2.sigma(amb_id) + 
                    ow3.sigma(amb_id) * ow3.sigma(amb_id) + ow4.sigma(amb_id) * ow4.sigma(amb_id));
        }
        _idx = { ow1.idx(), ow2.idx(), ow3.idx(), ow4.idx() };
    }*/

    bool check_sd_amb(const gnss_amb_ow &ow1, const gnss_amb_ow &ow2, double min_common_time)
    {
        // check receiver
        if (ow1.rec() != ow2.rec())
            return false;
        // check GSYS
        if (ow1.sat().substr(0, 1) != ow2.sat().substr(0, 1))
            return false;
        // check sat
        if (ow1.sat() == ow2.sat())
            return false;
        // check idx
        if (ow1.idx() < 0 || ow2.idx() < 0)
            return false;
        // check commen time
        base_time beg = MAX_TIME(ow1.t_beg(), ow2.t_beg());
        base_time end = MIN_TIME(ow1.t_end(), ow2.t_end());
        return (end - beg > min_common_time);
    }

    bool check_dd_amb(const gnss_amb_SD &sd1, const gnss_amb_SD &sd2, double min_common_time)
    {
        // check receiver
        if (sd1.rec() == sd2.rec())
            return false;
        // check satellites
        bool b1 = (sd1.sats() == sd2.sats());
        bool b2 = (sd1.sats().first == sd2.sats().second && sd1.sats().second == sd2.sats().first);
        if (!b1 && !b2)
            return false;
        // check idx
        if (sd1.idx()[0] < 0 || sd1.idx()[1] < 0 || sd2.idx()[0] < 0 || sd2.idx()[1] < 0)
            return false;
        // check commen time
        base_time beg = MAX_TIME(sd1.t_beg(), sd2.t_beg());
        base_time end = MIN_TIME(sd1.t_end(), sd2.t_end());
        return (end - beg > min_common_time);
    }
}
