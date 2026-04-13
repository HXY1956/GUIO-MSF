#include "hwa_gnss_coder_ambcon.h"
#include "hwa_gnss_amb_ow.h"
#include "hwa_gnss_data_ambcon.h"
#include "hwa_gnss_sys.h"

namespace hwa_gnss
{
    gnss_coder_ambcon::gnss_coder_ambcon(set_base *s, std::string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder()
    {
    }

    /** @brief destructor. */
    gnss_coder_ambcon::~gnss_coder_ambcon()
    {
    }

    int gnss_coder_ambcon::decode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {

        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        int consume = 0;
        int tmpsize = 0;
        std::string line;
        std::string word;
        try
        {
            while ((tmpsize = base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;
                if (line.find("%=AMB") == 0)
                { // first line
                    // double ver = base_type_conv::str2dbl(line.substr(6, 4));
                    std::string ac = line.substr(11, 3);
                    _beg.from_str("%Y:%j:%s", line.substr(34, 14));
                    _end.from_str("%Y:%j:%s", line.substr(49, 14));
                    _mode = line.substr(64, 1);
                    if (_mode == "D")
                        _amb_type = AMB_TYPE::DD;
                    else if (_mode == "S")
                        _amb_type = AMB_TYPE::SD;
                    else if (_mode == "U")
                        _amb_type = AMB_TYPE::UD;
                    else
                        _amb_type = AMB_TYPE::UNDEF;
                }
                else if (line.find("+AMB/SATELLITE") == 0)
                {
                    _block = "SATELLITE";
                }
                else if (line.find("+AMB/RECEIVER") == 0)
                {
                    _block = "RECEIVER";
                }
                else if (line.substr(0, 1) == "-")
                {
                    _block = "";
                }
                else if (line.substr(0, 1) == " " && !_block.empty())
                {
                    if (_block == "SATELLITE")
                    {
                        std::istringstream ss(line);
                        while (ss >> word)
                            _sats.insert(word);
                    }
                    else if (_block == "RECEIVER")
                    {
                        std::istringstream ss(line);
                        while (ss >> word)
                            _sites.insert(word);
                    }
                }
                else if (line.find("+AMB/SOLUTION") != std::string::npos || line.find("%=ENDAMB") != std::string::npos)
                {
                    base_coder::_consume(tmpsize);
                    _mutex.unlock();
                    return -1;
                }
                base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambcon::decode_head throw exception");
            return -1;
        }
    }

    int gnss_coder_ambcon::decode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {

        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };

        int consume = 0;
        int tmpsize = 0;
        // bool ambvalid = false;
        std::string line;
        try
        {
            while ((tmpsize = base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;
                if (line.find("-AMB/SOLUTION") != std::string::npos || line.find("%=ENDAMB") == 0)
                {
                    _mutex.unlock();
                    return 0;
                }
                else if (line.substr(0, 1) == " " && line.size() > 123 && gnss_amb::str2ambid(line.substr(8, 6)) != AMB_ID::UNDEF)
                {
                    int idx = base_type_conv::str2int(line.substr(1, 6));
                    AMB_ID id = gnss_amb::str2ambid(line.substr(8, 6));
                    std::string sat1(base_type_conv::trim(line.substr(15, 3))), sat2(base_type_conv::trim(line.substr(20, 3)));
                    std::string rec1(base_type_conv::trim(line.substr(25, 4))), rec2(base_type_conv::trim(line.substr(31, 4)));
                    base_time beg, end;
                    beg.from_str("%Y:%j:%s", line.substr(37, 14));
                    end.from_str("%Y:%j:%s", line.substr(52, 14));
                    double val = base_type_conv::str2dbl(line.substr(72, 21));
                    double sig = base_type_conv::str2dbl(line.substr(94, 11));
                    double upd = base_type_conv::str2dbl(line.substr(106, 14));
                    bool isfixed = (line.substr(121, 1) == "Y" || line.substr(121, 1) == "y");
                    if (idx > _idx_save)
                    {
                        gnss_amb amb(sat1, sat2, rec1, rec2, beg, end);
                        amb.add_amb(id, val, sig, upd, isfixed);
                        amb.set_type(_amb_type);
                        for (auto &iter : _data)
                        {
                            if (iter.second->id_type() == base_data::AMBCON)
                            {
                                dynamic_cast<gnss_data_ambcon *>(iter.second)->add_amb(amb);
                            }
                        }
                    }
                    else
                    {
                        for (auto &iter : _data)
                        {
                            if (iter.second->id_type() == base_data::AMBCON)
                            {
                                dynamic_cast<gnss_data_ambcon *>(iter.second)->add_ambe_onetype(id, beg, val, sig, upd, isfixed);
                            }
                        }
                    }
                    _idx_save = idx;
                }
                base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambcon::decode_data throw exception");
            return -1;
        }
    }

    int gnss_coder_ambcon::encode_head(char *buff, int sz, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                std::map<AMB_ID, std::map<GSYS, int>> num_all;
                std::map<AMB_ID, std::map<GSYS, int>> num_fixed;
                int namb = 0;
                std::set<std::string> gsys;
                for (const auto &iter : _data)
                {
                    if (iter.second->id_type() == base_data::AMBCON)
                    {
                        _sats = dynamic_cast<gnss_data_ambcon *>(iter.second)->sat_list();
                        _sites = dynamic_cast<gnss_data_ambcon *>(iter.second)->rec_list();
                        _beg = dynamic_cast<gnss_data_ambcon *>(iter.second)->t_beg();
                        _end = dynamic_cast<gnss_data_ambcon *>(iter.second)->t_end();
                        _amb_ids = dynamic_cast<gnss_data_ambcon *>(iter.second)->amb_ids();

                        AMB_TYPE tmp = dynamic_cast<gnss_data_ambcon *>(iter.second)->mode();
                        if (tmp == AMB_TYPE::SD)
                            _mode = "S";
                        else if (tmp == AMB_TYPE::UD)
                            _mode = "U";

                        gsys = dynamic_cast<gnss_data_ambcon *>(iter.second)->gsys();
                        num_all = dynamic_cast<gnss_data_ambcon *>(iter.second)->num_all();
                        num_fixed = dynamic_cast<gnss_data_ambcon *>(iter.second)->num_fixed();
                        namb = dynamic_cast<gnss_data_ambcon *>(iter.second)->get_amb_all().size();
                    }
                }

                _ss << "%=AMB 1.00 GRT " << base_time().str_ydoysod() << " GRT " << _beg.str_ydoysod() << " " << _end.str_ydoysod() << " " << _mode << " " << std::setw(8) << std::setfill('0') << namb << std::setfill(' ') << std::endl;
                _ss << "*-------------------------------------------------------------------------------" << std::endl;
                _ss << "+AMB/SUMMARY" << std::endl;
                _ss << "*AMB___ SYSTEM FIXED_NUM___ ALL_NUM_____ PERCENTAGE__" << std::endl;
                for (const auto &id : _amb_ids)
                {
                    for (const std::string &gs : gsys)
                    {
                        int num1 = num_fixed[id][gnss_sys::str2gsys(gs)];
                        int num2 = num_all[id][gnss_sys::str2gsys(gs)];
                        _ss << " " << std::setw(6) << std::left << gnss_amb::ambId2str(id) << " " << gs << "    " << std::setw(12) << std::right << num1 << " "
                            << std::setw(12) << num2 << " " << std::setw(11) << std::fixed << std::setprecision(2) << (num2 == 0 ? 0 : 100.0 * num1 / num2) << '%' << std::endl;
                    }
                }
                _ss << "-AMB/SUMMARY" << std::endl;
                _ss << "*-------------------------------------------------------------------------------" << std::endl;

                _ss << "+AMB/SATELLITE" << std::endl;
                int count = 0;
                auto iter_sat = _sats.begin();
                while (iter_sat != _sats.end())
                {
                    _ss << " " << *iter_sat;
                    if (++count % 16 == 0 && count != _sats.size())
                        _ss << std::endl;
                    iter_sat++;
                }
                _ss << std::endl;
                _ss << "-AMB/SATELLITE" << std::endl;
                _ss << "*-------------------------------------------------------------------------------" << std::endl;

                _ss << "+AMB/RECEIVER" << std::endl;
                count = 0;
                auto iter_rec = _sites.begin();
                while (iter_rec != _sites.end())
                {
                    _ss << " " << *iter_rec;
                    if (++count % 16 == 0 && count != _sites.size())
                        _ss << std::endl;
                    iter_rec++;
                }
                _ss << std::endl;
                _ss << "-AMB/RECEIVER" << std::endl;
                _ss << "*-------------------------------------------------------------------------------" << std::endl;
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambcon::encode_head throw exception");
            return -1;
        }
    }

    int gnss_coder_ambcon::encode_data(char *buff, int sz, int &cnt, std::vector<std::string> &errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                std::vector<std::shared_ptr<gnss_amb>> pamb;
                for (const auto &iter : _data)
                {
                    if (iter.second->id_type() == base_data::AMBCON)
                        pamb = dynamic_cast<gnss_data_ambcon *>(iter.second)->get_amb_all();
                }
                _ss << "+AMB/SOLUTION" << std::endl;
                _ss << "*INDEX_ AMB___ SAT1 SAT2 SITE1 SITE2 AMB_START_____ AMB_END_______ UNIT __ESTIMATED_VALUE____ _STD_DEV___ __UPD_VALUE___ FIXED__" << std::endl;
                int num = 1;
                for (const auto &amb : pamb)
                {
                    std::set<AMB_ID> amb_ids = amb->amb_ids();
                    for (const auto &id : _amb_ids)
                    {
                        if (amb_ids.find(id) == amb_ids.end())
                            continue;
                        _ss << " " << std::left << std::setw(6) << num << " " << std::setw(6) << gnss_amb::ambId2str(id) << " " << std::right << std::setw(3) << amb->sats().first << "  "
                            << std::setw(3) << amb->sats().second << "  " << std::setw(4) << amb->recs().first << "  " << std::setw(4) << amb->recs().second << "  "
                            << amb->t_beg().str_ydoysod() << " " << amb->t_end().str_ydoysod() << " cyc  "
                            << std::setw(21) << std::fixed << std::setprecision(3) << amb->value(id) << " " << std::setw(11) << std::fixed << std::setprecision(3) << amb->sigma(id) << " "
                            << std::setw(14) << std::fixed << std::setprecision(3) << amb->upd_cor(id) << " " << (amb->fixed(id) ? "YES" : "NO") << std::endl;
                    }
                    num++;
                }
                _ss << "-AMB/SOLUTION" << std::endl;
                _ss << "*-------------------------------------------------------------------------------" << std::endl;
                _ss << "%=ENDAMB";
            }

            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambcon::encode_data throw exception");
            return -1;
        }
    }
}