#include "hwa_set_inp.h"
#include "hwa_set_turboedit.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_file.h"
#include "hwa_base_log.h"
#include "hwa_gnss_coder_ambflag.h"
#include "hwa_gnss_data_turboedit.h"
#include "hwa_set_rec.h"
#include "hwa_set_gen.h"
#include <io.h>

using namespace std;
using namespace hwa_set;

namespace hwa_gnss
{
    hwa_gnss::gnss_data_turboedit::gnss_data_turboedit()
    {
    }

    //hwa_gnss::gnss_data_turboedit::gnss_data_turboedit(set_base *set, t_glog *log, bool isTriple) : gnss_data_cycleslip(set, log)
    hwa_gnss::gnss_data_turboedit::gnss_data_turboedit(set_base *gset, base_log spdlog, int index) : gnss_data_cycleslip(gset, spdlog)
    {
        if (nullptr == spdlog)
        {
            spdlog::critical("your spdlog is nullptr !");
            throw std::logic_error("");
        }
        else
        {
            _spdlog = spdlog;
        }
        _gset = gset;

        _index = index;
        // read log information from log file
        base_time beg_time = dynamic_cast<set_gen *>(gset)->beg();

        //recs to rec_all for processing both LEO and SITE in combined POD by zhangwei
        set<string> rec_list = dynamic_cast<set_gen *>(gset)->rec_all();

        // add lite mode
        bool realtime = dynamic_cast<set_gproc *>(gset)->realtime();
        bool lite_turboedit = dynamic_cast<set_turboedit *>(gset)->liteMode();
        if (realtime || _slip_model == SLIPMODEL::DEF_DETECT_MODEL)
            lite_turboedit = true;
        _apply_carrier_range = dynamic_cast<set_gproc *>(gset)->apply_carrier_range();

        if (!lite_turboedit)
        {
            _read_logfile(rec_list, beg_time, index);
        }
    }

    hwa_gnss::gnss_data_turboedit::~gnss_data_turboedit()
    {
    }

    set<string> hwa_gnss::gnss_data_turboedit::get_sitelist_of_logfile() const
    {
        set<string> site_list;
        for (auto iter = _amb_info_file_exist.begin(); iter != _amb_info_file_exist.end(); iter++)
        {
            if (iter->second)
            {
                site_list.insert(iter->first);
            }
        }
        return site_list;
    }

    void hwa_gnss::gnss_data_turboedit::merge_logfile_exist(const map<string, bool> &logfiles)
    {
        for (const auto &file : logfiles)
        {
            if (_amb_info_file_exist.count(file.first) == 0)
            {
                _amb_info_file_exist[file.first] = false;
            }
            else
            {
                _amb_info_file_exist[file.first] = (file.second && _amb_info_file_exist[file.first]);
            }
        }
    }

    hwa_map_all_ambflag gnss_data_turboedit::get_all_logfile()
    {
        return _gambflag->getAllAmbFlag();
    }

    //void hwa_gnss::gnss_data_turboedit::_read_logfie(const set<string> &rec, const base_time &epoch, bool isTriple)
    void hwa_gnss::gnss_data_turboedit::_read_logfie(const set<string> &rec, const base_time &epoch, int index)
    {
        // jdhuang
        // first, get all log file write in xml file
        multimap<IFMT, string> inp_xml = dynamic_cast<set_inp *>(_gset)->inputs_all();
        map<string, string> inp_log_xml;
        for (auto &inp : inp_xml)
        {
            if ((inp.first == IFMT::AMBFLAG12_INP && index == 2) ||
                (inp.first == IFMT::AMBFLAG13_INP && index == 3) ||
                (inp.first == IFMT::AMBFLAG14_INP && index == 4) ||
                (inp.first == IFMT::AMBFLAG15_INP && index == 5))
            {
                string _file_path = inp.second.substr(inp.second.find_first_of("/") + 2);
                string _file_name = file_name(inp.second);
                int pos = -1;
                if (index == 2)
                    pos = _file_name.find_last_not_of("log");
                else if (index == 3)
                    pos = _file_name.find_last_not_of("log13");
                else if (index == 4)
                    pos = _file_name.find_last_not_of("log14");
                else if (index == 5)
                    pos = _file_name.find_last_not_of("log15");
                else
                    continue;

                if (pos < 12)
                    continue;

                string rec_low = _file_name.substr(pos - 12, 4);
                inp_log_xml.insert(std::make_pair(rec_low, _file_path));
            }
        }

        std::fstream amb_info_file;
        for (auto rec_iter = rec.begin(); rec_iter != rec.end(); rec_iter++)
        {
            stringstream log_suffix;

            if (index == 2)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0."
                           << setw(2) << setfill('0') << epoch.yr() << "o.log";
            }
            else if (index == 3)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0."
                           << setw(2) << setfill('0') << epoch.yr() << "o.log13";
            }
            else if (index == 4)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0."
                           << setw(2) << setfill('0') << epoch.yr() << "o.log14";
            }
            else if (index == 5)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0."
                           << setw(2) << setfill('0') << epoch.yr() << "o.log15";
            }

            string rec_name_low;
            transform((*rec_iter).begin(), (*rec_iter).end(), back_inserter(rec_name_low), ::tolower);

            // jdhuang
            // first find the log file in xml input setting
            // if no log file in the path of xml file, find it in log_tb path
            string rec_amb_info_file_name = inp_log_xml[rec_name_low];
            if (ACCESS(rec_amb_info_file_name.c_str(), 0) != 0)
            {
                //std::cout << "can not find file : " + site_file_name << std::endl;
                rec_amb_info_file_name = "log_tb/" + rec_name_low + log_suffix.str();
            }

            amb_info_file.open(rec_amb_info_file_name, ios::in);
            if (!amb_info_file.is_open())
            {
                std::cout << rec_amb_info_file_name << ":can't open!" << std::endl;
                _amb_info_file_exist[*rec_iter] = false;
                continue;
            }

            if (_slip_model == SLIPMODEL::DEF_DETECT_MODEL)
            {
                _amb_info_file_exist[*rec_iter] = false;
                continue;
            }

            // read
            string line_txt;
            getline(amb_info_file, line_txt);
            line_txt = line_txt.substr(27);
            stringstream input(line_txt);
            double sod, intv;
            int temp, jd;
            input >> jd >> sod >> temp >> intv;

            // jdhuang,used to check two cycle log
            base_time beg_time(base_time::GPS);
            beg_time.from_mjd(jd, int(sod), sod - int(sod));
            line_txt.clear();
            while (!amb_info_file.eof())
            {
                getline(amb_info_file, line_txt);
                if (line_txt.size() <= 1)
                {
                    line_txt.clear();
                    continue;
                }
                if (line_txt[0] == '%' && line_txt[1] == 'M')
                {
                    stringstream oss(line_txt);
                    string str1, str2, str3, str4, str5, str6;
                    int max_amb;
                    oss >> str1 >> str2 >> str3 >> str4 >> str5 >> str6 >> max_amb;
                    _active_amb[*rec_iter] = max_amb;
                    line_txt.clear();
                    continue;
                }

                if (line_txt[0] < 'A' || line_txt[0] > 'Z')
                {
                    line_txt.clear();
                    continue;
                }
                stringstream os(line_txt);
                string identify;
                string sat_name;
                int beg_idx, end_idx, amb_flag;
                os >> identify >> sat_name >> beg_idx >> end_idx >> amb_flag;

                base_time beg, end;
                beg = beg_time + (beg_idx - 1) * intv;
                end = beg_time + (end_idx - 1) * intv;

                if (amb_flag == 1 || amb_flag == 2 || identify == "AMB")
                {
                    _cycle_flag[*rec_iter][sat_name].push_back(std::make_pair(beg, end));
                }
                else
                {
                    _cycle_flag_unused[*rec_iter][sat_name].push_back(std::make_pair(beg, end));
                }

                line_txt.clear();
            }

            amb_info_file.close();

            _amb_info_file_exist[*rec_iter] = true;
        }
    }

    void hwa_gnss::gnss_data_turboedit::_read_logfile(const set<string> &rec, const base_time &epoch, int index)
    {
        if (!_gambflag)
        {
            _gambflag = std::make_shared<gnss_all_ambflag>(_spdlog, base_data::AMBFLAG);
        }
        // first, get all log file write in xml file
        multimap<IFMT, string> inp_xml = dynamic_cast<set_inp *>(_gset)->inputs_all();
        map<string, string> inp_log_xml;
        for (auto &inp : inp_xml)
        {
            if ((inp.first == IFMT::AMBFLAG12_INP && index == 2) ||
                (inp.first == IFMT::AMBFLAG13_INP && index == 3) ||
                (inp.first == IFMT::AMBFLAG14_INP && index == 4) ||
                (inp.first == IFMT::AMBFLAG15_INP && index == 5))
            {
                string _file_path = inp.second.substr(inp.second.find_first_of("/") + 2);
                string _file_name = file_name(inp.second);
                int pos;
                if (index == 2)
                {
                    pos = _file_name.find_last_not_of("log");
                }
                else if (index == 3)
                {
                    pos = _file_name.find_last_not_of("log13");
                }
                else if (index == 4)
                {
                    pos = _file_name.find_last_not_of("log14");
                }
                else if (index == 5)
                {
                    pos = _file_name.find_last_not_of("log15");
                }
                else
                {
                    throw std::logic_error("The index should be : 2 3 4 5");
                }
                string rec_low = _file_name.substr(pos - 12, 4);
                inp_log_xml.insert(std::make_pair(rec_low, _file_path));
            }
        }
        base_data *gdata = nullptr;
        for (auto site_iter = rec.begin(); site_iter != rec.end(); ++site_iter)
        {
            stringstream log_suffix;
            if (index == 2)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0." << setw(2) << setfill('0') << epoch.yr() << "o.log";
            }
            else if (index == 3)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0." << setw(2) << setfill('0') << epoch.yr() << "o.log13";
            }
            else if (index == 4)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0." << setw(2) << setfill('0') << epoch.yr() << "o.log14";
            }
            else if (index == 5)
            {
                log_suffix << setw(3) << setfill('0') << epoch.doy() << "0." << setw(2) << setfill('0') << epoch.yr() << "o.log15";
            }

            string site_low;
            transform((*site_iter).begin(), (*site_iter).end(), back_inserter(site_low), ::tolower);
            // jdhuang
            // first find the log file in xml input setting
            // if no log file in the path of xml file, find it in log_tb path
            string site_file_name = inp_log_xml[site_low];
            if (ACCESS(site_file_name.c_str(), 0) != 0)
            {
                //std::cout << "can not find file : " + site_file_name << std::endl;
                site_file_name = "log_tb/" + site_low + log_suffix.str();
            }

            if (ACCESS(site_file_name.c_str(), 0) != 0)
            {
                _amb_info_file_exist[*site_iter] = false;
                continue;
            }

            if (_slip_model == SLIPMODEL::DEF_DETECT_MODEL)
            {
                _amb_info_file_exist[*site_iter] = false;
                continue;
            }

            gdata = _gambflag.get();
            base_coder *gcoder = new gnss_coder_ambflag(_gset, "", 4096);
            base_io *gio = new base_file(_spdlog);
            string path("file://" + site_file_name);
            gio->spdlog(_spdlog);
            gio->path(path);

            // Put the file into gcoder
            gcoder->clear();
            gcoder->path(path);
            gcoder->spdlog(_spdlog);

            // Put the data container into gcoder
            gcoder->add_data("ID0", gdata);
            gio->coder(gcoder);
            gio->run_read();

            // Delete
            delete gio;
            delete gcoder;

            _amb_info_file_exist[*site_iter] = true;
        }
    }

    bool hwa_gnss::gnss_data_turboedit::use_of_obs(const string &site, const string &prn, const base_time &time)
    {
        if (!_gambflag)
            return false;
        string lower_site(site);
        transform(site.begin(), site.end(), lower_site.begin(), ::tolower);
        if (_gambflag->getAllAmbFlag().find(lower_site) == _gambflag->getAllAmbFlag().end())
            return false;
        auto ambflag = _gambflag->getOneAmbFlag(lower_site);
        int pos = 0;
        return ambflag.isValid(prn, time, pos);
    }

    //bool hwa_gnss::gnss_data_turboedit::use_of_obs(const string& site, const string& prn, const base_time& time)
    //{
    //    if (!_amb_info_file_exist[site])
    //    {
    //        return false;
    //    }

    //    if (_cycle_flag.find(site) == _cycle_flag.end())
    //    {
    //        return false;
    //    }

    //    if (_cycle_flag[site].find(prn) == _cycle_flag[site].end())
    //    {
    //        return false;
    //    }

    //    // find in unused
    //    auto unused_end = _cycle_flag_unused[site][prn].end();
    //    for (auto time_iter = _cycle_flag_unused[site][prn].begin(); time_iter != unused_end; time_iter++)
    //    {
    //        if (time >= time_iter->first && time <= time_iter->second)
    //        {
    //            // UnUse
    //            return false;
    //        }
    //        else if (time < time_iter->first)
    //        {
    //            break;
    //        }
    //    }

    //    // Useful in amb arc
    //    return num_of_amb_arc(site, prn, time) > 0 ? true : false;
    //}

    //int hwa_gnss::gnss_data_turboedit::num_of_amb_arc(const string& site, const string& prn, const base_time& time)
    //{

    //    if (!_amb_info_file_exist[site])
    //    {
    //        return 0;
    //    }

    //    if (_cycle_flag.find(site) == _cycle_flag.end())
    //    {
    //        return 0;
    //    }

    //    if (_cycle_flag[site].find(prn) == _cycle_flag[site].end())
    //    {
    //        return 0;
    //    }

    //    // find in used
    //    int count = 0;
    //    auto used_end = _cycle_flag[site][prn].end();
    //    for (auto time_iter = _cycle_flag[site][prn].begin(); time_iter != used_end; time_iter++)
    //    {
    //        count++;
    //        if (time >= time_iter->first && time <= time_iter->second)
    //        {
    //            return count;
    //        }
    //        else if (time > time_iter->second)
    //        {
    //            continue;
    //        }
    //        else
    //        {
    //            return 0;
    //        }
    //    }
    //    return 0;
    //}

    int hwa_gnss::gnss_data_turboedit::num_of_amb_arc(const string &site, const string &prn, const base_time &time)
    {
        if (!_gambflag)
        {
            throw_logical_error(_spdlog, "check your xml file, you cannot caculate !");
        }
        string lower_site(site);
        transform(site.begin(), site.end(), lower_site.begin(), ::tolower);
        if (_gambflag->getAllAmbFlag().find(lower_site) == _gambflag->getAllAmbFlag().end())
            return -1;
        auto ambflag = _gambflag->getOneAmbFlag(lower_site);
        return ambflag.get_amb_pos(prn, time, _apply_carrier_range) + 1; // start from 1 (pos start from 0, maybe it should be changed later), jqwu
    }

    bool hwa_gnss::gnss_data_turboedit::cycle_slip(const gnss_data_sats &obsdata, const base_time &time)
    {
        // jdhuang
        // change some codes
        const string &sat = obsdata.sat();
        const string &rec = obsdata.site();

        if (_amb_info_file_exist[rec])
        {
            if (_amb_flag.find(rec) != _amb_flag.end() && _amb_flag[rec].find(sat) != _amb_flag[rec].end() && _amb_flag[rec][sat] == 0)
            {
                // todo
                throw runtime_error("Can't judge cycle slip unless already in one amb arc");
            }
            int flag_now = num_of_amb_arc(rec, sat, time);
            if (flag_now && flag_now != _amb_flag[rec][sat])
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        return false;
    }

    bool hwa_gnss::gnss_data_turboedit::cycle_slip123(gnss_data_sats &obsdata, const base_time &time)
    {
        // jdhuang
        // change some codes
        string sat = obsdata.sat();
        string rec = obsdata.site();

        if (_amb_info_file_exist[rec])
        {
            if (_amb_flag[rec][sat] == 0)
            {
                // todo
                //throw runtime_error("Can't judge cycle slip unless already in one amb arc");
            }
            int flag_now = num_of_amb_arc(rec, sat, time);
            if (flag_now && flag_now != _amb_flag[rec][sat])
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        return false;
    }

    int hwa_gnss::gnss_data_turboedit::get_active_amb(const string &site)
    {
        return _active_amb[site];
    }

    bool hwa_gnss::gnss_data_turboedit::is_carrier_range(const string &site, const string &prn, const base_time &time)
    {
        if (!_gambflag)
            return false;
        string lower_site(site);
        transform(site.begin(), site.end(), lower_site.begin(), ::tolower);
        if (_gambflag->getAllAmbFlag().find(lower_site) == _gambflag->getAllAmbFlag().end())
            return false;
        auto ambflag = _gambflag->getOneAmbFlag(lower_site);
        return ambflag.is_carrier_range(prn, time);
    }

    bool hwa_gnss::gnss_data_turboedit::is_carrier_range(const string &site, const string &prn, const base_time &time, double &c1, double &c2)
    {
        if (!_gambflag)
            return false;
        string lower_site(site);
        transform(site.begin(), site.end(), lower_site.begin(), ::tolower);
        if (_gambflag->getAllAmbFlag().find(lower_site) == _gambflag->getAllAmbFlag().end())
            return false;
        auto ambflag = _gambflag->getOneAmbFlag(lower_site);
        return ambflag.is_carrier_range(prn, time, c1, c2);
    }

    void hwa_gnss::gnss_data_turboedit::set_active_amb(const string &site, const int &active_num)
    {
        if (_slip_model != SLIPMODEL::TURBO_EDIT)
            _slip_model = SLIPMODEL::TURBO_EDIT;
        _amb_info_file_exist[site] = true;
        _active_amb[site] = active_num;
    }

    void hwa_gnss::gnss_data_turboedit::set_amb_flag(const string &rec, const string &sat, const int &flag)
    {
        _amb_flag[rec][sat] = flag;
        _new_amb[rec][sat] = true;
    }

    int hwa_gnss::gnss_data_turboedit::get_amb_flag(const string &rec, const string &sat)
    {
        return _amb_flag[rec][sat];
    }

    bool hwa_gnss::gnss_data_turboedit::new_amb(const string &rec, const string &sat)
    {
        if (_new_amb.find(rec) == _new_amb.end())
        {
            return false;
        }
        if (_new_amb[rec].find(sat) == _new_amb[rec].end())
        {
            return false;
        }
        return _new_amb[rec][sat];
    }

    void hwa_gnss::gnss_data_turboedit::set_new_amb(const string &rec, const string &sat, const bool &isNew)
    {
        _new_amb[rec][sat] = isNew;
    }

    base_time hwa_gnss::gnss_data_turboedit::get_crt_amb_end(const string &site, const string &sat)
    {
        if (_amb_flag[site][sat] <= 0)
        {
            string tmp = "sat : " + sat + " rec : " + site + " freq " + base_type_conv::int2str(_index);
            throw runtime_error("Crt amb isn't exist!!! " + tmp);
        }

        string lower_site(site);
        transform(site.begin(), site.end(), lower_site.begin(), ::tolower);
        auto ambflag = _gambflag->getOneAmbFlag(lower_site);
        auto one_ambflag = ambflag.getSatAmbFlag(sat).at(_amb_flag[site][sat] - 1);
        return ambflag.epoch2time(one_ambflag->end_epo);
    }

    //base_time hwa_gnss::gnss_data_turboedit::get_crt_amb_end(const string& rec, const string& sat)
    //{
    //    if (_amb_flag[rec][sat] <= 0)
    //    {
    //        throw runtime_error("Crt amb isn't exist!!!");
    //    }
    //    auto iter = _cycle_flag[rec][sat].begin();
    //    advance(iter, _amb_flag[rec][sat] - 1);
    //    return iter->second;
    //}

    void hwa_gnss::gnss_data_turboedit::add_ambflag(const string &site, const string &sat, const string &description, const base_time &beg, const base_time &end)
    {
        if (description == "AMB")
        {
            _cycle_flag[site][sat].push_back(std::make_pair(beg, end));
        }
        else
        {
            _cycle_flag_unused[site][sat].push_back(std::make_pair(beg, end));
        }
    }
}
