#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <memory>
#include "hwa_gnss_coder_biasinex.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_model_bias.h"
#include "hwa_gnss_all_bias.h"

using namespace std;

namespace hwa_gnss
{

    // constructor
    gnss_coder_biasinex::gnss_coder_biasinex(set_base *s, string version, int sz, string id)
        : base_coder(s, version, sz), gnss_coder_sinex(s, version, sz, id)
    {

        _allbias = 0;
    }

    /* -------- *
 * DECODER
 * -------- */

    int gnss_coder_biasinex::_decode_comm()
    {
        std::string::size_type idx;
        if ((idx = _line.find("PRN")) != string::npos)
            _mapidx["SAT"] = std::make_pair(idx, 3);
        if ((idx = _line.find("OBS1")) != string::npos)
            _mapidx["OBS1"] = std::make_pair(idx, 4);
        if ((idx = _line.find("OBS2")) != string::npos)
            _mapidx["OBS2"] = std::make_pair(idx, 4);

        if ((idx = _line.find("BIAS_START____")) != string::npos)
            _mapidx["BEG"] = std::make_pair(idx, 14);
        if ((idx = _line.find("BIAS_END______")) != string::npos)
            _mapidx["END"] = std::make_pair(idx, 14);

        if ((idx = _line.find("__ESTIMATED_VALUE____")) != string::npos)
            _mapidx["EST"] = std::make_pair(idx, 21);
        if ((idx = _line.find("_STD_DEV___")) != string::npos)
            _mapidx["STD"] = std::make_pair(idx, 11);

        return 1;
    }

    int gnss_coder_biasinex::_decode_block()
    {

        gnss_coder_sinex::_decode_block();

        // -------- "BIAS/DESCRIPTION" --------
        if (_block.find("BIAS/DESCRIPTION") != string::npos)
        {

            if (_line.find(" OBSERVATION_SAMPLING ") != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read BIAS/SMP: " + cut_crlf(_line.substr(31)));
            }
            else if (_line.find(" PARAMETER_SPACING ") != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read BIAS/SPC: " + cut_crlf(_line.substr(31)));
            }
            else if (_line.find(" DETERMINATION_METHOD ") != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read BIAS/MTD: " + cut_crlf(_line.substr(31)));
            }
            else if (_line.find(" BIAS_MODE ") != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read BIAS/MOD: " + cut_crlf(_line.substr(31)));
            }
            else if (_line.find(" TIME_SYSTEM ") != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read BIAS/TSY: " + cut_crlf(_line.substr(31)));
            }
        }
        else if (_block.find("BIAS/SOLUTION") != string::npos)
        {

            string prn = "";
            GOBS gobs1, gobs2;
            gobs1 = gobs2 = X;
            base_time beg = FIRST_TIME;
            base_time end = LAST_TIME;
            double dcb = 0.0;
            //  double std = 0.0;

            for (auto it = _mapidx.begin(); it != _mapidx.end(); it++)
            {
                size_t pos = it->second.first;
                size_t len = it->second.second;
                if (it->first == "SAT")
                    prn = _line.substr(pos, len);
                if (it->first == "OBS1")
                    gobs1 = str2gobs(_line.substr(pos, len));
                if (it->first == "OBS2")
                    gobs2 = str2gobs(_line.substr(pos, len));
                if (it->first == "BEG")
                    beg.from_str("%Y:%j:%s", _line.substr(pos, len));
                if (it->first == "END" && _line.substr(pos, len) != "0000:000:00000")
                    end.from_str("%Y:%j:%s", _line.substr(pos, len));
                if (it->first == "EST")
                    dcb = base_type_conv::str2dbl(_line.substr(pos, len));
                //      if(it->first == "STD") std = base_type_conv::str2dbl(_line.substr(pos, len));
            }

#ifdef DEBUG
            std::cout << "DCB decoding: " << _ac << " " << prn << " " << beg.str_ymdhms() << " " << end.str_ymdhms() << " " << gobs2str(gobs1) << " " << gobs2str(gobs2) << " " << dcb << endl;
            int ooo;
            cin >> ooo;
#endif

            shared_ptr<gnss_data_bias> p_bias;

            if (_allbias)
            {
                p_bias = make_shared<gnss_data_bias>(_spdlog);
                p_bias->set(beg, end, dcb * 1e-9 * CLIGHT, gobs1, gobs2);
                _allbias->add(_ac, beg, prn, p_bias);
            }
        }

        return 1;
    }

    void gnss_coder_biasinex::_add_data(string id, base_data *pt_data)
    {

        if (pt_data == 0)
            return;

        // ALL OBJECTS
        if (pt_data->id_type() == base_data::ALLBIAS)
        {
            if (!_allbias)
            {
                _allbias = dynamic_cast<gnss_all_bias *>(pt_data);
            }
        }

        return;
    }

} // namespace
