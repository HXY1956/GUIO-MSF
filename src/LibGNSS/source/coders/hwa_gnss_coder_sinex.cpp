#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <memory>
#include <algorithm>

#include "hwa_gnss_coder_sinex.h"
#include "hwa_gnss_sys.h"
#include "hwa_base_typeconv.h"
#include "hwa_gnss_prod_crd.h"
#include "hwa_gnss_data_pcv.h"
#include "hwa_base_data.h"
#include "hwa_base_globaltrans.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_sinex::gnss_coder_sinex(set_base *s, string version, int sz, string id)
        : base_coder(s, version, sz, id), gnss_base_coder(s, version, sz, id),
          _snx_type(SINEX_GNS),
          _technique('P'),
          _parindex(1), // should start from 1 (SINEX format)
          _tmpsize(0),
          _consume(0),
          _complete(true),
          _estimation(false),
          _code_label("CODE"), // LABEL FOR SITE
          _list_gnss(""),
          _line(""),
          _site(""),
          _block(""),
          _pco_mod("igsXX_XXXX"),
          _ac(""),
          _pt_prod(0),
          _file_beg(base_time::GPS),
          _file_end(base_time::GPS),
          _file_run(base_time::GPS),
          _allobj(0),
          _end_epoch(false),
          _end_estim(false)
    {
    }

    /* -------- *
 * DECODER
 * -------- */

    // SINEX head
    // ----------
    int gnss_coder_sinex::decode_head(char *buff, int sz, vector<string> &errmsg)
    {

        _mutex.lock();

        gnss_base_coder::_add2buffer(buff, sz);
        while ((_tmpsize = gnss_base_coder::_getline(_line)) >= 0)
        {
            if (_tmpsize < 65)
            {
                _mutex.unlock();
                return 0;
            }

#ifdef DEBUG
            std::cout << "REQ: _decode_head() :" << _buffsz << " :" << _buffer << ":\n"; // std::cout << "LINE:" << _line;
#endif

            // 1st line
            // -------- "FILE HEADER" --------
            if (_line.find("%=TRO") != string::npos || _line.find("%=BIA") != string::npos || _line.find("%=SNX") != string::npos)
            {
                _version = base_type_conv::trim(_line.substr(5, 5));
                base_type_conv::substitute(_version, " ", "");
                _ac = base_type_conv::trim(_line.substr(11, 3));

                if (_snx_type == SINEX_GNS || _line[17] == ':')
                { // SINEX style (YY instead of YYYY)
                    string file_run = base_type_conv::trim(_line.substr(15, 12));
                    base_type_conv::substitute(file_run, " ", "");
                    _file_run.from_str("%y:%j:%s", file_run);

                    string file_beg = base_type_conv::trim(_line.substr(32, 12));
                    base_type_conv::substitute(file_beg, " ", "");
                    _file_beg.from_str("%y:%j:%s", file_beg);
                    if (file_beg.compare("00:000:00000") == 0)
                        _file_beg = FIRST_TIME;

                    string file_end = base_type_conv::trim(_line.substr(45, 12));
                    base_type_conv::substitute(file_end, " ", "");
                    _file_end.from_str("%y:%j:%s", file_end);
                    if (file_end.compare("00:000:00000") == 0)
                        _file_end = LAST_TIME;

                    string num_est = base_type_conv::trim(_line.substr(60, 5));
                    base_type_conv::substitute(num_est, " ", "");
                    if (base_type_conv::str2int(num_est) > 0)
                        _estimation = true;
                }
                else
                {
                    string file_run = base_type_conv::trim(_line.substr(15, 14));
                    base_type_conv::substitute(file_run, " ", "");
                    _file_run.from_str("%Y:%j:%s", file_run);

                    string file_beg = base_type_conv::trim(_line.substr(34, 14));
                    base_type_conv::substitute(file_beg, " ", "");
                    _file_beg.from_str("%Y:%j:%s", file_beg);
                    if (file_beg.compare("00:000:00000") == 0)
                        _file_beg = FIRST_TIME;

                    string file_end = base_type_conv::trim(_line.substr(49, 14));
                    base_type_conv::substitute(file_end, " ", "");
                    _file_end.from_str("%Y:%j:%s", file_end);
                    if (file_end.compare("00:000:00000") == 0)
                        _file_end = LAST_TIME;
                }

#ifdef DEBUG
                std::cout << "_sinex_decode: " << _file_run.str_ymdhms(" run: ") << _file_beg.str_ymdhms(" beg: ") << _file_end.str_ymdhms(" end: ") << endl;
#endif

                if (_decode_vers() == 0)
                {
                    gnss_base_coder::_consume(_tmpsize);
                    _mutex.unlock();
                    return -1;
                }
            }
        }
        _mutex.unlock();
        return 0;
    }

    // SINEX data
    // ----------
    int gnss_coder_sinex::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {

        _mutex.lock();

        gnss_base_coder::_add2buffer(buff, sz);

        _complete = true;
        _consume = 0;
        _tmpsize = 0;
        _line = "";

        _decode_data();

        _mutex.unlock();
        return _consume;
    }

    /* -------- *
 * ENCODER
 * -------- */

    /* ----------
 * SINEX head
 */
    int gnss_coder_sinex::encode_head(char *buff, int sz, vector<string> &errmsg)
    {

        _mutex.lock();
        _parindex = 1;

        if (_ss_position == 0)
        {
            if (_initialize_data() < 1)
            {
                _mutex.unlock();
                return -1;
            }

            _file_run = base_time::current_time(base_time::GPS);
            _file_beg = LAST_TIME;
            _file_end = FIRST_TIME;

            std::ostringstream osXYZ, osIDE, osSOL, osECC, osANT, osREC, osPCO;
            for (itSET = _sites.begin(); itSET != _sites.end(); ++itSET)
            {
                string site = *itSET; // prefill site-specific metadata
                gnss_STC_META meta;
                if (_fill_site_INI(site, meta) > 0)
                { // skip if grec not found etc.
                    _fill_site_IDE(site, meta, osIDE);
                    _fill_site_SOL(site, meta, osSOL);
                    _fill_site_XYZ(site, meta, osXYZ);
                    _fill_site_ECC(site, meta, osECC);
                    _fill_site_ANT(site, meta, osANT);
                    _fill_site_REC(site, meta, osREC);
                    _fill_site_PCO(site, meta, osPCO);
                }
            }
            // complete metadata parts
            _fill_head_INI();
            _fill_head_IDE(osIDE);
            _fill_head_SOL(osSOL);
            _fill_head_XYZ(osXYZ);
            _fill_head_ECC(osECC);
            _fill_head_ANT(osANT);
            _fill_head_REC(osREC);
            _fill_head_PCO(osPCO);
        }

        int size = _fill_buffer(buff, sz);

        _mutex.unlock();
        return size;
    }

    /* ----------
 * SINEX data
 */
    int gnss_coder_sinex::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {

        _mutex.lock();
        _parindex = 1;

        if (_ss_position == 0)
        {
            if (_initialize_data() < 1)
            {
                _mutex.unlock();
                return -1;
            }

            std::ostringstream osEST, osAPR, osCOV;
            for (itSET = _sites.begin(); itSET != _sites.end(); ++itSET)
            {
                string site = *itSET; // prefill site-specific metadata

                gnss_STC_META meta;
                if (_fill_site_INI(site, meta) > 0)
                { // skip if grec not found etc.
                    _fill_site_APR(site, meta, osAPR);
                    _fill_site_EST(site, meta, osEST);
                    _fill_site_COV(site, meta, osCOV);
                }
            }
            // complete metadata parts
            _fill_data_STT();
            _fill_data_EST(osEST);
            _fill_data_APR(osAPR);
            _fill_data_COV(osCOV);

            if (_snx_type == SINEX_GNS)
                _ss << "%ENDSNX";
            else
                _ss << "%ENDTRO";
        }

        int size = _fill_buffer(buff, sz);

        _mutex.unlock();
        return size;
    }

    /* ----------
 * TECHNIQUE
 */
    void gnss_coder_sinex::technique(char c)
    {

        switch (c)
        {
        case 'c':
        case 'C':
            _technique = 'C';
            break; // COMBINED
        case 'd':
        case 'D':
            _technique = 'D';
            break; // DORIS
        case 'p':
        case 'P':
            _technique = 'P';
            break; // GNSS
        case 'r':
        case 'R':
            _technique = 'R';
            break; // VLBI
        case 'w':
        case 'W':
            _technique = 'W';
            break; // RADIOMETER
        case 's':
        case 'S':
            _technique = 'S';
            break; // RADIOSONDE
        case 'f':
        case 'F':
            _technique = 'F';
            break; // NWM FORECAST
        case 'm':
        case 'M':
            _technique = 'M';
            break; // NWM RE-ANALYSIS
        case 'n':
        case 'N':
            _technique = 'N';
            break; // CLIMATE MODEL
        default:
            _technique = 'P';
        }

        switch (_technique)
        {
        case 'C':
        case 'P':
        case 'D':
        case 'R':
            _snx_type = TROSNX_GNS;
            break;
        case 'F':
        case 'M':
        case 'N':
            _snx_type = TROSNX_NWM;
            break;
        case 'W':
        case 'S':
            _snx_type = TROSNX_OTH;
            break;
        default:
            _snx_type = TROSNX_OTH;
        }
    }

    /* ----------
 * TECHNIQUE
 */
    char gnss_coder_sinex::technique()
    {
        return _technique;
    }

    /* ------------------------ *
 * INTERNAL FUNCTIONS
 * ------------------------ */

    // check site decode head
    // ----------
    shared_ptr<gnss_data_rec> gnss_coder_sinex::_get_rec(string site)
    {

        if (_epo_pos.find(site) == _epo_pos.end())
        {

            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Warning: object not found for site: " + site + " [should not happen]");
            return 0;
        }

        if (_epo_pos[site].size() == 0)
        {

            if (_spdlog)
                SPDLOG_LOGGER_INFO(_spdlog, "Position not found for site: " + site);
            return 0;
        }

        return std::dynamic_pointer_cast<gnss_data_rec>(_mapobj[site]);
    }

    // decode head
    // ----------
    int gnss_coder_sinex::_decode_vers()
    {

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "Read VERS: " + _version);

        return 0;
    }

    // decode data
    // ----------
    int gnss_coder_sinex::_decode_data()
    {

        while (_complete && (_tmpsize = gnss_base_coder::_getline(_line)) >= 0)
        {

#ifdef DEBUG
            std::cout << "DATA: " << _tmpsize << " " << _consume << " :" << _line;
            std::cout.flush();
#endif

            _complete = true;

            // -------- "COMMENT" --------
            if (_line.find("*", 0) != string::npos)
            {

                _comment.push_back(_line); // _line.substr(0,60));

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read COMM: " + _line.substr(1, 25)); // DEFAULT: 3

                this->_decode_comm();
            }

            // -------- "BLOCK START" --------
            else if (_line[0] == '+' && _line[1] != ' ')
            { // sometimes in other block
                _block = cut_crlf(base_type_conv::trim(_line.substr(1)));
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "BLOCK beg: " + _block.substr(0, 13)); // DEFAULT: 1
            }

            // -------- "BLOCK END" --------
            else if (_line[0] == '-' && _line.find(_block) != string::npos)
            { // sometimes in other block
                string block_tmp = cut_crlf(base_type_conv::trim(_line.substr(1)));
                if (_block != block_tmp)
                {
                    cerr << "uncorectly closed SINEX BLOCK: " + _block << endl;
                    gnss_base_coder::_consume(_tmpsize);
                    return -1;
                }

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "BLOCK end: " + _block.substr(0, 13)); // DEFAULT: 1

                if (_block.compare("SOLUTION/EPOCHS") == 0)
                    _end_epoch = true;
                if (_block.compare("SOLUTION/ESTIMATE") == 0)
                    _end_estim = true;
                _block = "";
            }

            // -------- "BLOCK READING" --------
            else
                this->_decode_block();

            // -------- CONSUME --------
            if (_complete)
                _consume += gnss_base_coder::_consume(_tmpsize);
            else
                break;
        }

        // Set object coordinates
        if (_end_epoch && _end_estim)
        {
            _set_rec_crd();
        }

        return _consume;
    }

    // decode comm
    // ----------
    int gnss_coder_sinex::_decode_comm()
    {

        int idx = 0;
        if ((idx = _line.find("SITE")) != string::npos)
            _mapidx["SIT"] = std::make_pair(idx, 4);
        if ((idx = _line.find("STATION__")) != string::npos)
            _mapidx["SIT"] = std::make_pair(idx, 9); // NEW TRO-SINEX2

        if ((idx = _line.find("__STA_X_____")) != string::npos)
            _mapidx["X"] = std::make_pair(idx, 12);
        if ((idx = _line.find("__STA_Y_____")) != string::npos)
            _mapidx["Y"] = std::make_pair(idx, 12);
        if ((idx = _line.find("__STA_Z_____")) != string::npos)
            _mapidx["Z"] = std::make_pair(idx, 12);

        return 0;
    }

    // decode block
    // ----------
    int gnss_coder_sinex::_decode_block()
    {

        // -------- "FILE/REFERENCE" --------
        if (_block.find("FILE/REFERENCE") != string::npos)
        {

            if (_line.find(" DESC", 4) != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read FILE/DESC: " + _line.substr(20, 60));
            }
            else if (_line.find(" OUTP", 4) != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read FILE/OUTP: " + _line.substr(20, 60));
            }
            else if (_line.find(" CONT", 4) != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read FILE/CONT: " + _line.substr(20, 60));
            }
            else if (_line.find(" SOFT", 4) != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read FILE/SOFT: " + _line.substr(20, 60));
            }
            else if (_line.find(" HARD", 4) != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read FILE/HARD: " + _line.substr(20, 60));
            }
            else if (_line.find(" INPU", 4) != string::npos)
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Read FILE/INPT: " + _line.substr(20, 60));
            }
        }
        else if (_block.find("SITE/RECEIVER") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Read SITE/RECEIVER");

            if (_snx_type == SINEX_GNS)
            {
                string site = base_type_conv::trim(_line.substr(1, 4));
                transform(site.begin(), site.end(), site.begin(), ::toupper);
                string begfmt = base_type_conv::trim(_line.substr(16, 12));
                string endfmt = base_type_conv::trim(_line.substr(29, 12));
                string rcv = base_type_conv::trim(_line.substr(42, 20));
                base_time beg(base_time::GPS);
                base_time end(base_time::GPS);

                if (begfmt.compare("00:000:00000") == 0)
                    beg = LAST_TIME;
                else
                    beg.from_str("%y:%j:%s", begfmt);

                if (endfmt.compare("00:000:00000") == 0)
                    end = LAST_TIME;
                else
                    end.from_str("%y:%j:%s", endfmt);

                shared_ptr<gnss_data_rec> new_obj = make_shared<gnss_data_rec>(_spdlog);
                new_obj->id(site);
                new_obj->spdlog(_spdlog);
                new_obj->rec(rcv, beg, end);

                _complete_obj(new_obj, beg);
            }
        }
        else if (_block.find("SITE/ANTENNA") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Read SITE/ANTENNA");

            if (_snx_type == SINEX_GNS)
            {
                string site = base_type_conv::trim(_line.substr(1, 4));
                transform(site.begin(), site.end(), site.begin(), ::toupper);
                string begfmt = base_type_conv::trim(_line.substr(16, 12));
                string endfmt = base_type_conv::trim(_line.substr(29, 12));
                string ant = base_type_conv::trim(_line.substr(42, 20));
                base_time beg(base_time::GPS);
                base_time end(base_time::GPS);

                if (begfmt.compare("00:000:00000") == 0)
                    beg = LAST_TIME;
                else
                    beg.from_str("%y:%j:%s", begfmt);

                if (endfmt.compare("00:000:00000") == 0)
                    end = LAST_TIME;
                else
                    end.from_str("%y:%j:%s", endfmt);

                shared_ptr<gnss_data_rec> new_obj = make_shared<gnss_data_rec>(_spdlog);
                new_obj->id(site);
                new_obj->spdlog(_spdlog);
                new_obj->ant(ant, beg, end);
                _complete_obj(new_obj, beg);
            }
        }
        else if (_block.find("SITE/ECCENTRICITY") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Read SITE/ECCENTRICITY");

            if (_snx_type == SINEX_GNS)
            {
                string site = base_type_conv::trim(_line.substr(1, 4));
                transform(site.begin(), site.end(), site.begin(), ::toupper);
                string begfmt = base_type_conv::trim(_line.substr(16, 12));
                string endfmt = base_type_conv::trim(_line.substr(29, 12));
                base_time beg(base_time::GPS);
                base_time end(base_time::GPS);

                if (begfmt.compare("00:000:00000") == 0)
                    beg = LAST_TIME;
                else
                    beg.from_str("%y:%j:%s", begfmt);

                if (endfmt.compare("00:000:00000") == 0)
                    end = LAST_TIME;
                else
                    end.from_str("%y:%j:%s", endfmt);

                string eccsys = base_type_conv::trim(_line.substr(42, 3));
                Triple ecc;
                double a = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(46, 8)));
                double b = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(55, 8)));
                double c = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(64, 8)));

                shared_ptr<gnss_data_rec> new_obj = make_shared<gnss_data_rec>(_spdlog);
                new_obj->id(site);
                new_obj->spdlog(_spdlog);
                if (eccsys.compare("UNE") == 0)
                {
                    ecc[0] = b;
                    ecc[1] = c;
                    ecc[2] = a;
                    new_obj->eccneu(ecc, beg, end);
                }
                else if (eccsys.compare("XYZ") == 0)
                {
                    ecc[0] = a;
                    ecc[1] = b;
                    ecc[2] = c;
                    new_obj->eccxyz(ecc, beg, end);
                }

                _complete_obj(new_obj, beg);
            }
        }
        else if (_block.find("SITE/ID") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Read SITE/ID");

            if (_snx_type == SINEX_GNS)
            {
                string site = base_type_conv::trim(_line.substr(1, 4));
                transform(site.begin(), site.end(), site.begin(), ::toupper);
                string domes = base_type_conv::trim(_line.substr(9, 9));
                string descr = base_type_conv::trim(_line.substr(21, 22));

                int lon_d = base_type_conv::str2int(base_type_conv::trim(_line.substr(44, 3)));
                int lon_m = base_type_conv::str2int(base_type_conv::trim(_line.substr(48, 2)));
                double lon_s = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(51, 4)));
                double lon = double(lon_d) + double(lon_m) / 60 + lon_s / 3600;

                int lat_d = base_type_conv::str2int(base_type_conv::trim(_line.substr(56, 3)));
                int lat_m = base_type_conv::str2int(base_type_conv::trim(_line.substr(60, 2)));
                double lat_s = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(63, 4)));
                double lat = double(lat_d) + double(lat_m) / 60 + lat_s / 3600;

                double height = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(68, 7)));

                Triple ell(lat, lon, height);
                Triple xyz;
                ell2xyz(ell, xyz, true);

                if (xyz.norm() > 0 && !_estimation)
                {
                    shared_ptr<gnss_data_rec> new_obj = make_shared<gnss_data_rec>(_spdlog);
                    new_obj->id(site);
                    new_obj->spdlog(_spdlog);
                    Triple std(10, 10, 10);
                    new_obj->crd(xyz, std, _file_beg, _file_end);

                    _complete_obj(new_obj, _file_beg);
                }
            }
        }
        else if (_block.find("SOLUTION/EPOCHS") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Read SOLUTION/EPOCHS");

            string site = base_type_conv::trim(_line.substr(1, 4));
            transform(site.begin(), site.end(), site.begin(), ::toupper);

            string begfmt = base_type_conv::trim(_line.substr(16, 12));
            string endfmt = base_type_conv::trim(_line.substr(29, 12));
            base_time beg(base_time::GPS);
            base_time end(base_time::GPS);

            if (begfmt.compare("00:000:00000") == 0)
                beg = LAST_TIME;
            else
                beg.from_str("%y:%j:%s", begfmt);

            if (endfmt.compare("00:000:00000") == 0)
                end = LAST_TIME;
            else
                end.from_str("%y:%j:%s", endfmt);

            _sol_epoch[site].insert(beg);
            _sol_epoch[site].insert(end);
        }
        else if (_block.find("SOLUTION/ESTIMATE") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Read SOLUTION/ESTIMATE");

            string site = base_type_conv::trim(_line.substr(14, 4));
            transform(site.begin(), site.end(), site.begin(), ::toupper);

            string param = base_type_conv::trim(_line.substr(7, 6));
            transform(param.begin(), param.end(), param.begin(), ::toupper);

            string epofmt = base_type_conv::trim(_line.substr(27, 12));
            base_time epo(base_time::GPS);
            epo.from_str("%y:%j:%s", epofmt);

            double est = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(47, 21)));
            double std = base_type_conv::str2dbl(base_type_conv::trim(_line.substr(69, 11)));
            base_pair val(est, std);

            _sol_estim[site][epo][param] = val;
        }

        return 0;
    }

    /* ------------------------ *
 * INTERNAL FUNCTIONS
 * ------------------------ */
    void gnss_coder_sinex::_add_data(string id, base_data *pt_data)
    {

        if (pt_data == 0)
            return;

        // ORDER IS IMPORTANT!
        if (pt_data->id_type() == base_data::ALLPROD &&
            pt_data->id_group() == base_data::GRP_prodUCT)
        {
            // ALL PRODUCTS (MET) special case (input meteo parameters -> encode only)
            // ----> currently the product needs to be labeled "MET" to be recognized
            // ----> THIS SHOULD BE IN FUTURE CONSOLIDATED using gprodmet instead of gprod
            if (id != "MET")
            {
                _pt_prod = dynamic_cast<gnss_all_prod *>(pt_data);
            }
        }

        // ALL OBJECTS
        if (pt_data->id_type() == base_data::ALLOBJ)
        {
            if (!_allobj)
            {
                _allobj = dynamic_cast<gnss_all_obj *>(pt_data);
            }
        }

        return;
    }

    // beg time
    // ---------
    base_time gnss_coder_sinex::_site_beg(string site)
    {
        return *_epo_pos[site].begin();
    }

    // end time
    // ---------
    base_time gnss_coder_sinex::_site_end(string site)
    {
        return *_epo_pos[site].rbegin();
    }

    // prepare data structures
    // ---------
    int gnss_coder_sinex::_initialize_data()
    {

        if (!_pt_prod || !_allobj)
            return -1;

        // collect all available objects
        _mapobj = _allobj->objects(base_data::REC);

        // collect all sites
        _sites = _pt_prod->prod_sites();
        if (_sites.size() == 0)
        {
            return -1;
        }

        _initialized = true;

        for (itSET = _sites.begin(); itSET != _sites.end();)
        {
            string site = *itSET;

            // filter out sites
            if (_rec.size() == 0 || _rec.find(site) == _rec.end())
            {
                _sites.erase(itSET++);
                continue;
            }

            // no object found!
            if (_mapobj.find(site) == _mapobj.end())
            {

                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "Warning: OBJ not found for site " + site + " - data skipped!");
                _sites.erase(itSET++);
                continue;
            }
            _epo_pos[site] = _pt_prod->prod_epochs(site, base_data::POS);

            ++itSET; // at the end to avoid site erase
        }

        if (_epo_pos.size() == 0)
        {
            _initialized = false;
            return -1;
        }

        return 1;
    }

    // prefill metadata
    // ---------
    int gnss_coder_sinex::_fill_site_INI(string site, gnss_STC_META &meta)
    {

        shared_ptr<gnss_data_rec> grec = _get_rec(site);

        if (grec == 0)
        {
            return -1;
        }

        // use last coordinates!
        meta.begPOS = this->_site_beg(site);
        meta.endPOS = this->_site_end(site);

        // OBJECT coordinates (initial)
        meta.xyz = grec->crd(meta.endPOS);
        meta.ecc = grec->eccneu(meta.endPOS);
        meta.domes = grec->domes();
        meta.name = grec->name();
        meta.desc = grec->desc();
        meta.ant = grec->ant(meta.begPOS);
        meta.rec = grec->rec(meta.begPOS);
        meta.id = _mapobj[site]->name();

        shared_ptr<gnss_data_pcv> gpcv = grec->pcv(meta.begPOS);

        if (gpcv)
        {
            meta.gps_neu_L1 = gpcv->pco(G01) / 1000.0;
            meta.gps_neu_L2 = gpcv->pco(G02) / 1000.0;
            meta.snx_code = gpcv->snxcod();
        }

        // 4-chars (original name)
        if (meta.name[4] == ' ' && meta.name[5] != ' ')
            meta.name = meta.name.substr(0, 4);

        if (meta.endPOS > _file_end)
            _file_end = meta.endPOS; // std::cout << " FIRST POS: " << meta.begPOS->str_ymdhms() << endl; std::cout.flush();
        if (meta.begPOS < _file_beg)
            _file_beg = meta.begPOS; // std::cout << "  LAST POS: " << meta.endPOS->str_ymdhms() << endl; std::cout.flush();

        // cut BEG/END TIME if out_epo/out_len requested
        if (_out_len > 0 && _out_epo != LAST_TIME)
        {
            if (_out_epo - _out_len * 60 > _file_beg)
                _file_beg = _out_epo - _out_len * 60;
            if (_out_epo < _file_end)
                _file_end = _out_epo;
        }

#ifdef DEBUG
        std::cout << "_sinex_encode: " << _file_run.str_ymdhms(" run: ") << _file_beg.str_ymdhms(" beg: ") << _file_end.str_ymdhms(" end: ") << endl;
#endif

        // only for GNSS product, alternatively, XYZ/ELL is from apriori (RAO/NWM/..)
        shared_ptr<gnss_prod> tmp = _pt_prod->get(site, base_data::POS, meta.endPOS);

        if (tmp)
        {
            // LAST AVAILABLE IN PRODUCT! (replace OBJECT coordinates !)
            meta.xyz = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->xyz();
            meta.rms = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->xyz_rms();
            meta.var = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->xyz_var();
            meta.apr = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->apr();
            meta.std = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->apr_rms();

            meta.cov[0] = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->cov(COV_XY); // 1: COV_XY
            meta.cov[1] = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->cov(COV_XZ); // 1: COV_XZ
            meta.cov[2] = std::dynamic_pointer_cast<gnss_prod_crd>(tmp)->cov(COV_YZ); // 1: COV_YZ
        }

        xyz2ell(meta.xyz, meta.ell, true);

        meta.par.push_back("STAX");
        meta.idx[0] = _parindex++; // STAX parameter index
        meta.par.push_back("STAY");
        meta.idx[1] = _parindex++; // STAY parameter index
        meta.par.push_back("STAZ");
        meta.idx[2] = _parindex++; // STAZ parameter index

        return 1;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_IDE(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        //  "*STATION__ PT __DOMES__ T _STATION DESCRIPTION__ APPROX_LON_ APPROX_LAT_ _APP_H_"
        //  " ACOR.....  A 13434M001 P A Coruna, ES           351 36  3.9  43 21 51.8    66.9"

        int degB = int(floor(meta.ell[0]));
        int minB = int(floor((meta.ell[0] - degB) * 60));
        int degL = int(floor(meta.ell[1]));
        int minL = int(floor((meta.ell[1] - degL) * 60));

        // EXTENSION TO THE CURRENT TRO-SINEX !!!! /JD 2017-01-15
        double pres, temp, undu = 0.0;
        _ggpt.gpt_v1(51544.0, meta.ell[0] * D2R, meta.ell[1] * D2R, meta.ell[2], pres, temp, undu); // RADION

        os << std::fixed << setprecision(1)
           << left
           << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
           << right
           << " " << setw(2) << "A"
           << " " << setw(9) << meta.domes.substr(0, 9)
           << " " << setw(1) << _technique
           << left
           << " " << setw(22) << meta.desc.substr(0, 22)
           << right;

        if (_snx_type == SINEX_GNS) // STANDARD SINEX
            os << " " << setw(3) << degL
               << " " << setw(2) << minL
               << " " << setw(4) << meta.ell[1] * 3600 - degL * 3600 - minL * 60
               << " " << setw(3) << degB
               << " " << setw(2) << minB
               << " " << setw(4) << meta.ell[0] * 3600 - degB * 3600 - minB * 60
               << setprecision(1) << " " << setw(7 + 0) << meta.ell[2];

        else // TROPO SINEX
            os << setprecision(6)
               << " " << setw(10) << meta.ell[1]
               << " " << setw(10) << meta.ell[0]
               << setprecision(3)
               << " " << setw(9) << meta.ell[2] + meta.ecc[2]
               << " " << setw(9) << meta.ell[2] + meta.ecc[2] + undu;

        os << left << endl;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_SOL(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        // "*SITE PT SOLN T _data_START_ __data_END__ _MEAN_EPOCH_
        // " ACOR  A 0001 P 14:363:00000 14:363:86370 14:363:43200

        base_time mid(meta.begPOS + (meta.endPOS - meta.begPOS) / 2);

        os << left
           << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
           << right
           << " " << setw(2) << "A"
           << " " << setw(4) << "1"
           << " " << setw(1) << _technique
           << std::fixed << setfill('0');

        if (_snx_type == SINEX_GNS)
            os << " " << setw(2) << meta.begPOS.yr()
               << ":" << setw(3) << meta.begPOS.doy()
               << ":" << setw(5) << meta.begPOS.sod()
               << " " << setw(2) << meta.endPOS.yr()
               << ":" << setw(3) << meta.endPOS.doy()
               << ":" << setw(5) << meta.endPOS.sod()
               << " " << setw(2) << mid.yr()
               << ":" << setw(3) << mid.doy()
               << ":" << setw(5) << mid.sod() << endl;
        else
            os << " " << setw(2) << meta.begPOS.year()
               << ":" << setw(3) << meta.begPOS.doy()
               << ":" << setw(5) << meta.begPOS.sod()
               << " " << setw(2) << meta.endPOS.year()
               << ":" << setw(3) << meta.endPOS.doy()
               << ":" << setw(5) << meta.endPOS.sod()
               << " " << setw(2) << mid.year()
               << ":" << setw(3) << mid.doy()
               << ":" << setw(5) << mid.sod() << endl;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_XYZ(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        // "*STATION__ PT SOLN T __STA_X_____ __STA_Y_____ __STA_Z_____ SYSTEM REMRK"
        // " ACOR.....  A    1 P  4594489.604  -678367.569  4357066.217 IGS08  GOP"

        os << left
           << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
           << right
           << " " << setw(2) << "A"
           << " " << setw(4) << "1"
           << " " << setw(1) << _technique
           << std::fixed << setfill('0');

        if (_snx_type == SINEX_GNS)
            os << " " << setw(2) << meta.begPOS.yr()
               << ":" << setw(3) << meta.begPOS.doy()
               << ":" << setw(5) << meta.begPOS.sod()
               << " " << setw(2) << meta.endPOS.yr()
               << ":" << setw(3) << meta.endPOS.doy()
               << ":" << setw(5) << meta.endPOS.sod();
        else
            os << " " << setw(2) << meta.begPOS.year()
               << ":" << setw(3) << meta.begPOS.doy()
               << ":" << setw(5) << meta.begPOS.sod()
               << " " << setw(2) << meta.endPOS.year()
               << ":" << setw(3) << meta.endPOS.doy()
               << ":" << setw(5) << meta.endPOS.sod();

        os << setfill(' ') << std::fixed << setprecision(3)
           << " " << setw(12) << meta.xyz[0]
           << " " << setw(12) << meta.xyz[1]
           << " " << setw(12) << meta.xyz[2]
           << " " << setw(6) << "IGS08"
           << " " << setw(5) << "GOP"
           << endl;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_ECC(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        // "*STATION__ PT SOLN T DATA_START__ DATA_END____ AXE ARP->BENCHMARK(M)_________"
        // " ACOR.....  A    1 P 13:345:00000 13:345:86370 UNE   3.0460   0.0000   0.0000"

        os << left
           << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
           << right
           << " " << setw(2) << "A"
           << " " << setw(4) << "1"
           << " " << setw(1) << _technique
           << std::fixed << setfill('0');

        if (_snx_type == SINEX_GNS)
            os << " " << setw(2) << _file_beg.yr()
               << ":" << setw(3) << _file_beg.doy()
               << ":" << setw(5) << _file_beg.sod()
               << " " << setw(2) << _file_end.yr()
               << ":" << setw(3) << _file_end.doy()
               << ":" << setw(5) << _file_end.sod();
        else
            os << " " << setw(2) << _file_beg.year()
               << ":" << setw(3) << _file_beg.doy()
               << ":" << setw(5) << _file_beg.sod()
               << " " << setw(2) << _file_end.year()
               << ":" << setw(3) << _file_end.doy()
               << ":" << setw(5) << _file_end.sod();

        os << " " << setw(3) << "UNE"
           << setfill(' ') << std::fixed << setprecision(4)
           << " " << setw(8) << meta.ecc[2] // U
           << " " << setw(8) << meta.ecc[0] // N
           << " " << setw(8) << meta.ecc[1] // E
           << endl;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_REC(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        // "*STATION__ PT SOLN T DATA_START__ DATA_END____ DESCRIPTION_________ S/N__ FIRMW
        //   ABER.....  A ---- P 00:161:18000 00:161:61170 ASHTECH Z-XII3       ----- -----------

        os << left
           << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
           << right
           << " " << setw(2) << "A"
           << " " << setw(4) << "1"
           << " " << setw(1) << _technique
           << setfill('0');

        if (_snx_type == SINEX_GNS)
            os << " " << setw(2) << _file_beg.yr()
               << ":" << setw(3) << _file_beg.doy()
               << ":" << setw(5) << _file_beg.sod()
               << " " << setw(2) << _file_end.yr()
               << ":" << setw(3) << _file_end.doy()
               << ":" << setw(5) << _file_end.sod()
               << setfill(' ') << left
               << " " << setw(20) << meta.rec.substr(0, 20)
               << " " << setw(5) << "-----"
               << " " << setw(5) << "-----------";
        else
            os << " " << setw(2) << _file_beg.year()
               << ":" << setw(3) << _file_beg.doy()
               << ":" << setw(5) << _file_beg.sod()
               << " " << setw(2) << _file_end.year()
               << ":" << setw(3) << _file_end.doy()
               << ":" << setw(5) << _file_end.sod()
               << setfill(' ') << left
               << " " << setw(20) << meta.rec.substr(0, 20)
               << " " << setw(20) << "--------------------"
               << " " << setw(11) << "-----------";

        os << endl;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_ANT(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        // "*STATION__ PT SOLN T DATA_START__ DATA_END____ DESCRIPTION_________ S/N__
        //   ABER.....  A ---- P 00:161:18000 00:161:61170 ASH700936A_M         -----

        os << left
           << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
           << right
           << " " << setw(2) << "A"
           << " " << setw(4) << "1"
           << " " << setw(1) << _technique
           << setfill('0');

        if (_snx_type == SINEX_GNS)
            os << " " << setw(2) << _file_beg.yr()
               << ":" << setw(3) << _file_beg.doy()
               << ":" << setw(5) << _file_beg.sod()
               << " " << setw(2) << _file_end.yr()
               << ":" << setw(3) << _file_end.doy()
               << ":" << setw(5) << _file_end.sod()
               << setfill(' ') << left
               << " " << setw(20) << meta.ant.substr(0, 20)
               << " " << setw(5) << "-----";
        else
            os << " " << setw(2) << _file_beg.year()
               << ":" << setw(3) << _file_beg.doy()
               << ":" << setw(5) << _file_beg.sod()
               << " " << setw(2) << _file_end.year()
               << ":" << setw(3) << _file_end.doy()
               << ":" << setw(5) << _file_end.sod()
               << setfill(' ') << left
               << " " << setw(20) << meta.ant.substr(0, 20)
               << " " << setw(20) << "--------------------"
               //      << " " << setw(5)  << "-----"
               << " " << setw(10) << meta.snx_code;

        os << endl;
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_PCO(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS)
            // *                           UP____ NORTH_ EAST__ UP____ NORTH_ EAST__
            // *DESCRIPTION_________ S/N__ L1->ARP(m)__________ __L2->ARP(m)________ AZ_EL_____

            os << left
               << " " << setw(20) << meta.ant.substr(0, 20)
               << " " << setw(5) << "-----"
               << right << scientific << setprecision(4)
               << " " << setw(6) << base_type_conv::FloatWithoutZero(meta.gps_neu_L1[2], 4) // Up
               << " " << setw(6) << base_type_conv::FloatWithoutZero(meta.gps_neu_L1[0], 4) // North
               << " " << setw(6) << base_type_conv::FloatWithoutZero(meta.gps_neu_L1[1], 4) // East
               << " " << setw(6) << base_type_conv::FloatWithoutZero(meta.gps_neu_L2[2], 4) // Up
               << " " << setw(6) << base_type_conv::FloatWithoutZero(meta.gps_neu_L2[0], 4) // North
               << " " << setw(6) << base_type_conv::FloatWithoutZero(meta.gps_neu_L2[1], 4) // East
               << endl;
        /*   
  if( _snx_type == TROSNX_GNS )
    // *DESCRIPTION_________ S/N_________________ PCO_model_

    os  << left
    << " " << setw(20) << meta.ant.substr(0,20)
        << " " << setw(20) << ""
      << " " << setw(10) << _pco_mod
    << endl;
*/
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_EST(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        base_time mid(meta.begPOS + (meta.endPOS - meta.begPOS) / 2);

        for (int i = 0; i < 3; ++i)
        {
            os << right << " " << setw(5) << int(meta.idx[i])
               << left << " " << setw(6) << meta.par[i]
               << left << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
               << right << " " << setw(2) << "A"
               << " " << setw(4) << "1"
               << setfill('0');
            if (_snx_type == SINEX_GNS)
                os << " " << setw(2) << mid.yr()
                   << ":" << setw(3) << mid.doy()
                   << ":" << setw(5) << mid.sod();
            else
                os << " " << setw(2) << mid.year()
                   << ":" << setw(3) << mid.doy()
                   << ":" << setw(5) << mid.sod();

            os << setfill(' ')
               << left << " " << setw(4) << "m"
               << " " << setw(1) << "2"
               << right << scientific
               << setprecision(14) << " " << setw(21) << meta.xyz[i]
               << setprecision(5) << " " << setw(10) << meta.rms[i]
               << endl;
        }
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_APR(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        base_time mid(meta.begPOS + (meta.endPOS - meta.begPOS) / 2);

        for (int i = 0; i < 3; ++i)
        {
            os << right << " " << setw(5) << int(meta.idx[i])
               << left << " " << setw(6) << meta.par[i]
               << left << " " << setw(_code_label.size()) << meta.name.substr(0, _code_label.size())
               << right << " " << setw(2) << "A"
               << " " << setw(4) << "1"
               << setfill('0');
            if (_snx_type == SINEX_GNS)
            {
                os << " " << setw(2) << mid.yr()
                   << ":" << setw(3) << mid.doy()
                   << ":" << setw(5) << mid.sod();
            }
            else
            {
                os << " " << setw(2) << mid.year()
                   << ":" << setw(3) << mid.doy()
                   << ":" << setw(5) << mid.sod();
            }
            os << setfill(' ')
               << left << " " << setw(4) << "m"
               << " " << setw(1) << "2"
               << right << scientific
               << setprecision(14) << " " << setw(21) << meta.apr[i]
               << setprecision(5) << " " << setw(10) << meta.std[i]
               << endl;
        }
    }

    // prefill metadata
    // ---------
    void gnss_coder_sinex::_fill_site_COV(string site, gnss_STC_META &meta, std::ostringstream &os)
    {

        os << right << " " << setw(5) << int(meta.idx[0]) // X
           << right << " " << setw(5) << int(meta.idx[0]) // X
           << scientific << setprecision(14) << " " << setw(21) << meta.var[0]
           << setprecision(14) << " " << setw(21) << ""
           << setprecision(14) << " " << setw(21) << ""
           << endl;

        os << right << " " << setw(5) << int(meta.idx[1]) // Y
           << right << " " << setw(5) << int(meta.idx[0]) // X
           << scientific << setprecision(14) << " " << setw(21) << meta.cov[0]
           << setprecision(14) << " " << setw(21) << meta.var[1]
           << setprecision(14) << " " << setw(21) << ""
           << endl;

        os << right << " " << setw(5) << int(meta.idx[2]) // Z
           << right << " " << setw(5) << int(meta.idx[0]) // X
           << scientific << setprecision(14) << " " << setw(21) << meta.cov[1]
           << setprecision(14) << " " << setw(21) << meta.cov[2]
           << setprecision(14) << " " << setw(21) << meta.var[2]
           << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_INI()
    {

        string snx("SNX");
        if (_snx_type != SINEX_GNS)
            snx = "TRO";

        // HEADER LINE
        // %=TRO 2.00 GOP 15:006:43304 GPS 13:345:00000 13:346:00000 P  MIX"
        // %=SNX 2.02 GOP 15:006:43304 GPS 13:345:00000 13:346:00000 P  MIX"

        _ss << "%=" << setw(3) << snx
            << std::fixed << setprecision(2)
            << " " << setw(4) << _version
            << " " << setw(3) << "GOP"
            << setfill('0');

        if (_snx_type == SINEX_GNS)
            _ss << " " << setw(2) << _file_run.yr()
                << ":" << setw(3) << _file_run.doy()
                << ":" << setw(5) << _file_run.sod()
                << " " << setw(3) << "GOP"
                << " " << setw(2) << _file_beg.yr()
                << ":" << setw(3) << _file_beg.doy()
                << ":" << setw(5) << _file_beg.sod()
                << " " << setw(2) << _file_end.yr()
                << ":" << setw(3) << _file_end.doy()
                << ":" << setw(5) << _file_end.sod();
        else
            _ss << " " << setw(2) << _file_run.year()
                << ":" << setw(3) << _file_run.doy()
                << ":" << setw(5) << _file_run.sod()
                << " " << setw(3) << "GOP"
                << " " << setw(2) << _file_beg.year()
                << ":" << setw(3) << _file_beg.doy()
                << ":" << setw(5) << _file_beg.sod()
                << " " << setw(2) << _file_end.year()
                << ":" << setw(3) << _file_end.doy()
                << ":" << setw(5) << _file_end.sod();

        _ss << setfill(' ')
            << " " << setw(1) << _technique
            << " " << setw(4) << "MIX"
            << endl;

        int ver_numb = 1; // HARDWIRED unique (number) identifier for the solution
        _ss << "*-------------------------------------------------------------------------------" << endl
            << "+FILE/REFERENCE" << endl
            << "*INFO_TYPE_________ INFO________________________________________________________" << endl
            << " DESCRIPTION        GOP - Geodetic Observatory Pecny, RIGTC                     " << endl
            << " OUTPUT             Solution parameters                                         " << endl
            << " CONTACT            gnss@pecny.cz                                               " << endl
            << " SOFTWARE           " << _set->pgm() << endl
            << " INPUT              GNSS/NWM/RAO/OTH data                                       " << endl
            << " VERSION NUMBER     " << setfill('0') << setw(3) << ver_numb << setfill(' ') << endl;

        if (_snx_type != SINEX_GNS)

            _ss << "-FILE/REFERENCE" << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_IDE(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ID" << endl
                << "*" << _code_label
                << " PT __DOMES__ T _STATION DESCRIPTION__ APPROX_LON_ APPROX_LAT_ _APP_H_" << endl
                << os.str()
                << "-SITE/ID" << endl;
        else
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ID" << endl
                << "*" << _code_label
                << " PT __DOMES__ T _STATION DESCRIPTION__ _LONGITUDE _LATITUDE_ _HGT_ELI_ _HGT_MSL_" << endl
                << os.str()
                << "-SITE/ID" << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_SOL(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS) // ONLY SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SOLUTIONS/EPOCHS" << endl
                << "*" << _code_label
                << " PT SOLN T _data_START_ __data_END__ _MEAN_EPOCH_                          " << endl
                << os.str()
                << "-SOLUTIONS/EPOCHS" << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_XYZ(std::ostringstream &os)
    {

        if (_snx_type != SINEX_GNS) // ONLY TRO-SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/COORDINATES                                                               " << endl
                << "*" << _code_label
                << " PT SOLN T __data_START__ __data_END____ __STA_X_____ __STA_Y_____ __STA_Z_____ SYSTEM REMRK" << endl
                << os.str()
                << "-SITE/COORDINATES                                                               " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_ECC(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS) // ONLY SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ECCENTRICITY                                                              " << endl
                << "*                                                  UP______ NORTH___ EAST____   " << endl
                << "*" << _code_label
                << " PT SOLN T DATA_START__ DATA_END____ AXE ARP->BENCHMARK(M)_________   " << endl
                << os.str()
                << "-SITE/ECCENTRICITY                                                              " << endl;

        if (_snx_type == TROSNX_GNS) // ONLY TRO-SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ECCENTRICITY                                                              " << endl
                << "*                                                      UP______ NORTH___ EAST____" << endl
                << "*" << _code_label
                << " PT SOLN T __data_START__ __data_END____ AXE ARP->BENCHMARK(M)_________" << endl
                << os.str()
                << "-SITE/ECCENTRICITY                                                              " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_REC(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS) // ONLY SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/RECEIVER                                                                  " << endl
                << "*" << _code_label
                << " PT SOLN T DATA_START__ DATA_END____ DESCRIPTION_________ S/N__ FIRMW " << endl
                << os.str()
                << "-SITE/RECEIVER                                                                  " << endl;

        if (_snx_type == TROSNX_GNS) // ONLY TRO-SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/RECEIVER                                                                  " << endl
                << "*" << _code_label
                << " PT SOLN T __data_START__ __data_END____ DESCRIPTION_________ S/N_________________ FIRMW______" << endl
                << os.str()
                << "-SITE/RECEIVER                                                                  " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_ANT(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS) // ONLY SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ANTENNA                                                                   " << endl
                << "*" << _code_label
                << " PT SOLN T DATA_START__ DATA_END____ DESCRIPTION_________ S/N__       " << endl
                << os.str()
                << "-SITE/ANTENNA                                                                   " << endl;

        if (_snx_type == TROSNX_GNS) // ONLY TRO-SINEX !
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ANTENNA                                                                   " << endl
                << "*" << _code_label
                << " PT SOLN T __data_START__ __data_END____ DESCRIPTION_________ S/N__   " << endl
                << os.str()
                << "-SITE/ANTENNA                                                                   " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_head_PCO(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/GPS_PHASE_CENTER                                                          " << endl
                << "*                           UP____ NORTH_ EAST__ UP____ NORTH_ EAST__           " << endl
                << "*DESCRIPTION_________ S/N__ L1->ARP(m)__________ __L2->ARP(m)________ AZ_EL_____" << endl
                << os.str()
                << "-SITE/GPS_PHASE_CENTER                                                          " << endl;

        if (_snx_type == TROSNX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SITE/ANTENNA_PHASE_CENTER                                                      " << endl
                << "*DESCRIPTION_________ S/N_________________ PCO_model_                           " << endl
                << os.str()
                << "-SITE/ANTENNA_PHASE_CENTER                                                      " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_data_STT()
    {

        if (_snx_type == SINEX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SOLUTION/STATISTICS                                                            " << endl
                << "*_STATISTICAL PARAMETER________ __VALUE(S)____________                          " << endl
                << " SAMPLING INTERVAL (SECONDS)   " << right << setw(22) << _int << endl
                << " PHASE MEASUREMENTS SIGMA      " << right << setw(22) << 0.0200 << endl
                << "-SOLUTION/STATISTICS                                                            " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_data_EST(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SOLUTION/ESTIMATE                                                              " << endl
                << "*INDEX TYPE__ CODE PT SOLN _REF_EPOCH__ UNIT S __ESTIMATED VALUE____ _STD_DEV___" << endl
                << os.str()
                << "-SOLUTION/ESTIMATE                                                              " << endl;
    }

    // complete data
    // ---------
    void gnss_coder_sinex::_fill_data_APR(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SOLUTION/APRIORI                                                               " << endl
                << "*INDEX TYPE__ CODE PT SOLN _REF_EPOCH__ UNIT S __ESTIMATED VALUE____ _STD_DEV___" << endl
                << os.str()
                << "-SOLUTION/APRIORI                                                               " << endl;
    }

    // complete metadata
    // ---------
    void gnss_coder_sinex::_fill_data_COV(std::ostringstream &os)
    {

        if (_snx_type == SINEX_GNS)
            _ss << "*-------------------------------------------------------------------------------" << endl
                << "+SOLUTION/MATRIX_ESTIMATE L COVA                                                " << endl
                << "*PARA1 PARA2 ____PARA2+0__________ ____PARA2+1__________ ____PARA2+2__________  " << endl
                << os.str()
                << "-SOLUTION/MATRIX_ESTIMATE L COVA                                                " << endl;
    }

    // complete metadata to object from _allobj
    // --------
    void gnss_coder_sinex::_complete_obj(shared_ptr<gnss_data_rec> gobj, const base_time &epo)
    {
        if (!_allobj)
            return;

        string id = gobj->id();
        bool found = false;
        // 4-CH in Sinex and 9-CH in settings
        for (auto it = _rec.begin(); it != _rec.end(); it++)
        {
            if (it->compare(0, 4, id, 0, 4) == 0)
            {
                found = true;
                gobj->id(*it); // replace id from settings
            }
        }

        id = gobj->id();

        shared_ptr<gnss_data_rec> old_obj = std::dynamic_pointer_cast<gnss_data_rec>(_allobj->obj(id));

        if (old_obj == nullptr && found)
        {
            _allobj->add(gobj);
            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "Object created, using SINEX: " + id + epo.str_ymdhms(" "));
        }
        else if (old_obj != nullptr)
        {
            old_obj->compare(gobj, epo, "SINEX");
        }
    }

    // Set object coordinates from estimation block
    // Validity is from SOLUTION/EPOCH block
    void gnss_coder_sinex::_set_rec_crd()
    {
        for (auto itSITE = _sol_estim.begin(); itSITE != _sol_estim.end(); itSITE++)
        {
            string site = itSITE->first;
            Triple xyz;
            Triple std;
            base_time epo;
            base_time beg = FIRST_TIME;
            base_time end = LAST_TIME;

            auto itSITE2 = _sol_epoch.find(site);
            // if(itSITE2 == _sol_epoch.end()) return;
            if (itSITE2 == _sol_epoch.end())
                continue; // by ZHJ

            for (auto itEPO = itSITE->second.begin(); itEPO != itSITE->second.end(); itEPO++)
            {
                epo = itEPO->first;

                auto itBE = itSITE2->second.upper_bound(epo);
                if (itBE != itSITE2->second.begin() && itBE != itSITE2->second.end())
                {
                    end = *itBE;
                    itBE--;
                    beg = *itBE;
                }

                for (auto itPAR = itEPO->second.begin(); itPAR != itEPO->second.end(); itPAR++)
                {
                    string par = itPAR->first;
                    if (par.compare("STAX") == 0)
                    {
                        xyz[0] = itPAR->second[0];
                        std[0] = itPAR->second[1];
                    }
                    else if (par.compare("STAY") == 0)
                    {
                        xyz[1] = itPAR->second[0];
                        std[1] = itPAR->second[1];
                    }
                    else if (par.compare("STAZ") == 0)
                    {
                        xyz[2] = itPAR->second[0];
                        std[2] = itPAR->second[1];
                    }
                }
            }
            shared_ptr<gnss_data_rec> new_obj = make_shared<gnss_data_rec>(_spdlog);
            new_obj->id(site);
            new_obj->spdlog(_spdlog);
            new_obj->crd(xyz, std, beg, end);

            _complete_obj(new_obj, epo);
        }
    }

} // namespace
