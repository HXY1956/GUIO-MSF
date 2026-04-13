#include "hwa_gnss_all_pcv.h"
#include "hwa_gnss_coder_atx.h"
#include "hwa_gnss_sys.h"
#include "hwa_base_time.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{
    gnss_coder_atx::gnss_coder_atx(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {
    }

    /* ----------
 * NAV-RINEX header
 */
    int gnss_coder_atx::decode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        }

        string tmp;
        int consume = 0;
        int tmpsize = 0;
        while ((tmpsize = base_coder::_getline(tmp)) >= 0)
        {

            consume += tmpsize;
            if (tmp.find("ANTEX VERSION", 60) != string::npos)
            { // first line

                _version = tmp.substr(0, 8);
                if (_spdlog && base_type_conv::substitute(_version, " ", "") > 0)
                {
                    std::ostringstream ltmp;
                    ltmp << "version = " << _version;

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, ltmp.str());
                }
            }
            else if (tmp.find("PCV TYPE / REFANT", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "reading PCV TYPE / REFANT");
            }
            else if (tmp.find("COMMENT", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "reading COMMENT");
            }
            else if (tmp.find("END OF HEADER", 60) != string::npos)
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "reading END OF HEADER ");
                base_coder::_consume(tmpsize);
                _mutex.unlock();
                return -1;
            }
            base_coder::_consume(tmpsize);
        }
#ifdef DEBUG
        std::cout << "\nREST BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER\n\n";
        std::cout.flush();
#endif

        _mutex.unlock();
        return consume;
    }

    /* ----------
 * NAV-RINEX body
 */
    int gnss_coder_atx::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();

        if (base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        std::cout << " BUFFER : \n"
             << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        std::cout.flush();
#endif
        int tmpsize = 0;   // individual reading counter
        int consume = 0;   // total read counter
        int recsize = 0;   // single record counter
        int rms = false;   // indicator for rms
        bool freq = false; // indicator for frequency
        string line;
        std::istringstream istr;
        string freqstr;
        // loop over current buffer (all records)
        // always from beginning of the buffer
        while ((tmpsize = base_coder::_getline(line, recsize)) >= 0)
        {
            recsize = tmpsize; //initialize

            // START OF ANTENNA
            // identify first antenna record
            if (line.find("START OF ANTENNA", 60) != string::npos)
            {

                if ((tmpsize = base_coder::_getline(line, recsize)) <= 0)
                {
                    _mutex.unlock();
                    return consume;
                }

                GFRQ freq1(LAST_GFRQ); // freq1 = XXX;
                GFRQ freq2(LAST_GFRQ); // freq2 = XXX;

                Triple neu;
                gnss_data_pcv::hwa_map_Z mapZ;
                gnss_data_pcv::hwa_map_A mapA;
                shared_ptr<gnss_data_pcv> pcv = make_shared<gnss_data_pcv>(_spdlog);
                pcv->beg(FIRST_TIME);
                pcv->end(LAST_TIME);
                base_time beg(base_time::GPS), end(base_time::GPS);

                // loop over a single record (keeping unfinished record in buffer)
                // begin from START OF ANTENNA
                while ((tmpsize = base_coder::_getline(line, recsize)) >= 0)
                {
                    recsize += tmpsize; // increase counter

                    // END OF ANTENNA
                    // end of antenna record --> process it
                    if (line.find("END OF ANTENNA", 60) != string::npos)
                    {

                        // fill pcv
                        map<string, base_data *>::iterator it = _data.begin();
                        while (it != _data.end())
                        {
                            if (it->second->id_type() == base_data::ALLPCV)
                            {
                                dynamic_cast<gnss_all_pcv *>(it->second)->addpcv(pcv);
                            }

                            it++;
                        }
                        consume += base_coder::_consume(recsize);
                        recsize = 0;
                        cnt++;

#ifdef DEBUG
                        std::cout << "Pridavam antenu = " << pcv->anten() << endl
                             << endl;
#endif

                        break; // exit a single antenna record loop

                        // TYPE / SERIAL NO
                    }
                    else if (line.find("TYPE / SERIAL NO", 60) != string::npos)
                    {

                        pcv->anten(base_type_conv::trim(line.substr(0, 20)));
                        pcv->ident(base_type_conv::trim(line.substr(20, 20)));
                        pcv->svcod(base_type_conv::trim(line.substr(40, 20)));

                        //  string type    = line.substr( 0, 20);
                        //  string ser_prn = line.substr(20, 20);
                        //  string svn     = line.substr(40, 10);

                        // remove blanks at the end of string! (assume starts with non-blank char)
                        //  size_t last = type.find_last_not_of(' ');
                        //       type = type.substr(0,last+1);
                        //         last = ser_prn.find_last_not_of(' ');
                        //    ser_prn = ser_prn.substr(0,last+1);
                        //         last = svn.find_last_not_of(' ');
                        //   svn = svn.substr(0,last+1);
                        //
                        //  if( line.substr(0,6) == "BLOCK " ||
                        //      line.substr(0,6) == "GLONAS" ){
                        //
                        //    pcv->antenn(ser_prn); //  sat: PRN
                        //    pcv->sernum("");     //
                        //  }else{
                        //         pcv->antenn(type);    // rec: antenna name,
                        //         pcv->sernum(ser_prn);      // rec: antenna serial number
                        //  }
                        //

#ifdef DEBUG
                        std::cout << "TYPE / SERIAL NO   [" << pcv->anten() << "] [" << pcv->ident() << "] [" << pcv->svcod() << "]\n";
#endif

                        // METH / BY / # / DATE
                    }
                    else if (line.find("METH / BY / # / DATE", 60) != string::npos)
                    {
                        pcv->method(base_type_conv::trim(line.substr(0, 20)));
                        pcv->source(base_type_conv::trim(line.substr(20, 20)));
#ifdef DEBUG
                        std::cout << "TYPE / METHOD, BY  [" << pcv->method() << "] [" << pcv->source() << "]\n";
#endif

                        // DAZI
                    }
                    else if (line.find("DAZI", 60) != string::npos)
                    {
                        pcv->dazi(base_type_conv::str2dbl(line.substr(2, 6)));
#ifdef DEBUG
                        std::cout << "DAZI               [" << pcv->dazi() << "]\n";
#endif
                        // ZEN1 / ZEN2 / DZEN
                    }
                    else if (line.find("ZEN1 / ZEN2 / DZEN", 60) != string::npos)
                    {
                        pcv->zen1(base_type_conv::str2dbl(line.substr(2, 6)));
                        pcv->zen2(base_type_conv::str2dbl(line.substr(8, 6)));
                        pcv->dzen(base_type_conv::str2dbl(line.substr(14, 6)));
#ifdef DEBUG
                        std::cout << "ZEN1 / ZEN2 / DZEN [" << pcv->zen1() << "] [" << pcv->zen2() << "] [" << pcv->dzen() << "]\n";
#endif

                        // # OF FREQUENCIES
                    }
                    else if (line.find("# OF FREQUENCIES", 60) != string::npos)
                    {
#ifdef DEBUG
                        int ifrq = base_type_conv::str2int(line.substr(0, 6));
                        std::cout << "# OF FREQUENCIES   [" << ifrq << "]\n";
#endif

                        // VALID FROM
                    }
                    else if (line.find("VALID FROM", 60) != string::npos)
                    {

                        int yr, mn, dd, hr, mi;
                        double sc;
                        istr.clear();
                        istr.str(line);
                        istr >> yr >> mn >> dd >> hr >> mi >> sc;
                        if (istr.fail())
                        {
                            _mutex.unlock();
                            return consume;
                        }

                        beg.from_ymd(yr, mn, dd, (hr * 3600 + mi * 60), sc);
                        pcv->beg(beg);
#ifdef DEBUG
                        std::cout << "VALID FROM         [" << beg.str_ymdhms() << "]\n";
#endif

                        // VALID UNTIL
                    }
                    else if (line.find("VALID UNTIL", 60) != string::npos)
                    {
                        int yr, mn, dd, hr, mi;
                        double sc;

                        istr.clear();
                        istr.str(line);
                        istr >> yr >> mn >> dd >> hr >> mi >> sc;
                        if (istr.fail())
                        {
                            _mutex.unlock();
                            return consume;
                        }

                        end.from_ymd(yr, mn, dd, (hr * 3600 + mi * 60), sc);
                        pcv->end(end);
#ifdef DEBUG
                        std::cout << "VALID UNTIL        [" << end.str_ymdhms() << "]\n";
#endif

                        // SINEX CODE
                    }
                    else if (line.find("SINEX CODE", 60) != string::npos)
                    {
                        pcv->snxcod(line.substr(0, 10));
#ifdef DEBUG
                        std::cout << "SINEX CODE         [" << pcv->snxcod() << "]\n";
#endif

                        // COMMENT
                    }
                    else if (line.find("COMMENT", 60) != string::npos)
                    {
                        // not implemented
#ifdef DEBUG
                        std::cout << "COMMENT            [" << line << "]\n";
#endif

                        // START OF FREQUENCY
                    }
                    else if (line.find("START OF FREQUENCY", 60) != string::npos)
                    {
                        freq = true;
                        freqstr = line.substr(3, 3);
                        freq1 = t_gfreq::str2gfreq(freqstr);
#ifdef DEBUG
                        std::cout << "START OF FREQ      [" << freq1 << "]\n";
#endif

                        // NORTH / EAST / UP
                    }
                    else if (freq && line.find("NORTH / EAST / UP", 60) != string::npos)
                    {
                        istr.clear();
                        istr.str(line);
                        istr >> neu[0] >> neu[1] >> neu[2];
                        if (istr.fail())
                        {
                            _mutex.unlock();
                            return consume;
                        }
#ifdef DEBUG
                        std::cout << "reading NEU excentricities\n";
#endif

                        // END OF FREQUENCY
                        // end of frequency record -> break loop
                    }
                    else if (line.find("END OF FREQUENCY", 60) != string::npos)
                    {
                        freq = false;
                        freqstr = line.substr(3, 3);
                        freq2 = t_gfreq::str2gfreq(freqstr);
                        if (freq1 == freq2)
                        {
                            if (freq1 == LAST_GFRQ)
                            {
                                if (_spdlog)
                                    SPDLOG_LOGGER_ERROR(_spdlog, "Not defined frequency code " + line.substr(3, 3));
                            }
                            else
                            {
                                pcv->pco(freq1, neu);

                                pcv->pcvzen(freq1, mapZ);
                                pcv->pcvazi(freq1, mapA);
                            }
                        }
                        else
                        {
                            std::cout << "WARNING: INCOMPLETE FREQUENCY END " << freq1 << " x " << freq2 << " skipped" << endl;
                            consume += base_coder::_consume(recsize);
                            _mutex.unlock();
                            return consume;
                        }
#ifdef DEBUG
                        std::cout << "END OF FREQ        [" << freq2 << "]\n";
#endif

                        // START OF FREQ RMS
                    }
                    else if (line.find("START OF FREQ RMS", 60) != string::npos)
                    {
                        freq = true;
                        rms = true;

                        // NORTH / EAST / UP
                    }
                    else if (freq && rms && line.find("NORTH / EAST / UP", 60) != string::npos)
                    {
                        // not yet implemented !
                        //
                        // END OF FREQ RMS
                    }
                    else if (freq && rms && line.find("END OF FREQ RMS", 60) != string::npos)
                    {
                        freq = false;
                        rms = false;
                        // not yet implemented !

                        // NOAZI
                    }
                    else if (freq && line.find("NOAZI", 3) != string::npos)
                    {
                        istr.clear();
                        istr.str(line);
                        string dummy;
                        istr >> dummy;
                        for (double i = pcv->zen1(); i <= pcv->zen2(); i += pcv->dzen())
                            istr >> mapZ[i];

                        if (istr.fail())
                        {
                            _mutex.unlock();
                            return consume;
                        }
#ifdef DEBUG
                        std::cout << "reading NOAZI field\n";
#endif

                        // AZIMUTH-DEPENDENT
                    }
                    else if (freq)
                    {
                        istr.clear();
                        istr.str(line);
                        double azi;
                        istr >> azi;
                        gnss_data_pcv::hwa_map_Z mapZ_A;
                        for (double i = pcv->zen1(); i <= pcv->zen2(); i += pcv->dzen())
                        {
                            istr >> mapZ_A[i];
                        }
                        mapA[azi] = mapZ_A;

                        if (istr.fail())
                        {
                            _mutex.unlock();
                            return consume;
                        }
#ifdef DEBUG
                        std::cout << "reading AZI-dependent fields\n";
#endif
                    }
                    else
                    {
                        std::cout << endl
                             << "atx: record not recognized: " << line << endl
                             << endl;
                    }
                } // LOOP INSIDE INDIVIDIUAL ANTENNA
            }     // END OF ANTENNA START
        }         // LOOP OVER ALL ANTENNAS

        _mutex.unlock();
        return consume;
    }

    int gnss_coder_atx::encode_head(char *buff, int sz, vector<string> &errmsg)
    {
        _mutex.lock();
        if (_ss_position == 0)
        {
            _ss << "1.4            M                                            ANTEX VERSION / SYST" << endl;
            _ss << "A                                                           PCV TYPE / REFANT" << endl;
            _ss << "                                                            END OF HEADER" << endl;
        }
        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

    int gnss_coder_atx::encode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {
        _mutex.lock();
        if (_ss_position == 0)
        {
            for (auto it = _data.begin(); it != _data.end(); it++)
            {
                if (it->second->id_type() == base_data::ALLPCV)
                {
                    gnss_all_pcv *all_pcv = dynamic_cast<gnss_all_pcv *>(it->second);
                    map<string, map<string, map<base_time, shared_ptr<gnss_data_pcv>>>> mappcv = all_pcv->mappcv();
                    for (auto ant_iter = mappcv.begin(); ant_iter != mappcv.end(); ant_iter++)
                    {
                        string ant = ant_iter->first;
                        for (auto typ_iter = ant_iter->second.begin(); typ_iter != ant_iter->second.end(); typ_iter++)
                        {
                            string typ = typ_iter->first;
                            for (auto tim_iter = typ_iter->second.begin(); tim_iter != typ_iter->second.end(); tim_iter++)
                            {
                                shared_ptr<gnss_data_pcv> pcv = tim_iter->second;
                                base_time t = pcv->beg();
                                if (pcv->anten() == "" && pcv->ident() == "")
                                {
                                    continue;
                                }
                                _ss << "                                                            START OF ANTENNA" << endl;
                                _ss << pcv->anten() << string(20 - pcv->anten().size(), ' ');
                                _ss << pcv->ident() << string(20 - pcv->ident().size(), ' ');
                                _ss << pcv->svcod() << string(20 - pcv->svcod().size(), ' ');
                                _ss << "TYPE / SERIAL NO" << endl;
                                _ss << string(59 - t.str_ymd().size(), ' ') << t.str_ymd() << " METH / BY / # / DATE" << endl;
                                _ss << string(2, ' ') << setw(6) << std::fixed << setprecision(1) << right << pcv->dazi() << string(52, ' ') << "DAZI" << endl;
                                _ss << string(2, ' ') << setw(6) << std::fixed << setprecision(1) << right << pcv->zen1()
                                    << setw(6) << std::fixed << setprecision(1) << right << pcv->zen2()
                                    << setw(6) << std::fixed << setprecision(1) << right << pcv->dzen()
                                    << string(40, ' ') << "ZEN1 / ZEN2 / DZEN" << endl;
                                int nfreq;
                                bool lazi;
                                map<GFRQ, map<double, double>> zen_map;
                                map<GFRQ, map<double, map<double, double>>> azi_map;
                                map<GFRQ, Triple> pco_map = pcv->pco();
                                if (double_eq(pcv->dazi(), 0))
                                {
                                    zen_map = pcv->pcvzen();
                                    nfreq = zen_map.size();
                                    lazi = false;
                                }
                                else
                                {
                                    zen_map = pcv->pcvzen();
                                    azi_map = pcv->pcvazi();
                                    nfreq = azi_map.size();
                                    lazi = true;
                                }
                                _ss << setw(6) << right << nfreq << string(54, ' ') << "# OF FREQUENCIES" << endl;
                                _ss << setw(6) << right << t.year()
                                    << setw(6) << right << t.mon()
                                    << setw(6) << right << t.day()
                                    << setw(6) << right << t.hour()
                                    << setw(6) << right << t.mins()
                                    << setw(13) << setprecision(7) << std::fixed << right << t.dsec()
                                    << string(17, ' ') << "VALID FROM" << endl;
                                _ss << pcv->snxcod() << string(50, ' ') << "SINEX CODE" << endl;

                                if (!lazi)
                                {
                                    for (auto iter = zen_map.begin(); iter != zen_map.end(); iter++)
                                    {
                                        GFRQ freq = iter->first;
                                        Triple pco = pco_map[freq];
                                        map<double, double> zens = iter->second;
                                        _ss << string(3, ' ') << t_gfreq::gfreq2str(freq) << string(54, ' ') << "START OF FREQUENCY" << endl;
                                        _ss << setw(10) << std::fixed << setprecision(2) << right << pco[0]
                                            << setw(10) << std::fixed << setprecision(2) << right << pco[1]
                                            << setw(10) << std::fixed << setprecision(2) << right << pco[2]
                                            << string(30, ' ') << "NORTH / EAST / UP" << endl;
                                        _ss << string(3, ' ') << "NOAZI";
                                        for (auto tmp = zens.begin(); tmp != zens.end(); tmp++)
                                        {
                                            _ss << setw(10) << std::fixed << setprecision(2) << right << tmp->second;
                                        }
                                        _ss << endl;
                                        _ss << string(3, ' ') << t_gfreq::gfreq2str(freq) << string(54, ' ') << "END OF FREQUENCY" << endl;
                                    }
                                }
                                else
                                {
                                    for (auto iter = azi_map.begin(); iter != azi_map.end(); iter++)
                                    {
                                        GFRQ freq = iter->first;
                                        Triple pco = pco_map[freq];
                                        _ss << string(3, ' ') << t_gfreq::gfreq2str(freq) << string(54, ' ') << "START OF FREQUENCY" << endl;
                                        _ss << setw(10) << std::fixed << setprecision(2) << right << pco[0]
                                            << setw(10) << std::fixed << setprecision(2) << right << pco[1]
                                            << setw(10) << std::fixed << setprecision(2) << right << pco[2]
                                            << string(30, ' ') << "NORTH / EAST / UP" << endl;
                                        _ss << string(3, ' ') << "NOAZI";
                                        //for (auto tmp = zen_map[freq].begin(); tmp != zen_map[freq].end(); tmp++)
                                        for (auto tmp = iter->second.begin()->second.begin(); tmp != iter->second.begin()->second.end(); tmp++)
                                        {
                                            _ss << setw(10) << std::fixed << setprecision(2) << right << tmp->second;
                                        }
                                        _ss << endl;
                                        for (auto azi_iter = iter->second.begin(); azi_iter != iter->second.end(); azi_iter++)
                                        {
                                            double azi = azi_iter->first;
                                            map<double, double> zen_tmp = azi_iter->second;
                                            _ss << setw(8) << setprecision(1) << right << azi;
                                            for (auto zen_iter = zen_tmp.begin(); zen_iter != zen_tmp.end(); zen_iter++)
                                            {
                                                _ss << setw(10) << std::fixed << setprecision(2) << right << zen_iter->second;
                                            }
                                            _ss << endl;
                                        }
                                        _ss << string(3, ' ') << t_gfreq::gfreq2str(freq) << string(54, ' ') << "END OF FREQUENCY" << endl;
                                    }
                                }
                                _ss << string(60, ' ') << "END OF ANTENNA" << endl;
                            }
                        }
                    }
                }
            }
        }

        int size = _fill_buffer(buff, sz);
        _mutex.unlock();
        return size;
    }

} // namespace
