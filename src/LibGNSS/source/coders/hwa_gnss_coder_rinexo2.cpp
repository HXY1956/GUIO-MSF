#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <memory>
#include <algorithm>
#include "hwa_gnss_coder_rinexo2.h"
#include "hwa_base_fileconv.h"
#include "hwa_base_globaltrans.h"
#include "hwa_gnss_data_rec.h"
#include "hwa_gnss_data_trn.h"
#include "hwa_gnss_data_obsmanager.h"

using namespace std;

namespace hwa_gnss
{

    // constructor
    // ----------
    gnss_coder_rinexo2::gnss_coder_rinexo2(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz),
          _site(""),
          _epo_beg(LAST_TIME),
          _epo_end(FIRST_TIME),
          _xbeg(0), _xend(0), _xsmp(0),
          _xsys(0) //,  _xsat(0), _xsig(0)
    {
        this->clear();
    }
    gnss_coder_rinexo2::gnss_coder_rinexo2(base_time beg, base_time end, set_base *s, string version, int sz)
        : base_coder(beg, end, s, version, sz), gnss_base_coder(beg, end, s, version, sz),
          _site(""),
          _epo_beg(LAST_TIME),
          _epo_end(FIRST_TIME),
          _xbeg(0), _xend(0), _xsmp(0),
          _xsys(0) //,  _xsat(0), _xsig(0)
    {
        this->clear();
    }

    // destructor
    // ----------
    gnss_coder_rinexo2::~gnss_coder_rinexo2()
    {

        // fill statistics of filtered data
        map<string, base_data *>::iterator itDAT = _data.begin();

        while (itDAT != _data.end())
        {
            if (itDAT->second->id_type() != base_data::ALLOBS)
            {
                itDAT++;
                continue;
            }

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, _fname + " [filtered data] xBEG:" + base_type_conv::int2str(_xbeg) + " xEND:" + base_type_conv::int2str(_xend) + " xSMP:" + base_type_conv::int2str(_xsmp) + " xSYS:" + base_type_conv::int2str(_xsys));

            gnss_all_obs::hwa_stc_xfilter xflt;

            xflt.xdat[gnss_all_obs::XDATA_BEG] = _xbeg;
            xflt.xdat[gnss_all_obs::XDATA_END] = _xend;
            xflt.xdat[gnss_all_obs::XDATA_SMP] = _xsmp;
            xflt.xdat[gnss_all_obs::XDATA_SYS] = _xsys;

            xflt.beg = _epo_beg;
            xflt.end = _epo_end;

            ((gnss_all_obs *)itDAT->second)->xdata(_rnxhdr.markname(), _fname, xflt);

            itDAT++;
        }
    };

    // clear
    // ----------
    void gnss_coder_rinexo2::clear()
    {
        gnss_base_coder::clear();

        _epoch = base_time::current_time(base_time::GPS);

        _rnxhdr.clear();

        _line = "";
        _flag = '0';
        _nsat = 0;
        _count = 0;
        _tmpsize = 0;
        _consume = 0;
        _complete = true;
        _csys = " ";
        _xbeg = _xend = _xsmp = _xsys = 0;
        //_xsat = _xsig = 0;

        _pcosat.erase(_pcosat.begin(), _pcosat.end());
        _pcosys.erase(_pcosys.begin(), _pcosys.end());
        _pcoecc.erase(_pcoecc.begin(), _pcoecc.end());
        _mapobs.erase(_mapobs.begin(), _mapobs.end());
    }

    // ----------
    // OBS-RINEX header
    //   - read individual lines (tmpsize once used and consumed)
    //   - read block of lines at once (tmpsize cummulated and then consumed)
    // ----------
    int gnss_coder_rinexo2::_decode_head()
    {

        // -------- "PGM / RUN BY / DATE" --------
        if (_line.find("PGM / RUN BY / DATE", 60) != string::npos)
        {
            _rnxhdr.program(base_type_conv::trim(_line.substr(0, 20)));
            _rnxhdr.runby(base_type_conv::trim(_line.substr(20, 20)));
            base_time gtime(base_time::UTC);
            if (_line.substr(56, 3) != "UTC")
                gtime.tsys(base_time::LOC);

            if (gtime.from_str("%Y%m%d %H%M%S", _line.substr(40, 15)) == 0)
                ;
            else if (gtime.from_str("%Y-%m-%d %H-%M-%S", _line.substr(40, 20)) == 0)
                ;
            else
            {
                gtime = FIRST_TIME;
            }
            _rnxhdr.gtime(gtime);

            // -------- "COMMENT" --------
        }
        else if (_line.substr(60, 7).find("COMMENT") != string::npos)
        {

            _comment.push_back(_line.substr(0, 60));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "COMMENT");
            // -------- "MARKER NAME" --------
        }
        else if (_line.substr(60, 11).find("MARKER NAME") != string::npos)
        {

            string guess, marker, src, typ, sit; // select   short (4-CHAR) or long (9-CHAR)

            // check sitename according to filename (if available)
            string filename = base_name(_fname);

            // first, check the MARKER NAME (RINEX HEADER)
            int i2;
            char c4[4], c3[3];
            const int size = 9;
            char tmpbuff[size + 1];
            strncpy(tmpbuff, _line.substr(0, size).c_str(), size);
            tmpbuff[size] = '\0';

            if (sscanf(tmpbuff, "%4c%2d%3c", c4, &i2, c3) == 3)
            { // guess the length (4-char vs 9-char)  %* ==> not robust!
                marker = guess = _line.substr(0, 9);
                src = "MARKER";
                typ = "long";
                sit = "(9-CH)";
            } // LONG  SITE NAME
            else
            {
                marker = guess = _line.substr(0, 4);
                src = "MARKER";
                typ = "short";
                sit = "(4-CH)";
            } // SHORT SITE NAME
            transform(marker.begin(), marker.end(), marker.begin(), ::toupper);

            // second, check the FILE NAME
            size_t n = count(filename.begin(), filename.end(), '_');
            if (!_fname.empty())
            {
                if (_fname.length() > 8 + 4 && n >= 5)
                {
                    guess = filename.substr(0, 9);
                    src = "FILE";
                    typ = "long";
                    sit = "(9-CH)";
                } // LONG  FILE NAME
                else
                {
                    guess = filename.substr(0, 4);
                    src = "FILE";
                    typ = "short";
                    sit = "(4-CH)";
                } // SHORT FILE NAME
                transform(guess.begin(), guess.end(), guess.begin(), ::toupper);

                if (guess != marker)
                {

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, src + ": " + typ + " site name: " + guess + " " + sit);
                    if (guess.substr(0, 4) != marker.substr(0, 4))
                    {
                        mesg(GWARNING, "Warning: 4-CHAR: MARKER_NAME [" + marker + "] not equal to site from filename [" + guess + "]");
                    }
                }
            }

            transform(guess.begin(), guess.end(), guess.begin(), ::toupper);

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, src + ": " + typ + " site name: " + guess + " " + sit + " identified");

            // resolve situation of 4-CH MARKER with 9-CH in settings
            if (guess.length() == 4)
            {
                for (auto itREC = _rec.begin(); itREC != _rec.end(); ++itREC)
                {
                    if (itREC->length() > 4 && itREC->compare(0, 4, guess, 0, 4) == 0)
                    {

                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "4-CH [" + guess + "] found in RINEX, but user requests 9-CH [" + (*itREC) + "]");
                        guess = *itREC;
                    }
                }
            }
            // resolve opposite situation (9-CH MARKER with 4-CH in settings)
            else if (guess.length() == 9)
            {
                for (auto itREC = _rec.begin(); itREC != _rec.end(); ++itREC)
                {
                    if (itREC->length() == 4 && itREC->compare(0, 4, guess, 0, 4) == 0)
                    {

                        if (_spdlog)
                            SPDLOG_LOGGER_DEBUG(_spdlog, "9-CH [" + guess + "] found in RINEX, but user requests 4-CH [" + (*itREC) + "]");
                        guess = *itREC;
                    }
                }
            }
            _site = guess;
            _rnxhdr.markname(base_type_conv::trim(marker)); // capitals, cutted according to the station name guess
                                            //  _rnxhdr.markname( base_type_conv::trim(_line.substr(0,60)) ); // original text in MARKER_NAME

            // -------- "MARKER NUMBER" --------
        }
        else if (_line.substr(60, 13).find("MARKER NUMBER") != string::npos)
        {

            _rnxhdr.marknumb(base_type_conv::trim(_line.substr(0, 60)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "MARKER NUMB: " + _rnxhdr.marknumb());

            // -------- "MARKER TYPE" --------
        }
        else if (_line.substr(60, 11).find("MARKER TYPE") != string::npos)
        {

            _rnxhdr.marktype(base_type_conv::trim(_line.substr(0, 20)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "MARKER TYPE: " + _rnxhdr.marktype());

            // -------- "OBSERVER / AGENCY" --------
        }
        else if (_line.substr(60, 17).find("OBSERVER / AGENCY") != string::npos)
        {

            _rnxhdr.observer(base_type_conv::trim(_line.substr(0, 20)));
            _rnxhdr.agency(base_type_conv::trim(_line.substr(20, 40)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "OBSERVER / AGENCY: " + _rnxhdr.observer() + " " + _rnxhdr.agency());

            // -------- "REC # / TYPE / VERS" --------
        }
        else if (_line.substr(60, 19).find("REC # / TYPE / VERS") != string::npos)
        {

            _rnxhdr.recnumb(base_type_conv::trim(_line.substr(0, 20)));
            _rnxhdr.rectype(base_type_conv::trim(_line.substr(20, 20)));
            _rnxhdr.recvers(base_type_conv::trim(_line.substr(40, 20)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "REC # / TYPE / VERS: " + _rnxhdr.recnumb() + " " + _rnxhdr.rectype() + " " + _rnxhdr.recvers());

            // -------- "ANT # / TYPE" --------
        }
        else if (_line.substr(60, 12).find("ANT # / TYPE") != string::npos)
        {

            _rnxhdr.antnumb(base_type_conv::trim(_line.substr(0, 20)));
            _rnxhdr.anttype(base_type_conv::trim(_line.substr(20, 20)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANT # / TYPE: " + _rnxhdr.antnumb() + " " + _rnxhdr.anttype());

            // -------- "APPROX POSITION XYZ" --------
        }
        else if (_line.substr(60, 19).find("APPROX POSITION XYZ") != string::npos)
        {

            Triple apr;
            apr[0] = base_type_conv::str2dbl(_line.substr(0, 14));
            apr[1] = base_type_conv::str2dbl(_line.substr(14, 14));
            apr[2] = base_type_conv::str2dbl(_line.substr(28, 14));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "APPROX POSITION XYZ: " + base_type_conv::dbl2str(apr[0]) + " " + base_type_conv::dbl2str(apr[1]) + " " + base_type_conv::dbl2str(apr[2]));
            _rnxhdr.aprxyz(apr);

            // -------- "ANTENNA: DELTA X/Y/Z --------
        }
        else if (_line.substr(60, 20).find("ANTENNA: DELTA X/Y/Z") != string::npos)
        {

            Triple ecc;
            ecc[0] = base_type_conv::str2dbl(_line.substr(0, 14));
            ecc[1] = base_type_conv::str2dbl(_line.substr(14, 14));
            ecc[2] = base_type_conv::str2dbl(_line.substr(28, 14));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANTENNA: DELTA X/Y/Z: " + base_type_conv::dbl2str(ecc[0]) + " " + base_type_conv::dbl2str(ecc[1]) + " " + base_type_conv::dbl2str(ecc[2]));
            _rnxhdr.antxyz(ecc);

            // -------- "ANTENNA: DELTA H/E/N" --------
        }
        else if (_line.substr(60, 20).find("ANTENNA: DELTA H/E/N") != string::npos)
        {

            Triple ecc;
            ecc[1] = base_type_conv::str2dbl(_line.substr(14, 14)); // East  // --> REVERSE with N !
            ecc[0] = base_type_conv::str2dbl(_line.substr(28, 14)); // North // --> REVERSE with E !
            ecc[2] = base_type_conv::str2dbl(_line.substr(0, 14));  // Up

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANTENNA: DELTA H/E/N: " + base_type_conv::dbl2str(ecc[0]) + " " + base_type_conv::dbl2str(ecc[1]) + " " + base_type_conv::dbl2str(ecc[2]));
            _rnxhdr.antneu(ecc);
            if (!ecc.isZero() && _rnxhdr.antxyz().isZero() && !_rnxhdr.aprxyz().isZero())
            {
                Triple refell;
                xyz2ell(_rnxhdr.aprxyz(), refell, false);
                Triple antxyz;
                neu2xyz(refell, ecc, antxyz);
                _rnxhdr.antxyz(antxyz);
            }

            // -------- "ANTENNA: PHASECENTER" --------
        }
        else if (_line.substr(60, 20).find("ANTENNA: PHASECENTER") != string::npos)
        {

            Triple ecc;
            ecc[0] = base_type_conv::str2dbl(_line.substr(5, 9));   // N or X
            ecc[1] = base_type_conv::str2dbl(_line.substr(14, 14)); // E or Y
            ecc[2] = base_type_conv::str2dbl(_line.substr(28, 14)); // U or Z

            _pcosat.push_back(_line[0]);                 // GNSS (G/R/E/S/...)
            _pcosys.push_back(base_type_conv::trim(_line.substr(2, 3))); // coordinate system (NEU/XYZ)
            _pcoecc.push_back(ecc);                      // NEU or XYZ phase center vs. ARP eccentricities

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANTENNA: PHASECENTER: ");

            // -------- "ANTENNA: B.SIGHT XYZ" --------
        }
        else if (_line.substr(60, 20).find("ANTENNA: B.SIGHT XYZ") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANTENNA: B.SIGHT XYZ: " + string("not implemented"));

            // -------- "ANTENNA: ZERODIR AZI --------
        }
        else if (_line.substr(60, 20).find("ANTENNA: ZERODIR AZI") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANTENNA: ZERODIR AZI: " + string("not implemented"));

            // -------- "ANTENNA: ZERODIR XYZ" --------
        }
        else if (_line.substr(60, 20).find("ANTENNA: ZERODIR XYZ") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "ANTENNA: ZERODIR XYZ: " + string("not implemented"));

            // -------- "CENTER OF MASS: XYZ" --------
        }
        else if (_line.substr(60, 19).find("CENTER OF MASS: XYZ") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "CENTER OF MASS: XYZ: " + string("not implemented"));

            // -------- "# / TYPES OF OBSERV" --------
        }
        else if (_line.substr(60, 19).find("# / TYPES OF OBSERV") != string::npos)
        {

            int fact = 1;
            //  char sys( string(1,_rnxhdr.rnxsys());
            if (!_csys.empty() && _csys[0] != ' ')
            {

                // guarantee that all relevant lines are available
                int num = base_type_conv::str2int(_line.substr(0, 6));
                int nlines = (int)ceil(num / 9.0); // upto 9 values/line
                int addsize = 0;
                string tmp;
                for (int nl = 0; nl < nlines; ++nl)
                {
                    int irc = gnss_base_coder::_getline(tmp, _tmpsize + addsize);
                    if (irc >= 0)
                    {
                        addsize += irc;
                    }
                    else
                    {
                        // std::cout << "INCOMPLETE RECORD(OBS TYPES): " << tmp << "\n";
                        _complete = false;
                        break;
                    }
                }

                if (!_complete)
                    return -1;

                // sufficient number of lines is already guaranteed here
                for (int ii = 0; ii < num; ++ii)
                {
                    // make sure that the obs is not already in hwa_vector_obs
                    string go = _line.substr(10 + 6 * (ii % (9)), 2);
                    GOBS obs = str2gobs(go);

                    gnss_data_rnxhdr::hwa_vector_obs::iterator it = _mapobs[_csys].begin();

                    // record exists
                    for (it = _mapobs[_csys].begin(); it != _mapobs[_csys].end(); ++it)
                    {
                        if (it->first == obs)
                        {
                            it->second = fact;
                            break;
                        }
                    }

                    // new record
                    //if( it == _mapobs[_csys].end() ) _mapobs[_csys].push_back( std::make_pair(obs, fact) );
                    _mapobs[_csys].push_back(std::make_pair(obs, fact)); // changed by ZHJ some situations have same obstype skipping may cause bugs

                    if ((ii + 1) % 9 == 0 && num > 9) // get newline and check (read next for old system)
                        _tmpsize += gnss_base_coder::_getline(_line, _tmpsize);
                } // loop over # observations
            }

            // -------- "INTERVAL" --------
        }
        else if (_line.substr(60, 8).find("INTERVAL") != string::npos)
        {

            string sample = _line.substr(0, 9);
            _rnxhdr.interval(base_type_conv::str2dbl(sample));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "INTERVAL: " + base_type_conv::dbl2str(_rnxhdr.interval()));

            if (_int > 1.0 && _rnxhdr.interval() < 1.0 && _scl == 1)
            {

                int _dec = sample.substr(sample.find(".") + 1).length() - 1; // decimal digits resolution
                _scl = (int)pow(10, _dec);
                cerr << "#Notice [rinexo2]: use decimals in XML settings if high-rate file."
                     << " Used <int> " << std::fixed << setprecision(_dec) << _int << " </int>\n";
            }

            // -------- "TIME OF FIRST OBS" --------
        }
        else if (_line.substr(60, 17).find("TIME OF FIRST OBS") != string::npos)
        {

            string tsys = base_type_conv::trim(_line.substr(48, 3));
            base_time first;

            if (tsys == "GPS")
                first.tsys(base_time::GPS);
            else if (tsys == "GLO")
                first.tsys(base_time::GLO);
            else if (tsys == "GAL")
                first.tsys(base_time::GAL);
            else if (tsys == "BDT" || tsys == "BDS")
                first.tsys(base_time::BDS);
            else
                first.tsys(base_time::GPS);

            first.from_ymdhms(base_type_conv::str2int(_line.substr(0, 6)),    // year
                              base_type_conv::str2int(_line.substr(6, 6)),    // month
                              base_type_conv::str2int(_line.substr(12, 6)),   // day
                              base_type_conv::str2int(_line.substr(18, 6)),   // hours
                              base_type_conv::str2int(_line.substr(24, 6)),   // minutes
                              base_type_conv::str2dbl(_line.substr(30, 13))); // double seconds

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, first.str_ymdhms("TIME OF FIRST OBS: "));
            //lijie
            _epoch.tsys(first.tsys());
            _rnxhdr.first(first);

            // -------- "TIME OF LAST OBS" --------
        }
        else if (_line.substr(60, 16).find("TIME OF LAST OBS") != string::npos)
        {

            string tsys = base_type_conv::trim(_line.substr(48, 3));
            base_time last;

            if (tsys == "GPS")
                last.tsys(base_time::GPS);
            else if (tsys == "GLO")
                last.tsys(base_time::GLO);
            else if (tsys == "GAL")
                last.tsys(base_time::GAL);
            else if (tsys == "BDT" || tsys == "BDS")
                last.tsys(base_time::BDS);
            else
                last.tsys(base_time::GPS);

            last.from_ymdhms(base_type_conv::str2int(_line.substr(0, 6)),    // year
                             base_type_conv::str2int(_line.substr(6, 6)),    // month
                             base_type_conv::str2int(_line.substr(12, 6)),   // day
                             base_type_conv::str2int(_line.substr(18, 6)),   // hours
                             base_type_conv::str2int(_line.substr(24, 6)),   // minutes
                             base_type_conv::str2dbl(_line.substr(30, 13))); // double seconds

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, last.str_ymdhms("TIME OF LAST OBS: "));
            //_epoch.tsys(last.tsys());
            _rnxhdr.last(last);

            // -------- "LEAP SECONDS" --------
        }
        else if (_line.substr(60, 12).find("LEAP SECONDS") != string::npos)
        {

            // implemented only 1st part - total LEAP SEC.
            _rnxhdr.leapsec(base_type_conv::str2int(_line.substr(0, 6)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "LEAP SECONDS: " + base_type_conv::int2str(_rnxhdr.leapsec()));

            // -------- "# OF SATELLITES" --------
        }
        else if (_line.substr(60, 15).find("# OF SATELLITES") != string::npos)
        {

            _rnxhdr.numsats(base_type_conv::str2int(_line.substr(0, 6)));

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "# OF SATELLITES: " + base_type_conv::int2str(_rnxhdr.numsats()));

            // -------- "PRN / # OF OBS --------
        }
        else if (_line.substr(60, 14).find("PRN / # OF OBS") != string::npos)
        {

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "PRN / # OF OBS: " + string("not implemented"));
        }

        return 1;
    }

    // ----------
    // OBS-RINEX body
    //   - read block of lines for each epoch (tmpsize cummulated and then consumed)
    // ----------
    int gnss_coder_rinexo2::_decode_data()
    {

        vector<string> satellites;
        int irc = gnss_coder_rinexo2::_read_epoch();

        // not ready to read
        if (irc < 0)
        {
            _complete = false;
            return -1;
        }

        // skip special lines
        if (irc > 0)
        {

            int xlines = 1; // special event,      skip only number of lines
            if (irc >= 2)
            { // skip observations,  skip x-nsat lines
                if (_read_satvec(satellites) < 0)
                {
                    _complete = false;
                    return -1;
                }
                xlines = (int)floor((_mapobs[_csys].size() - 1) / 5) + 1;
            }
            for (int i = 0; i < _nsat * xlines; ++i)
            {
                int sz = gnss_base_coder::_getline(_line, _tmpsize);
                if (sz <= 0)
                {
                    _complete = false;
                    return -1;
                }
                _tmpsize += sz;
            }

            gnss_base_coder::_consume(_tmpsize);
            _consume += _tmpsize;
            _tmpsize = 0;
            return 0;
        }

        // DATA READING
        // ------------
        // read data for vector for satellites (RINEX v2.x specific)
        if (_read_satvec(satellites) < 0)
        {
            _complete = false;
            return -1;
        }

        // loop over satellits records (x-lines)
        for (int i = 0; i < _nsat; i++)
        {

            string sat = satellites[i];
            string key = sat;

            // read sat-line and define system/satellite key in _mapobs (RINEX v2.x specific)
            if (gnss_coder_rinexo2::_read_syskey(sat, key) < 0)
            {
                _complete = false;
                return -1;
            }

            // loop over all observation types for system/satellite key in _mapobs
            if (gnss_coder_rinexo2::_read_obstypes(sat, key) < 0)
            {
                _complete = false;
                return -1;
            }
        }

        // fill data - block (epoch-wise) is completed
        if (_complete)
        {

            if (beg_epoch == base_time(0, 0) && end_epoch == base_time(0, 0))
            {
                _count += this->_fill_data();
            }
            else
            {
                if ((_epoch >= beg_epoch) && (_epoch <= end_epoch))
                {
                    _count += this->_fill_data();
                }
            }
            gnss_base_coder::_consume(_tmpsize);
            _consume += _tmpsize;
            _tmpsize = 0;
        }

        return 0;
    }

    // read sat-system key (G,E,R,.. M, or specific satellite for _mapobs)
    // only RINEX 2.x version
    // ----------
    int gnss_coder_rinexo2::_read_syskey(string &sat, string &key)
    {

        // get newline and check the end of buffer
        int addsize = gnss_base_coder::_getline(_line, _tmpsize);

        // getline succeed only if 'EOL' found
        if (_line.length() <= 0)
            return _stop_read();

        // select proper key (sat/sys) for observation list
        if (_mapobs.count(sat.substr(0, 3)) != 0)
            key = sat.substr(0, 3);
        else if (_mapobs.count(sat.substr(0, 1)) != 0)
            key = sat.substr(0, 1);
        else
            key = _csys; // string(1,_rnxhdr.rnxsys());

        if (key[0] != ' ' && key[0] != 'M')
            sat = gnss_sys::eval_sat(sat, gnss_sys::str2gsys(key));

#ifdef DEBUG
        std::cout << " _READ_SYSKEY :" << sat << "|" << sat.substr(0, 1)
             << "[" << key << "] "
             << " [" << _csys << "]\n"; // _rnxhdr.rnxsys()

        gnss_data_rnxhdr::hwa_map_obs::iterator it = _mapobs.begin();
        std::cout << "SYSKEY[" << _mapobs.size() << "]:";
        while (it != _mapobs.end())
        {
            std::cout << ":" << it->first;
            it++;
        }
        std::cout << endl;
#endif

        // check if key exists
        if (_mapobs.find(key) == _mapobs.end())
        {
            cerr << "rinexo2: Warning: mapobs syskey not found [" + key + "]\n";
            mesg(GWARNING, "Warning: mapobs syskey not found [" + key + "]");
            return _stop_read();
        }

        // check if enough data (+1 EOL)
        if (addsize <= (int)_mapobs[key].size() * 16 + 1)
        {
            _tmpsize += addsize;
        }
        else
        {

            return _stop_read();
        }
        return 1;
    }

    // read epoch & number of satellites, return flag
    // ----------
    int gnss_coder_rinexo2::_read_epoch()
    {

        _nsat = 0;
        _flag = 'X';

        // std::cout << " epoch line  :" << _line ;

        // not enough data to recognize standard epoch or special event
        if (_line.length() < 32)
            return _stop_read();
        _flag = _line[28];
        _nsat = base_type_conv::str2int(_line.substr(29, 3));
        if (_line.substr(0, 6) == "      ")
        {
            switch (_flag)
            {
            case '2':

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: start moving antenna identified, but not yet implemented!");
                mesg(GWARNING, "Warning: start moving antenna identified, but not yet implemented!");
                //else              cerr <<  "rinexo2 - Warning: start moving antenna identified, but not yet implemented!\n";
                return 1;

            case '3':

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: new site occupation identified, but not yet implemented!");
                mesg(GWARNING, "Warning: new site occupation identified, but not yet implemented!");
                //else              cerr <<  "rinexo2 - Warning: new site occupation identified, but not yet implemented!\n";
                return 1;

            case '4':

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: new header information identified, but not fully implemented!");
                mesg(GWARNING, "Warning: new header information identified, but not fully implemented!");
                //else              cerr <<  "rinexo2 - Warning: new header information identified, but not fully implemented!\n";
                return 1;

            case '5':

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "Warning: external event identified, but not yet implemented!");
                mesg(GWARNING, "Warning: external event identified, but not yet implemented!");
                //else             cerr <<  "rinexo2 - Warning: external event identified, but not yet implemented!\n";
                return 1;
            }
        }

        std::istringstream is(_line);
        is.clear();

        int yr, mn, dd, hr, mi;
        double sc;

        is >> yr >> mn >> dd >> hr >> mi >> sc >> _flag >> _nsat;

        // check success
        if (is.fail())
        { // std::cout << "rinexo: problem reading EPOCH:" << _line;
            return _stop_read();
        }

        _epoch.from_ymdhms(yr, mn, dd, hr, mi, sc);
        epoch = _epoch;
        // filter out data
        if (_flag != '0' &&
            _flag != '1')
        {
            return 1;
        }
        if (_epoch > _end)
        {
            if (_epoch > _epo_end)
            {
                _epo_end = _epoch;
            }
            _xend++;
            return 2;
        }
        if (_epoch < _beg)
        {
            if (_epoch < _epo_beg)
            {
                _epo_beg = _epoch;
            }
            _xbeg++;
            return 2;
        }

        if (!_filter_epoch(_epoch))
        {
            _xsmp++;
            return 2;
        }

        return 0;
    }

    // read satellite list (vector)
    // ----------
    int gnss_coder_rinexo2::_read_satvec(vector<string> &satellites)
    {

        int ii = 0;
        for (int i = 0; i < _nsat; i++, ii++)
        {

            // getline succeed only if 'EOL' found
            if ((int)_line.length() <= 0)
                return _stop_read();

            // read extra line for each satellite (if complete)
            unsigned int idx = 32 + ii * 3;

            if (ii > 11)
            {
                ii = 0;
                idx = 32;
                unsigned int addsize = gnss_base_coder::_getline(_line, _tmpsize);

                if (addsize < idx + 3)
                    return _stop_read();
                else
                    _tmpsize += addsize;
            }

            if (_line.length() >= idx + 3)
            {

                string sat = gnss_sys::eval_sat(_line.substr(idx, 3));
                satellites.push_back(sat);
            }
        }

        return _tmpsize;
    }

    // read satellite observation types
    // ----------
    int gnss_coder_rinexo2::_read_obstypes(const string &sat, const string &sys)
    {

        int ii = 0;
        int addsize = 0;
        unsigned int idx = 0;

        // filter GNSS and SAT
        bool filter_sat = _filter_gnss(sat);
        if (!filter_sat)
        {
            _xsys++;

            if (_spdlog)
                SPDLOG_LOGGER_DEBUG(_spdlog, "skip " + sat);
        }

        // general sys- or sat- specific group of observations
        string key = sat;
        gnss_data_rnxhdr::hwa_map_obs::const_iterator itMAP = _mapobs.find(sat);
        if (itMAP == _mapobs.end())
        {
            key = _csys;
        }

        // check if key exists
        if (_mapobs.find(key) == _mapobs.end())
        {
            cerr << "rinexo2: Warning: _mapobs syskey not found [" << key << "]\n";
            mesg(GWARNING, "Warning: _mapobs syskey not found [" + key + "]");
            return _stop_read();
        }

        gnss_data_rnxhdr::hwa_vector_obs::const_iterator it = _mapobs[key].begin();

        //  hwa_spt_obsmanager obs = make_shared<gnss_data_obs_manager>(_rnxhdr.markname(), sat, _epoch);
        hwa_spt_obsmanager obs = make_shared<gnss_data_obs_manager>(_spdlog, _site, sat, _epoch);

        // loop over observation types
        while (it != _mapobs[key].end())
        {

            idx = 16 * ii;

            // read new line for satellite observations
            if (ii > 4)
            {
                if ((addsize = gnss_base_coder::_getline(_line, _tmpsize)) >= 0)
                {
                    _tmpsize += addsize;
                    ii = 0;
                    idx = 0;
                }
                else
                    return _stop_read();
            }

            // last check (after new getline or for missing record)
            if ((_line.length() < (idx + 14)) || base_type_conv::trim(_line.substr(idx, 14)).empty())
            {
                if (filter_sat)
                {
                    _null_log(sat, gobs2str(it->first));
                }
            }
            else if (filter_sat)
            {
                //        cerr << " JD fix: " << sat[0] << " go: " << gobs2str(it->first) << endl;
                string tmp = gobs2str(it->first);
                if (gnss_coder_rinexo2::_fix_band(string(1, sat[0]), tmp))
                {
                    _read_obs(idx, it, obs);
                }
            }

            ii++;
            it++;

        } // while (over observation types)

        if (filter_sat)
            _vobs.push_back(obs);

#ifdef DEBUG
        std::cout << " RNX: " << _version
             << " sys: " << sys
             << " sat: " << sat
             << " key: " << key
             << " epo: " << _epoch.str_ymdhms()
             //       << " code/phase ID"
             //       << " - band1:"  << " " << gobs2str( obs.code(1)  )
             //                       << " " << gobs2str( obs.phase(1) )
             //       << " - band2:"  << " " << gobs2str( obs.code(2)  )
             //                       << " " << gobs2str( obs.phase(2) )
             << endl;
#endif

        return 1;
    }

    // fill single observation element
    // ----------
    int gnss_coder_rinexo2::_read_obs(const unsigned int &idx,
                             const gnss_data_rnxhdr::hwa_vector_obs::const_iterator &it,
                             hwa_spt_obsmanager obs)
    {

        GOBS type = it->first;
        GSYS gsys = obs->gsys();

        // filter OBS   modifed by glfeng, fix bugs about rinexo2
        if (_obs[gsys].size() > 0 &&
            _obs[gsys].find(gobs2str(type)) == _obs[gsys].end())
            return 0;

        //string valstr = base_type_conv::trim(_line.substr(idx,14)); if( valstr.empty() ) return 0; // eliminate empty string
        //string valstr = _line.substr(idx,14).c_str();                              // base_type_conv::trim not necessary, empty => 0.0
        string valstr = _line.substr(idx, 14); // base_type_conv::trim not necessary, empty => 0.0
        double valdbl = base_type_conv::str2dbl(valstr);
        if (double_eq(valdbl, 0.0))
            return 0; // eliminate 0.000

        obs->addobs(type, valdbl * it->second);

#ifdef DEBUG
        std::cout << obs->sat()
             << " idx = " << setw(2) << idx << std::fixed << setprecision(3)
             << "  [" << type
             << "] [" << gobs2str(type)
             //     << "] ["     << setw(14) << base_type_conv::str2dbl(base_type_conv::trim(_line.substr(idx,14))) * it->second
             << "] [" << setw(14) << valdbl * it->second
             << "] " << _epoch.str_ymdhms()
             << " " << _epoch.dsec()
             << " dsize:" << obs->size()
             << endl;
        std::cout.flush();
#endif

        // only for C,L,D, but not SNR observations !!
        if (!(GOBS(type) >= 300 && GOBS(type) < 400) && // signal-to-noise ratio
            !(GOBS(type) >= 1300 && GOBS(type) < 1400))
        { // signal-to-noise ratio

            // read LLI
            if (_line.length() - 1 > (idx + 14))
            {
                int lli = base_type_conv::str2int(_line.substr(idx + 14, 1));
                // add by jqwu
                // RINEX305: Bit 2 set: BOC-tracking of an MBOC-modulated signal (may suffer from increased noise).
                if (lli > 3)
                    lli -= 4;
                obs->addlli(type, lli);
            }
            else
            {
                obs->addlli(type, 0);
            }

            // only for phase observations !
            if (GOBS(type) >= 100 && GOBS(type) < 200)
            {

                // read SNR type instead of original OBS type (always 3-char in G-Nut = RNX2.x 2-char + space)
                GOBS snrtype = pha2snr(type);
                // ---------------------------------
                // slower alternative!
                // GOBS snrtype = str2gobs( "S" + gobs2str(type).substr(1,2));
                // ---------------------------------

                // read SNR observation and convert flag to
                if (/* _line.length() -1 >= (idx + 14+2)*/ _line.length() - 1 >= (idx + 14 + 1))
                { // 14.3 + LLI + SSI  glfeng

                    string snr = base_type_conv::trim(_line.substr(idx + 14 + 1, 1)); // 14.3 + LLI + SSI  glfeng , ok
                    int i = base_type_conv::str2int(snr);

                    if (snr == "")
                        obs->addobs(snrtype, 0.0);
                    else if (i == 1)
                        obs->addobs(snrtype, 6.0);
                    else if (i == 2)
                        obs->addobs(snrtype, 15.0);
                    else if (i == 3)
                        obs->addobs(snrtype, 20.0);
                    else if (i == 4)
                        obs->addobs(snrtype, 27.0);
                    else if (i == 5)
                        obs->addobs(snrtype, 33.0);
                    else if (i == 6)
                        obs->addobs(snrtype, 39.0);
                    else if (i == 7)
                        obs->addobs(snrtype, 45.0);
                    else if (i == 8)
                        obs->addobs(snrtype, 50.0);
                    else if (i == 9)
                        obs->addobs(snrtype, 60.0);
                }
                else
                {
                    obs->addobs(snrtype, 0.0);
                }
            }
        }

        return 1;
    }

    // fill header information
    // ----------
    int gnss_coder_rinexo2::_fill_head()
    {

        int cnt = 0;

        map<string, base_data *>::iterator itDAT = _data.begin();
        while (itDAT != _data.end())
        {

            if (itDAT->second->id_type() == base_data::ALLOBJ)
            {
                gnss_all_obj *all_obj = (gnss_all_obj *)itDAT->second;

                base_time epo = _rnxhdr.first();

                shared_ptr<gnss_data_rec> old_obj, new_obj;
                old_obj = std::dynamic_pointer_cast<gnss_data_rec>(all_obj->obj(_site));
                new_obj = make_shared<gnss_data_rec>(_spdlog);
                new_obj->id(_site);   // uniq ident
                new_obj->name(_site); // prefer before MARKER_NAME which might be incorrect
                new_obj->spdlog(_spdlog);
                new_obj->addhdr(_rnxhdr, epo, _fname); // add full rinex header
                //add by xiongyun (to avoid rectype null)
                if (_rnxhdr.rectype() == "" && old_obj != 0)
                {
                    gnss_data_rec::hwa_map_rec maprec = old_obj->get_map_rec();
                    _rnxhdr.rectype(maprec.begin()->second);
                }
                new_obj->fill_rnxhdr(_rnxhdr); // set object by rinex header

                // complete from RNX
                if (old_obj == 0)
                {
                    all_obj->add(new_obj); // obj has been set -> add obj to gallobj

                    if (_spdlog)
                        SPDLOG_LOGGER_DEBUG(_spdlog, "Object created, using RINEX header: " + new_obj->id() + epo.str_ymdhms(" "));
                }
                else
                {
                    old_obj->compare(new_obj, epo, "RINEX HEADER");
                }
            }

            // add by glfeng ----------------------------------------------------
            else if (itDAT->second->id_type() == base_data::ALLOBS)
            {
                ((gnss_all_obs *)itDAT->second)->addsitecrd(_site, _rnxhdr.aprxyz());
            }
            // ------------------------------------------------------------------

            cnt++;
            itDAT++;
        }

        //std::cout << "End fill_head\n";

        return cnt;
    }

    // fill observation data structure
    // ----------
    int gnss_coder_rinexo2::_fill_data()
    {

        int cnt = 0;
        map<string, base_data *>::iterator itDAT = _data.begin();
        // adding channel canal to gnss_data_obs_manager from gallobj
        // !!!! must be before adding obs data to gnss_all_obs
        while (itDAT != _data.end())
        {
            if (itDAT->second->id_type() != base_data::ALLOBJ)
            {
                itDAT++;
                continue;
            }
            vector<hwa_spt_obsmanager>::iterator itOBS = _vobs.begin();
            while (itOBS != _vobs.end())
            {
                gnss_all_obj *all_obj = (gnss_all_obj *)itDAT->second;
                shared_ptr<gnss_data_trn> one_obj = std::dynamic_pointer_cast<gnss_data_trn>(all_obj->obj((*itOBS)->sat()));
                if (one_obj != 0 && one_obj->id_type() == base_data::TRN)
                {
                    int ch = one_obj->channel();
                    (*itOBS)->channel(ch);
                }
                itOBS++;
            }
            itDAT++;
        }

        // fill gnss_data_obs_manager
        itDAT = _data.begin();
        while (itDAT != _data.end())
        {

            if (itDAT->second->id_type() != base_data::ALLOBS)
            {
                itDAT++;
                continue;
            }

            vector<hwa_spt_obsmanager>::const_iterator itOBS = _vobs.begin();
            while (itOBS != _vobs.end())
            {

                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, (*itOBS)->sat() + " obs filled " + (*itOBS)->epoch().str_ymdhms() + " " + (*itOBS)->site());

                ((gnss_all_obs *)itDAT->second)->addobs(*itOBS);

                itOBS++;
            }
            cnt++;
            itDAT++;
        }

        return cnt;
    }

    // correct reading stop
    // ----------
    int gnss_coder_rinexo2::_check_head()
    {

        Triple zero(0.0, 0.0, 0.0);

        if (_rnxhdr.rnxsys() == ' ')
        {
            mesg(GERROR, "RINEX SYS not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.rnxver().empty())
        {
            mesg(GERROR, "RINEX VERS not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.program().empty() &&
            _rnxhdr.program().empty())
        {
            mesg(GERROR, "PGM/RUNBY not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.observer().empty() &&
            _rnxhdr.agency().empty())
        {
            mesg(GERROR, "OBSERVER/AGENCY not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.markname().empty())
        {
            mesg(GERROR, "MARKER NAME not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.recnumb().empty())
        {
            mesg(GERROR, "REC NUMB not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.rectype().empty())
        {
            mesg(GERROR, "REC TYPE not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.recvers().empty())
        {
            mesg(GERROR, "REC VERS not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.antnumb().empty())
        {
            mesg(GERROR, "ANT NUMB not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.anttype().empty())
        {
            mesg(GERROR, "ANT TYPE not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.mapobs().size() == 0)
        {
            mesg(GERROR, "OBS TYPE not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.aprxyz() == zero)
        {
            mesg(GERROR, "APR XYZ not available!");
            _irc++;
        } // mandatory
        if (_rnxhdr.first() == FIRST_TIME)
        {
            mesg(GERROR, "FIRST TIME not available!");
            _irc++;
        } // mandatory

        if (_rnxhdr.interval() <= 0)
        {
            mesg(GWARNING, "Warning: INTERVAL not available!");
        } // optional --> warning
        if (_rnxhdr.marknumb().empty())
        {
            mesg(GWARNING, "Warning: MARKER NUMB empty field!");
        } // optional --> warning

        return _irc;
    }

    // correct reading stop
    // ----------
    int gnss_coder_rinexo2::_null_log(const string &sat, const string &obstype)
    {

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, _epoch.str_ymdhms(sat + ": obs  null ") + " " + obstype);
        return 0;
    }

    // correct reading stop
    // ----------
    int gnss_coder_rinexo2::_stop_read()
    {
        _tmpsize = 0;
        _complete = false;
        return -1;
    }

    // fix band
    // ----------
    int gnss_coder_rinexo2::_fix_band(string sys, string &go)
    {

        GSYS gsys = (sys.length() < 3) ? gnss_sys::char2gsys(sys[0]) : gnss_sys::str2gsys(sys);

#ifdef DEBUG
        cerr << " sys : " << sys << " " << gnss_sys::gsys2str(gsys)
             << " gobs: " << go << "  GO : " << go.substr(1, 1)
             << " " << str2gobsband(go)
             << " len:" << sys.length()
             << endl;
#endif

        // EXCEPTIONs FOR EXTENDED RINEX 2.11 (L7 etc)
        switch (gsys)
        {
        case GLO:
            if (_version <= "2.12" && go[1] == '7')
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "Warning: extended RINEX 2.11 format (GLO L7 skipped)");
                return 0;
            };
            break;

        case GPS:
            if (_version <= "2.12" && go[1] == '7')
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "Warning: extended RINEX 2.11 format (GPS L7 skipped)");
                return 0;
            };
            break;
        case BDS:
            if (_version <= "2.12")
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "Warning: extended RINEX 2.11 format (BDS skipped)");
                return 0;
            };
            break;
        case QZS:
            if (_version <= "2.12")
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "Warning: extended RINEX 2.11 format (QZS skipped)");
                return 0;
            };
            break;
        case IRN:
            if (_version <= "2.12")
            {
                if (_spdlog)
                    SPDLOG_LOGGER_WARN(_spdlog, "Warning: extended RINEX 2.11 format (IRN skipped)");
                return 0;
            };
            break;

        default:
            return 1;
        }

        return 1;
    }

} // namespace
