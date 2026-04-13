#include <sstream>
#include <stdlib.h>
#include <algorithm>
#include "hwa_gnss_all_pcv.h"

namespace hwa_gnss
{
    gnss_all_pcv::gnss_all_pcv()
        : base_data(),
          _overwrite(false)
    {
        id_type(base_data::ALLPCV);
    }

    // destructor
    // ----------
    gnss_all_pcv::gnss_all_pcv(base_log spdlog)
        : base_data(spdlog),
          //if( _close_with_warning )
          //{

          _overwrite(false)
    {
        id_type(base_data::ALLPCV);
    }

    // get PCO (phase center offsets)
    // zen/azi [rad]
    // ----------
    /*
double gnss_all_pcv::pco( std::string ant, std::string ser, const base_time& t, double zen, double azi, GFRQ freq)
{
  boost::mutex::scoped_lock lock(_mutex);
  std::shared_ptr<gnss_data_pcv> tmp = _find( ant, ser, t );
  if( tmp == _pcvnull ){    
     return 0.0;
   }
   return tmp->pco( zen, azi, freq );
} 
 */

    // get PCV (phase center variations)
    // zen/azi [rad]
    // ----------
    gnss_all_pcv::~gnss_all_pcv()
    {
    }

    // return position (virtual)
    // ----------
    int gnss_all_pcv::addpcv(std::shared_ptr<gnss_data_pcv> pcv)
    {
        std::string key = pcv->pcvkey();
        std::string typ = pcv->pcvtyp();
        base_time beg = pcv->beg();
        base_time end = pcv->end();

        pcv->gnote(_gnote); // TRANSFER GNOTE

        if (_mappcv.find(key) == _mappcv.end() ||
            _mappcv[key].find(typ) == _mappcv[key].end() ||
            _mappcv[key][typ].find(beg) == _mappcv[key][typ].end())
        {

            // new instance
            _mappcv[key][typ][beg] = pcv;
#ifdef DEBUG
            std::cout << "NEW: " << key << " " << typ << " " << std::endl;
#endif
        }
        else
        {

            // exists, but overwrite ?
            if (_overwrite)
            {
#ifdef DEBUG
                std::cout << "DEL: " << key << " " << typ << " " << std::endl;
#endif
                _mappcv[key][typ][beg] = pcv;
#ifdef DEBUG
                std::cout << "RENEW: " << key << " " << typ << " " << std::endl;
#endif
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "already exists, not overwritten !\n");
#ifdef DEBUG
                std::cout << "OLD: " << key << " " << typ << " " << std::endl;
#endif
            }
        }
        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "add PCV " + key + " " + typ + beg.str_ymdhms(" ") + end.str_ymdhms(" "));
        return 1;
    }

    // find PCO+PCV model
    // ----------
    std::shared_ptr<gnss_data_pcv> gnss_all_pcv::gpcv(const std::string &ant, const std::string &ser, const base_time &t)
    {
        std::shared_ptr<gnss_data_pcv> gpcv = _find(ant, ser, t);
        return gpcv;
    }

    // return list of available antennas
    // ----------
    std::vector<std::string> gnss_all_pcv::antennas()
    {
        std::vector<std::string> all_ant;

        for (auto itant = _mappcv.begin(); itant != _mappcv.end(); ++itant)
        {

            std::string ant = itant->first;

            std::map<std::string, hwa_map_tiv>::iterator itnum;
            for (itnum = _mappcv[ant].begin(); itnum != _mappcv[ant].end(); ++itnum)
            {

                std::string num = itnum->first;

                all_ant.push_back(ant + " " + num);
            }
        }
        return all_ant;
    }

    // return list of available satellites
    // ----------
    std::shared_ptr<gnss_data_pcv> gnss_all_pcv::find(const std::string &ant, const std::string &ser, const base_time &t)
    {
        return _find(ant, ser, t);
    }

    // return list of available satellites
    // ----------
    std::shared_ptr<gnss_data_pcv> gnss_all_pcv::_find(const std::string &ant_name, const std::string &ser_num, const base_time &t)
    {
        std::string ant(ant_name);
        std::string ser(ser_num);
        std::string ant0 = ant; // DEFAULT (ANTENNA + RADOME)
        std::string ser0 = "";  // DEFAULT (TYPE CALLIBRATION)

#ifdef DEBUG
        std::cout << "gallpcv A : ant [" + ant + "]  ser [" + ser + "]  ser0 [" + ser0 + "]\n";
#endif

        // antenna not found in the list
        if (_mappcv.find(ant) == _mappcv.end())
        {

            // try alternative name for receiver antenna (REPLACE RADOME TO NONE!)
            if (ant.size() >= 19)
                ant0 = ant.replace(16, 4, "NONE"); // REPLACE DEFAULT ANT-NAME

            if (_mappcv.find(ant0) == _mappcv.end())
            {
                return _pcvnull;
            }
        }

        // individual antenna (serial number used, '*'=ANY!)
        std::map<std::string, hwa_map_tiv>::iterator itser;
        if (ser.find("*") == std::string::npos)
        { // SPECIAL SERIAL DEFINED (OR TYPE CALLIBRATION "")
            for (itser = _mappcv[ant0].begin(); itser != _mappcv[ant0].end(); ++itser)
            {

                // antenna (serial number) found in the list!
                if (_mappcv[ant0].find(ser) != _mappcv[ant0].end())
                {
                    ser0 = ser; // REPLACE DEFAULT SER-NUMB (INDIVIDUAL CALLIBRATION)
                    break;
                }
            }
        }

        // try to find the approximation (ser0=DEFAULT, ser=NOT FOUND)
        if (ser0 == "" && _mappcv[ant].find(ser) == _mappcv[ant].end())
        {
            ser = ""; // REPLACE ALWAYS WITH TYPE CALLIBRATION !!!
        }

        // check/return individual and check time validity
        if (ser0 == ser)
        { // SERIAL NUMBER FOUND!
            hwa_map_tiv::iterator it = _mappcv[ant][ser0].begin();
            while (it != _mappcv[ant][ser0].end())
            {
                if (((it->second)->beg() < t || (it->second)->beg() == t) &&
                    (t < (it->second)->end() || t == (it->second)->end()))
                {
                    return it->second;
                }
                it++;
            }
        }

#ifdef DEBUG
        std::cout << "gallpcv F : ant [" + ant + "]  ser [" + ser + "]  ser0 [" + ser0 + "]\n";
#endif

        return _pcvnull;
    }

} // namespace
