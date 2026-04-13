#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "hwa_base_data.h"

namespace hwa_base
{
    base_data::base_data() : _type(NONE),
        _group(GRP_NONE)
    {
    }

    // constructor
    // ----------
    base_data::base_data(base_log spdlog) : _type(NONE),
        _group(GRP_NONE)
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
        _gnote = nullptr;
    }

    // copy constructor
    // ----------
    base_data::base_data(const base_data& data)
    {
        _spdlog = data.spdlog();
        _gnote = data.gnote();
        _type = data.id_type();
        _group = data.id_group();
    }

    // destructor
    // ----------
    base_data::~base_data()
    {
    }

    // assignment operator
    // ----------
    base_data& base_data::operator=(const base_data& data)
    {
        _spdlog = data.spdlog();
        _gnote = data.gnote();
        _type = data.id_type();
        _group = data.id_group();
        return *this;
    }

    // std::set data type
    // ----------
    int base_data::id_type(const ID_TYPE& t)
    {
        unsigned int last = LAST;
        for (unsigned int i = 0; i < last; ++i)
        {
            if (t == ID_TYPE(i))
            {
                return _type = t;
            }
        }

        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, "Unknown data id. Restd::set to nullptr");
        _type = base_data::NONE;
        return 0;
    }

    // std::set group type
    // ----------
    int base_data::id_group(const ID_GROUP& g)
    {
        unsigned int last = GRP_LAST;
        for (unsigned int i = 0; i < last; ++i)
        {
            if (g == ID_GROUP(i))
            {
                return _group = g;
            }
        }
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, "Unknown group type. Restd::set to nullptr");
        _group = base_data::GRP_NONE;
        return 0;
    }

    // ID_TYPE to std::string
    // -----------------
    std::string base_data::type2str(ID_TYPE type)
    {
        std::string str = "";

        switch (type)
        {
        case NONE:
            str = "NONE";
            break;
        case OBS:
            str = "OBS";
            break;
        case OBSGNSS:
            str = "OBSGNSS";
            break;

        case QCDATA:
            str = "QCDATA";
            break;
        case QCPROD:
            str = "QCPROD";
            break;

        case EPH:
            str = "EPH";
            break;
            // case  EPHALL  :  str = "EPHALL";  break;
        case EPHGPS:
            str = "EPHGPS";
            break;
        case EPHGLO:
            str = "EPHGLO";
            break;
        case EPHGAL:
            str = "EPHGAL";
            break;
        case EPHQZS:
            str = "EPHQZS";
            break;
        case EPHBDS:
            str = "EPHBDS";
            break;
        case EPHIRN:
            str = "EPHIRN";
            break;
        case EPHSBS:
            str = "EPHSBS";
            break;
        case EPHPREC:
            str = "EPHPREC";
            break;
        case EPHRTCM:
            str = "EPHRTCM";
            break;

        case ALLGIO:
            str = "ALLGIO";
            break;
        case ALLNAV:
            str = "ALLNAV";
            break;
        case ALLPREC:
            str = "ALLPREC";
            break;
        case ALLRTCM:
            str = "ALLRTCM";
            break;
        case ALLOBS:
            str = "ALLOBS";
            break;
        case ALLOBJ:
            str = "ALLOBJ";
            break;
        case ALLPCV:
            str = "ALLPCV";
            break;
        case ALLOTL:
            str = "ALLOTL";
            break;

        case STRBUFF:
            str = "STRBUFF";
            break;
        case POS:
            str = "POS";
            break;
        case POST:
            str = "POST";
            break;
        case MET:
            str = "MET";
            break;
        case ION:
            str = "ION";
            break;
        case TRP:
            str = "TRP";
            break;
        case TRPSLT:
            str = "TRPSLT";
            break;

        case PCV:
            str = "PCV";
            break;
        case OTL:
            str = "OTL";
            break;
        case BIAS:
            str = "BIAS";
            break;
        case ERP:
            str = "ERP";
            break;

        case SOL:
            str = "SOL";
            break;
        case IMUDATA:
            str = "IMUDATA";
            break;
        case ALLIMUDATA:
            str = "ALLIMUDATA";
            break;
        case UWBDATA:
            str = "UWBDATA";
            break;
        case TAGINFO:
            str = "TAGDATA";
            break;
        case RSSI:
            str = "RSSIDATA";
            break;
        case RSSIMAP:
            str = "RSSIMAP";
            break;
        case ODODATA:
            str = "ODODATA";
            break;
        case LCI_POS:
            str = "LCI_POS";
            break;
        case CAMDATA:
            str = "CAMDATA";
            break;
        case LAST:
            str = "UNDEF";
            break;
        default:
            str = "UNDEF";
        }
        return str;
    }

    // get data type
    // ----------
    std::string base_data::str_type() const
    {
        std::string type = type2str(_type);

        return type;
    }

    // get data type
    // ----------
    std::string base_data::str_group() const
    {
        std::string group;
        switch (_group)
        {
        case GRP_NONE:
            group = "GRP_NONE";
            break;
        case GRP_obsERV:
            group = "GRP_obsERV";
            break;
        case GRP_QC:
            group = "GRP_QC";
            break;
        case GRP_EPHEM:
            group = "GRP_EPHEM";
            break;
        case GRP_prodUCT:
            group = "GRP_prodUCT";
            break;
        case GRP_model:
            group = "GRP_model";
            break;
        case GRP_SOLUT:
            group = "GRP_SOLUT";
            break;
        case GRP_GRID:
            group = "GRP_GRID";
            break;
        case GRP_GIO:
            group = "GRP_GIO";
            break;
        case GRP_LAST:
            group = "GRP_UNDEF";
            break;
        case GRP_IMU:
            group = "GRP_IMU";
            break;
        default:
            group = "GRP_UNDEF";
        }
        return group;
    }

    void base_data::spdlog(base_log spdlog)
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
    }
    // show function
    // ----------
    std::string base_data::show(int verb)
    {

        std::ostringstream os("");

        //  if( (!_log) || ( _log  && _log->verb() >= verb )){
        os << "gdata: nothing to show \n";
        //  }

        //  std::cout << _moni_id << ":" << std::endl;
        //  char buff[200 + 1] = "";
        //  sprintf( buff, "%i\n", 2222222222 );
        //  sprintf( buff, "%s: %i\n", _moni_id, _data ); // NEFUNGUJE
        //  buff[200+1] = '\0';

        return os.str();
    }

} // namespace
