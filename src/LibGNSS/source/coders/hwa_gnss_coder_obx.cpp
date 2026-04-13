#include "hwa_gnss_coder_obx.h"

namespace hwa_gnss
{
    hwa_gnss::gnss_data_obx::gnss_data_obx(set_base * s, std::string version, int sz): 
        base_coder(s, version, sz), gnss_base_coder(s,version,sz)
    {
    }

    gnss_data_obx::~gnss_data_obx()
    {
    }

    int gnss_data_obx::decode_head(char * buff, int sz, std::vector<std::string>& errmsg)
    {
        return 0;
    }

    int gnss_data_obx::decode_data(char * buff, int sz, int & cnt, std::vector<std::string>& errmsg)
    {
        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        };
#ifdef DEBUG
        std::cout << " BUFFER : \n"
            << _buffer << "\n size = " << sz << " END OF BUFFER \n\n";
        std::cout.flush();
#endif

        int consume = 0;
        int tmpsize = 0;
        std::string line;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(line)) >= 0)
            {
                consume += tmpsize;

                if (line.find("-EPHEMERIS/DATA") != std::string::npos)
                {
                    break;
                }

                if (line.find("##") != std::string::npos && line[0] == '#' && line[1] == '#')
                {
                    //## 2022 05 07 00 00  0.000000000000 113
                    std::stringstream ss(line);
                    int year = 0;
                    int mon = 0;
                    int day = 0;
                    int hour = 0;
                    int min = 0;
                    double sec = 0.0;
                    int sat_num = 0;
                    ss >> year >> mon >> day >> hour >> min >> sec >> sat_num;
                    if (!ss.fail())
                    {
                        // one epoch obx data
                        gnss_data_obx_epoch obx_epoch;

                        // read obx epoch
                        base_time epo;

                        epo.from_ymdhms(year, mon, day, hour, min, sec);
                        obx_epoch.addTime(epo);

                        // read obx record
                        for (int i = 0; i < sat_num; i++)
                        {
                            if ((tmpsize = gnss_base_coder::_getline(line)) < 0) break;
                            consume += tmpsize;

                            gnss_data_obx_record obx_record;
                            std::stringstream ss(line);
                            
                            //ATT G01              4  0.0994688219230238  0.0219771728046990  0.2870720884813037 - 0.9524770723517395
                            ss >> obx_record.rec
                                >> obx_record.id
                                >> obx_record.num
                                >> obx_record.q0
                                >> obx_record.q1
                                >> obx_record.q2
                                >> obx_record.q3;

                            if (!ss.fail())
                            {
                                obx_record.isValid = true;
                                obx_record.epo = epo;
                                obx_epoch.addObxRecord(obx_record);
                            }
                        }

                        std::map<std::string, base_data *>::iterator it = _data.begin();
                        while (it != _data.end())
                        {
                            if (it->second->id_type() == base_data::ION)
                            {
                                ((gnss_all_obx *)it->second)->addObx(epo, obx_epoch);
                            }
                            it++;
                        }

                        // read one epoch, break;
                        gnss_base_coder::_consume(tmpsize);
                        break;
                    }
                }
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ion::decode_data throw exception");

            return -1;
        }
    }

    int gnss_data_obx::encode_head(char * buff, int sz, std::vector<std::string>& errmsg)
    {
        _mutex.lock();
        try
        {
            if (_ss_position == 0)
            {
                _ss << "%=ORBEX  0.09                                                                                          " << std::endl;
                _ss << "%%                                                                                                     " << std::endl;
                _ss << "+FILE/DESCRIPTION                                                                                      " << std::endl;
                _ss << " DESCRIPTION         Satellite attitude base_quaternions of WHU GNSS rapid solution                         " << std::endl;
                _ss << " EPOCH_INTERVAL      30.000                                                                            " << std::endl;
                _ss << " COORD_SYSTEM        IGS14                                                                             " << std::endl;
                _ss << " FRAME_TYPE          ECEF                                                                              " << std::endl;
                _ss << " LIST_OF_REC_TYPES   ATT                                                                               " << std::endl;
                _ss << "-FILE/DESCRIPTION                                                                                      " << std::endl;
                _ss << "+EPHEMERIS/DATA                                                                                        " << std::endl;
                _ss << "*ATT RECORDS: TRANSFORMATION FROM TERRESTRIAL FRAME COORDINATES (T) TO SAT. BODY FRAME ONES (B) SUCH AS" << std::endl;
                _ss << "*                                 (0,B) = q.(0,T).trans(q)                                             " << std::endl;
                _ss << "*REC ID_              N ___q0_(scalar)_____ ____q1__x__________ ____q2__y__________ ____q3__z__________" << std::endl;
            }
            int size = _fill_buffer(buff, sz);
            _mutex.unlock();
            return size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambinp::encode_head throw exception");
            return -1;
        }
    }

    int gnss_data_obx::encode_data(char * buff, int sz, int & cnt, std::vector<std::string>& errmsg)
    {
        _mutex.lock();
        int all_size = 0;
        try
        {
            if (_ss_position == 0)
            {
                //get data from _data
                gnss_all_obx::hwa_map_time_obx obx_data;
                auto it = _data.begin();
                for (it = _data.begin(); it != _data.end(); ++it)
                {
                    if (it->second->id_type() == base_data::ALLOBX)
                    {
                        obx_data = dynamic_cast<gnss_all_obx*>(it->second)->getObxData();
                    }
                }

                // encode
                for (auto itepo = obx_data.begin(); itepo != obx_data.end(); ++itepo)
                {
                    const gnss_data_obx_epoch& obx_epoch = itepo->second;
                    const base_time& epo = obx_epoch.getTime();
                    //## 2022 05 06 00 00  0.000000000000 113
                    _ss << "## "
                        << std::setw(04) << epo.year() << " "
                        << std::setw(02) << epo.str_mon() << " "
                        << std::setw(02) << epo.str_day() << " "
                        << std::setw(02) << epo.str_hour() << " "
                        << std::setw(02) << epo.str_min() << " "
                        << std::setw(15) << std::setprecision(12) << epo.secs() << " "
                        << std::setw(03) << obx_epoch.getSatNum() << std::endl;

                    for (auto itsat = obx_epoch.obx_data.begin(); itsat != obx_epoch.obx_data.end(); itsat++)
                    {
                        const gnss_data_obx_record& obx_record = itsat->second;
                        //ATT G01              4 - 0.1013893836011327 - 0.0344874570098873 - 0.2852795205187917  0.9524423359839387
                        _ss << std::setw(04) << obx_record.rec
                            << std::setw(16) << obx_record.id
                            << std::setw(02) << obx_record.num
                            << std::setw(19) << std::setprecision(16) << obx_record.q0
                            << std::setw(19) << std::setprecision(16) << obx_record.q1
                            << std::setw(19) << std::setprecision(16) << obx_record.q2
                            << std::setw(19) << std::setprecision(16) << obx_record.q3 
                            << std::endl;
                    }
                }
                _ss << "-EPHEMERIS / DATA" << std::endl;
            }

            int size = _fill_buffer(buff, sz);
            all_size += size;
            _mutex.unlock();
            return all_size;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "gnss_coder_ambupd::encode_data throw exception");
            return -1;
        }
    }

}
