#include "hwa_ins_coder.h"
#include "hwa_ins_data.h"
#include "hwa_base_string.h"

hwa_ins::ins_coder::ins_coder(set_base* s, std::string version, int sz): base_coder(s, sz),
_ts(0.005),
_GyroUnit(DPS),
_AcceUnit(MPS2),
_complete(false),
_order("garfu")
{
    is_first = true;
    _ts = dynamic_cast<set_ins*>(s)->ts();
    _freq = dynamic_cast<set_ins*>(s)->freq();
    _GyroUnit = dynamic_cast<set_ins*>(s)->GyroUnit();
    _AcceUnit = dynamic_cast<set_ins*>(s)->AcceUnit();
    _MagUnit = dynamic_cast<set_ins*>(s)->MagUnit(); //add wh
    _order = dynamic_cast<set_ins*>(s)->order();
    t_beg = dynamic_cast<set_ins*>(_set)->start();
    t_end = dynamic_cast<set_ins*>(_set)->end();
}

int hwa_ins::ins_coder::decode_head(char* buff, int sz, std::vector<std::string>& errmsg)
{
    base_coder::_add2buffer(buff, sz);
    return -1;
}

int hwa_ins::ins_coder::decode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg)
{

    if (_complete || base_coder::_add2buffer(buff, sz) == 0) {
        return 0; 
    };

    int tmpsize = 0;

    bool isbinary = false;
    //if (_order == "starneto" || _order == "imr") isbinary = true;
    if (_order == "imr") isbinary = true;
    if (!isbinary)
    {
        std::string line;
        while (!_complete && (tmpsize = base_coder::_getline(line, 0)) >= 0)
        {
            if ((_order == "unicore" || _order == "great") && line[0] != '#')
            {
                base_coder::_consume(tmpsize);
                continue;
            }
            if ((line[0] == '%' && _order != "imr_fsas") || (line[0] == '#' && _order != "unicore" && _order != "great") || line[0] == 'T')
            {
                base_coder::_consume(tmpsize);
                continue;
            }
            for (int i = 0; i < line.size(); i++)
            {
                if (line[i] == ',' || line[i] == '*' || line[i] == ';')line[i] = ' ';
            }
            double t, gx, gy, gz, ax, ay, az, mx, my, mz;
            Triple g_tmp, a_tmp, m_tmp;
            Triple wtmp, vtmp, mtmp;
            std::stringstream ss(line);
            if (_order[0] == 'g' && _order[1] == 'a') // g->a
            {
                ss >> t >> gx >> gy >> gz >> ax >> ay >> az;
                g_tmp = Triple(gx, gy, gz); a_tmp = Triple(ax, ay, az);
                ///added wh
                if (_order[2] == 'm') // for mag data
                {
                    ss >> mx >> my >> mz;
                    m_tmp = Triple(mx, my, mz);
                }
            }
            else if (_order[0] == 'a' && _order[1] == 'g') // a->g
            {
                ss >> t >> ax >> ay >> az >> gx >> gy >> gz;
                g_tmp = Triple(gx, gy, gz); a_tmp = Triple(ax, ay, az);
                ///added wh
                if (_order[2] == 'm') // for mag data
                {
                    ss >> mx >> my >> mz;
                    m_tmp = Triple(mx, my, mz);
                }
            }
            else if (_order == "unicore")
            {
                std::vector<double> v;
                if (this->decode_unicore(line, t, v) > 0 && line.size() < 140)  //usually one line's size<140 --sbs 1018
                {
                    az = v[0]; ay = -v[1]; ax = v[2];
                    gz = v[3]; gy = -v[4]; gx = v[5];
                    g_tmp = Triple(gx, gy, gz); a_tmp = Triple(ax, ay, az);
                    wtmp = g_tmp; vtmp = a_tmp;  //--sbs 1007
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "decode unicore data failed!");
                    base_coder::_consume(tmpsize);
                    continue;
                }
            }
            else if (_order == "great")
            {
                if (line.find("CAMERA") != std::string::npos)
                {
                    std::cerr << "Please reconnect GREAT board." << std::endl;
                    base_time::gmsleep(2000); exit(0);
                }
                std::vector<double> v;
                if (this->decode_great(line, t, v) > 0 && line.size() < 140)  //usually one line's size<140 --sbs 1018
                {
                    az = v[0]; ay = -v[1]; ax = v[2];
                    gz = v[3]; gy = -v[4]; gx = v[5];
                    //g_tmp = Triple(gx, gy, gz); a_tmp = Triple(ax, ay, az);
                    g_tmp = Triple(gy, gx, gz); a_tmp = Triple(ay, ax, az); //qzy
                    wtmp = g_tmp; vtmp = a_tmp;  //--sbs 1007
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "decode unicore data failed!");
                    base_coder::_consume(tmpsize);
                    continue;
                }
            }
            else if (_order == "imr_fsas")
            {
                std::vector<double> v;
                if (this->decode_imr_fsas(line, t, v) > 0 && line.size() < 120)
                {
                    az = v[0]; ay = -v[1]; ax = v[2];
                    gz = v[3]; gy = -v[4]; gx = v[5];
                    g_tmp = Triple(gx, gy, gz); a_tmp = Triple(ax, ay, az);
                    wtmp = g_tmp * glv.sec; vtmp = a_tmp;
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "decode unicore data failed!");
                    base_coder::_consume(tmpsize);
                    continue;
                }
            }
            else if (_order == "starneto")
            {
                std::vector<double> v;
                if (this->decode_starneto(line, t, v) > 0)//&& line.size() < 120)
                {
                    ax = v[3]; ay = v[4]; az = v[5];
                    gx = v[0]; gy = v[1]; gz = v[2];
                    g_tmp = Triple(gx, gy, gz); a_tmp = Triple(ax, ay, az);
                    wtmp = g_tmp; vtmp = a_tmp;
                }
                else
                {
                    if (_spdlog)
                        SPDLOG_LOGGER_ERROR(_spdlog, "decode unicore data failed!");
                    base_coder::_consume(tmpsize);
                    continue;
                }
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "The imu data format is not exist!");
            }

            if (t < t_beg)
            {
                int _distance = (t_beg - t) / _ts;
                tmpsize = std::max((_distance - 50), 1) * tmpsize;
                base_coder::_ex_consume(tmpsize);
                continue;
            }
            if (t > t_end)
                _complete = true;

            _tt = t;

            ///<-- modified by wh  substr(2,3) -> substr(_order.size()-3,3) ;   for read mag data
            if (_order.substr(_order.size() - 3, 3) == "rfu")  // right -> forward -> up
            {
                wtmp = g_tmp; vtmp = a_tmp;

                if (_order[2] == 'm')
                {
                    mtmp = m_tmp;
                }
            }
            else if (_order.substr(_order.size() - 3, 3) == "flu") // forward -> left -> up
            {
                SO3 Rotation; Rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
                wtmp = Rotation * g_tmp; vtmp = Rotation * a_tmp;
                if (_order[2] == 'm')
                {
                    mtmp = Rotation * m_tmp;
                }
            }
            else if (_order.substr(_order.size() - 3, 3) == "frd") // forward -> right -> down
            {
                SO3 Rotation; Rotation << 0, 1, 0, 1, 0, 0, 0, 0, -1;
                wtmp = Rotation * g_tmp; vtmp = Rotation * a_tmp;
                if (_order[2] == 'm')
                {
                    mtmp = Rotation * m_tmp;
                }
            }
            else if (_order.substr(_order.size() - 3, 3) == "rbd") // right -> behind -> down
            {
                SO3 Rotation; Rotation << 1, 0, 0, 0, -1, 0, 0, 0, -1;
                wtmp = Rotation * g_tmp; vtmp = Rotation * a_tmp;
                if (_order[2] == 'm')
                {
                    mtmp = Rotation * m_tmp;
                }
            }
            else if (_order.substr(_order.size() - 3, 3) == "lbu") // left -> behind -> up
            {
                SO3 Rotation; Rotation << -1, 0, 0, 0, -1, 0, 0, 0, 1;
                wtmp = Rotation * g_tmp; vtmp = Rotation * a_tmp;
                if (_order[2] == 'm')
                {
                    mtmp = Rotation * m_tmp;
                }
            }

            switch (_GyroUnit)
            {
            case RAD:
                break;
            case DEG:
                wtmp *= glv.deg; break;
            case RPS:
                wtmp *= _ts; break;
            case DPS:
                wtmp *= glv.dps * _ts; //cerr << glv.dps;
                break;
            case RPH:
                wtmp /= glv.hur * _ts; break;
            case DPH:
                wtmp *= glv.dph * _ts; break;
            default:
                break;
            }
            switch (_AcceUnit)
            {
            case MPS:
                break;
            case MPS2:
                vtmp *= _ts; break;
            case DEG:
            case RPS:
            case RPH:
            case DPH:
            case RAD:
            case UNDF:
            default:
                break;
            }

            ///add wh
            switch (_MagUnit)
            {
            case MPS:
                break;
            case MPS2:
                mtmp *= _ts; break;
            case DEG:
            case RPS:
            case RPH:
            case DPH:
            case RAD:
            case UNDF:
            default:
                break;
            }

            std::map<std::string, base_data*>::iterator it = _data.begin();
            while (it != _data.end())
            {
                ((ins_data*)it->second)->set_ts(_ts);
                if (it->second->id_type() == base_data::IMUDATA)
                {
                    ///modified by wh  read mag data
                    if (_order[2] == 'm')
                        ((ins_data*)it->second)->add_IMU(t, wtmp, vtmp, mtmp);
                    else
                        ((ins_data*)it->second)->add_IMU(t, wtmp, vtmp);
                }
                ++it;
            }
            if (ss.fail())
            {
                if (_spdlog)
                    SPDLOG_LOGGER_DEBUG(_spdlog, "imufile", "warning: incorrect IMU data record: " + ss.str());
                base_coder::_consume(tmpsize);
                return -1;
            }
            base_coder::_consume(tmpsize);
            cnt++;
        }
    }
    else
    {
        const char* block;

        // decode all the data in the buffer in one time (instead of using a loop)
        if (!_complete && (tmpsize = base_coder::_getbuffer(block)) >= 0)
        {
            std::vector<std::pair<double, std::vector<double>>> v; // imu data should be RAD, MPS
            int consume_size = -1;

            if (_order == "starneto") consume_size = decode_starneto(block, tmpsize, v);
            else if (_order == "imr") consume_size = -1; /* not implemented yet */

            if (consume_size > 0)
            {
                tmpsize = consume_size;
            }
            else
            {
                if (_spdlog)
                    SPDLOG_LOGGER_ERROR(_spdlog, "decode starneto data failed!");
                base_coder::_consume(tmpsize);
                return -1;
            };

            // add data records to base_data
            if (v.size() > 0 && v.back().first >= dynamic_cast<set_ins*>(_set)->start())
            {
                if (v.front().first > dynamic_cast<set_ins*>(_set)->end() + 5)
                    _complete = true;

                _tt = v.back().first;

                std::map<std::string, base_data*>::iterator it = _data.begin();
                while (it != _data.end())
                {
                    if (it->second->id_type() == base_data::IMUDATA)
                    {
                        ((ins_data*)it->second)->set_ts(_ts);
                        for (int i = 0; i < v.size(); i++)
                        {
                            Triple wtmp, vtmp;
                            for (int j = 0; j < 3; j++) wtmp(j) = v[i].second[j];
                            for (int j = 0; j < 3; j++) vtmp(j) = v[i].second[j + 3];
                            ((ins_data*)it->second)->add_IMU(v[i].first, wtmp, vtmp);
                        }
                    }
                    ++it;
                }
            }
            base_coder::_consume(tmpsize);
            cnt++;
        }
    }
    return 0;
}

int hwa_ins::ins_coder::decode_unicore(const std::string& line, double& t, std::vector<double>& v)
{

    try
    {
        std::vector<std::string> ret;
        split(line, " ", ret);
        if (ret.size() < 22)  return -1;  // incomplete  --sbs

        // int week = stoi(ret[5]);
        t = stod(ret[6]);
        for (int i = 15; i <= 17; i++)
        {
            //std::cout << stod(ret[i]) << " " << ACC_SCALE << endl;
            v.push_back(stod(ret[i]) * UNICORE_ACC_SCALE);
        }
        for (int i = 18; i <= 20; i++)
            v.push_back(stod(ret[i]) * UNICORE_GYRO_SCALE);  //--sbs 1007
        return 1;
    }
    catch (...)
    {
        return -1;
    }
}

int hwa_ins::ins_coder::decode_great(const std::string& line, double& t, std::vector<double>& v)
{

    try
    {
        std::vector<std::string> ret;
        split(line, " ", ret);
        if (ret.size() < 13)  return -1;  // incomplete  --sbs

        // int week = stoi(ret[1]);
        t = stod(ret[2]);

        v.push_back(stod(ret[4]) * hwa_ACC_SCALE);
        v.push_back(stod(ret[5]) * hwa_ACC_SCALE);
        v.push_back(stod(ret[6]) * hwa_ACC_SCALE);
        v.push_back(stod(ret[7]) * hwa_GYRO_SCALE);
        v.push_back(stod(ret[8]) * hwa_GYRO_SCALE);
        v.push_back(stod(ret[9]) * hwa_GYRO_SCALE);

        return 1;
    }
    catch (...)
    {
        return -1;
    }
}

int hwa_ins::ins_coder::decode_imr_fsas(const std::string& line, double& t, std::vector<double>& v)
{

    try
    {
        std::vector<std::string> ret;
        split(line, " ", ret);
        if (ret.size() != 13)  return -1;  // incomplete  --sbs

        // int week = stoi(ret[3]);
        t = stod(ret[4]);
        for (int i = 6; i <= 8; i++)
        {
            //std::cout << stod(ret[i]) << " " << ACC_SCALE << endl;
            v.push_back(stod(ret[i]) * IMR_FSAS_ACCE_SCALE);
        }
        for (int i = 9; i <= 11; i++)
            v.push_back(stod(ret[i]) * IMR_FSAS_GYRO_SCALE);  //--sbs 1007
        return 1;
    }
    catch (...)
    {
        return -1;
    }
}

int hwa_ins::ins_coder::decode_starneto(const std::string& line, double& t, std::vector<double>& v)
{

    try
    {
        std::vector<std::string> ret;
        split(line, " ", ret);
        if (ret.size() != 11)  return -1;  // incomplete  --sbs

        // int week = stoi(ret[1]);
        t = std::stod(ret[2]);

        // double gx,gy,gz,ax,ay,az;


        //         gx *= glv.deg * _ts;
        //         gy *= glv.deg * _ts;
        //         gz *= glv.deg * _ts;
        //         ax *= STARNETO_G * _ts;
        //         ay *= STARNETO_G * _ts;
        //         az *= STARNETO_G * _ts;

        for (int i = 3; i <= 5; i++)
        {
            //std::cout << stod(ret[i]) << " " << ACC_SCALE << endl;
            v.push_back(stod(ret[i]) * glv.deg * _ts);
        }
        for (int i = 6; i <= 8; i++)
            v.push_back(stod(ret[i]) * STARNETO_G * _ts);  //--sbs 1007
        return 1;
    }
    catch (...)
    {
        return -1;
    }
}

int hwa_ins::ins_coder::decode_starneto(const char* block, int sz, std::vector<std::pair<double, std::vector<double>>>& v)
{
    try
    {
        const int MARGIN = 50;
        int consume_size = 0;

        for (int i = 0; i < sz - MARGIN; i++)
        {
            if (block[i] == (char)0xAA && block[i + 1] == (char)0x55 && block[i + 2] == (char)0x05)
            {
                i += 3;
                int byten = 0;
                uint16_t    week;
                uint32_t    sow;
                double      gx;
                double      gy;
                double      gz;
                double      ax;
                double      ay;
                double      az;

                byten = sizeof(week); memcpy(&week, block + i, byten);  i += byten;
                byten = sizeof(sow);  memcpy(&sow, block + i, byten);  i += byten;
                byten = sizeof(gx);   memcpy(&gx, block + i, byten);  i += byten;
                byten = sizeof(gy);   memcpy(&gy, block + i, byten);  i += byten;
                byten = sizeof(gz);   memcpy(&gz, block + i, byten);  i += byten;
                byten = sizeof(ax);   memcpy(&ax, block + i, byten);  i += byten;
                byten = sizeof(ay);   memcpy(&ay, block + i, byten);  i += byten;
                byten = sizeof(az);   memcpy(&az, block + i, byten);  i += byten;

                gx *= glv.deg * _ts;
                gy *= glv.deg * _ts;
                gz *= glv.deg * _ts;
                ax *= STARNETO_G * _ts;
                ay *= STARNETO_G * _ts;
                az *= STARNETO_G * _ts;

                v.push_back(std::make_pair(sow / 1000.0, std::vector<double>({ gx,gy,gz,ax,ay,az })));
            }
            else if (block[i] == (char)0xAA && block[i + 1] == (char)0x44 && block[i + 2] == (char)0x13)
            {
                int byten = 0;
                i += 6;

                uint16_t    week = 0;
                uint32_t    sow = 0;
                uint32_t    week2 = 0;
                double      sow2 = 0;
                int32_t     state = 0;
                int32_t     az = 0;
                int32_t     ay = 0;
                int32_t     ax = 0;
                int32_t     gz = 0;
                int32_t     gy = 0;
                int32_t     gx = 0;
                byten = sizeof(week);  memcpy(&week, block + i, byten); i += byten;
                byten = sizeof(sow);   memcpy(&sow, block + i, byten); i += byten;
                byten = sizeof(week2); memcpy(&week2, block + i, byten); i += byten;
                byten = sizeof(sow2);  memcpy(&sow2, block + i, byten); i += byten;
                byten = sizeof(state); memcpy(&state, block + i, byten); i += byten;
                byten = sizeof(az);    memcpy(&az, block + i, byten); i += byten;
                byten = sizeof(ay);    memcpy(&ay, block + i, byten); i += byten;
                byten = sizeof(ax);    memcpy(&ax, block + i, byten); i += byten;
                byten = sizeof(gz);    memcpy(&gz, block + i, byten); i += byten;
                byten = sizeof(gy);    memcpy(&gy, block + i, byten); i += byten;
                byten = sizeof(gx);    memcpy(&gx, block + i, byten); i += byten;

                double gyro[3] = { gx,-gy,gz };
                double accel[3] = { ax,-ay,az };

                for (int i = 0; i < 3; i++)  gyro[i] *= STARNETO_GYRO_SCALE;
                for (int i = 0; i < 3; i++) accel[i] *= STARNETO_ACC_SCALE;
                v.push_back(std::make_pair(sow / 1000.0, std::vector<double>({ gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2] })));

            }
            consume_size = i;
        }

        return consume_size;
    }
    catch (...)
    {
        return -1;
    }
}

bool hwa_ins::ins_coder::available(const base_time& now)
{
    double t = now.sow() + now.dsec();
    if (_tt > t)return true;
    else
    {
        return false;
    }
}

int hwa_ins::ins_coder::encode_head(char* buff, int sz, std::vector<std::string>& errmsg)
{
    return -1;
}
int hwa_ins::ins_coder::encode_data(char* buff, int sz, int& cnt, std::vector<std::string>& errmsg)
{
    try
    {
        if (_ss_position == 0)
        {
            ins_data* imudata = NULL;
            for (auto it = _data.begin(); it != _data.end(); ++it)
            {
                if (it->second->id_type() == base_data::IMUDATA)
                {
                    imudata = dynamic_cast<ins_data*>(it->second);
                }
            }
            if (imudata)
            {
                while (true)
                {
                    std::vector<Triple> wm;
                    std::vector<Triple> vm;
                    double t = -1.0, ts = -1.0;
                    bool status;
                    imudata->load(wm, vm, t, ts, 1, status); // rfu
                    if (t <= 0)
                        break;
                    Triple g_tmp = wm[0]; // RAD
                    Triple a_tmp = vm[0]; // MPS

                    Triple wtmp, vtmp;

                    if (_order.substr(2, 3) == "rfu")  // right -> forward -> up
                    {
                        wtmp = g_tmp; vtmp = a_tmp;
                    }
                    else if (_order.substr(2, 3) == "flu") // forward -> left -> up
                    {
                        SO3 Rotation; Rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
                        wtmp = Rotation.transpose() * g_tmp; vtmp = Rotation.transpose() * a_tmp;
                    }
                    else if (_order.substr(2, 3) == "frd") // forward -> right -> down
                    {
                        SO3 Rotation; Rotation << 0, 1, 0, 1, 0, 0, 0, 0, -1;
                        wtmp = Rotation.transpose() * g_tmp; vtmp = Rotation.transpose() * a_tmp;
                    }
                    else if (_order.substr(2, 3) == "rbd") // right -> behind -> down
                    {
                        SO3 Rotation; Rotation << 1, 0, 0, 0, -1, 0, 0, 0, -1;
                        wtmp = Rotation.transpose() * g_tmp; vtmp = Rotation.transpose() * a_tmp;
                    }

                    switch (_GyroUnit)
                    {
                    case RAD:
                        break;
                    case DEG:
                        wtmp /= glv.deg; break;
                    case RPS:
                        wtmp /= _ts; break;
                    case DPS:
                        wtmp /= glv.dps * _ts; break;
                    case RPH:
                        wtmp *= glv.hur * _ts; break;
                    case DPH:
                        wtmp /= glv.dph * _ts; break;
                    default:
                        break;
                    }
                    switch (_AcceUnit)
                    {
                    case MPS:
                        break;
                    case MPS2:
                        vtmp /= _ts; break;
                    case RAD:
                    case RPS:
                    case DPS:
                    case RPH:
                    case DPH:
                    case UNDF:
                    default:
                        break;
                    }

                    if (_order[0] == 'g' && _order[1] == 'a') // g->a
                    {
                        _ss << std::setw(15) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << t << " " << std::resetiosflags(std::ios::fixed)
                            << std::setw(20) << std::setprecision(10) << wtmp(0) << " "
                            << std::setw(20) << std::setprecision(10) << wtmp(1) << " "
                            << std::setw(20) << std::setprecision(10) << wtmp(2) << " "
                            << std::setw(20) << std::setprecision(10) << vtmp(0) << " "
                            << std::setw(20) << std::setprecision(10) << vtmp(1) << " "
                            << std::setw(20) << std::setprecision(10) << vtmp(2) << " " << std::endl;
                    }
                    else if (_order[0] == 'a' && _order[1] == 'g') // a->g
                    {
                        _ss << std::setw(15) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << t << " " << std::resetiosflags(std::ios::fixed)
                            << std::setw(20) << std::setprecision(10) << vtmp(0) << " "
                            << std::setw(20) << std::setprecision(10) << vtmp(1) << " "
                            << std::setw(20) << std::setprecision(10) << vtmp(2) << " "
                            << std::setw(20) << std::setprecision(10) << wtmp(0) << " "
                            << std::setw(20) << std::setprecision(10) << wtmp(1) << " "
                            << std::setw(20) << std::setprecision(10) << wtmp(2) << " " << std::endl;
                    }
                    else
                    {
                        if (_spdlog)
                            SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : ins_coder::encode_data order bot supported.");
                    }
                }
            }
        }

    }
    catch (...)
    {
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : ins_coder::encode_data throw exception");
        return -1;
    }

    int size = _fill_buffer(buff, sz);
    return size;
}


