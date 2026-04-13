#include "hwa_gnss_coder_satparam.h"
#include "hwa_base_typeconv.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{

    /**
    * @brief constructor.
    * @param[in]  s        setbase control
    * @param[in]  version  version of the gcoder
    * @param[in]  sz       size of the buffer
    */
    gnss_coder_satparam::gnss_coder_satparam(set_base *s, string version, int sz)
        : base_coder(s, version, sz), gnss_base_coder(s, version, sz)
    {

        _flag = 0;
    }

    /** @brief destructor. */
    gnss_coder_satparam::~gnss_coder_satparam()
    {
    }

    /**
    * @brief decode header of sat_parameters_new file
    * @param[in]  buff        buffer of the data
    * @param[in]  sz          buffer size of the data
    * @param[in]  errmsg      error message of the data decoding
    * @return consume size of header decoding
    */
    int gnss_coder_satparam::decode_head(char *buff, int sz, vector<string> &errmsg)
    {

        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        }
        string tmp;
        int consume = 0;
        int tmpsize = 0;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp)) >= 0)
            {
                if (tmp.substr(0, 1) != "#")
                {
                    _mutex.unlock();
                    return -1;
                }
                gnss_base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : gnss_coder_satparam::decode_head throw exception");
            return -1;
        }
    }

    /**
    * @brief decode data body of sat_parameters_new file
    * @param[in]  buff        buffer of the data
    * @param[in]  sz          buffer size of the data
    * @param[in]  errmsg      error message of the data decoding
    * @return consume size for data body decoding
    */
    int gnss_coder_satparam::decode_data(char *buff, int sz, int &cnt, vector<string> &errmsg)
    {

        _mutex.lock();

        if (gnss_base_coder::_add2buffer(buff, sz) == 0)
        {
            _mutex.unlock();
            return 0;
        }
        string tmp;
        int consume = 0;
        int tmpsize = 0;
        try
        {
            while ((tmpsize = gnss_base_coder::_getline(tmp)) >= 0)
            {
                //judge the data type
                if (tmp.substr(0, 12) == "+prn_indexed")
                {
                    _flag = 1;
                }
                else if (tmp.substr(0, 12) == "+svn_indexed")
                {
                    _flag = 2;
                }
                else if (tmp.substr(0, 12) == "-svn_indexed")
                {
                    _mutex.unlock();
                    return consume;
                }
                //read prn_indexed data
                if (tmp.substr(0, 1) == " ")
                {
                    stringstream ss(tmp);
                    consume += tmpsize;
                    string prn, svn, cosparid, blocktype;
                    base_time st, et;
                    double satmass = 0.0, maxyaw = 0.0; //yaw unit is deg
                    double lra[3] = {0.0}, lpower = 0.0;
                    int lratype = 0, fid = 0;
                    string start, end; //tmp time
                    //read prn_indexed data
                    if (_flag == 1)
                    {
                        ss >> prn >> svn >> start >> end >> cosparid >> satmass >> maxyaw >> fid >> lra[0] >> lra[1] >> lra[2] >> lratype >> lpower >> blocktype;
                        //GPS satellites' block type has two part
                        if (prn.size() == 3 && prn.substr(0, 1) == "G")
                        {
                            string blocktype1;
                            ss >> blocktype1;
                            blocktype += blocktype1;
                        }
                    }
                    //read svn_indexed data
                    if (_flag == 2)
                    {
                        ss >> svn >> prn >> start >> end >> cosparid >> satmass >> maxyaw >> fid >> lra[0] >> lra[1] >> lra[2] >> lratype >> lpower >> blocktype;
                        //GPS satellites' block type has two part
                        if (prn.size() == 3 && prn.substr(0, 1) == "G")
                        {
                            string blocktype1;
                            ss >> blocktype1;
                            blocktype += blocktype1;
                        }
                    }

                    //transfer the time type
                    int year = 0, doy = 0, sod = 0, month = 0, day = 0;
                    //start time
                    year = base_type_conv::str2int(start.substr(0, 4));
                    doy = base_type_conv::str2int(start.substr(4, 3));
                    sod = base_type_conv::str2int(start.substr(8, 7));
                    // day of year to month and day
                    yeardoy2monthday(year, doy, &month, &day);
                    st.from_ymd(year, month, day, sod, 0.0);
                    //end time
                    year = base_type_conv::str2int(end.substr(0, 4));
                    doy = base_type_conv::str2int(end.substr(4, 3));
                    sod = base_type_conv::str2int(end.substr(8, 7));
                    //day of year to month and day
                    yeardoy2monthday(year, doy, &month, &day);
                    et.from_ymd(year, month, day, sod, 0.0);
                    //if the sat haven't decommissioned
                    if (year == 0 && doy == 0 && sod == 0)
                    {
                        et = LAST_TIME;
                    }
                    //fill one sat data
                    map<string, base_data *>::iterator it = _data.begin();
                    while (it != _data.end())
                    {
                        if (it->second->id_type() == base_data::SATPARS)
                            dynamic_cast<gnss_all_satinfo *>(it->second)->addData(prn, svn, st, et, cosparid, satmass, maxyaw, fid, lra, lratype, lpower, blocktype);
                        it++;
                    }
                }
                gnss_base_coder::_consume(tmpsize);
            }
            _mutex.unlock();
            return consume;
        }
        catch (...)
        {
            if (_spdlog)
                SPDLOG_LOGGER_ERROR(_spdlog, "ERROR : gnss_coder_satparam::decode_data throw exception");
            return -1;
        }
    }

    /**
    * @brief day of year to month and day
    * @param[in]   year        year
    * @param[in]   doy         day of year
    * @param[out]  month       month
    * @param[out]  day           day
    */
    void gnss_coder_satparam::yeardoy2monthday(int year, int doy, int *month, int *day)
    {
        int days_inmonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        int d = doy;
        if ((year % 4) == 0 && ((year % 100) != 0 || (year % 400 == 0)))
        {
            days_inmonth[1] = 29;
        }
        for (*month = 1; *month <= 12; (*month)++)
        {
            d = d - days_inmonth[(*month) - 1];
            if (d > 0)
                continue;
            *day = d + days_inmonth[*month - 1];
            break;
        }
    }
}