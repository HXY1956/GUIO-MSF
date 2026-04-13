#include "hwa_gnss_all_satparam.h"

using namespace std;

namespace hwa_gnss
{
    /** @brief constructor. */
    gnss_all_satinfo::gnss_all_satinfo()
    {

        id_type(base_data::SATPARS);
    }
    gnss_all_satinfo::gnss_all_satinfo(base_log spdlog)
    {
        id_type(base_data::SATPARS);
    }

    /** @brief  destructor. */
    gnss_all_satinfo::~gnss_all_satinfo()
    {
    }

    /**
    * @brief push one satellite parameters data into the data map.
    * @param[in]  prn        satellite name.
    * @param[in]  svn        satellite svn.
    * @param[in]  start        launched time of the satellite.
    * @param[in]  end        decommissioned time of the satellite.
    * @param[in]  cosparid    cospar_id of the satellite.
    * @param[in]  mass        mass of the satellite.
    * @param[in]  maxyaw    max_yaw of the satellite.
    * @param[in]  fid        frequence id of the satellite.
    * @param[in]  LRA[3]    LRA COM correction
    * @param[in]  lratype   LRA type
    * @param[in]  blocktype    BLOCK-TYPE of the satellite.
    * @return    whether exit normally
    */
    int gnss_all_satinfo::addData(const string &prn, const string &svn,
                               const base_time &start, const base_time &end, const string &cosparid,
                               const double &mass, const double &maxyaw, const int &fid, double lra[],
                               const int &lratype,
                               const double &lpower, const string &blocktype)
    {
        // jdhuang,
        gnss_data_satparam tmp;
        tmp.set_prn(prn);
        tmp.set_svn(svn);
        tmp.set_start(start);
        tmp.set_end(end);
        tmp.set_cosparid(cosparid);
        tmp.set_mass(mass);
        tmp.set_maxyaw(maxyaw);
        tmp.set_lra(lra[0], lra[1], lra[2]);
        tmp.set_lra_type(lratype);
        tmp.set_lpower(lpower);
        tmp.set_fid(fid);
        tmp.set_blocktype(blocktype);

        bool notrecorded = true;
        for (vector<gnss_data_satparam>::iterator itRec = _allsatinfo[prn][svn].begin(); itRec != _allsatinfo[prn][svn].end(); ++itRec)
        {
            base_time satpars_st = (*itRec).start();
            base_time satpars_et = (*itRec).end();
            double delt_st = satpars_st.dmjd() - start.dmjd();
            double delt_et = satpars_et.dmjd() - end.dmjd();
            double eps = 5.0 / 86400.0;
            if (delt_st < 0)
                delt_st = delt_st * (-1);
            if (delt_et < 0)
                delt_et = delt_et * (-1);
            if (delt_st < eps || delt_et < eps)
            {
                notrecorded = false;
                break;
            }
        }
        if (notrecorded)
        {
            _allsatinfo[prn][svn].push_back(tmp);
        }

        return 0;
    }

    /** @brief whether the satellite parameters data is empty.
    * @return  bool
    *    @retval   true    satellite parameters data is empty
    *   @retval   false   satellite parameters data is existent
    */
    bool gnss_all_satinfo::isEmpty()
    {
        if (_allsatinfo.size() == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
    * @brief get valid satellite parameters data.
    * @param[in]  sat            satellite name.
    * @param[in]  ics_start        begin time from ics data.
    * @param[in]  ics_end        end time from ics data.
    * @param[out] sat_param        corresponding satellite parameters data.
    * @return    if find the correct satellite parameters data.
    *     @retval true    get data successfully
    *     @retval false    get data failed
    */
    bool gnss_all_satinfo::getSatinfoUsed(const string &sat, const base_time &ics_start, const base_time &ics_end, gnss_data_satparam &sat_param)
    {
        if (_allsatinfo.count(sat) == 0)
        {
            std::cout << "ERROR: " << sat << "is not existed in sat_paramters file" << endl;
            return false;
        }

        //map<string, gnss_data_satparam>::iterator itSvn = _allsatinfo[sat].begin();
        for (auto itSvn = _allsatinfo[sat].begin(); itSvn != _allsatinfo[sat].end(); ++itSvn)
        {
            vector<gnss_data_satparam>::iterator itSat;
            for (itSat = itSvn->second.begin(); itSat != itSvn->second.end(); ++itSat)
            {
                base_time satpars_st = (*itSat).start();
                base_time satpars_et = (*itSat).end();
                double delt_st = satpars_st.dmjd() - ics_start.dmjd();
                double delt_et = satpars_et.dmjd() - ics_end.dmjd();
                double eps = 5.0 / 86400.0;
                if (delt_st > eps || delt_et < -eps)
                {
                    continue;
                }
                (*itSat).valid = true;
                sat_param = *itSat;
                return true;
            }
        }
        if (_spdlog)
            SPDLOG_LOGGER_ERROR(_spdlog, "ics time is out of range,can't find corresponding sat information");
        return false;
    }
    int gnss_all_satinfo::getGloFid(const string &sat, const base_time &epoch)
    {
        gnss_data_satparam tmp_satinfo;
        if (getSatinfoUsed(sat, epoch, epoch, tmp_satinfo))
        {
            return tmp_satinfo.fid();
        }
        return 0;
    }
}