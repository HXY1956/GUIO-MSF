#include "hwa_gnss_data_satparam.h"

using namespace std;

namespace hwa_gnss
{
    /** @brief constructor. */
    gnss_data_satparam::gnss_data_satparam()
    {
        id_type(base_data::SATPARS);
        _mass = 0.0;
    }
    gnss_data_satparam::gnss_data_satparam(base_log spdlog) : base_data(spdlog)
    {
        id_type(base_data::SATPARS);
        _mass = 0.0;
    }

    /** @brief destructor. */
    gnss_data_satparam::~gnss_data_satparam()
    {
    }

    /**
    * @brief set satellite prn.
    * @param[in]  prn    satellite prn
    */
    void gnss_data_satparam::set_prn(const std::string &prn)
    {
        _prn = prn;
    }

    /**
    * @brief set satellite svn.
    * @param[in]  svn    satellite svn
    */
    void gnss_data_satparam::set_svn(const std::string &svn)
    {
        _svn = svn;
    }

    /**
    * @brief set satellite launch time.
    * @param[in]  start     satellite launch time
    */
    void gnss_data_satparam::set_start(const base_time &start)
    {
        _start = start;
    }

    /**
    * @brief set satellite decommissioned time.
    * @param[in]  end      satellite decommissioned time
    */
    void gnss_data_satparam::set_end(const base_time &end)
    {
        _end = end;
    }

    /**
    * @brief set satellite cospar_id.
    * @param[in]  cosparid      satellite cospar_id
    */
    void gnss_data_satparam::set_cosparid(const std::string &cosparid)
    {
        _cosparid = cosparid;
    }

    /**
    * @brief set satellite mass.
    * @param[in]  mass      satellite mass
    */
    void gnss_data_satparam::set_mass(double mass)
    {
        _mass = mass;
    }

    /**
    * @brief set satellite max_yaw.
    * @param[in]  maxyaw      satellite max_yaw
    */
    void gnss_data_satparam::set_maxyaw(double maxyaw)
    {
        _maxyaw = maxyaw;
    }

    /**
    * @brief set satellite LRA COM correction.
    * @param[in]  lrax,lray,lraz
    */
    void gnss_data_satparam::set_lra(double x, double y, double z)
    {
        _lra[0] = x;
        _lra[1] = y;
        _lra[2] = z;
    }

    /**
    * @brief set the type of LRA.
    * @param[in]  itype
    */
    void gnss_data_satparam::set_lra_type(int itype)
    {
        _lratype = itype;
    }

    /**
    * @brief save satellite antenna transmit power(W).
    * @param[in] lpower
    */
    void gnss_data_satparam::set_lpower(double lpower)
    {
        _lpower = lpower;
    }

    /**
    * @brief set satellite frequency id.
    * @param[in]  fid      satellite frequency id
    */
    void gnss_data_satparam::set_fid(int fid)
    {
        _fid = fid;
    }

    /**
    * @brief set satellite block type.
    * @param[in]  blocktype      satellite block type
    */
    void gnss_data_satparam::set_blocktype(const std::string &blocktype)
    {
        _blocktype = blocktype;
    }

    /**
    * @brief get satellite prn.
    * @return     satellite prn.
    */
    std::string gnss_data_satparam::prn()
    {
        return _prn;
    }

    /**
    * @brief get satellite svn.
    * @return     satellite svn.
    */
    std::string gnss_data_satparam::svn()
    {
        return _svn;
    }

    /**
    * @brief get satellite launch time.
    * @return     satellite launch time.
    */
    base_time gnss_data_satparam::start()
    {
        return _start;
    }

    /**
    * @brief get satellite decommissioned time.
    * @return     satellite decommissioned time.
    */
    base_time gnss_data_satparam::end()
    {
        return _end;
    }

    /**
    * @brief get satellite cospar_id.
    * @return     satellite cospar_id.
    */
    std::string gnss_data_satparam::cosparid()
    {
        return _cosparid;
    }

    /**
    * @brief get satellite power.
    * @return     satellite power.
    */
    double gnss_data_satparam::power()
    {
        return _lpower;
    }

    /**
    * @brief get satellite mass.
    * @return     satellite mass.
    */
    double gnss_data_satparam::mass()
    {
        return _mass;
    }

    /**
    * @brief get satellite max_yaw.
    * @return     satellite max_yaw.
    */
    double gnss_data_satparam::maxyaw()
    {
        return _maxyaw;
    }

    /**
    * @brief get satellite frequency id.
    * @return     satellite frequency id.
    */
    int gnss_data_satparam::fid()
    {
        return _fid;
    }

    /**
    * @brief get satellite block type.
    * @return     satellite block type.
    */
    std::string gnss_data_satparam::blocktype()
    {
        return _blocktype;
    }

    /**
    * @brief get satellite's LRA COM correction
    * @return     LRA COM correction
    */
    double *gnss_data_satparam::lra()
    {
        return _lra;
    }

    /**
    * @brief get satellite's LRA COM correction
    * @return     LRA COM correction
    */
    int gnss_data_satparam::lratype()
    {
        return _lratype;
    }
}