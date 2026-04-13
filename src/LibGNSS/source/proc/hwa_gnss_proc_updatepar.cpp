#include "hwa_gnss_proc_Updatepar.h"
#include "hwa_base_eigendef.h"

using namespace std;
using namespace hwa_base;

namespace hwa_gnss
{

    gnss_proc_update_par_info::gnss_proc_update_par_info()
    {
    }
    gnss_proc_update_par_info::~gnss_proc_update_par_info()
    {
    }

    bool gnss_proc_update_par_info::exist(int id)
    {
        return _remove_id.count(id) != 0 ? true : false;
    }

    void gnss_proc_update_par_info::add(int id)
    {
        if (_remove_id.count(id) != 0)
        {
            throw std::logic_error("Remove id have repeated!");
        }

        _remove_id.insert(id);
    }

    void gnss_proc_update_par_info::add(base_par newpar)
    {
        _new_parlist.push_back(newpar);
    }

    void gnss_proc_update_par_info::add(int id, base_par newpar)
    {
        this->add(id);
        _new_parlist.push_back(newpar);
    }

    void gnss_proc_update_par_info::add(vector<pair<int, base_par>> update_par, gnss_proc_lsq_equationmatrix state_equ)
    {
        for (auto iter : update_par)
        {
            //this->add(iter.first, iter.second);
            this->add(iter.first);
            _equ_parlist.push_back(iter.second);
        }
        _state_equ.add_equ(state_equ);
    }

    void gnss_proc_update_par_info::get(vector<int> &remove_id)
    {
        remove_id = vector<int>(_remove_id.begin(), _remove_id.end());
    }

    void gnss_proc_update_par_info::get(vector<base_par> &newparlist)
    {
        newparlist = _new_parlist;
    }

    void gnss_proc_update_par_info::get(vector<base_par> &update_newpar, gnss_proc_lsq_equationmatrix &equ)
    {
        update_newpar = _equ_parlist;
        equ = _state_equ;
    }

    void gnss_proc_update_par_info::get(vector<int> &remove_id, vector<base_par> &newparlist, vector<base_par> &equ_parlist, gnss_proc_lsq_equationmatrix &equ)
    {
        get(remove_id);
        get(newparlist);
        get(equ_parlist, equ);
    }

    gnss_proc_update_par::gnss_proc_update_par()
    {
    }

    gnss_proc_update_par::~gnss_proc_update_par()
    {
    }

    gnss_proc_update_par::gnss_proc_update_par(const gnss_proc_update_par &Other) : _cycleslip(Other._cycleslip),_state_mode(Other._state_mode)
    {
    }

    void gnss_proc_update_par::set_interval(double interval)
    {
        _intv = interval;
    }

    void gnss_proc_update_par::set_sig_ion(double sigion)
    {
        _sig_ion = sigion;
    }

    void gnss_proc_update_par::set_cycleslip(shared_ptr<gnss_data_cycleslip> cycleslip)
    {
        _cycleslip = cycleslip;
    }

    void gnss_proc_update_par::set_amb_update_way(bool way)
    {
        _lite_update_amb = way;
    }

    void gnss_proc_update_par::set_par_state_mode(par_type type, int order, double dt, double noise)
    {
        _state_mode[type] = gnss_model_state_mode(order, dt, noise);
    }

    void gnss_proc_update_par::update_amb_pars(const base_time &epoch, base_allpar &allpars, const vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info)
    {
        this->_update_amb_pars(epoch, allpars, obsdata, update_info);
    }

    gnss_proc_update_par_info gnss_proc_update_par::get_all_update_parameters(const base_time &epoch, base_allpar &allpars, const vector<gnss_data_sats> &obsdata)
    {
        gnss_proc_update_par_info update_info;
        // update AMB
        _update_amb_pars(epoch, allpars, obsdata, update_info);

        // update Other pars
        _update_process_pars(epoch, allpars, update_info);

        return update_info;
    }

    void gnss_proc_update_par::_update_process_pars(const base_time &epoch, base_allpar &allpars, gnss_proc_update_par_info &update_info)
    {

        // update process pars
        vector<base_par> parnew;
        for (unsigned int ipar = 0; ipar < allpars.parNumber(); ipar++)
        {
            // according to time,lremove,update_info
            if (allpars[ipar].end < epoch && allpars[ipar].lremove && !update_info.exist(ipar + 1))
            {

                // judge state mode
                if (_state_mode.find(allpars[ipar].parType) == _state_mode.end())
                {

                    _update_process_par(epoch, ipar, allpars, update_info);
                }
                else
                {
                    _update_state_par(epoch, ipar, allpars, update_info);
                }
            }
        }
    }

    void gnss_proc_update_par::_update_state_par(const base_time &epoch, int update_id, base_allpar &allpars, gnss_proc_update_par_info &update_info)
    {
        // record the new add par location
        std::vector<std::pair<int, base_par>> parnewlist;
        gnss_proc_lsq_equationmatrix state_equ;
        gnss_proc_lsq_equationmatrix state_equ_tmp;
        std::vector<base_par> equ_parlist;

        update_info.get(equ_parlist, state_equ_tmp);
        int newpar_location = allpars.parNumber() + 1 + equ_parlist.size();
        par_type par_type = allpars[update_id].parType;

        for (int i = 0; i < _state_mode[par_type].order; i++)
        {
            base_par par_temp = allpars[update_id + i];
            double intv = par_temp.end - par_temp.beg;
            par_temp.setTime(epoch, epoch + intv);
            parnewlist.push_back(std::make_pair(update_id + i + 1, par_temp));
        }

        // create the state_equation
        for (int Row = 1; Row <= _state_mode[par_type].order; Row++)
        {
            std::vector<std::pair<int, double>> B;
            double P;
            // for B
            B.push_back(std::make_pair(update_id + Row, 1.0));
            for (int Col = Row; Col <= _state_mode[par_type].order; Col++)
            {
                B.push_back(std::make_pair(newpar_location + Col - 1, -1.0));
            }

            // for P
            P = _state_mode[par_type].P(Row, Row);

            gnss_data_obscombtype type;
            state_equ.add_equ(B, P, 0.0, "", "", type, false);
        }

        update_info.add(parnewlist, state_equ);
    }

    void gnss_proc_update_par::_update_process_par(const base_time &epoch, int update_id, base_allpar &allpars, gnss_proc_update_par_info &update_info)
    {
        base_par par_tmp = allpars[update_id];
        double intv = par_tmp.end - par_tmp.beg;
        par_tmp.setTime(epoch, epoch + intv);
        update_info.add(update_id + 1, par_tmp);
    }

    void gnss_proc_update_par::_udpate_gps_rec_ifb_pars(const base_time &epoch, base_allpar &allpars, const vector<gnss_data_sats> &obsdata, gnss_proc_update_par_info &update_info)
    {
        // for debug
        // double my_mjd = epoch.mjd();
        // double my_sod = epoch.sod();

        std::set<string> mapRec;
        for (auto &data : obsdata)
        {
            auto rec = data.site();
            if (mapRec.count(rec) > 0)
                continue;
            mapRec.insert(rec);
            // add new ifb par
            int idx_gps_ifb = allpars.getParam(rec, par_type::GPS_REC_IFB_C3, "");
            if (idx_gps_ifb < 0)
            {
                int idx_max = allpars[allpars.parNumber() - 1].index;
                base_par par_gps_rec_ifb = base_par(rec, par_type::GPS_REC_IFB_C3, idx_max + 1, "");
                par_gps_rec_ifb.value(0.0);
                par_gps_rec_ifb.setTime(epoch, epoch);
                par_gps_rec_ifb.apriori(9000);
                update_info.add(par_gps_rec_ifb);
            }
        }

        // remove old ifb par
        // jdhuang : change mapPRN to mapRec
        for (unsigned int i = 0; i < allpars.parNumber(); i++)
        {
            if (allpars[i].parType == par_type::GPS_REC_IFB_C3 && mapRec.count(allpars[i].site) == 0)
            {
                update_info.add(i + 1);
            }
        }
    }

}
