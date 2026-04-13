#include "hwa_gnss_all_rslt.h"

using namespace std;

namespace hwa_gnss
{
    gnss_all_rslt::gnss_all_rslt()
    {
    }

    void gnss_all_rslt::append(const gnss_all_rslt::result &rslt)
    {
        if (rslt.type == "CRD_X" || rslt.type == "CRD_Y" || rslt.type == "CRD_Z")
        {
            set_crd.insert(rslt);
        }
        else if (rslt.type == "TRP")
        {
            set_trp.insert(rslt);
        }
        else if (rslt.type == "CLK")
        {
            set_clk.insert(rslt);
        }
        else if (rslt.type == "AMB")
        {
            set_amb.insert(rslt);
        }
        else
        {
            cerr << "WRONG TYPE. POSSSIBLE ARE: CRD_X, CRD_Y, CRD_Z, CLK, TRP, AMB" << endl;
        }

        v_rslt.push_back(rslt);
    }

    // append values
    // ----------
    void gnss_all_rslt::append(const string &type, const base_time &beg, const base_time &end, int idx,
                            const string &prn, double adj, double rms, double val)
    {
        gnss_all_rslt::result rslt;

        rslt.type = type;
        rslt.beg = beg;
        rslt.end = end;
        rslt.index = idx;
        rslt.prn = prn;
        rslt.adj = adj;
        rslt.rms = rms;
        rslt.val = val;

        if (type == "CRD_X" || type == "CRD_Y" || type == "CRD_Z")
        {
            set_crd.insert(rslt);
        }
        else if (type == "TRP")
        {
            set_trp.insert(rslt);
        }
        else if (type == "CLK")
        {
            set_clk.insert(rslt);
        }
        else if (type == "AMB")
        {
            set_amb.insert(rslt);
        }
        else
        {
            cerr << "WRONG TYPE. POSSSIBLE ARE: CRD_X, CRD_Y, CRD_Z, CLK, TRP, AMB" << endl;
        }

        v_rslt.push_back(rslt);
    }

    // operator <
    // ----------
    bool gnss_all_rslt::result::operator<(const gnss_all_rslt::result &rslt) const
    {
        if ((beg < rslt.beg) ||
            (beg == rslt.beg && end < rslt.end) ||
            (beg == rslt.beg && end == rslt.end && prn < rslt.prn) ||
            (beg == rslt.beg && end == rslt.end && prn == rslt.prn && type < rslt.type))
        {
            return true;
        }
        return false;
    }

} // namespace
