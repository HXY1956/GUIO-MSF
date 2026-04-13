#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "hwa_gnss_all_fltmat.h"
#include "hwa_base_typeconv.h"

using namespace std;

namespace hwa_gnss
{
    gnss_all_fltmat::gnss_all_fltmat() : base_data()
    {
        clear_addAmb();
    }
    gnss_all_fltmat::gnss_all_fltmat(base_log spdlog) : base_data(spdlog)
    {
        clear_addAmb();
    }

    // destructor
    // ----------
    gnss_all_fltmat::~gnss_all_fltmat()
    {
        data.clear();
        clear_addAmb();
    }

    void gnss_all_fltmat::add(const gnss_proc_fltmat &mat)
    {
        data.push_back(mat);
    }

    // sync par1 and par2
    // only common parameter remains
    // -----------------------------
    int gnss_all_fltmat::common(base_allpar &par1, Symmetric &Q1, base_allpar &par2, Symmetric &Q2, Diag *Noise)
    {

        for (unsigned int i = 0; i < par1.parNumber(); i++)
        {
            int idx = par2.getParam(par1[i].site, par1[i].parType, par1[i].prn);
            if (idx >= 0)
                continue;
            else
            {
                int index = par1[i].index;
                Q1.Matrix_remRC(index, index);
                par1.delParam(i);
                par1.decIndex(index);
                i--;
            }
        }

        for (unsigned int i = 0; i < par2.parNumber(); i++)
        {
            int idx = par1.getParam(par2[i].site, par2[i].parType, par2[i].prn);
            if (idx >= 0)
                continue;
            else
            {
                //cout << "common: Budu mazat par2 " << par2[i].str_type() << " " << par2[i].prn << endl;
                int index = par2[i].index;
                Q2.Matrix_remRC(index, index);
                if (Noise)
                    Noise->Matrix_remRC(index);
                par2.delParam(i);
                par2.decIndex(index);
                i--;
            }
        }

        return 1;
    }

    // sync smooth vector according to par1
    // -------------------------------
    int gnss_all_fltmat::syncSMT(base_allpar &Xu, Symmetric &Qu, base_allpar &Xsm, Symmetric &Qsm, Diag *Noise)
    {

        // removing from Xsm
        for (unsigned int i = 0; i < Xsm.parNumber(); i++)
        {
            int idx = Xu.getParam(Xsm[i].site, Xsm[i].parType, Xsm[i].prn);
            if (idx >= 0)
                continue;
            else
            {
                //cout << "syncSMT: Budu mazat par " << Xsm[i].str_type() << " " << Xsm[i].prn << endl;
                int index = Xsm[i].index;
                Qsm.Matrix_remRC(index, index);
                Xsm.delParam(i);
                Xsm.decIndex(index);
                i--;
            }
        }

        // adding missing par to Xsm
        for (unsigned int i = 0; i < Xu.parNumber(); i++)
        {
            int ii = Xu.getParam(i + 1);
            base_par parTMP = Xu[ii];
            int idx = Xsm.getParam(parTMP.site, parTMP.parType, parTMP.prn);
            if (idx >= 0)
                continue;
            else
            {
                //cout << "syncSMT: Budu pridavat par " << parTMP.str_type() << " " << parTMP.prn << endl;
                // introduce new  parameter (Xu[i] -> Xsm)
                Xsm.incIndex(parTMP.index);
                Xsm.addParam(parTMP);
                Qsm.Matrix_addRC(parTMP.index, parTMP.index);
                /*
                     // copy data from Qu into expanded Qsm
                     for (int i=1; i<=Qsm.rows(); i++){
                        for (int j=1; j<=Qsm.cols(); j++){
                           if ( i == parTMP.index || j == parTMP.index ){
                          int iS = Xsm.getParam(i);
                          int jS = Xsm.getParam(j);
                          int iU = Xu.getParam(Xsm[iS].site, Xsm[iS].parType, Xsm[iS].prn);
                          int jU = Xu.getParam(Xsm[jS].site, Xsm[jS].parType, Xsm[jS].prn);
                          int r = Xu[iU].index;
                          int c = Xu[jU].index;
                          Qsm(i,j) = Qu(r,c);
                           }
                        }
                     }
                */
                double tmp = Qu.get(parTMP.index, parTMP.index);
                Qsm.set(tmp, parTMP.index, parTMP.index);
                if (parTMP.parType == par_type::AMB_IF)
                    addAmb.insert(parTMP.index);
            }
        }

        return 1;
    }

    // checking reset ambiguities
    // -----------------------
    int gnss_all_fltmat::checkSlp(base_allpar &Xu, Symmetric &Qu, base_allpar &Xp, Symmetric &Qp, base_allpar &Xsm, Symmetric &Qsm, set<string> &slips)
    {
        for (unsigned int i = 0; i < Xsm.parNumber(); i++)
        {
            string prn = Xsm[i].prn;
            string site = Xsm[i].site;
            int idxu = Xu.getParam(site, par_type::AMB_IF, prn);
            int idxp = Xp.getParam(site, par_type::AMB_IF, prn);
            if (slips.find(prn) != slips.end())
            {
                Xp[idxp].value(Xu[idxu].value());
                Xsm[i].value(Xu[idxu].value());
                Qsm.Matrix_remRC(Xp[idxp].index, Xp[idxp].index);
                Qsm.Matrix_addRC(Xp[idxp].index, Xp[idxp].index);
                Qsm.set(Qu(Xu[idxu].index, Xu[idxu].index), Xsm[i].index, Xsm[i].index);
            }
        }

        return 1;
    }

    // checking outliers
    // -----------------------
    int gnss_all_fltmat::checkOutl(base_allpar &Xp, Symmetric &Qp, base_allpar &Xsm, Symmetric &Qu)
    {
        base_allpar diff = Xsm - Xp;
        for (unsigned int i = 0; i < diff.parNumber(); i++)
        {
            if (diff[i].parType == par_type::AMB_IF && fabs(diff[i].value()) > 2)
            {
                int index = diff[i].index;
                addAmb.insert(index);
            }
        }
        return 1;
    }

    // swap rows and cols for same order
    // only common pars have to be included
    // return: both X1 and X2 have the sama position in cov matrix
    int gnss_all_fltmat::reorder(base_allpar &X1, Symmetric &Q1, base_allpar &X2, Symmetric &Q2)
    {
        if (X1.parNumber() != X2.parNumber())
        {
            cerr << "gnss_all_fltmat:reorder - not synchronized!";
            return -1;
        }

        for (unsigned int i = 0; i < X1.parNumber(); i++)
        {
            int idx = X2.getParam(X1[i].site, X1[i].parType, X1[i].prn);
            if (idx >= 0)
            {
                if (X1[i].index != X2[idx].index)
                {
                    //cout << "reorder: Prehazuju " << X1[i].index << " za " << X2[idx].index << endl;
                    //cout << "Before: " << fixed << setprecision(6) << Q1 << endl;
                    Q1.Matrix_swap(X1[i].index, X2[idx].index);
                    //cout << "After: " << fixed << setprecision(6) << Q1 << endl;
                    int in = X1.getParam(X2[idx].index);
                    X1[in].index = X1[i].index;
                    X1[i].index = X2[idx].index;
                }
            }
            else
            {
                cerr << "gnss_all_fltmat::reorder - not common params!" << endl;
                return -1;
            }
        }
        return 1;
    }

    int gnss_all_fltmat::reorder2(base_allpar &X1, Symmetric &Q1, base_allpar &X2, Symmetric &Q2)
    {
        if (X1.parNumber() != X2.parNumber())
        {
            cerr << "gnss_all_fltmat:reorder - not synchronized!";
            return -1;
        }

        Symmetric Q_sav = Q1;

        for (int i = 0; i < Q1.rows(); i++)
        {
            for (int j = 0; j < Q1.cols(); j++)
            {
                int i1 = X1.getParam(i);
                int j1 = X1.getParam(j);
                int i2 = X2.getParam(X1[i1].site, X1[i1].parType, X1[i1].prn);
                int j2 = X2.getParam(X1[j1].site, X1[j1].parType, X1[j1].prn);
                int r = X2[i2].index;
                int c = X2[j2].index;
                Q1.set(Q_sav(r, c), i, j);
            }
        }

        for (unsigned int i = 0; i < X1.parNumber(); i++)
        {
            int idx = X2.getParam(X1[i].site, X1[i].parType, X1[i].prn);
            X1[i].index = X2[idx].index;
        }

        return 1;
    }

    // clear data
    // -------------------------
    void gnss_all_fltmat::clear()
    {
        data.clear();
    }

    // clear data
    // -------------------------
    void gnss_all_fltmat::clear_addAmb()
    {
        addAmb.clear();
    }

} // namespace
