#include "hwa_gnss_data_aug.h"

using namespace std;

namespace hwa_gnss
{
    gnss_data_aug::gnss_data_aug()
        : base_data()
    {

        id_type(base_data::AUG);
    }

    gnss_data_aug::~gnss_data_aug()
    {
    }
    base_time gnss_data_aug::endTime_aug(string &site)
    {
        hwa_map_ti_augtype_value *ihwa_map_ti_augtype_value = this->getAug(site);
        if (!ihwa_map_ti_augtype_value)
            return base_time();
        auto it_endepo = ihwa_map_ti_augtype_value->end();
        if (it_endepo != ihwa_map_ti_augtype_value->begin())
            it_endepo--;
        else
            return base_time();

        return it_endepo->first;
    }
    void gnss_data_aug::add_data(string site, const base_time &epo, string sat, hwa_pair_augtype type, double value)
    {
        _gaug[site][epo][sat][type] = value;
    }
    hwa_map_iti_augtype_value *gnss_data_aug::getAug()
    {
        return &_gaug;
    }
    hwa_map_ti_augtype_value *gnss_data_aug::getAug(string site)
    {
        if (_gaug.find(site) != _gaug.end())
            return &_gaug[site];
        else
            return NULL;
    }
    hwa_map_id_augtype_value *gnss_data_aug::getAug(string site, const base_time &epo)
    {
        if (_gaug.find(site) != _gaug.end() &&
            _gaug[site].find(epo) != _gaug[site].end())
            return &_gaug[site][epo];

        if (_gaug.find(site) != _gaug.end())
        {
            auto &epoaug = _gaug[site];
            if (epoaug.find(epo) != epoaug.end())
            {
                return &epoaug[epo];
            }
            else
            {
                auto latter = epoaug.lower_bound(epo);
                auto former = latter;
                if (latter == epoaug.end())
                {
                    if (latter != epoaug.begin())
                    {
                        latter--;
                        if (epo.diff(latter->first) < 300)
                            return &(latter->second);
                        else
                            return NULL;
                    }
                }
                else
                {
                    if (former != epoaug.begin())
                        former--;
                    double diff1 = latter->first.diff(epo);
                    double diff2 = epo.diff(former->first);
                    if (diff1 >= 600 && diff2 >= 600)
                        return NULL;
                    else
                    {
                        return &(diff1 < diff2 ? _gaug[site][former->first] : _gaug[site][latter->first]);
                    }
                }
            }
        }
        return NULL;
    }
    hwa_map_augtype_value *gnss_data_aug::getAug(string site, const base_time &epo, string sat)
    {
        if (_gaug.find(site) != _gaug.end() &&
            _gaug[site].find(epo) != _gaug[site].end() &&
            _gaug[site][epo].find(sat) != _gaug[site][epo].end())
            return &_gaug[site][epo][sat];
        else
            return NULL;
    }

    // clean function lvhb added in 20210413
    // ----------
    void gnss_data_aug::clean_outer(const string &site,
                             const base_time &beg,
                             const base_time &end)
    {

        if (end < beg)
            return;
        // loop over all object [default]
        hwa_map_iti_augtype_value::const_iterator itSite = _gaug.begin();
        hwa_map_iti_augtype_value::const_iterator itSite2 = _gaug.end();

        if (_spdlog)
            SPDLOG_LOGGER_DEBUG(_spdlog, "gaug", "aug clean request: " + site + beg.str_ymdhms(": ") + end.str_ymdhms(" - "));

        // loop over a single object (if obj defined !)
        if (!site.empty())
            itSite = itSite2 = _gaug.find(site);
        if (itSite2 != _gaug.end())
            itSite2++;

        for (; itSite != itSite2; ++itSite)
        {

            string site = itSite->first;

            // find and CLEAN all data (epochs) out of the specified period !
            hwa_map_ti_augtype_value::iterator it;
            hwa_map_ti_augtype_value::iterator itFirst = _gaug[site].begin();
            hwa_map_ti_augtype_value::iterator itLast = _gaug[site].end();
            hwa_map_ti_augtype_value::iterator itBeg = _gaug[site].lower_bound(beg); // greater|equal
            hwa_map_ti_augtype_value::iterator itEnd = _gaug[site].upper_bound(end); // greater only!

            hwa_map_ti_augtype_value::iterator itOBS;

            // delete before BEGIN request
            if (itBeg != itLast && itBeg != itFirst)
            {

                for (it = itFirst; it != itBeg; ++it)
                {
                    base_time tt(it->first);

                    _gaug[site][tt].erase(_gaug[site][tt].begin(), _gaug[site][tt].end());
                }

                _gaug[site].erase(itFirst, itBeg); // erase all but itBeg
            }

            // delete after END request
            if (itEnd != itLast)
            { // && ++itEnd != itLast ){

                for (it = itEnd; it != itLast; ++it)
                {

                    base_time tt(it->first);

                    _gaug[site][tt].erase(_gaug[site][tt].begin(), _gaug[site][tt].end());
                }
                _gaug[site].erase(itEnd, itLast);
            }
            //    itOBJ++; // WHILE LOOP ONLY
        }
        return;
    }

}