#ifndef hwa_base_allpar_h
#define hwa_base_allpar_h
#include <vector>
#include <map>
#include <string>
#include <set>
#include <unordered_map>
#include <algorithm>
#include "hwa_base_par.h"
#include "hwa_base_mutex.h"
#include "hwa_base_eigendef.h"

namespace hwa_base
{
    /**
     *@brief Class for base_allpar
     */
    class base_allpar
    {
    public:
        /**
        *@brief add parameter
        */
        void addParam(const base_par& par);

        /**
        *@brief delete parameter
        */
        void delParam(const int& i);


        std::map<std::string, int> freq_sats_num(const int& freq);
        /**
        *@brief delete all parameter
        */
        void delAllParam();

        /**
        *@brief reset all parameter
        */
        void resetAllParam();

        /**
        *@brief get parameter
        */
        /**
        *@brief get parameter
        */
        int getParam(const std::string& site, const par_type& type, const std::string& prn,
            const base_time& beg = FIRST_TIME, const base_time& end = LAST_TIME) const;

        /**
         * @brief Get the Param object
         *
         * @param index
         * @return int
         */
        int getParam(const int& index);

        /**
        *@brief get par index
        */
        int getParIndex(const int& idx);

        /**
        *@brief get par value
        */
        double getParValue(const int& idx);

        /**
        *@brief std::set par value
        */
        void setParValue(const int& idx, const double& value);

        /**
        *@brief std::set par remove
        */
        void setParRemove(const int& idx, const bool& judge);

        /**
        *@brief get par/amb/orbPar Number
        */
        unsigned int parNumber() const;

        unsigned int orbParNumber() const;

        std::vector<int> delAmb();

        std::set<std::string> amb_prns();

        std::set<std::string> amb_prns(const par_type& type);

        const base_par& getAmbParam(const int& idx) const;

        int getAmbParam(const std::string& site, const std::string& prn, const par_type& type,
            const base_time& beg = FIRST_TIME, const base_time& end = LAST_TIME) const;

        int getCrdParam(const std::string& site, Triple& crd, const base_time& beg = FIRST_TIME, const base_time& end = LAST_TIME) const;

        int getVelParam(const std::string& site, Triple& crd, const base_time& beg = FIRST_TIME, const base_time& end = LAST_TIME) const;

        std::vector<int> getPartialIndex(const std::string& site, const std::string& sat);

        Vector get_cvect(const base_allpar& par);

        const base_par& getPar(const int& idx) const;

        base_par& operator[](const size_t idx);
        base_allpar operator-(const base_allpar& par);
        base_allpar operator+(const base_allpar& par);

        static int mult(const Matrix& K, base_allpar& par, base_allpar& res);

        void reIndex();

        void decIndex(const int& i);

        void incIndex(const int& i);

        void setSite(const std::string& site);

        int sum(base_allpar& X1, base_allpar& X2);

        void setOrbPar(const std::string& sat, const std::vector<base_par>& pars);


        std::vector<base_par> getOrbPar(const std::string& sat);

        std::vector<base_par> getAllPar();

        std::map<std::string, std::vector<base_par>> getAllOrbPar();

        friend std::ostream& operator<<(std::ostream& os, base_allpar& x);

        int get_last_idx();

    private:
        std::vector<base_par> _vParam;
        std::map<std::string, std::vector<base_par>> _vOrbParam; ///< add for orb param

        long _max_point = 0;
        std::vector<long> _point_par; /// point -> vParam

        std::map<base_par_head, std::map<base_time_arc, long>> _index_par; /// index -> point
        // Fast get parital
        std::map<std::pair<std::string, std::string>, std::vector<int>> _index_for_parital; /// index -> param
        std::pair<long, int> _last_point;
        void _update_partial_index();

        base_mutex _allpar_mtx;
    };

}

#endif